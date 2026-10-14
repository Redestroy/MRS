#include "mrs/platform/WebotsPlatform.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <webots/Accelerometer.hpp>
#include <webots/Altimeter.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/Connector.hpp>
#include <webots/Device.hpp>
#include <webots/Emitter.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <webots/Node.hpp>

namespace MRS {
	namespace Platform {
		namespace P = ::MRS::Port;

		namespace {
			const char* NodeTypeName(int type) {
				switch (type) {
				case webots::Node::GPS: return "GPS";
				case webots::Node::INERTIAL_UNIT: return "InertialUnit";
				case webots::Node::GYRO: return "Gyro";
				case webots::Node::ACCELEROMETER: return "Accelerometer";
				case webots::Node::COMPASS: return "Compass";
				case webots::Node::ALTIMETER: return "Altimeter";
				case webots::Node::ROTATIONAL_MOTOR: return "RotationalMotor";
				case webots::Node::LED: return "LED";
				case webots::Node::EMITTER: return "Emitter";
				case webots::Node::RECEIVER: return "Receiver";
				case webots::Node::CAMERA: return "Camera";
				case webots::Node::CONNECTOR: return "Connector";
				default: return "";
				}
			}

			// Read-side ports: one new sample per sampling period.
			class SampledPort : public P::Port {
			public:
				SampledPort(P::PortAssignment a, webots::Robot& robot, int period_ms)
				    : P::Port(std::move(a)), robot_(robot), period_s_(period_ms / 1000.0) {}

			protected:
				// True when a new sample is due; call once per Read.
				bool Due() {
					const double t = robot_.getTime();
					if (t + 1e-9 < next_) return false;
					next_ = t + period_s_;
					return true;
				}
				webots::Robot& robot_;

			private:
				double period_s_;
				double next_ = 0.0;
			};

			class VectorPort : public SampledPort {
			public:
				using Getter = const double* (*)(webots::Device*);
				VectorPort(P::PortAssignment a, webots::Robot& robot, int period_ms, webots::Device* device, Getter get, int n)
				    : SampledPort(std::move(a), robot, period_ms), device_(device), get_(get), n_(n) {}
				bool Read(std::vector<double>& values) override {
					if (!Due()) return false;
					const double* v = get_(device_);
					if (!v) return false;
					values.assign(v, v + n_);
					for (double x : values)
						if (!std::isfinite(x)) return false;  // not yet valid (first step)
					return true;
				}

			private:
				webots::Device* device_;
				Getter get_;
				int n_;
			};

			class AltimeterPort : public SampledPort {
			public:
				AltimeterPort(P::PortAssignment a, webots::Robot& robot, int period_ms, webots::Altimeter* device)
				    : SampledPort(std::move(a), robot, period_ms), device_(device) {}
				bool Read(std::vector<double>& values) override {
					if (!Due()) return false;
					values = {device_->getValue()};
					return std::isfinite(values[0]);
				}

			private:
				webots::Altimeter* device_;
			};

			class BatteryPort : public SampledPort {
			public:
				BatteryPort(P::PortAssignment a, webots::Robot& robot, int period_ms) : SampledPort(std::move(a), robot, period_ms) {}
				bool Read(std::vector<double>& values) override {
					if (!Due()) return false;
					const double joules = robot_.batterySensorGetValue();
					if (!std::isfinite(joules) || joules < 0) return false;
					// Webots reports 0 from enabling until the first sample arrives; read as an empty
					// battery, that would raise BATTERY_CRITICAL on the ground and stop the robot.
					if (joules == 0.0 && !sampled_) return false;
					sampled_ = true;
					values = {joules / 3600.0};
					return true;
				}

			private:
				bool sampled_ = false;
			};

			class MotorPort : public P::Port {
			public:
				MotorPort(P::PortAssignment a, webots::Motor* motor) : P::Port(std::move(a)), motor_(motor) {
					position_ = Assignment().params.Text("mode", "velocity") == "position";  // gimbal axes
					if (position_) return;
					motor_->setPosition(std::numeric_limits<double>::infinity());  // velocity mode
					motor_->setVelocity(0.0);
				}
				bool Write(const std::vector<double>& values) override {
					if (values.empty() || !std::isfinite(values[0])) return false;
					if (position_) {
						const double lo = motor_->getMinPosition(), hi = motor_->getMaxPosition();
						double v = values[0];
						if (lo < hi) v = std::min(std::max(v, lo), hi);  // both 0: no limits
						motor_->setPosition(v);
					} else {
						motor_->setVelocity(values[0]);
					}
					return true;
				}

			private:
				webots::Motor* motor_;
				bool position_ = false;
			};

			// A write saves the camera's current image as <prefix><robot>_<shot>.jpg in the
			// controller's folder.
			class CameraPort : public P::Port {
			public:
				CameraPort(P::PortAssignment a, webots::Robot& robot, webots::Camera* camera, int period_ms)
				    : P::Port(std::move(a)), robot_(robot), camera_(camera) {
					camera_->enable(period_ms);
					prefix_ = Assignment().params.Text("prefix", "shot_");
				}
				bool Write(const std::vector<double>& values) override {
					if (values.empty()) return false;
					const std::string file = prefix_ + robot_.getName() + "_" + std::to_string(static_cast<long>(values[0])) + ".jpg";
					return camera_->saveImage(file, 90) == 0;
				}

			private:
				webots::Robot& robot_;
				webots::Camera* camera_;
				std::string prefix_;
			};

			// Payload latch: reads 1 while locked to a package, a write of 0 unlocks, 1 locks.
			class ConnectorPort : public P::Port {
			public:
				ConnectorPort(P::PortAssignment a, webots::Connector* connector, int period_ms) : P::Port(std::move(a)), connector_(connector) {
					connector_->enablePresence(period_ms);
				}
				bool Read(std::vector<double>& values) override {
					values = {connector_->isLocked() ? 1.0 : 0.0};
					return true;
				}
				bool Write(const std::vector<double>& values) override {
					if (values.empty()) return false;
					if (values[0] >= 0.5) connector_->lock();
					else connector_->unlock();
					return true;
				}

			private:
				webots::Connector* connector_;
			};

			class LedPort : public P::Port {
			public:
				LedPort(P::PortAssignment a, webots::LED* led) : P::Port(std::move(a)), led_(led) {
					rgb_ = Assignment().params.Bool("rgb", false);
				}
				bool Write(const std::vector<double>& values) override {
					if (values.empty()) return false;
					const int colour = static_cast<int>(values[0]);
					led_->set(rgb_ ? colour : (colour != 0 ? 1 : 0));
					return true;
				}

			private:
				webots::LED* led_;
				bool rgb_ = false;
			};

			class EmitterPort : public P::Port {
			public:
				EmitterPort(P::PortAssignment a, webots::Emitter* emitter) : P::Port(std::move(a)), emitter_(emitter) {
					const auto& p = Assignment().params;
					emitter_->setChannel(static_cast<int>(p.Int("channel", 1)));
					if (p.Has("range_m")) emitter_->setRange(p.Num("range_m", -1));
				}
				bool Send(const std::string& data) override {
					return emitter_->send(data.data(), static_cast<int>(data.size())) == 1;
				}

			private:
				webots::Emitter* emitter_;
			};

			// The robot id N of a message from "rN": the sender is the fifth token, "M: <code> <ver> <mission> <sender>".
			int SenderId(const std::string& data) {
				std::size_t at = 0;
				for (int k = 0; k < 4; ++k) {
					at = data.find(' ', at);
					if (at == std::string::npos) return -1;
					++at;
				}
				if (at >= data.size() || data[at] != 'r') return -1;
				int id = 0, digits = 0;
				for (std::size_t k = at + 1; k < data.size() && data[k] >= '0' && data[k] <= '9'; ++k, ++digits) id = id * 10 + (data[k] - '0');
				return digits > 0 ? id : -1;
			}

			class ReceiverPort : public P::Port {
			public:
				ReceiverPort(P::PortAssignment a, webots::Receiver* receiver, int period_ms, webots::Robot& robot,
				             std::shared_ptr<WebotsPlatform::Bearings> bearings)
				    : P::Port(std::move(a)), receiver_(receiver), robot_(robot), bearings_(std::move(bearings)) {
					receiver_->setChannel(static_cast<int>(Assignment().params.Int("channel", 1)));
					receiver_->enable(period_ms);
				}
				std::optional<std::string> Receive() override {
					if (receiver_->getQueueLength() <= 0) return std::nullopt;
					std::string data(static_cast<const char*>(receiver_->getData()), static_cast<std::size_t>(receiver_->getDataSize()));
					const int sender = SenderId(data);
					if (sender >= 0) {  // for the ranging port
						auto& b = (*bearings_)[sender];
						b.t = robot_.getTime();
						const double* d = receiver_->getEmitterDirection();
						for (int k = 0; k < 3; ++k) b.dir[k] = d[k];
						b.strength = receiver_->getSignalStrength();
					}
					receiver_->nextPacket();
					// Senders that pass C strings include the terminating NUL; the parser would reject it.
					while (!data.empty() && data.back() == '\0') data.pop_back();
					return data;
				}

			private:
				webots::Receiver* receiver_;
				webots::Robot& robot_;
				std::shared_ptr<WebotsPlatform::Bearings> bearings_;
			};

			// Relative positions from the radio bearings (spec 15 §3.1): r = 1/sqrt(strength), turned
			// from the receiver (body) frame into ENU with the inertial unit's roll, pitch and yaw.
			class RangingPort : public SampledPort {
			public:
				RangingPort(P::PortAssignment a, webots::Robot& robot, int period_ms, webots::InertialUnit* imu,
				            std::shared_ptr<const WebotsPlatform::Bearings> bearings)
				    : SampledPort(std::move(a), robot, period_ms), imu_(imu), bearings_(std::move(bearings)),
				      window_(std::max(period_ms / 1000.0, 0.1)), range_(Assignment().params.Num("range_m", 1e9)) {}
				bool Read(std::vector<double>& v) override {
					if (!Due()) return false;
					const double* rpy = imu_->getRollPitchYaw();
					const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]), cp = std::cos(rpy[1]), sp = std::sin(rpy[1]),
					             cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
					// R = Rz(yaw) Ry(pitch) Rx(roll), body to world.
					const double R[9] = {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
					                     sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
					                     -sp,     cp * sr,                cp * cr};
					v.clear();
					const double t = robot_.getTime();
					for (const auto& [id, b] : *bearings_) {
						if (t - b.t > window_ || b.strength <= 0.0) continue;
						const double r = 1.0 / std::sqrt(b.strength);
						if (r > range_) continue;
						v.push_back(static_cast<double>(id));
						for (int k = 0; k < 3; ++k) v.push_back(r * (R[3 * k] * b.dir[0] + R[3 * k + 1] * b.dir[1] + R[3 * k + 2] * b.dir[2]));
					}
					return true;
				}

			private:
				webots::InertialUnit* imu_;
				std::shared_ptr<const WebotsPlatform::Bearings> bearings_;
				double window_, range_;
			};
		}

		WebotsPlatform::WebotsPlatform(webots::Robot& robot)
		    : robot_(robot), basic_ms_(static_cast<int>(robot.getBasicTimeStep())) {
			if (basic_ms_ < 1) basic_ms_ = 1;
		}

		int WebotsPlatform::PeriodMs(const P::PortAssignment& a) const {
			const double rate = a.params.Num("rate_hz", 0.0);
			if (rate <= 0.0) return basic_ms_;
			const int wanted = static_cast<int>(std::ceil(1000.0 / rate - 1e-9));
			const int steps = (wanted + basic_ms_ - 1) / basic_ms_;
			return (steps < 1 ? 1 : steps) * basic_ms_;
		}

		std::vector<P::PortInfo> WebotsPlatform::Scan() {
			std::vector<P::PortInfo> out;
			for (int k = 0; k < robot_.getNumberOfDevices(); ++k) {
				webots::Device* d = robot_.getDeviceByIndex(k);
				if (d) out.push_back({P::PortType::SIM, d->getName(), NodeTypeName(d->getNodeType())});
			}
			out.push_back({P::PortType::SIM, "battery", "Battery"});
			out.push_back({P::PortType::SIM, "ranging", "Ranging"});
			return out;
		}

		std::unique_ptr<P::Port> WebotsPlatform::Open(const P::PortAssignment& a) {
			if (a.type != P::PortType::SIM) return nullptr;
			const int period = PeriodMs(a);
			if (a.address == "battery") {
				robot_.batterySensorEnable(period);
				return std::make_unique<BatteryPort>(a, robot_, period);
			}
			if (a.address == "ranging") {
				webots::Device* imu = robot_.getDevice(a.params.Text("inertial", "inertial unit"));
				if (!imu || imu->getNodeType() != webots::Node::INERTIAL_UNIT) return nullptr;
				static_cast<webots::InertialUnit*>(imu)->enable(period);
				return std::make_unique<RangingPort>(a, robot_, period, static_cast<webots::InertialUnit*>(imu), bearings_);
			}
			webots::Device* d = robot_.getDevice(a.address);
			if (!d) return nullptr;
			switch (d->getNodeType()) {
			case webots::Node::GPS: {
				auto* gps = static_cast<webots::GPS*>(d);
				const bool wgs84 = gps->getCoordinateSystem() == webots::GPS::WGS84;
				const std::string frame = a.params.Text("frame", "wgs84");
				if (wgs84 != (frame == "wgs84")) return nullptr;  // the world's gpsCoordinateSystem differs from the node's frame
				gps->enable(period);
				return std::make_unique<VectorPort>(a, robot_, period, d, [](webots::Device* x) { return static_cast<webots::GPS*>(x)->getValues(); }, 3);
			}
			case webots::Node::INERTIAL_UNIT:
				static_cast<webots::InertialUnit*>(d)->enable(period);
				return std::make_unique<VectorPort>(a, robot_, period, d,
				                                    [](webots::Device* x) { return static_cast<webots::InertialUnit*>(x)->getRollPitchYaw(); }, 3);
			case webots::Node::GYRO:
				static_cast<webots::Gyro*>(d)->enable(period);
				return std::make_unique<VectorPort>(a, robot_, period, d, [](webots::Device* x) { return static_cast<webots::Gyro*>(x)->getValues(); }, 3);
			case webots::Node::ACCELEROMETER:
				static_cast<webots::Accelerometer*>(d)->enable(period);
				return std::make_unique<VectorPort>(a, robot_, period, d,
				                                    [](webots::Device* x) { return static_cast<webots::Accelerometer*>(x)->getValues(); }, 3);
			case webots::Node::COMPASS:
				static_cast<webots::Compass*>(d)->enable(period);
				return std::make_unique<VectorPort>(a, robot_, period, d, [](webots::Device* x) { return static_cast<webots::Compass*>(x)->getValues(); }, 3);
			case webots::Node::ALTIMETER: {
				auto* alt = static_cast<webots::Altimeter*>(d);
				alt->enable(period);
				return std::make_unique<AltimeterPort>(a, robot_, period, alt);
			}
			case webots::Node::ROTATIONAL_MOTOR: return std::make_unique<MotorPort>(a, static_cast<webots::Motor*>(d));
			case webots::Node::LED: return std::make_unique<LedPort>(a, static_cast<webots::LED*>(d));
			case webots::Node::EMITTER: return std::make_unique<EmitterPort>(a, static_cast<webots::Emitter*>(d));
			case webots::Node::RECEIVER: return std::make_unique<ReceiverPort>(a, static_cast<webots::Receiver*>(d), period, robot_, bearings_);
			case webots::Node::CAMERA: return std::make_unique<CameraPort>(a, robot_, static_cast<webots::Camera*>(d), period);
			case webots::Node::CONNECTOR: return std::make_unique<ConnectorPort>(a, static_cast<webots::Connector*>(d), period);
			default: return nullptr;
			}
		}

		double WebotsPlatform::Time() const { return robot_.getTime(); }

		bool WebotsPlatform::Step() { return robot_.step(basic_ms_) != -1; }
	}
}
