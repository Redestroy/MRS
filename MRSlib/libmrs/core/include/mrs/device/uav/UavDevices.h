#pragma once
// UAV device classes (spec 04 §3). They talk only to Ports, so the same classes serve every
// platform; a registry key such as gnss.webots only adds that platform's default device names.
#include <array>
#include <map>
#include <string>
#include <vector>

#include "mrs/device/DeviceNode.h"
#include "mrs/device/DeviceRegistry.h"
#include "mrs/device/uav/FlightControlUnit.h"

namespace MRS {
	namespace Device {
		namespace Uav {
			// One rotor: A_MOT, angular speed rad/s, written to its port.
			class RotorMotor : public Actuator {
			public:
				ActionStatus Apply(const Action& action, double t) override;
				double Speed() const { return speed_; }

			protected:
				void OnConfigure() override;

			private:
				double speed_ = 0.0;
			};

			// Attitude (V_ATT) and body rates (V_RATE); V_ACC when an accelerometer is named.
			class Imu : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;

			protected:
				void OnConfigure() override;
			};

			// V_GEO (frame wgs84) or V_POS3 (frame local).
			class Gnss : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;

			protected:
				void OnConfigure() override;

			private:
				bool local_ = false;
			};

			// V_VEL3: ENU velocity from a source that estimates it, such as an autopilot (spec 14 §5).
			class Velocity : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;

			protected:
				void OnConfigure() override;
			};

			// V_REL3 per peer in range: a relative position sensor (spec 15 §3.1). Its port reads
			// (robot id, dx, dy, dz) for each peer, ENU, one after another.
			class Ranging : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;

			protected:
				void OnConfigure() override;
			};

			// V_MAG: heading = atan2(n_y, n_x) of the body-frame north vector, wrapped to [0, 2π).
			class Compass : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;
				static double Heading(double north_x, double north_y);

			protected:
				void OnConfigure() override;
			};

			// V_BARO: altitude AMSL.
			class Barometer : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;

			protected:
				void OnConfigure() override;
			};

			// A_L: bit k of the mask sets LED k to the colour (0 is off).
			class LedArray : public Actuator {
			public:
				ActionStatus Apply(const Action& action, double t) override;
				std::size_t Count() const { return count_; }

			protected:
				void OnConfigure() override;

			private:
				std::size_t count_ = 0;
			};

			// Energy store, K_Q energy_wh, and V_BAT from its port (energy Wh, optional voltage V). A port
			// that knows only the remaining fraction reads -1, voltage, remaining (an autopilot, spec 14 §3).
			class Battery : public StorageDevice {
			public:
				void Sample(double t, std::vector<View>& out) override;

			protected:
				void OnConfigure() override;

			private:
				double capacity_wh_ = 0.0;
				double voltage_ = 0.0;
			};

			// Broadcast radio: an emitter port and a receiver port.
			class Radio : public CommunicationDevice {
			public:
				bool Send(const std::string& message, const std::string& recipient) override;
				std::vector<std::string> Receive() override;

			protected:
				void OnConfigure() override;
			};

			// A_GMB: gimbal pitch and yaw (rad), each written to a position-mode motor port.
			class Gimbal : public Actuator {
			public:
				ActionStatus Apply(const Action& action, double t) override;

			protected:
				void OnConfigure() override;

			private:
				double min_pitch_ = -1.5708, max_pitch_ = 0.5;
			};

			// A_CAM shots interval_ms: a burst of pictures. Each shot is one write of its index to the
			// camera port; the action is RUNNING until the last shot.
			class Camera : public Actuator {
			public:
				ActionStatus Apply(const Action& action, double t) override;
				std::size_t Shots() const { return shots_; }

			protected:
				void OnConfigure() override;

			private:
				std::uint64_t burst_ = 0;  // argument of the burst in progress
				bool active_ = false;
				std::uint32_t taken_ = 0;
				double next_ = 0.0;
				std::size_t shots_ = 0;    // every shot since start
			};

			// The release half of a payload bay: its port reads 1 while a package is held and takes a
			// write of 0 to open. A_REL package releases the held package, if it is that package.
			class PayloadLatch : public Actuator {
			public:
				ActionStatus Apply(const Action& action, double t) override;
				// The package held now, 0 when the bay is empty.
				std::int64_t Carried();
				void SetPackage(std::int64_t package) { package_ = package; }

			protected:
				void OnConfigure() override;

			private:
				std::int64_t package_ = 0;
				bool held_ = false;
				bool released_ = false;
			};

			// The sense half of a payload bay: V_PAY with the package the latch holds.
			class PayloadSense : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;
				void SetLatch(PayloadLatch* latch) { latch_ = latch; }

			protected:
				void OnConfigure() override;

			private:
				PayloadLatch* latch_ = nullptr;
			};

			// payload.*: expands to latch (A_REL) and cargo (V_PAY). Parameters: device (the latch's
			// port address) and package (the package loaded before the flight, 0 for none).
			class Payload : public ComplexDevice {
			public:
				void Expand(const DeviceRegistry& registry) override;
			};

			// quadrotor.*: expands to m_fl, m_fr, m_rl, m_rr, fcu and imu (spec 04 §3). The fcu is
			// fcu.<platform> when the registry has it (spec 14 §4), otherwise fcu.default.
			class Quadrotor : public ComplexDevice {
			public:
				void Expand(const DeviceRegistry& registry) override;
			};

			// head.default, fcu.default and the .webots keys of spec 04 §3.1.
			void RegisterUavDevices(DeviceRegistry& registry);
		}
	}
}
