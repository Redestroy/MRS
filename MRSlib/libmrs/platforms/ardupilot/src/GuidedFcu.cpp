#include "mrs/platform/GuidedFcu.h"

#include <algorithm>
#include <cmath>

#include "mrs/BuildError.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/platform/ArduPilotPlatform.h"

namespace MRS {
	namespace Platform {
		using Device::ActionStatus;
		using Device::Uav::FcuMode;
		using Device::Uav::FlightState;

		namespace {
			constexpr double kPi = 3.14159265358979323846;

			double Wrap(double a) {
				while (a > kPi) a -= 2 * kPi;
				while (a < -kPi) a += 2 * kPi;
				return a;
			}
			double Cmd(GuidedCommand c) { return static_cast<double>(static_cast<int>(c)); }
		}

		void GuidedFcu::OnConfigure() {
			FlightControlUnit::OnConfigure();
			arm_timeout_ = GetParams().Num("arm_timeout", arm_timeout_);
			DeclarePort("guided", Port::PortType::MAVLINK, GetParams().Text("guided", "GUIDED"), true, RateParams());
		}

		bool GuidedFcu::ReadStatus(double t) {
			Port::Port* port = GetPort("guided");
			std::vector<double> v;
			if (!port || !port->Read(v) || v.size() < 8) return false;
			status_.alive = v[0] > 0.5;
			status_.armed = v[1] > 0.5;
			status_.mode = static_cast<std::uint32_t>(v[2]);
			status_.landed_state = static_cast<int>(v[3]);
			status_.local_ok = v[4] > 0.5;
			status_.east = v[5];
			status_.north = v[6];
			status_.up = v[7];
			status_t_ = t;
			return true;
		}

		bool GuidedFcu::Send(std::vector<double> values) {
			Port::Port* port = GetPort("guided");
			if (!port || !port->Write(values)) return false;
			++sent_;
			return true;
		}

		void GuidedFcu::HoldHere(const FlightState& s) {
			h_mode_ = v_mode_ = Axis::POSITION;
			yaw_rate_mode_ = false;
			px_ = s.x;
			py_ = s.y;
			pz_ = s.z;
			yaw_cmd_ = s.yaw;
		}

		ActionStatus GuidedFcu::Apply(const Device::Action& action, double t) {
			if (!HasCapability(Device::CapabilityKind::Action, action.code)) return ActionStatus::REJECTED;
			const auto v = Device::UnpackReals(Device::ArgLayout::F32X2, action.arg);
			const auto state = state_ ? state_(t) : std::nullopt;
			const std::string& code = action.code;
			if (mode_ == FcuMode::DISARMED && code != "A_TO") {
				if (code == "A_LD" || code == "A_HD") return ActionStatus::DONE;
				return ActionStatus::REJECTED;
			}
			setpoints_[code] = v;
			last_code_ = code;
			if (code != "A_HD") hold_until_ = -1.0;

			if (code == "A_TO") {
				if (mode_ == FcuMode::TAKEOFF) {
					if (takeoff_failed_) {
						mode_ = status_.armed ? FcuMode::FLY : FcuMode::DISARMED;
						return ActionStatus::FAILED;
					}
					const bool reached = state && state->agl >= takeoff_agl_ - g_.takeoff_tol;
					if (!reached) return ActionStatus::RUNNING;
					mode_ = FcuMode::FLY;
					HoldHere(*state);
					return ActionStatus::DONE;
				}
				if (mode_ == FcuMode::FLY && state && std::fabs(state->agl - v[0]) <= g_.takeoff_tol) return ActionStatus::DONE;
				const bool airborne = mode_ == FcuMode::FLY;
				mode_ = FcuMode::TAKEOFF;
				takeoff_agl_ = v[0];
				takeoff_started_ = t;
				takeoff_sent_ = -1;
				next_try_ = -1;
				takeoff_failed_ = false;
				if (airborne && state) {
					// Already flying: a climb or descent to the new height above ground.
					HoldHere(*state);
					pz_ = state->z + (takeoff_agl_ - state->agl);
				}
				return ActionStatus::RUNNING;
			}
			if (code == "A_LD") {
				if (mode_ != FcuMode::LANDING) {
					mode_ = FcuMode::LANDING;
					next_try_ = -1;
				}
				return ActionStatus::RUNNING;
			}

			// Setpoints and hold: FLY.
			if (mode_ == FcuMode::LANDING || mode_ == FcuMode::TAKEOFF) {
				if (state) HoldHere(*state);
				mode_ = FcuMode::FLY;
			}
			if (code == "A_HD") {
				if (hold_until_ < 0.0 || hold_arg_ != action.arg) {
					if (state) HoldHere(*state);
					hold_until_ = v[0] > 0 ? t + v[0] : t;
					hold_arg_ = action.arg;
				}
				if (v[0] <= 0.0) {
					hold_until_ = -1.0;
					return ActionStatus::DONE;
				}
				if (t + 1e-9 < hold_until_) return ActionStatus::RUNNING;
				hold_until_ = -1.0;
				return ActionStatus::DONE;
			}
			if (code == "A_PXY") {
				h_mode_ = Axis::POSITION;
				px_ = v[0];
				py_ = v[1];
			} else if (code == "A_PZY") {
				v_mode_ = Axis::POSITION;
				yaw_rate_mode_ = false;
				pz_ = v[0];
				yaw_cmd_ = v[1];
			} else if (code == "A_VXY") {
				h_mode_ = Axis::VELOCITY;
				vx_cmd_ = v[0];
				vy_cmd_ = v[1];
				h_vel_stamp_ = t;
			} else if (code == "A_VZY") {
				v_mode_ = Axis::VELOCITY;
				yaw_rate_mode_ = true;
				vz_cmd_ = v[0];
				r_cmd_ = v[1];
				v_vel_stamp_ = t;
			}
			return ActionStatus::DONE;
		}

		void GuidedFcu::SendTarget(const FlightState& s, double t) {
			if (t < next_send_) return;
			next_send_ = t + send_period_;
			if (!status_.local_ok) return;
			// The autopilot's local frame starts at its own origin; the mission frame at the mission's
			// geo reference. Both come from the same estimate, so their difference is the offset.
			const double ox = s.x - status_.east, oy = s.y - status_.north, oz = s.z - status_.up;
			if (h_mode_ == Axis::POSITION && v_mode_ == Axis::POSITION) {
				Send({Cmd(GuidedCommand::POSITION), py_ - oy, px_ - ox, -(pz_ - oz), Wrap(kPi / 2 - yaw_cmd_)});
				return;
			}
			// Mixed axes: ArduPilot takes a target in one kind only, so position axes become velocities.
			double vx = vx_cmd_, vy = vy_cmd_, vz = vz_cmd_;
			if (h_mode_ == Axis::POSITION) {
				vx = g_.kp_pos * (px_ - s.x);
				vy = g_.kp_pos * (py_ - s.y);
			}
			const double vxy = std::hypot(vx, vy);
			if (vxy > g_.max_speed_xy) {
				vx *= g_.max_speed_xy / vxy;
				vy *= g_.max_speed_xy / vxy;
			}
			if (v_mode_ == Axis::POSITION) vz = g_.kp_pos * (pz_ - s.z);
			vz = std::clamp(vz, -g_.max_climb, g_.max_climb);
			double r = yaw_rate_mode_ ? r_cmd_ : g_.kp_yaw * Wrap(yaw_cmd_ - s.yaw);
			r = std::clamp(r, -g_.max_yaw_rate, g_.max_yaw_rate);
			Send({Cmd(GuidedCommand::VELOCITY), vy, vx, -vz, -r});
		}

		void GuidedFcu::Update(double t) {
			const bool fresh = ReadStatus(t);
			// No word from the autopilot: InFailsafe, which the safety layer reads (spec 08 §6).
			if (!fresh || !status_.alive) {
				if (lost_since_ < 0) lost_since_ = t;
				failsafe_ = t - lost_since_ > 2.0 && mode_ != FcuMode::DISARMED;
				if (!status_.alive) return;
			} else {
				lost_since_ = -1;
				failsafe_ = false;
			}
			const bool armed = status_.armed;
			const auto state = state_ ? state_(t) : std::nullopt;

			if (mode_ == FcuMode::DISARMED) {
				was_armed_ = armed;
				return;
			}
			// Disarmed by the autopilot (landed, a crash, its own failsafe): the flight is over.
			if (was_armed_ && !armed && mode_ != FcuMode::TAKEOFF) {
				mode_ = FcuMode::DISARMED;
				was_armed_ = false;
				return;
			}
			if (armed) was_armed_ = true;

			if (mode_ == FcuMode::TAKEOFF) {
				if (!armed) {
					if (t - takeoff_started_ > arm_timeout_) {
						takeoff_failed_ = true;
						return;
					}
					if (t >= next_try_) {
						next_try_ = t + 1.0;
						if (status_.mode != CopterMode::GUIDED) Send({Cmd(GuidedCommand::SET_MODE), static_cast<double>(CopterMode::GUIDED)});
						else Send({Cmd(GuidedCommand::ARM), 1.0});
					}
					return;
				}
				if (state && state->agl > 1.0 && takeoff_sent_ < 0) {
					// Airborne already: climb with a position target instead of a takeoff command.
					SendTarget(*state, t);
					return;
				}
				if (takeoff_sent_ < 0 || (t - takeoff_sent_ > 3.0 && (!state || state->agl < 0.3))) {
					Send({Cmd(GuidedCommand::SPEED), g_.max_speed_xy});
					Send({Cmd(GuidedCommand::TAKEOFF), takeoff_agl_});
					takeoff_sent_ = t;
				}
				return;
			}
			if (mode_ == FcuMode::LANDING) {
				if (t >= next_try_ && status_.mode != CopterMode::LAND) {
					next_try_ = t + 1.0;
					Send({Cmd(GuidedCommand::SET_MODE), static_cast<double>(CopterMode::LAND)});
				}
				return;
			}
			// FLY.
			if (!state) return;
			if (h_mode_ == Axis::VELOCITY && t - h_vel_stamp_ > g_.setpoint_timeout) {
				h_mode_ = Axis::POSITION;
				px_ = state->x;
				py_ = state->y;
			}
			if (v_mode_ == Axis::VELOCITY && t - v_vel_stamp_ > g_.setpoint_timeout) {
				v_mode_ = Axis::POSITION;
				pz_ = state->z;
				yaw_rate_mode_ = false;
				yaw_cmd_ = state->yaw;
			}
			SendTarget(*state, t);
		}

		void RegisterArduPilotDevices(Device::DeviceRegistry& r) {
			using Device::IdValue;
			using Device::StrValue;
			r.Register<Device::Uav::Quadrotor>("quadrotor.mavlink", {{"port", IdValue("MAVLINK")},
			                                                         {"motor_fl", StrValue("SERVO:3")},
			                                                         {"motor_fr", StrValue("SERVO:1")},
			                                                         {"motor_rl", StrValue("SERVO:2")},
			                                                         {"motor_rr", StrValue("SERVO:4")},
			                                                         {"inertial", StrValue("ATTITUDE")},
			                                                         {"gyro", StrValue("ATTITUDE_RATES")}});
			r.Register<Device::Uav::RotorMotor>("motor.mavlink", {{"port", IdValue("MAVLINK")}});
			r.Register<Device::Uav::Imu>("imu.mavlink",
			                             {{"port", IdValue("MAVLINK")}, {"inertial", StrValue("ATTITUDE")}, {"gyro", StrValue("ATTITUDE_RATES")}});
			r.Register<GuidedFcu>("fcu.mavlink", {{"guided", StrValue("GUIDED")}});
			r.Register<Device::Uav::Gnss>("gnss.mavlink", {{"port", IdValue("MAVLINK")}, {"device", StrValue("GLOBAL_POSITION_INT")}});
			r.Register<Device::Uav::Velocity>("vel.mavlink", {{"port", IdValue("MAVLINK")}, {"device", StrValue("VELOCITY")}});
			r.Register<Device::Uav::LedArray>("led.mavlink", {{"port", IdValue("MAVLINK")}});
			r.Register<Device::Uav::Battery>("battery.mavlink", {{"port", IdValue("MAVLINK")}, {"device", StrValue("BATTERY_STATUS")}});
		}
	}
}
