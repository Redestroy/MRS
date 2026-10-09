#pragma once
// The flight control unit (spec 08 §2): flight actions in, four motor speeds out.
#include <array>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "mrs/device/DeviceNode.h"

namespace MRS {
	namespace Device {
		namespace Uav {
			class RotorMotor;

			// What the fcu needs from the worldview (spec 08 §2.1). ENU, rad, body rates.
			struct FlightState {
				double x = 0, y = 0, z = 0;
				double vx = 0, vy = 0, vz = 0;
				double roll = 0, pitch = 0, yaw = 0;
				double p = 0, q = 0, r = 0;
				double agl = 0;
			};
			using FlightStateSource = std::function<std::optional<FlightState>(double t)>;

			enum class FcuMode { DISARMED, TAKEOFF, FLY, LANDING };
			const char* FcuModeName(FcuMode m);

			struct FcuGains {
				double hover_speed = 68.5, max_motor = 576;
				double max_speed_xy = 8, max_climb = 2, max_tilt = 0.35, max_yaw_rate = 1.0;
				double kp_pos = 0.9, kp_vel = 1.6, ki_vel = 0.5;
				double kv = 4.0, ki_z = 6.0;
				double kp_yaw = 1.5, kr = 6.0;
				double kp_att = 50, kd_att = 4;
				double setpoint_timeout = 0.5, takeoff_tol = 0.3, state_timeout = 0.25;
			};

			class FlightControlUnit : public Actuator {
			public:
				ActionStatus Apply(const Action& action, double t) override;
				// The cascade of spec 08 §2.3, one control tick.
				void Update(double t) override;
				std::vector<const DeviceNode*> DependsOn() const override;

				void SetMotors(std::array<RotorMotor*, 4> motors) { motors_ = motors; }
				const std::array<RotorMotor*, 4>& Motors() const { return motors_; }
				void SetStateSource(FlightStateSource source) { state_ = std::move(source); }

				FcuMode Mode() const { return mode_; }
				bool Armed() const { return mode_ != FcuMode::DISARMED; }
				bool InFailsafe() const { return failsafe_; }
				const FcuGains& Gains() const { return g_; }
				// The last values of each flight action code (two halves each).
				const std::map<std::string, std::vector<double>>& Setpoints() const { return setpoints_; }
				const std::string& LastCode() const { return last_code_; }
				// The last motor speed commands, m_fl, m_fr, m_rl, m_rr (signed).
				const std::array<double, 4>& MotorCommands() const { return commands_; }

			protected:
				void OnConfigure() override;

				// For flight controllers that run elsewhere (spec 14 §4): the same actions, gains and
				// state, with their own Apply and Update.
				FcuGains g_;
				FlightStateSource state_;
				FcuMode mode_ = FcuMode::DISARMED;
				bool failsafe_ = false;
				std::map<std::string, std::vector<double>> setpoints_;
				std::string last_code_;

			private:
				enum class Axis { POSITION, VELOCITY };
				void HoldHere(const FlightState& s);
				void WriteMotors(double t);

				std::array<RotorMotor*, 4> motors_{};
				double missing_since_ = -1;  // no flight state since

				// Commanded values.
				Axis h_mode_ = Axis::POSITION, v_mode_ = Axis::POSITION;
				bool yaw_rate_mode_ = false;
				double px_ = 0, py_ = 0, pz_ = 0, yaw_cmd_ = 0;
				double vx_cmd_ = 0, vy_cmd_ = 0, vz_cmd_ = 0, r_cmd_ = 0;
				double h_vel_stamp_ = -1, v_vel_stamp_ = -1;
				double rate_ = 1.0;          // climb rate of TAKEOFF, descent rate of LANDING
				double takeoff_agl_ = 0;
				bool target_pending_ = false;  // TAKEOFF: set the target z from the first state
				double landed_since_ = -1;

				// Controller state.
				double ivx_ = 0, ivy_ = 0, iz_ = 0;
				double last_t_ = -1;
				std::array<double, 4> commands_{};

				double hold_until_ = -1.0;
				std::uint64_t hold_arg_ = 0;
			};
		}
	}
}
