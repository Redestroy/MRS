#include "mrs/device/uav/FlightControlUnit.h"

#include <algorithm>
#include <cmath>

#include "mrs/device/uav/UavDevices.h"

namespace MRS {
	namespace Device {
		namespace Uav {
			namespace {
				constexpr double kPi = 3.14159265358979323846;
				constexpr double kG = 9.81;

				double Wrap(double a) {
					while (a > kPi) a -= 2 * kPi;
					while (a < -kPi) a += 2 * kPi;
					return a;
				}
			}

			const char* FcuModeName(FcuMode m) {
				switch (m) {
				case FcuMode::DISARMED: return "DISARMED";
				case FcuMode::TAKEOFF: return "TAKEOFF";
				case FcuMode::FLY: return "FLY";
				case FcuMode::LANDING: return "LANDING";
				}
				return "DISARMED";
			}

			void FlightControlUnit::OnConfigure() {
				const Params& p = GetParams();
				auto num = [&](const char* key, double& v) { v = p.Num(key, v); };
				num("hover_speed", g_.hover_speed);
				num("max_motor", g_.max_motor);
				num("max_speed_xy", g_.max_speed_xy);
				num("max_climb", g_.max_climb);
				num("max_tilt", g_.max_tilt);
				num("max_yaw_rate", g_.max_yaw_rate);
				num("kp_pos", g_.kp_pos);
				num("kp_vel", g_.kp_vel);
				num("ki_vel", g_.ki_vel);
				num("kv", g_.kv);
				num("ki_z", g_.ki_z);
				num("kp_yaw", g_.kp_yaw);
				num("kr", g_.kr);
				num("kp_att", g_.kp_att);
				num("kd_att", g_.kd_att);
				num("setpoint_timeout", g_.setpoint_timeout);
				num("state_timeout", g_.state_timeout);
				num("takeoff_tol", g_.takeoff_tol);
				for (const char* code : {"A_TO", "A_LD", "A_HD", "A_PXY", "A_PZY", "A_VXY", "A_VZY"})
					Declare(Capability::Act(code, {{"max_climb", g_.max_climb}, {"max_speed_xy", g_.max_speed_xy}}));
			}

			std::vector<const DeviceNode*> FlightControlUnit::DependsOn() const {
				std::vector<const DeviceNode*> deps;
				for (RotorMotor* m : motors_)
					if (m) deps.push_back(m);
				return deps;
			}

			void FlightControlUnit::HoldHere(const FlightState& s) {
				h_mode_ = v_mode_ = Axis::POSITION;
				yaw_rate_mode_ = false;
				px_ = s.x;
				py_ = s.y;
				pz_ = s.z;
				yaw_cmd_ = s.yaw;
			}

			ActionStatus FlightControlUnit::Apply(const Action& action, double t) {
				if (!HasCapability(CapabilityKind::Action, action.code)) return ActionStatus::REJECTED;
				const auto v = UnpackReals(ArgLayout::F32X2, action.arg);
				const auto state = state_ ? state_(t) : std::nullopt;
				const std::string& code = action.code;
				if (mode_ == FcuMode::DISARMED && code != "A_TO") {
					// On the ground and still: a landing or a hold is already done.
					if (code == "A_LD" || code == "A_HD") {
						return ActionStatus::DONE;
					}
					return ActionStatus::REJECTED;
				}
				setpoints_[code] = v;
				last_code_ = code;
				if (code != "A_HD") hold_until_ = -1.0;

				if (code == "A_TO") {
					if (mode_ == FcuMode::TAKEOFF) {
						const bool reached = state && std::fabs(state->agl - takeoff_agl_) <= g_.takeoff_tol;
						if (!reached) return ActionStatus::RUNNING;
						mode_ = FcuMode::FLY;
						HoldHere(*state);
						return ActionStatus::DONE;
					}
					if (mode_ == FcuMode::FLY && state && std::fabs(state->agl - v[0]) <= g_.takeoff_tol) return ActionStatus::DONE;
					mode_ = FcuMode::TAKEOFF;
					takeoff_agl_ = v[0];
					rate_ = v[1] > 0 ? std::min<double>(v[1], g_.max_climb) : g_.max_climb;
					if (state) {
						HoldHere(*state);
						pz_ = state->z + (takeoff_agl_ - state->agl);
					} else {
						target_pending_ = true;
					}
					return ActionStatus::RUNNING;
				}
				if (code == "A_LD") {
					if (mode_ != FcuMode::LANDING) {
						mode_ = FcuMode::LANDING;
						landed_since_ = -1;
						rate_ = v[0] > 0 ? std::min<double>(v[0], g_.max_climb) : 0.7;
						if (state) HoldHere(*state);
					}
					return ActionStatus::RUNNING;
				}

				// Setpoints and hold: FLY.
				if (mode_ == FcuMode::LANDING || mode_ == FcuMode::TAKEOFF) {
					if (mode_ == FcuMode::TAKEOFF && state) HoldHere(*state);
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

			void FlightControlUnit::Update(double t) {
				const double dt = last_t_ < 0 ? 0.0 : std::min(t - last_t_, 0.1);
				last_t_ = t;
				if (mode_ == FcuMode::DISARMED) {
					commands_ = {0, 0, 0, 0};
					ivx_ = ivy_ = iz_ = 0;
					failsafe_ = false;
					WriteMotors(t);
					return;
				}
				const auto state = state_ ? state_(t) : std::nullopt;
				if (!state) {
					// A short gap (the first ticks, one late sample) keeps the last commands.
					if (missing_since_ < 0) missing_since_ = t;
					if (t - missing_since_ < g_.state_timeout) {
						WriteMotors(t);
						return;
					}
					// Failsafe: level, descend just below hover.
					failsafe_ = true;
					const double base = g_.hover_speed * 0.97 + iz_;
					commands_ = {base, -base, -base, base};
					WriteMotors(t);
					return;
				}
				failsafe_ = false;
				missing_since_ = -1;
				const FlightState& s = *state;
				if (target_pending_) {
					HoldHere(s);
					pz_ = s.z + (takeoff_agl_ - s.agl);
					target_pending_ = false;
				}

				// Velocity setpoints that are not renewed become a position hold.
				if (h_mode_ == Axis::VELOCITY && t - h_vel_stamp_ > g_.setpoint_timeout) {
					h_mode_ = Axis::POSITION;
					px_ = s.x;
					py_ = s.y;
				}
				if (v_mode_ == Axis::VELOCITY && t - v_vel_stamp_ > g_.setpoint_timeout) {
					v_mode_ = Axis::POSITION;
					pz_ = s.z;
					yaw_rate_mode_ = false;
					yaw_cmd_ = s.yaw;
				}

				// 1. Position -> velocity.
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
				double climb_limit = g_.max_climb, descent_limit = g_.max_climb;
				if (mode_ == FcuMode::TAKEOFF) climb_limit = rate_;
				if (mode_ == FcuMode::LANDING) {
					vz = -rate_;
					if (s.agl < 1.0) vz = -std::max(0.3, rate_ * 0.5);  // slow down near the ground
				} else {
					if (v_mode_ == Axis::POSITION) vz = g_.kp_pos * (pz_ - s.z);
					vz = std::clamp(vz, -descent_limit, climb_limit);
				}

				// 2. Velocity -> acceleration (PI).
				const double ex = vx - s.vx, ey = vy - s.vy, ez = vz - s.vz;
				const double i_lim = 2.0;
				ivx_ = std::clamp(ivx_ + ex * dt, -i_lim, i_lim);
				ivy_ = std::clamp(ivy_ + ey * dt, -i_lim, i_lim);
				double ax = g_.kp_vel * ex + g_.ki_vel * ivx_, ay = g_.kp_vel * ey + g_.ki_vel * ivy_;
				const double a_max = kG * std::tan(g_.max_tilt);
				const double axy = std::hypot(ax, ay);
				if (axy > a_max) {
					ax *= a_max / axy;
					ay *= a_max / axy;
				}
				// Vertical: the integrator learns the hover offset (not while sitting on the ground).
				const bool on_ground = s.agl < 0.1 && mode_ != FcuMode::FLY && s.vz < 0.2;
				if (!(mode_ == FcuMode::LANDING && on_ground)) iz_ = std::clamp(iz_ + g_.ki_z * ez * dt, -0.5 * g_.hover_speed, 0.5 * g_.hover_speed);
				double u_z = g_.kv * ez + iz_;

				// 3. Acceleration -> attitude, in the yaw frame.
				const double c = std::cos(s.yaw), sn = std::sin(s.yaw);
				const double a_fwd = c * ax + sn * ay, a_left = -sn * ax + c * ay;
				const double pitch_cmd = std::clamp(a_fwd / kG, -g_.max_tilt, g_.max_tilt);
				const double roll_cmd = std::clamp(-a_left / kG, -g_.max_tilt, g_.max_tilt);

				// 4. Yaw.
				double r_cmd = yaw_rate_mode_ ? r_cmd_ : g_.kp_yaw * Wrap(yaw_cmd_ - s.yaw);
				r_cmd = std::clamp(r_cmd, -g_.max_yaw_rate, g_.max_yaw_rate);
				double u_yaw = std::clamp(g_.kr * (r_cmd - s.r), -10.0, 10.0);

				// 5. Attitude -> torque inputs.
				double u_roll = std::clamp(g_.kp_att * (s.roll - roll_cmd) + g_.kd_att * s.p, -30.0, 30.0);
				double u_pitch = std::clamp(g_.kp_att * (s.pitch - pitch_cmd) + g_.kd_att * s.q, -30.0, 30.0);

				// Landing: once down and still, disarm.
				if (mode_ == FcuMode::LANDING) {
					const double speed = std::sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz);
					if (s.agl < 0.15 && speed < 0.2) {
						if (landed_since_ < 0) landed_since_ = t;
						if (t - landed_since_ >= 0.5) {
							mode_ = FcuMode::DISARMED;
							commands_ = {0, 0, 0, 0};
							WriteMotors(t);
							return;
						}
					} else {
						landed_since_ = -1;
					}
					if (s.agl < 0.15) {  // touching down: cut the torque inputs and thrust
						u_roll = u_pitch = u_yaw = 0;
						u_z = std::min(u_z, -0.2 * g_.hover_speed);
					}
				}
				// TAKEOFF tilts nothing until clear of the ground.
				if (mode_ == FcuMode::TAKEOFF && s.agl < 0.2) u_yaw = 0;

				// 6. Mixer.
				const double base = g_.hover_speed + u_z;
				auto clampm = [&](double w) { return std::clamp(w, 0.0, g_.max_motor); };
				commands_ = {clampm(base - u_roll + u_pitch - u_yaw), -clampm(base + u_roll + u_pitch + u_yaw),
				             -clampm(base - u_roll - u_pitch + u_yaw), clampm(base + u_roll - u_pitch - u_yaw)};
				WriteMotors(t);
			}

			void FlightControlUnit::WriteMotors(double t) {
				for (std::size_t k = 0; k < 4; ++k) {
					if (!motors_[k]) continue;
					motors_[k]->Apply({"A_MOT", *PackArgument(ArgLayout::F64, std::vector<double>{commands_[k]})}, t);
				}
			}
		}
	}
}
