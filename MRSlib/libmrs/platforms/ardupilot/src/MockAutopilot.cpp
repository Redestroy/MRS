#include "mrs/platform/MockAutopilot.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "MavlinkInclude.h"

namespace MRS {
	namespace Platform {
		namespace {
			constexpr double kPi = 3.14159265358979323846;

			struct Pipe {
				std::deque<std::uint8_t> ab, ba;
			};

			class LoopbackEnd : public Net::IByteLink {
			public:
				LoopbackEnd(std::shared_ptr<Pipe> pipe, bool a) : pipe_(std::move(pipe)), a_(a) {}
				bool Write(const std::uint8_t* data, std::size_t size) override {
					auto& q = a_ ? pipe_->ab : pipe_->ba;
					q.insert(q.end(), data, data + size);
					return true;
				}
				std::size_t Read(std::uint8_t* data, std::size_t capacity) override {
					auto& q = a_ ? pipe_->ba : pipe_->ab;
					const std::size_t n = std::min(capacity, q.size());
					std::copy(q.begin(), q.begin() + static_cast<std::ptrdiff_t>(n), data);
					q.erase(q.begin(), q.begin() + static_cast<std::ptrdiff_t>(n));
					return n;
				}
				void Wait(double) override {}
				bool Connected() const override { return true; }
				std::string Describe() const override { return "loopback"; }

			private:
				std::shared_ptr<Pipe> pipe_;
				bool a_;
			};

			double Wrap(double a) {
				while (a > kPi) a -= 2 * kPi;
				while (a < -kPi) a += 2 * kPi;
				return a;
			}

			constexpr std::uint32_t kGuided = 4, kLand = 9, kRtl = 6, kLoiter = 5, kStabilize = 0;
		}

		std::pair<std::unique_ptr<Net::IByteLink>, std::unique_ptr<Net::IByteLink>> MakeLoopbackPair() {
			auto pipe = std::make_shared<Pipe>();
			return {std::make_unique<LoopbackEnd>(pipe, true), std::make_unique<LoopbackEnd>(pipe, false)};
		}

		struct MockAutopilot::Impl {
			mavlink_message_t rx{}, frame{};
			mavlink_status_t rx_status{}, frame_status{};
		};

		MockAutopilot::MockAutopilot(MockAutopilotConfig config)
		    : c_(config), impl_(std::make_unique<Impl>()), geo_(config.home_lat, config.home_lon, config.home_alt) {
			auto pair = MakeLoopbackPair();
			mine_ = std::move(pair.first);
			theirs_ = std::move(pair.second);
			speed_ = c_.max_speed;
		}

		MockAutopilot::~MockAutopilot() = default;

		std::unique_ptr<Net::IByteLink> MockAutopilot::TakeLink() { return std::move(theirs_); }

		void MockAutopilot::SendRaw(const void* raw) {
			std::uint8_t buf[MAVLINK_MAX_PACKET_LEN];
			const auto n = mavlink_msg_to_send_buffer(buf, static_cast<const mavlink_message_t*>(raw));
			mine_->Write(buf, n);
		}

		void MockAutopilot::Ack(int command, int result) {
			mavlink_message_t m;
			mavlink_msg_command_ack_pack(static_cast<std::uint8_t>(c_.system_id), 1, &m, static_cast<std::uint16_t>(command),
			                             static_cast<std::uint8_t>(result), 0, 0, 0, 0);
			SendRaw(&m);
		}

		void MockAutopilot::Handle(const void* raw) {
			const auto& m = *static_cast<const mavlink_message_t*>(raw);
			const bool on_ground = pos_[2] > -0.05;
			if (m.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
				mavlink_command_long_t c;
				mavlink_msg_command_long_decode(&m, &c);
				if (c.target_system != c_.system_id) return;
				++commands_;
				int result = MAV_RESULT_ACCEPTED;
				switch (c.command) {
				case MAV_CMD_DO_SET_MODE: {
					const auto mode = static_cast<std::uint32_t>(c.param2);
					if (mode == kGuided || mode == kLand || mode == kRtl || mode == kLoiter || mode == kStabilize) {
						mode_ = mode;
						if (mode_ != kGuided) taking_off_ = false;
					} else {
						result = MAV_RESULT_DENIED;
					}
					break;
				}
				case MAV_CMD_COMPONENT_ARM_DISARM:
					if (c.param1 > 0.5) {
						if (t_ < c_.ready_after || mode_ != kGuided) result = MAV_RESULT_TEMPORARILY_REJECTED;
						else if (!armed_) {
							armed_ = true;
							armed_at_ = t_;
						}
					} else if (on_ground) {
						armed_ = false;
					} else {
						result = MAV_RESULT_DENIED;
					}
					break;
				case MAV_CMD_NAV_TAKEOFF:
					if (!armed_ || mode_ != kGuided || !on_ground) {
						result = MAV_RESULT_DENIED;
					} else {
						taking_off_ = true;
						takeoff_alt_ = c.param7;
						target_pos_ = {pos_[0], pos_[1], -takeoff_alt_};
						target_ = Target::POSITION;
					}
					break;
				case MAV_CMD_DO_CHANGE_SPEED:
					if (c.param2 > 0) speed_ = c.param2;
					break;
				case MAV_CMD_SET_MESSAGE_INTERVAL: break;
				default: result = MAV_RESULT_UNSUPPORTED; break;
				}
				Ack(c.command, result);
				return;
			}
			if (m.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
				mavlink_set_position_target_local_ned_t p;
				mavlink_msg_set_position_target_local_ned_decode(&m, &p);
				if (p.target_system != c_.system_id || mode_ != kGuided || !armed_ || taking_off_) return;
				++targets_;
				if (!(p.type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE)) {
					target_ = Target::POSITION;
					target_pos_ = {p.x, p.y, p.z};
					if (!(p.type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE)) {
						yaw_target_ = p.yaw;
						yaw_rate_ = 0.0;
					}
				} else {
					target_ = Target::VELOCITY;
					target_vel_ = {p.vx, p.vy, p.vz};
					target_t_ = t_;
					if (!(p.type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)) yaw_rate_ = p.yaw_rate;
				}
			}
		}

		void MockAutopilot::Step(double dt) {
			std::uint8_t buf[2048];
			for (;;) {
				const std::size_t n = mine_->Read(buf, sizeof(buf));
				if (n == 0) break;
				for (std::size_t i = 0; i < n; ++i)
					if (mavlink_frame_char_buffer(&impl_->frame, &impl_->frame_status, buf[i], &impl_->rx, &impl_->rx_status) == MAVLINK_FRAMING_OK)
						Handle(&impl_->rx);
			}
			t_ += dt;

			// Desired velocity, NED.
			std::array<double, 3> want{0, 0, 0};
			const bool on_ground = pos_[2] > -0.05;
			if (armed_) {
				if (mode_ == kLand || mode_ == kRtl) {
					want = {0, 0, c_.descent};
				} else if (mode_ == kGuided) {
					if (target_ == Target::VELOCITY && t_ - target_t_ > 3.0) {
						target_ = Target::POSITION;
						target_pos_ = pos_;
					}
					if (target_ == Target::POSITION) {
						for (int k = 0; k < 3; ++k) want[k] = 1.0 * (target_pos_[k] - pos_[k]);
					} else if (target_ == Target::VELOCITY) {
						want = target_vel_;
					}
					const double h = std::hypot(want[0], want[1]);
					const double limit = taking_off_ ? 0.0 : speed_;
					if (h > limit && h > 0) {
						want[0] *= limit / h;
						want[1] *= limit / h;
					}
					want[2] = std::clamp(want[2], -c_.climb, c_.descent * 1.5);
					if (taking_off_ && -pos_[2] >= takeoff_alt_ - 0.1) taking_off_ = false;
				}
			}
			const double a = std::min(1.0, dt / c_.tau);
			for (int k = 0; k < 3; ++k) {
				vel_[k] += (want[k] - vel_[k]) * a;
				pos_[k] += vel_[k] * dt;
			}
			if (pos_[2] > 0.0) {  // the ground
				pos_[2] = 0.0;
				vel_ = {0, 0, 0};
			}
			max_speed_seen_ = std::max(max_speed_seen_, std::hypot(vel_[0], vel_[1]));
			if (target_ == Target::POSITION && yaw_rate_ == 0.0) {
				const double e = Wrap(yaw_target_ - yaw_);
				yaw_ += std::clamp(e, -dt, dt);  // 1 rad/s
			} else {
				yaw_ = Wrap(yaw_ + yaw_rate_ * dt);
			}
			if (armed_) remaining_ = std::max(0.0, remaining_ - dt / c_.endurance);

			// Disarm on the ground: after landing, or when no takeoff came.
			if (armed_ && on_ground && pos_[2] > -0.01) {
				if (mode_ == kLand || mode_ == kRtl) {
					if (on_ground_since_ < 0) on_ground_since_ = t_;
					if (t_ - on_ground_since_ > 1.0) {
						armed_ = false;
						on_ground_since_ = -1;
						target_ = Target::HOLD;
					}
				} else if (!taking_off_ && t_ - armed_at_ > c_.disarm_delay) {
					armed_ = false;
				}
			} else {
				on_ground_since_ = -1;
			}
			Emit();
		}

		void MockAutopilot::Emit() {
			const auto sys = static_cast<std::uint8_t>(c_.system_id);
			const auto boot = static_cast<std::uint32_t>(t_ * 1000.0);
			mavlink_message_t m;
			if (t_ >= next_slow_) {
				next_slow_ = t_ + 1.0;
				const std::uint8_t base = static_cast<std::uint8_t>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (armed_ ? MAV_MODE_FLAG_SAFETY_ARMED : 0));
				mavlink_msg_heartbeat_pack(sys, 1, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, base, mode_,
				                           armed_ ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
				SendRaw(&m);
				std::uint16_t cells[10];
				std::fill(std::begin(cells), std::end(cells), UINT16_MAX);
				cells[0] = static_cast<std::uint16_t>(11100 + 1500 * remaining_);
				std::uint16_t ext[4] = {0, 0, 0, 0};
				mavlink_msg_battery_status_pack(sys, 1, &m, 0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, INT16_MAX, cells, -1, -1, -1,
				                                static_cast<std::int8_t>(std::lround(remaining_ * 100)), 0, MAV_BATTERY_CHARGE_STATE_OK, ext, 0, 0);
				SendRaw(&m);
				const bool on_ground = pos_[2] > -0.05;
				mavlink_msg_extended_sys_state_pack(sys, 1, &m, MAV_VTOL_STATE_UNDEFINED,
				                                    !armed_ || on_ground ? MAV_LANDED_STATE_ON_GROUND : MAV_LANDED_STATE_IN_AIR);
				SendRaw(&m);
			}
			if (t_ < next_stream_) return;
			next_stream_ = t_ + 1.0 / c_.stream_hz;
			double lat, lon, alt;
			geo_.ToGeodetic({pos_[1], pos_[0], -pos_[2]}, lat, lon, alt);
			double hdg = yaw_ * 180.0 / kPi;
			if (hdg < 0) hdg += 360.0;
			mavlink_msg_global_position_int_pack(sys, 1, &m, boot, static_cast<std::int32_t>(std::lround(lat * 1e7)),
			                                     static_cast<std::int32_t>(std::lround(lon * 1e7)), static_cast<std::int32_t>(std::lround(alt * 1000)),
			                                     static_cast<std::int32_t>(std::lround(-pos_[2] * 1000)), static_cast<std::int16_t>(std::lround(vel_[0] * 100)),
			                                     static_cast<std::int16_t>(std::lround(vel_[1] * 100)), static_cast<std::int16_t>(std::lround(vel_[2] * 100)),
			                                     static_cast<std::uint16_t>(std::lround(hdg * 100)) % 36000);
			SendRaw(&m);
			mavlink_msg_attitude_pack(sys, 1, &m, boot, 0, 0, static_cast<float>(yaw_), 0, 0, static_cast<float>(yaw_rate_));
			SendRaw(&m);
			mavlink_msg_local_position_ned_pack(sys, 1, &m, boot, static_cast<float>(pos_[0]), static_cast<float>(pos_[1]), static_cast<float>(pos_[2]),
			                                    static_cast<float>(vel_[0]), static_cast<float>(vel_[1]), static_cast<float>(vel_[2]));
			SendRaw(&m);
		}
	}
}
