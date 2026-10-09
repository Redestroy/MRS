#include "mrs/platform/ArduPilotPlatform.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "MavlinkInclude.h"

namespace MRS {
	namespace Platform {
		namespace P = ::MRS::Port;

		namespace {
			constexpr double kPi = 3.14159265358979323846;

			double SteadySeconds() {
				return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
			}
			double UnixSeconds() {
				return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
			}

			// Reads the state through a member pointer to the message's counter.
			class SampledPort : public P::Port {
			public:
				using Fill = void (*)(const AutopilotState&, std::vector<double>&);
				SampledPort(P::PortAssignment a, const ArduPilotPlatform& platform, const long AutopilotState::*seq, Fill fill)
				    : P::Port(std::move(a)), platform_(platform), seq_(seq), fill_(fill) {}
				bool Read(std::vector<double>& values) override {
					const long seq = platform_.State().*seq_;
					if (seq == 0 || seq == last_) return false;
					last_ = seq;
					fill_(platform_.State(), values);
					return true;
				}

			private:
				const ArduPilotPlatform& platform_;
				const long AutopilotState::*seq_;
				Fill fill_;
				long last_ = 0;
			};

			class ServoPort : public P::Port {
			public:
				using P::Port::Port;
				bool Write(const std::vector<double>&) override { return false; }
			};

			class RelayPort : public P::Port {
			public:
				RelayPort(P::PortAssignment a, ArduPilotPlatform& platform, int relay) : P::Port(std::move(a)), platform_(platform), relay_(relay) {}
				bool Write(const std::vector<double>& values) override {
					if (values.empty()) return false;
					const bool on = values[0] != 0.0;
					if (sent_ && on == on_) return true;  // only changes go on the link
					sent_ = true;
					on_ = on;
					return platform_.SetRelay(relay_, on);
				}

			private:
				ArduPilotPlatform& platform_;
				int relay_;
				bool sent_ = false, on_ = false;
			};

			class GuidedPort : public P::Port {
			public:
				GuidedPort(P::PortAssignment a, ArduPilotPlatform& platform) : P::Port(std::move(a)), platform_(platform) {}
				bool Write(const std::vector<double>& values) override { return platform_.Command(values); }
				bool Read(std::vector<double>& values) override {
					const AutopilotState& s = platform_.State();
					const bool alive = s.heartbeat && platform_.Time() - s.heartbeat_t < 3.0;
					values = {alive ? 1.0 : 0.0,
					          s.armed ? 1.0 : 0.0,
					          static_cast<double>(s.custom_mode),
					          static_cast<double>(s.landed_state),
					          s.local_seq > 0 ? 1.0 : 0.0,
					          s.east,
					          s.north,
					          -s.down,
					          static_cast<double>(s.ack_seq),
					          static_cast<double>(s.ack_command),
					          static_cast<double>(s.ack_result)};
					return s.heartbeat;
				}

			private:
				ArduPilotPlatform& platform_;
			};

			struct Channel {
				const char* address;
				const long AutopilotState::*seq;
				SampledPort::Fill fill;
			};

			const Channel kChannels[] = {
				{"GLOBAL_POSITION_INT", &AutopilotState::global_seq,
				 [](const AutopilotState& s, std::vector<double>& v) { v = {s.lat, s.lon, s.alt_amsl}; }},
				{"VELOCITY", &AutopilotState::global_seq,
				 [](const AutopilotState& s, std::vector<double>& v) { v = {s.ve, s.vn, -s.vd}; }},
				{"ATTITUDE", &AutopilotState::att_seq,
				 [](const AutopilotState& s, std::vector<double>& v) {
					 double yaw = kPi / 2 - s.yaw;  // NED yaw (from North, clockwise) to ENU (from East, counter-clockwise)
					 while (yaw > kPi) yaw -= 2 * kPi;
					 while (yaw < -kPi) yaw += 2 * kPi;
					 v = {s.roll, -s.pitch, yaw};
				 }},
				{"ATTITUDE_RATES", &AutopilotState::att_seq,
				 [](const AutopilotState& s, std::vector<double>& v) { v = {s.roll_rate, -s.pitch_rate, -s.yaw_rate}; }},
				{"BATTERY_STATUS", &AutopilotState::battery_seq,
				 [](const AutopilotState& s, std::vector<double>& v) { v = {-1.0, s.voltage, s.remaining}; }},
			};
		}

		struct ArduPilotPlatform::Impl {
			mavlink_message_t rx{};
			mavlink_status_t rx_status{};
			mavlink_message_t frame{};
			mavlink_status_t frame_status{};
			int target_component = 1;
			bool battery_status_seen = false;
		};

		ArduPilotPlatform::ArduPilotPlatform(ArduPilotConfig config, std::unique_ptr<Net::IByteLink> link)
		    : c_(std::move(config)), link_(std::move(link)), impl_(std::make_unique<Impl>()) {
			if (!link_) link_ = Net::OpenLink(c_.link);
			wall0_ = SteadySeconds();
			if (c_.clock == ClockMode::WALL) {
				const double unix_now = UnixSeconds();
				const double epoch = c_.epoch >= 0.0 ? c_.epoch : std::floor(unix_now / 86400.0) * 86400.0;
				epoch_offset_ = unix_now - epoch;
			}
			time_ = Now();
		}

		ArduPilotPlatform::~ArduPilotPlatform() = default;

		double ArduPilotPlatform::Now() const {
			switch (c_.clock) {
			case ClockMode::WALL: return epoch_offset_ + (SteadySeconds() - wall0_);
			case ClockMode::AUTOPILOT: return boot0_ < 0.0 ? 0.0 : state_.boot_ms / 1000.0 - boot0_;
			case ClockMode::MANUAL: return time_;
			}
			return time_;
		}

		std::vector<P::PortInfo> ArduPilotPlatform::Scan() {
			std::vector<P::PortInfo> out;
			const std::string identity = state_.heartbeat ? "ardupilot" : "";
			for (const auto& ch : kChannels) out.push_back({P::PortType::MAVLINK, ch.address, identity});
			out.push_back({P::PortType::MAVLINK, "GUIDED", identity});
			for (int k = 1; k <= 8; ++k) out.push_back({P::PortType::MAVLINK, "SERVO:" + std::to_string(k), identity});
			for (int k = 0; k <= 5; ++k) out.push_back({P::PortType::MAVLINK, "RELAY:" + std::to_string(k), identity});
			return out;
		}

		std::unique_ptr<P::Port> ArduPilotPlatform::Open(const P::PortAssignment& a) {
			if (a.type != P::PortType::MAVLINK) return nullptr;
			for (const auto& ch : kChannels)
				if (a.address == ch.address) return std::make_unique<SampledPort>(a, *this, ch.seq, ch.fill);
			if (a.address == "GUIDED") return std::make_unique<GuidedPort>(a, *this);
			if (a.address.rfind("SERVO:", 0) == 0) return std::make_unique<ServoPort>(a);
			if (a.address.rfind("RELAY:", 0) == 0) {
				try {
					return std::make_unique<RelayPort>(a, *this, std::stoi(a.address.substr(6)));
				} catch (const std::exception&) {
					return nullptr;
				}
			}
			return nullptr;
		}

		void ArduPilotPlatform::Pump() {
			std::uint8_t buf[2048];
			for (int round = 0; round < 64; ++round) {
				const std::size_t n = link_->Read(buf, sizeof(buf));
				if (n == 0) break;
				for (std::size_t i = 0; i < n; ++i) {
					if (mavlink_frame_char_buffer(&impl_->frame, &impl_->frame_status, buf[i], &impl_->rx, &impl_->rx_status) ==
					    MAVLINK_FRAMING_OK)
						Handle(&impl_->rx);
				}
			}
		}

		void ArduPilotPlatform::Handle(const void* raw) {
			const auto& m = *static_cast<const mavlink_message_t*>(raw);
			if (m.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
				mavlink_heartbeat_t hb;
				mavlink_msg_heartbeat_decode(&m, &hb);
				if (hb.autopilot == MAV_AUTOPILOT_INVALID) return;  // a GCS or another companion
				if (c_.target_system == 0) c_.target_system = static_cast<int>(m.sysid);
				if (static_cast<int>(m.sysid) != c_.target_system) return;
				impl_->target_component = m.compid;
				state_.heartbeat = true;
				state_.heartbeat_t = time_;
				state_.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
				state_.custom_mode = hb.custom_mode;
				++messages_in_;
				return;
			}
			if (c_.target_system != 0 && static_cast<int>(m.sysid) != c_.target_system) return;
			++messages_in_;
			switch (static_cast<std::uint32_t>(m.msgid)) {
			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
				mavlink_global_position_int_t g;
				mavlink_msg_global_position_int_decode(&m, &g);
				state_.boot_ms = g.time_boot_ms;
				state_.lat = g.lat / 1e7;
				state_.lon = g.lon / 1e7;
				state_.alt_amsl = g.alt / 1000.0;
				state_.rel_alt = g.relative_alt / 1000.0;
				state_.vn = g.vx / 100.0;
				state_.ve = g.vy / 100.0;
				state_.vd = g.vz / 100.0;
				++state_.global_seq;
				break;
			}
			case MAVLINK_MSG_ID_ATTITUDE: {
				mavlink_attitude_t a;
				mavlink_msg_attitude_decode(&m, &a);
				state_.boot_ms = a.time_boot_ms;
				state_.roll = a.roll;
				state_.pitch = a.pitch;
				state_.yaw = a.yaw;
				state_.roll_rate = a.rollspeed;
				state_.pitch_rate = a.pitchspeed;
				state_.yaw_rate = a.yawspeed;
				++state_.att_seq;
				break;
			}
			case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
				mavlink_local_position_ned_t l;
				mavlink_msg_local_position_ned_decode(&m, &l);
				state_.north = l.x;
				state_.east = l.y;
				state_.down = l.z;
				++state_.local_seq;
				break;
			}
			case MAVLINK_MSG_ID_BATTERY_STATUS: {
				mavlink_battery_status_t b;
				mavlink_msg_battery_status_decode(&m, &b);
				if (b.id != 0) break;
				impl_->battery_status_seen = true;
				if (b.voltages[0] != UINT16_MAX) state_.voltage = b.voltages[0] / 1000.0;
				state_.remaining = b.battery_remaining >= 0 ? b.battery_remaining / 100.0 : -1.0;
				++state_.battery_seq;
				break;
			}
			case MAVLINK_MSG_ID_SYS_STATUS: {
				if (impl_->battery_status_seen) break;
				mavlink_sys_status_t s;
				mavlink_msg_sys_status_decode(&m, &s);
				state_.voltage = s.voltage_battery / 1000.0;
				state_.remaining = s.battery_remaining >= 0 ? s.battery_remaining / 100.0 : -1.0;
				++state_.battery_seq;
				break;
			}
			case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
				mavlink_extended_sys_state_t e;
				mavlink_msg_extended_sys_state_decode(&m, &e);
				state_.landed_state = e.landed_state;
				break;
			}
			case MAVLINK_MSG_ID_COMMAND_ACK: {
				mavlink_command_ack_t a;
				mavlink_msg_command_ack_decode(&m, &a);
				state_.ack_command = a.command;
				state_.ack_result = a.result;
				++state_.ack_seq;
				break;
			}
			case MAVLINK_MSG_ID_STATUSTEXT: {
				mavlink_statustext_t t;
				mavlink_msg_statustext_decode(&m, &t);
				state_.status_text.assign(t.text, strnlen(t.text, sizeof(t.text)));
				break;
			}
			default: break;
			}
			if (c_.clock == ClockMode::AUTOPILOT && boot0_ < 0.0 && state_.boot_ms > 0) boot0_ = state_.boot_ms / 1000.0;
		}

		bool ArduPilotPlatform::SendRaw(const void* raw) {
			const auto& m = *static_cast<const mavlink_message_t*>(raw);
			std::uint8_t buf[MAVLINK_MAX_PACKET_LEN];
			const std::uint16_t n = mavlink_msg_to_send_buffer(buf, &m);
			return link_->Write(buf, n);
		}

		bool ArduPilotPlatform::SendCommandLong(int command, const std::vector<float>& p) {
			if (c_.target_system == 0) return false;
			float v[7] = {0, 0, 0, 0, 0, 0, 0};
			for (std::size_t i = 0; i < p.size() && i < 7; ++i) v[i] = p[i];
			mavlink_message_t m;
			mavlink_msg_command_long_pack(static_cast<std::uint8_t>(c_.system_id), static_cast<std::uint8_t>(c_.component_id), &m,
			                              static_cast<std::uint8_t>(c_.target_system), static_cast<std::uint8_t>(impl_->target_component),
			                              static_cast<std::uint16_t>(command), 0, v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
			return SendRaw(&m);
		}

		bool ArduPilotPlatform::SetRelay(int relay, bool on) {
			return SendCommandLong(MAV_CMD_DO_SET_RELAY, {static_cast<float>(relay), on ? 1.0f : 0.0f});
		}

		bool ArduPilotPlatform::Command(const std::vector<double>& v) {
			if (v.empty() || c_.target_system == 0) return false;
			auto arg = [&](std::size_t i) { return i < v.size() ? v[i] : 0.0; };
			const auto ts = static_cast<std::uint8_t>(c_.target_system), tc = static_cast<std::uint8_t>(impl_->target_component);
			switch (static_cast<GuidedCommand>(static_cast<int>(v[0]))) {
			case GuidedCommand::SET_MODE:
				return SendCommandLong(MAV_CMD_DO_SET_MODE, {static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), static_cast<float>(arg(1))});
			case GuidedCommand::ARM: return SendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, {static_cast<float>(arg(1))});
			case GuidedCommand::TAKEOFF: return SendCommandLong(MAV_CMD_NAV_TAKEOFF, {0, 0, 0, 0, 0, 0, static_cast<float>(arg(1))});
			case GuidedCommand::SPEED: return SendCommandLong(MAV_CMD_DO_CHANGE_SPEED, {1, static_cast<float>(arg(1)), -1});
			case GuidedCommand::POSITION: {
				const std::uint16_t mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
				                           POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
				                           POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
				                           POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
				mavlink_message_t m;
				mavlink_msg_set_position_target_local_ned_pack(
				    static_cast<std::uint8_t>(c_.system_id), static_cast<std::uint8_t>(c_.component_id), &m, state_.boot_ms, ts, tc,
				    MAV_FRAME_LOCAL_NED, mask, static_cast<float>(arg(1)), static_cast<float>(arg(2)), static_cast<float>(arg(3)), 0, 0, 0, 0,
				    0, 0, static_cast<float>(arg(4)), 0);
				return SendRaw(&m);
			}
			case GuidedCommand::VELOCITY: {
				const std::uint16_t mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE |
				                           POSITION_TARGET_TYPEMASK_Z_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
				                           POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
				                           POSITION_TARGET_TYPEMASK_YAW_IGNORE;
				mavlink_message_t m;
				mavlink_msg_set_position_target_local_ned_pack(
				    static_cast<std::uint8_t>(c_.system_id), static_cast<std::uint8_t>(c_.component_id), &m, state_.boot_ms, ts, tc,
				    MAV_FRAME_LOCAL_NED, mask, 0, 0, 0, static_cast<float>(arg(1)), static_cast<float>(arg(2)), static_cast<float>(arg(3)), 0,
				    0, 0, 0, static_cast<float>(arg(4)));
				return SendRaw(&m);
			}
			}
			return false;
		}

		void ArduPilotPlatform::Housekeeping() {
			const double wall = SteadySeconds() - wall0_;
			if (wall >= next_heartbeat_) {
				next_heartbeat_ = wall + 1.0;
				mavlink_message_t m;
				mavlink_msg_heartbeat_pack(static_cast<std::uint8_t>(c_.system_id), static_cast<std::uint8_t>(c_.component_id), &m,
				                           MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
				SendRaw(&m);
			}
			if (c_.target_system != 0 && wall >= next_streams_) {
				next_streams_ = wall + 10.0;
				const float us = static_cast<float>(1e6 / c_.stream_hz), slow = 500000.0f;
				const std::pair<int, float> streams[] = {{MAVLINK_MSG_ID_GLOBAL_POSITION_INT, us},
				                                         {MAVLINK_MSG_ID_ATTITUDE, us},
				                                         {MAVLINK_MSG_ID_LOCAL_POSITION_NED, us},
				                                         {MAVLINK_MSG_ID_BATTERY_STATUS, slow},
				                                         {MAVLINK_MSG_ID_SYS_STATUS, slow},
				                                         {MAVLINK_MSG_ID_EXTENDED_SYS_STATE, slow}};
				for (const auto& [id, interval] : streams) SendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL, {static_cast<float>(id), interval});
			}
		}

		bool ArduPilotPlatform::WaitForAutopilot(double seconds) {
			const double end = SteadySeconds() + seconds;
			while (SteadySeconds() < end) {
				Pump();
				Housekeeping();
				// A position of 0, 0 means the autopilot's estimator has no origin yet.
				if (state_.heartbeat && state_.global_seq > 0 && (state_.lat != 0.0 || state_.lon != 0.0)) return true;
				link_->Wait(0.05);
			}
			return false;
		}

		bool ArduPilotPlatform::Step() {
			if (stop_) return false;
			switch (c_.clock) {
			case ClockMode::MANUAL:
				Pump();
				time_ += c_.period;
				break;
			case ClockMode::WALL: {
				const double target = time_ + c_.period;
				for (;;) {
					Pump();
					const double now = Now();
					if (now >= target) {
						// Behind by more than a period (a slow tick): catch up instead of bursting.
						time_ = now;
						break;
					}
					link_->Wait(std::min(target - now, 0.005));
				}
				break;
			}
			case ClockMode::AUTOPILOT: {
				const double target = time_ + c_.period;
				const double give_up = SteadySeconds() + 0.5;
				for (;;) {
					Pump();
					if (Now() >= target || SteadySeconds() > give_up) break;
					link_->Wait(0.002);
				}
				time_ = std::max(time_, Now());
				break;
			}
			}
			Housekeeping();
			return !stop_;
		}
	}
}
