#pragma once
// ArduPilot platform (plan §4.4, spec 14 §3): MAVLink ports to a flight controller that flies
// the airframe itself. SITL over TCP or UDP during development; serial-to-UDP or UDP on hardware.
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "mrs/platform/Net.h"
#include "mrs/port/Port.h"

namespace MRS {
	namespace Platform {
		enum class ClockMode {
			WALL,       // seconds since `epoch` on the computer's clock: robots and the operator agree
			AUTOPILOT,  // the autopilot's boot time: follows SITL's speed-up; one robot only
			MANUAL,     // one period per Step, for tests with a simulated autopilot
		};

		struct ArduPilotConfig {
			std::string link = "tcp:127.0.0.1:5760";
			int system_id = 245;       // this companion's MAVLink system id
			int component_id = 191;    // MAV_COMP_ID_ONBOARD_COMPUTER
			int target_system = 0;     // 0: the first autopilot heard
			double period = 0.02;      // s per Step
			double stream_hz = 10.0;   // rate asked for each message the ports use
			ClockMode clock = ClockMode::WALL;
			double epoch = -1.0;       // unix s of mission time 0; below 0: the last UTC midnight
		};

		// What the autopilot last said, in its own units and frames (NED, FRD), decoded.
		struct AutopilotState {
			bool heartbeat = false;
			double heartbeat_t = -1;   // platform time of the last heartbeat
			bool armed = false;
			std::uint32_t custom_mode = 0;
			std::uint8_t landed_state = 0;  // MAV_LANDED_STATE; 0 when the autopilot does not send it
			std::uint32_t boot_ms = 0;
			// GLOBAL_POSITION_INT
			long global_seq = 0;
			double lat = 0, lon = 0, alt_amsl = 0, rel_alt = 0, vn = 0, ve = 0, vd = 0;
			// ATTITUDE
			long att_seq = 0;
			double roll = 0, pitch = 0, yaw = 0, roll_rate = 0, pitch_rate = 0, yaw_rate = 0;
			// LOCAL_POSITION_NED
			long local_seq = 0;
			double north = 0, east = 0, down = 0;
			// BATTERY_STATUS, or SYS_STATUS when no BATTERY_STATUS comes
			long battery_seq = 0;
			double voltage = 0, remaining = -1;  // remaining 0..1, below 0 unknown
			// COMMAND_ACK
			long ack_seq = 0;
			int ack_command = 0, ack_result = 0;
			std::string status_text;  // the last STATUSTEXT
		};

		// GUIDED port commands, values[0] (spec 14 §3.3).
		enum class GuidedCommand {
			SET_MODE = 1,    // custom mode (ArduCopter: 4 GUIDED, 9 LAND, 6 RTL)
			ARM = 2,         // 1 arm, 0 disarm
			TAKEOFF = 3,     // altitude above home, m
			POSITION = 4,    // north east down (m, local NED), yaw (rad, NED)
			VELOCITY = 5,    // vn ve vd (m/s), yaw rate (rad/s, NED)
			SPEED = 6,       // horizontal speed for position targets, m/s
		};

		// Port contracts, type MAVLINK, by address:
		//   GLOBAL_POSITION_INT  Read: lat lon alt_amsl (V_GEO, frame wgs84)
		//   VELOCITY             Read: vx vy vz, ENU m/s (from GLOBAL_POSITION_INT)
		//   ATTITUDE             Read: roll pitch yaw, ENU/FLU rad (spec 00 §3)
		//   ATTITUDE_RATES       Read: p q r, FLU rad/s
		//   BATTERY_STATUS       Read: -1, voltage V, remaining 0..1
		//   RELAY:<n>            Write: on (non-zero) or off, MAV_CMD_DO_SET_RELAY (LEDs on a relay output)
		//   SERVO:<n>            Write: refused (the autopilot drives the motors; the node is for the self model)
		//   GUIDED               Write: a GuidedCommand and its values. Read: heartbeat_ok armed custom_mode
		//                        landed_state local_ok east north up (local ENU) ack_seq ack_command ack_result
		// Sampled ports return true once per new message.
		class ArduPilotPlatform : public Port::IPlatform {
		public:
			// `link` null: opened from config.link.
			explicit ArduPilotPlatform(ArduPilotConfig config, std::unique_ptr<Net::IByteLink> link = nullptr);
			~ArduPilotPlatform() override;

			std::vector<Port::PortInfo> Scan() override;
			std::unique_ptr<Port::Port> Open(const Port::PortAssignment& assignment) override;
			double Time() const override { return time_; }
			bool Step() override;

			// Reads the link until the autopilot's heartbeat and a position fix arrive, or `seconds` pass.
			bool WaitForAutopilot(double seconds);
			void Stop() { stop_ = true; }

			const AutopilotState& State() const { return state_; }
			const ArduPilotConfig& Config() const { return c_; }
			const Net::IByteLink& Link() const { return *link_; }
			long MessagesIn() const { return messages_in_; }

			// Sends a GUIDED command now (the GUIDED port calls this).
			bool Command(const std::vector<double>& values);
			bool SetRelay(int relay, bool on);

		private:
			struct Impl;
			void Pump();
			void Handle(const void* message);
			double Now() const;
			void Housekeeping();
			bool SendCommandLong(int command, const std::vector<float>& params);
			bool SendRaw(const void* message);

			ArduPilotConfig c_;
			std::unique_ptr<Net::IByteLink> link_;
			std::unique_ptr<Impl> impl_;
			AutopilotState state_;
			double time_ = 0.0;
			double wall0_ = 0.0;            // steady clock at construction, s
			double epoch_offset_ = 0.0;     // mission time at construction (WALL)
			double boot0_ = -1.0;           // first boot time seen (AUTOPILOT)
			double next_heartbeat_ = 0.0, next_streams_ = 0.0;
			long messages_in_ = 0;
			bool stop_ = false;
		};

		// ArduCopter flight modes (custom_mode).
		namespace CopterMode {
			constexpr std::uint32_t STABILIZE = 0, GUIDED = 4, LOITER = 5, RTL = 6, LAND = 9;
		}
	}
}
