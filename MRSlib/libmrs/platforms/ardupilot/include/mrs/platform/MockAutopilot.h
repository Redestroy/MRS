#pragma once
// A stand-in for ArduCopter that speaks MAVLink over an in-process link (spec 14 §7): GUIDED,
// LAND, arming, takeoff, position and velocity targets, a kinematic airframe. For tests and CI,
// where SITL is not available. Not a model of ArduPilot's controllers.
#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "mrs/platform/Net.h"
#include "mrs/world/GeoReference.h"

namespace MRS {
	namespace Platform {
		struct MockAutopilotConfig {
			int system_id = 1;
			double home_lat = 56.9496, home_lon = 24.1052, home_alt = 10.0;  // the autopilot's origin
			double ready_after = 2.0;   // s before it may arm (EKF settling)
			double max_speed = 10.0;    // m/s, until a DO_CHANGE_SPEED
			double climb = 2.5, descent = 1.0;  // m/s; LAND descends at `descent`
			double tau = 0.4;           // s, velocity response
			double endurance = 1200.0;  // s of flight from full to empty
			double stream_hz = 10.0;
			double disarm_delay = 10.0; // s armed on the ground without a takeoff
		};

		// Two ends of a byte pipe: what one writes the other reads.
		std::pair<std::unique_ptr<Net::IByteLink>, std::unique_ptr<Net::IByteLink>> MakeLoopbackPair();

		class MockAutopilot {
		public:
			explicit MockAutopilot(MockAutopilotConfig config = {});
			~MockAutopilot();

			// The companion's end of the link; call once and give it to ArduPilotPlatform.
			std::unique_ptr<Net::IByteLink> TakeLink();
			// Reads commands, moves the airframe by dt and sends what is due.
			void Step(double dt);

			double Time() const { return t_; }
			bool Armed() const { return armed_; }
			std::uint32_t Mode() const { return mode_; }
			// Local NED from the home point.
			const std::array<double, 3>& Position() const { return pos_; }
			double Remaining() const { return remaining_; }
			long Commands() const { return commands_; }
			long Targets() const { return targets_; }
			double MaxSpeedSeen() const { return max_speed_seen_; }

		private:
			struct Impl;
			void Handle(const void* message);
			void Ack(int command, int result);
			void Emit();
			void SendRaw(const void* message);

			MockAutopilotConfig c_;
			std::unique_ptr<Impl> impl_;
			std::unique_ptr<Net::IByteLink> mine_, theirs_;
			Environment::GeoReference geo_;
			double t_ = 0.0, next_stream_ = 0.0, next_slow_ = 0.0;
			bool armed_ = false;
			std::uint32_t mode_ = 0;  // STABILIZE
			double armed_at_ = -1.0;
			bool taking_off_ = false;
			double takeoff_alt_ = 0.0;
			std::array<double, 3> pos_{0, 0, 0}, vel_{0, 0, 0};
			double yaw_ = 0.0, yaw_target_ = 0.0, yaw_rate_ = 0.0;
			enum class Target { HOLD, POSITION, VELOCITY } target_ = Target::HOLD;
			std::array<double, 3> target_pos_{0, 0, 0}, target_vel_{0, 0, 0};
			double target_t_ = 0.0;
			double speed_ = 10.0;
			double remaining_ = 1.0;
			double on_ground_since_ = -1.0;
			long commands_ = 0, targets_ = 0;
			double max_speed_seen_ = 0.0;
		};
	}
}
