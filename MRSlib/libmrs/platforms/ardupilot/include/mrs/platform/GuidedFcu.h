#pragma once
// The flight control unit on ArduPilot (spec 14 §4): the generic flight actions of spec 02 as
// GUIDED-mode commands. The autopilot flies the airframe; this node only tells it where to go.
#include <vector>

#include "mrs/device/DeviceRegistry.h"
#include "mrs/device/uav/FlightControlUnit.h"

namespace MRS {
	namespace Platform {
		struct GuidedStatus {
			bool alive = false, armed = false, local_ok = false;
			std::uint32_t mode = 0;
			int landed_state = 0;
			double east = 0, north = 0, up = 0;  // local position, ENU, from the autopilot's origin
		};

		class GuidedFcu : public Device::Uav::FlightControlUnit {
		public:
			Device::ActionStatus Apply(const Device::Action& action, double t) override;
			void Update(double t) override;
			std::vector<const Device::DeviceNode*> DependsOn() const override { return {}; }

			const GuidedStatus& Status() const { return status_; }
			long CommandsSent() const { return sent_; }

		protected:
			void OnConfigure() override;

		private:
			enum class Axis { POSITION, VELOCITY };
			bool ReadStatus(double t);
			bool Send(std::vector<double> values);
			void HoldHere(const Device::Uav::FlightState& s);
			void SendTarget(const Device::Uav::FlightState& s, double t);

			GuidedStatus status_;
			double status_t_ = -1, lost_since_ = -1;
			bool was_armed_ = false;
			long sent_ = 0;

			// Takeoff and landing.
			double takeoff_agl_ = 0, takeoff_started_ = -1, next_try_ = -1, takeoff_sent_ = -1;
			bool takeoff_failed_ = false;
			double arm_timeout_ = 60.0;

			// Commanded values (mission ENU frame), as in the library's own fcu.
			Axis h_mode_ = Axis::POSITION, v_mode_ = Axis::POSITION;
			bool yaw_rate_mode_ = false;
			double px_ = 0, py_ = 0, pz_ = 0, yaw_cmd_ = 0;
			double vx_cmd_ = 0, vy_cmd_ = 0, vz_cmd_ = 0, r_cmd_ = 0;
			double h_vel_stamp_ = -1, v_vel_stamp_ = -1;
			double hold_until_ = -1.0;
			std::uint64_t hold_arg_ = 0;
			double next_send_ = -1, send_period_ = 0.1;
		};

		// The .mavlink type keys of spec 14 §4: quadrotor, motor, imu, gnss, vel, battery and fcu.
		void RegisterArduPilotDevices(Device::DeviceRegistry& registry);
	}
}
