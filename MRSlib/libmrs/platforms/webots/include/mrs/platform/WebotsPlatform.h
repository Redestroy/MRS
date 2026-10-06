#pragma once
// Webots platform (spec 04 §3, §6): SIM ports are Webots devices, addressed by device name.
// Built only when WEBOTS_HOME is set (spec 07).
#include <memory>
#include <vector>

#include "mrs/port/Port.h"

namespace webots {
	class Robot;
}

namespace MRS {
	namespace Platform {
		// Port contracts, by Webots node type:
		//   GPS            Read: x y z (frame local) or lat lon alt (frame wgs84)
		//   InertialUnit   Read: roll pitch yaw (rad)
		//   Gyro, Accelerometer, Compass   Read: x y z
		//   Altimeter      Read: altitude m
		//   RotationalMotor Write: angular speed rad/s (velocity mode)
		//   LED            Write: colour 0xRRGGBB; 0 is off. With `rgb F` (default) any non-zero value is LED state 1
		//   Emitter        Send; parameters channel, range_m
		//   Receiver       Receive; parameter channel
		//   address "battery"  Read: energy Wh (the Robot.battery field)
		// `rate_hz` sets the sampling period, rounded up to a multiple of the basic time step.
		// Read returns true once per new sample.
		class WebotsPlatform : public Port::IPlatform {
		public:
			explicit WebotsPlatform(webots::Robot& robot);

			std::vector<Port::PortInfo> Scan() override;
			std::unique_ptr<Port::Port> Open(const Port::PortAssignment& assignment) override;
			double Time() const override;
			bool Step() override;

			int BasicStepMs() const { return basic_ms_; }

		private:
			int PeriodMs(const Port::PortAssignment& assignment) const;
			webots::Robot& robot_;
			int basic_ms_;
		};
	}
}
