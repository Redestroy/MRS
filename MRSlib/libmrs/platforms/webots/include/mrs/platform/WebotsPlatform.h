#pragma once
// Webots platform (spec 04 §3, §6): SIM ports are Webots devices, addressed by device name.
// Built only when WEBOTS_HOME is set (spec 07).
#include <map>
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
		//   address "ranging"  Read: (robot id, dx, dy, dz) ENU per peer heard by the radio receiver within
		//                      1/rate_hz and range_m (spec 15 §3.1); parameter inertial (default "inertial unit")
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

			// What the radio receiver heard of each peer: the emitter's direction and signal strength.
			struct Bearing {
				double t = -1.0;
				double dir[3] = {0.0, 0.0, 0.0};  // unit vector, receiver frame
				double strength = 0.0;            // 1 / r^2
			};
			using Bearings = std::map<int, Bearing>;  // by robot id

		private:
			int PeriodMs(const Port::PortAssignment& assignment) const;
			webots::Robot& robot_;
			int basic_ms_;
			std::shared_ptr<Bearings> bearings_ = std::make_shared<Bearings>();
		};
	}
}
