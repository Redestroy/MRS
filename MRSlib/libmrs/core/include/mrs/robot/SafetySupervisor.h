#pragma once
// The safety supervisor (spec 08 §5): wraps the actuator block as the executor's sink and
// keeps every flight action inside the geofence and the flight limits.
#include <array>
#include <optional>

#include "mrs/task/TaskExecutor.h"
#include "mrs/world/Worldview.h"

namespace MRS {
	namespace Robot {
		struct SafetyConfig {
			double fence_margin = 2.0;  // m
			double min_alt = 0.5;       // m above home ground, while airborne
			double max_speed_xy = 8.0;  // m/s
			double max_climb = 2.0;     // m/s
			// Opt-in use of the repulsion (spec 15 §4.3): s of lead for position setpoints; 0: off.
			double repulsion_lead = 0.0;
		};

		// The geofence box of the mission (geofence.xmin ... zmax); z runs from 0.
		struct Fence {
			double xmin, xmax, ymin, ymax, zmax;
			// The box shrunk by m on every side but the ground.
			bool Inside(const std::array<double, 3>& p, double m = 0.0) const;
		};
		std::optional<Fence> FenceOf(const Environment::Worldview& w);

		class SafetySupervisor : public Task::IActionSink {
		public:
			SafetySupervisor(Task::IActionSink& inner, const Environment::Worldview& world, SafetyConfig config = {});

			Device::ActionStatus Dispatch(const Device::ActionMap& action, double t) override;
			// The filtered form of an action at time t (what Dispatch sends on).
			Device::ActionMap Filter(const Device::ActionMap& action, double t) const;

			// Whether a task target is allowed: inside the fence shrunk by fence_margin, and at
			// least min_alt above home ground. True when no fence is known.
			bool TargetAllowed(const std::array<double, 3>& p) const;

			// Actions the filter changed so far.
			long Changed() const { return changed_; }
			const SafetyConfig& Config() const { return c_; }
			void SetConfig(const SafetyConfig& c) { c_ = c; }

		private:
			Task::IActionSink& inner_;
			const Environment::Worldview& w_;
			SafetyConfig c_;
			long changed_ = 0;
		};
	}
}
