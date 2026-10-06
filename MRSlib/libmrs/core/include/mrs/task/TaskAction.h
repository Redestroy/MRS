#pragma once
// One entry of a task's action list: a leaf action, a combined action (A_MAP) or a
// parametric action (A_FN) whose values are computed every tick (spec 02 §5, §6).
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mrs/device/Action.h"
#include "mrs/task/Function.h"

namespace MRS {
	namespace Task {
		class TaskAction {
		public:
			static TaskAction Leaf(Device::Action action);
			static TaskAction Map(std::vector<std::pair<std::string, TaskAction>> entries);
			static TaskAction Parametric(std::string code, std::vector<std::shared_ptr<const Function>> functions);

			// "A_N", "A_PXY", "A_MAP", "A_FN", ...
			const std::string& Code() const { return code_; }
			bool IsMap() const { return code_ == "A_MAP"; }
			bool IsParametric() const { return code_ == "A_FN"; }
			// A_FN anywhere in this action.
			bool ContainsParametric() const;
			// The leaf action, for A_N, A_W, A_I and other leaf codes.
			const Device::Action& Action() const { return action_; }
			// For A_W: the wait in seconds.
			double WaitSeconds() const;

			// Evaluates the functions and packs the values (spec 02 §2, §6). nullopt when a
			// field a function reads is missing or stale (MISSING_FIELD).
			std::optional<Device::ActionMap> Resolve(const Environment::Worldview& w, double t) const;

			void Fields(std::vector<std::string>& out) const;
			// Action codes that reach an actuator, for the implicit R_A requirement.
			void ActionCodes(std::vector<std::string>& out) const;

		private:
			std::string code_;
			Device::Action action_;
			std::string produced_;  // A_FN: the code it produces
			std::vector<std::shared_ptr<const Function>> functions_;
			std::vector<std::pair<std::string, TaskAction>> entries_;  // A_MAP
		};
	}
}
