#include "mrs/task/Task.h"

#include <algorithm>

namespace MRS {
	namespace Task {
		const char* TaskStateName(TaskState s) {
			switch (s) {
			case TaskState::NEW: return "NEW";
			case TaskState::IDLE: return "IDLE";
			case TaskState::QUEUED: return "QUEUED";
			case TaskState::STARTED: return "STARTED";
			case TaskState::IN_PROGRESS: return "IN_PROGRESS";
			case TaskState::SUCCEEDED: return "SUCCEEDED";
			case TaskState::FAILED: return "FAILED";
			}
			return "?";
		}

		const char* FailReasonName(FailReason r) {
			switch (r) {
			case FailReason::NONE: return "NONE";
			case FailReason::NO_BEHAVIOUR: return "NO_BEHAVIOUR";
			case FailReason::START_UNREACHABLE: return "START_UNREACHABLE";
			case FailReason::END_NOT_MET: return "END_NOT_MET";
			case FailReason::MISSING_FIELD: return "MISSING_FIELD";
			case FailReason::RUNTIME_CONDITION: return "RUNTIME_CONDITION";
			case FailReason::NO_ACTUATOR: return "NO_ACTUATOR";
			case FailReason::ACTUATOR_FAILED: return "ACTUATOR_FAILED";
			case FailReason::IMPOSSIBLE: return "IMPOSSIBLE";
			case FailReason::PREEMPTED_OUT: return "PREEMPTED_OUT";
			case FailReason::ABORTED: return "ABORTED";
			}
			return "?";
		}

		namespace {
			void CollectRuntime(const Requirement& r, std::vector<std::shared_ptr<const Condition>>& out) {
				if (r.code == "R_C" && r.condition) out.push_back(r.condition);
				for (const auto& c : r.children) CollectRuntime(c, out);
			}

			void CollectFields(const Requirement& r, std::vector<std::string>& out) {
				if (r.code == "R_F") out.insert(out.end(), r.items.begin(), r.items.end());
				if (r.code == "R_C" && r.condition) r.condition->Fields(out);
				for (const auto& c : r.children) CollectFields(c, out);
			}

			void CollectActions(const Requirement& r, std::vector<std::string>& out) {
				if (r.code == "R_A") out.insert(out.end(), r.items.begin(), r.items.end());
				for (const auto& c : r.children) CollectActions(c, out);
			}

			bool RolesMet(const Requirement& r, const CapabilityProfile& p) {
				if (r.code == "R_R" && !p.roles.count(r.name)) return false;
				for (const auto& c : r.children)
					if (!RolesMet(c, p)) return false;
				return true;
			}

			// Fields the mission or the executor writes, not the robot's sensors.
			bool IsMissionField(const std::string& f) {
				for (const char* prefix : {"target.", "task.", "home.", "layer."})
					if (f.rfind(prefix, 0) == 0) return true;
				return f == "time";
			}

			// "pose.enu.x" is provided when the profile lists "pose.enu" (or the component itself).
			bool Provides(const CapabilityProfile& p, const std::string& field) {
				std::string f = field;
				while (true) {
					if (p.fields.count(f)) return true;
					auto dot = f.rfind('.');
					if (dot == std::string::npos) return false;
					f.resize(dot);
				}
			}
		}

		void Task::SetCommon(std::string code, TaskType type, std::string id, double priority, TaskState state,
		                     std::shared_ptr<const Condition> start, std::shared_ptr<const Condition> end,
		                     std::optional<Requirement> requirements, Protocol::Record source) {
			code_ = std::move(code);
			type_ = type;
			id_ = std::move(id);
			priority_ = priority;
			effective_priority_ = priority;
			state_ = state;
			start_ = std::move(start);
			end_ = std::move(end);
			requirements_ = std::move(requirements);
			source_ = std::move(source);
		}

		void Task::SetId(std::string id) {
			id_ = std::move(id);
			if (!source_.fields.empty()) source_.fields[0].s = id_;
		}

		std::vector<std::shared_ptr<const Condition>> Task::RuntimeConditions() const {
			std::vector<std::shared_ptr<const Condition>> out;
			if (requirements_) CollectRuntime(*requirements_, out);
			return out;
		}

		void Task::RequiredFields(std::vector<std::string>& out) const {
			start_->Fields(out);
			end_->Fields(out);
			if (requirements_) CollectFields(*requirements_, out);
		}

		void Task::RequiredActions(std::vector<std::string>& out) const {
			if (requirements_) CollectActions(*requirements_, out);
		}

		bool Task::MeetsStaticRequirements(const CapabilityProfile& profile) const {
			std::vector<std::string> fields, actions;
			RequiredFields(fields);
			RequiredActions(actions);
			for (const auto& f : fields)
				if (!IsMissionField(f) && !Provides(profile, f)) return false;
			for (const auto& a : actions)
				if (!profile.actions.count(a)) return false;
			return !requirements_ || RolesMet(*requirements_, profile);
		}

		Protocol::Record Task::ToRecord() const {
			Protocol::Record r = source_;
			if (r.fields.size() > 2) r.fields[2].i = static_cast<std::int64_t>(state_);
			return r;
		}

		void ATask::RequiredFields(std::vector<std::string>& out) const {
			Task::RequiredFields(out);
			for (const auto& a : actions_) a.Fields(out);
		}

		void ATask::RequiredActions(std::vector<std::string>& out) const {
			Task::RequiredActions(out);
			for (const auto& a : actions_) a.ActionCodes(out);
		}

		Behaviour::Behaviour(const Behaviour& other)
		    : Task(other), until_(other.until_), base_(other.base_ ? std::make_unique<ATask>(*other.base_) : nullptr) {}

		void Behaviour::RequiredFields(std::vector<std::string>& out) const {
			Task::RequiredFields(out);
			if (base_) base_->RequiredFields(out);
		}

		void Behaviour::RequiredActions(std::vector<std::string>& out) const {
			Task::RequiredActions(out);
			if (base_) base_->RequiredActions(out);
		}

		ComplexTask::ComplexTask(const ComplexTask& other) : Task(other), k_(other.k_) {
			for (const auto& c : other.children_) children_.push_back(c->Clone());
		}

		void ComplexTask::RequiredFields(std::vector<std::string>& out) const {
			Task::RequiredFields(out);
			for (const auto& c : children_) c->RequiredFields(out);
		}

		void ComplexTask::RequiredActions(std::vector<std::string>& out) const {
			Task::RequiredActions(out);
			for (const auto& c : children_) c->RequiredActions(out);
		}
	}
}
