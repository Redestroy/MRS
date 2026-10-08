#pragma once
// The task family (spec 03 §1-§3, §5, §6). Tasks are built from protocol records by
// TaskFactory; the TaskExecutor keeps the run-time state (iterators, contexts).
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mrs/protocol/Record.h"
#include "mrs/task/Condition.h"
#include "mrs/task/TaskAction.h"

namespace MRS {
	namespace Task {
		enum class TaskType { ATOMIC, PARAMETRIC, BEHAVIOUR, SEQUENTIAL, PARALLEL, CHOICE };

		// Values keep the 2021 MRS::Task::TaskState numbering (spec 03 §3.1).
		enum class TaskState { NEW = 0, IDLE = 1, QUEUED = 2, STARTED = 3, IN_PROGRESS = 4, SUCCEEDED = 5, FAILED = 6 };

		// Failure reasons (spec 03 §8.7).
		enum class FailReason {
			NONE,
			NO_BEHAVIOUR,
			START_UNREACHABLE,
			END_NOT_MET,
			MISSING_FIELD,
			RUNTIME_CONDITION,
			NO_ACTUATOR,
			ACTUATOR_FAILED,
			IMPOSSIBLE,
			PREEMPTED_OUT,
			ABORTED
		};

		const char* TaskStateName(TaskState s);
		const char* FailReasonName(FailReason r);

		// What a robot can do, from its self model (spec 04 §7), for static requirement checks.
		struct CapabilityProfile {
			std::set<std::string> actions;  // action codes
			std::set<std::string> fields;   // worldview fields it can provide, e.g. "pose.enu"
			std::set<std::string> roles;
			std::string robot;  // "r<id>", for R_I
		};

		// A requirement record R_* (spec 03 §5).
		struct Requirement {
			std::string code;
			double max_age = -1.0;            // R_F
			std::vector<std::string> items;   // R_F fields, R_A action codes
			std::string name;                 // R_R role, R_E resource, R_K task id, R_I robot
			double amount = 0.0;              // R_E
			std::shared_ptr<const Condition> condition;  // R_C
			std::vector<Requirement> children;           // R_S
		};

		class Task {
		public:
			virtual ~Task() = default;
			virtual std::unique_ptr<Task> Clone() const = 0;

			TaskType Type() const { return type_; }
			const std::string& Code() const { return code_; }
			const std::string& Id() const { return id_; }
			double Priority() const { return priority_; }
			// Own priority times every ancestor's priority (spec 03 §2).
			double EffectivePriority() const { return effective_priority_; }
			TaskState State() const { return state_; }
			void SetState(TaskState s) { state_ = s; }

			const Condition& StartCondition() const { return *start_; }
			const Condition& EndCondition() const { return *end_; }
			std::shared_ptr<const Condition> StartConditionPtr() const { return start_; }
			const std::optional<Requirement>& Requirements() const { return requirements_; }

			// The R_C conditions in the requirements (spec 03 §8.5).
			std::vector<std::shared_ptr<const Condition>> RuntimeConditions() const;

			// Implicit and explicit requirements (spec 03 §5).
			virtual void RequiredFields(std::vector<std::string>& out) const;
			virtual void RequiredActions(std::vector<std::string>& out) const;
			bool MeetsStaticRequirements(const CapabilityProfile& profile) const;

			// The record the task was built from, with the current state in its state slot
			// (for journals, spec 06 §8).
			Protocol::Record ToRecord() const;

			// Set by the factory.
			void SetCommon(std::string code, TaskType type, std::string id, double priority, TaskState state,
			               std::shared_ptr<const Condition> start, std::shared_ptr<const Condition> end,
			               std::optional<Requirement> requirements, Protocol::Record source);
			void SetId(std::string id);
			void SetEffectivePriority(double p) { effective_priority_ = p; }

		protected:
			Task() = default;
			Task(const Task&) = default;

		private:
			std::string code_;
			TaskType type_ = TaskType::ATOMIC;
			std::string id_ = "0";
			double priority_ = 1.0;
			double effective_priority_ = 1.0;
			TaskState state_ = TaskState::NEW;
			std::shared_ptr<const Condition> start_, end_;
			std::optional<Requirement> requirements_;
			Protocol::Record source_;
		};

		// T_A: an atomic task with a list of actions.
		class ATask : public Task {
		public:
			std::unique_ptr<Task> Clone() const override { return std::make_unique<ATask>(*this); }
			const std::vector<TaskAction>& Actions() const { return actions_; }
			void SetActions(std::vector<TaskAction> actions) { actions_ = std::move(actions); }
			void RequiredFields(std::vector<std::string>& out) const override;
			void RequiredActions(std::vector<std::string>& out) const override;

		private:
			std::vector<TaskAction> actions_;
		};

		// T_P: like T_A, and its actions may be parametric (A_FN).
		class ParametricATask : public ATask {
		public:
			std::unique_ptr<Task> Clone() const override { return std::make_unique<ParametricATask>(*this); }
		};

		// T_B: repeats its base task until `until` holds (spec 03 §8.4).
		class Behaviour : public Task {
		public:
			Behaviour() = default;
			Behaviour(const Behaviour& other);
			std::unique_ptr<Task> Clone() const override { return std::make_unique<Behaviour>(*this); }

			const Condition& Until() const { return *until_; }
			std::shared_ptr<const Condition> UntilPtr() const { return until_; }
			void SetUntil(std::shared_ptr<const Condition> until) { until_ = std::move(until); }
			// The base task. Version 0.1 runs a T_A or T_P base; its own start and end
			// conditions are not evaluated (the behaviour's are).
			const ATask& Base() const { return *base_; }
			void SetBase(std::unique_ptr<ATask> base) { base_ = std::move(base); }
			void RequiredFields(std::vector<std::string>& out) const override;
			void RequiredActions(std::vector<std::string>& out) const override;

		private:
			std::shared_ptr<const Condition> until_;
			std::unique_ptr<ATask> base_;
		};

		// T_S, T_L and T_O (spec 03 §6).
		class ComplexTask : public Task {
		public:
			ComplexTask() = default;
			ComplexTask(const ComplexTask& other);
			std::unique_ptr<Task> Clone() const override { return std::make_unique<ComplexTask>(*this); }

			const std::vector<std::unique_ptr<Task>>& Children() const { return children_; }
			void AddChild(std::unique_ptr<Task> child) { children_.push_back(std::move(child)); }
			// T_L: the number of children that must succeed; 0 means all.
			long long RequiredSuccesses() const { return k_; }
			void SetRequiredSuccesses(long long k) { k_ = k; }
			void RequiredFields(std::vector<std::string>& out) const override;
			void RequiredActions(std::vector<std::string>& out) const override;

		private:
			std::vector<std::unique_ptr<Task>> children_;
			long long k_ = 0;
		};

		using TaskPtr = std::unique_ptr<Task>;
	}
}
