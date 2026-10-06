#pragma once
// The task executor (spec 03 §8): an active-task stack, start-condition fulfilment
// through the behaviour library, the action iterator, behaviours, complex tasks on one
// robot (spec 03 §6.1), runtime conditions and preemption.
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mrs/device/Action.h"
#include "mrs/task/BehaviourLibrary.h"
#include "mrs/task/Task.h"
#include "mrs/world/GeoReference.h"
#include "mrs/world/Worldview.h"

namespace MRS {
	namespace Task {
		// The device side of dispatch (spec 04 §4.2). WP1 tests use a scripted sink.
		class IActionSink {
		public:
			virtual ~IActionSink() = default;
			virtual Device::ActionStatus Dispatch(const Device::ActionMap& action, double t) = 0;
		};

		struct ExecutorConfig {
			int max_behaviour_depth = 2;
			int max_fulfil_attempts = 3;
			double start_unknown_timeout = 5.0;  // s
		};

		struct TaskEvent {
			std::string task_id;  // a behaviour reports its library name
			TaskState state;
			FailReason reason = FailReason::NONE;
			bool behaviour = false;  // pushed to fulfil a start condition
		};

		struct TickResult {
			std::optional<Device::ActionMap> dispatched;  // at most one per tick
			std::vector<TaskEvent> events;
		};

		class TaskExecutor {
		public:
			TaskExecutor(const BehaviourLibrary& library, IActionSink& sink, ExecutorConfig config = {});
			~TaskExecutor();

			// Step 1 of spec 03 §8.1: where a task comes from when the stack is empty.
			void SetTaskSource(std::function<std::unique_ptr<Task>()> source) { source_ = std::move(source); }
			void SetGeoReference(const Environment::GeoReference* geo) { geo_ = geo; }
			void SetPoolState(std::function<std::optional<std::string>(const std::string&)> f) { pool_state_ = std::move(f); }
			void SetProfile(const CapabilityProfile* profile) { profile_ = profile; }

			// Starts a task when the stack is empty, or preempts the top task (spec 03 §8.6).
			void Push(std::unique_ptr<Task> task);

			TickResult Tick(Environment::Worldview& w, double t);

			bool Empty() const;
			std::size_t Depth() const;
			// The task (or behaviour) at the top of the stack, or nullptr.
			const Task* Top() const;
			std::size_t TopIterator() const;

		private:
			struct Frame;
			struct Run;

			const BehaviourLibrary& library_;
			IActionSink& sink_;
			ExecutorConfig config_;
			std::function<std::unique_ptr<Task>()> source_;
			const Environment::GeoReference* geo_ = nullptr;
			std::function<std::optional<std::string>(const std::string&)> pool_state_;
			const CapabilityProfile* profile_ = nullptr;
			std::vector<std::unique_ptr<Frame>> stack_;
		};
	}
}
