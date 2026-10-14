#pragma once
// The robot controller (spec 08 §3): the agent loop of one robot, its events (§4), safety
// supervisor (§5), resources (§6), journal (§7) and, in WP4, a local task list.
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#include "mrs/device/Robot.h"
#include "mrs/device/uav/FlightControlUnit.h"
#include "mrs/robot/Resources.h"
#include "mrs/robot/SafetySupervisor.h"
#include "mrs/robot/TaskJournal.h"
#include "mrs/task/BehaviourLibrary.h"
#include "mrs/task/TaskExecutor.h"
#include "mrs/task/TaskFactory.h"
#include "mrs/world/UavProcessors.h"

namespace MRS {
	namespace Robot {
		enum class ControllerEvent { FAULT, BATTERY_CRITICAL, BATTERY_LOW, GEOFENCE };
		const char* ControllerEventName(ControllerEvent e);

		struct RaisedEvent {
			ControllerEvent event;
			double t;
			std::string detail;
		};

		struct LoggedTaskEvent {
			double t;
			Task::TaskEvent event;
		};

		struct ControllerConfig {
			std::string robot = "r1";       // name in the journal
			std::int64_t robot_id = 1;      // id in the mission header
			Environment::UavWorldviewConfig world;
			SafetyConfig safety;            // speed and climb limits come from the fcu when there is one
			ResourceConfig resources;       // cruise speed and climb rate likewise
			Task::ExecutorConfig executor;
		};

		class RobotController {
		public:
			// The library is copied, so priorities can change per robot (spec 08 §8).
			RobotController(Device::Robot& robot, Task::BehaviourLibrary library, const Task::FunctionRegistry& registry,
			                ControllerConfig config = {});
			virtual ~RobotController();
			RobotController(const RobotController&) = delete;
			RobotController& operator=(const RobotController&) = delete;

			// The mission header (H_M, spec 06 §6): geo reference, home, layer and geofence.
			void SetMission(const Protocol::Record& header, double t = 0.0);
			void SetJournal(std::ostream* out) { journal_.SetStream(out); }
			bool HasMission() const { return header_.has_value(); }
			const std::string& MissionId() const { return mission_; }

			// Adds every top-level task of a .mrst text to the end of the list. Throws TaskLoadError.
			std::size_t AddTasks(const std::string& text);
			void AddTask(std::unique_ptr<Task::Task> task);

			// Restores the unfinished tasks of a journal of the current mission (spec 08 §7) and
			// lifts off at the next tick. False, with the reason in `why`, for another mission or
			// an unreadable journal; nothing is restored then.
			bool Resume(const std::string& journal_text, std::string* why = nullptr);

			// Takes a list task back (spec 09 §5): from the list, or off the executor's stack.
			// Its state goes back to IDLE; no task event is reported. False when it is not there,
			// or while a safety behaviour (return, landing) runs above it.
			bool CancelTask(const std::string& id, double t);
			// The list task on the executor's stack, or empty.
			const std::string& CurrentTask() const { return current_id_; }
			// No task waiting or running, and new tasks are not blocked.
			bool Idle() const { return pending_.empty() && executor_.Empty() && !blocked_ && !stopped_; }
			// Called for every task event of a list task (not behaviours), after the journal.
			void SetTaskListener(std::function<void(const Task::TaskEvent&, double)> f) { listener_ = std::move(f); }
			// Called for every controller event (FAULT, BATTERY_*, GEOFENCE) when it is raised.
			void SetEventListener(std::function<void(const RaisedEvent&)> f) { event_listener_ = std::move(f); }
			// Views from outside the robot's sensors (peer states, spec 09 §2.3), used at the next tick.
			void InjectViews(std::vector<Environment::View> views);
			// Whether to journal J_E end when the list runs empty (off when an allocator feeds it).
			void SetJournalEnd(bool on) { journal_end_ = on; }
			// The capability profile tasks are checked against.
			const Task::CapabilityProfile& Profile() const { return profile_; }

			// One tick of spec 08 §3.1 at time t; the platform owns the outer loop.
			void Tick(double t);
			// Stops task execution; the actuators keep their control loop.
			void Stop(double t);

			// The robot is down for a battery swap (J_E swap_land written).
			bool SwapLanded() const { return swap_landed_; }
			bool Stopped() const { return stopped_; }
			// Every task of the list has ended.
			bool ListDone() const { return list_done_; }
			bool TasksBlocked() const { return blocked_; }
			bool Returning() const { return returning_; }

			// The last state of a task of the list, by id.
			std::optional<Task::TaskState> StateOf(const std::string& id) const;
			std::optional<Task::FailReason> FailReasonOf(const std::string& id) const;
			const std::vector<RaisedEvent>& Events() const { return events_; }
			const std::vector<LoggedTaskEvent>& TaskLog() const { return task_log_; }

			Environment::Worldview& World() { return model_.world; }
			Environment::WorldModel& Model() { return model_; }
			Task::TaskExecutor& Executor() { return executor_; }
			Task::BehaviourLibrary& Library() { return library_; }
			SafetySupervisor& Safety() { return safety_; }
			ResourceManager& Resources() { return resources_; }
			Device::Uav::FlightControlUnit* Fcu() { return fcu_; }
			Device::Robot& DeviceSide() { return robot_; }

		protected:
			// Life cycle hooks (spec 08 §3), in the order they run. OnInit and OnStart run once,
			// before the first tick's work.
			virtual void OnInit() {}
			virtual void OnStart() {}
			virtual void OnPreUpdate(double t) { (void)t; }
			virtual void OnAct(double t) { (void)t; }
			virtual void OnPostUpdate(double t) { (void)t; }
			virtual void OnStop() {}

		private:
			std::unique_ptr<Task::Task> NextTask();
			bool Admissible(const Task::Task& task) const;
			bool FlightCritical() const;
			bool Airborne() const;
			void IdleAtLayer(double t);
			bool Landing() const;
			bool SafetyAbove(const std::string& id) const;
			void HandleEvents(double t);
			void HandleTaskEvents(const std::vector<Task::TaskEvent>& events, double t);
			void Raise(ControllerEvent e, double t, std::string detail = {});
			void PushSafety(const std::string& name);
			void PushReturn();
			void Journal(const std::string& id, const std::string& pool, Task::TaskState state, double t);
			void JournalStack(double t);
			void Start(double t);

			Device::Robot& robot_;
			Task::BehaviourLibrary library_;
			Task::TaskFactory factory_;
			ControllerConfig c_;
			Environment::WorldModel model_;
			Device::Uav::FlightControlUnit* fcu_ = nullptr;
			SafetySupervisor safety_;
			ResourceManager resources_;
			Task::CapabilityProfile profile_;
			Task::TaskExecutor executor_;
			JournalWriter journal_;

			std::optional<Protocol::Record> header_;
			std::string mission_;
			double now_ = 0.0;
			bool started_ = false, stopped_ = false;

			// The task list.
			std::deque<std::unique_ptr<Task::Task>> pending_;
			std::map<std::string, Protocol::Record> records_;  // every task of the list, by id
			std::map<std::string, std::pair<Task::TaskState, Task::FailReason>> states_;
			std::vector<std::string> order_;                   // ids in the order they were added
			const Task::Task* current_ = nullptr;              // the list task on the stack
			std::string current_id_;
			std::vector<std::string> last_stack_;
			bool any_task_ = false, list_done_ = false, resume_pending_ = false;

			// Events.
			bool blocked_ = false, swap_pending_ = false, returning_ = false, swap_landed_ = false;
			bool crit_prev_ = false, low_prev_ = false, fence_prev_ = true, failsafe_prev_ = false;
			std::vector<RaisedEvent> events_;
			std::vector<LoggedTaskEvent> task_log_;
			std::function<void(const Task::TaskEvent&, double)> listener_;
			std::function<void(const RaisedEvent&)> event_listener_;
			std::vector<Environment::View> injected_;
			bool journal_end_ = true;
			std::optional<double> idle_yaw_;    // yaw held while idle at the layer
			std::optional<double> idle_since_;  // when the robot last ran out of tasks in the air
		};
	}
}
