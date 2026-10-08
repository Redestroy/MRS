#include "mrs/robot/RobotController.h"

#include <algorithm>

namespace MRS {
	namespace Robot {
		const char* ControllerEventName(ControllerEvent e) {
			switch (e) {
			case ControllerEvent::FAULT: return "FAULT";
			case ControllerEvent::BATTERY_CRITICAL: return "BATTERY_CRITICAL";
			case ControllerEvent::BATTERY_LOW: return "BATTERY_LOW";
			case ControllerEvent::GEOFENCE: return "GEOFENCE";
			}
			return "?";
		}

		namespace {
			const char* kSafetyPrefix = "safety.";
			const char* kFlightCritical[] = {"m_fl", "m_fr", "m_rl", "m_rr", "fcu"};

			Device::Uav::FlightControlUnit* FindFcu(Device::Robot& robot) {
				auto* node = dynamic_cast<Device::Uav::FlightControlUnit*>(robot.tree.Find("fcu"));
				return node && node->Available() ? node : nullptr;
			}

			SafetyConfig SafetyFor(const Device::Uav::FlightControlUnit* fcu, SafetyConfig c) {
				if (fcu) {
					c.max_speed_xy = fcu->Gains().max_speed_xy;
					c.max_climb = fcu->Gains().max_climb;
				}
				return c;
			}

			ResourceConfig ResourcesFor(const Device::Uav::FlightControlUnit* fcu, ResourceConfig c) {
				if (fcu) {
					c.cruise_speed = 0.75 * fcu->Gains().max_speed_xy;
					c.climb_rate = fcu->Gains().max_climb;
				}
				return c;
			}

			bool IsSafety(const std::string& label) { return label.rfind(kSafetyPrefix, 0) == 0; }
		}

		RobotController::RobotController(Device::Robot& robot, Task::BehaviourLibrary library, const Task::FunctionRegistry& registry,
		                                 ControllerConfig config)
		    : robot_(robot),
		      library_(std::move(library)),
		      factory_(registry),
		      c_(std::move(config)),
		      model_(Environment::WorldModel::ForViews(robot.self.views, c_.world)),
		      fcu_(FindFcu(robot)),
		      safety_(robot.blocks.actuators, model_.world, SafetyFor(fcu_, c_.safety)),
		      resources_(ResourcesFor(fcu_, c_.resources)),
		      profile_(robot.self.ToProfile()),
		      executor_(library_, safety_, c_.executor) {
			resources_.SetCapacity(robot.self.Capacity("energy_wh"));
			executor_.SetProfile(&profile_);
			executor_.SetTaskSource([this]() { return NextTask(); });
			if (fcu_) {
				fcu_->SetStateSource([this](double t) -> std::optional<Device::Uav::FlightState> {
					const auto& w = model_.world;
					Device::Uav::FlightState s;
					const std::pair<const char*, double*> fields[] = {
					    {"pose.enu.x", &s.x},  {"pose.enu.y", &s.y},  {"pose.enu.z", &s.z},  {"vel.enu.x", &s.vx},
					    {"vel.enu.y", &s.vy},  {"vel.enu.z", &s.vz},  {"att.roll", &s.roll}, {"att.pitch", &s.pitch},
					    {"att.yaw", &s.yaw},   {"rate.body.x", &s.p}, {"rate.body.y", &s.q}, {"rate.body.z", &s.r},
					    {"alt.agl", &s.agl}};
					for (const auto& [path, out] : fields) {
						const auto v = w.Scalar(path, t);
						if (!v) return std::nullopt;
						*out = *v;
					}
					return s;
				});
			}
		}

		RobotController::~RobotController() {
			if (fcu_) fcu_->SetStateSource(nullptr);
		}

		void RobotController::SetMission(const Protocol::Record& header, double t) {
			header_ = header;
			mission_ = header.fields.empty() ? std::string() : header.fields[0].s;
			Environment::ApplyMissionHeader(header, c_.robot_id, model_.world, t);
			executor_.SetGeoReference(model_.world.Geo());
			if (started_) journal_.Header(t, c_.robot, header);
		}

		bool RobotController::SafetyAbove(const std::string& id) const {
			// A safety behaviour pushed over the task stays: a return or landing is not cut short.
			const auto labels = executor_.StackLabels(true);
			auto it = std::find(labels.begin(), labels.end(), id);
			return it != labels.end() && std::any_of(it + 1, labels.end(), [](const std::string& l) { return IsSafety(l); });
		}

		void RobotController::InjectViews(std::vector<Environment::View> views) {
			for (auto& v : views) injected_.push_back(std::move(v));
		}

		bool RobotController::CancelTask(const std::string& id, double t) {
			auto it = std::find_if(pending_.begin(), pending_.end(), [&](const auto& p) { return p->Id() == id; });
			bool found = false;
			if (it != pending_.end()) {
				pending_.erase(it);
				found = true;
			} else if (current_id_ == id && !SafetyAbove(id) && executor_.Withdraw(id, model_.world)) {
				current_ = nullptr;
				current_id_.clear();
				found = true;
			}
			if (!found) return false;
			states_[id] = {Task::TaskState::IDLE, Task::FailReason::NONE};
			Journal(id, "AVAILABLE", Task::TaskState::IDLE, t);
			JournalStack(t);
			return true;
		}

		std::size_t RobotController::AddTasks(const std::string& text) {
			auto tasks = factory_.BuildTasks(text);
			const std::size_t n = tasks.size();
			for (auto& task : tasks) AddTask(std::move(task));
			return n;
		}

		void RobotController::AddTask(std::unique_ptr<Task::Task> task) {
			const std::string id = task->Id();
			records_[id] = task->ToRecord();
			states_[id] = {Task::TaskState::IDLE, Task::FailReason::NONE};
			if (std::find(order_.begin(), order_.end(), id) == order_.end()) order_.push_back(id);
			any_task_ = true;
			list_done_ = false;
			if (started_) Journal(id, "AVAILABLE", Task::TaskState::IDLE, now_);
			pending_.push_back(std::move(task));
		}

		bool RobotController::Resume(const std::string& journal_text, std::string* why) {
			const JournalState j = ReadJournal(journal_text);
			auto fail = [&](std::string reason) {
				if (why) *why = std::move(reason);
				return false;
			};
			if (!j.ok) return fail("unreadable journal: " + j.error);
			if (j.mission != mission_) return fail("journal of mission " + j.mission + ", current mission " + mission_);

			auto unfinished = j.Unfinished();
			// The task at the top of the stack goes first.
			for (auto it = j.stack.rbegin(); it != j.stack.rend(); ++it) {
				auto top = std::find_if(unfinished.begin(), unfinished.end(), [&](const JournalTask* t) { return t->id == *it; });
				if (top != unfinished.end()) {
					std::rotate(unfinished.begin(), top, top + 1);
					break;
				}
			}
			std::vector<std::unique_ptr<Task::Task>> restored;
			for (const JournalTask* jt : unfinished) {
				Protocol::Record r = jt->task;
				SetRecordState(r, Task::TaskState::IDLE);
				auto task = factory_.BuildTask(r);
				task->SetState(Task::TaskState::IDLE);
				restored.push_back(std::move(task));
			}
			// Restored tasks replace tasks of the same id already in the list, and go first.
			for (auto it = restored.rbegin(); it != restored.rend(); ++it) {
				const std::string id = (*it)->Id();
				pending_.erase(std::remove_if(pending_.begin(), pending_.end(), [&](const auto& p) { return p->Id() == id; }), pending_.end());
				records_[id] = (*it)->ToRecord();
				states_[id] = {Task::TaskState::IDLE, Task::FailReason::NONE};
				if (std::find(order_.begin(), order_.end(), id) == order_.end()) order_.push_back(id);
				pending_.push_front(std::move(*it));
			}
			any_task_ = any_task_ || !restored.empty();
			list_done_ = false;
			resume_pending_ = true;
			return true;
		}

		std::optional<Task::TaskState> RobotController::StateOf(const std::string& id) const {
			auto it = states_.find(id);
			if (it == states_.end()) return std::nullopt;
			return it->second.first;
		}

		std::optional<Task::FailReason> RobotController::FailReasonOf(const std::string& id) const {
			auto it = states_.find(id);
			if (it == states_.end()) return std::nullopt;
			return it->second.second;
		}

		// --- tick ------------------------------------------------------------------------------

		void RobotController::Start(double t) {
			started_ = true;
			OnInit();
			OnStart();
			if (header_) journal_.Header(t, c_.robot, *header_);
			journal_.Event(t, resume_pending_ ? "restart" : "start");
			for (const auto& task : pending_) Journal(task->Id(), "AVAILABLE", Task::TaskState::IDLE, t);
			if (!robot_.faults.empty()) {
				std::string nodes;
				for (const auto& f : robot_.faults) nodes += (nodes.empty() ? "" : " ") + f.node;
				Raise(ControllerEvent::FAULT, t, nodes);
			}
			// A robot that cannot fly takes no tasks.
			if (FlightCritical()) blocked_ = true;
			if (resume_pending_ && !blocked_) PushSafety("liftoff");
			resume_pending_ = false;
		}

		void RobotController::Tick(double t) {
			now_ = t;
			if (!started_) Start(t);
			OnPreUpdate(t);

			// 1. The fcu's armed state, then the views (spec 08 §3.1 steps 1-2).
			if (fcu_) model_.world.SetBool("armed", fcu_->Armed(), t, "fcu");
			auto views = robot_.blocks.sensors.Sample(t);
			for (auto& v : injected_) views.push_back(std::move(v));
			injected_.clear();
			model_.Update(views, t);

			if (!stopped_) {
				// 3. Events.
				HandleEvents(t);
				// 4. The executor; its dispatch goes through the safety supervisor.
				if (!stopped_) {
					auto result = executor_.Tick(model_.world, t);
					HandleTaskEvents(result.events, t);
					JournalStack(t);
					// A swap waits for the current task to end (spec 08 §4 BATTERY_LOW).
					if (swap_pending_ && !returning_ && !swap_landed_ && executor_.Empty()) {
						if (Airborne()) {
							PushReturn();
						} else {
							swap_landed_ = true;
							journal_.Event(t, "swap_land");
							Stop(t);
						}
					}
					// An idle robot in the air waits at its own layer (spec 08 §3.3).
					// The second it waits first lets the next task of a chain start where the last one ended.
					if (fcu_ && !blocked_ && !returning_ && pending_.empty() && executor_.Empty() && Airborne()) {
						if (!idle_since_) idle_since_ = t;
						if (t - *idle_since_ >= 1.0) IdleAtLayer(t);
					} else {
						idle_since_.reset();
						idle_yaw_.reset();
					}
					if (!list_done_ && any_task_ && !blocked_ && pending_.empty() && executor_.Empty()) {
						list_done_ = true;
						if (journal_end_) journal_.Event(t, "end");
					}
				}
			}
			OnAct(t);
			// 5. Control step of the actuators (the fcu cascade).
			robot_.blocks.actuators.Update(t);
			// 6. Resources.
			resources_.Update(model_.world, t);
			OnPostUpdate(t);
		}

		void RobotController::Stop(double t) {
			(void)t;
			if (stopped_) return;
			stopped_ = true;
			OnStop();
		}

		// --- task list -------------------------------------------------------------------------

		std::unique_ptr<Task::Task> RobotController::NextTask() {
			while (!blocked_ && !pending_.empty()) {
				auto task = std::move(pending_.front());
				pending_.pop_front();
				if (!Admissible(*task)) {
					const std::string id = task->Id();
					states_[id] = {Task::TaskState::FAILED, Task::FailReason::IMPOSSIBLE};
					task_log_.push_back({now_, {id, Task::TaskState::FAILED, Task::FailReason::IMPOSSIBLE, false}});
					Journal(id, "FAILED", Task::TaskState::FAILED, now_);
					if (listener_) listener_(task_log_.back().event, now_);
					continue;
				}
				current_ = task.get();
				current_id_ = task->Id();
				return task;
			}
			return nullptr;
		}

		bool RobotController::Admissible(const Task::Task& task) const {
			if (!task.MeetsStaticRequirements(profile_)) return false;
			for (const auto& target : ResourceManager::Targets(task))
				if (!safety_.TargetAllowed(target)) return false;
			return true;
		}

		bool RobotController::FlightCritical() const {
			for (const auto& f : robot_.faults)
				for (const char* n : kFlightCritical)
					if (f.node == n) return true;
			return fcu_ && !robot_.self.Provides("pose.enu");
		}

		void RobotController::IdleAtLayer(double t) {
			const auto& w = model_.world;
			const auto layer = w.Raw("layer.alt");
			const auto z = w.Scalar("pose.enu.z", t);
			if (!layer || !std::holds_alternative<double>(layer->value) || !z) return;
			if (!idle_yaw_) idle_yaw_ = w.Scalar("att.yaw", t).value_or(0.0);
			// The horizontal setpoint stays where the last task left it (spec 02 §4.2).
			Device::ActionMap a;
			Device::Action pzy;
			pzy.code = "A_PZY";
			pzy.arg = *Device::PackArgument(Device::ArgLayout::F32X2, std::vector<double>{std::get<double>(layer->value), *idle_yaw_});
			a.entries.push_back({"any", pzy});
			safety_.Dispatch(a, t);
		}

		bool RobotController::Airborne() const { return model_.world.Bool("airborne", now_).value_or(false); }

		bool RobotController::Landing() const {
			if (fcu_ && fcu_->Mode() == Device::Uav::FcuMode::LANDING) return true;
			for (const auto& label : executor_.StackLabels(true))
				if (label == "safety.emergency_land" || label == "safety.land" || label == "land" || label == "emergency_land") return true;
			return false;
		}

		// --- events ----------------------------------------------------------------------------

		void RobotController::Raise(ControllerEvent e, double t, std::string detail) {
			events_.push_back({e, t, std::move(detail)});
			if (event_listener_) event_listener_(events_.back());
		}

		void RobotController::PushSafety(const std::string& name) {
			const Task::BehaviourEntry* entry = library_.ByName(name);
			if (!entry) return;
			auto task = entry->behaviour->Clone();
			task->SetId(kSafetyPrefix + name);
			executor_.Push(std::move(task));
		}

		void RobotController::PushReturn() {
			// Pushed in reverse: the robot flies home, then lands.
			PushSafety("land");
			PushSafety("return_home");
			returning_ = true;
		}

		void RobotController::HandleEvents(double t) {
			auto& w = model_.world;
			const bool airborne = Airborne();
			const bool armed = fcu_ && fcu_->Armed();

			// FAULT: the fcu lost its flight state.
			const bool failsafe = fcu_ && fcu_->InFailsafe();
			if (failsafe && !failsafe_prev_) {
				Raise(ControllerEvent::FAULT, t, "fcu failsafe");
				blocked_ = true;
				if (armed && !Landing()) PushSafety("emergency_land");
			}
			failsafe_prev_ = failsafe;

			const bool critical = w.Bool("battery.critical", t).value_or(false);
			if (critical && !crit_prev_) {
				Raise(ControllerEvent::BATTERY_CRITICAL, t);
				blocked_ = true;
				swap_pending_ = true;
				if (!armed) {
					swap_landed_ = true;
					journal_.Event(t, "swap_land");
					Stop(t);
				} else if (!Landing()) {
					PushSafety("emergency_land");  // land where it is, not at home
					returning_ = true;
				}
			}
			crit_prev_ = critical;

			const bool low = w.Bool("battery.low", t).value_or(false);
			if (low && !low_prev_) {
				Raise(ControllerEvent::BATTERY_LOW, t);
				blocked_ = true;
				swap_pending_ = true;
				// Finish the current task only if the energy for it, the way home and the reserve is there.
				if (airborne && !returning_ && current_ && !resources_.Feasible(*current_, w, t)) PushReturn();
			}
			low_prev_ = low;

			const auto inside = w.Bool("geofence.inside", t);
			if (inside) {
				if (!*inside && fence_prev_ && airborne) {
					Raise(ControllerEvent::GEOFENCE, t);
					if (!returning_) PushReturn();
				}
				fence_prev_ = *inside;
			}
		}

		void RobotController::HandleTaskEvents(const std::vector<Task::TaskEvent>& events, double t) {
			for (const auto& e : events) {
				task_log_.push_back({t, e});
				const bool ended = e.state == Task::TaskState::SUCCEEDED || e.state == Task::TaskState::FAILED;
				if (!e.behaviour && IsSafety(e.task_id)) {
					if (!ended) continue;
					const std::string name = e.task_id.substr(std::string(kSafetyPrefix).size());
					if (name == "land" || name == "emergency_land") {
						returning_ = false;
						if (swap_pending_) {
							swap_landed_ = true;
							journal_.Event(t, "swap_land");
							Stop(t);
						} else if (name == "emergency_land") {
							Stop(t);  // a fault: stay down
						}
					}
					continue;
				}
				if (e.behaviour || !records_.count(e.task_id)) continue;  // a child of a complex task
				states_[e.task_id] = {e.state, e.reason};
				const char* pool = e.state == Task::TaskState::SUCCEEDED ? "DONE" : e.state == Task::TaskState::FAILED ? "FAILED" : "ACTIVE";
				Journal(e.task_id, pool, e.state, t);
				if (ended && e.task_id == current_id_) {
					current_ = nullptr;
					current_id_.clear();
				}
				if (listener_) listener_(e, t);
			}
		}

		// --- journal ---------------------------------------------------------------------------

		void RobotController::Journal(const std::string& id, const std::string& pool, Task::TaskState state, double t) {
			auto it = records_.find(id);
			if (it == records_.end()) return;
			journal_.TaskRecord(t, pool, executor_.IteratorOf(id), it->second, state);
		}

		void RobotController::JournalStack(double t) {
			std::vector<std::string> ids;
			for (const auto& label : executor_.StackLabels())
				if (records_.count(label)) ids.push_back(label);
			if (ids == last_stack_) return;
			last_stack_ = ids;
			journal_.Stack(t, ids);
		}
	}
}
