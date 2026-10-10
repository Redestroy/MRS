#include "mrs/task/TaskExecutor.h"

#include <algorithm>
#include <set>

namespace MRS {
	namespace Task {
		struct TaskExecutor::Frame {
			enum class Role { Root, Fulfil, Child };

			std::unique_ptr<Task> owned;  // roots and behaviours
			Task* task = nullptr;         // children point into their parent's task
			Role role = Role::Root;
			std::string label;            // id, or the behaviour's library name
			int depth = 0;                // fulfil behaviours in the chain below and including this frame

			std::optional<double> context_start;  // first entry into STARTED or IN_PROGRESS
			std::optional<double> unknown_since;  // start condition UNKNOWN since
			int fulfil_attempts = 0;              // behaviours that ended SUCCEEDED with the start still FALSE
			std::set<std::string> tried;          // behaviours that FAILED for the current start condition
			FailReason behaviour_failure = FailReason::NONE;

			std::size_t iterator = 0;             // leaf tasks and behaviour bases
			std::optional<double> wait_start;     // A_W
			std::optional<double> sync_start;     // A_SY: when the peers were asked

			// Behaviours.
			std::shared_ptr<const Condition> until;
			double until_context = 0.0;
			std::vector<std::pair<std::string, std::optional<Environment::WorldFieldEntry>>> saved_targets;

			// Complex tasks.
			std::vector<std::size_t> order;  // child indices in the order they run
			std::size_t next = 0;            // position in order
			long long successes = 0;
			bool children_done = false;
			FailReason child_failure = FailReason::NONE;
		};

		// One tick's work: the worldview, the time and the result being filled.
		struct TaskExecutor::Run {
			TaskExecutor& ex;
			Environment::Worldview& w;
			double t;
			TickResult result;
			int steps = 0;

			EvalContext Context(double context_start) const {
				EvalContext c{w, t, context_start, ex.geo_, ex.pool_state_};
				return c;
			}
			EvalContext Context(const Frame& f) const { return Context(f.context_start.value_or(t)); }

			void Emit(const Frame& f, TaskState s, FailReason r = FailReason::NONE) {
				f.task->SetState(s);
				result.events.push_back({f.label, s, r, f.role == Frame::Role::Fulfil});
			}

			void Enter(Frame& f, TaskState s) {
				if (!f.context_start) f.context_start = t;
				Emit(f, s);
			}

			Frame& Top() { return *ex.stack_.back(); }

			// Ends the top frame and tells the frame below.
			void Finish(TaskState s, FailReason reason) {
				std::unique_ptr<Frame> done = std::move(ex.stack_.back());
				ex.stack_.pop_back();
				Emit(*done, s, reason);
				if (done->role == Frame::Role::Fulfil) {
					// Restore the target fields of the behaviour below (spec 05 §4.3).
					for (const auto& p : w.PathsWithPrefix("target.")) w.Erase(p);
					for (const auto& [path, entry] : done->saved_targets) w.Restore(path, entry);
				}
				if (ex.stack_.empty()) return;
				Frame& below = Top();
				switch (done->role) {
				case Frame::Role::Root:
					if (below.task->State() == TaskState::IDLE) Emit(below, TaskState::QUEUED);  // resume (spec 03 §8.6)
					break;
				case Frame::Role::Fulfil:
					// The start condition is evaluated again on the next tick (spec 03 §8.4).
					// SUCCEEDED counts as a fulfil attempt. FAILED marks the behaviour as tried, so
					// the next lookup takes the next entry for the condition; the task fails only
					// when every entry has failed.
					if (s == TaskState::SUCCEEDED) {
						++below.fulfil_attempts;
						below.tried.clear();
					} else {
						below.tried.insert(done->label);
						below.behaviour_failure = reason;
					}
					break;
				case Frame::Role::Child: ChildEnded(below, s, reason); break;
				}
			}

			void ChildEnded(Frame& parent, TaskState s, FailReason reason) {
				auto& c = static_cast<ComplexTask&>(*parent.task);
				const long long total = static_cast<long long>(parent.order.size());
				++parent.next;
				const long long remaining = total - static_cast<long long>(parent.next);
				if (s == TaskState::SUCCEEDED) ++parent.successes;
				else parent.child_failure = reason;
				switch (c.Type()) {
				case TaskType::SEQUENTIAL:
					if (s == TaskState::FAILED) return Finish(TaskState::FAILED, reason);
					parent.children_done = remaining == 0;
					break;
				case TaskType::PARALLEL: {
					const long long needed = c.RequiredSuccesses() == 0 ? static_cast<long long>(c.Children().size()) : c.RequiredSuccesses();
					if (parent.successes >= needed) parent.children_done = true;
					else if (parent.successes + remaining < needed) return Finish(TaskState::FAILED, reason);
					break;
				}
				default:  // CHOICE
					if (s == TaskState::SUCCEEDED) parent.children_done = true;
					else if (remaining == 0) return Finish(TaskState::FAILED, reason);
					break;
				}
			}

			// Spec 03 §8.3: the end condition decides.
			void EndWithCondition(Frame& f) {
				switch (f.task->EndCondition().Evaluate(Context(f))) {
				case Truth::True: return Finish(TaskState::SUCCEEDED, FailReason::NONE);
				case Truth::False: return Finish(TaskState::FAILED, FailReason::END_NOT_MET);
				case Truth::Unknown: return Finish(TaskState::FAILED, FailReason::MISSING_FIELD);
				}
			}

			// Spec 03 §8.5. Returns false when a task failed this tick.
			bool CheckRuntimeConditions() {
				for (std::size_t i = 0; i < ex.stack_.size(); ++i) {
					Frame& f = *ex.stack_[i];
					if (!f.context_start) continue;
					for (const auto& c : f.task->RuntimeConditions()) {
						if (c->Evaluate(Context(f)) == Truth::True) continue;
						while (ex.stack_.size() > i + 1) Finish(TaskState::FAILED, FailReason::RUNTIME_CONDITION);
						if (ex.stack_.size() == i + 1) Finish(TaskState::FAILED, FailReason::RUNTIME_CONDITION);
						return false;
					}
				}
				return true;
			}

			void Work() {
				if (ex.stack_.empty() || ++steps > 64) return;
				Frame& f = Top();
				const TaskState s = f.task->State();
				if (s == TaskState::QUEUED || s == TaskState::STARTED) {
					switch (f.task->StartCondition().Evaluate(Context(f))) {
					case Truth::True:
						f.unknown_since.reset();
						Enter(f, TaskState::IN_PROGRESS);
						return Progress(f);
					case Truth::False:
						f.unknown_since.reset();
						return Fulfil(f);
					case Truth::Unknown:
						if (!f.unknown_since) f.unknown_since = t;
						// A world field peers may know: ask them first (spec 15 §2.5). The answer, not
						// the asking, decides; the wait below still bounds it.
						if (f.depth + 1 <= ex.config_.max_behaviour_depth) {
							if (const BehaviourEntry* entry = ex.library_.Find(f.task->StartCondition(), ex.profile_, f.tried, true))
								return PushBehaviour(f, *entry);
						}
						if (t - *f.unknown_since >= ex.config_.start_unknown_timeout) Finish(TaskState::FAILED, FailReason::MISSING_FIELD);
						return;
					}
				}
				if (s == TaskState::IN_PROGRESS) Progress(f);
			}

			// Spec 03 §8.1 step 3 (FALSE) and §8.4.
			void Fulfil(Frame& f) {
				if (f.fulfil_attempts >= ex.config_.max_fulfil_attempts) return Finish(TaskState::FAILED, FailReason::START_UNREACHABLE);
				if (f.depth + 1 > ex.config_.max_behaviour_depth) return Finish(TaskState::FAILED, FailReason::NO_BEHAVIOUR);
				const Condition& unmet = f.task->StartCondition();
				const BehaviourEntry* entry = ex.library_.Find(unmet, ex.profile_, f.tried);
				if (!entry) {
					// Every entry for the condition failed: fail with the last behaviour's reason.
					if (!f.tried.empty()) return Finish(TaskState::FAILED, f.behaviour_failure);
					return Finish(TaskState::FAILED, FailReason::NO_BEHAVIOUR);
				}
				PushBehaviour(f, *entry);
			}

			// Binds a library behaviour for the frame's start condition and pushes it (spec 03 §7).
			void PushBehaviour(Frame& f, const BehaviourEntry& entry) {
				const Condition& unmet = f.task->StartCondition();
				if (f.task->State() != TaskState::STARTED) Enter(f, TaskState::STARTED);

				auto b = std::make_unique<Frame>();
				b->owned = entry.behaviour->Clone();
				b->task = b->owned.get();
				b->role = Frame::Role::Fulfil;
				b->label = entry.name;
				b->depth = f.depth + 1;
				b->until = f.task->StartConditionPtr();  // the unmet condition replaces the placeholder
				b->until_context = *f.context_start;
				for (const auto& p : w.PathsWithPrefix("target.")) b->saved_targets.emplace_back(p, w.Raw(p));
				unmet.Bind(w, t, ex.geo_);
				ex.stack_.push_back(std::move(b));
				Emit(Top(), TaskState::QUEUED);
				Work();  // step 2 for the pushed behaviour, in the same tick
			}

			void Progress(Frame& f) {
				switch (f.task->Type()) {
				case TaskType::ATOMIC:
				case TaskType::PARAMETRIC:
					if (RunActions(f, static_cast<const ATask&>(*f.task).Actions())) EndWithCondition(f);
					return;
				case TaskType::BEHAVIOUR: {
					const EvalContext until_ctx = Context(f.until ? f.until_context : f.context_start.value_or(t));
					const Condition& until = f.until ? *f.until : static_cast<const Behaviour&>(*f.task).Until();
					if (until.Evaluate(until_ctx) == Truth::True) return EndWithCondition(f);
					// A_N does not take a tick in a behaviour: the base starts again at once (JB, 2026-10-06).
					const auto& base = static_cast<const Behaviour&>(*f.task).Base().Actions();
					if (RunActions(f, base)) {
						f.iterator = 0;
						RunActions(f, base);  // at most once more, so a base of only A_N cannot loop
					}
					return;
				}
				default: return ProgressComplex(f);
				}
			}

			void ProgressComplex(Frame& f) {
				auto& c = static_cast<ComplexTask&>(*f.task);
				if (f.order.empty() && !c.Children().empty() && !f.children_done && f.next == 0) {
					for (std::size_t i = 0; i < c.Children().size(); ++i) f.order.push_back(i);
					if (c.Type() == TaskType::CHOICE) {
						std::stable_sort(f.order.begin(), f.order.end(), [&](std::size_t a, std::size_t b) {
							return c.Children()[a]->EffectivePriority() > c.Children()[b]->EffectivePriority();
						});
						if (ex.profile_) {
							f.order.erase(std::remove_if(f.order.begin(), f.order.end(),
							                             [&](std::size_t i) { return !c.Children()[i]->MeetsStaticRequirements(*ex.profile_); }),
							              f.order.end());
							if (f.order.empty()) return Finish(TaskState::FAILED, FailReason::NO_BEHAVIOUR);
						}
					}
				}
				if (f.children_done || f.next >= f.order.size()) {
					// Cancel the rest (T_L, T_O) and check the parent's end condition (spec 03 §6.1).
					return EndWithCondition(f);
				}
				auto child = std::make_unique<Frame>();
				child->task = c.Children()[f.order[f.next]].get();
				child->role = Frame::Role::Child;
				child->label = child->task->Id();
				child->depth = f.depth;
				ex.stack_.push_back(std::move(child));
				Emit(Top(), TaskState::QUEUED);
				Work();
			}

			// `A_SY timeout` (spec 15 §2.5): ask the live peers for the world fields of the condition the
			// frame works towards, then wait for the answer. The condition is never assumed: it must
			// become TRUE from what arrives (the behaviour's until ends it), else NOT_KNOWN.
			bool Sync(Frame& f, const TaskAction& a) {
				if (!f.sync_start) {
					const Condition& goal = f.until                                  ? *f.until
					                        : f.task->Type() == TaskType::BEHAVIOUR ? static_cast<const Behaviour&>(*f.task).Until()
					                                                                 : f.task->EndCondition();
					const auto topics = SharedTopics(goal);
					const int asked = topics.empty() || !ex.sync_ ? 0 : ex.sync_(topics, t);
					if (asked <= 0) {
						Finish(TaskState::FAILED, FailReason::NOT_KNOWN);
						return false;
					}
					f.sync_start = t;
				}
				if (t - *f.sync_start >= a.WaitSeconds()) {
					Finish(TaskState::FAILED, FailReason::NOT_KNOWN);
					return false;
				}
				return false;
			}

			// Runs the action at the iterator (spec 03 §8.2). Returns true when the iterator is
			// at A_N or past the last action; the caller decides what that means.
			bool RunActions(Frame& f, const std::vector<TaskAction>& actions) {
				if (f.iterator >= actions.size()) return true;
				const TaskAction& a = actions[f.iterator];
				if (a.Code() == "A_N") return true;
				if (a.Code() == "A_I") {
					Finish(TaskState::FAILED, FailReason::IMPOSSIBLE);
					return false;
				}
				if (a.Code() == "A_SY") return Sync(f, a);
				if (a.Code() == "A_W") {
					if (!f.wait_start) f.wait_start = t;
					if (t - *f.wait_start >= a.WaitSeconds()) {  // >=, not > (fixes the 2021 inversion)
						f.wait_start.reset();
						++f.iterator;
					}
					return false;
				}
				auto command = a.Resolve(w, t);
				if (!command) {
					Finish(TaskState::FAILED, FailReason::MISSING_FIELD);
					return false;
				}
				result.dispatched = *command;
				switch (ex.sink_.Dispatch(*command, t)) {
				case Device::ActionStatus::DONE: ++f.iterator; break;
				case Device::ActionStatus::RUNNING: break;
				case Device::ActionStatus::REJECTED: Finish(TaskState::FAILED, FailReason::NO_ACTUATOR); break;
				case Device::ActionStatus::FAILED: Finish(TaskState::FAILED, FailReason::ACTUATOR_FAILED); break;
				}
				return false;
			}
		};

		TaskExecutor::TaskExecutor(const BehaviourLibrary& library, IActionSink& sink, ExecutorConfig config)
		    : library_(library), sink_(sink), config_(config) {}

		TaskExecutor::~TaskExecutor() = default;

		void TaskExecutor::Push(std::unique_ptr<Task> task) {
			if (!stack_.empty()) {
				Frame& top = *stack_.back();
				top.task->SetState(TaskState::IDLE);  // preempted; its iterator and contexts are kept
			}
			auto f = std::make_unique<Frame>();
			f->owned = std::move(task);
			f->task = f->owned.get();
			f->label = f->task->Id();
			f->task->SetState(TaskState::QUEUED);
			stack_.push_back(std::move(f));
		}

		bool TaskExecutor::Withdraw(const std::string& label, Environment::Worldview& w) {
			auto it = std::find_if(stack_.begin(), stack_.end(), [&](const auto& f) { return f->label == label; });
			if (it == stack_.end()) return false;
			stack_.erase(it, stack_.end());
			for (const auto& p : w.PathsWithPrefix("target.")) w.Erase(p);
			if (!stack_.empty()) {
				Frame& top = *stack_.back();
				if (top.task->State() == TaskState::IDLE) top.task->SetState(TaskState::QUEUED);
			}
			return true;
		}

		TickResult TaskExecutor::Tick(Environment::Worldview& w, double t) {
			Run run{*this, w, t, {}};
			if (stack_.empty() && source_) {
				if (auto task = source_()) {
					Push(std::move(task));
					run.result.events.push_back({stack_.back()->label, TaskState::QUEUED, FailReason::NONE, false});
				}
			}
			if (stack_.empty()) return run.result;
			if (run.CheckRuntimeConditions()) run.Work();
			if (!stack_.empty()) {
				const Frame& top = *stack_.back();
				w.SetId("task.active", top.label, t);
				if (top.context_start) w.SetScalar("task.elapsed", t - *top.context_start, t);
			}
			return run.result;
		}

		bool TaskExecutor::Empty() const { return stack_.empty(); }
		std::size_t TaskExecutor::Depth() const { return stack_.size(); }
		const Task* TaskExecutor::Top() const { return stack_.empty() ? nullptr : stack_.back()->task; }
		std::size_t TaskExecutor::TopIterator() const { return stack_.empty() ? 0 : stack_.back()->iterator; }

		std::vector<std::string> TaskExecutor::StackLabels(bool with_behaviours) const {
			std::vector<std::string> out;
			for (const auto& f : stack_)
				if (with_behaviours || f->role != Frame::Role::Fulfil) out.push_back(f->label);
			return out;
		}

		std::size_t TaskExecutor::IteratorOf(const std::string& label) const {
			for (const auto& f : stack_)
				if (f->label == label) return f->iterator;
			return 0;
		}
	}
}
