#include "mrs/algorithms/MrsLayer.h"

#include <algorithm>
#include <cmath>

namespace MRS {
	namespace Algorithms {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;
		using Task::FailReason;
		using Task::TaskState;

		namespace {
			std::string RobotName(const Device::Robot& robot) { return "r" + std::to_string(robot.self.id); }

			Field Tid(const std::string& id) { return Field::MakeText(FieldType::TaskId, id); }
			Field Id(const std::string& id) { return Field::MakeText(FieldType::Id, id); }

		}

		MrsLayer::MrsLayer(Robot::RobotController& robot, Comm::ITransport& transport, const Task::FunctionRegistry& registry,
		                   std::unique_ptr<IAllocator> allocator, MrsConfig config, std::unique_ptr<ISizeEstimator> size)
		    : robot_(robot),
		      messenger_(transport, RobotName(robot.DeviceSide())),
		      factory_(registry),
		      allocator_(std::move(allocator)),
		      size_(size ? std::move(size) : std::make_unique<SpatialSizeEstimator>()),
		      c_(config),
		      self_(RobotName(robot.DeviceSide())) {
			robot_.SetJournalEnd(false);
			robot_.SetTaskListener([this](const Task::TaskEvent& e, double t) { robot_events_.push_back({e, t}); });
			robot_.SetEventListener([this](const Robot::RaisedEvent& e) { raised_.push_back(e); });
			if (robot_.HasMission()) messenger_.SetMission(robot_.MissionId());
			AllocatorContext ctx;
			ctx.pool = &pool_;
			ctx.peers = &peers_;
			ctx.self = self_;
			ctx.profile = &robot_.Profile();
			ctx.size = size_.get();
			ctx.resources = &robot_.Resources();
			ctx.peer_timeout = c_.peer_timeout;
			ctx.claim = [this](const std::string& task, double expiry, double score) {
				auto r = messenger_.Begin("M_CLAIM", "all", now_);
				r.fields.push_back(Tid(task));
				r.fields.push_back(Field::MakeNum(expiry));
				r.fields.push_back(Field::MakeNum(score));
				messenger_.Post(r);
				if (auto* e = pool_.Find(task)) e->claims[self_] = {self_, expiry, score};
			};
			ctx.release = [this](const std::string& task) {
				auto r = messenger_.Begin("M_RELEASE", "all", now_);
				r.fields.push_back(Tid(task));
				messenger_.Post(r);
				if (auto* e = pool_.Find(task)) e->claims.erase(self_);
			};
			ctx.runnable = [this](const PoolEntry& e, double t) {
				// A runtime condition that is FALSE now rules the task out before the robot flies to
				// it (spec 11 §2.4): a release point is not for a robot carrying another package.
				Task::EvalContext ec{robot_.World(), t, 0.0, robot_.World().Geo(), {}};
				for (const auto& c : e.task->RuntimeConditions())
					if (c->Evaluate(ec) == Task::Truth::False) return false;
				return true;
			};
			allocator_->Bind(ctx);
		}

		void MrsLayer::Tick(double t) {
			now_ = t;
			for (const auto& m : messenger_.Receive()) Handle(m, t);
			pool_.ExpireClaims(t);
			UpdateTrees(t);
			if (robot_.HasMission()) Allocate(t);
			robot_.Tick(t);
			AfterRobot(t);
			if (robot_.HasMission()) {
				if (t - last_profile_ >= c_.profile_period) SendProfile(t);
				if (t - last_state_ >= c_.state_period) SendState(t);
			}
		}

		// --- incoming ----------------------------------------------------------------------------

		void MrsLayer::Handle(const Comm::Message& m, double t) {
			allocator_->OnMessage(m, t);
			const std::string& code = m.code;
			if (code == "M_MISSION") {
				if (!robot_.HasMission()) {
					const Record& h = m.Child(0);
					robot_.SetMission(h, t);
					messenger_.SetMission(robot_.MissionId());
					if (h.fields.size() > 11) c_.claim_grace = h.fields[11].n;
				}
				return;
			}
			if (code == "M_TASK") return OnTasks(m, t);
			if (code == "M_STATE") {
				auto view = Environment::ViewFromRecord(m.Child(0));
				if (!view) return;
				robot_.InjectViews({*view});
				Peer& p = peers_.Touch(m.sender, t);
				if (view->values.size() >= 8) {
					p.pose = {view->values[1], view->values[2], view->values[3]};
					p.vel = {view->values[4], view->values[5], view->values[6]};
					p.battery = view->values[7];
				}
				p.task = view->text;
				return;
			}
			if (code == "M_PROFILE") {
				Peer& p = peers_.Touch(m.sender, t);
				p.has_profile = true;
				p.robot_type = m.Slot(0).s;
				const auto n = static_cast<std::size_t>(m.Slot(1).i);
				p.roles.clear();
				for (std::size_t k = 0; k < n; ++k) p.roles.insert(m.Slot(2 + k).s);
				p.actions.clear();
				for (const auto& c : m.record.children)
					if (c.code == "K_A" && !c.fields.empty()) p.actions.insert(c.fields[0].s);
				return;
			}
			if (m.SlotCount() == 0) return;
			PoolEntry* e = pool_.Find(m.Slot(0).s);
			if (!e) {
				// A complex node whose end condition failed on the robot that finished it.
				if (code == "M_FAIL")
					if (auto it = tree_of_.find(m.Slot(0).s); it != tree_of_.end()) it->second->state.Override(m.Slot(0).s, PoolState::FAILED);
				return;  // otherwise a task we have not heard of yet
			}
			if (code == "M_DONE") {
				OnFinished(e->id, PoolState::DONE, m.sender, t);
			} else if (code == "M_FAIL") {
				OnFinished(e->id, m.Slot(1).s == "IMPOSSIBLE" ? PoolState::IMPOSSIBLE : PoolState::FAILED, m.sender, t);
			} else if (code == "M_DUMP") {
				e->claims.erase(m.sender);
				if (m.Slot(2).b) {
					e->static_dumps.insert(m.sender);
					CheckImpossible(*e, t);
				}
			} else if (code == "M_CLAIM") {
				e->claims[m.sender] = {m.sender, m.Slot(1).n, m.Slot(2).n};
			} else if (code == "M_RELEASE") {
				e->claims.erase(m.sender);
			} else if (code == "M_CMD") {
				if (m.Slot(0).s == "cancel" || m.Slot(0).s == "abort") OnFinished(e->id, PoolState::CANCELLED, m.sender, t);
			}
		}

		void MrsLayer::OnTasks(const Comm::Message& m, double t) {
			for (std::size_t k = 0; k < m.SlotCount(); ++k) {
				if (m.Slot(k).type != FieldType::Ref) continue;
				const Record& r = m.Child(k);
				if (c_.split_trees && IsComplexTask(r)) {
					AddTree(r, t);
					continue;
				}
				std::shared_ptr<const Task::Task> task;
				try {
					task = factory_.BuildTask(r);
				} catch (const Task::TaskLoadError&) {
					continue;  // a task this robot cannot load is a task it cannot do
				}
				AddEntry(task, t);
			}
		}

		void MrsLayer::AddEntry(std::shared_ptr<const Task::Task> task, double t) {
			PoolEntry* e = pool_.Add(task, t);
			if (!e) return;
			++stats_.tasks_received;
			if (!StaticOk(*task)) {
				// Dump rule 1 (spec 06 §5): never on this robot.
				e->state = PoolState::DUMPED_SELF;
				e->static_dumps.insert(self_);
				SendDump(e->id, "IMPOSSIBLE", true, 0, t);
				CheckImpossible(*e, t);
			}
			allocator_->OnTaskReceived(e->id, t);
		}

		// --- tree tasks (spec 11 §2) -------------------------------------------------------------

		void MrsLayer::AddTree(const Record& root, double t) {
			if (root.fields.empty() || trees_.count(root.fields[0].s)) return;  // a repeat
			std::unique_ptr<Tree> tree;
			try {
				tree = std::make_unique<Tree>(Decompose(root));
			} catch (const DecomposeError&) {
				return;
			}
			std::vector<std::shared_ptr<const Task::Task>> leaves;
			try {
				for (const auto& id : tree->tree.leaves) leaves.push_back(factory_.BuildTask(tree->tree.Node(id).task));
			} catch (const Task::TaskLoadError&) {
				return;  // a leaf this robot cannot load: the robot takes no part in the tree
			}
			Tree* raw = tree.get();
			for (const auto& [id, n] : raw->tree.nodes) tree_of_[id] = raw;
			trees_[raw->tree.root] = std::move(tree);
			++stats_.trees;
			for (auto& leaf : leaves) {
				AddEntry(leaf, t);
				if (PoolEntry* e = pool_.Find(leaf->Id()); e && e->state == PoolState::AVAILABLE) e->state = PoolState::BLOCKED;
			}
			UpdateTrees(t);
		}

		std::optional<PoolState> MrsLayer::StateOf(const std::string& id) const {
			if (const PoolEntry* e = pool_.Find(id)) return e->state;
			auto it = tree_of_.find(id);
			if (it == tree_of_.end()) return std::nullopt;
			const Tree& tr = *it->second;
			return tr.state.Of(id, [this](const std::string& leaf) {
				const PoolEntry* e = pool_.Find(leaf);
				return e ? e->state : PoolState::AVAILABLE;
			});
		}

		const TaskTree* MrsLayer::TreeOf(const std::string& id) const {
			auto it = tree_of_.find(id);
			return it == tree_of_.end() ? nullptr : &it->second->tree;
		}

		bool MrsLayer::Gated(const TaskTree& tree, const TreeNode& leaf, const TreeState& state, double t) const {
			const auto leaf_state = [this](const std::string& id) {
				const PoolEntry* e = pool_.Find(id);
				return e ? e->state : PoolState::AVAILABLE;
			};
			for (const auto& before : leaf.after)
				if (state.Of(before, leaf_state) != PoolState::DONE) return true;
			for (const auto& g : leaf.gates) {
				std::unique_ptr<Task::Condition> c;
				try {
					c = factory_.BuildCondition(g);
				} catch (const Task::TaskLoadError&) {
					return true;
				}
				Task::EvalContext ctx{robot_.World(), t, 0.0, robot_.World().Geo(), {}};
				if (c->Evaluate(ctx) != Task::Truth::True) return true;
			}
			// Affinity (R_K): only the robot that did the partner may take it.
			if (!leaf.affinity.empty() && tree.Has(leaf.affinity)) {
				const PoolEntry* p = pool_.Find(leaf.affinity);
				if (!p || p->state != PoolState::DONE || p->done_by != self_) return true;
			}
			return false;
		}

		void MrsLayer::UpdateTrees(double t) {
			const auto leaf_state = [this](const std::string& id) {
				const PoolEntry* e = pool_.Find(id);
				return e ? e->state : PoolState::AVAILABLE;
			};
			for (auto& [root, tr] : trees_) {
				// Nodes that just ended: their end condition, then the leaves they no longer need.
				for (const auto& [id, n] : tr->tree.nodes) {
					if (n.leaf || tr->finished.count(id)) continue;
					PoolState s = tr->state.Of(id, leaf_state);
					if (!Finished(s)) continue;
					tr->finished.insert(id);
					if (s == PoolState::DONE && !n.end.code.empty() && n.end.code != "C_N" && tr->last_finisher == self_) {
						// The robot that finished the last leaf checks the end condition (spec 03 §6.2 rule 6).
						bool ok = false;
						try {
							auto c = factory_.BuildCondition(n.end);
							Task::EvalContext ctx{robot_.World(), t, 0.0, robot_.World().Geo(), {}};
							ok = c->Evaluate(ctx) == Task::Truth::True;
						} catch (const Task::TaskLoadError&) {
						}
						if (!ok) {
							tr->state.Override(id, PoolState::FAILED);
							auto r = messenger_.Begin("M_FAIL", "all", t);
							r.fields.push_back(Tid(id));
							r.fields.push_back(Id("END_NOT_MET"));
							messenger_.Post(r);
						}
					}
					for (const auto& leaf : LeavesUnder(tr->tree, id))
						if (const PoolEntry* e = pool_.Find(leaf); e && !Finished(e->state)) OnFinished(leaf, PoolState::CANCELLED, self_, t);
				}
				// Gates: BLOCKED until the predecessors are done, the gate conditions hold and the affinity fits.
				for (const auto& id : tr->tree.leaves) {
					PoolEntry* e = pool_.Find(id);
					if (!e || (e->state != PoolState::AVAILABLE && e->state != PoolState::BLOCKED)) continue;
					e->state = Gated(tr->tree, tr->tree.Node(id), tr->state, t) ? PoolState::BLOCKED : PoolState::AVAILABLE;
				}
			}
		}

		bool MrsLayer::StaticOk(const Task::Task& task) const { return task.MeetsStaticRequirements(robot_.Profile()); }

		void MrsLayer::OnFinished(const std::string& id, PoolState s, const std::string& by, double t) {
			PoolEntry* e = pool_.Find(id);
			if (!e || Finished(e->state)) return;
			e->state = s;
			if (s == PoolState::DONE) {
				e->done_by = by;
				if (auto it = tree_of_.find(id); it != tree_of_.end()) it->second->last_finisher = by;
			}
			e->claims.clear();
			if (assigned_ == id) {
				if (robot_.CancelTask(id, t)) ++stats_.cancels;
				assigned_.clear();
			}
			allocator_->OnTaskFinished(id, s, t);
		}

		void MrsLayer::CheckImpossible(PoolEntry& e, double t) {
			if (Finished(e.state)) return;
			if (!e.static_dumps.count(self_)) return;
			for (const Peer* p : peers_.Alive(t, c_.peer_timeout))
				if (!e.static_dumps.count(p->name)) return;
			// Every live robot has dumped it for good (spec 06 §5).
			e.state = PoolState::IMPOSSIBLE;
			auto r = messenger_.Begin("M_FAIL", "all", t);
			r.fields.push_back(Tid(e.id));
			r.fields.push_back(Id("IMPOSSIBLE"));
			messenger_.Post(r);
			allocator_->OnTaskFinished(e.id, e.state, t);
		}

		// --- allocation --------------------------------------------------------------------------

		void MrsLayer::Allocate(double t) {
			// A task the robot no longer has (cancelled elsewhere, or failed) is not ours.
			if (!assigned_.empty() && robot_.CurrentTask() != assigned_ && robot_.Idle()) assigned_.clear();
			if (robot_.TasksBlocked() || robot_.Stopped()) return;
			CurrentTask current;
			current.id = assigned_;
			current.started = !assigned_.empty() && robot_.StateOf(assigned_) == TaskState::IN_PROGRESS;
			const Decision d = allocator_->Select(robot_.World(), current, t);
			switch (d.kind) {
			case Decision::Kind::KEEP: break;
			case Decision::Kind::SWITCH:
				if (d.task != assigned_) Assign(d.task, t);
				break;
			case Decision::Kind::IDLE:
				if (!assigned_.empty()) Unassign(t);
				break;
			}
		}

		void MrsLayer::Assign(const std::string& id, double t) {
			PoolEntry* e = pool_.Find(id);
			if (!e) return;
			if (!assigned_.empty()) {
				Unassign(t);
				if (!assigned_.empty()) return;  // the robot could not let go yet (a return or landing runs)
				++stats_.switches;
			}
			robot_.AddTask(e->task->Clone());
			assigned_ = id;
			e->state = PoolState::ACTIVE;
		}

		void MrsLayer::Unassign(double t) {
			if (assigned_.empty()) return;
			if (!robot_.CancelTask(assigned_, t)) return;
			++stats_.cancels;
			if (PoolEntry* e = pool_.Find(assigned_); e && e->state == PoolState::ACTIVE) e->state = PoolState::AVAILABLE;
			assigned_.clear();
		}

		void MrsLayer::AfterRobot(double t) {
			while (!raised_.empty()) {
				const auto ev = raised_.front();
				raised_.pop_front();
				// Before landing for a swap, hold the active task for claim_grace (spec 06 §4.1).
				if ((ev.event == Robot::ControllerEvent::BATTERY_LOW || ev.event == Robot::ControllerEvent::BATTERY_CRITICAL) &&
				    !assigned_.empty()) {
					auto r = messenger_.Begin("M_CLAIM", "all", t);
					r.fields.push_back(Tid(assigned_));
					r.fields.push_back(Field::MakeNum(t + c_.claim_grace));
					r.fields.push_back(Field::MakeNum(0.0));
					messenger_.Post(r);
				}
			}
			while (!robot_events_.empty()) {
				const auto [e, et] = robot_events_.front();
				robot_events_.pop_front();
				PoolEntry* entry = pool_.Find(e.task_id);
				if (!entry) continue;  // a task the robot had from elsewhere (a resumed journal)
				if (e.state == TaskState::SUCCEEDED) {
					if (assigned_ == e.task_id) assigned_.clear();
					if (!Finished(entry->state)) {
						auto r = messenger_.Begin("M_DONE", "all", et);
						r.fields.push_back(Tid(e.task_id));
						messenger_.Post(r);
						++stats_.done;
					}
					OnFinished(e.task_id, PoolState::DONE, self_, et);
				} else if (e.state == TaskState::FAILED) {
					if (assigned_ == e.task_id) assigned_.clear();
					const long progress = static_cast<long>(robot_.Executor().IteratorOf(e.task_id));
					switch (e.reason) {
					case FailReason::NO_BEHAVIOUR:
					case FailReason::NO_ACTUATOR:
					case FailReason::IMPOSSIBLE:
						entry->state = PoolState::DUMPED_SELF;
						entry->static_dumps.insert(self_);
						entry->claims.erase(self_);
						SendDump(e.task_id, Task::FailReasonName(e.reason), true, progress, et);
						CheckImpossible(*entry, et);
						break;
					case FailReason::END_NOT_MET:
					case FailReason::ABORTED: {
						auto r = messenger_.Begin("M_FAIL", "all", et);
						r.fields.push_back(Tid(e.task_id));
						r.fields.push_back(Id(Task::FailReasonName(e.reason)));
						messenger_.Post(r);
						++stats_.failed;
						OnFinished(e.task_id, PoolState::FAILED, self_, et);
						break;
					}
					default:  // MISSING_FIELD, RUNTIME_CONDITION, START_UNREACHABLE, PREEMPTED_OUT, ...
						entry->state = PoolState::AVAILABLE;
						entry->retry_after = et + c_.retry_cooldown;
						entry->claims.erase(self_);
						SendDump(e.task_id, Task::FailReasonName(e.reason), false, progress, et);
						break;
					}
				}
			}
		}

		// --- outgoing ----------------------------------------------------------------------------

		void MrsLayer::SendDump(const std::string& id, const std::string& reason, bool is_static, long progress, double t) {
			auto r = messenger_.Begin("M_DUMP", "all", t);
			r.fields.push_back(Tid(id));
			r.fields.push_back(Id(reason));
			r.fields.push_back(Field::MakeBool(is_static));
			r.fields.push_back(Field::MakeInt(progress));
			messenger_.Post(r);
			if (is_static) ++stats_.dumps_static;
			else ++stats_.dumps_dynamic;
		}

		void MrsLayer::SendState(double t) {
			last_state_ = t;
			auto& w = robot_.World();
			auto x = w.Scalar("pose.enu.x", t), y = w.Scalar("pose.enu.y", t), z = w.Scalar("pose.enu.z", t);
			if (!x || !y || !z) return;
			Environment::View v;
			v.code = "V_PEER";
			v.stamp = std::round(t * 1000.0) / 1000.0;
			v.values = {static_cast<double>(robot_.DeviceSide().self.id), *x, *y, *z, w.Scalar("vel.enu.x", t).value_or(0.0),
			            w.Scalar("vel.enu.y", t).value_or(0.0), w.Scalar("vel.enu.z", t).value_or(0.0),
			            w.Scalar("battery.remaining", t).value_or(-1.0)};
			v.text = assigned_.empty() ? "0" : assigned_;
			auto r = messenger_.Begin("M_STATE", "all", t);
			r.fields.push_back(Field::MakeRef(0));
			r.children.push_back(Environment::ToRecord(v));
			messenger_.Post(r);
		}

		void MrsLayer::SendProfile(double t) {
			last_profile_ = t;
			const Record p = robot_.DeviceSide().self.ToProfileMessage(messenger_.Mission(), 0, t);
			auto r = messenger_.Begin("M_PROFILE", "all", t);
			r.fields.insert(r.fields.end(), p.fields.begin() + 6, p.fields.end());
			r.children = p.children;
			messenger_.Post(r);
		}
	}
}
