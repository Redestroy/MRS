#include "mrs/algorithms/TaskIssuer.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Algorithms {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		TaskIssuer::TaskIssuer(Comm::ITransport& transport, Record mission_header, IssuerConfig config)
		    : messenger_(transport, config.name), header_(std::move(mission_header)), c_(std::move(config)) {
			messenger_.SetMission(header_.fields.at(0).s);
		}

		void TaskIssuer::LoadTimeline(const std::string& text) {
			auto parsed = Protocol::Parse(text);
			if (!parsed.Ok())
				throw std::runtime_error(std::string(Protocol::ErrorClassName(parsed.error->error_class)) + " at byte " +
				                         std::to_string(parsed.error->offset) + ": " + parsed.error->message);
			for (const auto& r : parsed.document.records) {
				if (r.code != "L_D") continue;
				std::vector<Record> tasks;
				for (std::size_t k = 1; k < r.fields.size(); ++k)
					if (r.fields[k].type == FieldType::Ref) tasks.push_back(r.children.at(r.fields[k].ref));
				Schedule(r.fields[0].n, tasks);
			}
		}

		void TaskIssuer::Schedule(double t, const std::vector<Record>& tasks) {
			// Sorted by time, file order kept for equal times (fixes the 2021 supervisor).
			auto it = std::upper_bound(timeline_.begin() + static_cast<long>(next_), timeline_.end(), t,
			                           [](double v, const Entry& e) { return v < e.t; });
			timeline_.insert(it, Entry{t, tasks});
		}

		void TaskIssuer::Tick(double t) {
			if (t - last_mission_ >= c_.mission_period) {
				last_mission_ = t;
				auto r = messenger_.Begin("M_MISSION", "all", t);
				r.fields.push_back(Field::MakeRef(0));
				r.children.push_back(header_);
				messenger_.Post(r);
			}
			bool replan = false;
			while (next_ < timeline_.size() && timeline_[next_].t <= t) {
				const Entry& e = timeline_[next_++];
				// One message per task, so each fits the link (spec 06 §2).
				for (const auto& task : e.tasks) {
					auto r = messenger_.Begin("M_TASK", "all", t);
					r.fields.push_back(Field::MakeRef(0));
					r.children.push_back(task);
					messenger_.Post(r);
					const std::string id = task.fields.at(0).s;
					records_[id] = task;
					if (!issued_.count(id)) order_.push_back(id);
					replan = true;
					IssuedTask& it = issued_[id];
					it.id = id;
					it.dispatch = t;
					if (!first_dispatch_) first_dispatch_ = t;
					if (IsComplexTask(task) && !trees_.count(id)) {
						try {
							auto tree = std::make_unique<IssuedTree>(Decompose(task));
							for (const auto& [node, n] : tree->tree.nodes) root_of_[node] = id;
							for (const auto& leaf : tree->tree.leaves) leaves_[leaf] = IssuedTask{leaf, t, {}, {}, 0, {}, {}};
							for (const auto& u : tree->tree.units)
								if (!tree->tree.Node(u).leaf && u != id) units_[u] = IssuedTask{u, t, {}, {}, 0, {}, {}};
							trees_[id] = std::move(tree);
						} catch (const DecomposeError& err) {
							it.failed = std::string("DECOMPOSE: ") + err.what();
						}
					}
				}
			}
			if (c_.task_period > 0 && t - last_repeat_ >= c_.task_period) {
				last_repeat_ = t;
				for (const auto& [id, it] : issued_) {
					if (it.done || it.failed) continue;
					auto r = messenger_.Begin("M_TASK", "all", t);
					r.fields.push_back(Field::MakeRef(0));
					r.children.push_back(records_.at(id));
					messenger_.Post(r);
				}
			}
			for (const auto& m : messenger_.Receive()) {
				if (planner_) {
					planner_->OnMessage(m, t);
					if (m.code == "M_DUMP" && m.SlotCount() >= 3 && m.Slot(2).b) replan = true;
				}
				if (m.SlotCount() == 0 || m.Slot(0).type != FieldType::TaskId) continue;
				if (auto r = root_of_.find(m.Slot(0).s); r != root_of_.end() && r->first != r->second) {
					OnTreeMessage(m, r->second);
					continue;
				}
				auto it = issued_.find(m.Slot(0).s);
				if (it == issued_.end()) continue;
				if (m.code == "M_DONE") {
					++it->second.done_count;
					if (!it->second.done) {
						it->second.done = m.stamp;
						it->second.done_by = m.sender;
						replan = true;  // the robot is free: plan with where it is now
					}
				} else if (m.code == "M_FAIL") {
					if (!it->second.failed && !it->second.done) {
						it->second.failed = m.Slot(1).s;
						replan = true;
					}
				} else if (m.code == "M_DUMP" && m.Slot(2).b) {
					it->second.dumped_by.push_back(m.sender);
				}
			}
			if (planner_ && replan) SendPlans(t);
		}

		const TaskTree* TaskIssuer::Tree(const std::string& root) const {
			auto it = trees_.find(root);
			return it == trees_.end() ? nullptr : &it->second->tree;
		}

		void TaskIssuer::OnTreeMessage(const Comm::Message& m, const std::string& root) {
			IssuedTree& tr = *trees_.at(root);
			const std::string id = m.Slot(0).s;
			IssuedTask* found = nullptr;
			if (auto leaf = leaves_.find(id); leaf != leaves_.end()) found = &leaf->second;
			else if (auto unit = units_.find(id); unit != units_.end()) found = &unit->second;
			if (found) {
				IssuedTask& it = *found;
				if (m.code == "M_DONE") {
					++it.done_count;
					if (!it.done) {
						it.done = m.stamp;
						it.done_by = m.sender;
					}
				} else if (m.code == "M_FAIL") {
					if (!it.failed && !it.done) it.failed = m.Slot(1).s;
				} else if (m.code == "M_DUMP" && m.Slot(2).b) {
					it.dumped_by.push_back(m.sender);
				}
			} else if (m.code == "M_FAIL") {
				tr.state.Override(id, PoolState::FAILED);  // a node whose end condition failed
			}
			IssuedTask& top = issued_.at(root);
			if (top.done || top.failed) return;
			const PoolState s = tr.state.Of(root, [this](const std::string& unit) {
				auto u = units_.find(unit);
				auto l = leaves_.find(unit);
				if (u == units_.end() && l == leaves_.end()) return PoolState::AVAILABLE;  // the root as one unit
				const IssuedTask& it = u != units_.end() ? u->second : l->second;
				if (it.done) return PoolState::DONE;
				if (it.failed) return *it.failed == "IMPOSSIBLE" ? PoolState::IMPOSSIBLE : PoolState::FAILED;
				return PoolState::AVAILABLE;
			});
			if (s == PoolState::DONE) {
				top.done = m.stamp;
				top.done_by = m.sender;
				top.done_count = 1;
			} else if (Finished(s)) {
				top.failed = "TREE";
			}
		}

		void TaskIssuer::SendPlans(double t) {
			std::vector<Record> open;
			for (const auto& id : order_) {
				const IssuedTask& it = issued_.at(id);
				// The central planner plans atomic tasks only; tree tasks are out of its scope (spec 11 §2.5).
				if (!it.done && !it.failed && !trees_.count(id)) open.push_back(records_.at(id));
			}
			const Plan plan = planner_->Replan(open, t);
			for (const auto& [robot, route] : plan.routes) {
				auto r = messenger_.Begin("M_PLAN", robot, t);
				r.fields.push_back(Field::MakeInt(planner_->Revision()));
				r.fields.push_back(Field::MakeInt(static_cast<std::int64_t>(route.size())));
				for (const auto& id : route) r.fields.push_back(Field::MakeText(FieldType::TaskId, id));
				messenger_.Post(r);
				++plans_sent_;
			}
		}

		bool TaskIssuer::Done() const {
			if (next_ < timeline_.size()) return false;
			for (const auto& [id, it] : issued_)
				if (!it.done && !it.failed) return false;
			return true;
		}

		std::optional<double> TaskIssuer::Makespan() const {
			if (!Done() || !first_dispatch_) return std::nullopt;
			double last = *first_dispatch_;
			for (const auto& [id, it] : issued_)
				if (it.done) last = std::max(last, *it.done);
			return last - *first_dispatch_;
		}

		void TaskIssuer::Cancel(const std::string& task, double t) {
			auto r = messenger_.Begin("M_CMD", "all", t);
			r.fields.push_back(Field::MakeText(FieldType::Id, "cancel"));
			r.fields.push_back(Field::MakeText(FieldType::TaskId, task));
			messenger_.Post(r);
			if (auto it = issued_.find(task); it != issued_.end() && !it->second.done) it->second.failed = "CANCELLED";
		}

		// --- the 2021 task sets ------------------------------------------------------------------

		namespace {
			std::string Num(double v) {
				std::ostringstream s;
				s << std::setprecision(6) << (std::fabs(v) < 5e-7 ? 0.0 : v);
				return s.str();
			}

			// An LED colour per 2021 LED index, so tasks are told apart in Webots.
			long Colour(long k) {
				static const long kColours[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0x00FFFF, 0xFF00FF, 0xFFFFFF, 0xFF8000};
				return kColours[((k % 8) + 8) % 8];
			}
		}

		std::string Port2021TaskSet(const std::string& text, const Port2021Config& c) {
			std::ostringstream out;
			out << "# Ported from a 2021 E-puck task set (spec 09 §7): " << c.scale << " m per unit, origin at (" << c.cx << ", "
			    << c.cy << "), altitude " << c.altitude << " m.\n@: MRS 0.1/\n";
			std::istringstream lines(text);
			std::string line;
			while (std::getline(lines, line)) {
				if (line.find("T:") == std::string::npos) continue;
				std::istringstream s(line);
				double time = 0;
				std::string tk, kind;
				long id = 0, state = 0;
				double prio = 0;
				s >> time >> tk >> id >> state >> kind >> prio;
				const auto cp = line.find("C_P");
				if (!s || cp == std::string::npos) throw std::runtime_error("not a 2021 task line: " + line);
				double x = 0, y = 0, z = 0, tol = 0;
				std::istringstream p(line.substr(cp + 3));
				p >> x >> y >> z >> tol;
				std::vector<std::string> actions;
				bool led_on = false;
				for (std::size_t at = line.find("/A_"); at != std::string::npos; at = line.find("/A_", at + 1)) {
					std::istringstream a(line.substr(at + 1));
					std::string code;
					double v = 0;
					a >> code >> v;
					if (code == "A_L") {
						led_on = !led_on;  // 2021 sets toggled the LED: on, wait, off, wait
						actions.push_back("A_L 3 " + std::to_string(led_on ? Colour(static_cast<long>(v)) : 0));
					} else if (code == "A_W") {
						actions.push_back("A_W " + Num(v * c.wait_scale));
					}
				}
				actions.push_back("A_N");
				const double ex = (x - c.cx) * c.scale, ey = (y - c.cy) * c.scale;
				out << "\n# 2021 task " << id << " at (" << Num(x) << ", " << Num(y) << ")\n";
				out << "L: L_D " << Num(time) << " T_1/\n";
				out << "T_1: T_A " << c.issuer << "." << (id + 1) << " " << Num(prio) << " 0 C_1 C_2 A_1";
				if (actions.size() > 1) out << ".." << actions.size();
				out << "/\nC_1: C_P3 " << Num(ex) << " " << Num(ey) << " " << Num(c.altitude) << " " << Num(c.tol_xy) << " "
				    << Num(c.tol_z) << " 0 -1/\nC_2: C_N/\n";
				for (std::size_t k = 0; k < actions.size(); ++k) out << "A_" << (k + 1) << ": " << actions[k] << "/\n";
			}
			return out.str();
		}
	}
}
