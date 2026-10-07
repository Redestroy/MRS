#include "mrs/algorithms/Planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "mrs/world/View.h"

namespace MRS {
	namespace Algorithms {
		using Protocol::FieldType;
		using Protocol::Record;

		// --- task geometry -----------------------------------------------------------------------

		namespace {
			void Walk(const Record& r, TaskGeometry& g) {
				if (r.code == "C_P3" && r.fields.size() >= 3 && !g.has_target) {
					g.has_target = true;
					g.target = {r.fields[0].n, r.fields[1].n, r.fields[2].n};
				}
				if (r.code == "C_T" && !r.fields.empty()) g.release = std::max(g.release, static_cast<double>(r.fields[0].i) / 1000.0);
				if ((r.code == "A_W" || r.code == "A_HD") && !r.fields.empty()) g.hold += std::max(0.0, r.fields[0].n);
				for (const auto& c : r.children) Walk(c, g);
			}
		}

		TaskGeometry GeometryOf(const Record& task) {
			TaskGeometry g;
			if (!task.fields.empty() && task.fields[0].type == FieldType::TaskId) g.id = task.fields[0].s;
			Walk(task, g);
			return g;
		}

		const char* FlightModeName(FlightMode m) {
			switch (m) {
			case FlightMode::LAYERED: return "layered";
			case FlightMode::ALTITUDE_FIRST: return "altitude_first";
			case FlightMode::DIRECT: return "direct";
			}
			return "?";
		}

		const char* PlanObjectiveName(PlanObjective o) { return o == PlanObjective::MAKESPAN ? "makespan" : "completion"; }

		double TravelModel::Time(const std::array<double, 3>& a, const std::array<double, 3>& b, double layer) const {
			const double across = std::hypot(b[0] - a[0], b[1] - a[1]);
			const double t = a[2] < 0.3 ? takeoff : 0.0;
			if (across <= switch_radius || mode == FlightMode::DIRECT)
				return t + std::max(across / v_xy, std::fabs(b[2] - a[2]) / v_z);
			// Up or down to the crossing altitude, across, then to the target's altitude.
			const double cross = mode == FlightMode::LAYERED ? layer : std::max(cruise_alt, b[2]);
			return t + std::fabs(cross - a[2]) / v_z + across / v_xy + std::fabs(cross - b[2]) / v_z;
		}

		// --- route planning ----------------------------------------------------------------------

		namespace {
			struct RouteEval {
				double end = 0.0;
				double sum = 0.0;
			};

			struct Cost {
				double makespan = 0.0;
				double sum = 0.0;
			};

			// Lexicographic: the objective's measure first, the other one to break ties.
			bool Better(PlanObjective o, const Cost& a, const Cost& b) {
				constexpr double eps = 1e-6;
				const double a1 = o == PlanObjective::MAKESPAN ? a.makespan : a.sum, b1 = o == PlanObjective::MAKESPAN ? b.makespan : b.sum;
				const double a2 = o == PlanObjective::MAKESPAN ? a.sum : a.makespan, b2 = o == PlanObjective::MAKESPAN ? b.sum : b.makespan;
				if (a1 < b1 - eps) return true;
				if (a1 > b1 + eps) return false;
				return a2 < b2 - eps;
			}

			class Search {
			public:
				Search(const std::vector<PlanRobot>& robots, const std::vector<PlanTask>& tasks, double now, const PlannerConfig& c)
				    : robots_(robots), tasks_(tasks), now_(now), c_(c), routes_(robots.size()), evals_(robots.size()) {}

				RouteEval Evaluate(std::size_t r, const std::vector<int>& route, std::map<std::string, double>* finish = nullptr) const {
					RouteEval e;
					double t = now_;
					std::array<double, 3> p = robots_[r].pos;
					for (int k : route) {
						const TaskGeometry& g = tasks_[static_cast<std::size_t>(k)].g;
						if (g.has_target) {
							t += c_.travel.Time(p, g.target, robots_[r].layer);
							p = g.target;
						}
						t = std::max(t, g.release);
						t += g.hold + c_.travel.settle;
						e.sum += t;
						if (finish) (*finish)[g.id] = t;
					}
					e.end = t;
					return e;
				}

				// The cost with route r replaced by its evaluation e (and route q by f, if q is valid).
				Cost With(std::size_t r, const RouteEval& e, std::size_t q = npos, const RouteEval& f = {}) const {
					Cost c;
					for (std::size_t k = 0; k < routes_.size(); ++k) {
						const RouteEval& x = k == r ? e : (k == q ? f : evals_[k]);
						c.makespan = std::max(c.makespan, x.end - now_);
						c.sum += x.sum;
					}
					return c;
				}

				Cost Current() const { return With(npos, {}); }

				bool Better(const Cost& a, const Cost& b) const { return Algorithms::Better(c_.objective, a, b); }

				bool Allowed(std::size_t r, int k) const { return !tasks_[static_cast<std::size_t>(k)].excluded.count(robots_[r].name); }
				std::size_t First(std::size_t r) const { return fixed_[r] ? 1 : 0; }

				void Build(std::vector<std::string>& unassigned) {
					fixed_.assign(robots_.size(), false);
					std::vector<int> order;
					for (std::size_t k = 0; k < tasks_.size(); ++k) {
						bool placed = false;
						for (std::size_t r = 0; r < robots_.size(); ++r)
							if (!robots_[r].fixed.empty() && robots_[r].fixed == tasks_[k].g.id && routes_[r].empty()) {
								routes_[r].push_back(static_cast<int>(k));
								fixed_[r] = true;
								placed = true;
								break;
							}
						if (!placed) order.push_back(static_cast<int>(k));
					}
					for (std::size_t r = 0; r < robots_.size(); ++r) evals_[r] = Evaluate(r, routes_[r]);
					std::stable_sort(order.begin(), order.end(), [this](int a, int b) {
						return tasks_[static_cast<std::size_t>(a)].g.release < tasks_[static_cast<std::size_t>(b)].g.release;
					});
					for (int k : order)
						if (!InsertBest(k)) unassigned.push_back(tasks_[static_cast<std::size_t>(k)].g.id);
				}

				// Inserts task k where it costs least. False when no robot may take it.
				bool InsertBest(int k) {
					bool found = false;
					Cost best;
					std::size_t br = 0, bp = 0;
					RouteEval be;
					for (std::size_t r = 0; r < robots_.size(); ++r) {
						if (!Allowed(r, k)) continue;
						for (std::size_t p = First(r); p <= routes_[r].size(); ++p) {
							auto route = routes_[r];
							route.insert(route.begin() + static_cast<long>(p), k);
							const RouteEval e = Evaluate(r, route);
							const Cost c = With(r, e);
							if (!found || Better(c, best)) {
								found = true;
								best = c;
								br = r;
								bp = p;
								be = e;
							}
						}
					}
					if (!found) return false;
					routes_[br].insert(routes_[br].begin() + static_cast<long>(bp), k);
					evals_[br] = be;
					return true;
				}

				void Improve() {
					for (int round = 0; round < c_.improve_rounds; ++round) {
						bool improved = false;
						improved |= Relocate();
						improved |= Swap();
						improved |= TwoOpt();
						if (!improved) break;
					}
				}

				Plan Result(std::vector<std::string> unassigned) const {
					Plan plan;
					for (std::size_t r = 0; r < robots_.size(); ++r) {
						auto& out = plan.routes[robots_[r].name];
						for (int k : routes_[r]) out.push_back(tasks_[static_cast<std::size_t>(k)].g.id);
						const RouteEval e = Evaluate(r, routes_[r], &plan.finish);
						plan.makespan = std::max(plan.makespan, e.end - now_);
					}
					plan.unassigned = std::move(unassigned);
					return plan;
				}

			private:
				static constexpr std::size_t npos = static_cast<std::size_t>(-1);

				bool Relocate() {
					bool improved = false;
					for (std::size_t r = 0; r < routes_.size(); ++r) {
						for (std::size_t i = First(r); i < routes_[r].size(); ++i) {
							const int k = routes_[r][i];
							const Cost before = Current();
							auto from = routes_[r];
							from.erase(from.begin() + static_cast<long>(i));
							const RouteEval fe = Evaluate(r, from);
							bool moved = false;
							for (std::size_t q = 0; q < routes_.size() && !moved; ++q) {
								if (!Allowed(q, k)) continue;
								const auto& base = q == r ? from : routes_[q];
								for (std::size_t p = First(q); p <= base.size(); ++p) {
									if (q == r && p == i) continue;
									auto to = base;
									to.insert(to.begin() + static_cast<long>(p), k);
									const RouteEval te = Evaluate(q, to);
									const Cost c = q == r ? With(r, te) : With(r, fe, q, te);
									if (Better(c, before)) {
										if (q == r) {
											routes_[r] = to;
											evals_[r] = te;
										} else {
											routes_[r] = from;
											evals_[r] = fe;
											routes_[q] = to;
											evals_[q] = te;
										}
										moved = improved = true;
										break;
									}
								}
							}
						}
					}
					return improved;
				}

				bool Swap() {
					bool improved = false;
					for (std::size_t r = 0; r < routes_.size(); ++r)
						for (std::size_t q = r + 1; q < routes_.size(); ++q)
							for (std::size_t i = First(r); i < routes_[r].size(); ++i)
								for (std::size_t j = First(q); j < routes_[q].size(); ++j) {
									const int a = routes_[r][i], b = routes_[q][j];
									if (!Allowed(q, a) || !Allowed(r, b)) continue;
									auto ra = routes_[r], rb = routes_[q];
									ra[i] = b;
									rb[j] = a;
									const RouteEval ea = Evaluate(r, ra), eb = Evaluate(q, rb);
									if (Better(With(r, ea, q, eb), Current())) {
										routes_[r] = ra;
										routes_[q] = rb;
										evals_[r] = ea;
										evals_[q] = eb;
										improved = true;
									}
								}
					return improved;
				}

				bool TwoOpt() {
					bool improved = false;
					for (std::size_t r = 0; r < routes_.size(); ++r) {
						const std::size_t n = routes_[r].size();
						for (std::size_t i = First(r); i + 1 < n; ++i)
							for (std::size_t j = i + 1; j < n; ++j) {
								auto route = routes_[r];
								std::reverse(route.begin() + static_cast<long>(i), route.begin() + static_cast<long>(j) + 1);
								const RouteEval e = Evaluate(r, route);
								if (Better(With(r, e), Current())) {
									routes_[r] = route;
									evals_[r] = e;
									improved = true;
								}
							}
					}
					return improved;
				}

				const std::vector<PlanRobot>& robots_;
				const std::vector<PlanTask>& tasks_;
				double now_;
				const PlannerConfig& c_;
				std::vector<std::vector<int>> routes_;
				std::vector<RouteEval> evals_;
				std::vector<bool> fixed_;
			};
		}

		Plan PlanRoutes(const std::vector<PlanRobot>& robots, const std::vector<PlanTask>& tasks, double now, const PlannerConfig& c) {
			Search s(robots, tasks, now, c);
			std::vector<std::string> unassigned;
			s.Build(unassigned);
			s.Improve();
			return s.Result(std::move(unassigned));
		}

		// --- route following ---------------------------------------------------------------------

		namespace {
			// The first task of the route this robot may do now; KEEP for the current one.
			Decision Follow(const std::vector<std::string>& route, TaskPool& pool, const CurrentTask& current, double t,
			                const std::function<bool(const PoolEntry&, double)>& eligible) {
				if (current.started) {
					const PoolEntry* cur = pool.Find(current.id);
					if (cur && eligible(*cur, t)) return {Decision::Kind::KEEP, current.id};
				}
				for (const auto& id : route) {
					const PoolEntry* e = pool.Find(id);
					if (!e || !eligible(*e, t)) continue;
					if (id == current.id) return {Decision::Kind::KEEP, id};
					return {Decision::Kind::SWITCH, id};
				}
				return {Decision::Kind::IDLE, {}};
			}

			void Erase(std::vector<std::string>& route, const std::string& id) {
				route.erase(std::remove(route.begin(), route.end(), id), route.end());
			}
		}

		void PlannerAllocator::OnTaskReceived(const std::string&, double) { dirty_ = true; }

		void PlannerAllocator::OnTaskFinished(const std::string& task, PoolState, double) { Erase(route_, task); }

		Decision PlannerAllocator::Select(const Environment::Worldview& w, const CurrentTask& current, double t) {
			auto eligible = [this](const PoolEntry& e, double now) { return Eligible(e, now); };
			if (dirty_) {
				auto x = w.Scalar("pose.enu.x", t), y = w.Scalar("pose.enu.y", t), z = w.Scalar("pose.enu.z", t);
				if (x && y && z) {
					PlanRobot self;
					self.name = ctx_.self;
					self.pos = {*x, *y, *z};
					if (auto layer = w.Raw("layer.alt"); layer && std::holds_alternative<double>(layer->value))
						self.layer = std::get<double>(layer->value);
					if (current.started) self.fixed = current.id;
					std::vector<PlanTask> tasks;
					for (const PoolEntry* e : ctx_.pool->Entries())
						if (Eligible(*e, t)) tasks.push_back({GeometryOf(e->task->ToRecord()), {}});
					route_ = PlanRoutes({self}, tasks, t, c_).routes[ctx_.self];
					dirty_ = false;
					++replans_;
				}
			}
			return Follow(route_, *ctx_.pool, current, t, eligible);
		}

		void PlanFollowerAllocator::OnMessage(const Comm::Message& m, double) {
			if (m.code != "M_PLAN" || m.SlotCount() < 2) return;
			const long revision = static_cast<long>(m.Slot(0).i);
			if (revision <= revision_) return;
			revision_ = revision;
			route_.clear();
			const auto n = static_cast<std::size_t>(m.Slot(1).i);
			for (std::size_t k = 0; k < n && 2 + k < m.SlotCount(); ++k) route_.push_back(m.Slot(2 + k).s);
		}

		void PlanFollowerAllocator::OnTaskFinished(const std::string& task, PoolState, double) { Erase(route_, task); }

		Decision PlanFollowerAllocator::Select(const Environment::Worldview&, const CurrentTask& current, double t) {
			auto eligible = [this](const PoolEntry& e, double now) { return Eligible(e, now); };
			return Follow(route_, *ctx_.pool, current, t, eligible);
		}

		// --- the central planner -----------------------------------------------------------------

		CentralPlanner::CentralPlanner(const Record& h, PlannerConfig c) : c_(c) {
			// H_M: mission lat0 lon0 alt0 xmin xmax ymin ymax zmax layer_base layer_step claim_grace n, n × (robot x y z).
			const auto num = [&h](std::size_t k) {
				const auto& f = h.fields.at(k);
				return f.type == FieldType::Int ? static_cast<double>(f.i) : f.n;
			};
			const double base = num(9), step = num(10);
			const auto n = static_cast<std::size_t>(h.fields.at(12).i);
			for (std::size_t k = 0; k < n; ++k) {
				const std::size_t at = 13 + 4 * k;
				const long id = static_cast<long>(num(at));
				Known r;
				r.pos = {num(at + 1), num(at + 2), num(at + 3)};
				r.layer = base + static_cast<double>(id - 1) * step;
				robots_["r" + std::to_string(id)] = r;
			}
		}

		void CentralPlanner::OnMessage(const Comm::Message& m, double) {
			if (m.code == "M_STATE") {
				auto it = robots_.find(m.sender);
				if (it == robots_.end()) return;
				auto view = Environment::ViewFromRecord(m.Child(0));
				if (!view || view->values.size() < 4) return;
				it->second.pos = {view->values[1], view->values[2], view->values[3]};
				it->second.current = view->text == "0" ? std::string() : view->text;
			} else if (m.code == "M_DUMP" && m.SlotCount() >= 3 && m.Slot(2).b) {
				excluded_[m.Slot(0).s].insert(m.sender);
			}
		}

		Plan CentralPlanner::Replan(const std::vector<Record>& open, double t) {
			std::vector<PlanTask> tasks;
			for (const auto& r : open) {
				PlanTask task{GeometryOf(r), {}};
				if (auto it = excluded_.find(task.g.id); it != excluded_.end()) task.excluded = it->second;
				tasks.push_back(task);
			}
			std::vector<PlanRobot> robots;
			for (const auto& [name, k] : robots_) {
				PlanRobot r;
				r.name = name;
				r.pos = k.pos;
				r.layer = k.layer;
				// A robot keeps the task it is on: moving tasks between robots in flight makes them
				// turn back and forth as the plan changes (spec 10 §2.3).
				for (const auto& task : tasks)
					if (task.g.id == k.current) r.fixed = k.current;
				robots.push_back(r);
			}
			std::sort(robots.begin(), robots.end(), [](const PlanRobot& a, const PlanRobot& b) { return RobotNumber(a.name) < RobotNumber(b.name); });
			++revision_;
			const auto start = std::chrono::steady_clock::now();
			Plan plan = PlanRoutes(robots, tasks, t, c_);
			const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
			++calls;
			total_s += s;
			worst_s = std::max(worst_s, s);
			return plan;
		}
	}
}
