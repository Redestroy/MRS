#include "mrs/sim/Experiment.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>

#include "mrs/sim/TaskSets.h"

namespace MRS {
	namespace Sim {
		const char* ConditionName(Condition c) {
			switch (c) {
			case Condition::S1: return "S1";
			case Condition::S1_ORACLE: return "S1*";
			case Condition::G_RTA: return "G-RTA";
			case Condition::G_RTA_X: return "G-RTA-X";
			case Condition::G_C: return "G-C";
			case Condition::G_STA: return "G-STA";
			case Condition::G_CBBA: return "G-CBBA";
			case Condition::G_LDTA2: return "G-LDTA2";
			}
			return "?";
		}

		bool ParseCondition(const std::string& s, Condition& c) {
			for (Condition x : {Condition::S1, Condition::S1_ORACLE, Condition::G_RTA, Condition::G_RTA_X, Condition::G_C, Condition::G_STA, Condition::G_CBBA, Condition::G_LDTA2})
				if (s == ConditionName(x)) {
					c = x;
					return true;
				}
			return false;
		}

		bool IsSingle(Condition c) { return c == Condition::S1 || c == Condition::S1_ORACLE; }

		namespace {
			std::uint64_t Mix(std::uint64_t z) {
				z += 0x9E3779B97F4A7C15ULL;
				z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
				z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
				return z ^ (z >> 31);
			}

			// The homes in a row, in an order drawn from the seed (spec 10 §3.4).
			std::vector<std::array<double, 3>> Homes(int n, double spacing, std::uint64_t seed) {
				auto homes = RowHomes(n, spacing);
				std::uint64_t s = seed;
				for (std::size_t i = homes.size(); i > 1; --i) {
					s = Mix(s);
					std::swap(homes[i - 1], homes[static_cast<std::size_t>(s % i)]);
				}
				return homes;
			}
		}

		RunResult RunOne(const RobotFiles& files, const RunSpec& spec) {
			const auto wall = std::chrono::steady_clock::now();
			const int n = IsSingle(spec.condition) ? 1 : spec.n;

			TeamConfig c;
			c.mission = spec.mission;
			c.mission.homes = Homes(n, spec.home_spacing, spec.seed);
			c.quad.gps_noise = spec.gps_noise;
			c.quad.capacity_wh = spec.battery_wh;
			c.quad.seed = Mix(spec.seed) & 0xFFFFFFFFULL;
			c.bitrate_bps = spec.bitrate_bps;
			c.repulsion_lead = spec.repulsion_lead;
			c.ranging_noise = spec.ranging_noise;
			// A limited channel loses messages, so the issuer repeats open tasks (spec 13 §3).
			if (spec.bitrate_bps > 0.0) c.issuer.task_period = 15.0;
			Algorithms::TravelModel travel = spec.travel;
			travel.mode = IsSingle(spec.condition) ? Algorithms::FlightMode::ALTITUDE_FIRST : Algorithms::FlightMode::LAYERED;
			Algorithms::PlannerConfig pc;
			pc.travel = travel;
			// The single UAV flies altitude-first; the groups keep the layered default (plan §10.2).
			if (IsSingle(spec.condition)) {
				c.behaviour_priority["flyto.altitude_first"] = 11;
				c.mrs.split_trees = false;  // the oracle form is one robot's own sequence (spec 10 §3.3)
			}
			switch (spec.condition) {
			case Condition::S1:
			case Condition::S1_ORACLE: c.allocator = [pc](int) { return std::make_unique<Algorithms::PlannerAllocator>(pc); }; break;
			case Condition::G_RTA:
			case Condition::G_RTA_X: {
				Algorithms::RtaConfig rta = spec.rta;
				rta.exclusive = spec.condition == Condition::G_RTA_X;
				c.allocator = [rta](int) { return std::make_unique<Algorithms::RtaAllocator>(rta); };
				break;
			}
			case Condition::G_C: c.allocator = [](int) { return std::make_unique<Algorithms::PlanFollowerAllocator>(); }; break;
			case Condition::G_STA: {
				Algorithms::StaConfig sta = spec.sta;
				sta.rta = spec.rta;
				c.allocator = [sta](int) { return std::make_unique<Algorithms::StaAllocator>(sta); };
				break;
			}
			case Condition::G_CBBA: {
				Algorithms::CbbaConfig cb = spec.cbba;
				cb.travel = travel;
				c.allocator = [cb](int) { return std::make_unique<Algorithms::CbbaAllocator>(cb); };
				break;
			}
			case Condition::G_LDTA2: {
				Algorithms::Ldta2Config lc;
				lc.rta = spec.rta;
				c.allocator = [lc](int) { return std::make_unique<Algorithms::Ldta2Allocator>(lc); };
				break;
			}
			}
			// Tasks keep arriving, so the central planner minimises the sum of completions and lets
			// the makespan break ties (spec 10 §2.3).
			Algorithms::PlannerConfig central = pc;
			central.objective = Algorithms::PlanObjective::COMPLETION;

			Team team(files, c);
			if (spec.condition == Condition::G_C) team.issuer.SetPlanner(std::make_unique<Algorithms::CentralPlanner>(c.mission.Header(), central));
			team.issuer.LoadTimeline(spec.condition == Condition::S1_ORACLE ? OracleTimeline(spec.timeline) : spec.timeline);
			const auto release = ReleaseTimes(spec.timeline);

			RunResult r;
			r.completed = team.Run(spec.time_limit);
			r.sim_time = team.Time();
			r.tasks = static_cast<int>(release.size());
			double first = 1e18, last = 0.0, latency_sum = 0.0;
			for (const auto& [id, t] : release) first = std::min(first, t);
			for (const auto& [id, it] : team.issuer.Tasks()) {
				if (it.failed) ++r.failed;
				if (!it.done) continue;
				++r.done;
				r.duplicates += std::max(0, it.done_count - 1);
				const double rel = release.count(id) ? release.at(id) : it.dispatch;
				const double lat = *it.done - rel;
				latency_sum += lat;
				r.latency_max = std::max(r.latency_max, lat);
				last = std::max(last, *it.done);
			}
			if (r.done > 0) {
				r.makespan = last - first;
				r.latency_mean = latency_sum / r.done;
			}
			if (!r.completed) r.makespan = r.sim_time - first;  // a stalled run counts up to the limit
			std::set<std::string> leaf_robots;
			for (const auto& [id, it] : team.issuer.Leaves()) {
				++r.leaves;
				r.leaf_duplicates += std::max(0, it.done_count - 1);
				if (it.done) leaf_robots.insert(it.done_by);
			}
			long calls = 0;
			double total = 0.0;
			for (const auto& robot : team.robots) {
				r.distance_m += robot->sim.distance_flown;
				r.energy_wh += robot->sim.energy_used_wh;
				r.crashed = r.crashed || robot->sim.crashed;
				calls += robot->timing->calls;
				total += robot->timing->total_s;
				r.decision_worst_us = std::max(r.decision_worst_us, robot->timing->worst_s * 1e6);
				r.busy_robots += team.DoneBy(robot->layer->Self()) > 0 || leaf_robots.count(robot->layer->Self());
				if (auto* p = dynamic_cast<Algorithms::PlannerAllocator*>(&robot->timing->Inner())) {
					r.plan_calls += p->Replans();
					r.plan_worst_ms = std::max(r.plan_worst_ms, robot->timing->worst_s * 1e3);
				}
			}
			if (calls > 0) r.decision_mean_us = total / static_cast<double>(calls) * 1e6;
			if (const auto* planner = team.issuer.Planner()) {
				r.plan_calls = planner->calls;
				r.plan_worst_ms = planner->worst_s * 1e3;
			}
			r.dropped = team.air.dropped;
			if (team.air.messages > 0) r.delay_mean = team.air.delay_sum / static_cast<double>(team.air.messages);
			r.delay_max = team.air.delay_max;
			r.messages = team.air.messages;
			r.bytes = team.air.bytes;
			r.separation_breaches = team.metrics.separation_breaches;
			r.min_separation_m = team.metrics.min_separation > 1e8 ? -1.0 : team.metrics.min_separation;
			r.fence_exits = team.metrics.fence_exits;
			r.wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count();
			return r;
		}

		std::string CsvHeader() {
			return "set,family,dispatch,tasks,seed,condition,n,completed,done,failed,makespan_s,latency_mean_s,latency_max_s,"
			       "distance_m,energy_wh,duplicates,busy_robots,messages,bytes,decision_mean_us,decision_worst_us,plan_calls,"
			       "plan_worst_ms,separation_breaches,min_separation_m,fence_exits,crashed,sim_time_s,wall_s,leaves,leaf_duplicates,bitrate_bps,dropped,delay_mean_s,delay_max_s,"
			       "repulsion_lead_s,ranging_noise_m";
		}

		std::string CsvRow(const RunSpec& s, const RunResult& r) {
			std::ostringstream o;
			o << std::fixed << std::setprecision(3);
			o << s.set << "," << s.family << "," << s.dispatch << "," << r.tasks << "," << s.seed << "," << ConditionName(s.condition) << ","
			  << (IsSingle(s.condition) ? 1 : s.n) << "," << (r.completed ? 1 : 0) << "," << r.done << "," << r.failed << "," << r.makespan
			  << "," << r.latency_mean << "," << r.latency_max << "," << r.distance_m << "," << r.energy_wh << "," << r.duplicates << ","
			  << r.busy_robots << "," << r.messages << "," << r.bytes << "," << r.decision_mean_us << "," << r.decision_worst_us << ","
			  << r.plan_calls << "," << r.plan_worst_ms << "," << r.separation_breaches << "," << r.min_separation_m << "," << r.fence_exits
			  << "," << (r.crashed ? 1 : 0) << "," << r.sim_time << "," << r.wall_s << "," << r.leaves << "," << r.leaf_duplicates << "," << s.bitrate_bps << "," << r.dropped
			  << "," << r.delay_mean << "," << r.delay_max << "," << s.repulsion_lead << "," << s.ranging_noise;
			return o.str();
		}
	}
}
