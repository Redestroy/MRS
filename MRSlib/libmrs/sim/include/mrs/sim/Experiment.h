#pragma once
// One evaluation run (plan §10, spec 10 §5): a task set flown by one condition with one seed
// in the test simulator, and the metrics it gives.
#include <cstdint>
#include <string>

#include "mrs/algorithms/Allocator.h"
#include "mrs/algorithms/Cbba.h"
#include "mrs/algorithms/Ldta2.h"
#include "mrs/algorithms/Planner.h"
#include "mrs/sim/Team.h"

namespace MRS {
	namespace Sim {
		// The conditions of plan §10.2 that WP6 covers, G-STA (spec 12 §4), G-CBBA and G-LDTA2 (spec 13).
		enum class Condition { S1, S1_ORACLE, G_RTA, G_RTA_X, G_C, G_STA, G_CBBA, G_LDTA2 };
		const char* ConditionName(Condition c);  // "S1", "S1*", "G-RTA", "G-RTA-X", "G-C", "G-STA", "G-CBBA", "G-LDTA2"
		bool ParseCondition(const std::string& s, Condition& c);
		bool IsSingle(Condition c);

		struct RunSpec {
			std::string set;          // a name for the task set
			std::string family = "ported", dispatch = "even";
			std::string timeline;     // .mrsl text
			Condition condition = Condition::G_RTA;
			int n = 5;                // robots (1 for S1 and S1*)
			std::uint64_t seed = 1;   // home order and GPS noise (spec 10 §3.4)
			double time_limit = 4000.0;  // s of simulated time before the run counts as stalled
			MissionSpec mission;      // homes are filled in from n and the seed
			double home_spacing = 4.0;
			double gps_noise = 0.05;  // m
			double battery_wh = 200.0;  // no battery swaps in v1 (spec 10 §5)
			Algorithms::RtaConfig rta;
			Algorithms::StaConfig sta;
			Algorithms::CbbaConfig cbba;
			double bitrate_bps = 0.0;  // the shared channel; 0: no limit (spec 13 §3)
			Algorithms::SpatialSizeConfig size;
			Algorithms::TravelModel travel;  // its mode follows the condition
		};

		struct RunResult {
			bool completed = false;   // every task done or failed before the time limit
			int tasks = 0, done = 0, failed = 0;
			double makespan = 0.0;    // first release to last completion (s)
			double latency_mean = 0.0, latency_max = 0.0;  // release to completion (s)
			double distance_m = 0.0, energy_wh = 0.0;      // all robots
			int duplicates = 0;       // M_DONE beyond the first, summed over tasks
			int leaves = 0;           // leaves of split trees (spec 12 §4)
			int leaf_duplicates = 0;  // M_DONE beyond the first, summed over those leaves
			long dropped = 0;           // messages lost on a limited channel (spec 13 §3)
			double delay_mean = 0.0, delay_max = 0.0;  // s, queueing on the channel
			int busy_robots = 0;      // robots that completed a task
			long messages = 0, bytes = 0;
			double decision_mean_us = 0.0, decision_worst_us = 0.0;  // robot-side Select
			long plan_calls = 0;      // central replans (G-C) or own replans (S1, S1*)
			double plan_worst_ms = 0.0;
			long separation_breaches = 0;
			double min_separation_m = 0.0;
			long fence_exits = 0;
			bool crashed = false;
			double sim_time = 0.0, wall_s = 0.0;
		};

		RunResult RunOne(const RobotFiles& files, const RunSpec& spec);

		std::string CsvHeader();
		std::string CsvRow(const RunSpec& spec, const RunResult& r);
	}
}
