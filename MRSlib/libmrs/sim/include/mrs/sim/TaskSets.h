#pragma once
// Task sets for the evaluation (plan §10.3, spec 10 §3): generators for the new 3D sets, the
// oracle form of a timeline, and the release times of a timeline.
#include <cstdint>
#include <map>
#include <string>

namespace MRS {
	namespace Sim {
		enum class Family { GRID, RADIAL, CLUSTER, MULTICLUSTER, RANDOM };
		enum class Dispatch { STATIC, EVEN, CLUSTERED, RANDOM };
		const char* FamilyName(Family f);
		const char* DispatchName(Dispatch d);
		bool ParseFamily(const std::string& s, Family& f);
		bool ParseDispatch(const std::string& s, Dispatch& d);

		struct GenConfig {
			Family family = Family::RANDOM;
			Dispatch dispatch = Dispatch::EVEN;
			int tasks = 30;
			std::uint64_t seed = 1;
			double half_size = 60.0;          // m: tasks lie in [-half_size, half_size]² around the origin
			double z_min = 5.0, z_max = 25.0;  // m
			double interval = 5.0;            // s between tasks (EVEN), and the mean gap (RANDOM)
			int burst = 5;                    // tasks per burst (CLUSTERED)
			double burst_gap = 25.0;          // s between bursts (CLUSTERED)
			double first = 1.0;               // s: the first dispatch (STATIC: all)
			double tol_xy = 1.0, tol_z = 0.5;
			std::string issuer = "op";
		};

		// A timeline (.mrsl) of LED tasks like the ported 2021 sets (spec 09 §7): fly to a 3D
		// point, LED on for 3 s, off for 3 s. Deterministic for a seed on every platform.
		std::string GenerateTaskSet(const GenConfig& c);

		// The oracle form (spec 10 §3.3): every task is sent at time 0 as a T_S whose first child
		// flies to the task's target and whose second child waits for its release (C_T) and
		// then does the task. Throws std::runtime_error on a parse error.
		std::string OracleTimeline(const std::string& timeline);

		// Task id -> release time (s) of every top-level task in a timeline.
		std::map<std::string, double> ReleaseTimes(const std::string& timeline);

		// --- tree-task sets (spec 12 §4) -------------------------------------------------------

		enum class TreeFamily { COVERAGE, PERIMETER, SEARCH, MIXED };
		const char* TreeFamilyName(TreeFamily f);  // "coverage", "perimeter", "search", "mixed"
		bool ParseTreeFamily(const std::string& s, TreeFamily& f);

		struct TreeGenConfig {
			TreeFamily family = TreeFamily::MIXED;
			int trees = 3;             // tree tasks in the set
			int parts = 6;             // coverage cells, perimeter arcs, points of interest per search
			std::uint64_t seed = 1;
			double half_size = 60.0;   // m: every waypoint lies in [-half_size, half_size]²
			double interval = 20.0;    // s between tree releases
			double first = 1.0;        // s: the first release
			std::string issuer = "op";
		};

		// A timeline (.mrsl) of tree tasks from the WP7 generators: coverage of a rotated
		// rectangle (cells of boustrophedon chains), the perimeter of one (arcs), or a spiral search
		// around points of interest. MIXED cycles coverage, perimeter, search. Deterministic for a
		// seed on every platform.
		std::string GenerateTreeSet(const TreeGenConfig& c);
	}
}
