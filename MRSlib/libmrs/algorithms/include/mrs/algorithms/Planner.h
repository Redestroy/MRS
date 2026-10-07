#pragma once
// The planner baseline (plan §10.2, spec 10 §2): a route planner for one or several UAVs, the
// single-UAV planner allocator (S1, S1*) and the central planner with its plan follower (G-C).
#include <array>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mrs/algorithms/Allocator.h"
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Algorithms {
		// What the planner needs to know about a task, read from its record.
		struct TaskGeometry {
			std::string id;
			bool has_target = false;
			std::array<double, 3> target{};  // the first C_P3 in the tree
			double release = 0.0;            // s: the latest C_T in the tree (0 without one)
			double hold = 0.0;               // s: its A_W and A_HD durations
		};
		TaskGeometry GeometryOf(const Protocol::Record& task);

		// How a UAV gets from one point to another (the fly-to behaviours of spec 08 §8).
		enum class FlightMode { LAYERED, ALTITUDE_FIRST, DIRECT };
		const char* FlightModeName(FlightMode m);

		// Defaults measured on the test simulator with the spec 08 gains (spec 10 §2.1).
		struct TravelModel {
			FlightMode mode = FlightMode::LAYERED;
			double v_xy = 7.5;           // m/s across, mean over a leg
			double v_z = 2.0;            // m/s up or down
			double settle = 1.5;         // s per task to settle at the target
			double takeoff = 0.0;        // s added when the leg starts on the ground
			double switch_radius = 3.0;  // m: a shorter leg flies straight to the target
			double cruise_alt = 20.0;    // m: the cruise altitude of flyto.altitude_first
			// Time from a to b; `layer` is the robot's layer altitude (LAYERED only).
			double Time(const std::array<double, 3>& a, const std::array<double, 3>& b, double layer) const;
		};

		struct PlanRobot {
			std::string name;
			std::array<double, 3> pos{};
			double layer = 20.0;  // m
			std::string fixed;    // a task that stays first on its route (the one it is doing)
		};

		struct PlanTask {
			TaskGeometry g;
			std::set<std::string> excluded;  // robots that cannot do it (static dumps)
		};

		// What the planner minimises first; the other measure breaks ties.
		enum class PlanObjective {
			MAKESPAN,    // the last planned completion
			COMPLETION,  // the sum of planned completion times
		};
		const char* PlanObjectiveName(PlanObjective o);

		struct PlannerConfig {
			TravelModel travel;
			PlanObjective objective = PlanObjective::MAKESPAN;
			int improve_rounds = 30;  // local search passes (relocate, swap, 2-opt)
		};

		struct Plan {
			std::map<std::string, std::vector<std::string>> routes;  // by robot; every robot has an entry
			std::map<std::string, double> finish;                    // planned completion time per task
			double makespan = 0.0;                                   // planned, from `now`
			std::vector<std::string> unassigned;                     // tasks no robot may take
		};

		// Plans routes that minimise the planned makespan, then the sum of completion times:
		// cheapest insertion in release order, then relocate, swap and 2-opt moves until none
		// improves. A task is not started before its release. Deterministic for equal inputs.
		Plan PlanRoutes(const std::vector<PlanRobot>& robots, const std::vector<PlanTask>& tasks, double now, const PlannerConfig& c);

		// S1 and S1* (plan §10.2): the robot plans one route over every task it knows and
		// replans when a task arrives. With the whole timeline sent at the start (an oracle
		// timeline, spec 10 §3) it is S1*.
		class PlannerAllocator : public IAllocator {
		public:
			explicit PlannerAllocator(PlannerConfig c = {}) : c_(c) {}
			void OnTaskReceived(const std::string& task, double t) override;
			void OnTaskFinished(const std::string& task, PoolState s, double t) override;
			Decision Select(const Environment::Worldview& w, const CurrentTask& current, double t) override;
			AllocatorInfo Info() const override { return {"PLAN-1", false, false}; }
			const std::vector<std::string>& Route() const { return route_; }
			long Replans() const { return replans_; }

		private:
			PlannerConfig c_;
			std::vector<std::string> route_;
			bool dirty_ = false;
			long replans_ = 0;
		};

		// G-C (plan §10.2): routes come from a central planner in M_PLAN messages (spec 06 §4).
		// The robot does the first task of its latest route that it has received.
		class PlanFollowerAllocator : public IAllocator {
		public:
			void OnMessage(const Comm::Message& m, double t) override;
			void OnTaskFinished(const std::string& task, PoolState s, double t) override;
			Decision Select(const Environment::Worldview& w, const CurrentTask& current, double t) override;
			AllocatorInfo Info() const override { return {"PLAN-C", false, true}; }
			const std::vector<std::string>& Route() const { return route_; }

		private:
			std::vector<std::string> route_;
			long revision_ = -1;
		};

		// The planner the issuer runs for G-C. It follows the robots through M_STATE and M_DUMP
		// and plans every open task when tasks arrive.
		class CentralPlanner {
		public:
			// Robots and their layers come from the mission header (spec 06 §6).
			CentralPlanner(const Protocol::Record& mission_header, PlannerConfig c = {});
			void OnMessage(const Comm::Message& m, double t);
			// Plans the open tasks (dispatched, not finished), given in dispatch order.
			Plan Replan(const std::vector<Protocol::Record>& open, double t);
			long Revision() const { return revision_; }
			long calls = 0;
			double worst_s = 0.0, total_s = 0.0;  // planning time

		private:
			struct Known {
				std::array<double, 3> pos{};
				double layer = 20.0;
				std::string current;
			};
			PlannerConfig c_;
			std::map<std::string, Known> robots_;
			std::map<std::string, std::set<std::string>> excluded_;  // task -> robots
			long revision_ = 0;
		};
	}
}
