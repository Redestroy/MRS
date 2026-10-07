// WP6 tests (spec 10, plan §11 WP6): the route planner, the task-set generators, the oracle form
// and the evaluation runs of each condition.
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "mrs/algorithms/Planner.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"
#include "mrs/sim/Experiment.h"
#include "mrs/sim/TaskSets.h"

using namespace MRS;
using Algorithms::PlanRobot;
using Algorithms::PlanTask;

namespace {
	std::filesystem::path TaskSets2021() { return Test::ExamplesDir() / ".." / ".." / "experiments" / "uav_spatial" / "tasksets_2021"; }

	Sim::RobotFiles Files() {
		return {Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsd"), Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsp"),
		        (Test::ExamplesDir() / "uav_behaviours.mrsb").string()};
	}

	PlanTask PTask(const std::string& id, double x, double y, double z, double release = 0.0, double hold = 6.0) {
		PlanTask t;
		t.g.id = id;
		t.g.has_target = true;
		t.g.target = {x, y, z};
		t.g.release = release;
		t.g.hold = hold;
		return t;
	}

	PlanRobot PRobot(const std::string& name, double x, double y, double z, double layer = 20.0) {
		PlanRobot r;
		r.name = name;
		r.pos = {x, y, z};
		r.layer = layer;
		return r;
	}

	Algorithms::PlannerConfig Direct() {
		Algorithms::PlannerConfig c;
		c.travel.mode = Algorithms::FlightMode::DIRECT;
		c.travel.settle = 0.0;
		return c;
	}

	const Protocol::Record& FirstTask(const Protocol::Document& d) {
		for (const auto& r : d.records)
			if (r.code == "L_D") return r.children.at(r.fields.at(1).ref);
		throw std::runtime_error("no L_D record");
	}
}

TEST_CASE("the travel model: layered, altitude-first and direct legs") {
	Algorithms::TravelModel m;  // v_xy 7.5, v_z 2, cruise 20, switch radius 3
	const std::array<double, 3> a{0, 0, 15}, b{75, 0, 15};
	m.mode = Algorithms::FlightMode::LAYERED;
	CHECK(m.Time(a, b, 26.0) == doctest::Approx(5.5 + 10.0 + 5.5));
	m.mode = Algorithms::FlightMode::ALTITUDE_FIRST;
	CHECK(m.Time(a, b, 26.0) == doctest::Approx(2.5 + 10.0 + 2.5));  // cruises at 20 m
	CHECK(m.Time(a, {75, 0, 25}, 26.0) == doctest::Approx(5.0 + 10.0));  // above the cruise altitude
	m.mode = Algorithms::FlightMode::DIRECT;
	CHECK(m.Time(a, b, 26.0) == doctest::Approx(10.0));
	m.mode = Algorithms::FlightMode::LAYERED;
	CHECK(m.Time(a, {2, 0, 19}, 26.0) == doctest::Approx(2.0));  // inside the switch radius: straight
}

TEST_CASE("task geometry comes from the first C_P3, the latest C_T and the waits") {
	auto doc = Protocol::Parse(
	    "T: T_S op.4 1 0 C_1 C_2 T_1 T_2/\nC_1: C_N/\nC_2: C_N/\n"
	    "T_1: T_A op.4 1 0 C_1 C_2 A_1/\nC_1: C_P3 10 -20 15 1 0.5 0 -1/\nC_2: C_N/\nA_1: A_N/\n"
	    "T_2: T_A op.4 1 0 C_1 C_2 A_1..3/\nC_1: C_T 42500/\nC_2: C_N/\nA_1: A_W 3/\nA_2: A_W 2.5/\nA_3: A_N/\n");
	REQUIRE(doc.Ok());
	const auto g = Algorithms::GeometryOf(doc.document.records.at(0));
	CHECK(g.id == "op.4");
	REQUIRE(g.has_target);
	CHECK(g.target[0] == doctest::Approx(10.0));
	CHECK(g.target[1] == doctest::Approx(-20.0));
	CHECK(g.target[2] == doctest::Approx(15.0));
	CHECK(g.release == doctest::Approx(42.5));
	CHECK(g.hold == doctest::Approx(5.5));
}

TEST_CASE("the planner orders a route along a line and splits work between robots") {
	const auto c = Direct();
	SUBCASE("one robot visits the points in order along the line") {
		const auto plan = Algorithms::PlanRoutes({PRobot("r1", 0, 0, 10)},
		                                         {PTask("c", 30, 0, 10), PTask("a", 10, 0, 10), PTask("b", 20, 0, 10)}, 0.0, c);
		CHECK(plan.routes.at("r1") == std::vector<std::string>{"a", "b", "c"});
		CHECK(plan.makespan == doctest::Approx(30.0 / 7.5 + 3 * 6.0));
		CHECK(plan.unassigned.empty());
	}
	SUBCASE("two robots at opposite ends take the tasks on their own side") {
		const auto plan = Algorithms::PlanRoutes({PRobot("r1", -50, 0, 10), PRobot("r2", 50, 0, 10)},
		                                         {PTask("w1", -40, 0, 10), PTask("e1", 40, 0, 10), PTask("w2", -30, 0, 10), PTask("e2", 30, 0, 10)},
		                                         0.0, c);
		CHECK(plan.routes.at("r1") == std::vector<std::string>{"w1", "w2"});
		CHECK(plan.routes.at("r2") == std::vector<std::string>{"e1", "e2"});
	}
	SUBCASE("a robot does not start a task before its release") {
		const auto plan = Algorithms::PlanRoutes({PRobot("r1", 0, 0, 10)}, {PTask("late", 7.5, 0, 10, 50.0)}, 10.0, c);
		CHECK(plan.finish.at("late") == doctest::Approx(56.0));
		CHECK(plan.makespan == doctest::Approx(46.0));  // from `now`
	}
	SUBCASE("excluded robots are skipped, and a task no robot may do is unassigned") {
		PlanTask only_r2 = PTask("x", -45, 0, 10);
		only_r2.excluded = {"r1"};
		PlanTask none = PTask("y", 0, 0, 10);
		none.excluded = {"r1", "r2"};
		const auto plan = Algorithms::PlanRoutes({PRobot("r1", -50, 0, 10), PRobot("r2", 50, 0, 10)}, {only_r2, none}, 0.0, c);
		CHECK(plan.routes.at("r1").empty());
		CHECK(plan.routes.at("r2") == std::vector<std::string>{"x"});
		CHECK(plan.unassigned == std::vector<std::string>{"y"});
	}
	SUBCASE("a fixed task stays first on its robot's route") {
		PlanRobot r1 = PRobot("r1", 0, 0, 10);
		r1.fixed = "far";
		const auto plan = Algorithms::PlanRoutes({r1, PRobot("r2", 60, 0, 10)}, {PTask("near", 5, 0, 10), PTask("far", 55, 0, 10)}, 0.0, c);
		REQUIRE_FALSE(plan.routes.at("r1").empty());
		CHECK(plan.routes.at("r1").front() == "far");
	}
	SUBCASE("the makespan objective takes the short detour first; the completion objective serves the cluster first") {
		const std::vector<PlanTask> tasks{PTask("a", -7.5, 0, 10), PTask("b1", 15, 0, 10), PTask("b2", 15, 0, 10), PTask("b3", 15, 0, 10)};
		const auto by_makespan = Algorithms::PlanRoutes({PRobot("r1", 0, 0, 10)}, tasks, 0.0, c);
		CHECK(by_makespan.routes.at("r1").front() == "a");
		CHECK(by_makespan.makespan == doctest::Approx(28.0));
		auto cc = c;
		cc.objective = Algorithms::PlanObjective::COMPLETION;
		const auto by_completion = Algorithms::PlanRoutes({PRobot("r1", 0, 0, 10)}, tasks, 0.0, cc);
		CHECK(by_completion.routes.at("r1").back() == "a");
		CHECK(by_completion.makespan == doctest::Approx(29.0));
	}
	SUBCASE("equal inputs give equal plans") {
		std::vector<PlanTask> tasks;
		for (int k = 0; k < 20; ++k) tasks.push_back(PTask("t" + std::to_string(k), std::sin(k) * 50, std::cos(3 * k) * 50, 5 + k % 4 * 5, k * 2.0));
		const std::vector<PlanRobot> robots{PRobot("r1", 0, 0, 0), PRobot("r2", 4, 0, 0, 23), PRobot("r3", 8, 0, 0, 26)};
		const auto a = Algorithms::PlanRoutes(robots, tasks, 0.0, {}), b = Algorithms::PlanRoutes(robots, tasks, 0.0, {});
		CHECK(a.routes == b.routes);
		CHECK(a.makespan == b.makespan);
		std::set<std::string> seen;
		for (const auto& [robot, route] : a.routes) seen.insert(route.begin(), route.end());
		CHECK(seen.size() == tasks.size());
	}
}

TEST_CASE("generated task sets are deterministic, parse, and follow their dispatch") {
	for (auto family : {Sim::Family::GRID, Sim::Family::RADIAL, Sim::Family::CLUSTER, Sim::Family::MULTICLUSTER, Sim::Family::RANDOM}) {
		Sim::GenConfig c;
		c.family = family;
		c.tasks = 20;
		c.seed = 7;
		const std::string a = Sim::GenerateTaskSet(c);
		CHECK(a == Sim::GenerateTaskSet(c));
		auto parsed = Protocol::Parse(a);
		REQUIRE_MESSAGE(parsed.Ok(), Sim::FamilyName(family));
		const auto release = Sim::ReleaseTimes(a);
		CHECK(release.size() == 20);
		for (const auto& [id, t] : release) {
			CHECK(t >= c.first);
			CHECK(t <= c.first + 19 * c.interval + 1e-9);
		}
		// Every target lies inside the area and the altitude band.
		for (const auto& r : parsed.document.records) {
			if (r.code != "L_D") continue;
			const auto g = Algorithms::GeometryOf(r.children.at(r.fields.at(1).ref));
			REQUIRE(g.has_target);
			CHECK(std::fabs(g.target[0]) <= c.half_size);
			CHECK(std::fabs(g.target[1]) <= c.half_size);
			CHECK(g.target[2] >= c.z_min);
			CHECK(g.target[2] <= c.z_max);
		}
		c.seed = 8;
		if (family != Sim::Family::GRID) CHECK(a != Sim::GenerateTaskSet(c));
	}
	Sim::GenConfig s;
	s.dispatch = Sim::Dispatch::STATIC;
	s.tasks = 10;
	for (const auto& [id, t] : Sim::ReleaseTimes(Sim::GenerateTaskSet(s))) CHECK(t == doctest::Approx(s.first));
}

TEST_CASE("the oracle form sends every task at 0 and keeps its release in a C_T") {
	const std::string timeline = Test::ReadFile(TaskSets2021() / "taskset1.mrsl");
	const auto release = Sim::ReleaseTimes(timeline);
	const std::string oracle = Sim::OracleTimeline(timeline);
	auto parsed = Protocol::Parse(oracle);
	REQUIRE(parsed.Ok());
	int tasks = 0;
	for (const auto& r : parsed.document.records) {
		if (r.code != "L_D") continue;
		++tasks;
		CHECK(r.fields.at(0).n == doctest::Approx(0.0));
		const auto& t = r.children.at(r.fields.at(1).ref);
		CHECK(t.code == "T_S");
		const auto g = Algorithms::GeometryOf(t);
		REQUIRE(release.count(g.id));
		CHECK(g.release == doctest::Approx(release.at(g.id)));
		CHECK(g.has_target);
		CHECK(g.hold == doctest::Approx(6.0));
	}
	CHECK(tasks == static_cast<int>(release.size()));
	// The task keeps its geometry through the oracle form.
	const auto plain = Algorithms::GeometryOf(FirstTask(Protocol::Parse(timeline).document));
	const auto wrapped = Algorithms::GeometryOf(FirstTask(parsed.document));
	CHECK(plain.target == wrapped.target);
}

TEST_CASE("M_PLAN carries a revision and a route") {
	auto parsed = Protocol::Parse("M: M_PLAN 0.1 m1 op r2 76 160.3 4 2 op.18 op.21/\n");
	REQUIRE(parsed.Ok());
	CHECK(Protocol::Write(parsed.document) == "M: M_PLAN 0.1 m1 op r2 76 160.3 4 2 op.18 op.21/\n");
	CHECK_FALSE(Protocol::Parse("M: M_PLAN 0.1 m1 op r2 76 160.3 4 3 op.18 op.21/\n").Ok());  // count and ids disagree
}

TEST_CASE("every condition completes a ported 2021 task set; the groups beat one UAV") {
	const auto files = Files();
	Sim::RunSpec s;
	s.set = "ported/taskset1";
	s.timeline = Test::ReadFile(TaskSets2021() / "taskset1.mrsl");
	s.mission.layer_step = 3.0;
	s.seed = 1;
	std::map<Sim::Condition, Sim::RunResult> r;
	for (auto c : {Sim::Condition::S1, Sim::Condition::S1_ORACLE, Sim::Condition::G_RTA, Sim::Condition::G_RTA_X, Sim::Condition::G_C}) {
		s.condition = c;
		s.n = 5;
		r[c] = Sim::RunOne(files, s);
		INFO(Sim::ConditionName(c));
		CHECK(r[c].completed);
		CHECK(r[c].done == 15);
		CHECK(r[c].failed == 0);
		CHECK_FALSE(r[c].crashed);
		CHECK(r[c].fence_exits == 0);
	}
	// Knowing the whole timeline does not make one UAV slower.
	CHECK(r[Sim::Condition::S1_ORACLE].makespan <= r[Sim::Condition::S1].makespan + 1.0);
	for (auto c : {Sim::Condition::G_RTA, Sim::Condition::G_RTA_X, Sim::Condition::G_C})
		CHECK(r[c].makespan < 0.6 * r[Sim::Condition::S1].makespan);
	// Central plans and claims keep the robots off each other's tasks.
	CHECK(r[Sim::Condition::G_C].duplicates == 0);
	CHECK(r[Sim::Condition::G_RTA_X].duplicates == 0);
	CHECK(r[Sim::Condition::G_C].plan_calls > 0);
	CHECK(r[Sim::Condition::S1].busy_robots == 1);
}
