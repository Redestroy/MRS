// WP8 tests (spec 12, plan §11 WP8): trees are split only where they can be split, complex units
// run on one robot and report their leaves, MRS-STA's active-task stack and kinship, the
// tree-task sets, and G-STA flying a mixed tree set.
#include <set>
#include <string>
#include <vector>
#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "mrs/algorithms/Allocator.h"
#include "mrs/algorithms/TaskTree.h"
#include "mrs/generators/Generators.h"
#include "mrs/protocol/Parser.h"
#include "mrs/sim/Experiment.h"
#include "mrs/sim/TaskSets.h"
#include "mrs/sim/Team.h"

using namespace MRS;
using Algorithms::PoolState;

namespace {
	Protocol::Record FirstTask(const std::string& text) {
		auto r = Protocol::Parse(text);
		REQUIRE_MESSAGE(r.Ok(), (r.Ok() ? "" : r.error->message));
		for (const auto& rec : r.document.records)
			if (rec.kind == 'T') return rec;
		FAIL("no task");
		return {};
	}

	Sim::RobotFiles Files() {
		return {Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsd"), Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsp"),
		        (Test::ExamplesDir() / "uav_behaviours.mrsb").string()};
	}

	struct Team : Sim::Team {
		explicit Team(int n) : Sim::Team(Files(), Config(n)) {}
		static Sim::TeamConfig Config(int n) {
			Sim::TeamConfig c;
			c.mission.homes = Sim::RowHomes(n);
			c.allocator = [](int) { return std::make_unique<Algorithms::StaAllocator>(); };
			return c;
		}
	};

	const std::string kLeaf = "T_A 0 1 0 C_1 C_2 A_1/\nC_1: C_N/\nC_2: C_N/\nA_1: A_N/\n";
	std::string Leaf(const std::string& slot) { return slot + ": " + kLeaf; }
	std::string BoundLeaf(const std::string& slot, const std::string& partner) {
		return slot + ": T_A 0 1 0 C_1 C_2 R_1 A_1/\nC_1: C_N/\nC_2: C_N/\nR_1: R_K " + partner + "/\nA_1: A_N/\n";
	}
}

// --- splittability (spec 12 §2.1) -------------------------------------------------------------

TEST_CASE("a waypoint chain is one unit; a T_L of chains splits into its chains") {
	Generators::ChainOptions o;
	const auto chain = Generators::ChainTask("op.1", {{0, 0, 10}, {10, 0, 10}, {20, 0, 10}}, o);
	const auto one = Algorithms::Decompose(chain);
	CHECK_FALSE(Algorithms::Splittable(one, "op.1"));
	CHECK(one.units == std::vector<std::string>{"op.1"});
	CHECK(one.leaves.size() == 3);
	for (const auto& leaf : one.leaves) CHECK(one.unit_of.at(leaf) == "op.1");

	Generators::CoverageOptions c;
	c.cells = 3;
	c.footprint = 8;
	const auto cover = Algorithms::Decompose(Generators::CoverageTask("op.2", {{0, 0, 0}, {40, 0, 0}, {40, 40, 0}, {0, 40, 0}}, c));
	CHECK(Algorithms::Splittable(cover, "op.2"));
	CHECK(cover.units == std::vector<std::string>{"op.2.1", "op.2.2", "op.2.3"});
	// The unit's record carries the subtree with the decomposer's ids.
	const auto& cell = cover.Node("op.2.1").task;
	CHECK(cell.code == "T_S");
	CHECK(cell.fields[0].s == "op.2.1");
	std::size_t inner = 0;
	for (const auto& f : cell.fields)
		if (f.type == Protocol::FieldType::Ref && cell.children.at(f.ref).kind == 'T') CHECK(cell.children.at(f.ref).fields[0].s == "op.2.1." + std::to_string(++inner));
	CHECK(inner == Algorithms::LeavesUnder(cover, "op.2.1").size());
}

TEST_CASE("a T_S splits unless every later child is bound to the one before it") {
	// Free leaves: anyone may take the second once the first is done.
	auto free = Algorithms::Decompose(FirstTask("@: MRS 0.1/\nT: T_S op.3 1 0 C_1 C_2 T_1 T_2/\nC_1: C_N/\nC_2: C_N/\n" + Leaf("T_1") + Leaf("T_2")));
	CHECK(Algorithms::Splittable(free, "op.3"));
	CHECK(free.units == std::vector<std::string>{"op.3.1", "op.3.2"});
	CHECK(free.Node("op.3.2").after == std::vector<std::string>{"op.3.1"});
	// Bound: land there, then release (R_K to the landing) is one robot's sequence.
	auto bound = Algorithms::Decompose(FirstTask("@: MRS 0.1/\nT: T_S op.4 1 0 C_1 C_2 T_1 T_2/\nC_1: C_N/\nC_2: C_N/\n" + Leaf("T_1") +
	                                             BoundLeaf("T_2", "op.4.1")));
	CHECK_FALSE(Algorithms::Splittable(bound, "op.4"));
	CHECK(bound.units == std::vector<std::string>{"op.4"});
	// Bound to a leaf that is not the previous child: splittable.
	auto loose = Algorithms::Decompose(FirstTask("@: MRS 0.1/\nT: T_S op.5 1 0 C_1 C_2 T_1 T_2 T_3/\nC_1: C_N/\nC_2: C_N/\n" + Leaf("T_1") +
	                                             Leaf("T_2") + BoundLeaf("T_3", "op.5.1")));
	CHECK(Algorithms::Splittable(loose, "op.5"));
	CHECK(loose.units.size() == 3);
	CHECK(loose.Node("op.5.3").affinity == "op.5.1");
}

TEST_CASE("a complex unit takes the constraints of its first leaf from outside it") {
	// T_S op.6 (start C_T) of a free leaf and a T_L of two bound pairs.
	const std::string text = "@: MRS 0.1/\nT: T_S op.6 2 0 C_1 C_2 T_1 T_2/\nC_1: C_T 1000/\nC_2: C_N/\n" + Leaf("T_1") +
	                         "T_2: T_L 0 1 0 0 C_1 C_2 T_1 T_2/\nC_1: C_N/\nC_2: C_N/\n"
	                         "T_1: T_S 0 0.5 0 C_1 C_2 T_1 T_2/\nC_1: C_N/\nC_2: C_N/\n" +
	                         Leaf("T_1") + BoundLeaf("T_2", "op.6.2.1.1") + "T_2: T_S 0 1 0 C_1 C_2 T_1 T_2/\nC_1: C_N/\nC_2: C_N/\n" + Leaf("T_1") +
	                         BoundLeaf("T_2", "op.6.2.2.1");
	const auto tree = Algorithms::Decompose(FirstTask(text));
	CHECK(tree.units == std::vector<std::string>{"op.6.1", "op.6.2.1", "op.6.2.2"});
	const auto& pair = tree.Node("op.6.2.1");
	CHECK(pair.unit);
	CHECK(pair.after == std::vector<std::string>{"op.6.1"});
	CHECK(pair.gates.empty());  // op.6's start gates op.6.1 only
	CHECK(pair.affinity.empty());  // its R_K points inside it
	CHECK(pair.task.fields[1].n == doctest::Approx(1.0));  // 2 × 1 × 0.5
	CHECK(tree.Node("op.6.1").gates.size() == 1);
	// A leaf inside a unit asks the caller for its state; the unit decides the tree.
	Algorithms::TreeState state(tree);
	const auto leaf = [](const std::string& id) { return id == "op.6.1" || id == "op.6.2.1" || id == "op.6.2.2" ? PoolState::DONE : PoolState::AVAILABLE; };
	CHECK(state.Of("op.6", leaf) == PoolState::DONE);
}

// --- MRS-STA (spec 12 §3) ---------------------------------------------------------------------

TEST_CASE("kinship is the share of a unit's path below the root it has in common with another") {
	Algorithms::TaskTree tree;
	tree.root = "op.1";
	CHECK(Algorithms::StaAllocator::Kinship(tree, "op.1.2.3", "op.1.2.1") == doctest::Approx(0.5));
	CHECK(Algorithms::StaAllocator::Kinship(tree, "op.1.2", "op.1.3") == doctest::Approx(0.0));
	CHECK(Algorithms::StaAllocator::Kinship(tree, "op.1.2.3.1", "op.1.2.3.2") == doctest::Approx(2.0 / 3.0));
	CHECK(Algorithms::StaAllocator::Kinship(tree, "op.1.2", "op.12.2") == doctest::Approx(0.0));
	CHECK(Algorithms::StaAllocator().Info().name == "MRS-STA");
}

TEST_CASE("a chain unit is flown by one robot that reports each leaf") {
	Generators::ChainOptions o;
	const auto chain = Generators::ChainTask("op.1", {{10, 0, 10}, {20, 0, 10}, {20, 10, 10}}, o);
	Team team(2);
	team.issuer.Schedule(1.0, {chain});
	REQUIRE(team.Run(300));
	const auto& root = team.issuer.Tasks().at("op.1");
	REQUIRE(root.done);
	std::set<std::string> by;
	for (const auto& entry : team.issuer.Leaves()) {
		const std::string& id = entry.first;
		const auto& leaf = entry.second;
		CAPTURE(id);
		REQUIRE(leaf.done);
		CHECK(leaf.done_count == 1);
		by.insert(leaf.done_by);
	}
	CHECK(by.size() == 1);
	CHECK(*by.begin() == root.done_by);
}

TEST_CASE("G-STA keeps a tree on the stack while it has units, then lets it go") {
	Generators::CoverageOptions c;
	c.cells = 4;
	c.footprint = 8;
	c.altitude = 12;
	const auto cover = Generators::CoverageTask("op.1", {{-20, 10, 0}, {20, 10, 0}, {20, 40, 0}, {-20, 40, 0}}, c);
	Team team(2);
	team.issuer.Schedule(1.0, {cover});
	bool stacked = false, done = false;
	for (int k = 0; k < 800 && !done; ++k) {
		done = team.Run(0.5);
		for (const auto& r : team.robots)
			if (auto* sta = dynamic_cast<Algorithms::StaAllocator*>(&r->timing->Inner()))
				stacked = stacked || (sta->Stack().size() == 1 && sta->Stack()[0] == "op.1");
	}
	REQUIRE(done);
	CHECK(stacked);
	team.Run(1.0, false);  // one more round of Select after the last unit
	for (const auto& r : team.robots)
		if (auto* sta = dynamic_cast<Algorithms::StaAllocator*>(&r->timing->Inner())) CHECK(sta->Stack().empty());
	std::set<std::string> cells_by;
	for (const auto& [id, u] : team.issuer.Units()) cells_by.insert(u.done_by);
	CHECK(cells_by.size() == 2);  // both robots took cells
}

// --- evaluation v2 (spec 12 §4) ---------------------------------------------------------------

TEST_CASE("tree-task sets are deterministic, parse and cycle the families when mixed") {
	Sim::TreeGenConfig c;
	c.seed = 4;
	const std::string a = Sim::GenerateTreeSet(c), b = Sim::GenerateTreeSet(c);
	CHECK(a == b);
	const auto release = Sim::ReleaseTimes(a);
	CHECK(release.size() == 3);
	CHECK(release.at("op.2") == doctest::Approx(21.0));
	auto parsed = Protocol::Parse(a);
	REQUIRE(parsed.Ok());
	std::vector<std::size_t> units;
	for (const auto& r : parsed.document.records)
		if (r.code == "L_D") units.push_back(Algorithms::Decompose(r.children.at(r.fields[1].ref)).units.size());
	CHECK(units == std::vector<std::size_t>{6, 6, 6});
	Sim::TreeFamily f;
	CHECK(Sim::ParseTreeFamily("perimeter", f));
	CHECK(f == Sim::TreeFamily::PERIMETER);
	CHECK_FALSE(Sim::ParseTreeFamily("orchard", f));
}

TEST_CASE("WP8 acceptance: G-STA splits a mixed tree set between three UAVs and beats one UAV") {
	Sim::TreeGenConfig tc;
	tc.seed = 2;
	Sim::RunSpec s;
	s.set = "tree/mixed-3x6/s2";
	s.timeline = Sim::GenerateTreeSet(tc);
	s.seed = 2;
	s.time_limit = 3000;
	s.mission.layer_step = 3.0;
	s.condition = Sim::Condition::G_STA;
	s.n = 3;
	const auto files = Files();
	const auto group = Sim::RunOne(files, s);
	CHECK(group.completed);
	CHECK(group.done == 3);
	CHECK(group.busy_robots == 3);
	CHECK(group.leaf_duplicates == 0);
	CHECK(group.fence_exits == 0);
	CHECK_FALSE(group.crashed);
	s.condition = Sim::Condition::S1;
	const auto single = Sim::RunOne(files, s);
	REQUIRE(single.completed);
	MESSAGE("G-STA n=3 makespan ", group.makespan, " s, S1 ", single.makespan, " s");
	CHECK(group.makespan < single.makespan * 0.6);
}
