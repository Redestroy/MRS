// WP9 tests (spec 13): CBBA's bid rule and consensus, the bitrate-limited channel and the
// communication budget, and G-CBBA flying generated sets with and without a bitrate limit.
#include <string>
#include <vector>
#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "mrs/algorithms/Cbba.h"
#include "mrs/sim/Experiment.h"
#include "mrs/sim/TaskSets.h"
#include "mrs/sim/Team.h"

using namespace MRS;

namespace {
	Sim::RobotFiles Files() {
		return {Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsd"), Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsp"),
		        (Test::ExamplesDir() / "uav_behaviours.mrsb").string()};
	}

	Sim::RunSpec Spec(Sim::Condition c, int n, double bitrate, Sim::Family f = Sim::Family::CLUSTER, Sim::Dispatch d = Sim::Dispatch::STATIC) {
		Sim::GenConfig g;
		g.family = f;
		g.dispatch = d;
		g.tasks = 20;
		g.seed = 3;
		Sim::RunSpec s;
		s.set = "gen/test";
		s.timeline = Sim::GenerateTaskSet(g);
		s.seed = 3;
		s.condition = c;
		s.n = n;
		s.bitrate_bps = bitrate;
		s.time_limit = 2000;
		s.mission.layer_step = 3.0;
		return s;
	}
}

TEST_CASE("a CBBA bid beats the entry when higher, or equal from a lower robot number") {
	using Bid = Algorithms::CbbaAllocator::Bid;
	CHECK(Algorithms::CbbaAllocator::Beats(1.0, "r2", Bid{}));
	CHECK_FALSE(Algorithms::CbbaAllocator::Beats(0.0, "r2", Bid{}));
	CHECK(Algorithms::CbbaAllocator::Beats(2.0, "r3", Bid{1.0, "r1", 0}));
	CHECK_FALSE(Algorithms::CbbaAllocator::Beats(1.0, "r3", Bid{1.0, "r1", 0}));
	CHECK(Algorithms::CbbaAllocator::Beats(1.0, "r1", Bid{1.0, "r3", 0}));
	CHECK_FALSE(Algorithms::CbbaAllocator::Beats(5.0, "", Bid{}));
	CHECK(Algorithms::CbbaAllocator().Info().name == "CBBA");
}

TEST_CASE("the limited channel carries messages at its bitrate and drops the stale ones") {
	Sim::Air air;
	air.bitrate_bps = 8000.0;  // 1000 bytes per second
	air.max_delay = 2.0;
	const std::string msg(500, 'x');
	for (int i = 0; i < 4; ++i) air.Operator().Send(msg, "all");
	// No robot is on the channel, so count deliveries instead of inboxes.
	air.Deliver(0.0);
	CHECK(air.messages == 0);  // no credit yet, but the head message may save up for itself
	air.Deliver(0.5);
	CHECK(air.messages == 1);
	air.Deliver(1.0);
	CHECK(air.messages == 2);  // the third is now on the air
	air.Deliver(3.0);
	CHECK(air.messages == 3);  // the third waited 3 s but was on the air, so it is not dropped
	CHECK(air.dropped == 1);   // the fourth waited 3 s behind it, longer than max_delay
	CHECK(air.delay_max == doctest::Approx(3.0));
}

TEST_CASE("G-CBBA completes a static cluster set with every robot busy and no duplicates") {
	const auto files = Files();
	const auto r = Sim::RunOne(files, Spec(Sim::Condition::G_CBBA, 3, 0.0));
	CHECK(r.completed);
	CHECK(r.done == 20);
	CHECK(r.duplicates == 0);
	CHECK(r.busy_robots == 3);
	CHECK(r.fence_exits == 0);
	CHECK_FALSE(r.crashed);
	const auto single = Sim::RunOne(files, Spec(Sim::Condition::S1, 1, 0.0));
	REQUIRE(single.completed);
	MESSAGE("G-CBBA n=3 makespan ", r.makespan, " s, S1 ", single.makespan, " s");
	CHECK(r.makespan < single.makespan * 0.6);
}

TEST_CASE("on a 9.6 kbit/s channel G-CBBA and G-RTA-X still finish, with messages queued") {
	const auto files = Files();
	for (auto c : {Sim::Condition::G_CBBA, Sim::Condition::G_RTA_X}) {
		CAPTURE(Sim::ConditionName(c));
		const auto r = Sim::RunOne(files, Spec(c, 3, 9600.0, Sim::Family::RANDOM, Sim::Dispatch::EVEN));
		CHECK(r.completed);
		CHECK(r.done == 20);
		CHECK(r.delay_mean > 0.0);
		const auto free = Sim::RunOne(files, Spec(c, 3, 0.0, Sim::Family::RANDOM, Sim::Dispatch::EVEN));
		CHECK(r.messages < free.messages);  // heartbeats and bids slow down to fit the budget
	}
}

// --- LDTA² (spec 13 §5) ------------------------------------------------------------------------

TEST_CASE("LDTA2 wants robots per type in proportion to open tasks, adding up to the team") {
	using A = Algorithms::Ldta2Allocator;
	auto c = A::Wanted({{"op.1", 6}, {"op.2", 3}, {"op.3", 1}}, 5);
	CHECK(c.at("op.1") == 3);
	CHECK(c.at("op.2") == 2);  // 1.5 and 0.5 tie on the remainder; the first type gets the spare robot
	CHECK(c.at("op.3") == 0);
	// Never more robots than open tasks; the rest is left over.
	c = A::Wanted({{"op.1", 1}, {"op.2", 1}}, 5);
	CHECK(c.at("op.1") == 1);
	CHECK(c.at("op.2") == 1);
	CHECK(A::Wanted({}, 3).empty());
	CHECK(A().Info().name == "LDTA2");
}

TEST_CASE("G-LDTA2 spreads three UAVs over overlapping trees and finishes them") {
	Sim::TreeGenConfig tc;
	tc.trees = 6;
	tc.parts = 4;
	tc.interval = 5;
	tc.seed = 1;
	Sim::RunSpec s;
	s.set = "tree/test";
	s.timeline = Sim::GenerateTreeSet(tc);
	s.seed = 1;
	s.condition = Sim::Condition::G_LDTA2;
	s.n = 3;
	s.time_limit = 3000;
	s.mission.layer_step = 3.0;
	const auto r = Sim::RunOne(Files(), s);
	CHECK(r.completed);
	CHECK(r.done == 6);
	CHECK(r.busy_robots == 3);
	CHECK(r.leaf_duplicates == 0);
	CHECK_FALSE(r.crashed);
}
