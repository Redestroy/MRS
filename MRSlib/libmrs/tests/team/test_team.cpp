// WP5 team tests (spec 09, plan §11 WP5 acceptance): messages, the task pool, MRS-RTA and the
// task issuer, with five UAVs in the test simulator sharing one radio channel.
#include <cmath>
#include <deque>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "mrs/algorithms/MrsLayer.h"
#include "mrs/algorithms/TaskIssuer.h"
#include "mrs/device/Robot.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/protocol/Parser.h"
#include "mrs/sim/Team.h"

using namespace MRS;

namespace {
	std::filesystem::path TaskSets2021() { return Test::ExamplesDir() / ".." / ".." / "experiments" / "uav_spatial" / "tasksets_2021"; }

	Sim::MissionSpec Mission(int n) {
		Sim::MissionSpec m;  // fence ±100 m, 60 m high; layers from 20 m every 5 m; claim grace 120 s
		m.homes = Sim::RowHomes(n);
		return m;
	}

	Protocol::Record Header(int n) { return Mission(n).Header(); }

	Sim::RobotFiles Files() {
		return {Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsd"), Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsp"),
		        (Test::ExamplesDir() / "uav_behaviours.mrsb").string()};
	}

	Sim::TeamConfig Config(int n, Algorithms::RtaConfig rta, std::set<int> without_leds) {
		Sim::TeamConfig c;
		c.mission = Mission(n);
		c.without_leds = std::move(without_leds);
		c.allocator = [rta](int) { return std::make_unique<Algorithms::RtaAllocator>(rta); };
		return c;
	}

	// Five UAVs in a row at y = -5, 4 m apart, by default with open MRS-RTA.
	struct Team : Sim::Team {
		explicit Team(int n, Algorithms::RtaConfig rta = {}, std::set<int> without_leds = {})
		    : Sim::Team(Files(), Config(n, rta, std::move(without_leds))) {}
	};

	// One end of a two-way link: what one end sends, the other polls.
	class Pipe : public Comm::ITransport {
	public:
		void Connect(Pipe* other) { other_ = other; }
		bool Send(const std::string& text, const std::string&) override {
			other_->queue.push_back(text);
			return true;
		}
		std::vector<std::string> Poll() override {
			auto out = queue;
			queue.clear();
			return out;
		}
		std::vector<std::string> queue;

	private:
		Pipe* other_ = nullptr;
	};

	// A loopback transport for messenger tests.
	class Loop : public Comm::ITransport {
	public:
		bool Send(const std::string& text, const std::string&) override {
			queue.push_back(text);
			return true;
		}
		std::vector<std::string> Poll() override {
			auto out = queue;
			queue.clear();
			return out;
		}
		std::vector<std::string> queue;
	};
}

TEST_CASE("the messenger fills the envelope and filters what it receives") {
	Loop link;
	Comm::Messenger r1(link, "r1");
	r1.SetMission("m1");
	auto m = r1.Begin("M_DONE", "all", 12.3456);
	m.fields.push_back(Protocol::Field::MakeText(Protocol::FieldType::TaskId, "op.4"));
	REQUIRE(r1.Post(m));
	CHECK(link.queue.at(0) == "M: M_DONE 0.1 m1 r1 all 1 12.346 op.4/");

	Comm::Messenger r2(link, "r2");
	r2.SetMission("m1");
	auto got = r2.Receive();
	REQUIRE(got.size() == 1);
	CHECK(got[0].code == "M_DONE");
	CHECK(got[0].sender == "r1");
	CHECK(got[0].seq == 1);
	CHECK(got[0].Slot(0).s == "op.4");

	// A duplicate, another mission, a unicast for someone else, a broken message, our own message.
	link.queue = {"M: M_DONE 0.1 m1 r1 all 1 12.346 op.4/", "M: M_DONE 0.1 m2 r1 all 2 13 op.4/",
	              "M: M_DONE 0.1 m1 r1 r3 3 13 op.4/", "M: M_DONE 0.1 m1 r1 all/", "M: M_DONE 0.1 m1 r2 all 9 13 op.4/",
	              "M: M_RELEASE 0.1 m1 r1 r2 4 14 op.4/"};
	got = r2.Receive();
	REQUIRE(got.size() == 1);
	CHECK(got[0].code == "M_RELEASE");  // a unicast to r2 passes
	const auto& s = r2.Stats();
	CHECK(s.dropped_duplicate == 1);
	CHECK(s.dropped_mission == 1);
	CHECK(s.not_for_us == 1);
	CHECK(s.dropped_parse == 1);
	CHECK(s.own == 1);

	// Before a mission is known, only M_MISSION passes.
	Comm::Messenger r3(link, "r3");
	link.queue = {"M: M_DONE 0.1 m1 r1 all 5 15 op.4/", "M: M_MISSION 0.1 m1 op all 1 0 H_1/H_1: H_M m1 0 0 0 -1 1 -1 1 5 1 1 1 0/"};
	got = r3.Receive();
	REQUIRE(got.size() == 1);
	CHECK(got[0].code == "M_MISSION");
	CHECK(got[0].Child(0).code == "H_M");
}

TEST_CASE("claims: the higher score wins, then the lower robot id") {
	using Algorithms::Claim;
	using Algorithms::TaskPool;
	CHECK(TaskPool::Beats(Claim{"r3", 10, 0.9}, Claim{"r1", 10, 0.5}));
	CHECK(TaskPool::Beats(Claim{"r2", 10, 0.5}, Claim{"r10", 10, 0.5}));
	CHECK_FALSE(TaskPool::Beats(Claim{"r10", 10, 0.5}, Claim{"r2", 10, 0.5}));
	Algorithms::PoolEntry e;
	e.claims["r2"] = {"r2", 5, 0.5};
	e.claims["r4"] = {"r4", 50, 0.7};
	CHECK(TaskPool::BestClaim(e)->owner == "r4");
	CHECK(TaskPool::BestClaim(e, "r4")->owner == "r2");
	TaskPool pool;
	Task::FunctionRegistry f;
	Task::TaskFactory factory(f);
	auto tasks = factory.BuildTasks("T: T_A op.1 1 0 C_1 C_2 A_1/\nC_1: C_N/\nC_2: C_N/\nA_1: A_N/\n");
	auto* p = pool.Add(std::shared_ptr<const Task::Task>(std::move(tasks[0])), 1.0);
	REQUIRE(p);
	p->claims = e.claims;
	pool.ExpireClaims(10.0);
	CHECK(p->claims.size() == 1);
	CHECK(p->claims.count("r4") == 1);
	CHECK(pool.Add(p->task, 2.0) == nullptr);  // a known id changes nothing
}

TEST_CASE("the 2021 task sets port to timelines, and the issuer sends them in time order") {
	const std::string line = "12 T: 3 0 T_A 10 /C_S C_P 160.21 253.13 0.00 5.00/C_E C_N/T_A 4 /A_L 3/A_W 30/A_L 3/A_W 30/\n"
	                         "5 T: 4 0 T_A 10 /C_S C_P 800 500 0 5/C_E C_N/T_A 4 /A_L 4/A_W 30/A_L 4/A_W 30/\n";
	const std::string timeline = Algorithms::Port2021TaskSet(line);
	auto parsed = Protocol::Parse(timeline);
	REQUIRE(parsed.Ok());
	REQUIRE(parsed.document.records.size() == 3);  // @ and two L_D
	const auto& task = parsed.document.records[1].children.at(0);
	CHECK(task.fields[0].s == "op.4");
	const auto& c = task.children.at(0);
	CHECK(c.code == "C_P3");
	CHECK(c.fields[0].n == doctest::Approx(-33.979));
	CHECK(c.fields[1].n == doctest::Approx(-24.687));
	CHECK(c.fields[2].n == doctest::Approx(15));
	CHECK(timeline.find("A_W 3/") != std::string::npos);

	// Every ported 2021 set loads.
	for (int k = 1; k <= 25; ++k) {
		CAPTURE(k);
		auto set = Protocol::Parse(Test::ReadFile(TaskSets2021() / ("taskset" + std::to_string(k) + ".mrsl")));
		CHECK(set.Ok());
	}

	Pipe op_end, robot_end;
	op_end.Connect(&robot_end);
	robot_end.Connect(&op_end);
	Algorithms::TaskIssuer issuer(op_end, Header(1));
	issuer.LoadTimeline(timeline);
	Comm::Messenger robot(robot_end, "r1");
	robot.SetMission("m1");
	std::vector<std::pair<double, std::string>> sent;
	for (double t = 0; t < 14; t += 0.5) {
		issuer.Tick(t);
		for (const auto& m : robot.Receive())
			if (m.code == "M_TASK") sent.push_back({t, m.Child(0).fields[0].s});
	}
	REQUIRE(sent.size() == 2);
	CHECK(sent[0].second == "op.5");  // dispatched at 5 although it comes second in the file
	CHECK(sent[0].first == doctest::Approx(5));
	CHECK(sent[1].second == "op.4");
	CHECK(sent[1].first == doctest::Approx(12));
	CHECK_FALSE(issuer.Done());
}

TEST_CASE("five UAVs complete a ported 2021 task set with MRS-RTA") {
	// The experiments' mission header is the one the team flies here.
	auto file = Protocol::Parse(Test::ReadFile(TaskSets2021() / ".." / "mission_5uav.mrs"));
	REQUIRE(file.Ok());
	CHECK(file.document.records.back() == Header(5));

	Team team(5);
	team.issuer.LoadTimeline(Test::ReadFile(TaskSets2021() / "taskset1.mrsl"));
	REQUIRE(team.Run(600.0));
	REQUIRE(team.issuer.Tasks().size() == 15);
	for (const auto& entry : team.issuer.Tasks()) {
		const auto& it = entry.second;
		CAPTURE(entry.first);
		CHECK(it.done.has_value());
		CHECK_FALSE(it.failed.has_value());
	}
	std::ostringstream share;
	int busy = 0;
	for (int k = 1; k <= 5; ++k) {
		share << " r" << k << ":" << team.DoneBy("r" + std::to_string(k));
		busy += team.DoneBy("r" + std::to_string(k)) > 0;
	}
	CHECK(busy >= 4);  // the work is shared (the pursuit factor, spec 09 §4.2)
	for (auto& r : team.robots) CHECK_FALSE(r->sim.crashed);
	MESSAGE("makespan " << *team.issuer.Makespan() << " s, " << team.air.messages << " messages, done by" << share.str());
}

TEST_CASE("exclusive MRS-RTA: claims keep robots off each other's tasks") {
	Algorithms::RtaConfig rta;
	rta.exclusive = true;
	Team team(5, rta);
	team.issuer.LoadTimeline(Test::ReadFile(TaskSets2021() / "taskset1.mrsl"));
	REQUIRE(team.Run(600.0));
	for (const auto& entry : team.issuer.Tasks()) {
		const auto& it = entry.second;
		CAPTURE(entry.first);
		CHECK(it.done.has_value());
		CHECK(it.done_count == 1);  // no duplicate arrivals
	}
	std::ostringstream share;
	for (int k = 1; k <= 5; ++k) share << " r" << k << ":" << team.DoneBy("r" + std::to_string(k));
	MESSAGE("makespan " << *team.issuer.Makespan() << " s, done by" << share.str());
}

TEST_CASE("a UAV without LEDs dumps the LED tasks and the others do them") {
	Team team(5, {}, {5});
	REQUIRE_FALSE(team.robots[4]->ctl->Profile().actions.count("A_L"));
	team.issuer.LoadTimeline(Test::ReadFile(TaskSets2021() / "taskset1.mrsl"));
	REQUIRE(team.Run(600.0));
	for (const auto& entry : team.issuer.Tasks()) {
		const auto& it = entry.second;
		CAPTURE(entry.first);
		CHECK(it.done.has_value());
		CHECK(std::find(it.dumped_by.begin(), it.dumped_by.end(), "r5") != it.dumped_by.end());
	}
	CHECK(team.DoneBy("r5") == 0);
	CHECK(team.robots[4]->layer->Stats().dumps_static == 15);
	for (const auto* e : team.robots[4]->layer->Pool().Entries()) CHECK(e->state != Algorithms::PoolState::ACTIVE);
}

TEST_CASE("a task no robot can do becomes impossible") {
	Team team(2, {}, {1, 2});
	team.issuer.Schedule(2.0, Protocol::Parse("T: T_A op.1 1 0 C_1 C_2 A_1..2/\nC_1: C_P3 10 10 15 1 0.5 0 -1/\nC_2: C_N/\nA_1: A_L 3 255/\nA_2: A_N/\n")
	                              .document.records);
	REQUIRE(team.Run(30.0));
	const auto& it = team.issuer.Tasks().at("op.1");
	CHECK(it.failed == std::optional<std::string>("IMPOSSIBLE"));
	CHECK(it.dumped_by.size() == 2);
	for (auto& r : team.robots) CHECK(r->layer->Pool().Find("op.1")->state == Algorithms::PoolState::IMPOSSIBLE);
}

TEST_CASE("a done task is taken back from the robots still flying to it") {
	// Open mode, two robots, one task: both may set off; the first to finish tells the other.
	Team team(2);
	team.issuer.Schedule(1.0, Protocol::Parse("T: T_A op.1 1 0 C_1 C_2 A_1/\nC_1: C_P3 20 0 15 1 0.5 0 -1/\nC_2: C_N/\nA_1: A_N/\n")
	                              .document.records);
	REQUIRE(team.Run(120.0));
	team.Run(2.0, false);  // let the DONE arrive
	for (auto& r : team.robots) {
		CHECK(r->layer->Assigned().empty());
		CHECK(r->ctl->CurrentTask().empty());
		CHECK(r->layer->Pool().Find("op.1")->state == Algorithms::PoolState::DONE);
	}
}
