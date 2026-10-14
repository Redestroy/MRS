// WP11 tests (spec 15 §6): worldview sync, the ranging sensor and relative positions, and the
// repulsion processor with its opt-in use in the safety supervisor; the sync behaviour and the
// capability guard (spec 15 §2.5).
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "../task/Harness.h"
#include "mrs/comm/Messenger.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"
#include "mrs/robot/SafetySupervisor.h"
#include "mrs/sim/Team.h"
#include "mrs/world/Neighbours.h"
#include "mrs/world/UavProcessors.h"

using namespace MRS;
using Environment::View;
using Environment::WorldModel;
using Task::FailReason;
using Task::TaskState;

namespace {
	Sim::RobotFiles Files() {
		return {Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsd"), Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsp"),
		        (Test::ExamplesDir() / "uav_behaviours.mrsb").string()};
	}

	Sim::TeamConfig Config(int n) {
		Sim::TeamConfig c;
		c.mission.homes = Sim::RowHomes(n);  // (-4, -5), (0, -5), (4, -5) for three
		return c;
	}

	View Detection(double stamp, double x, double y, const std::string& source = {}) {
		return {"V_DET", stamp, {x, y, 0.0, 0.9}, "person", source};
	}

	// A model with the views a radio robot has (GNSS, attitude, battery, peers).
	WorldModel RadioModel() { return WorldModel::ForViews({"V_GEO", "V_ATT", "V_RATE", "V_MAG", "V_BAT", "V_PEER", "V_FLD", "V_DET"}); }

	class NullSink : public Task::IActionSink {
	public:
		Device::ActionStatus Dispatch(const Device::ActionMap&, double) override { return Device::ActionStatus::DONE; }
	};

	Device::ActionMap One(const char* code, double a, double b) {
		Device::ActionMap m;
		m.entries.push_back({"any", {code, *Device::PackArgument(Device::ArgLayout::F32X2, std::vector<double>{a, b})}});
		return m;
	}

	std::vector<double> Values(const Device::ActionMap& m) { return Device::UnpackReals(Device::ArgLayout::F32X2, m.entries.at(0).action.arg); }

	Environment::Neighbour At(double x, double y, double z) {
		Environment::Neighbour n;
		n.id = "r2";
		n.rel = {x, y, z};
		return n;
	}
}

TEST_CASE("V_FLD and M_SYNC parse and round-trip; M_SYNC carries only shared views") {
	const std::string text = "M: M_SYNC 0.1 m1 r1 all 5 12.5 V_1 V_2/V_1: V_FLD 12.4 wind.speed 3.5/V_2: V_DET 12.1 person 10 20 0 0.9/";
	auto parsed = Protocol::Parse(text);
	REQUIRE(parsed.Ok());
	auto m = Comm::Decode(text);
	REQUIRE(m);
	CHECK(m->code == "M_SYNC");
	auto v = Environment::ViewFromRecord(m->record.children.at(0));
	REQUIRE(v);
	CHECK(v->code == "V_FLD");
	CHECK(v->text == "wind.speed");
	CHECK(v->values == std::vector<double>{3.5});
	CHECK(Protocol::Parse(Protocol::Write(Environment::ToRecord(*v))).Ok());
	CHECK(Comm::Encode(m->record) == text);
	CHECK_FALSE(Protocol::Parse("M: M_SYNC 0.1 m1 r1 all 5 12.5 V_1/V_1: V_BAT 12 11 0.5 20/").Ok());
	CHECK_FALSE(Protocol::Parse("M: M_SYNC 0.1 m1 r1 all 5 12.5/").Ok());
}

TEST_CASE("WorldviewSyncProcessor: peer fields land under sync.rN and fill only fields the robot lacks") {
	auto model = RadioModel();
	REQUIRE(model.chain.Find("WorldviewSyncProcessor"));
	model.Update({{"V_FLD", 9.5, {3.5}, "wind.speed", "peer:r2"}, {"V_FLD", 9.5, {0.1}, "battery.remaining", "peer:r2"}}, 10.0);
	auto& w = model.world;
	CHECK(w.Scalar("sync.r2.wind.speed", 10.0) == doctest::Approx(3.5));
	CHECK(w.Scalar("wind.speed", 10.0) == doctest::Approx(3.5));
	CHECK(w.SelectedSource("wind.speed") == "peer:r2");
	// The robot's own battery field is not overwritten by a peer's.
	CHECK(w.Scalar("sync.r2.battery.remaining", 10.0) == doctest::Approx(0.1));
	CHECK_FALSE(w.Has("battery.remaining"));
	// A source of the robot's own wins over any peer, whatever their names; among peers the newest wins.
	w.OfferScalar("wind.speed", "anemometer", 2.0, 10.0);
	model.Update({{"V_FLD", 10.0, {4.0}, "wind.speed", "peer:r3"}}, 10.1);
	CHECK(w.SelectedSource("wind.speed") == "anemometer");
	CHECK(w.Scalar("wind.speed", 10.1) == doctest::Approx(2.0));
	w.Erase("wind.speed");
	Environment::Worldview only_peers;
	only_peers.OfferScalar("x", "peer:r2", 1.0, 5.0);
	only_peers.OfferScalar("x", "peer:r3", 2.0, 6.0);
	CHECK(only_peers.SelectedSource("x") == "peer:r3");
	// A view without a peer source is ignored: V_FLD comes only from peers.
	model.Update({{"V_FLD", 10.2, {9.0}, "wind.gust", {}}}, 10.2);
	CHECK_FALSE(w.Has("wind.gust"));
}

TEST_CASE("a detection one UAV makes reaches its peers once, and a later peer learns it by asking") {
	SUBCASE("broadcast") {
		Sim::Team team(Files(), Config(3));
		team.Run(1.0, false);
		team.robots[0]->ctl->InjectViews({Detection(team.Time(), 10, 20)});
		team.Run(3.0, false);
		for (int k = 1; k < 3; ++k) {
			const auto* o = team.robots[k]->ctl->World().Object("det.person.1");
			REQUIRE(o);
			CHECK(o->source == "peer:r1");
			CHECK(o->position.x == doctest::Approx(10));
			CHECK(team.robots[k]->ctl->World().Scalar("det.person.1.enu.y", team.Time()) == doctest::Approx(20));
		}
		// r1 keeps its own detection; r2 and r3 do not send it on.
		CHECK(team.robots[0]->ctl->World().Object("det.person.1")->source == "detector");
		CHECK(team.robots[0]->layer->Stats().sync_views_sent == 1);
		CHECK(team.robots[1]->layer->Stats().sync_views_sent == 0);
		CHECK(team.robots[2]->layer->Stats().sync_views_sent == 0);
		CHECK(team.robots[0]->layer->Stats().sync_views_received == 0);
		// Nothing changed: no second M_SYNC.
		team.Run(3.0, false);
		CHECK(team.robots[0]->layer->Stats().sync_views_sent == 1);
	}
	SUBCASE("asking a new peer") {
		// r1 holds a detection it heard of before the others joined (source peer:r9), so it does not
		// broadcast it; r2 and r3 learn it only from r1's answer to their M_INFOREQ.
		Sim::Team team(Files(), Config(3));
		team.robots[0]->ctl->InjectViews({Detection(0.0, -30, 15, "peer:r9")});
		team.Run(3.0, false);
		for (int k = 1; k < 3; ++k) {
			const auto* o = team.robots[k]->ctl->World().Object("det.person.1");
			REQUIRE(o);
			CHECK(Environment::IsPeerSource(o->source));
			CHECK(o->position.x == doctest::Approx(-30));
			CHECK(team.robots[k]->layer->Stats().info_requests == 2);
		}
		CHECK(team.robots[0]->layer->Stats().sync_sent >= 2);  // two M_INFO answers
	}
}

TEST_CASE("the ranging sensor reports peers in range, ENU, and writes rel.rN") {
	Sim::QuadSim a, b, c;
	a.pos = {0, 0, 5};
	b.pos = {3, -4, 6};
	c.pos = {40, 0, 5};
	a.peers = {{2, &b}, {3, &c}};
	Port::PortAssignment pa;
	pa.type = Port::PortType::SIM;
	pa.address = "ranging";
	pa.params.Add("range_m", Device::NumValue(10.0));
	auto port = a.Open(pa);
	REQUIRE(port);
	std::vector<double> v;
	REQUIRE(port->Read(v));
	REQUIRE(v.size() == 4);
	CHECK(v[0] == 2);
	CHECK(v[1] == doctest::Approx(3));
	CHECK(v[2] == doctest::Approx(-4));
	CHECK(v[3] == doctest::Approx(1));

	// In a team every UAV with the sensor sees the others; r1 at (-4, -5) sees r2 at (0, -5) 4 m east.
	auto config = Config(3);
	config.ranging_noise = 0.0;
	Sim::Team team(Files(), config);
	REQUIRE(team.robots[0]->robot.faults.empty());
	CHECK(team.robots[0]->robot.self.views.count("V_REL3"));
	team.Run(1.0, false);
	const auto& w = team.robots[0]->ctl->World();
	CHECK(w.Scalar("rel.r2.enu.x", team.Time()) == doctest::Approx(4));
	CHECK(w.Scalar("rel.r2.enu.y", team.Time()) == doctest::Approx(0));
	CHECK(w.Scalar("rel.r3.range", team.Time()) == doctest::Approx(8));
	CHECK(w.SelectedSource("rel.r2.enu") == "");
	CHECK(w.Raw("rel.r2.enu.x")->source == "ranging");
}

TEST_CASE("Neighbours prefer a fresh relative measurement over peer states") {
	Environment::Worldview w;
	w.SetVec3("pose.enu", 0, 0, 10, 10.0, "gnss");
	w.SetVec3("vel.enu", 1, 0, 0, 10.0, "nav");
	w.SetVec3("peer.r2.pose.enu", 5, 1, 10, 9.8, "r2");
	w.SetVec3("peer.r2.vel.enu", -1, 0, 0, 9.8, "r2");
	w.SetVec3("peer.r3.pose.enu", 0, 9, 10, 5.0, "r3");  // stale: more than 2 s old
	auto n = Environment::Neighbours(w, 10.0);
	REQUIRE(n.size() == 1);
	CHECK(n[0].id == "r2");
	CHECK_FALSE(n[0].measured);
	CHECK(n[0].rel.x == doctest::Approx(5));
	CHECK(n[0].rel_vel.x == doctest::Approx(-2));
	w.SetVec3("rel.r2.enu", 4.5, 1.2, 0.1, 9.9, "ranging");
	n = Environment::Neighbours(w, 10.0);
	REQUIRE(n.size() == 1);
	CHECK(n[0].measured);
	CHECK(n[0].rel.x == doctest::Approx(4.5));
	// rel fields go stale after 1 s.
	n = Environment::Neighbours(w, 11.0);
	CHECK(n.empty());
}

TEST_CASE("repulsion points away from close robots and fades with distance") {
	using Environment::RepulsionFrom;
	auto none = RepulsionFrom({});
	CHECK(none.count == 0);
	CHECK(none.nearest < 0);
	CHECK(none.force.x == 0);

	auto near = RepulsionFrom({At(1, 0, 0)});  // 1 m east: push west
	CHECK(near.count == 1);
	CHECK(near.nearest == doctest::Approx(1));
	CHECK(near.force.x < 0);
	CHECK(std::fabs(near.force.y) < 1e-9);
	CHECK(near.force.x == doctest::Approx(-0.5 * (4.0 - 1.0)));  // gain · (1/s − 1), s = 1/4

	auto farther = RepulsionFrom({At(3, 0, 0)});
	CHECK(farther.force.x < 0);
	CHECK(std::fabs(farther.force.x) < std::fabs(near.force.x));
	CHECK(RepulsionFrom({At(4.5, 0, 0)}).count == 0);  // outside the 4 m radius

	// A robot one 3 m layer up does not push; one 1 m up pushes down.
	CHECK(RepulsionFrom({At(0.5, 0, 3)}).count == 0);
	auto above = RepulsionFrom({At(0, 0, 1)});
	CHECK(above.force.z < 0);
	// Capped, and straight up when on top of each other.
	auto on_top = RepulsionFrom({At(0, 0, 0)});
	CHECK(on_top.force.z == doctest::Approx(3));
	// Closing in: the look-ahead pushes earlier.
	auto still = RepulsionFrom({At(3, 0, 0)});
	auto n = At(3, 0, 0);
	n.rel_vel = {-2, 0, 0};
	auto closing = RepulsionFrom({n});
	CHECK(std::fabs(closing.force.x) > std::fabs(still.force.x));
	// Two neighbours on opposite sides cancel.
	auto both = RepulsionFrom({At(2, 0, 0), At(-2, 0, 0)});
	CHECK(both.count == 2);
	CHECK(std::fabs(both.force.x) < 1e-9);
}

TEST_CASE("the repulsion processor writes repulse.*, and the supervisor uses it only when enabled") {
	auto model = RadioModel();
	REQUIRE(model.chain.Find("RepulsionProcessor"));
	auto& w = model.world;
	model.Update({}, 1.0);
	CHECK(w.Scalar("repulse.enu.x", 1.0) == doctest::Approx(0));
	CHECK(w.Scalar("repulse.count", 1.0) == doctest::Approx(0));
	CHECK_FALSE(w.Has("repulse.nearest"));
	w.SetVec3("pose.enu", 0, 0, 20, 2.0, "gnss");
	w.SetVec3("rel.r2.enu", 2, 0, 0, 2.0, "ranging");
	model.Update({}, 2.0);
	REQUIRE(w.Scalar("repulse.enu.x", 2.0));
	CHECK(*w.Scalar("repulse.enu.x", 2.0) == doctest::Approx(-0.5));
	CHECK(w.Scalar("repulse.nearest", 2.0) == doctest::Approx(2));
	CHECK(w.Scalar("repulse.count", 2.0) == doctest::Approx(1));

	w.SetBool("airborne", true, 2.0, "flight");
	NullSink sink;
	Robot::SafetySupervisor off(sink, w);
	CHECK(Values(off.Filter(One("A_PXY", 10, 5), 2.0))[0] == doctest::Approx(10));
	Robot::SafetyConfig sc;
	sc.repulsion_lead = 2.0;
	Robot::SafetySupervisor on(sink, w, sc);
	CHECK(Values(on.Filter(One("A_PXY", 10, 5), 2.0))[0] == doctest::Approx(9));
	CHECK(Values(on.Filter(One("A_VXY", 1, 0), 2.0))[0] == doctest::Approx(0.5));
	CHECK(Values(on.Filter(One("A_PZY", 20, 0), 2.0))[0] == doctest::Approx(20));
	// On the ground the setpoints are left alone.
	w.SetBool("airborne", false, 2.0, "flight");
	CHECK(Values(on.Filter(One("A_PXY", 10, 5), 2.0))[0] == doctest::Approx(10));
}

namespace {
	const char* kDetectionStart = "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_V V_1/ V_1: V_DET 0 person 10 20 0 0.9/ C_2: C_N/ A_1: A_N/";

	struct SyncFixture {
		Task::FunctionRegistry registry;
		Task::TaskFactory factory{registry};
		Task::BehaviourLibrary library;
		Environment::Worldview w;
		Test::ScriptedSink sink;
		std::vector<std::vector<std::string>> asked;  // topics of every A_SY
		int peers = 2;
		double t = 0.0;

		SyncFixture() {
			Task::RegisterUavFunctions(registry);
			library.PopulateFromFile((Test::ExamplesDir() / "uav_behaviours.mrsb").string(), factory);
		}

		std::vector<Task::TaskEvent> Run(Task::TaskExecutor& ex, double seconds, const std::function<void()>& each = {}) {
			std::vector<Task::TaskEvent> events;
			for (double end = t + seconds; t < end && !ex.Empty(); t += 0.5) {
				if (each) each();
				auto r = ex.Tick(w, t);
				events.insert(events.end(), r.events.begin(), r.events.end());
			}
			return events;
		}

		void Hook(Task::TaskExecutor& ex) {
			ex.SetSync([this](const std::vector<std::string>& topics, double) {
				asked.push_back(topics);
				return peers;
			});
		}
	};

	const Task::TaskEvent* Final(const std::vector<Task::TaskEvent>& events, const std::string& id) {
		const Task::TaskEvent* out = nullptr;
		for (const auto& e : events)
			if (e.task_id == id) out = &e;
		return out;
	}
}

TEST_CASE("the sync behaviour asks the peers, and only an answer makes the condition TRUE") {
	SyncFixture f;
	Task::TaskExecutor ex(f.library, f.sink);
	f.Hook(ex);
	ex.Push(Test::OneTask(f.factory, kDetectionStart));

	SUBCASE("a peer answers") {
		auto events = f.Run(ex, 1.0);
		REQUIRE(f.asked.size() == 1);
		CHECK(f.asked[0] == std::vector<std::string>{"det.person"});
		CHECK_FALSE(ex.Empty());  // waiting for the answer
		// The answer arrives: a detection with a peer source, as the MRS layer injects it.
		f.w.SetVec3("det.person.1.enu", 10.5, 20, 0, f.t, "peer:r2");
		events = f.Run(ex, 1.0);
		REQUIRE(Final(events, "sync.view"));
		CHECK(Final(events, "sync.view")->state == TaskState::SUCCEEDED);
		CHECK(Final(events, "op.1")->state == TaskState::SUCCEEDED);
		CHECK(f.asked.size() == 1);
	}
	SUBCASE("no answer: NOT_KNOWN after the timeout, never assumed") {
		auto events = f.Run(ex, 5.0);
		CHECK(f.asked.size() == 1);
		REQUIRE(Final(events, "op.1"));
		CHECK(Final(events, "sync.view")->reason == FailReason::NOT_KNOWN);
		CHECK(Final(events, "op.1")->state == TaskState::FAILED);
		CHECK(Final(events, "op.1")->reason == FailReason::NOT_KNOWN);
		CHECK(f.t <= 3.0);
	}
	SUBCASE("no peer to ask: NOT_KNOWN at once") {
		f.peers = 0;
		auto events = f.Run(ex, 1.0);  // the behaviour fails in the first tick, the task in the next
		REQUIRE(Final(events, "op.1"));
		CHECK(Final(events, "op.1")->state == TaskState::FAILED);
		CHECK(Final(events, "op.1")->reason == FailReason::NOT_KNOWN);
	}
}

TEST_CASE("an UNKNOWN world field is asked for; the robot's own state is not") {
	SyncFixture f;
	Task::TaskExecutor ex(f.library, f.sink);
	f.Hook(ex);
	SUBCASE("world field") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_m wind.speed lt 10 0/ C_2: C_N/ A_1: A_N/"));
		f.Run(ex, 1.0);
		REQUIRE(f.asked.size() == 1);
		CHECK(f.asked[0] == std::vector<std::string>{"wind.speed"});
		f.w.SetScalar("wind.speed", 3.0, f.t, "peer:r3");
		auto events = f.Run(ex, 1.0);
		REQUIRE(Final(events, "op.1"));
		CHECK(Final(events, "sync.field")->state == TaskState::SUCCEEDED);
		CHECK(Final(events, "op.1")->state == TaskState::SUCCEEDED);
	}
	SUBCASE("a stale field that stays unknown still fails with MISSING_FIELD") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_m wind.speed lt 10 0/ C_2: C_N/ A_1: A_N/"));
		auto events = f.Run(ex, 10.0);
		CHECK(f.asked.size() == 1);  // asked once, not again
		REQUIRE(Final(events, "op.1"));
		CHECK(Final(events, "op.1")->reason == FailReason::MISSING_FIELD);
	}
	SUBCASE("own state") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_m battery.remaining gt 0.5 0/ C_2: C_N/ A_1: A_N/"));
		auto events = f.Run(ex, 10.0);
		CHECK(f.asked.empty());
		REQUIRE(Final(events, "op.1"));
		CHECK(Final(events, "op.1")->reason == FailReason::MISSING_FIELD);
	}
}

TEST_CASE("knowing from peers is not measuring: the capability guard") {
	Task::FunctionRegistry registry;
	Task::RegisterUavFunctions(registry);
	Task::TaskFactory factory{registry};
	auto uses = Test::OneTask(factory, kDetectionStart);
	// A search: fly a pattern until a person is detected (the end condition).
	auto finds = Test::OneTask(factory, "T: T_A op.2 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_V V_1/ V_1: V_DET 0 person 10 20 0 0.9/ A_1: A_HD 0 0/");

	// The Webots Mavic has a radio and no camera: it can know detections, not make them.
	Sim::Team team(Files(), Config(1));
	const auto& self = team.robots[0]->robot.self;
	CHECK(self.Provides("det"));
	CHECK_FALSE(self.measured.count("det"));
	CHECK(self.measured.count("pose.enu"));
	CHECK(self.measured.count("peer"));
	CHECK(self.Accepts("A_SY"));
	const auto radio = self.ToProfile();
	CHECK(uses->MeetsStaticRequirements(radio));
	CHECK_FALSE(finds->MeetsStaticRequirements(radio));

	auto camera = radio;
	camera.measured->insert("det");
	CHECK(finds->MeetsStaticRequirements(camera));

	// Without a radio a robot neither knows nor asks.
	Task::CapabilityProfile alone;
	alone.fields = alone.measured.emplace(*radio.measured);
	alone.actions = radio.actions;
	alone.actions.erase("A_SY");
	CHECK_FALSE(uses->MeetsStaticRequirements(alone));
	Task::BehaviourLibrary library;
	library.PopulateFromFile((Test::ExamplesDir() / "uav_behaviours.mrsb").string(), factory);
	CHECK(library.Find(uses->StartCondition(), &radio));
	CHECK(library.Find(uses->StartCondition(), &radio)->name == "sync.view");
	CHECK_FALSE(library.Find(uses->StartCondition(), &alone));
}

TEST_CASE("a UAV asks its peer for a detection it needs, and gets only what the peer holds") {
	Sim::Team team(Files(), Config(2));
	team.Run(2.0, false);  // first contact; nothing to share yet
	// r1 heard of a person from a robot that has since left (source peer:r9), so it does not broadcast it.
	team.robots[0]->ctl->InjectViews({Detection(team.Time(), 10, 20, "peer:r9")});
	team.Run(1.0, false);
	auto& r2 = *team.robots[1]->ctl;
	REQUIRE(r2.World().Object("det.person.1") == nullptr);

	Task::FunctionRegistry registry;
	Task::TaskFactory factory{registry};
	auto final_state = [&](const std::string& id) -> const Task::TaskEvent* {
		const Task::TaskEvent* out = nullptr;
		for (const auto& e : r2.TaskLog())
			if (e.event.task_id == id) out = &e.event;
		return out;
	};
	SUBCASE("the peer knows") {
		r2.Executor().Push(Test::OneTask(factory, kDetectionStart));
		team.Run(3.0, false);
		REQUIRE(final_state("op.1"));
		CHECK(final_state("sync.view")->state == TaskState::SUCCEEDED);
		CHECK(final_state("op.1")->state == TaskState::SUCCEEDED);
		REQUIRE(r2.World().Object("det.person.1"));
		CHECK(r2.World().Object("det.person.1")->source == "peer:r1");
	}
	SUBCASE("nobody knows") {
		r2.Executor().Push(Test::OneTask(factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_V V_1/ V_1: V_DET 0 car 10 20 0 0.9/ C_2: C_N/ A_1: A_N/"));
		team.Run(4.0, false);
		REQUIRE(final_state("op.1"));
		CHECK(final_state("op.1")->state == TaskState::FAILED);
		CHECK(final_state("op.1")->reason == FailReason::NOT_KNOWN);
		CHECK(r2.World().Object("det.car.1") == nullptr);
	}
}
