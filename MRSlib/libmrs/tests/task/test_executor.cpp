// WP1 acceptance: task strings run step by step against a scripted worldview (plan §11).
#include "doctest.h"

#include "../protocol/TestFiles.h"
#include "Harness.h"

using namespace MRS;
using Task::FailReason;
using Task::TaskState;

namespace {
	struct Fixture {
		Task::FunctionRegistry registry;
		Task::TaskFactory factory{registry};
		Task::BehaviourLibrary library;
		Environment::Worldview w;
		Test::ScriptedSink sink;
		double t = 0.0;

		Fixture() { Task::RegisterUavFunctions(registry); }

		void LoadUavLibrary() { library.PopulateFromFile((Test::ExamplesDir() / "uav_behaviours.mrsb").string(), factory); }

		Task::TickResult Tick(Task::TaskExecutor& ex, double dt = 0.5) {
			auto r = ex.Tick(w, t);
			t += dt;
			return r;
		}
	};

	bool HasEvent(const Task::TickResult& r, const std::string& id, TaskState s) {
		for (const auto& e : r.events)
			if (e.task_id == id && e.state == s) return true;
		return false;
	}
}

TEST_CASE("the null action ends the task through the end condition (spec 03 §8.3)") {
	Fixture f;
	Task::TaskExecutor ex(f.library, f.sink);

	SUBCASE("end condition TRUE: SUCCEEDED") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1 A_2/ C_1: C_N/ C_2: C_N/ A_1: A_L 1 255/ A_2: A_N/"));
		auto r1 = f.Tick(ex);
		CHECK(HasEvent(r1, "op.1", TaskState::IN_PROGRESS));
		REQUIRE(r1.dispatched);
		CHECK(r1.dispatched->entries[0].action.code == "A_L");
		auto r2 = f.Tick(ex);  // A_N is reached on the next tick
		CHECK_FALSE(r2.dispatched);
		CHECK(HasEvent(r2, "op.1", TaskState::SUCCEEDED));
		CHECK(ex.Empty());
	}
	SUBCASE("end condition FALSE: END_NOT_MET") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_F/ A_1: A_N/"));
		auto r = f.Tick(ex);
		REQUIRE(r.events.size() == 2);
		CHECK(r.events[1].state == TaskState::FAILED);
		CHECK(r.events[1].reason == FailReason::END_NOT_MET);
	}
	SUBCASE("end condition UNKNOWN: MISSING_FIELD") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_H 10 1/ A_1: A_N/"));
		auto r = f.Tick(ex);
		CHECK(r.events.back().reason == FailReason::MISSING_FIELD);
	}
	SUBCASE("past the last action behaves as A_N") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_L 1 0/"));
		f.Tick(ex);
		CHECK(HasEvent(f.Tick(ex), "op.1", TaskState::SUCCEEDED));
	}
}

TEST_CASE("A_W completes when at least d seconds have passed (spec 03 §8.2)") {
	Fixture f;
	Task::TaskExecutor ex(f.library, f.sink);
	ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1..3/ C_1: C_N/ C_2: C_N/ A_1: A_W 2/ A_2: A_L 1 1/ A_3: A_N/"));
	// Ticks every 0.5 s from t = 0: the wait starts at 0 and is done at exactly t = 2.0.
	for (int i = 0; i < 5; ++i) {
		auto r = f.Tick(ex);
		CHECK_FALSE(r.dispatched);  // A_W never reaches an actuator
	}
	CHECK(ex.TopIterator() == 1);       // done at t = 2.0 (>=, not >)
	auto r = f.Tick(ex);                // t = 2.5: the next action
	REQUIRE(r.dispatched);
	CHECK(f.sink.log.back().t == 2.5);
}

TEST_CASE("actuator answers move the iterator or end the task (spec 03 §8.2)") {
	Fixture f;
	Task::TaskExecutor ex(f.library, f.sink);
	const std::string text = "T: T_A op.1 1 0 C_1 C_2 A_1 A_2/ C_1: C_N/ C_2: C_N/ A_1: A_TO 3 1.5/ A_2: A_N/";

	SUBCASE("RUNNING dispatches the same action again") {
		int calls = 0;
		f.sink.respond = [&](const Device::ActionMap&, double) {
			return ++calls < 3 ? Device::ActionStatus::RUNNING : Device::ActionStatus::DONE;
		};
		ex.Push(Test::OneTask(f.factory, text));
		f.Tick(ex);
		f.Tick(ex);
		CHECK(ex.TopIterator() == 0);
		f.Tick(ex);
		CHECK(ex.TopIterator() == 1);
		CHECK(f.sink.log.size() == 3);
	}
	SUBCASE("REJECTED: NO_ACTUATOR") {
		f.sink.respond = [](const Device::ActionMap&, double) { return Device::ActionStatus::REJECTED; };
		ex.Push(Test::OneTask(f.factory, text));
		CHECK(f.Tick(ex).events.back().reason == FailReason::NO_ACTUATOR);
	}
	SUBCASE("FAILED: ACTUATOR_FAILED") {
		f.sink.respond = [](const Device::ActionMap&, double) { return Device::ActionStatus::FAILED; };
		ex.Push(Test::OneTask(f.factory, text));
		CHECK(f.Tick(ex).events.back().reason == FailReason::ACTUATOR_FAILED);
	}
	SUBCASE("A_I: IMPOSSIBLE") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_I/"));
		CHECK(f.Tick(ex).events.back().reason == FailReason::IMPOSSIBLE);
	}
	SUBCASE("exactly one action per tick") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1..3/ C_1: C_N/ C_2: C_N/ A_1: A_L 1 1/ A_2: A_L 1 2/ A_3: A_N/"));
		f.Tick(ex);
		CHECK(f.sink.log.size() == 1);
		f.Tick(ex);
		CHECK(f.sink.log.size() == 2);
	}
}

TEST_CASE("parametric values are recalculated every tick (spec 02 §6)") {
	Fixture f;
	Task::TaskExecutor ex(f.library, f.sink);
	f.sink.respond = [](const Device::ActionMap&, double) { return Device::ActionStatus::RUNNING; };
	ex.Push(Test::OneTask(f.factory, "T: T_P op.1 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_FN A_PXY F_1 F_2/"
	                                 "F_1: F_L 1 2 1 goal.x/ F_2: F_C -5 5 F_1/ F_1: F_L 2 0.8 -0.8 0 goal.x pose.enu.x/"));
	for (int i = 0; i < 4; ++i) {
		f.w.SetScalar("goal.x", 10.0 * i, f.t);
		f.w.SetVec3("pose.enu", 1.0 * i, 0, 0, f.t);
		auto r = f.Tick(ex);
		REQUIRE(r.dispatched);
		const auto& a = r.dispatched->entries[0].action;
		CHECK(a.code == "A_PXY");
		CHECK(a.FirstHalfAsDouble() == doctest::Approx(1 + 20.0 * i));
		CHECK(a.SecondHalfAsDouble() == doctest::Approx(std::min(5.0, 0.8 * (10.0 * i - 1.0 * i))));
	}
	SUBCASE("a missing field fails the task with MISSING_FIELD") {
		f.w.Erase("goal.x");
		CHECK(f.Tick(ex).events.back().reason == FailReason::MISSING_FIELD);
	}
}

TEST_CASE("combined actions are dispatched as one map") {
	Fixture f;
	Task::TaskExecutor ex(f.library, f.sink);
	ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_MAP any A_1 any A_2/ A_1: A_PXY 1 2/ A_2: A_PZY 15 0/"));
	auto r = f.Tick(ex);
	REQUIRE(r.dispatched);
	CHECK(r.dispatched->combined);
	REQUIRE(r.dispatched->entries.size() == 2);
	CHECK(r.dispatched->entries[1].action.code == "A_PZY");
}

TEST_CASE("an unmet start condition is fulfilled by a behaviour, which is bound and popped") {
	Fixture f;
	f.LoadUavLibrary();
	Test::ToyUav uav;
	uav.armed = true;
	uav.z = uav.sz = 20;  // already airborne at its layer
	Task::TaskExecutor ex(f.library, uav);
	ex.Push(Test::OneTask(f.factory, "T: T_A op.5 1 0 C_1 C_2 A_1/ C_1: C_P3 30 -40 20 1 0.5 0 -1/ C_2: C_N/ A_1: A_N/"));

	uav.Write(f.w, f.t);
	auto r = f.Tick(ex, 0.1);
	CHECK(HasEvent(r, "op.5", TaskState::STARTED));
	CHECK(HasEvent(r, "flyto.layered", TaskState::QUEUED));  // priority 10 beats flyto.direct
	CHECK(HasEvent(r, "flyto.layered", TaskState::IN_PROGRESS));
	CHECK(f.w.Scalar("target.x", f.t) == 30.0);
	CHECK(f.w.Scalar("target.y", f.t) == -40.0);
	REQUIRE(r.dispatched);  // the first setpoint goes out in the same tick

	auto events = Test::RunUntilEmpty(
	    ex, f.w, f.t, 0.1, 400, [&](double t) { uav.Write(f.w, t); }, [&](double, const Task::TickResult&) { uav.Step(0.1); });
	const auto* fly = Test::FinalEvent(events, "flyto.layered");
	REQUIRE(fly);
	CHECK(fly->state == TaskState::SUCCEEDED);
	const auto* done = Test::FinalEvent(events, "op.5");
	REQUIRE(done);
	CHECK(done->state == TaskState::SUCCEEDED);
	CHECK(std::hypot(uav.x - 30, uav.y + 40) <= 1.0);
	CHECK_FALSE(f.w.Has("target.x"));  // restored when the behaviour was popped
}

TEST_CASE("the spec example task runs from the ground: liftoff, layered fly-to, LEDs, wait") {
	Fixture f;
	f.LoadUavLibrary();
	Test::ToyUav uav;
	Task::TaskExecutor ex(f.library, uav);
	ex.Push(std::move(f.factory.BuildTasks(Test::ReadFile(Test::ExamplesDir() / "uav_point_task.mrst")).at(0)));

	std::vector<std::string> behaviours;
	auto events = Test::RunUntilEmpty(
	    ex, f.w, f.t, 0.1, 2000, [&](double t) { uav.Write(f.w, t); },
	    [&](double, const Task::TickResult& r) {
		    for (const auto& e : r.events)
			    if (e.behaviour && e.state == TaskState::QUEUED) behaviours.push_back(e.task_id);
		    uav.Step(0.1);
	    });
	const auto* done = Test::FinalEvent(events, "op.17");
	REQUIRE(done);
	CHECK(done->state == TaskState::SUCCEEDED);
	CHECK(behaviours == std::vector<std::string>{"flyto.layered", "liftoff"});
	CHECK(std::hypot(uav.x - 120, uav.y + 40) <= 1.0);
	CHECK(std::fabs(uav.z - 15) <= 0.5);
	CHECK(uav.led_mask == 3);
	CHECK(uav.led_colour == 0);  // turned off after the wait
	// Liftoff, then the layered fly-to climbed to the layer (20 m) before descending to 15 m.
	CHECK(std::find(uav.codes.begin(), uav.codes.end(), "A_TO") != uav.codes.end());
	CHECK(uav.max_alt > 19.0);
}

TEST_CASE("behaviour depth and lookup failures") {
	Fixture f;
	Task::TaskExecutor ex(f.library, f.sink);

	SUBCASE("no entry for the condition: NO_BEHAVIOUR") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_F/ C_2: C_N/ A_1: A_N/"));
		CHECK(f.Tick(ex).events.back().reason == FailReason::NO_BEHAVIOUR);
	}
	SUBCASE("deeper than max_behaviour_depth: NO_BEHAVIOUR") {
		// a needs b, b needs c, c needs d: the third level is refused.
		f.library.Populate("B: B_E need_b C_? a 1 T_1/ T_1: T_B 0 1 0 C_1..3 T_1/ C_1: C_? ?_1/ ?_1: b T/ C_2: C_N/ C_3: C_N/"
		                   "T_1: T_A 0 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_N/\n"
		                   "B: B_E need_c C_? b 1 T_1/ T_1: T_B 0 1 0 C_1..3 T_1/ C_1: C_? ?_1/ ?_1: c T/ C_2: C_N/ C_3: C_N/"
		                   "T_1: T_A 0 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_N/\n"
		                   "B: B_E need_d C_? c 1 T_1/ T_1: T_B 0 1 0 C_1..3 T_1/ C_1: C_N/ C_2: C_N/ C_3: C_N/"
		                   "T_1: T_A 0 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_N/",
		                   f.factory);
		for (const char* p : {"a", "b", "c"}) f.w.SetBool(p, false, 0);
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_? ?_1/ ?_1: a T/ C_2: C_N/ A_1: A_N/"));
		auto r = f.Tick(ex);
		CHECK(HasEvent(r, "need_b", TaskState::QUEUED));
		CHECK(HasEvent(r, "need_c", TaskState::QUEUED));
		CHECK_FALSE(HasEvent(r, "need_d", TaskState::QUEUED));
		const auto* end = Test::FinalEvent(r.events, "op.1");
		REQUIRE(end);
		CHECK(end->reason == FailReason::NO_BEHAVIOUR);
		CHECK(ex.Empty());
	}
	SUBCASE("a C_? that wants F has no behaviour") {
		f.LoadUavLibrary();
		f.w.SetBool("airborne", true, 0);
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_? ?_1/ ?_1: airborne F/ C_2: C_N/ A_1: A_N/"));
		CHECK(f.Tick(ex).events.back().reason == FailReason::NO_BEHAVIOUR);
	}
}

TEST_CASE("start condition that stays FALSE after the behaviours: START_UNREACHABLE") {
	Fixture f;
	// The behaviour ends as soon as `flag` is TRUE; the script turns it FALSE again.
	f.library.Populate("B: B_E set_flag C_? flag 1 T_1/ T_1: T_B 0 1 0 C_1..3 T_1/ C_1: C_N/ C_2: C_N/ C_3: C_N/"
	                   "T_1: T_A 0 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_N/",
	                   f.factory);
	Task::TaskExecutor ex(f.library, f.sink);
	ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_? ?_1/ ?_1: flag T/ C_2: C_N/ A_1: A_N/"));
	int pushes = 0;
	FailReason reason = FailReason::NONE;
	for (int i = 0; i < 20 && !ex.Empty(); ++i) {
		f.w.SetBool("flag", i % 2 == 1, f.t);  // FALSE when the task looks, TRUE when the behaviour looks
		auto r = f.Tick(ex);
		for (const auto& e : r.events) {
			if (e.task_id == "set_flag" && e.state == TaskState::QUEUED) ++pushes;
			if (e.task_id == "op.1" && e.state == TaskState::FAILED) reason = e.reason;
		}
	}
	CHECK(pushes == 3);
	CHECK(reason == FailReason::START_UNREACHABLE);
}

TEST_CASE("an UNKNOWN start condition fails after start_unknown_timeout") {
	Fixture f;
	Task::TaskExecutor ex(f.library, f.sink);
	ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_H 10 1/ C_2: C_N/ A_1: A_N/"));
	FailReason reason = FailReason::NONE;
	double failed_at = -1;
	for (int i = 0; i < 20 && !ex.Empty(); ++i) {
		const double now = f.t;
		auto r = f.Tick(ex);
		if (!r.events.empty() && r.events.back().state == TaskState::FAILED) {
			reason = r.events.back().reason;
			failed_at = now;
		}
	}
	CHECK(reason == FailReason::MISSING_FIELD);
	CHECK(failed_at == 5.0);
}

TEST_CASE("runtime conditions are checked every tick (spec 03 §8.5)") {
	Fixture f;
	f.sink.respond = [](const Device::ActionMap&, double) { return Device::ActionStatus::RUNNING; };
	Task::TaskExecutor ex(f.library, f.sink);
	ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 R_1 A_1/ C_1: C_N/ C_2: C_N/ R_1: R_C C_1/ C_1: C_m battery.remaining ge 0.3 0/"
	                                 "A_1: A_TO 3 1/"));
	f.w.SetScalar("battery.remaining", 0.5, f.t);
	CHECK(f.Tick(ex).dispatched);
	f.w.SetScalar("battery.remaining", 0.2, f.t);
	auto r = f.Tick(ex);
	CHECK_FALSE(r.dispatched);
	CHECK(r.events.back().reason == FailReason::RUNTIME_CONDITION);
}

TEST_CASE("preemption keeps the iterator, and the start condition is evaluated again (spec 03 §8.6)") {
	Fixture f;
	Task::TaskExecutor ex(f.library, f.sink);
	ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1..3/ C_1: C_m go ge 1 0/ C_2: C_N/ A_1: A_L 1 1/ A_2: A_L 1 2/ A_3: A_N/"));
	f.w.SetScalar("go", 1, 0);
	f.Tick(ex);
	CHECK(ex.TopIterator() == 1);

	ex.Push(Test::OneTask(f.factory, "T: T_A op.2 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_L 4 4/"));
	CHECK(ex.Depth() == 2);
	f.Tick(ex);  // op.2 dispatches
	auto r = f.Tick(ex);  // op.2 ends; op.1 is queued again
	CHECK(HasEvent(r, "op.2", TaskState::SUCCEEDED));
	CHECK(HasEvent(r, "op.1", TaskState::QUEUED));

	SUBCASE("start condition still TRUE: it continues at the kept iterator") {
		auto resumed = f.Tick(ex);
		CHECK(HasEvent(resumed, "op.1", TaskState::IN_PROGRESS));
		REQUIRE(resumed.dispatched);
		CHECK(resumed.dispatched->entries[0].action.arg ==
		      Device::PackArgument(Device::ArgLayout::U32X2, std::vector<std::int64_t>{1, 2}));
	}
	SUBCASE("start condition FALSE now: the behaviour lookup runs again") {
		f.w.SetScalar("go", 0, f.t);
		auto r2 = f.Tick(ex);
		CHECK_FALSE(r2.dispatched);
		CHECK(r2.events.back().reason == FailReason::NO_BEHAVIOUR);  // no behaviour fulfils C_m here
	}
}

TEST_CASE("hover behaviours fulfil C_W and C_T (C_T in milliseconds)") {
	Fixture f;
	f.LoadUavLibrary();
	Task::TaskExecutor ex(f.library, f.sink);

	SUBCASE("C_W 1.5: in progress once 1.5 s have passed since the task started") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1 A_2/ C_1: C_W 1.5/ C_2: C_N/ A_1: A_L 1 1/ A_2: A_N/"));
		double started = -1;
		for (int i = 0; i < 10 && started < 0; ++i) {
			const double now = f.t;
			auto r = f.Tick(ex);
			if (HasEvent(r, "op.1", TaskState::IN_PROGRESS)) started = now;
		}
		CHECK(started == 2.0);  // hover ends at 1.5; the task looks again on the next tick
		CHECK(f.sink.log.front().action.entries[0].action.code == "A_HD");
	}
	SUBCASE("C_T 1500 ms: absolute mission time") {
		ex.Push(Test::OneTask(f.factory, "T: T_A op.1 1 0 C_1 C_2 A_1/ C_1: C_T 1500/ C_2: C_N/ A_1: A_N/"));
		std::string behaviour;
		double started = -1;
		for (int i = 0; i < 10 && started < 0; ++i) {
			const double now = f.t;
			auto r = f.Tick(ex);
			for (const auto& e : r.events)
				if (e.behaviour && e.state == TaskState::QUEUED) behaviour = e.task_id;
			if (HasEvent(r, "op.1", TaskState::IN_PROGRESS)) started = now;
		}
		CHECK(behaviour == "hover_until");
		CHECK(started == 2.0);
	}
}

TEST_CASE("complex tasks on one robot (spec 03 §6.1)") {
	Fixture f;
	Task::TaskExecutor ex(f.library, f.sink);
	auto run = [&](const std::string& text) {
		ex.Push(Test::OneTask(f.factory, text));
		std::vector<Task::TaskEvent> events;
		for (int i = 0; i < 50 && !ex.Empty(); ++i) {
			auto r = f.Tick(ex);
			events.insert(events.end(), r.events.begin(), r.events.end());
		}
		return events;
	};
	auto finished = [](const std::vector<Task::TaskEvent>& events) {
		std::vector<std::string> out;
		for (const auto& e : events)
			if (e.state == TaskState::SUCCEEDED || e.state == TaskState::FAILED)
				out.push_back(e.task_id + (e.state == TaskState::SUCCEEDED ? " ok" : " fail"));
		return out;
	};
	const std::string ok = "C_1: C_N/ C_2: C_N/ A_1: A_N/";
	const std::string bad = "C_1: C_N/ C_2: C_F/ A_1: A_N/";

	SUBCASE("T_S runs children in order and assigns ids p.i") {
		auto e = run("T: T_S op.40 1 0 C_1 C_2 T_1 T_2/ C_1: C_N/ C_2: C_N/ T_1: T_A 0 1 0 C_1 C_2 A_1/ " + ok +
		             " T_2: T_A 0 1 0 C_1 C_2 A_1/ " + ok);
		CHECK(finished(e) == std::vector<std::string>{"op.40.1 ok", "op.40.2 ok", "op.40 ok"});
	}
	SUBCASE("T_S: a failed child fails the parent") {
		auto e = run("T: T_S op.40 1 0 C_1 C_2 T_1 T_2/ C_1: C_N/ C_2: C_N/ T_1: T_A 0 1 0 C_1 C_2 A_1/ " + bad +
		             " T_2: T_A 0 1 0 C_1 C_2 A_1/ " + ok);
		CHECK(finished(e) == std::vector<std::string>{"op.40.1 fail", "op.40 fail"});
	}
	SUBCASE("T_L with k = 1: done when one child succeeds, the rest are cancelled") {
		auto e = run("T: T_L op.41 1 0 1 C_1 C_2 T_1..3/ C_1: C_N/ C_2: C_N/ T_1: T_A 0 1 0 C_1 C_2 A_1/ " + bad +
		             " T_2: T_A 0 1 0 C_1 C_2 A_1/ " + ok + " T_3: T_A 0 1 0 C_1 C_2 A_1/ " + ok);
		CHECK(finished(e) == std::vector<std::string>{"op.41.1 fail", "op.41.2 ok", "op.41 ok"});
	}
	SUBCASE("T_O tries children by effective priority, highest first") {
		auto e = run("T: T_O op.42 2 0 C_1 C_2 T_1..3/ C_1: C_N/ C_2: C_N/ T_1: T_A 0 1 0 C_1 C_2 A_1/ " + ok +
		             " T_2: T_A 0 3 0 C_1 C_2 A_1/ " + bad + " T_3: T_A 0 2 0 C_1 C_2 A_1/ " + ok);
		CHECK(finished(e) == std::vector<std::string>{"op.42.2 fail", "op.42.3 ok", "op.42 ok"});
	}
	SUBCASE("effective priority is the product of the ancestors' priorities") {
		auto task = Test::OneTask(f.factory, "T: T_S op.43 2 0 C_1 C_2 T_1/ C_1: C_N/ C_2: C_N/ T_1: T_S 0 3 0 C_1 C_2 T_1/ C_1: C_N/ C_2: C_N/"
		                                     "T_1: T_A 0 0.5 0 C_1 C_2 A_1/ " + ok);
		const auto& leaf = *static_cast<const Task::ComplexTask&>(*static_cast<const Task::ComplexTask&>(*task).Children()[0]).Children()[0];
		CHECK(leaf.Id() == "op.43.1.1");
		CHECK(leaf.EffectivePriority() == 3.0);
	}
}

TEST_CASE("the task source feeds an empty stack (spec 03 §8.1 step 1)") {
	Fixture f;
	Task::TaskExecutor ex(f.library, f.sink);
	int served = 0;
	ex.SetTaskSource([&]() -> std::unique_ptr<Task::Task> {
		if (served++ > 0) return nullptr;
		return Test::OneTask(f.factory, "T: T_A op.9 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_N/");
	});
	auto r = f.Tick(ex);
	CHECK(HasEvent(r, "op.9", TaskState::QUEUED));
	CHECK(HasEvent(r, "op.9", TaskState::SUCCEEDED));
	CHECK_FALSE(f.Tick(ex).dispatched);
}
