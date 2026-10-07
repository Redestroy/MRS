// WP4 flight tests (spec 08, plan §11 WP4 acceptance): one UAV in the test simulator flies
// its task list through the robot controller.
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "QuadSim.h"
#include "mrs/device/Robot.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/protocol/Parser.h"
#include "mrs/robot/RobotController.h"
#include "mrs/task/TaskFactory.h"

using namespace MRS;

namespace {
	// Fence x and y from -100 to 100, up to 60 m; layers from 20 m; robot 1 lives at (0, 0, 0).
	const char* kMission = "H: H_M m1 56.9496 24.1052 10 -100 100 -100 100 60 20 5 120 1 1 0 0 0/";

	Protocol::Record MissionHeader(const char* text = kMission) {
		auto parsed = Protocol::Parse(text);
		REQUIRE(parsed.Ok());
		return parsed.document.records.at(0);
	}

	// A take-off task, so a list can start with "fly to".
	std::string Liftoff(const std::string& id) {
		return "T: T_A " + id + " 1 0 C_1 C_2 A_1..2/\nC_1: C_N/\nC_2: C_? ?_1/\n?_1: airborne T/\nA_1: A_TO 5 1.5/\nA_2: A_N/\n";
	}

	// Fly to (x, y, z), then wait `wait` seconds.
	std::string FlyTo(const std::string& id, double x, double y, double z, double wait = 0) {
		std::ostringstream s;
		s << "T: T_A " << id << " 1 0 C_1 C_2 A_1" << (wait > 0 ? "..2" : "") << "/\nC_1: C_P3 " << x << " " << y << " " << z
		  << " 1 0.5 0 -1/\nC_2: C_N/\n";
		if (wait > 0) s << "A_1: A_W " << wait << "/\nA_2: A_N/\n";
		else s << "A_1: A_N/\n";
		return s.str();
	}

	std::string Land(const std::string& id) { return "T: T_A " + id + " 1 0 C_1 C_2 A_1/\nC_1: C_? ?_1/\n?_1: landed T/\nC_2: C_N/\nA_1: A_N/\n"; }

	struct Flight {
		explicit Flight(Test::QuadParams params = {}) : sim(params) {
			Device::Uav::RegisterUavDevices(devices);
			Task::RegisterUavFunctions(functions);
			Task::TaskFactory factory(functions);
			library.PopulateFromFile((Test::ExamplesDir() / "uav_behaviours.mrsb").string(), factory);
			const auto map = Port::PortMap::Parse(Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsp"));
			robot = Device::BuildRobot(Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsd"), devices, sim, &map);
			ctl = std::make_unique<Robot::RobotController>(robot, library, functions);
			ctl->SetMission(MissionHeader());
			ctl->SetJournal(&journal);
		}

		// Ticks until `done` is true or `seconds` have passed. Returns whether `done` became true.
		bool Run(double seconds, const std::function<bool()>& done, const std::function<void()>& each = {}) {
			const double end = sim.Time() + seconds;
			while (sim.Time() < end) {
				ctl->Tick(sim.Time());
				if (each) each();
				if (done()) return true;
				sim.Step();
			}
			return false;
		}

		bool RunList(double seconds) { return Run(seconds, [&] { return ctl->ListDone(); }); }

		// The first behaviour pushed to fulfil a start condition.
		std::string FirstBehaviourFor(const std::string& prefix) const {
			for (const auto& e : ctl->TaskLog())
				if (e.event.behaviour && e.event.state == Task::TaskState::QUEUED && e.event.task_id.rfind(prefix, 0) == 0)
					return e.event.task_id;
			return {};
		}

		bool Logged(const std::string& id, Task::TaskState s) const {
			for (const auto& e : ctl->TaskLog())
				if (e.event.task_id == id && e.event.state == s) return true;
			return false;
		}

		bool Raised(Robot::ControllerEvent ev) const {
			for (const auto& e : ctl->Events())
				if (e.event == ev) return true;
			return false;
		}

		double DistanceXY(double x, double y) const { return std::hypot(sim.pos[0] - x, sim.pos[1] - y); }

		Test::QuadSim sim;
		Device::DeviceRegistry devices;
		Task::FunctionRegistry functions;
		Task::BehaviourLibrary library;
		Device::Robot robot;
		std::ostringstream journal;
		std::unique_ptr<Robot::RobotController> ctl;
	};

	Device::ActionMap One(const char* code, double a, double b) {
		Device::ActionMap m;
		m.entries.push_back({"any", {code, *Device::PackArgument(Device::ArgLayout::F32X2, std::vector<double>{a, b})}});
		return m;
	}

	std::vector<double> Values(const Device::ActionMap& m) {
		return Device::UnpackReals(Device::ArgLayout::F32X2, m.entries.at(0).action.arg);
	}
}

TEST_CASE("one UAV flies liftoff, fly-to, LED flash and landing from task strings") {
	Flight f;
	REQUIRE(f.robot.faults.empty());
	REQUIRE(f.ctl->AddTasks(Test::ReadFile(Test::ExamplesDir() / "uav_single_flight.mrst")) == 3);
	double led_max = 0.0;
	REQUIRE(f.Run(150.0, [&] { return f.ctl->ListDone(); }, [&] {
		for (const auto& [name, v] : f.sim.leds) led_max = std::max(led_max, v);
	}));
	for (const char* id : {"sf.1", "sf.2", "sf.3"}) {
		CAPTURE(id);
		CHECK(f.ctl->StateOf(id) == Task::TaskState::SUCCEEDED);
	}
	CHECK(f.FirstBehaviourFor("flyto.") == "flyto.layered");  // the group default has the highest priority
	CHECK(f.Logged("land", Task::TaskState::SUCCEEDED));
	CHECK(led_max > 0.0);
	CHECK(f.DistanceXY(40, 20) < 1.5);
	CHECK(f.sim.pos[2] < 0.05);
	CHECK_FALSE(f.sim.crashed);
	CHECK_FALSE(f.ctl->Fcu()->Armed());
	const std::string j = f.journal.str();
	CHECK(j.find("J_E 0 start/") != std::string::npos);
	CHECK(j.find("end/") != std::string::npos);
	CHECK(j.find("J_T") != std::string::npos);
	CHECK(j.find("J_K") != std::string::npos);
	// The journal is valid protocol text.
	CHECK(Protocol::Parse(j).Ok());
}

TEST_CASE("each fly-to variant reaches its target") {
	for (const char* variant : {"flyto.layered", "flyto.direct", "flyto.altitude_first", "flyto.velocity"}) {
		CAPTURE(variant);
		Flight f;
		REQUIRE(f.ctl->Library().SetPriority(variant, 100));
		f.ctl->AddTasks(Liftoff("v.1") + FlyTo("v.2", 30, -20, 8, 1) + Land("v.3"));
		REQUIRE(f.RunList(150.0));
		CHECK(f.FirstBehaviourFor("flyto.") == variant);
		CHECK(f.ctl->StateOf("v.2") == Task::TaskState::SUCCEEDED);
		CHECK(f.ctl->StateOf("v.3") == Task::TaskState::SUCCEEDED);
		CHECK(f.DistanceXY(30, -20) < 1.5);
		CHECK_FALSE(f.sim.crashed);
	}
	CHECK_FALSE(Task::BehaviourLibrary().SetPriority("flyto.none", 1));
}

TEST_CASE("the safety supervisor keeps flight actions inside the fence and the limits") {
	Flight f;
	f.ctl->Tick(0.0);
	auto& safety = f.ctl->Safety();
	// Fence -100..100, 60 m high, margin 2 m.
	auto v = Values(safety.Filter(One("A_PXY", 500, -150), 0.0));
	CHECK(v[0] == doctest::Approx(98));
	CHECK(v[1] == doctest::Approx(-98));
	v = Values(safety.Filter(One("A_PZY", 100, 0.5), 0.0));
	CHECK(v[0] == doctest::Approx(58));
	CHECK(v[1] == doctest::Approx(0.5));
	v = Values(safety.Filter(One("A_TO", 80, 1), 0.0));
	CHECK(v[0] == doctest::Approx(58));
	v = Values(safety.Filter(One("A_VXY", 30, 40), 0.0));
	CHECK(std::hypot(v[0], v[1]) == doctest::Approx(8).epsilon(1e-4));
	v = Values(safety.Filter(One("A_VZY", -5, 0.2), 0.0));
	CHECK(v[0] == doctest::Approx(-2));
	// Inside the limits nothing changes.
	const auto in = One("A_PXY", 10, 20);
	CHECK(safety.Filter(in, 0.0).entries[0].action.arg == in.entries[0].action.arg);
	// Task targets: inside the shrunk fence and at least min_alt up.
	CHECK(safety.TargetAllowed({10, 10, 5}));
	CHECK_FALSE(safety.TargetAllowed({99, 0, 5}));
	CHECK_FALSE(safety.TargetAllowed({0, 0, 59}));
	CHECK_FALSE(safety.TargetAllowed({0, 0, 0.2}));
}

TEST_CASE("a task whose target is outside the geofence fails as impossible") {
	Flight f;
	f.ctl->AddTasks(Liftoff("g.1") + FlyTo("g.2", 150, 0, 10) + FlyTo("g.3", 10, 0, 6) + Land("g.4"));
	REQUIRE(f.RunList(120.0));
	CHECK(f.ctl->StateOf("g.2") == Task::TaskState::FAILED);
	CHECK(f.ctl->FailReasonOf("g.2") == Task::FailReason::IMPOSSIBLE);
	CHECK(f.ctl->StateOf("g.3") == Task::TaskState::SUCCEEDED);
	CHECK(f.ctl->StateOf("g.4") == Task::TaskState::SUCCEEDED);
}

TEST_CASE("leaving the geofence sends the UAV home to land, then the task resumes") {
	Flight f;
	f.ctl->SetMission(MissionHeader("H: H_M m1 56.9496 24.1052 10 -100 30 -100 100 60 20 5 120 1 1 0 0 0/"));
	f.ctl->Library().SetPriority("flyto.direct", 100);
	f.ctl->AddTasks(Liftoff("f.1") + FlyTo("f.2", 25, 0, 8, 20) + Land("f.3"));
	bool wind_on = false, wind_done = false;
	double land_x = 1e9, land_y = 1e9;
	std::size_t seen = 0;
	REQUIRE(f.Run(300.0, [&] { return f.ctl->ListDone(); }, [&] {
		// Gusts push the UAV out of the fence during its hold at (25, 0).
		if (!wind_on && !wind_done && f.ctl->Executor().TopIterator() == 0 && f.DistanceXY(25, 0) < 1.0 &&
		    f.ctl->StateOf("f.2") == Task::TaskState::IN_PROGRESS) {
			f.sim.wind = {3.0, 0, 0};
			wind_on = true;
		}
		if (wind_on && f.Raised(Robot::ControllerEvent::GEOFENCE)) {
			f.sim.wind = {0, 0, 0};
			wind_on = false;
			wind_done = true;
		}
		for (; seen < f.ctl->TaskLog().size(); ++seen) {
			const auto& e = f.ctl->TaskLog()[seen].event;
			if (e.task_id == "safety.land" && e.state == Task::TaskState::SUCCEEDED) {
				land_x = f.sim.pos[0];
				land_y = f.sim.pos[1];
			}
		}
	}));
	CHECK(f.Raised(Robot::ControllerEvent::GEOFENCE));
	CHECK(f.Logged("safety.return_home", Task::TaskState::SUCCEEDED));
	CHECK(std::hypot(land_x, land_y) < 1.5);  // landed at home
	CHECK(f.ctl->StateOf("f.2") == Task::TaskState::SUCCEEDED);  // resumed after the return
	CHECK(f.ctl->StateOf("f.3") == Task::TaskState::SUCCEEDED);
	CHECK_FALSE(f.sim.crashed);
}

TEST_CASE("low battery: finish the task when the energy allows, then land at home for a swap") {
	Flight f;
	f.ctl->Library().SetPriority("flyto.direct", 100);
	f.ctl->AddTasks(Liftoff("b.1") + FlyTo("b.2", 40, 0, 10, 3) + FlyTo("b.3", -40, 0, 10) + Land("b.4"));
	bool drained = false;
	REQUIRE(f.Run(200.0, [&] { return f.ctl->SwapLanded(); }, [&] {
		if (!drained && f.sim.pos[0] > 20.0) {
			f.sim.SetEnergy(14.9);  // 29.8 %: low on the next battery reading
			drained = true;
		}
	}));
	CHECK(f.Raised(Robot::ControllerEvent::BATTERY_LOW));
	CHECK(f.ctl->StateOf("b.2") == Task::TaskState::SUCCEEDED);  // finished first
	CHECK(f.ctl->StateOf("b.3") == Task::TaskState::IDLE);       // blocked
	CHECK(f.DistanceXY(0, 0) < 1.5);
	CHECK(f.sim.pos[2] < 0.05);
	CHECK(f.journal.str().find("swap_land/") != std::string::npos);
	CHECK(f.sim.Energy() > f.ctl->Resources().Reserve());
}

TEST_CASE("low battery without the energy for the task: return at once, and resume after the swap") {
	std::string journal;
	{
		Flight f;
		f.ctl->Library().SetPriority("flyto.direct", 100);
		f.ctl->AddTasks(Liftoff("r.1") + FlyTo("r.2", 30, 10, 8, 600) + FlyTo("r.3", 0, 30, 8) + Land("r.4"));
		bool drained = false;
		REQUIRE(f.Run(200.0, [&] { return f.ctl->SwapLanded(); }, [&] {
			if (!drained && f.ctl->StateOf("r.2") == Task::TaskState::IN_PROGRESS) {
				f.sim.SetEnergy(14.9);
				drained = true;
			}
		}));
		CHECK(f.Raised(Robot::ControllerEvent::BATTERY_LOW));
		CHECK(f.ctl->StateOf("r.2") != Task::TaskState::SUCCEEDED);  // preempted, not finished
		CHECK(f.ctl->StateOf("r.2") != Task::TaskState::FAILED);
		CHECK(f.DistanceXY(0, 0) < 1.5);
		journal = f.journal.str();
	}
	REQUIRE(journal.find("swap_land/") != std::string::npos);
	const auto state = Robot::ReadJournal(journal + "J: J_T 99 DONE 0 T_1");  // a partly written last record
	REQUIRE(state.ok);
	CHECK(state.mission == "m1");
	CHECK(state.stack == std::vector<std::string>{"r.2"});
	REQUIRE(state.Unfinished().size() == 3);

	// After the swap: a new controller on a full battery reads the journal and carries on.
	Flight g;
	std::string why;
	REQUIRE(g.ctl->Resume(journal, &why));
	REQUIRE(g.RunList(900.0));
	CHECK(g.Logged("safety.liftoff", Task::TaskState::SUCCEEDED));
	CHECK(g.ctl->StateOf("r.2") == Task::TaskState::SUCCEEDED);
	CHECK(g.ctl->StateOf("r.3") == Task::TaskState::SUCCEEDED);
	CHECK(g.ctl->StateOf("r.4") == Task::TaskState::SUCCEEDED);
	CHECK_FALSE(g.ctl->StateOf("r.1"));  // done before the swap
	CHECK(g.journal.str().find("restart/") != std::string::npos);

	// A journal of another mission is not loaded.
	Flight h;
	h.ctl->SetMission(MissionHeader("H: H_M m2 56.9496 24.1052 10 -100 100 -100 100 60 20 5 120 1 1 0 0 0/"));
	CHECK_FALSE(h.ctl->Resume(journal, &why));
	CHECK(why.find("m1") != std::string::npos);
}

TEST_CASE("critical battery: emergency landing where the UAV is") {
	Flight f;
	f.ctl->Library().SetPriority("flyto.direct", 100);
	f.ctl->AddTasks(Liftoff("c.1") + FlyTo("c.2", 50, 0, 10) + Land("c.3"));
	bool drained = false;
	double drained_x = 0;
	REQUIRE(f.Run(200.0, [&] { return f.ctl->SwapLanded(); }, [&] {
		if (!drained && f.sim.pos[0] > 25.0) {
			f.sim.SetEnergy(7.0);  // 14 %: low and critical at once
			drained = true;
			drained_x = f.sim.pos[0];
		}
	}));
	CHECK(f.Raised(Robot::ControllerEvent::BATTERY_CRITICAL));
	CHECK(f.Logged("safety.emergency_land", Task::TaskState::SUCCEEDED));
	for (const auto& e : f.ctl->TaskLog()) CHECK(e.event.task_id != "safety.return_home");
	CHECK(f.sim.pos[0] > drained_x - 2.0);  // down near where it was, not at home
	CHECK(f.sim.pos[2] < 0.05);
	CHECK_FALSE(f.sim.crashed);
}

TEST_CASE("the energy estimate is within 15 % of the simulated battery") {
	Flight f;
	f.ctl->Library().SetPriority("flyto.direct", 100);
	// Warm-up flight to calibrate the model, then the measured leg: out, hold, back, land.
	const std::string measured = FlyTo("e.4", 60, 30, 10, 20);
	f.ctl->AddTasks(Liftoff("e.1") + FlyTo("e.2", 40, 0, 10, 10) + FlyTo("e.3", 0, 0, 10, 5) + measured + FlyTo("e.5", 0, 0, 10) +
	                Land("e.6"));
	Task::TaskFactory factory(f.functions);
	const auto leg = factory.BuildTasks(measured);
	REQUIRE(leg.size() == 1);
	std::optional<double> predicted, start_energy;
	REQUIRE(f.Run(300.0, [&] { return f.ctl->ListDone(); }, [&] {
		if (!predicted && f.ctl->StateOf("e.4") != Task::TaskState::IDLE) {
			predicted = f.ctl->Resources().TaskCost(*leg[0], f.ctl->World(), f.sim.Time());
			start_energy = f.sim.Energy();
		}
	}));
	REQUIRE(predicted);
	const double used = *start_energy - f.sim.Energy();
	CAPTURE(*predicted);
	CAPTURE(used);
	const auto* model = dynamic_cast<const Robot::QuadEnergyModel*>(&f.ctl->Resources().Model());
	REQUIRE(model);
	CAPTURE(model->HoverPower());
	CHECK(model->Samples() > 20);
	CHECK(std::fabs(*predicted - used) <= 0.15 * used);
}

TEST_CASE("the energy model calibrates by recursive least squares") {
	Robot::QuadEnergyModel m;
	// Simulated truth: 50 W hover, c_v 0.01.
	for (int k = 0; k < 200; ++k) {
		const double v = (k % 5) * 2.0;
		m.Observe(50.0 * (1 + 0.01 * v * v), v, 0.0);
	}
	CHECK(m.HoverPower() == doctest::Approx(50).epsilon(0.01));
	CHECK(m.SpeedCoefficient() == doctest::Approx(0.01).epsilon(0.02));
	CHECK(m.Energy({{3600, 0, 0}}) == doctest::Approx(50).epsilon(0.01));
}
