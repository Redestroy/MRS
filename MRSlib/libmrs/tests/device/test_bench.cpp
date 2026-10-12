// Spec 16 §8: bench mode.
#include <string>

#include "doctest.h"

#include "MockDevices.h"
#include "MockPlatform.h"
#include "mrs/BuildError.h"
#include "mrs/device/Bench.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/protocol/Parser.h"
#include "mrs/robot/RobotController.h"

using namespace MRS;
using Port::PortType;

namespace {
	// A motor and an encoder on the bench, and a lamp port no node holds.
	const char* kBench = "@: MRS 0.2/\n"
	                     "D: D_H bench head.default 2 robot_type rig id 1 D_1/\n"
	                     "D_1: D_J base mech_bus 0 D_1..2/\n"
	                     "D_1: D_A wheel motor.mock 2 device \"m1\" max_speed 50/\n"
	                     "D_2: D_S enc encoder.mock 1 device \"e1\"/\n";

	Test::MockPlatform Host() {
		Test::MockPlatform p;
		p.Add(PortType::SIM, "m1");
		p.Add(PortType::SIM, "e1");
		p.Add(PortType::SIM, "lamp");
		return p;
	}

	Device::Action Speed(double v) { return {"A_MOT", *Device::PackArgument(Device::ArgLayout::F64, std::vector<double>{v})}; }

	Test::MockMotor& Wheel(const Device::Bench& bench) { return *dynamic_cast<Test::MockMotor*>(bench.GetRobot().tree.Find("wheel")); }

	Device::BenchSettings Allow(double timeout, Device::TimeoutMode mode = Device::TimeoutMode::Default) {
		Device::BenchSettings s;
		s.allow = {"A_MOT"};
		s.timeout = timeout;
		s.on_timeout = mode;
		return s;
	}
}

TEST_CASE("a bench robot is only for the bench") {
	const auto registry = Test::MockRegistry();
	auto host = Host();
	CHECK_THROWS_AS(Device::Bench(Device::BuildRobot(kBench, registry, host), host), BuildError);

	Device::BuildOptions options;
	options.bench = true;
	auto robot = Device::BuildRobot(kBench, registry, host, options);
	Task::FunctionRegistry functions;
	CHECK_THROWS_AS(Robot::RobotController(robot, Task::BehaviourLibrary{}, functions), BuildError);
}

TEST_CASE("motion codes need the allow list, and values must be within limits") {
	const auto registry = Test::MockRegistry();
	auto host = Host();
	auto bench = Device::Bench::Build(kBench, registry, host);
	CHECK(bench.Apply("wheel", Speed(10)) == Device::ActionStatus::REJECTED);
	CHECK(host.Get("m1").writes.empty());

	auto allowed = Device::Bench::Build(kBench, registry, host, {}, Allow(5));
	CHECK(allowed.Apply("wheel", Speed(10)) == Device::ActionStatus::DONE);
	CHECK(allowed.Apply("wheel", Speed(60)) == Device::ActionStatus::REJECTED);   // max 50
	CHECK(allowed.Apply("wheel", Speed(-60)) == Device::ActionStatus::REJECTED);  // min -50
	CHECK(allowed.Apply("enc", Speed(10)) == Device::ActionStatus::REJECTED);     // not an actuator
	CHECK(allowed.Apply("any", Speed(20)) == Device::ActionStatus::DONE);         // routed by code
	CHECK(host.Get("m1").writes.size() == 2);

	// From an A record, as the C API sends it.
	const auto rec = Protocol::Parse("@: MRS 0.2/\nA: A_MOT 30/\n").document.records.back();
	CHECK(allowed.Apply("wheel", rec) == Device::ActionStatus::DONE);
	CHECK(host.Get("m1").writes.back() == std::vector<double>{30});

	// Settings from parameters.
	const auto s = Device::BenchSettings::FromParams({{"bench_timeout", Device::NumValue(0)},
	                                                  {"bench_on_timeout", Device::IdValue("continue")},
	                                                  {"allow", Device::IdValue("A_MOT")}});
	CHECK(s.timeout == 0);
	CHECK(s.on_timeout == Device::TimeoutMode::Continue);
	CHECK(s.allow.count("A_MOT"));
	CHECK_THROWS_AS(Device::BenchSettings::FromParams({{"bench_on_timeout", Device::IdValue("stop")}}), BuildError);
}

TEST_CASE("the command timeout runs the chosen behaviour") {
	const auto registry = Test::MockRegistry();
	auto host = Host();  // 8 ms steps
	auto steps = [](Device::Bench& b, int n) {
		for (int k = 0; k < n; ++k) b.Step();
	};

	SUBCASE("default: the actuator's own behaviour, once") {
		auto bench = Device::Bench::Build(kBench, registry, host, {}, Allow(0.1));
		REQUIRE(bench.Apply("wheel", Speed(10)) == Device::ActionStatus::DONE);
		steps(bench, 10);
		CHECK(Wheel(bench).timeouts == 0);
		steps(bench, 5);
		CHECK(Wheel(bench).timeouts == 1);
		CHECK(Wheel(bench).safes == 1);  // the mock's default is its safe state
		CHECK(host.Get("m1").writes.back() == std::vector<double>{0});
		steps(bench, 20);
		CHECK(Wheel(bench).timeouts == 1);
	}
	SUBCASE("a new command restarts the timeout, with its own settings") {
		auto bench = Device::Bench::Build(kBench, registry, host, {}, Allow(0.1));
		bench.Apply("wheel", Speed(10));
		steps(bench, 10);
		bench.Apply("wheel", Speed(12), 1.0, Device::TimeoutMode::Safe);
		steps(bench, 100);
		CHECK(Wheel(bench).safes == 0);
		steps(bench, 30);
		CHECK(Wheel(bench).timeouts == 0);
		CHECK(Wheel(bench).safes == 1);
	}
	SUBCASE("continue: the last action keeps being applied") {
		auto bench = Device::Bench::Build(kBench, registry, host, {}, Allow(0.1, Device::TimeoutMode::Continue));
		bench.Apply("wheel", Speed(10));
		steps(bench, 20);
		const int applied = Wheel(bench).applied;
		CHECK(applied > 1);
		steps(bench, 10);
		CHECK(Wheel(bench).applied == applied + 10);
		CHECK(host.Get("m1").writes.back() == std::vector<double>{10});
		// Until Stop().
		bench.Stop();
		steps(bench, 10);
		CHECK(Wheel(bench).applied == applied + 10);
		CHECK(host.Get("m1").writes.back() == std::vector<double>{0});
	}
	SUBCASE("a timeout of 0 is none") {
		auto bench = Device::Bench::Build(kBench, registry, host, {}, Allow(0));
		bench.Apply("wheel", Speed(10));
		steps(bench, 1000);
		CHECK(Wheel(bench).timeouts == 0);
		CHECK(Wheel(bench).safes == 0);
	}
}

TEST_CASE("sampling, nodes and raw ports") {
	const auto registry = Test::MockRegistry();
	auto host = Host();
	auto bench = Device::Bench::Build(kBench, registry, host);
	host.Get("e1").Push({1.5});
	bench.Step();
	host.Get("e1").Push({2.5});
	const auto views = bench.Sample("enc");
	REQUIRE(views.size() == 2);  // one from the step, one now
	CHECK(views[0].values == std::vector<double>{1.5});
	CHECK(views[1].values == std::vector<double>{2.5});
	CHECK(bench.Sample("enc").empty());

	const auto nodes = bench.Nodes();
	REQUIRE(nodes.size() == 4);
	CHECK(nodes[2].name == "wheel");
	CHECK(nodes[2].kind == "D_A");
	CHECK(nodes[2].type == "motor.mock");

	// A port no node holds opens; one a node holds exclusively does not.
	const int lamp = bench.OpenRaw("platform", PortType::SIM, "lamp");
	REQUIRE(lamp > 0);
	REQUIRE(bench.Raw(lamp));
	CHECK(bench.Raw(lamp)->Write({1}));
	CHECK(host.Get("lamp").writes.size() == 1);
	CHECK_THROWS_AS(bench.OpenRaw("platform", PortType::SIM, "m1"), BuildError);
	CHECK_THROWS_AS(bench.OpenRaw("can0", PortType::CAN, "x"), BuildError);
	CHECK(bench.OpenRaw("platform", PortType::SIM, "nothing") == -1);
	bench.CloseRaw(lamp);
	CHECK_FALSE(bench.Raw(lamp));
}

TEST_CASE("the flight control unit holds on a timeout and lands when made safe") {
	Device::DeviceRegistry registry;
	Device::Uav::RegisterUavDevices(registry);
	auto node = registry.Create("fcu.default", "fcu", {});
	auto& fcu = dynamic_cast<Device::Uav::FlightControlUnit&>(*node);
	Device::Uav::FlightState state;
	state.agl = 0.5;
	fcu.SetStateSource([&state](double) { return std::optional<Device::Uav::FlightState>(state); });

	fcu.Safe(0);  // on the ground: nothing
	CHECK(fcu.Mode() == Device::Uav::FcuMode::DISARMED);
	REQUIRE(fcu.Apply({"A_TO", *Device::PackArgument(Device::ArgLayout::F32X2, std::vector<double>{5, 1})}, 0) ==
	        Device::ActionStatus::RUNNING);
	fcu.OnTimeout(1);
	CHECK(fcu.Mode() == Device::Uav::FcuMode::FLY);
	CHECK(fcu.LastCode() == "A_HD");
	fcu.Safe(2);
	CHECK(fcu.Mode() == Device::Uav::FcuMode::LANDING);
	fcu.OnTimeout(3);  // landing goes on
	CHECK(fcu.Mode() == Device::Uav::FcuMode::LANDING);
}
