// Spec 16 §7.3: the boot check compares the identities a port map saved with what is there now.
#include <string>

#include "doctest.h"

#include "MockDevices.h"
#include "MockPlatform.h"
#include "mrs/device/Robot.h"

using namespace MRS;
using Port::PortType;

namespace {
	const char* kBot = "@: MRS 0.2/\n"
	                   "D: D_H bot head.default 2 robot_type rover id 1 D_1/\n"
	                   "D_1: D_J base mech_bus 0 D_1/\n"
	                   "D_1: D_A m motor.mock 0/\n";
	const char* kMap = "@: MRS 0.1/\nP: P_A m motor SIM \"m1\" 1 identity \"acme:motor:3@A1\"/\n";

	const Device::CheckFinding* Finding(const Device::Robot& robot, const std::string& result) {
		for (const auto& f : robot.check)
			if (f.result == result) return &f;
		return nullptr;
	}

	double MaxSpeed(const Device::Robot& robot) {
		for (const auto& c : robot.tree.Find("m")->Capabilities())
			if (c.code == "A_MOT") return c.limits.at("max");
		return 0;
	}
}

TEST_CASE("the same device passes the boot check") {
	const auto registry = Test::MockRegistry();
	const auto map = Port::PortMap::Parse(kMap);
	Test::MockPlatform host;
	host.Add(PortType::SIM, "m1", "acme:motor:3@A1");
	Device::BuildOptions options;
	options.port_map = &map;
	auto robot = Device::BuildRobot(kBot, registry, host, options);
	CHECK(robot.check.empty());
	CHECK(robot.faults.empty());
	// The port map written back keeps the identity found.
	const auto again = Port::PortMap::FromDocument(Port::PortManager::WritePortMap(robot.ports));
	CHECK(again.Find("m", "motor")->params.Text("identity") == "acme:motor:3@A1");
}

TEST_CASE("another unit of the same model is swapped and used") {
	const auto registry = Test::MockRegistry();
	const auto map = Port::PortMap::Parse(kMap);
	Test::MockPlatform host;
	host.Add(PortType::SIM, "m1", "acme:motor:3@B2");
	Device::BuildOptions options;
	options.port_map = &map;
	auto robot = Device::BuildRobot(kBot, registry, host, options);
	const auto* f = Finding(robot, "swapped");
	REQUIRE(f);
	CHECK(f->node == "m");
	CHECK(f->expected == "acme:motor:3@A1");
	CHECK(f->found == "acme:motor:3@B2");
	CHECK(robot.faults.empty());
	const auto rec = f->ToRecord();
	CHECK(rec.code == "I_R");
	CHECK(rec.fields.at(0).s == "swapped");
}

TEST_CASE("a new revision of the same type is updated, and its mapping's parameters take effect") {
	const auto registry = Test::MockRegistry();
	const auto map = Port::PortMap::Parse(kMap);
	Test::MockPlatform host;
	host.Add(PortType::SIM, "m1", "acme:motor:4@A1");
	Device::BuildOptions options;
	options.port_map = &map;

	// From the registry alone: no new parameters.
	auto robot = Device::BuildRobot(kBot, registry, host, options);
	REQUIRE(Finding(robot, "updated"));
	CHECK(robot.faults.empty());
	CHECK(MaxSpeed(robot) == doctest::Approx(100));

	// The robot's identity map gives the new revision a higher speed; the robot is built again with it.
	const auto identities = Device::IdentityMap::Parse("@: MRS 0.2/\nI: I_M \"acme:motor:4\" motor.mock 1 max_speed 500/\n");
	options.identity_map = &identities;
	robot = Device::BuildRobot(kBot, registry, host, options);
	REQUIRE(Finding(robot, "updated"));
	CHECK(robot.faults.empty());
	CHECK(MaxSpeed(robot) == doctest::Approx(500));
	CHECK(host.Get("m1").open_count == 3);  // once per build

	// Parameters in the definition still win.
	const std::string defined = std::string(kBot).replace(std::string(kBot).find("motor.mock 0/"), 13, "motor.mock 1 max_speed 50/");
	robot = Device::BuildRobot(defined, registry, host, options);
	CHECK(MaxSpeed(robot) == doctest::Approx(50));
}

TEST_CASE("a device of another type is changed and left unavailable") {
	const auto registry = Test::MockRegistry();
	const auto map = Port::PortMap::Parse(kMap);
	Test::MockPlatform host;
	host.Add(PortType::SIM, "m1", "acme:encoder:1");
	Device::BuildOptions options;
	options.port_map = &map;
	auto robot = Device::BuildRobot(kBot, registry, host, options);
	REQUIRE(Finding(robot, "changed"));
	REQUIRE(robot.faults.size() == 1);
	CHECK(robot.faults[0].node == "m");
	CHECK(robot.faults[0].reason.find("acme:encoder:1") != std::string::npos);
	CHECK(host.Get("m1").open_count == 0);

	// An unknown device is changed too.
	host.Add(PortType::SIM, "m1", "zz:thing");
	robot = Device::BuildRobot(kBot, registry, host, options);
	CHECK(Finding(robot, "changed"));

	// Remapping it in the robot's identity map makes it an update (spec 16 §3.3).
	const auto identities = Device::IdentityMap::Parse("@: MRS 0.2/\nI: I_M \"zz:thing\" motor.mock 0/\n");
	options.identity_map = &identities;
	robot = Device::BuildRobot(kBot, registry, host, options);
	CHECK(Finding(robot, "updated"));
	CHECK(robot.faults.empty());
}

TEST_CASE("a device that is gone is missing, and known unused devices are extra") {
	const auto registry = Test::MockRegistry();
	const auto map = Port::PortMap::Parse(kMap);
	Test::MockPlatform host;
	host.Add(PortType::SIM, "m2", "acme:motor:3@C3");
	host.Add(PortType::SIM, "x", "zz:thing");
	host.Add(PortType::SIM, "d", "cyberbotics:display");
	Device::BuildOptions options;
	options.port_map = &map;
	const auto identities = Device::IdentityMap::Parse("@: MRS 0.2/\nI: I_M \"cyberbotics:display\" ignore 0/\n");
	options.identity_map = &identities;
	auto robot = Device::BuildRobot(kBot, registry, host, options);
	const auto* missing = Finding(robot, "missing");
	REQUIRE(missing);
	CHECK(missing->node == "m");
	CHECK(missing->expected == "acme:motor:3@A1");
	CHECK(robot.faults.size() == 1);

	// Unknown and ignored devices are not extra.
	REQUIRE(robot.check.size() == 2);
	const auto* extra = Finding(robot, "extra");
	REQUIRE(extra);
	CHECK(extra->node == "platform");
	CHECK(extra->expected == "m2");
	CHECK(extra->found == "acme:motor:3@C3");
}
