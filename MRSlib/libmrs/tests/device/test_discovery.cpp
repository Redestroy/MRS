// Spec 16 §7.2: discovery proposes a definition and a port map from what the providers report.
#include <string>

#include "doctest.h"

#include "MockDevices.h"
#include "MockPlatform.h"
#include "mrs/device/Discovery.h"
#include "mrs/device/Robot.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

using namespace MRS;
using Port::PortType;

namespace {
	// A head record; its children are ignored.
	Protocol::Record Head() {
		const auto result = Protocol::Parse("@: MRS 0.2/\nD: D_H bot head.default 2 robot_type rover id 1 D_1/\nD_1: D_A old motor.mock 0/\n");
		REQUIRE_MESSAGE(result.Ok(), (result.Ok() ? std::string() : result.error->message));
		return result.document.records.back();
	}

	// A pump motor on the platform, two CAN hubs with a motor each, an unknown and an ignored device.
	Test::MockPlatform Host() {
		Test::MockPlatform p;
		p.Add(PortType::SIM, "Pump", "acme:motor:2");
		p.Add(PortType::UART, "/dev/ttyUSB0", "acme:canhub:1@S0");
		p.Add(PortType::UART, "/dev/ttyUSB1", "acme:canhub:1@S1");
		p.Add(PortType::SIM, "thing", "zz:thing");
		p.Add(PortType::SIM, "display", "cyberbotics:display");
		auto& buses = Test::Buses();
		buses.clear();
		buses["can0"].Add(PortType::CAN, "node12", "acme:motor:3");
		buses["can1"].Add(PortType::CAN, "node12", "acme:motor:3");
		return p;
	}

	// Which hub is which bus is the robot's own knowledge, by serial; revision 3 motors are on CAN.
	const char* kRobotMap = "@: MRS 0.2/\n"
	                        "I: I_M \"acme:canhub:1@S0\" bus.mock 2 name can0 bus can0/\n"
	                        "I: I_M \"acme:canhub:1@S1\" bus.mock 2 name can1 bus can1/\n"
	                        "I: I_M \"acme:motor:3\" motor.mock 1 port_type CAN/\n"
	                        "I: I_M \"cyberbotics:display\" ignore 0/\n";

	Device::ActionMap Motor(const std::string& target, double speed) {
		return {{{target, {"A_MOT", *Device::PackArgument(Device::ArgLayout::F64, std::vector<double>{speed})}}}, false};
	}
}

TEST_CASE("discovery proposes a tree and a port map that build") {
	const auto registry = Test::MockRegistry();
	const auto robot_map = Device::IdentityMap::Parse(kRobotMap);
	auto host = Host();
	Device::DiscoveryOptions options;
	options.robot_map = &robot_map;
	const auto report = Device::Discover(Head(), registry, host, options);

	const std::string mrsd = Protocol::Write(report.mrsd);
	// In scan order. Each provider's devices are under it, the platform's under `base`.
	CHECK(mrsd == "@: MRS 0.2/\n\n"
	              "D: D_H bot head.default 2 robot_type rover id 1 D_1/\n"
	              "D_1: D_J base mech_bus 0 D_1..3/\n"
	              "D_1: D_A pump motor.mock 0/\n"
	              "D_2: D_J can0 mech_bus 2 provider bus.mock bus can0 D_1/\n"
	              "D_1: D_A node12 motor.mock 1 port_type CAN/\n"
	              "D_3: D_J can1 mech_bus 2 provider bus.mock bus can1 D_1/\n"
	              "D_1: D_A motor_1 motor.mock 1 port_type CAN/\n");
	CHECK(Protocol::Write(report.mrsp) == "@: MRS 0.2/\n\n"
	                                      "P: P_A pump motor SIM \"Pump\" 1 identity \"acme:motor:2\"/\n\n"
	                                      "P: P_A can0 link UART \"/dev/ttyUSB0\" 1 identity \"acme:canhub:1@S0\"/\n\n"
	                                      "P: P_A node12 motor CAN \"node12\" 2 identity \"acme:motor:3\" via can0/\n\n"
	                                      "P: P_A can1 link UART \"/dev/ttyUSB1\" 1 identity \"acme:canhub:1@S1\"/\n\n"
	                                      "P: P_A motor_1 motor CAN \"node12\" 2 identity \"acme:motor:3\" via can1/\n");
	CHECK(Protocol::Write(report.unknown) == "@: MRS 0.2/\n\nI: I_U platform \"thing\" SIM \"zz:thing\"/\n");

	// The proposal builds, passes the boot check and drives the motors.
	const auto map = Port::PortMap::FromDocument(report.mrsp);
	Device::BuildOptions build;
	build.port_map = &map;
	build.identity_map = &robot_map;
	auto robot = Device::BuildRobot(report.mrsd, registry, host, build);
	CHECK(robot.faults.empty());
	CHECK(robot.check.empty());
	CHECK(robot.blocks.actuators.Dispatch(Motor("motor_1", 2.0), 0) == Device::ActionStatus::DONE);
	CHECK(Test::Buses()["can1"].Get("node12", PortType::CAN).writes.size() == 1);
}

TEST_CASE("without the robot's map, ports nothing resolves are reported, not guessed") {
	const auto registry = Test::MockRegistry();
	auto host = Host();
	const auto report = Device::Discover(Head(), registry, host);
	// The hubs resolve from the registry but their bus is unknown: they are proposed with no devices.
	// The display is unknown without the robot's `ignore`.
	CHECK(Protocol::Write(report.unknown) == "@: MRS 0.2/\n\n"
	                                         "I: I_U platform \"display\" SIM \"cyberbotics:display\"/\n\n"
	                                         "I: I_U platform \"thing\" SIM \"zz:thing\"/\n");
	const std::string mrsd = Protocol::Write(report.mrsd);
	CHECK(mrsd.find("D_J _dev_ttyusb0 mech_bus 1 provider bus.mock/") != std::string::npos);
	CHECK(mrsd.find("D_A pump motor.mock 0/") != std::string::npos);
}
