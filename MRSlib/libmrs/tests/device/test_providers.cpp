// Spec 16 §4: joints that provide ports, open port types and `via`.
#include <string>

#include "doctest.h"

#include "MockDevices.h"
#include "MockPlatform.h"
#include "mrs/BuildError.h"
#include "mrs/device/Robot.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

using namespace MRS;
using Port::PortType;

namespace {
	// A rover with a motor on the platform and two CAN buses, each with a motor at node12.
	const char* kRover = "@: MRS 0.2/\n"
	                     "D: D_H bot head.default 2 robot_type rover id 1 D_1/\n"
	                     "D_1: D_J base mech_bus 0 D_1..3/\n"
	                     "D_1: D_A pump motor.mock 1 device \"pump\"/\n"
	                     "D_2: D_J can0 bus 3 provider bus.mock bus can0 link \"/dev/ttyUSB0\" D_1/\n"
	                     "D_1: D_A left motor.mock 2 port_type CAN device \"node12\"/\n"
	                     "D_3: D_J can1 mech_bus 3 provider bus.mock bus can1 link \"/dev/ttyUSB1\" D_1/\n"
	                     "D_1: D_A right motor.mock 2 port_type CAN device \"node12\"/\n";

	Test::MockPlatform Host() {
		Test::MockPlatform p;
		p.Add(PortType::SIM, "pump");
		p.Add(PortType::UART, "/dev/ttyUSB0", "acme:canhub:1");
		p.Add(PortType::UART, "/dev/ttyUSB1", "acme:canhub:1");
		auto& buses = Test::Buses();
		buses.clear();
		buses["can0"].Add(PortType::CAN, "node12", "acme:motor:3");
		buses["can1"].Add(PortType::CAN, "node12", "acme:motor:3");
		return p;
	}

	Device::ActionMap Motor(const std::string& target, double speed) {
		return {{{target, {"A_MOT", *Device::PackArgument(Device::ArgLayout::F64, std::vector<double>{speed})}}}, false};
	}
}

TEST_CASE("port types are open names") {
	CHECK(PortType::FromName("ROS2"));
	CHECK(PortType::FromName("SIM") == PortType::SIM);
	CHECK(PortType::FromName("MAVLINK")->BuiltIn());
	CHECK_FALSE(PortType::FromName("ROS2")->BuiltIn());
	for (const char* bad : {"", "ros2", "2CAN", "MY_BUS", "ABCDEFGHIJKLMNOP"}) CHECK_MESSAGE(!PortType::FromName(bad), bad);
	CHECK(Protocol::Parse("@: MRS 0.2/\nP: P_A cam image ROS2 \"/camera/image\" 0/\n").Ok());
	CHECK_FALSE(Protocol::Parse("@: MRS 0.2/\nP: P_A cam image ros2 \"/camera/image\" 0/\n").Ok());
}

TEST_CASE("a joint provides the ports of its subtree") {
	const auto registry = Test::MockRegistry();
	auto host = Host();
	auto robot = Device::BuildRobot(kRover, registry, host);
	CHECK(robot.faults.empty());
	REQUIRE(robot.providers.size() == 2);

	// The same address on two buses is two ports.
	std::map<std::string, std::string> provider_of;
	for (const auto& a : robot.ports.assigned) provider_of[a.node + "." + a.requirement] = a.provider;
	CHECK(provider_of["pump.motor"] == "platform");
	CHECK(provider_of["can0.link"] == "platform");
	CHECK(provider_of["left.motor"] == "can0");
	CHECK(provider_of["right.motor"] == "can1");

	CHECK(robot.blocks.actuators.Dispatch(Motor("left", 3.0), 0) == Device::ActionStatus::DONE);
	CHECK(Test::Buses()["can0"].Get("node12", PortType::CAN).writes.size() == 1);
	CHECK(Test::Buses()["can1"].Get("node12", PortType::CAN).writes.empty());

	// The writer keeps the interface kind and the provider parameter.
	auto written = robot.tree.ToDocument();
	auto parsed = Protocol::Parse(kRover).document;
	written.records.erase(written.records.begin());
	parsed.records.erase(parsed.records.begin());
	CHECK(written == parsed);

	// Providers are updated each tick.
	Device::UpdateProviders(robot, 0.0);
	CHECK(dynamic_cast<Test::MockBus*>(robot.tree.Find("can0"))->updates == 1);

	// The written port map says which provider, and reads back.
	const auto doc = Port::PortManager::WritePortMap(robot.ports);
	const std::string text = Protocol::Write(doc);
	CHECK(text.find("P: P_A left motor CAN \"node12\" 2 identity \"acme:motor:3\" via can0/") != std::string::npos);
	const auto map = Port::PortMap::FromDocument(doc);
	auto host2 = Host();
	auto again = Device::BuildRobot(kRover, registry, host2, &map);
	CHECK(again.faults.empty());
	for (const auto& a : again.ports.assigned) CHECK(a.source == Port::AssignmentSource::PortMap);

	// `via` must name the provider the rule picks.
	const auto wrong = Port::PortMap::Parse("@: MRS 0.2/\nP: P_A left motor CAN \"node12\" 1 via can1/\n");
	CHECK_THROWS_AS(Device::BuildRobot(kRover, registry, host2, &wrong), BuildError);
}

TEST_CASE("an unavailable provider takes its subtree with it") {
	const auto registry = Test::MockRegistry();
	SUBCASE("the provider's own port is missing") {
		auto host = Host();
		host.Remove(PortType::UART, "/dev/ttyUSB0");
		auto robot = Device::BuildRobot(kRover, registry, host);
		std::map<std::string, std::string> faults;
		for (const auto& f : robot.faults) faults[f.node] = f.reason;
		CHECK(faults.size() == 2);
		CHECK(faults.count("can0"));
		CHECK(faults["left"] == "provider can0 is unavailable");
		CHECK(robot.blocks.actuators.Dispatch(Motor("right", 1.0), 0) == Device::ActionStatus::DONE);
	}
	SUBCASE("the provider does not start") {
		std::string text = kRover;
		text.replace(text.find("bus can1"), 8, "bus can1 fail T");
		text.replace(text.find("D_J can1 mech_bus 3"), 19, "D_J can1 mech_bus 4");
		auto host = Host();
		auto robot = Device::BuildRobot(text, registry, host);
		std::map<std::string, std::string> faults;
		for (const auto& f : robot.faults) faults[f.node] = f.reason;
		CHECK(faults["can1"] == "bus does not answer");
		CHECK(faults["right"] == "provider can1 is unavailable");
		CHECK_FALSE(faults.count("left"));
	}
}

TEST_CASE("provider and port type errors") {
	const auto registry = Test::MockRegistry();
	auto host = Host();
	// No provider offers ROS2.
	std::string ros = kRover;
	ros.replace(ros.find("port_type CAN device \"node12\"/\nD_3"), 13, "port_type ROS2");
	CHECK_THROWS_AS(Device::BuildRobot(ros, registry, host), BuildError);
	// A bus that offers it.
	std::string offered = ros;
	offered.replace(offered.find("D_J can0 bus 3"), 14, "D_J can0 bus 4 types ROS2");
	Test::Buses()["can0"].Add(Port::PortType::FromName("ROS2").value(), "node12");
	CHECK(Device::BuildRobot(offered, registry, host).faults.empty());
	// A provider joint must carry data.
	std::string mech = kRover;
	mech.replace(mech.find("D_J can0 bus"), 12, "D_J can0 mech");
	CHECK_THROWS_AS(Device::BuildRobot(mech, registry, host), BuildError);
	// The provider key must be a provider joint.
	std::string not_provider = kRover;
	not_provider.replace(not_provider.find("provider bus.mock bus can0"), 26, "provider motor.mock bus can0");
	CHECK_THROWS_AS(Device::BuildRobot(not_provider, registry, host), BuildError);
}
