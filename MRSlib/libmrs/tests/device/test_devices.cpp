// WP2: device tree, port assignment, blocks and self model (spec 04).
#include <cmath>
#include <set>
#include <string>

#include "doctest.h"

#include "../protocol/TestFiles.h"
#include "MockPlatform.h"
#include "mrs/BuildError.h"
#include "mrs/device/Robot.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"
#include "mrs/task/TaskFactory.h"

using namespace MRS;
using Device::ActionStatus;
using Port::PortType;

namespace {
	constexpr double kPi = 3.14159265358979323846;

	const Device::DeviceRegistry& Registry() {
		static const Device::DeviceRegistry registry = [] {
			Device::DeviceRegistry r;
			Device::Uav::RegisterUavDevices(r);
			return r;
		}();
		return registry;
	}

	std::string MavicText() { return Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsd"); }
	Port::PortMap MavicMap() { return Port::PortMap::Parse(Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsp")); }

	// The definition without one line; the joint's label list is given in place of D_1..6.
	std::string Without(const std::string& line_start, const std::string& joint_labels) {
		std::string text = MavicText();
		const auto at = text.find(line_start);
		REQUIRE(at != std::string::npos);
		text.erase(at, text.find('\n', at) + 1 - at);
		const auto range = text.find("D_1..6");
		text.replace(range, 6, joint_labels);
		return text;
	}

	Device::ActionMap One(const std::string& target, const std::string& code, std::vector<double> values) {
		const auto info = *Device::FindAction(code);
		return {{{target, {code, *Device::PackArgument(info.layout, values)}}}, false};
	}

	Device::ActionMap OneInt(const std::string& target, const std::string& code, std::vector<std::int64_t> values) {
		const auto info = *Device::FindAction(code);
		return {{{target, {code, *Device::PackArgument(info.layout, values)}}}, false};
	}

	std::set<std::string> Names(const Device::DeviceTree& tree) {
		std::set<std::string> out;
		for (auto* n : tree.Nodes()) out.insert(n->Name());
		return out;
	}
}

TEST_CASE("the Webots Mavic is built from its definition file and port map") {
	auto platform = Test::MockPlatform::Mavic();
	const auto map = MavicMap();
	auto robot = Device::BuildRobot(MavicText(), Registry(), platform, &map);

	CHECK(robot.faults.empty());
	CHECK(robot.ports.unresolved.empty());
	CHECK(Names(robot.tree) == std::set<std::string>{"uav", "frame", "quadrotor", "m_fl", "m_fr", "m_rl", "m_rr", "fcu", "imu",
	                                                 "gnss", "compass", "leds", "bat", "radio"});
	CHECK(robot.tree.Find("m_fl")->IsVirtual());
	CHECK_FALSE(robot.tree.Find("gnss")->IsVirtual());

	// Every assignment came from the port map, with its rates.
	CHECK(robot.ports.assigned.size() == map.entries.size());
	for (const auto& a : robot.ports.assigned) CHECK(a.source == Port::AssignmentSource::PortMap);
	CHECK(platform.Get("gps").opened_with.Num("rate_hz", 0) == doctest::Approx(31.25));
	CHECK(platform.Get("emitter").opened_with.Int("channel", 0) == 1);

	const auto& self = robot.self;
	CHECK(self.robot_type == "mavic2pro");
	CHECK(self.id == 1);
	CHECK(self.roles == std::vector<std::string>{"camera_uav"});
	CHECK(self.actions == std::set<std::string>{"A_MOT", "A_TO", "A_LD", "A_HD", "A_PXY", "A_PZY", "A_VXY", "A_VZY", "A_L"});
	CHECK(self.views == std::set<std::string>{"V_ATT", "V_RATE", "V_GEO", "V_MAG", "V_BAT", "V_PEER"});
	CHECK(self.Capacity("energy_wh") == doctest::Approx(50));
	bool broadcast = false;
	for (const auto& c : self.capabilities)
		if (c.kind == Device::CapabilityKind::Message && c.code == "broadcast") {
			broadcast = true;
			CHECK(c.node == "radio");
			CHECK(c.limits.at("range_m") == doctest::Approx(200));
		}
	CHECK(broadcast);
	for (const char* f : {"time", "geo.position", "pose.enu", "alt.amsl", "alt.agl", "att", "heading", "rate.body", "vel.enu",
	                      "acc.enu", "battery", "battery.low", "battery.critical", "airborne", "landed", "home", "geofence.inside",
	                      "peer"})
		CHECK_MESSAGE(self.Provides(f), f);

	// Overrides from the file: the K_A limit on the LEDs and the fcu limits from the quadrotor line.
	for (const auto& c : self.capabilities) {
		if (c.code == "A_L") CHECK(c.limits.at("rate_hz") == doctest::Approx(10));
		if (c.code == "A_TO") CHECK(c.limits.at("max_speed_xy") == doctest::Approx(8));
	}

	// The point task of spec 03 is within the robot's static profile.
	Task::FunctionRegistry functions;
	Task::TaskFactory factory{functions};
	auto tasks = factory.BuildTasks(Test::ReadFile(Test::ExamplesDir() / "uav_point_task.mrst"));
	REQUIRE(tasks.size() == 1);
	CHECK(tasks[0]->MeetsStaticRequirements(self.ToProfile()));
}

TEST_CASE("removing the GNSS line removes pose.enu from what the robot provides") {
	auto platform = Test::MockPlatform::Mavic();
	auto robot = Device::BuildRobot(Without("D_2: D_S gnss", "D_1 D_3..6"), Registry(), platform);
	CHECK(robot.faults.empty());
	CHECK(robot.tree.Find("gnss") == nullptr);
	CHECK_FALSE(robot.self.views.count("V_GEO"));
	for (const char* f : {"geo.position", "pose.enu", "vel.enu", "geofence.inside", "alt.agl", "airborne"})
		CHECK_FALSE_MESSAGE(robot.self.Provides(f), f);
	CHECK(robot.self.Provides("heading"));
	CHECK(robot.self.Provides("battery"));

	Task::FunctionRegistry functions;
	Task::TaskFactory factory{functions};
	auto tasks = factory.BuildTasks(Test::ReadFile(Test::ExamplesDir() / "uav_point_task.mrst"));
	CHECK_FALSE(tasks.at(0)->MeetsStaticRequirements(robot.self.ToProfile()));
}

TEST_CASE("without a port map, addresses come from the device parameters") {
	auto platform = Test::MockPlatform::Mavic();
	auto robot = Device::BuildRobot(MavicText(), Registry(), platform);
	CHECK(robot.faults.empty());
	for (const auto& a : robot.ports.assigned) CHECK(a.source == Port::AssignmentSource::Address);
	CHECK(robot.self.Provides("pose.enu"));

	// First-time setup: the written port map has the same addresses as the example map.
	const auto written = Port::PortMap::FromDocument(Port::PortManager::WritePortMap(robot.ports));
	const auto example = MavicMap();
	REQUIRE(written.entries.size() == example.entries.size());
	for (const auto& e : example.entries) {
		const auto* w = written.Find(e.node, e.requirement);
		REQUIRE_MESSAGE(w, (e.node + "." + e.requirement));
		CHECK(w->address == e.address);
		CHECK(w->type == e.type);
	}
	// It is valid protocol text.
	CHECK(Protocol::Parse(Protocol::Write(Port::PortManager::WritePortMap(robot.ports))).Ok());
}

TEST_CASE("a missing device makes its node unavailable, and dependants with it") {
	SUBCASE("no GPS on the robot") {
		auto platform = Test::MockPlatform::Mavic();
		platform.Remove(PortType::SIM, "gps");
		auto robot = Device::BuildRobot(MavicText(), Registry(), platform);
		REQUIRE(robot.faults.size() == 1);
		CHECK(robot.faults[0].node == "gnss");
		CHECK(robot.faults[0].reason.find("not present") != std::string::npos);
		CHECK_FALSE(robot.self.Provides("pose.enu"));
		CHECK(robot.self.Accepts("A_TO"));
	}
	SUBCASE("a motor that does not open takes the fcu with it") {
		auto platform = Test::MockPlatform::Mavic();
		platform.Get("front left propeller").opens = false;
		auto robot = Device::BuildRobot(MavicText(), Registry(), platform);
		std::set<std::string> faulted;
		for (const auto& f : robot.faults) faulted.insert(f.node);
		CHECK(faulted == std::set<std::string>{"m_fl", "fcu"});
		CHECK_FALSE(robot.self.Accepts("A_TO"));
		CHECK(robot.self.Accepts("A_MOT"));  // the other three motors
		CHECK(robot.blocks.actuators.Dispatch(One("any", "A_TO", {10, 1}), 0) == ActionStatus::REJECTED);
	}
}

TEST_CASE("port assignment never picks a port because it is free") {
	using Port::NodePorts;
	using Port::PortRequirement;
	auto req = [](std::string name, std::string address, std::string identity = {}, bool exclusive = true) {
		PortRequirement r{std::move(name), PortType::UART, std::move(address), exclusive, {}};
		if (!identity.empty()) r.params.Add("identity", Device::StrValue(identity));
		return r;
	};
	Test::MockPlatform platform;
	platform.Add(PortType::UART, "/dev/ttyUSB0", "gnss_ublox");
	platform.Add(PortType::UART, "/dev/ttyUSB1", "telemetry");
	platform.Add(PortType::UART, "/dev/ttyS0");
	Port::PortManager manager(platform);

	SUBCASE("any without an identity is unresolved, even with free ports") {
		auto report = manager.Assign({{"gnss", {req("gnss", "any")}}});
		CHECK(report.assigned.empty());
		REQUIRE(report.unresolved.size() == 1);
		CHECK(report.unresolved[0].reason.find("port map") != std::string::npos);
	}
	SUBCASE("a scan resolves any by identity") {
		auto report = manager.Assign({{"gnss", {req("gnss", "any", "gnss_ublox")}}});
		REQUIRE(report.assigned.size() == 1);
		CHECK(report.assigned[0].address == "/dev/ttyUSB0");
		CHECK(report.assigned[0].source == Port::AssignmentSource::Scan);
	}
	SUBCASE("an identity matching no port or several is unresolved") {
		platform.Add(PortType::UART, "/dev/ttyUSB2", "telemetry");
		auto report = manager.Assign({{"a", {req("x", "any", "lidar")}}, {"b", {req("y", "any", "telemetry")}}});
		CHECK(report.assigned.empty());
		CHECK(report.unresolved.size() == 2);
	}
	SUBCASE("the port map comes first, then the address") {
		auto map = Port::PortMap::Parse("P: P_A gnss gnss UART \"/dev/ttyS0\" 1 baud 9600/");
		auto report = manager.Assign({{"gnss", {req("gnss", "/dev/ttyUSB0")}}, {"radio", {req("link", "/dev/ttyUSB1")}}}, &map);
		REQUIRE(report.assigned.size() == 2);
		CHECK(report.assigned[0].address == "/dev/ttyS0");
		CHECK(report.assigned[0].source == Port::AssignmentSource::PortMap);
		CHECK(report.assigned[0].params.Num("baud", 0) == doctest::Approx(9600));
		CHECK(report.assigned[1].address == "/dev/ttyUSB1");
		CHECK(report.assigned[1].source == Port::AssignmentSource::Address);
	}
	SUBCASE("a port the scan does not list is not present") {
		auto report = manager.Assign({{"gnss", {req("gnss", "/dev/ttyACM0")}}});
		REQUIRE(report.unresolved.size() == 1);
		CHECK(report.unresolved[0].reason.find("not present") != std::string::npos);
	}
	SUBCASE("an exclusive port given twice is a build error") {
		CHECK_THROWS_AS(manager.Assign({{"a", {req("x", "/dev/ttyS0")}}, {"b", {req("y", "/dev/ttyS0")}}}), BuildError);
		CHECK_NOTHROW(manager.Assign({{"a", {req("x", "/dev/ttyS0", {}, false)}}, {"b", {req("y", "/dev/ttyS0", {}, false)}}}));
	}
	SUBCASE("bad port map entries are build errors") {
		auto unknown = Port::PortMap::Parse("P: P_A gnss nmea UART \"/dev/ttyS0\" 0/");
		CHECK_THROWS_AS(manager.Assign({{"gnss", {req("gnss", "any")}}}, &unknown), BuildError);
		auto wrong_type = Port::PortMap::Parse("P: P_A gnss gnss I2C \"0x42\" 0/");
		CHECK_THROWS_AS(manager.Assign({{"gnss", {req("gnss", "any")}}}, &wrong_type), BuildError);
		CHECK_THROWS_AS(Port::PortMap::Parse("P: P_A gnss gnss UART any 0/"), BuildError);
	}
}

TEST_CASE("action dispatch follows spec 04 §4.2") {
	auto platform = Test::MockPlatform::Mavic();
	auto robot = Device::BuildRobot(MavicText(), Registry(), platform);
	auto& sink = robot.blocks.actuators;
	auto* fcu = dynamic_cast<Device::Uav::FlightControlUnit*>(robot.tree.Find("fcu"));
	REQUIRE(fcu);

	CHECK(sink.Dispatch(One("any", "A_TO", {10, 1}), 0) == ActionStatus::RUNNING);
	CHECK(sink.Dispatch(One("any", "A_PXY", {120, -40}), 0) == ActionStatus::DONE);
	CHECK(fcu->Setpoints().at("A_PXY") == std::vector<double>{120, -40});

	// Four motors share A_MOT and none is default: no `any` route; by name it works.
	CHECK(sink.Dispatch(One("any", "A_MOT", {400}), 0) == ActionStatus::REJECTED);
	CHECK(sink.Dispatch(One("m_fr", "A_MOT", {-410.5}), 0) == ActionStatus::DONE);
	CHECK(platform.Get("front right propeller").writes.back() == std::vector<double>{-410.5});

	// A named node that does not accept the code, or does not exist.
	CHECK(sink.Dispatch(One("leds", "A_PXY", {1, 2}), 0) == ActionStatus::REJECTED);
	CHECK(sink.Dispatch(One("m_xx", "A_MOT", {1}), 0) == ActionStatus::REJECTED);
	CHECK(sink.Dispatch(One("any", "A_W", {1}), 0) == ActionStatus::REJECTED);

	// A map with one bad entry applies nothing.
	const auto before = platform.Get("rear left propeller").writes.size();
	Device::ActionMap map = One("m_rl", "A_MOT", {300});
	map.entries.push_back(One("m_xx", "A_MOT", {300}).entries[0]);
	map.combined = true;
	CHECK(sink.Dispatch(map, 0) == ActionStatus::REJECTED);
	CHECK(platform.Get("rear left propeller").writes.size() == before);

	// A full motor map.
	Device::ActionMap motors;
	motors.combined = true;
	for (const char* m : {"m_fl", "m_fr", "m_rl", "m_rr"}) motors.entries.push_back(One(m, "A_MOT", {412.5}).entries[0]);
	CHECK(sink.Dispatch(motors, 0) == ActionStatus::DONE);
	CHECK(platform.Get("rear left propeller").writes.back() == std::vector<double>{412.5});

	// LEDs: bit k of the mask selects LED k.
	CHECK(sink.Dispatch(OneInt("any", "A_L", {2, 0x00FF00}), 0) == ActionStatus::DONE);
	CHECK(platform.Get("front left led").writes.back() == std::vector<double>{0});
	CHECK(platform.Get("front right led").writes.back() == std::vector<double>{0x00FF00});

	// Hold for 2 s.
	CHECK(sink.Dispatch(One("any", "A_HD", {2, 0}), 10.0) == ActionStatus::RUNNING);
	CHECK(sink.Dispatch(One("any", "A_HD", {2, 0}), 11.0) == ActionStatus::RUNNING);
	CHECK(sink.Dispatch(One("any", "A_HD", {2, 0}), 12.0) == ActionStatus::DONE);
	CHECK(sink.Dispatch(One("any", "A_HD", {0, 0}), 12.0) == ActionStatus::DONE);
}

TEST_CASE("two default actuators for one code is a build error") {
	const std::string text = R"(D: D_H uav head.default 2 robot_type t id 1 D_1 D_2/
D_1: D_A a led.webots 2 device "l1" default T/
D_2: D_A b led.webots 2 device "l2" default T/
)";
	Test::MockPlatform platform;
	platform.Add(PortType::SIM, "l1");
	platform.Add(PortType::SIM, "l2");
	CHECK_THROWS_AS(Device::BuildRobot(text, Registry(), platform), BuildError);
}

TEST_CASE("sensors, battery and radio go through their ports") {
	auto platform = Test::MockPlatform::Mavic();
	auto robot = Device::BuildRobot(MavicText(), Registry(), platform);
	CHECK(robot.blocks.sensors.Sample(0.0).empty());  // nothing sampled yet

	platform.Get("gps").Push({56.9496, 24.1052, 30.0});
	platform.Get("compass").Push({0.0, 1.0, 0.0});  // north is to the left: heading east
	platform.Get("inertial unit").Push({0.01, -0.02, 1.2});
	platform.Get("battery").Push({40.0});
	const auto views = robot.blocks.sensors.Sample(1.5);
	std::map<std::string, Device::View> by_code;
	for (const auto& v : views) by_code[v.code] = v;
	REQUIRE(by_code.size() == 4);
	CHECK(by_code["V_GEO"].values == std::vector<double>{56.9496, 24.1052, 30.0});
	CHECK(by_code["V_GEO"].stamp == doctest::Approx(1.5));
	CHECK(by_code["V_MAG"].values.at(0) == doctest::Approx(kPi / 2));
	CHECK(by_code["V_ATT"].values.at(2) == doctest::Approx(1.2));
	CHECK(by_code["V_BAT"].values == std::vector<double>{11.55, 0.8, 40.0});

	CHECK(Device::Uav::Compass::Heading(1, 0) == doctest::Approx(0));
	CHECK(Device::Uav::Compass::Heading(0, -1) == doctest::Approx(3 * kPi / 2));

	CHECK(robot.blocks.comms.Send("M: M_DONE 0.1 m1 r1 all 1 2 op.1/"));
	CHECK(platform.Get("emitter").sent.size() == 1);
	platform.Get("receiver").inbox.push_back("a");
	platform.Get("receiver").inbox.push_back("b");
	CHECK(robot.blocks.comms.Receive() == std::vector<std::string>{"a", "b"});
}

TEST_CASE("the PROFILE message carries the self model") {
	auto platform = Test::MockPlatform::Mavic();
	auto robot = Device::BuildRobot(MavicText(), Registry(), platform);
	Protocol::Document doc;
	doc.records.push_back(robot.self.ToProfileMessage("m1", 3, 1.0));
	const std::string text = Protocol::Write(doc, {true});
	CHECK(text.rfind("M: M_PROFILE 0.1 m1 r1 all 3 1 mavic2pro 1 camera_uav K_1", 0) == 0);
	auto parsed = Protocol::Parse(text);
	REQUIRE(parsed.Ok());
	CHECK(parsed.document == doc);
}

TEST_CASE("definition errors are build errors") {
	Test::MockPlatform platform;
	auto build = [&](const std::string& body) {
		return Device::BuildRobot("D: D_H uav head.default 2 robot_type t id 1 D_1/\n" + body, Registry(), platform);
	};
	CHECK_THROWS_WITH_AS(build("D_1: D_S g gnss.mavlink 0/\n"), doctest::Contains("unknown device type"), BuildError);
	CHECK_THROWS_WITH_AS(build("D_1: D_A g gnss.webots 0/\n"), doctest::Contains("not a D_A"), BuildError);
	CHECK_THROWS_WITH_AS(build("D_1: D_S g gnss.webots 0 K_1/\nK_1: K_V V_BARO 0/\n"), doctest::Contains("only overrides limits"),
	                     BuildError);
	CHECK_THROWS_WITH_AS(build("D_1: D_A leds led.webots 0 P_1/\nP_1: P_R SIM \"x\" T 0/\n"), doctest::Contains("req"), BuildError);
	CHECK_THROWS_WITH_AS(build("D_1: D_J j mech 1 accepts D_A D_1/\nD_1: D_S g gnss.webots 0/\n"), doctest::Contains("does not accept"),
	                     BuildError);
	CHECK_THROWS_WITH_AS(build("D_1: D_J j mech 1 max_children 1 D_1 D_2/\nD_1: D_S g gnss.webots 0/\nD_2: D_S c compass.webots 0/\n"),
	                     doctest::Contains("max_children"), BuildError);
	CHECK_THROWS_WITH_AS(build("D_1: D_J j mech 0 D_1 D_2/\nD_1: D_S g gnss.webots 0/\nD_2: D_S g compass.webots 0/\n"),
	                     doctest::Contains("used twice"), BuildError);
	CHECK_THROWS_WITH_AS(Device::BuildRobot("D: D_J j mech 0 D_1/\nD_1: D_S g gnss.webots 0/\n", Registry(), platform),
	                     doctest::Contains("root"), BuildError);
	CHECK_THROWS_WITH_AS(build("D_1: D_M b battery.webots 0/\n"), doctest::Contains("capacity_wh"), BuildError);

	// A P_R with req overrides one of several requirements.
	platform.Add(PortType::SIM, "x");
	platform.Add(PortType::SIM, "l1");
	auto robot = build("D_1: D_A leds led.webots 2 device \"l1\" device \"l2\" P_1/\nP_1: P_R SIM \"x\" T 1 req led1/\n");
	CHECK(robot.faults.empty());
	CHECK(robot.tree.Find("leds")->GetPort("led1")->Assignment().address == "x");
}
