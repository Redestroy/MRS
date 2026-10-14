// Spec 16 §6: group.default, parts and the device library.
#include <filesystem>
#include <fstream>
#include <set>
#include <string>

#include "doctest.h"

#include "../protocol/TestFiles.h"
#include "MockDevices.h"
#include "MockPlatform.h"
#include "mrs/BuildError.h"
#include "mrs/device/Robot.h"

using namespace MRS;
using Port::PortType;

namespace {
	Device::DeviceLibrary ExampleLibrary() { return Device::DeviceLibrary::FromSearchPath({Test::ExamplesDir() / "library"}); }

	std::set<std::string> Names(const Device::DeviceTree& tree) {
		std::set<std::string> out;
		for (auto* n : tree.Nodes()) out.insert(n->Name());
		return out;
	}

	Test::MockPlatform DrivePlatform() {
		Test::MockPlatform p;
		for (const char* name : {"left motor", "left encoder", "right motor", "right encoder", "motor", "encoder"}) p.Add(PortType::SIM, name);
		return p;
	}

	Device::ActionMap Motor(const std::string& target, double speed) {
		return {{{target, {"A_MOT", *Device::PackArgument(Device::ArgLayout::F64, std::vector<double>{speed})}}}, false};
	}
}

TEST_CASE("group.default is a complex device made of its explicit children") {
	const auto registry = Test::MockRegistry();
	auto platform = DrivePlatform();
	const std::string text = "@: MRS 0.2/\n"
	                         "D: D_H bot head.default 2 robot_type rover id 1 D_1/\n"
	                         "D_1: D_J base mech_bus 0 D_1/\n"
	                         "D_1: D_X wheel group.default 0 D_1..2/\n"
	                         "D_1: D_A motor motor.mock 1 device \"motor\"/\n"
	                         "D_2: D_S encoder encoder.mock 1 device \"encoder\"/\n";
	auto robot = Device::BuildRobot(text, registry, platform);
	CHECK(robot.faults.empty());
	CHECK(Names(robot.tree) == std::set<std::string>{"bot", "base", "wheel", "wheel.motor", "wheel.encoder"});
	CHECK(robot.tree.Find("wheel")->Capabilities().empty());
	CHECK(robot.tree.Find("wheel")->PortRequirements().empty());
	CHECK(robot.self.actions.count("A_MOT"));
	CHECK(robot.blocks.actuators.Dispatch(Motor("motor", 2.0), 0) == Device::ActionStatus::DONE);
	CHECK(platform.Get("motor").writes.back() == std::vector<double>{2.0});
}

TEST_CASE("the library loads parts and identity maps from its search path") {
	const auto lib = ExampleLibrary();
	REQUIRE(lib.Part("motor_block"));
	REQUIRE(lib.Part("drive"));
	CHECK(lib.Part("missing") == nullptr);
	REQUIRE(lib.Maps().size() == 1);
	CHECK(lib.Maps()[0]->Find(*Device::Identity::Parse("acme:motor_block:2"))->type == "part.motor_block");

	// The first folder on the search path wins.
	const auto dir = std::filesystem::temp_directory_path() / "mrs_test_library";
	std::filesystem::create_directories(dir / "parts");
	{
		std::ofstream out(dir / "parts" / "motor_block.mrsd");
		out << "@: MRS 0.2/\nD: D_X block group.default 0 D_1/\nD_1: D_A motor motor.mock 1 device \"other\"/\n";
	}
	const auto first = Device::DeviceLibrary::FromSearchPath({dir, Test::ExamplesDir() / "library", dir / "does_not_exist"});
	CHECK(first.Part("motor_block")->children.size() == 1);
	CHECK(first.Part("drive") != nullptr);
	std::filesystem::remove_all(dir);

	// A part is one D_X record.
	Device::DeviceLibrary bad;
	CHECK_THROWS_AS(bad.AddPart("x", "@: MRS 0.2/\nD: D_S s encoder.mock 0/\n"), BuildError);
	CHECK_THROWS_AS(bad.AddPart("x", "@: MRS 0.2/\nD: D_X a group.default 0/\n\nD: D_X b group.default 0/\n"), BuildError);
}

TEST_CASE("parts are included with overrides, and nested parts get scoped names") {
	const auto registry = Test::MockRegistry();
	const auto lib = ExampleLibrary();
	auto platform = DrivePlatform();
	Device::BuildOptions options;
	options.library = &lib;

	const std::string text = "@: MRS 0.2/\n"
	                         "D: D_H bot head.default 2 robot_type rover id 1 D_1/\n"
	                         "D_1: D_J base mech_bus 0 D_1/\n"
	                         "D_1: D_X axle part.drive 1 left.motor.max_speed 120/\n";
	// left.motor.max_speed: the key motor.max_speed on the node `left`, which is itself a part include.
	auto robot = Device::BuildRobot(text, registry, platform, options);
	CHECK(robot.faults.empty());
	CHECK(Names(robot.tree) == std::set<std::string>{"bot", "base", "axle", "axle.left", "axle.left.motor", "axle.left.encoder", "axle.right",
	                                                 "axle.right.motor", "axle.right.encoder"});
	CHECK(robot.tree.Find("axle")->Part() == "drive");
	CHECK(robot.tree.Find("axle.left")->Part() == "motor_block");
	CHECK(robot.tree.Find("axle.left")->TypeKey() == "group.default");
	CHECK(robot.tree.Find("left.motor")->GetParams().Num("max_speed", 0) == doctest::Approx(120));
	CHECK(robot.tree.Find("right.motor")->GetParams().Num("max_speed", 0) == doctest::Approx(300));
	CHECK(robot.tree.Find("motor") == nullptr);  // two motors: a short name must be unique

	// Each copy got its own port through motor.device.
	CHECK(robot.blocks.actuators.Dispatch(Motor("left.motor", 1.5), 0) == Device::ActionStatus::DONE);
	CHECK(platform.Get("left motor").writes.size() == 1);
	CHECK(platform.Get("right motor").writes.empty());

	// The include's own K and D sub-records apply to the root.
	const std::string extra = "@: MRS 0.2/\n"
	                          "D: D_H bot head.default 2 robot_type rover id 1 D_1/\n"
	                          "D_1: D_J base mech_bus 0 D_1/\n"
	                          "D_1: D_X wheel part.motor_block 0 D_1/\n"
	                          "D_1: D_S spare encoder.mock 1 device \"left encoder\"/\n";
	robot = Device::BuildRobot(extra, registry, platform, options);
	CHECK(robot.tree.Find("wheel.spare") != nullptr);
	CHECK(robot.tree.Find("wheel")->Children().size() == 3);
}

TEST_CASE("part include errors are build errors") {
	const auto registry = Test::MockRegistry();
	const auto lib = ExampleLibrary();
	auto platform = DrivePlatform();
	Device::BuildOptions options;
	options.library = &lib;
	auto build = [&](const std::string& line) {
		const std::string text = "@: MRS 0.2/\nD: D_H bot head.default 2 robot_type rover id 1 D_1/\nD_1: D_J base mech_bus 0 D_1/\n" + line;
		return Device::BuildRobot(text, registry, platform, options);
	};
	CHECK_NOTHROW(build("D_1: D_X w part.motor_block 0/\n"));
	CHECK_THROWS_AS(build("D_1: D_X w part.nothing 0/\n"), BuildError);
	CHECK_THROWS_AS(build("D_1: D_S w part.motor_block 0/\n"), BuildError);       // a part is included on D_X
	CHECK_THROWS_AS(build("D_1: D_X w part.motor_block 1 wheel.x 1/\n"), BuildError);  // no node `wheel` in the part
	CHECK_THROWS_AS(Device::BuildRobot("@: MRS 0.2/\nD: D_H bot head.default 2 robot_type rover id 1 D_1/\nD_1: D_J base mech_bus 0 D_1/\n"
	                                   "D_1: D_X w part.motor_block 0/\n",
	                                   registry, platform),
	                BuildError);  // no library

	Device::DeviceLibrary loop;
	loop.AddPart("a", "@: MRS 0.2/\nD: D_X a group.default 0 D_1/\nD_1: D_X b part.a 0/\n");
	options.library = &loop;
	CHECK_THROWS_AS(build("D_1: D_X w part.a 0/\n"), BuildError);
}
