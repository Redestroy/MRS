// Spec 16 §7.1: the tree writer.
#include <set>
#include <string>

#include "doctest.h"

#include "../protocol/TestFiles.h"
#include "MockDevices.h"
#include "mrs/device/DeviceTree.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

using namespace MRS;

namespace {
	std::vector<Protocol::Record> WithoutVersion(const Protocol::Document& doc) {
		std::vector<Protocol::Record> out;
		for (const auto& r : doc.records)
			if (r.kind != '@') out.push_back(r);
		return out;
	}

	std::set<std::string> Names(const Device::DeviceTree& tree) {
		std::set<std::string> out;
		for (auto* n : tree.Nodes()) out.insert(n->Name());
		return out;
	}
}

TEST_CASE("the writer gives back the definition of the example robots") {
	Device::DeviceRegistry registry;
	Device::Uav::RegisterUavDevices(registry);
	for (const char* file : {"mavic_webots.mrsd", "mavic_delivery_webots.mrsd"}) {
		CAPTURE(file);
		const auto parsed = Protocol::Parse(Test::ReadFile(Test::ExamplesDir() / file));
		REQUIRE(parsed.Ok());
		const auto tree = Device::TreeBuilder(registry).Build(parsed.document);
		const auto written = tree.ToDocument();
		// Virtual nodes and registry defaults are left out, overrides are kept.
		CHECK(WithoutVersion(written) == WithoutVersion(parsed.document));
		// Canonical text that builds the same tree.
		const auto again = Protocol::Parse(Protocol::Write(written));
		REQUIRE(again.Ok());
		CHECK(Names(Device::TreeBuilder(registry).Build(again.document)) == Names(tree));
	}
}

TEST_CASE("the writer keeps part includes as includes") {
	const auto registry = Test::MockRegistry();
	const auto lib = Device::DeviceLibrary::FromSearchPath({Test::ExamplesDir() / "library"});
	const std::string text = "@: MRS 0.2/\n"
	                         "D: D_H bot head.default 2 robot_type rover id 1 D_1/\n"
	                         "D_1: D_J base mech_bus 0 D_1..2/\n"
	                         "D_1: D_X axle part.drive 1 left.motor.max_speed 120/\n"
	                         "D_2: D_X wheel part.motor_block 0 D_1/\n"
	                         "D_1: D_S spare encoder.mock 1 device \"spare\"/\n";
	const auto parsed = Protocol::Parse(text);
	REQUIRE(parsed.Ok());
	const Device::TreeBuilder builder(registry, &lib);
	const auto tree = builder.Build(parsed.document);
	CHECK(tree.Find("wheel.motor")->FromPart());
	CHECK_FALSE(tree.Find("wheel.spare")->FromPart());

	const auto written = tree.ToDocument();
	CHECK(WithoutVersion(written) == WithoutVersion(parsed.document));
	CHECK(Names(builder.Build(written)) == Names(tree));
	CHECK(Protocol::Write(written).find("part.drive") != std::string::npos);
}
