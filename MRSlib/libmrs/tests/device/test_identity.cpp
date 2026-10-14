// Spec 16 §2-§3: identities, patterns, identity maps and resolution.
#include <string>

#include "doctest.h"

#include "../protocol/TestFiles.h"
#include "mrs/BuildError.h"
#include "mrs/device/DeviceRegistry.h"
#include "mrs/device/Identity.h"

using namespace MRS;
using Device::Identity;
using Device::IdentityPattern;

namespace {
	class Dummy : public Device::Sensor {
	public:
		void Sample(double, std::vector<Device::View>&) override {}
	};
}

TEST_CASE("identities follow vendor:model[:rev][@serial]") {
	auto id = Identity::Parse("acme:motor_block:2@A13F07");
	REQUIRE(id);
	CHECK(id->vendor == "acme");
	CHECK(id->model == "motor_block");
	CHECK(id->rev == "2");
	CHECK(id->serial == "A13F07");
	CHECK(id->ToString() == "acme:motor_block:2@A13F07");
	CHECK(Identity::Parse("cyberbotics:gps")->rev.empty());
	CHECK(Identity::Parse("bosch:bmi088@0x68")->serial == "0x68");

	for (const char* bad : {"", "ublox", "Ublox:neo", "ublox:", ":neo", "a:b:c:d", "a:b@", "a:b:*", "a b:c", "GPS"})
		CHECK_MESSAGE(!Identity::Parse(bad), bad);
}

TEST_CASE("patterns match with rev and serial optional, and rank by specificity") {
	const auto any_rev = *IdentityPattern::Parse("ublox:neo-m9n:*");
	const auto no_rev = *IdentityPattern::Parse("ublox:neo-m9n");
	const auto rev = *IdentityPattern::Parse("ublox:neo-m9n:4.04");
	const auto unit = *IdentityPattern::Parse("ublox:neo-m9n:4.04@77");
	const auto id = *Identity::Parse("ublox:neo-m9n:4.04@77");
	CHECK(any_rev.Matches(id));
	CHECK(no_rev.Matches(id));
	CHECK(rev.Matches(id));
	CHECK(unit.Matches(id));
	CHECK_FALSE(IdentityPattern::Parse("ublox:neo-m8n")->Matches(id));
	CHECK_FALSE(IdentityPattern::Parse("ublox:neo-m9n:4.05")->Matches(id));
	CHECK_FALSE(IdentityPattern::Parse("ublox:neo-m9n@78")->Matches(id));
	CHECK(any_rev.Specificity() == 0);
	CHECK(rev.Specificity() == 1);
	CHECK(unit.Specificity() == 2);
	CHECK(IdentityPattern::Parse("ublox:neo-m9n@78")->ToString() == "ublox:neo-m9n:*@78");

	// Version 0.1 free-form identities are compared exactly.
	CHECK(Device::IdentityMatches("GPS", "GPS"));
	CHECK_FALSE(Device::IdentityMatches("GPS", "cyberbotics:gps"));
	CHECK(Device::IdentityMatches("cyberbotics:gps", "cyberbotics:gps:r2025a"));
}

TEST_CASE("the example identity map resolves the most specific mapping") {
	const auto map = Device::IdentityMap::Parse(Test::ReadFile(Test::ExamplesDir() / "robot_example.mrsi"));
	REQUIRE(map.Entries().size() == 4);

	const auto* gnss = map.Find(*Identity::Parse("ublox:neo-m9n:4.04"));
	REQUIRE(gnss);
	CHECK(gnss->type == "gnss.serial");
	CHECK(gnss->params.Num("rate_hz", 0) == doctest::Approx(10));

	const auto* block = map.Find(*Identity::Parse("acme:motor_block:2@B00001"));
	REQUIRE(block);
	CHECK_FALSE(block->params.Has("encoder.offset"));
	const auto* calibrated = map.Find(*Identity::Parse("acme:motor_block:2@A13F07"));
	REQUIRE(calibrated);
	CHECK(calibrated->params.Num("encoder.offset", 0) == doctest::Approx(0.013));

	CHECK(map.Find(*Identity::Parse("cyberbotics:display"))->type == Device::kIgnoreType);
	CHECK(map.Find(*Identity::Parse("acme:motor_block:3")) == nullptr);

	// Written back, it parses to the same map.
	const auto again = Device::IdentityMap::FromDocument(map.ToDocument());
	CHECK(again.Entries().size() == 4);
	CHECK(again.Find(*Identity::Parse("acme:motor_block:2@A13F07"))->params.Has("encoder.offset"));
}

TEST_CASE("equally specific mappings that disagree are an error") {
	const auto map = Device::IdentityMap::Parse("@: MRS 0.2/\nI: I_M \"a:b\" x.default 0/\nI: I_M \"a:b:*\" y.default 0/\n");
	CHECK_THROWS_AS(map.Find(*Identity::Parse("a:b:1")), BuildError);
	const auto same = Device::IdentityMap::Parse("@: MRS 0.2/\nI: I_M \"a:b\" x.default 0/\nI: I_M \"a:b:*\" x.default 0/\n");
	CHECK(same.Find(*Identity::Parse("a:b:1"))->type == "x.default");
	CHECK_THROWS_AS(Device::IdentityMap::Parse("@: MRS 0.2/\nI: I_R missing gnss \"a:b\" \"\"/\n"), BuildError);
}

TEST_CASE("resolution order: robot map, descriptor, library, registry") {
	Device::DeviceRegistry registry;
	registry.Register<Dummy>("tof.default", {}, {"st:vl53l1x"});
	registry.Register<Dummy>("tof.fast", {}, {"st:vl53l1x:2"});
	registry.Register<Dummy>("tof.serial", {}, {"st:vl53l1x:*"});
	const auto library = Device::IdentityMap::Parse("@: MRS 0.2/\nI: I_M \"st:vl53l1x:3\" tof.default 1 range_m 3/\n");
	const auto robot = Device::IdentityMap::Parse("@: MRS 0.2/\nI: I_M \"st:vl53l1x@SN1\" tof.serial 0/\n");
	const std::string descriptor = "D: D_S x tof.fast 1 range_m 4/";
	Device::IdentityResolver resolver(registry, &robot, {&library});
	using Source = Device::IdentityMatch::Source;

	// The registry: the most specific declared pattern.
	auto m = resolver.Resolve("st:vl53l1x:2");
	REQUIRE(m);
	CHECK(m->source == Source::Registry);
	CHECK(m->type == "tof.fast");
	CHECK_THROWS_AS(resolver.Resolve("st:vl53l1x:1"), BuildError);  // tof.default and tof.serial tie

	// The library before the registry.
	m = resolver.Resolve("st:vl53l1x:3");
	CHECK(m->source == Source::Library);
	CHECK(m->params.Num("range_m", 0) == doctest::Approx(3));

	// The descriptor before the library.
	m = resolver.Resolve("st:vl53l1x:3", descriptor);
	CHECK(m->source == Source::Descriptor);
	CHECK(m->type == "tof.fast");
	CHECK(m->params.Num("range_m", 0) == doctest::Approx(4));
	REQUIRE(m->descriptor);

	// The robot's own map before everything.
	m = resolver.Resolve("st:vl53l1x:3@SN1", descriptor);
	CHECK(m->source == Source::RobotMap);
	CHECK(m->type == "tof.serial");

	// Unknown, or not in the identity format.
	CHECK_FALSE(resolver.Resolve("ti:ina219"));
	CHECK_FALSE(resolver.Resolve("Ranging"));
	CHECK_THROWS_AS(resolver.Resolve("ti:ina219", "D: D_H h head.default 0 D_1/\nD_1: D_S x tof.fast 0/"), BuildError);
	CHECK_THROWS_AS(Device::DeviceRegistry().Register<Dummy>("x.default", {}, {"not an identity"}), BuildError);
}
