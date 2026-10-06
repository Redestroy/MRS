// WP3: worldview pipeline (spec 05 §5).
#include <cmath>
#include <set>
#include <string>

#include "doctest.h"

#include "../device/MockPlatform.h"
#include "../protocol/TestFiles.h"
#include "mrs/BuildError.h"
#include "mrs/device/Robot.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/protocol/Parser.h"
#include "mrs/world/UavProcessors.h"
#include "mrs/world/WorldviewRequirements.h"

using namespace MRS;
using Environment::View;
using Environment::WorldModel;

namespace {
	constexpr double kPi = 3.14159265358979323846;
	const std::set<std::string> kMavicViews = {"V_ATT", "V_RATE", "V_GEO", "V_MAG", "V_BAT", "V_PEER"};

	Protocol::Record MissionHeader() {
		auto parsed = Protocol::Parse("H: H_M m1 56.9496 24.1052 10 -500 500 -500 500 120 20 5 120 3 1 0 0 0 2 5 0 0 3 10 0 0/");
		REQUIRE(parsed.Ok());
		return parsed.document.records.at(0);
	}

	// Ground truth: a climbing circle, radius 20 m, 4 m/s, nose along the track.
	struct Truth {
		double x, y, z, vx, vy, vz, yaw;
	};
	Truth TruthAt(double t) {
		const double w = 4.0 / 20.0;
		const double a = w * t;
		return {20 * std::cos(a), 20 * std::sin(a), 5 + 0.5 * t, -4 * std::sin(a), 4 * std::cos(a), 0.5, a + kPi / 2};
	}

	// Views a Mavic would produce at time t: GNSS and compass every 32 ms, IMU every tick.
	std::vector<View> MavicViews(const Environment::GeoReference& geo, double t, bool gnss, bool compass) {
		const Truth s = TruthAt(t);
		std::vector<View> v;
		v.push_back({"V_ATT", t, {0.0, 0.0, s.yaw}, {}});
		v.push_back({"V_RATE", t, {0.0, 0.0, 0.2}, {}});
		if (gnss) {
			double lat, lon, alt;
			geo.ToGeodetic({s.x, s.y, s.z}, lat, lon, alt);
			v.push_back({"V_GEO", t, {lat, lon, alt}, {}});
		}
		if (compass) v.push_back({"V_MAG", t, {Environment::HeadingFromYaw(s.yaw)}, {}});
		return v;
	}

	double Get(const Environment::Worldview& w, const std::string& path, double t) {
		auto v = w.Scalar(path, t);
		REQUIRE_MESSAGE(v.has_value(), path);
		return *v;
	}

	double AngleDiff(double a, double b) {
		double d = std::fmod(a - b, 2 * kPi);
		if (d > kPi) d -= 2 * kPi;
		if (d < -kPi) d += 2 * kPi;
		return std::fabs(d);
	}
}

TEST_CASE("the minimum fields follow ground truth through the pipeline") {
	auto model = WorldModel::ForViews(kMavicViews);
	Environment::ApplyMissionHeader(MissionHeader(), 1, model.world, 0.0);
	const Environment::GeoReference geo(56.9496, 24.1052, 10);
	const double dt = 0.008;
	double max_pos = 0, max_alt = 0, max_heading = 0, max_vel = 0;
	for (int k = 0; k <= 2500; ++k) {
		const double t = k * dt;
		model.Update(MavicViews(geo, t, k % 4 == 0, k % 4 == 0), t);
		if (t < 2.0) continue;  // let the velocity filter settle
		const Truth s = TruthAt((k - k % 4) * dt);  // the last GNSS sample
		const auto& w = model.world;
		max_pos = std::max({max_pos, std::fabs(Get(w, "pose.enu.x", t) - s.x), std::fabs(Get(w, "pose.enu.y", t) - s.y),
		                    std::fabs(Get(w, "pose.enu.z", t) - s.z)});
		max_alt = std::max(max_alt, std::fabs(Get(w, "alt.agl", t) - s.z));
		max_heading = std::max(max_heading, AngleDiff(Get(w, "heading", t), Environment::HeadingFromYaw(TruthAt(t).yaw)));
		max_vel = std::max({max_vel, std::fabs(Get(w, "vel.enu.x", t) - s.vx), std::fabs(Get(w, "vel.enu.y", t) - s.vy),
		                    std::fabs(Get(w, "vel.enu.z", t) - s.vz)});
	}
	CHECK(max_pos < 1e-4);
	CHECK(max_alt < 1e-4);
	CHECK(max_heading < 0.03);  // compass at 32 ms, while the robot turns at 0.2 rad/s
	CHECK(max_vel < 0.1);

	const auto& w = model.world;
	const double t = 2500 * dt;
	CHECK(w.SelectedSource("pose.enu") == "gnss");
	CHECK(w.SelectedSource("alt.amsl") == "gnss");  // the stock Mavic has no barometer
	CHECK(w.SelectedSource("alt.agl") == "amsl");
	CHECK(w.SelectedSource("heading") == "compass");
	CHECK(w.Bool("airborne", t) == true);
	CHECK(w.Bool("landed", t) == false);
	CHECK(w.Bool("geofence.inside", t) == true);
	CHECK(Get(w, "layer.alt", t) == doctest::Approx(20));
	CHECK(Get(w, "time", t) == doctest::Approx(t));
	CHECK(Get(w, "rate.body.z", t) == doctest::Approx(0.2));
	REQUIRE(w.History("pose.enu.x"));
	CHECK(w.History("pose.enu.x")->Size() == 256);
}

TEST_CASE("the worldview falls back to the next source when one goes stale") {
	auto model = WorldModel::ForViews(kMavicViews);
	Environment::ApplyMissionHeader(MissionHeader(), 1, model.world, 0.0);
	const Environment::GeoReference geo(56.9496, 24.1052, 10);
	double t = 0;
	for (int k = 0; k < 50; ++k, t += 0.01) model.Update(MavicViews(geo, t, true, true), t);
	CHECK(model.world.SelectedSource("heading") == "compass");

	// The compass stops. After its max_age (0.2 s) the heading comes from the attitude.
	for (int k = 0; k < 30; ++k, t += 0.01) model.Update(MavicViews(geo, t, true, false), t);
	CHECK(model.world.SelectedSource("heading") == "attitude");
	CHECK(model.world.IsFresh("heading", t));
	CHECK(AngleDiff(*model.world.Scalar("heading", t), Environment::HeadingFromYaw(TruthAt(t - 0.01).yaw)) < 1e-9);

	// It comes back and is preferred again.
	model.Update(MavicViews(geo, t, true, true), t);
	CHECK(model.world.SelectedSource("heading") == "compass");

	// The source order is per robot.
	model.world.SetSourceOrder("heading", {"attitude", "compass"});
	t += 0.01;
	model.Update(MavicViews(geo, t, true, true), t);
	CHECK(model.world.SelectedSource("heading") == "attitude");

	// GNSS stops: pose.enu goes stale, with no other source.
	for (int k = 0; k < 80; ++k, t += 0.01) model.Update(MavicViews(geo, t, false, true), t);
	CHECK_FALSE(model.world.IsFresh("pose.enu", t));
	CHECK_FALSE(model.world.Scalar("alt.agl", t).has_value());
	CHECK_FALSE(model.world.Bool("airborne", t).has_value());
}

TEST_CASE("processors can be added and removed without touching the others") {
	const Environment::GeoReference geo(56.9496, 24.1052, 10);
	SUBCASE("removing the compass processor leaves the heading to the attitude") {
		auto model = WorldModel::ForViews(kMavicViews);
		REQUIRE(model.chain.Remove("CompassHeadingProcessor"));
		Environment::ApplyMissionHeader(MissionHeader(), 1, model.world, 0.0);
		for (int k = 0; k < 10; ++k) model.Update(MavicViews(geo, k * 0.01, true, true), k * 0.01);
		CHECK(model.world.SelectedSource("heading") == "attitude");
		CHECK(model.world.IsFresh("pose.enu", 0.09));
	}
	SUBCASE("a rangefinder and a barometer become the preferred altitude sources") {
		std::set<std::string> views = kMavicViews;
		views.insert({"V_RNG", "V_BARO"});
		auto model = WorldModel::ForViews(views);
		Environment::ApplyMissionHeader(MissionHeader(), 1, model.world, 0.0);
		for (int k = 0; k < 10; ++k) {
			const double t = k * 0.01;
			auto v = MavicViews(geo, t, true, true);
			v.push_back({"V_RNG", t, {7.5}, {}});
			v.push_back({"V_BARO", t, {17.0}, {}});
			model.Update(v, t);
		}
		CHECK(model.world.SelectedSource("alt.agl") == "range");
		CHECK(*model.world.Scalar("alt.agl", 0.09) == doctest::Approx(7.5));
		CHECK(model.world.SelectedSource("alt.amsl") == "baro");
		CHECK(model.world.OfferedSources("alt.amsl") == std::vector<std::string>{"baro", "gnss"});
	}
	SUBCASE("a robot without GNSS loses the processors that need it") {
		auto model = WorldModel::ForViews({"V_ATT", "V_RATE", "V_MAG", "V_BAT"});
		std::set<std::string> names;
		for (auto* p : model.chain.Order()) names.insert(p->Name());
		CHECK(names == std::set<std::string>{"Clock", "AttitudeProcessor", "RateProcessor", "CompassHeadingProcessor",
		                                     "AttitudeHeadingProcessor", "BatteryProcessor"});
	}
}

TEST_CASE("the chain enforces spec 05 §5 and orders by needs") {
	struct P : Environment::IViewProcessor {
		std::string name;
		std::vector<std::string> provides, needs;
		std::vector<Environment::Offer> offers;
		std::string Name() const override { return name; }
		std::vector<std::string> Subscriptions() const override { return {}; }
		std::vector<std::string> Provides() const override { return provides; }
		std::vector<std::string> Needs() const override { return needs; }
		std::vector<Environment::Offer> Offers() const override { return offers; }
	};
	auto make = [](std::string name, std::vector<std::string> provides, std::vector<std::string> needs,
	               std::vector<Environment::Offer> offers = {}) {
		auto p = std::make_unique<P>();
		p->name = std::move(name);
		p->provides = std::move(provides);
		p->needs = std::move(needs);
		p->offers = std::move(offers);
		return p;
	};

	Environment::ProcessorChain chain;
	chain.Add(make("c", {"z"}, {"y"}));
	chain.Add(make("b", {}, {"x"}, {{"y", "one"}}));
	chain.Add(make("a", {"x"}, {}));
	std::vector<std::string> order;
	for (auto* p : chain.Order()) order.push_back(p->Name());
	CHECK(order == std::vector<std::string>{"a", "b", "c"});

	chain.Add(make("d", {"x"}, {}));
	CHECK_THROWS_WITH_AS(chain.Build(), doctest::Contains("provided by both"), BuildError);
	chain.Remove("d");
	chain.Add(make("e", {}, {}, {{"x", "two"}}));
	CHECK_THROWS_WITH_AS(chain.Build(), doctest::Contains("offered"), BuildError);
	chain.Remove("e");
	chain.Add(make("f", {"w"}, {"z"}));
	chain.Remove("a");
	chain.Add(make("a", {"x"}, {"w"}));
	CHECK_THROWS_WITH_AS(chain.Build(), doctest::Contains("cycle"), BuildError);
}

TEST_CASE("the self model and the chain agree") {
	Device::DeviceRegistry registry;
	Device::Uav::RegisterUavDevices(registry);
	auto platform = Test::MockPlatform::Mavic();
	auto robot = Device::BuildRobot(Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsd"), registry, platform);
	auto model = WorldModel::ForViews(robot.self.views);
	std::vector<std::string> chain_names;
	for (const auto* p : model.chain.Processors()) chain_names.push_back(p->Name());
	CHECK(chain_names == robot.self.processors);
	CHECK(robot.self.Provides("alt.agl"));
	CHECK(robot.self.Provides("rate.body"));

	// End to end: device views from the mock platform into the worldview.
	Environment::ApplyMissionHeader(MissionHeader(), 1, model.world, 0.0);
	platform.Get("gps").Push({56.9496, 24.1052, 25.0});
	platform.Get("compass").Push({1.0, 0.0, 0.0});
	model.Update(robot.blocks.sensors.Sample(0.5), 0.5);
	CHECK(*model.world.Scalar("alt.agl", 0.5) == doctest::Approx(15.0));
	CHECK(std::fabs(*model.world.Scalar("pose.enu.x", 0.5)) < 1e-6);
	CHECK(*model.world.Scalar("heading", 0.5) == doctest::Approx(0.0));
}

TEST_CASE("peers, detections and flight state") {
	auto model = WorldModel::ForViews({"V_PEER", "V_DET", "V_POS3"});
	auto& w = model.world;
	Environment::ApplyMissionHeader(MissionHeader(), 1, w, 0.0);

	// A peer's STATE message, as in the spec example.
	auto parsed = Protocol::Parse("V: V_PEER 142.48 2 118.2 -39.1 20.4 1.5 -0.6 -0.9 0.71 op.17/");
	REQUIRE(parsed.Ok());
	auto peer = Environment::ViewFromRecord(parsed.document.records[0]);
	REQUIRE(peer);
	CHECK(Environment::ToRecord(*peer) == parsed.document.records[0]);
	model.Update({*peer, {"V_DET", 142.5, {10, 10, 0, 0.9}, "person"}, {"V_DET", 142.5, {11, 10, 0, 0.8}, "person"},
	              {"V_DET", 142.5, {40, 10, 0, 0.7}, "person"}},
	             142.5);
	CHECK(w.IsFresh("peer.r2.pose.enu", 142.6));
	CHECK(w.Id("peer.r2.task", 142.6) == std::string("op.17"));
	CHECK(w.Object("r2")->cls == "peer");
	CHECK(w.PathsWithPrefix("det.person.1.").size() == 4);  // enu x y z and confidence
	CHECK(*w.Scalar("det.person.1.enu.x", 142.6) == doctest::Approx(11));
	CHECK(w.Object("det.person.2"));
	CHECK_FALSE(w.Object("det.person.3"));

	// Local position source: landed after 1 s at rest on the ground, at home.
	double t = 200;
	for (int k = 0; k <= 150; ++k, t += 0.01) model.Update({{"V_POS3", t, {0.2, 0.1, 0.05}, {}}}, t);
	t -= 0.01;
	CHECK(w.SelectedSource("pose.enu") == "local");
	CHECK(w.Bool("landed", t) == true);
	CHECK(w.Bool("airborne", t) == false);
	CHECK(w.Bool("home", t) == true);
	model.Update({{"V_POS3", t + 0.5, {0.2, 0.1, 3.0}, {}}}, t + 0.5);
	CHECK(w.Bool("landed", t + 0.5) == false);
}

TEST_CASE("time series and requirements") {
	Environment::TimeSeries s(3);
	s.Add(0, 0);
	s.Add(1, 10);
	s.Add(0.5, 99);  // older: ignored
	s.Add(2, 30);
	CHECK(*s.At(1.5) == doctest::Approx(20));
	CHECK_FALSE(s.At(2.5).has_value());
	s.Add(3, 40);
	CHECK(s.Size() == 3);
	CHECK_FALSE(s.At(0.5).has_value());
	CHECK(s.Window(1.5, 3).size() == 2);

	Environment::Worldview w;
	w.SetVec3("pose.enu", 1, 2, 3, 10.0);
	Environment::WorldviewRequirements r{{"pose.enu", -1}, {"alt.agl", 1.0}};
	CHECK(r.Missing(w, 10.2) == std::vector<std::string>{"alt.agl"});
	w.SetScalar("alt.agl", 3, 10.0);
	CHECK(r.Satisfies(w, 10.2));
	CHECK_FALSE(r.Satisfies(w, 10.6));  // pose.enu max_age is 0.5 s
	CHECK(r.Satisfiable({"pose.enu", "alt.agl"}));
	CHECK_FALSE(r.Satisfiable({"pose.enu"}));
	CHECK(Environment::WorldviewRequirements{{"pose.enu.x", -1}}.Satisfiable({"pose.enu"}));
}
