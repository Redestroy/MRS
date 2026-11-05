// Package delivery (spec 18): the sling in the test simulator, the delivery generator and the
// delivery behaviours on one robot.
#include <cmath>
#include <regex>
#include <sstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "mrs/algorithms/MrsLayer.h"
#include "mrs/generators/Generators.h"
#include "mrs/task/BehaviourLibrary.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"
#include "mrs/sim/Team.h"

using namespace MRS;

namespace {
	Sim::RobotFiles SlingFiles() {
		return {Test::ReadFile(Test::ExamplesDir() / "mavic_sling_webots.mrsd"), Test::ReadFile(Test::ExamplesDir() / "mavic_sling_webots.mrsp"),
		        (Test::ExamplesDir() / "uav_behaviours.mrsb").string()};
	}

	// The piles of a job in a package world: each source's packages stacked bottom first.
	void LoadPiles(Sim::PackageWorld& world, const Generators::DeliveryJob& job, const std::vector<Generators::Point>& sources,
	               const Generators::DeliveryOptions& o) {
		for (std::size_t s = 0; s < sources.size(); ++s) {
			std::vector<Sim::Package> pile;
			for (const auto& e : job.manifest) {
				if (e.source != s) continue;
				Sim::Package p;
				p.id = e.package;
				p.dest = e.dest;
				p.area = static_cast<std::int64_t>(e.area + 1);
				p.mass = e.mass;
				p.height = o.package_height;
				pile.push_back(p);
			}
			world.AddPile(sources[s][0], sources[s][1], pile);
		}
		world.loop_height = o.loop_height;
	}

	std::string TimelineOf(const Generators::DeliveryJob& job, double at = 1.0) {
		std::vector<std::pair<double, Protocol::Record>> entries;
		for (const auto& t : job.tasks) entries.push_back({at, t});
		return Generators::Timeline(entries);
	}
}

TEST_CASE("delivery generator: one bound five-step unit per package, and a manifest") {
	Generators::DeliveryOptions o;
	o.seed = 7;
	const std::vector<Generators::Point> sources{{0, 0, 0}, {10, 0, 0}};
	const std::vector<Generators::Point> dests{{40, 0, 0}, {0, 40, 0}, {-30, 20, 0}};
	auto job = Generators::DeliveryTasks(5, sources, dests, o);
	REQUIRE(job.tasks.size() == 5);
	REQUIRE(job.manifest.size() == 5);
	CHECK(job.manifest[0].source == 0);
	CHECK(job.manifest[1].source == 1);
	for (const auto& e : job.manifest) {
		CHECK(e.mass >= o.mass_min);
		CHECK(e.mass <= o.mass_max);
		CHECK(e.area < dests.size());
	}
	const auto& t = job.tasks[0];
	CHECK(t.code == "T_S");
	CHECK(t.fields.at(0).s == "op.1");
	const std::string text = Protocol::Write(t);
	CHECK(text.find("R_K op.1.1") != std::string::npos);
	CHECK(text.find("R_K op.1.4") != std::string::npos);
	CHECK(text.find("payload.at_dest") != std::string::npos);
	CHECK(text.find("A_FN A_REL") != std::string::npos);
	// Three packages on source 1: the point above the pile clears the top loop by length + approach.
	CHECK(text.find("C_P3 0 0 3.85 ") != std::string::npos);
	CHECK(Protocol::Parse(text).Ok());
	CHECK(Generators::ManifestCsv(job).rfind("package,source,area,dest_x,dest_y,dest_z,mass\n1001,1,", 0) == 0);
	// The same seed gives the same job.
	CHECK(Generators::ManifestCsv(Generators::DeliveryTasks(5, sources, dests, o)) == Generators::ManifestCsv(job));
}

TEST_CASE("sling in the test simulator: the passive hook catches the top loop, and the load goes slack on the ground") {
	Sim::PackageWorld world;
	Sim::Package a, b;
	a.id = 1;
	b.id = 2;
	b.dest = {5, 0, 0};
	world.AddPile(0, 0, {a, b});
	CHECK(world.Top(world.packages[1]));
	CHECK_FALSE(world.Top(world.packages[0]));
	// The top loop is at 0.25 + 0.25 + 0.1 = 0.6 m.
	CHECK(world.Catch({0.1, 0, 0.62}) == &world.packages[1]);
	CHECK(world.Catch({0.5, 0, 0.62}) == nullptr);
	CHECK(world.Catch({0, 0, 0.35}) == nullptr);  // the lower loop is covered

	Sim::QuadSim sim;
	sim.world = &world;
	sim.pos = {0, 0, 2.6};  // hook at 0.6 m
	sim.Step();
	CHECK(sim.carried == 2);
	CHECK_FALSE(sim.carried_resting);  // hook at the loop: the package hangs just off the pile
	sim.pos = {5, 0, 1.0};
	sim.Step();
	CHECK(sim.carried_resting);  // the package stands on the ground: the line is slack
	CHECK(sim.HangingMass() == 0.0);
	sim.pos[2] = 4.0;
	sim.Step();
	CHECK(sim.HangingMass() == doctest::Approx(0.3));
	sim.pos[2] = 2.4;  // package bottom at 0.05 m
	sim.Step();
	sim.OpenHook();
	CHECK(sim.carried == 0);
	CHECK(world.packages[1].state == Sim::Package::State::PLACED);
	CHECK(world.packages[1].pos[2] == 0.0);
	CHECK(world.packages[1].drop_height == doctest::Approx(0.05).epsilon(0.05));
	sim.Step();
	CHECK(sim.carried == 0);  // a placed package is never caught again
}

TEST_CASE("one robot delivers three packages from a pile to destinations read at pickup") {
	Generators::DeliveryOptions o;
	o.seed = 3;
	const std::vector<Generators::Point> sources{{0, 0, 0}};
	const std::vector<Generators::Point> dests{{30, 20, 0}, {-25, 30, 0}};
	auto job = Generators::DeliveryTasks(3, sources, dests, o);
	Sim::PackageWorld world;
	LoadPiles(world, job, sources, o);

	Sim::TeamConfig c;
	c.mission.homes = {{0, -6, 0}};
	c.packages = &world;
	c.sling_length = o.sling_length;
	Sim::Team team(SlingFiles(), c);
	team.issuer.LoadTimeline(TimelineOf(job));
	const bool done = team.Run(600.0);
	for (const auto& [id, it] : team.issuer.Tasks()) {
		CAPTURE(id);
		CHECK(it.done.has_value());
		CHECK_FALSE(it.failed.has_value());
	}
	REQUIRE(done);
	CHECK(world.Placed() == 3);
	for (const auto& p : world.packages) {
		CAPTURE(p.id);
		CHECK(p.state == Sim::Package::State::PLACED);
		CHECK(std::hypot(p.pos[0] - p.dest[0], p.pos[1] - p.dest[1]) < 1.5);
		CHECK(p.drop_height < world.hard_drop);
		CHECK(p.carrier == 1);
	}
	CHECK_FALSE(team.robots[0]->sim.crashed);
	MESSAGE("makespan " << *team.issuer.Makespan() << " s");
}


TEST_CASE("a package whose unit is lost after the pickup is carried on by the robot's own task") {
	// The unit that hooked the package ends while the package is on the hook (here it is cancelled;
	// in Webots two robots had taken the same unit). The robot must not fly about with it: it
	// carries it on under a task of its own (spec 18 §7.5).
	Generators::DeliveryOptions o;
	o.seed = 3;
	const std::vector<Generators::Point> sources{{0, 0, 0}};
	const std::vector<Generators::Point> dests{{30, 20, 0}};
	auto job = Generators::DeliveryTasks(1, sources, dests, o);
	Sim::PackageWorld world;
	LoadPiles(world, job, sources, o);
	Sim::TeamConfig c;
	c.mission.homes = {{0, -6, 0}};
	c.packages = &world;
	c.sling_length = o.sling_length;
	Sim::Team team(SlingFiles(), c);
	team.issuer.LoadTimeline(TimelineOf(job));
	auto& r = *team.robots[0];
	while (team.Time() < 300.0 && world.packages[0].state != Sim::Package::State::CARRIED) team.Run(0.5, false);
	REQUIRE(world.packages[0].state == Sim::Package::State::CARRIED);
	team.Run(5.0, false);
	REQUIRE(r.ctl->CancelTask(r.ctl->CurrentTask(), team.Time()));
	while (team.Time() < 600.0 && world.packages[0].state != Sim::Package::State::PLACED) team.Run(1.0, false);
	CHECK(r.layer->Carries() == 1);
	team.Run(30.0, false);
	CHECK(r.layer->StateOf("op.1") == Algorithms::PoolState::DONE);  // the carry finished the lost unit
	CHECK(world.packages[0].state == Sim::Package::State::PLACED);
	CHECK(std::hypot(world.packages[0].pos[0] - 30, world.packages[0].pos[1] - 20) < 1.5);
	CHECK_FALSE(r.sim.crashed);
}

namespace {
	std::vector<Algorithms::ZoneSpec> ZonesOf(const Generators::DeliveryJob& job) {
		std::vector<Algorithms::ZoneSpec> out;
		for (const auto& z : job.zones) {
			Algorithms::ZoneSpec s;
			s.id = z.id;
			s.x = z.centre[0];
			s.y = z.centre[1];
			s.z = z.centre[2];
			s.radius = z.radius;
			s.hold_x = z.hold[0];
			s.hold_y = z.hold[1];
			out.push_back(s);
		}
		return out;
	}

	// Robots closer than `d` horizontally while both are below `below` m: a conflict at a zone.
	struct ZoneWatch {
		long conflicts = 0;
		void Check(const Sim::Team& team, double d = 2.0, double below = 7.0) {
			for (std::size_t a = 0; a < team.robots.size(); ++a)
				for (std::size_t b = a + 1; b < team.robots.size(); ++b) {
					const auto& p = team.robots[a]->sim.pos;
					const auto& q = team.robots[b]->sim.pos;
					if (p[2] > 0.3 && q[2] > 0.3 && p[2] < below && q[2] < below && std::hypot(p[0] - q[0], p[1] - q[1]) < d) ++conflicts;
				}
		}
	};
}

TEST_CASE("behaviour qualifiers: '_' matches one segment, and a predicate binds its number as target.arg") {
	CHECK(Task::QualifierMatches("zone._.mine", "zone.3.mine"));
	CHECK(Task::QualifierMatches("zone._.mine", "zone.dest.mine"));
	CHECK_FALSE(Task::QualifierMatches("zone._.mine", "zone.3.place"));
	CHECK_FALSE(Task::QualifierMatches("zone._.mine", "zone.mine"));
	CHECK(Task::QualifierMatches("payload.held", "payload.held"));
	CHECK_FALSE(Task::QualifierMatches("payload.held", "payload.grounded"));
}

TEST_CASE("three robots share one pile: they wait in line, and every package arrives") {
	Generators::DeliveryOptions o;
	o.seed = 11;
	o.zones = true;
	const std::vector<Generators::Point> sources{{0, 0, 0}};
	const std::vector<Generators::Point> dests{{35, 15, 0}, {-30, 25, 0}, {5, 45, 0}};
	auto job = Generators::DeliveryTasks(9, sources, dests, o);
	REQUIRE(job.zones.size() == 4);
	Sim::PackageWorld world;
	LoadPiles(world, job, sources, o);

	Sim::TeamConfig c;
	c.mission.homes = Sim::RowHomes(3, 4.0, -8.0);
	c.packages = &world;
	c.sling_length = o.sling_length;
	c.mrs.zones = ZonesOf(job);
	Sim::Team team(SlingFiles(), c);
	team.issuer.LoadTimeline(TimelineOf(job));
	ZoneWatch watch;
	bool done = false;
	for (int k = 0; k < 9000 && !done; ++k) {  // 900 s
		team.Run(0.1, false);
		watch.Check(team);
		done = team.Time() > 2.0 && team.issuer.Done();
	}
	for (const auto& [id, it] : team.issuer.Tasks()) {
		CAPTURE(id);
		CHECK(it.done.has_value());
		CHECK_FALSE(it.failed.has_value());
		if (it.failed) MESSAGE("failed: " << *it.failed << " at t " << team.Time());
	}
	REQUIRE(done);
	CHECK(world.Placed() == 9);
	int carriers[4] = {0, 0, 0, 0};
	for (const auto& p : world.packages) {
		CAPTURE(p.id);
		CHECK(std::hypot(p.pos[0] - p.dest[0], p.pos[1] - p.dest[1]) < 1.5);
		CHECK(p.drop_height < world.hard_drop);
		if (p.carrier >= 1 && p.carrier <= 3) ++carriers[p.carrier];
	}
	CHECK(carriers[1] > 0);
	CHECK(carriers[2] > 0);
	CHECK(carriers[3] > 0);
	CHECK(watch.conflicts == 0);
	for (auto& r : team.robots) CHECK_FALSE(r->sim.crashed);
	MESSAGE("makespan " << *team.issuer.Makespan() << " s, carried " << carriers[1] << "/" << carriers[2] << "/" << carriers[3]
	                    << ", breaches " << team.metrics.separation_breaches);
}

TEST_CASE("a robot low on battery recharges at a pad and finishes the deliveries") {
	Generators::DeliveryOptions o;
	o.seed = 5;
	o.zones = true;
	const std::vector<Generators::Point> sources{{0, 0, 0}};
	const std::vector<Generators::Point> dests{{30, 20, 0}, {-25, 30, 0}};
	auto job = Generators::DeliveryTasks(4, sources, dests, o);
	Sim::PackageWorld world;
	LoadPiles(world, job, sources, o);

	Sim::TeamConfig c;
	c.mission.homes = {{0, -8, 0}};
	c.packages = &world;
	c.mrs.zones = ZonesOf(job);
	Algorithms::ZoneSpec pad;
	pad.id = 101;
	pad.x = 8;
	pad.y = -8;
	pad.radius = 1.5;
	pad.hold_x = 8;
	pad.hold_y = -14;
	pad.pad = true;
	c.mrs.zones.push_back(pad);
	c.pads = {{8, -8, 0}};
	c.charge_w = 3000.0;  // a fast charger, so the test stays short
	c.recharge = true;
	Sim::Team team(SlingFiles(), c);
	team.robots[0]->sim.SetEnergy(0.34 * 50.0);
	team.issuer.LoadTimeline(TimelineOf(job));
	bool done = false;
	while (!done && team.Time() < 900.0) {
		team.Run(0.5, false);
		done = team.Time() > 2.0 && team.issuer.Done();
	}
	for (const auto& [id, it] : team.issuer.Tasks()) {
		CAPTURE(id);
		CHECK(it.done.has_value());
		CHECK_FALSE(it.failed.has_value());
	}
	REQUIRE(done);
	CHECK(world.Placed() == 4);
	auto& r = *team.robots[0];
	CHECK(r.sim.energy_charged_wh > 10.0);
	CHECK_FALSE(r.ctl->SwapLanded());
	CHECK_FALSE(r.ctl->Stopped());
	CHECK_FALSE(r.sim.crashed);
	MESSAGE("makespan " << *team.issuer.Makespan() << " s, charged " << r.sim.energy_charged_wh << " Wh");
}

namespace {
	// One package for (400, 0), drop areas at x = 150 and 280, one robot with a small battery and a
	// charging pad at home (spec 18 §7). Runs until the package is placed, stranded or 1800 s pass.
	struct LegRun {
		Sim::Package pkg;
		long planned = 0, stranded = 0;
		bool crashed = false;
	};
	LegRun RunLegs(double capacity_wh) {
		Generators::DeliveryOptions o;
		o.seed = 2;
		const std::vector<Generators::Point> sources{{0, 0, 0}};
		const std::vector<Generators::Point> dests{{400, 0, 0}};
		auto job = Generators::DeliveryTasks(1, sources, dests, o, "op", 1);
		Sim::PackageWorld world;
		LoadPiles(world, job, sources, o);

		Sim::TeamConfig c;
		c.mission.xmin = -50;
		c.mission.xmax = 450;
		c.mission.homes = {{0, -8, 0}};
		c.packages = &world;
		c.quad.capacity_wh = capacity_wh;
		c.mrs.drop_areas = {{150, 0, 5}, {280, 0, 5}};
		c.mrs.frame_mass = c.quad.mass;
		Algorithms::ZoneSpec pad;
		pad.id = 101;
		pad.x = 4;
		pad.y = -8;
		pad.radius = 1.5;
		pad.hold_x = 4;
		pad.hold_y = -14;
		pad.pad = true;
		c.mrs.zones = {pad};
		c.pads = {{4, -8, 0}};
		c.charge_w = 2000.0;
		c.recharge = true;
		auto files = SlingFiles();
		std::ostringstream cap;
		cap << "capacity_wh " << capacity_wh;
		files.definition = std::regex_replace(files.definition, std::regex("capacity_wh 50"), cap.str());
		Sim::Team team(files, c);
		team.issuer.LoadTimeline(TimelineOf(job));
		auto& r = *team.robots[0];
		while (team.Time() < 1800.0 && world.packages[0].state != Sim::Package::State::PLACED && r.layer->Stranded() == 0) team.Run(1.0, false);
		return {world.packages[0], r.layer->LegsPlanned(), r.layer->Stranded(), r.sim.crashed};
	}
}

TEST_CASE("a delivery out of range is left in drop areas and finished from there") {
	// 7 Wh (the sling Mavic descends at 1 m/s, with soft position gains): the full trip does not fit, so the package goes to the area at 150, then 280, then
	// to the destination, with recharges at home in between.
	const auto run = RunLegs(7.0);
	CHECK(run.planned >= 2);
	CHECK(run.pkg.legs >= 2);
	CHECK(run.stranded == 0);
	CHECK(run.pkg.state == Sim::Package::State::PLACED);
	CHECK(std::hypot(run.pkg.pos[0] - 400, run.pkg.pos[1]) < 1.5);
	CHECK_FALSE(run.crashed);
	MESSAGE("delivered at t " << run.pkg.t_placed << " s after " << run.pkg.legs << " intermediate drop(s)");
}

TEST_CASE("a package no robot can carry further stays in its drop area") {
	// 6.75 Wh: even a fully charged robot cannot take the package on from 280, so it is put down
	// there and no new delivery is posted.
	const auto run = RunLegs(6.75);
	CHECK(run.stranded == 1);
	CHECK(run.pkg.state == Sim::Package::State::PILE);
	CHECK(std::hypot(run.pkg.pos[0] - 280, run.pkg.pos[1]) < 5.0);
	CHECK_FALSE(run.crashed);
}

namespace {
	// One package from the pile at the origin to (130, 0); the operator at the origin with a 60 m
	// radio, three robots relaying (spec 18 §6). Returns when the issuer heard the release leaf's
	// M_DONE, or not at all, and when the package was placed.
	struct RelayRun {
		std::optional<double> heard;
		double placed = -1.0;
		std::size_t relays = 0;
		long out_of_range = 0;
	};
	RelayRun RunRelay(bool relay_tasks) {
		Generators::DeliveryOptions o;
		o.seed = 4;
		o.relay_range = relay_tasks ? 60.0 : 0.0;
		const std::vector<Generators::Point> sources{{0, 0, 0}};
		const std::vector<Generators::Point> dests{{130, 0, 0}};
		auto job = Generators::DeliveryTasks(1, sources, dests, o);
		Sim::PackageWorld world;
		LoadPiles(world, job, sources, o);

		Sim::TeamConfig c;
		c.mission.xmin = -50;
		c.mission.xmax = 180;
		c.mission.homes = {{-4, -6, 0}, {0, -6, 0}, {4, -6, 0}};
		c.packages = &world;
		c.radio_range = 60.0;
		c.operator_pos = {0, 0, 0};
		c.mrs.relay = true;
		Sim::Team team(SlingFiles(), c);
		team.issuer.LoadTimeline(TimelineOf(job));
		auto& pkg = world.packages[0];
		while (team.Time() < 400.0 && !(pkg.state == Sim::Package::State::PLACED && team.Time() > pkg.t_placed + 20.0)) team.Run(1.0, false);
		RelayRun r;
		r.placed = pkg.t_placed;
		r.relays = job.relays.size();
		r.out_of_range = team.air.out_of_range;
		const auto& leaves = team.issuer.Leaves();
		if (auto it = leaves.find("op.1.4"); it != leaves.end()) r.heard = it->second.done;
		return r;
	}
}

TEST_CASE("relay positions: the generator spaces relays so no link is longer than the spacing") {
	Generators::DeliveryOptions o;
	o.relay_range = 60.0;
	const auto points = Generators::RelayPoints({{35, 15, 0}, {130, 0, 0}, {-20, 110, 0}}, o);
	// (35, 15) is in range; 130 m needs two relays, 111.8 m needs two; none is shared.
	REQUIRE(points.size() == 4);
	CHECK(points[0][0] == doctest::Approx(130.0 / 3));
	CHECK(points[1][0] == doctest::Approx(260.0 / 3));
	for (const auto& p : points) CHECK(p[2] == doctest::Approx(o.relay_altitude));
	auto job = Generators::DeliveryTasks(2, {{0, 0, 0}}, {{130, 0, 0}}, o);
	REQUIRE(job.relays.size() == 2);
	REQUIRE(job.tasks.size() == 4);
	CHECK(job.tasks[0].fields.at(0).s == "relay.1");
	CHECK(Generators::RelayPoints({{40, 0, 0}}, o).empty());
}

TEST_CASE("relay positions: idle robots hold a chain, and the issuer hears the release at a far destination") {
	const auto with = RunRelay(true);
	REQUIRE(with.relays == 2);
	REQUIRE(with.placed > 0);
	REQUIRE(with.heard.has_value());
	CHECK(*with.heard - with.placed < 3.0);
	const auto without = RunRelay(false);
	REQUIRE(without.placed > 0);
	// Without relays the release M_DONE is sent out of range: the issuer hears it late or never.
	CHECK((!without.heard || *without.heard - without.placed > 10.0));
	MESSAGE("release heard " << (*with.heard - with.placed) << " s after placing with relays; without: "
	                         << (without.heard ? std::to_string(*without.heard - without.placed) + " s" : std::string("never")));
}
