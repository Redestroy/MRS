// WP7 tests (spec 11, plan §11 WP7 acceptance): the tree decomposer and its derived states, the
// task generators and their GeoJSON input, R_I, the camera, gimbal and payload devices, idle at
// layer, and a GeoJSON coverage polygon flown as one tree by three UAVs.
#include <algorithm>
#include <cmath>
#include <regex>
#include <set>
#include <string>
#include <vector>
#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "mrs/algorithms/TaskTree.h"
#include "mrs/generators/Generators.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"
#include "mrs/sim/Team.h"
#include "mrs/device/uav/UavDevices.h"

using namespace MRS;
using Algorithms::PoolState;

namespace {
	Protocol::Record FirstTask(const std::string& text) {
		auto r = Protocol::Parse(text);
		REQUIRE_MESSAGE(r.Ok(), (r.Ok() ? "" : r.error->message));
		for (const auto& rec : r.document.records)
			if (rec.kind == 'T') return rec;
		FAIL("no task");
		return {};
	}

	std::filesystem::path TreesDir() { return Test::ExamplesDir() / ".." / ".." / "experiments" / "uav_trees"; }

	Sim::MissionSpec Mission(int n) {
		Sim::MissionSpec m;
		m.homes = Sim::RowHomes(n);
		return m;
	}

	Environment::GeoReference Geo() {
		const auto m = Mission(1);
		return Environment::GeoReference(m.lat0, m.lon0, m.alt0);
	}

	Sim::RobotFiles Files(const char* definition = "mavic_webots") {
		const std::string d = definition;
		return {Test::ReadFile(Test::ExamplesDir() / (d + ".mrsd")), Test::ReadFile(Test::ExamplesDir() / (d + ".mrsp")),
		        (Test::ExamplesDir() / "uav_behaviours.mrsb").string()};
	}

	struct Team : Sim::Team {
		explicit Team(int n, const char* definition = "mavic_webots") : Sim::Team(Files(definition), Config(n)) {}
		static Sim::TeamConfig Config(int n) {
			Sim::TeamConfig c;
			c.mission = Mission(n);
			c.allocator = [](int) { return std::make_unique<Algorithms::RtaAllocator>(Algorithms::RtaConfig{}); };
			return c;
		}
		void Issue(const std::vector<Protocol::Record>& tasks, double at = 0.0) { issuer.Schedule(at, tasks); }
	};

	double Distance(const Generators::Point& a, const Generators::Point& b) { return std::hypot(a[0] - b[0], a[1] - b[1]); }

	// Leaf state from a map, AVAILABLE when not listed.
	Algorithms::TreeState::LeafState From(const std::map<std::string, PoolState>& m) {
		return [m](const std::string& id) {
			auto it = m.find(id);
			return it == m.end() ? PoolState::AVAILABLE : it->second;
		};
	}
}

// --- decomposer ------------------------------------------------------------------------------

TEST_CASE("the spec example tree decomposes into p.i leaves with precedence") {
	const auto tree = Algorithms::Decompose(FirstTask(Test::ReadFile(Test::ExamplesDir() / "uav_tree_task.mrst")));
	CHECK(tree.root == "op.40");
	CHECK(tree.leaves == std::vector<std::string>{"op.40.1.1", "op.40.1.2", "op.40.2"});
	CHECK(tree.Node("op.40.1").code == "T_L");
	CHECK(tree.Node("op.40.1.1").after.empty());
	CHECK(tree.Node("op.40.1.2").after.empty());
	// The landing leaf waits for the whole parallel node (rule 3).
	CHECK(tree.Node("op.40.2").after == std::vector<std::string>{"op.40.1"});
	// The leaf record carries its id and the priority product.
	CHECK(tree.Node("op.40.1.1").task.fields[0].s == "op.40.1.1");
	CHECK(Algorithms::FirstLeaves(tree, "op.40") == std::vector<std::string>{"op.40.1.1", "op.40.1.2"});
	CHECK(Algorithms::LeavesUnder(tree, "op.40.1").size() == 2);
}

TEST_CASE("complex start conditions gate the first leaves and R_K becomes affinity") {
	const std::string text = "@: MRS 0.1/\n"
	                         "T: T_S op.7 2 0 C_1 C_2 T_1 T_2/\nC_1: C_T 5000/\nC_2: C_N/\n"
	                         "T_1: T_A 0 1 0 C_1 C_2 A_1/\nC_1: C_N/\nC_2: C_N/\nA_1: A_N/\n"
	                         "T_2: T_A 0 0.5 0 C_1 C_2 R_1 A_1/\nC_1: C_N/\nC_2: C_N/\nR_1: R_K op.7.1/\nA_1: A_N/\n";
	const auto tree = Algorithms::Decompose(FirstTask(text));
	CHECK(tree.Node("op.7.1").gates.size() == 1);
	CHECK(tree.Node("op.7.1").gates[0].code == "C_T");
	CHECK(tree.Node("op.7.2").gates.empty());  // the root's start gates only its first leaves
	CHECK(tree.Node("op.7.2").affinity == "op.7.1");
	CHECK(tree.Node("op.7.2").task.fields[1].n == doctest::Approx(1.0));  // 2 × 0.5
}

TEST_CASE("trees without an id, reserved codes and k above the children are rejected") {
	const std::string leaf = "T_1: T_A 0 1 0 C_1 C_2 A_1/\nC_1: C_N/\nC_2: C_N/\nA_1: A_N/\n";
	CHECK_THROWS_AS(Algorithms::Decompose(FirstTask("@: MRS 0.1/\nT: T_S 0 1 0 C_1 C_2 T_1/\nC_1: C_N/\nC_2: C_N/\n" + leaf)),
	                Algorithms::DecomposeError);
	CHECK_THROWS_AS(Algorithms::Decompose(FirstTask("@: MRS 0.1/\nT: T_L op.1 1 0 2 C_1 C_2 T_1/\nC_1: C_N/\nC_2: C_N/\n" + leaf)),
	                Algorithms::DecomposeError);
	CHECK_THROWS_AS(Algorithms::Decompose(FirstTask(Test::ReadFile(Test::ExamplesDir() / "uav_point_task.mrst"))),
	                Algorithms::DecomposeError);
}

TEST_CASE("derived states of T_S, T_L and T_O") {
	const std::string leaf = "T_%: T_A 0 1 0 C_1 C_2 A_1/\nC_1: C_N/\nC_2: C_N/\nA_1: A_N/\n";
	auto leaves = [&](int n) {
		std::string s;
		for (int k = 1; k <= n; ++k) s += std::regex_replace(leaf, std::regex("%"), std::to_string(k));
		return s;
	};
	const auto seq = Algorithms::Decompose(FirstTask("@: MRS 0.1/\nT: T_S s.1 1 0 C_1 C_2 T_1..3/\nC_1: C_N/\nC_2: C_N/\n" + leaves(3)));
	const auto two_of = Algorithms::Decompose(FirstTask("@: MRS 0.1/\nT: T_L l.1 1 0 2 C_1 C_2 T_1..3/\nC_1: C_N/\nC_2: C_N/\n" + leaves(3)));
	const auto any = Algorithms::Decompose(FirstTask("@: MRS 0.1/\nT: T_O o.1 1 0 C_1 C_2 T_1..3/\nC_1: C_N/\nC_2: C_N/\n" + leaves(3)));
	Algorithms::TreeState s(seq), l(two_of), o(any);

	CHECK(s.Of("s.1", From({{"s.1.1", PoolState::DONE}, {"s.1.2", PoolState::DONE}})) == PoolState::AVAILABLE);
	CHECK(s.Of("s.1", From({{"s.1.1", PoolState::DONE}, {"s.1.2", PoolState::DONE}, {"s.1.3", PoolState::DONE}})) == PoolState::DONE);
	CHECK(s.Of("s.1", From({{"s.1.2", PoolState::FAILED}})) == PoolState::FAILED);

	CHECK(l.Of("l.1", From({{"l.1.1", PoolState::DONE}})) == PoolState::AVAILABLE);
	CHECK(l.Of("l.1", From({{"l.1.1", PoolState::DONE}, {"l.1.3", PoolState::DONE}})) == PoolState::DONE);
	CHECK(l.Of("l.1", From({{"l.1.1", PoolState::IMPOSSIBLE}})) == PoolState::AVAILABLE);
	CHECK(l.Of("l.1", From({{"l.1.1", PoolState::IMPOSSIBLE}, {"l.1.2", PoolState::CANCELLED}})) == PoolState::FAILED);

	CHECK(o.Of("o.1", From({{"o.1.2", PoolState::DONE}})) == PoolState::DONE);
	CHECK(o.Of("o.1", From({{"o.1.1", PoolState::FAILED}, {"o.1.2", PoolState::FAILED}})) == PoolState::AVAILABLE);
	CHECK(o.Of("o.1", From({{"o.1.1", PoolState::FAILED}, {"o.1.2", PoolState::FAILED}, {"o.1.3", PoolState::FAILED}})) == PoolState::FAILED);

	// A node whose end condition was FALSE keeps its override.
	s.Override("s.1", PoolState::FAILED);
	CHECK(s.Of("s.1", From({{"s.1.1", PoolState::DONE}, {"s.1.2", PoolState::DONE}, {"s.1.3", PoolState::DONE}})) == PoolState::FAILED);
}

// --- generators ------------------------------------------------------------------------------

TEST_CASE("coverage tracks are one swath apart less the overlap and cover the polygon") {
	const std::vector<Generators::Point> square{{0, 0, 0}, {100, 0, 0}, {100, 60, 0}, {0, 60, 0}};
	Generators::CoverageOptions o;
	o.footprint = 20;
	o.overlap = 0.25;
	const auto tracks = Generators::CoverageTracks(square, o);
	REQUIRE(tracks.size() >= 4);
	// Tracks run along the longest edge (x), 15 m apart, and alternate direction.
	for (std::size_t k = 0; k < tracks.size(); ++k) {
		CHECK(tracks[k].first[1] == doctest::Approx(tracks[k].second[1]));
		CHECK(tracks[k].first[2] == doctest::Approx(o.altitude));
		if (k > 0) {
			CHECK(std::fabs(tracks[k].first[1] - tracks[k - 1].first[1]) == doctest::Approx(15.0));
			CHECK((tracks[k].second[0] - tracks[k].first[0]) * (tracks[k - 1].second[0] - tracks[k - 1].first[0]) < 0);
		}
	}
	// The outer tracks lie within half a swath of the edges.
	std::vector<double> ys;
	for (const auto& t : tracks) ys.push_back(t.first[1]);
	CHECK(*std::min_element(ys.begin(), ys.end()) <= 10.0 + 1e-9);
	CHECK(*std::max_element(ys.begin(), ys.end()) >= 50.0 - 1e-9);

	o.cells = 2;
	const auto task = Generators::CoverageTask("op.3", square, o);
	CHECK(task.code == "T_L");
	const auto tree = Algorithms::Decompose(task);
	CHECK(tree.Node("op.3").children.size() == 2);
	// One robot flies a cell: every later leg is bound to the leg before it.
	for (const auto& leaf : Algorithms::LeavesUnder(tree, "op.3.1")) {
		const auto& n = tree.Node(leaf);
		if (leaf == "op.3.1.1") CHECK(n.affinity.empty());
		else CHECK(!n.affinity.empty());
		if (!n.affinity.empty()) CHECK(n.affinity.rfind("op.3.1.", 0) == 0);
	}
}

TEST_CASE("the search spiral keeps the pitch between turns and stays within the radius") {
	Generators::SearchOptions o;
	o.radius = 30;
	o.footprint = 10;
	o.overlap = 0.1;
	o.step = 4;
	const Generators::Point c{10, -20, 0};
	const auto pts = Generators::SpiralWaypoints(c, o);
	REQUIRE(pts.size() > 10);
	CHECK(Distance(pts.front(), c) < o.footprint);
	double last_r = 0;
	for (std::size_t k = 0; k < pts.size(); ++k) {
		const double r = Distance(pts[k], c);
		CHECK(r <= o.radius + 1e-6);
		CHECK(r + 1e-6 >= last_r);  // outwards only
		last_r = r;
		CHECK(pts[k][2] == doctest::Approx(o.altitude));
		if (k > 1) CHECK(Distance(pts[k], pts[k - 1]) <= o.step + 1e-6);  // the first leg is half a pitch out
	}
	CHECK(last_r >= o.radius - o.footprint * (1 - o.overlap));
	// Archimedean: after one full turn the radius grows by footprint × (1 − overlap).
	const double pitch = o.footprint * (1 - o.overlap);
	int compared = 0;
	for (std::size_t k = 1; k < pts.size(); ++k)
		for (std::size_t j = k + 1; j < pts.size(); ++j) {
			const double rk = Distance(pts[k], c), rj = Distance(pts[j], c);
			const double a = std::atan2(pts[k][1] - c[1], pts[k][0] - c[0]), b = std::atan2(pts[j][1] - c[1], pts[j][0] - c[0]);
			if (std::fabs(std::remainder(a - b, 2 * 3.14159265358979)) < 0.01 && rj - rk > pitch / 2 && rj - rk < 1.5 * pitch) {
				CHECK(rj - rk == doctest::Approx(pitch).epsilon(0.02));
				++compared;
			}
		}
	CHECK(compared > 0);
	const auto two = Generators::SearchTask("op.9", {c, {60, 40, 0}}, o);
	CHECK(two.code == "T_L");
	CHECK(Algorithms::Decompose(two).Node("op.9").children.size() == 2);
}

TEST_CASE("the perimeter is closed and split into arcs") {
	const std::vector<Generators::Point> square{{0, 0, 0}, {80, 0, 0}, {80, 40, 0}, {0, 40, 0}};
	Generators::PerimeterOptions o;
	o.spacing = 25;
	o.arcs = 2;
	const auto tree = Algorithms::Decompose(Generators::PerimeterTask("op.4", square, o));
	CHECK(tree.Node("op.4").code == "T_L");
	CHECK(tree.Node("op.4").children.size() == 2);
	// 240 m of boundary in legs of at most 25 m: at least 10 legs, plus each arc's first leaf.
	CHECK(tree.leaves.size() >= 11);
}

TEST_CASE("point presets") {
	using Generators::PointAction;
	const Generators::Point p{5, 6, 15};
	auto make = [&](PointAction a) {
		Generators::PointOptions o;
		o.action = a;
		o.package = 7;
		return Generators::PointTask("op.2", p, o);
	};
	for (auto a : {PointAction::LEDS, PointAction::HOVER, PointAction::LAND, PointAction::PICTURE, PointAction::GIMBAL}) {
		CAPTURE(Generators::PointActionName(a));
		const auto t = make(a);
		CHECK(t.code == "T_A");
		// Written and read back unchanged.
		CHECK(FirstTask("@: MRS 0.1/\n" + Protocol::Write(t)) == t);
		PointAction back{};
		CHECK(Generators::ParsePointAction(Generators::PointActionName(a), back));
		CHECK(back == a);
	}
	const std::string picture = Protocol::Write(make(PointAction::PICTURE));
	CHECK(picture.find("A_GMB") != std::string::npos);
	CHECK(picture.find("A_CAM") != std::string::npos);
	// Release: land there if carrying package 7, then release it once landed, by the same robot.
	const auto tree = Algorithms::Decompose(make(PointAction::RELEASE));
	REQUIRE(tree.leaves.size() == 2);
	const std::string land = Protocol::Write(tree.Node("op.2.1").task);
	CHECK(land.find("payload.id") != std::string::npos);
	CHECK(land.find("A_LD") != std::string::npos);
	CHECK(tree.Node("op.2.2").affinity == "op.2.1");
	CHECK(tree.Node("op.2.2").after == std::vector<std::string>{"op.2.1"});
	CHECK(Protocol::Write(tree.Node("op.2.2").task).find("A_REL 7") != std::string::npos);
	PointAction a{};
	CHECK_FALSE(Generators::ParsePointAction("dance", a));
}

TEST_CASE("ThenLand puts a landing leaf per robot after the tree") {
	Generators::CoverageOptions o;
	o.cells = 2;
	const auto cover = Generators::CoverageTask("op.5", {{0, 0, 0}, {60, 0, 0}, {60, 40, 0}, {0, 40, 0}}, o);
	const auto tree = Algorithms::Decompose(Generators::ThenLand("op.5", cover, {"r1", "r2"}));
	CHECK(tree.Node("op.5").code == "T_S");
	CHECK(tree.Node("op.5.1").code == "T_L");
	CHECK(tree.Node("op.5.2").children.size() == 2);
	CHECK(tree.Node("op.5.2.1").after == std::vector<std::string>{"op.5.1"});
	// Affinity ids moved with the tree.
	for (const auto& leaf : Algorithms::LeavesUnder(tree, "op.5.1"))
		if (!tree.Node(leaf).affinity.empty()) CHECK(tree.Has(tree.Node(leaf).affinity));
	const std::string landing = Protocol::Write(tree.Node("op.5.2.2").task);
	CHECK(landing.find("R_I r2") != std::string::npos);
}

TEST_CASE("GeoJSON features become tasks") {
	const std::string text = R"({"type": "FeatureCollection", "features": [
	  {"type": "Feature", "properties": {"task": "picture", "pitch": -1.0},
	   "geometry": {"type": "Point", "coordinates": [24.1052, 56.9497, 12]}},
	  {"type": "Feature", "properties": {"task": "search", "group": "a", "radius": 15},
	   "geometry": {"type": "MultiPoint", "coordinates": [[24.1050, 56.9494], [24.1056, 56.9494]]}},
	  {"type": "Feature", "properties": {},
	   "geometry": {"type": "LineString", "coordinates": [[24.1048, 56.9493], [24.1052, 56.9493], [24.1052, 56.9496]]}},
	  {"type": "Feature", "properties": {"task": "perimeter"},
	   "geometry": {"type": "Polygon", "coordinates": [[[24.1048, 56.9493], [24.1056, 56.9493], [24.1056, 56.9498], [24.1048, 56.9493]]]}}
	]})";
	const auto features = Generators::ReadGeoJson(text, Geo());
	REQUIRE(features.size() == 5);  // the MultiPoint gives two
	CHECK(features[0].geometry == "Point");
	// 0.0001° of latitude is about 11.1 m north of the reference; the altitude is above it.
	CHECK(features[0].points[0][1] == doctest::Approx(11.13).epsilon(0.01));
	CHECK(features[0].points[0][2] == doctest::Approx(12.0));
	CHECK(features[4].points.size() == 3);  // the closing corner is dropped
	Generators::GeoJsonOptions o;
	o.first_id = 10;
	const auto tasks = Generators::TasksFromGeoJson(features, o);
	REQUIRE(tasks.size() == 4);  // picture, path, perimeter, one search tree for group a
	std::set<std::string> ids;
	for (const auto& t : tasks) ids.insert(t.fields[0].s);
	CHECK(ids == std::set<std::string>{"op.10", "op.11", "op.12", "op.13"});
	CHECK(tasks[0].code == "T_A");
	CHECK(Algorithms::Decompose(tasks.back()).Node(tasks.back().fields[0].s).children.size() == 2);

	CHECK_THROWS_AS(Generators::ReadGeoJson("{\"type\": \"Point\", \"coordinates\": [24.1]}", Geo()), Generators::JsonError);
	CHECK_THROWS_AS(Generators::ReadGeoJson("{\"type\": \"Feature\", \"geometry\": {\"type\": \"Circle\", \"coordinates\": []}}", Geo()),
	                Generators::JsonError);
	CHECK_THROWS_AS(Generators::ParseJson("{\"a\": [1, 2,]}"), Generators::JsonError);
	CHECK_THROWS_AS(Generators::TasksFromGeoJson(Generators::ReadGeoJson(
	                    R"({"type": "Feature", "properties": {"task": "dance"}, "geometry": {"type": "Point", "coordinates": [24.1, 56.9]}})", Geo()), {}),
	                Generators::JsonError);
}

// --- robots ----------------------------------------------------------------------------------

TEST_CASE("R_I: only the named robot meets it") {
	const std::string text = "@: MRS 0.1/\nT: T_A op.1 1 0 C_1 C_2 R_1 A_1/\nC_1: C_N/\nC_2: C_N/\nR_1: R_I r2/\nA_1: A_N/\n";
	Team team(2);
	team.Issue({FirstTask(text)});
	CHECK(team.Run(30));
	CHECK(team.issuer.Tasks().at("op.1").done_by == "r2");
}

TEST_CASE("an idle robot waits at its own layer") {
	// r1's layer is 20 m; the hover point is at 32 m.
	Generators::PointOptions o;
	o.action = Generators::PointAction::HOVER;
	o.duration = 1;
	Team team(1);
	team.Issue({Generators::PointTask("op.1", {10, 10, 32}, o)});
	CHECK(team.Run(80));
	team.Run(15, false);
	const auto& sim = team.robots[0]->sim;
	CHECK(sim.pos[2] == doctest::Approx(20.0).epsilon(0.05));
	CHECK(std::hypot(sim.pos[0] - 10, sim.pos[1] - 10) < 2.0);  // it holds its place over the last point
}

TEST_CASE("picture and gimbal presets drive the camera and the gimbal") {
	Generators::PointOptions o;
	o.action = Generators::PointAction::PICTURE;
	o.pitch = -0.8;
	Team team(1, "mavic_delivery_webots");
	team.Issue({Generators::PointTask("op.1", {0, 15, 18}, o)});
	CHECK(team.Run(80));
	const auto& sim = team.robots[0]->sim;
	REQUIRE(sim.shots.size() == 1);
	CHECK(sim.shots[0].pitch == doctest::Approx(-0.8).epsilon(1e-3));
	CHECK(std::hypot(sim.shots[0].pos[0], sim.shots[0].pos[1] - 15) < 1.5);
	CHECK(sim.shots[0].pos[2] == doctest::Approx(18).epsilon(0.05));

	o.action = Generators::PointAction::GIMBAL;
	o.pitch = 0.9;  // above max_pitch 0.5: clamped
	team.Issue({Generators::PointTask("op.2", {5, 15, 18}, o)}, team.Time());
	CHECK(team.Run(60));
	CHECK(sim.gimbal_pitch == doctest::Approx(0.5).epsilon(1e-3));
}

TEST_CASE("a package is released only by its carrier, on the ground at the point") {
	Generators::PointOptions o;
	o.action = Generators::PointAction::RELEASE;
	o.package = 7;
	Team team(3, "mavic_delivery_webots");
	// Only r3 carries package 7; r1 is closest to the point but carries package 4, r2 nothing.
	auto* l1 = dynamic_cast<Device::Uav::PayloadLatch*>(team.robots[0]->robot.tree.Find("latch"));
	auto* l2 = dynamic_cast<Device::Uav::PayloadLatch*>(team.robots[1]->robot.tree.Find("latch"));
	REQUIRE(l1);
	REQUIRE(l2);
	l1->SetPackage(4);
	l2->SetPackage(0);
	team.Issue({Generators::PointTask("op.1", {-10, 20, 15}, o)}, 2.0);
	CHECK(team.Run(150));
	CHECK(team.issuer.Tasks().at("op.1").done);
	CHECK(team.issuer.Leaves().at("op.1.2").done_by == "r3");
	CHECK(team.robots[0]->sim.drops.empty());
	CHECK(team.robots[1]->sim.drops.empty());
	const auto& sim = team.robots[2]->sim;
	REQUIRE(sim.drops.size() == 1);
	CHECK(sim.drops[0].pos[2] < 0.05);
	CHECK(std::hypot(sim.drops[0].pos[0] + 10, sim.drops[0].pos[1] - 20) < 1.5);
	CHECK_FALSE(sim.package_held);
}

TEST_CASE("WP7 acceptance: a GeoJSON polygon becomes a coverage tree that 3 UAVs fly, then land") {
	Generators::GeoJsonOptions o;
	o.land_robots = {"r1", "r2", "r3"};
	const auto tasks = Generators::TasksFromGeoJson(Generators::ReadGeoJson(Test::ReadFile(TreesDir() / "field.geojson"), Geo()), o);
	REQUIRE(tasks.size() == 1);
	Team team(3);
	team.Issue(tasks, 1.0);
	REQUIRE(team.Run(600));
	const auto& root = team.issuer.Tasks().at("op.1");
	REQUIRE(root.done);
	CHECK_FALSE(root.failed);
	const auto* tree = team.issuer.Tree("op.1");
	REQUIRE(tree);
	double last_strip = 0, first_landing = 1e9;
	std::set<std::string> cell_robots;
	for (const auto& entry : team.issuer.Leaves()) {
		const std::string& id = entry.first;
		const auto& leaf = entry.second;
		CAPTURE(id);
		REQUIRE(leaf.done);
		CHECK(leaf.done_count == 1);  // no leaf flown twice
		if (id.rfind("op.1.1.", 0) == 0) {
			last_strip = std::max(last_strip, *leaf.done);
			// The legs of one cell are flown by one robot.
			const std::string cell = id.substr(0, id.rfind('.'));
			CHECK(leaf.done_by == team.issuer.Leaves().at(cell + ".1").done_by);
			cell_robots.insert(leaf.done_by);
		} else {
			first_landing = std::min(first_landing, *leaf.done);
		}
	}
	CHECK(cell_robots.size() == 3);         // the cells were split between the three
	CHECK(first_landing > last_strip);      // landing waits for every strip
	for (const auto& r : team.robots) CHECK(r->sim.OnGround());
	CHECK(team.metrics.fence_exits == 0);
}
