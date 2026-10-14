// Spec 17 tests: the track file (writer, reader, interpolation, errors), task markers from the
// protocol files, events applied to markers, and a QuadSim run that writes its track.
#include <cmath>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>
#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "mrs/protocol/Parser.h"
#include "mrs/replay/TaskMarkers.h"
#include "mrs/replay/Track.h"
#include "mrs/sim/Experiment.h"
#include "mrs/sim/Team.h"

using namespace MRS;

namespace {
	Replay::Track Read(const std::string& text) {
		std::istringstream in(text);
		return Replay::ReadTrack(in);
	}

	std::size_t ErrorLine(const std::string& text) {
		try {
			Read(text);
		} catch (const Replay::TrackError& e) {
			return e.line;
		}
		return 0;
	}

	Replay::TaskSet Markers(const std::string& text, const Environment::GeoReference* geo = nullptr) {
		auto parsed = Protocol::Parse(text);
		REQUIRE_MESSAGE(parsed.Ok(), (parsed.Ok() ? "" : parsed.error->message));
		Replay::TaskSet set;
		Replay::AddMarkers(parsed.document, geo, set);
		return set;
	}

	const Replay::TaskMarker* Find(const Replay::TaskSet& set, const std::string& task) {
		for (const auto& m : set.markers)
			if (m.task == task) return &m;
		return nullptr;
	}

	std::filesystem::path WorldsDir() { return Test::ExamplesDir() / ".." / ".." / ".." / ".." / "MRSlib" / "libmrs" / "platforms" / "webots" / "worlds"; }
}

TEST_CASE("a written track reads back with its mission, poses and events") {
	Sim::MissionSpec mission;
	mission.homes = Sim::RowHomes(2);
	const auto header = mission.Header();
	std::ostringstream out;
	Replay::TrackWriter w(out, {"r1", "r2"}, &header);
	w.Sample(0.0, {{-2, -5, 0, 0, 0, 0}, {2, -5, 0, 0, 0, 1.5}});
	w.Event(0.5, "dispatch", "op.1");
	w.Sample(1.0, {{-2, -5, 10, 0.1, -0.2, 3.0}, {2, -5, 4, 0, 0, 1.5}});
	w.Event(0.9, "done", "op.1", "r2");
	w.Sample(1.0004, {{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}});  // not later, to the millisecond: dropped

	const auto t = Read(out.str());
	REQUIRE(t.robots == std::vector<std::string>{"r1", "r2"});
	CHECK(t.oriented == std::vector<bool>{true, true});
	REQUIRE(t.times.size() == 2);
	CHECK(t.samples[1][0].z == doctest::Approx(10));
	CHECK(t.samples[1][0].pitch == doctest::Approx(-0.2));
	REQUIRE(t.mission);
	CHECK(t.mission->code == "H_M");
	CHECK(Replay::GeoOf(*t.mission).ToEnu(56.9496, 24.1052, 10).x == doctest::Approx(0).epsilon(1e-6));
	REQUIRE(t.events.size() == 2);
	CHECK(t.events[1].kind == "done");
	CHECK(t.events[1].task == "op.1");
	CHECK(t.events[1].detail == "r2");
	CHECK(t.events[1].t == doctest::Approx(0.9));
}

TEST_CASE("poses are interpolated between rows, yaw the short way round") {
	const auto t = Read(
	    "t,a.x,a.y,a.z,a.yaw\n"
	    "0,0,0,0,3.0\n"
	    "2,4,2,10,-3.0\n");
	Replay::Pose p;
	REQUIRE(t.At(0, 1.0, p));
	CHECK(p.x == doctest::Approx(2));
	CHECK(p.y == doctest::Approx(1));
	CHECK(p.z == doctest::Approx(5));
	// 3.0 to -3.0 crosses ±π: halfway is π, not 0.
	CHECK(std::fabs(std::sin(p.yaw)) < 1e-6);
	CHECK(std::cos(p.yaw) == doctest::Approx(-1));
	REQUIRE(t.At(0, 5.0, p));  // after the end: the last pose
	CHECK(p.x == doctest::Approx(4));
	CHECK(t.Index(1.99) == 0);
	CHECK(t.Index(2.0) == 1);
}

TEST_CASE("a track without robot prefixes takes the columns as x, y, z of robots 1, 2, ...") {
	const auto t = Read("t,x1,y1,z1,x2,y2,z2\n0,1,2,3,4,5,6\n");
	REQUIRE(t.robots == std::vector<std::string>{"1", "2"});
	CHECK(t.oriented == std::vector<bool>{false, false});
	CHECK(t.samples[0][1].y == doctest::Approx(5));
}

TEST_CASE("empty cells mean the robot is not there") {
	const auto t = Read("t,a.x,a.y,a.z,b.x,b.y,b.z\n0,1,1,1,,,\n1,2,2,2,3,3,3\n");
	Replay::Pose p;
	CHECK_FALSE(t.At(1, 0.0, p));
	CHECK(t.At(1, 1.0, p));
	CHECK(t.At(0, 0.5, p));
}

TEST_CASE("track errors carry the line") {
	CHECK(ErrorLine("x,a.x\n") == 1);                                      // no t column
	CHECK(ErrorLine("# c\nt,a.x,a.y\n") == 2);                             // no z
	CHECK(ErrorLine("t,a.x,a.y,a.z,a.speed\n") == 1);                      // unknown axis
	CHECK(ErrorLine("t,a.x,a.y,a.z\n0,1,2,3\n0,1,2,3\n") == 3);            // t does not increase
	CHECK(ErrorLine("t,a.x,a.y,a.z\n0,1,2\n") == 2);                       // short row
	CHECK(ErrorLine("t,a.x,a.y,a.z\n0,1,x,3\n") == 2);                     // not a number
	CHECK(ErrorLine("t,a.x,a.y,a.z\n0,1,,3\n") == 2);                      // partly empty
	CHECK(ErrorLine("t,a.x,a.y,a.z\n# event 1 landed op.1\n") == 2);       // unknown event
	CHECK(ErrorLine("# only comments\n") == 1);
}

TEST_CASE("the Webots task set gives one flagged marker per task at its dispatch time") {
	Replay::TaskSet set;
	Replay::AddMarkerFile((WorldsDir() / "taskset1.mrsl").string(), nullptr, set);
	CHECK(set.warnings.empty());
	REQUIRE(set.markers.size() == 15);
	const auto* m = Find(set, "op.2");
	REQUIRE(m);
	CHECK(m->x == doctest::Approx(10.507));
	CHECK(m->y == doctest::Approx(32.336));
	CHECK(m->z == doctest::Approx(15));
	CHECK(m->tol_xy == doctest::Approx(1));
	CHECK(m->tol_z == doctest::Approx(0.5));
	CHECK(m->dispatch == doctest::Approx(6));
	CHECK(m->StateAt(5.9) == Replay::MarkerState::HIDDEN);
	CHECK(m->StateAt(6.0) == Replay::MarkerState::OPEN);
}

TEST_CASE("tree markers carry the decomposer's leaf ids") {
	const auto set = Markers(Test::ReadFile(Test::ExamplesDir() / "uav_tree_task.mrst"));
	REQUIRE(set.markers.size() == 2);
	CHECK(set.markers[0].task == "op.40.1.1");
	CHECK(set.markers[1].task == "op.40.1.2");
	CHECK(set.markers[1].x == doctest::Approx(50));
}

TEST_CASE("C_G needs a geo reference: from the caller, or a mission header in the same file") {
	const std::string timeline = Test::ReadFile(Test::ExamplesDir() / "mission_timeline.mrsl");
	auto set = Markers(timeline);
	CHECK(set.markers.size() == 2);
	CHECK(set.warnings.size() == 1);
	const Environment::GeoReference geo(56.9496, 24.1052, 10);
	set = Markers(timeline, &geo);
	REQUIRE(set.markers.size() == 3);
	const auto* m = Find(set, "op.3");
	REQUIRE(m);
	CHECK(m->dispatch == doctest::Approx(5));
	CHECK(m->x == doctest::Approx(geo.ToEnu(56.9505, 24.1060, 25).x));
	CHECK(m->z == doctest::Approx(15).epsilon(0.01));

	set = Markers("H: H_M m1 56.9496 24.1052 10 -100 100 -100 100 60 20 5 120 0/\n" + timeline);
	CHECK(set.markers.size() == 3);
	CHECK(set.mission);
}

TEST_CASE("events end the markers of a task and of the leaves under it") {
	auto set = Markers(Test::ReadFile(Test::ExamplesDir() / "uav_tree_task.mrst"));
	Replay::ApplyEvents({{2.0, "dispatch", "op.40.1.1", ""}, {10.0, "done", "op.40.1.1", "r1"}, {20.0, "failed", "op.40", "TREE"}}, set);
	const auto& a = set.markers[0];
	const auto& b = set.markers[1];
	CHECK(a.dispatch == doctest::Approx(2));
	CHECK(a.done_by == "r1");
	CHECK(a.StateAt(1.0) == Replay::MarkerState::HIDDEN);
	CHECK(a.StateAt(9.0) == Replay::MarkerState::OPEN);
	CHECK(a.StateAt(10.0) == Replay::MarkerState::DONE);
	CHECK(b.StateAt(19.0) == Replay::MarkerState::OPEN);
	CHECK(b.StateAt(20.0) == Replay::MarkerState::FAILED);
}

TEST_CASE("a QuadSim run writes a track the viewer reads, with every task done in it") {
	Sim::RunSpec s;
	s.set = "track";
	s.timeline =
	    "L: L_D 0 T_1 T_2/\n"
	    "T_1: T_A op.1 1 0 C_1 C_2 A_1/\nC_1: C_P3 10 0 15 1 0.5 0 -1/\nC_2: C_N/\nA_1: A_N/\n"
	    "T_2: T_A op.2 1 0 C_1 C_2 A_1/\nC_1: C_P3 -10 10 15 1 0.5 0 -1/\nC_2: C_N/\nA_1: A_N/\n";
	s.condition = Sim::Condition::G_RTA;
	s.n = 2;
	s.time_limit = 300;
	s.track_path = (std::filesystem::temp_directory_path() / "mrs_replay_test_track.csv").string();
	s.track_period = 0.2;
	const Sim::RobotFiles files{Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsd"), Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsp"),
	                            (Test::ExamplesDir() / "uav_behaviours.mrsb").string()};
	const auto r = Sim::RunOne(files, s);
	REQUIRE(r.completed);

	const auto t = Replay::ReadTrackFile(s.track_path);
	CHECK(t.robots == std::vector<std::string>{"r1", "r2"});
	REQUIRE(t.mission);
	CHECK(t.times.size() > 10);
	CHECK(t.End() == doctest::Approx(r.sim_time).epsilon(0.01));

	auto set = Markers(s.timeline);
	Replay::ApplyEvents(t.events, set);
	REQUIRE(set.markers.size() == 2);
	for (const auto& m : set.markers) {
		CHECK(m.StateAt(t.End()) == Replay::MarkerState::DONE);
		// The robot that did it was within the tolerance of the marker when it did.
		std::size_t k = 0;
		while (k < t.robots.size() && t.robots[k] != m.done_by) ++k;
		REQUIRE(k < t.robots.size());
		Replay::Pose p;
		REQUIRE(t.At(k, *m.done, p));
		CHECK(std::hypot(p.x - m.x, p.y - m.y) < 2.0);
	}
	std::filesystem::remove(s.track_path);
}
