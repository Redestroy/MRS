#pragma once
// Task generators (plan §9.2, spec 11 §3): operator geometry in, protocol tasks out. Points
// become atomic tasks with a preset action; lines, areas, perimeters and points of interest
// become tree tasks (spec 03 §6) that the robots split between them.
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "mrs/generators/GeoJson.h"
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Generators {
		// --- points ----------------------------------------------------------------------------

		// The preset tasks of a point (JB, 2026-10-08).
		enum class PointAction {
			LEDS,     // flash the LEDs: colour on, wait, off, wait
			HOVER,    // hold over the point
			LAND,     // land at the point
			PICTURE,  // take a picture (A_CAM)
			GIMBAL,   // orient the gimbal (A_GMB)
			RELEASE,  // land and release a package, if the robot carries that package (A_REL)
		};
		const char* PointActionName(PointAction a);
		bool ParsePointAction(const std::string& s, PointAction& a);

		struct PointOptions {
			PointAction action = PointAction::LEDS;
			double priority = 1.0;
			double tol_xy = 1.0, tol_z = 0.5;  // m
			std::int64_t colour = 0xFF0000;    // LEDS
			int led_mask = 3;                  // LEDS: every LED of the Mavic
			double duration = 3.0;             // s: LEDS on and off, HOVER, PICTURE settle
			double pitch = -1.5708, yaw = 0.0; // rad: GIMBAL, and PICTURE before the shot
			std::int64_t package = 0;          // RELEASE: the package id the robot must carry
			double descent = 1.0;              // m/s: LAND, RELEASE
		};

		// One task for a point: a T_A, or for RELEASE a T_S of "land there" and "release" with the
		// package check (spec 11 §3.1).
		Protocol::Record PointTask(const std::string& id, const Point& p, const PointOptions& o);

		// --- waypoint chains -------------------------------------------------------------------

		struct ChainOptions {
			double priority = 1.0;
			double tol_xy = 1.5, tol_z = 1.0;  // m, at each waypoint
		};

		// A T_S over the waypoints (spec 11 §3.2): the first leaf flies to the first waypoint by the
		// robot's own fly-to; every later leaf is a level leg (a T_B that holds a position
		// setpoint until the waypoint is reached) with affinity to the leaf before it, so one
		// robot flies the chain. Inner ids are written as 0 and the affinity ids as the
		// decomposer will assign them under `id`.
		Protocol::Record ChainTask(const std::string& id, const std::vector<Point>& waypoints, const ChainOptions& o);

		// PathTaskGenerator: a line string as one chain.
		Protocol::Record PathTask(const std::string& id, const std::vector<Point>& line, const ChainOptions& o);

		// --- areas -----------------------------------------------------------------------------

		struct CoverageOptions {
			double altitude = 20.0;   // m
			double footprint = 20.0;  // m: the sensor's swath across the track
			double overlap = 0.2;     // fraction of the swath shared by neighbouring tracks
			int cells = 3;            // parallel cells, each flown by one robot
			double heading = std::numeric_limits<double>::quiet_NaN();  // track direction, rad ENU; NaN: along the longest edge
			ChainOptions chain;
		};

		// AreaCoverageGenerator: boustrophedon tracks across the polygon, cut into `cells` groups of
		// neighbouring tracks of about equal length; a T_L (k = 0) of one chain per cell. The
		// polygon is treated as convex (each track spans its first and last crossing).
		Protocol::Record CoverageTask(const std::string& id, const std::vector<Point>& polygon, const CoverageOptions& o);

		// The tracks before they are cut into cells: pairs of track ends, in sweep order.
		std::vector<std::pair<Point, Point>> CoverageTracks(const std::vector<Point>& polygon, const CoverageOptions& o);

		struct PerimeterOptions {
			double altitude = 20.0;  // m
			double spacing = 25.0;   // m: the longest leg along the boundary
			int arcs = 1;            // arcs flown in parallel
			ChainOptions chain;
		};

		// PerimeterGenerator: the closed boundary at one altitude, cut into `arcs` arcs of about equal
		// length; one chain, or a T_L (k = 0) of chains.
		Protocol::Record PerimeterTask(const std::string& id, const std::vector<Point>& polygon, const PerimeterOptions& o);

		// --- points of interest ----------------------------------------------------------------

		struct SearchOptions {
			double altitude = 15.0;   // m, when a point has no altitude of its own (z = 0)
			double radius = 30.0;     // m: how far from the point to search
			double footprint = 10.0;  // m: the sensor's swath
			double overlap = 0.1;     // fraction shared by neighbouring turns
			double step = 6.0;        // m: the longest leg along the spiral
			ChainOptions chain;
		};

		// The spiral of one point of interest: an Archimedean spiral outwards from the point with
		// `footprint × (1 − overlap)` between turns, so neighbouring turns overlap by `overlap`.
		std::vector<Point> SpiralWaypoints(const Point& centre, const SearchOptions& o);

		// SearchGenerator: one spiral chain per point of interest, in a T_L (k = 0), or the chain
		// alone for one point.
		Protocol::Record SearchTask(const std::string& id, const std::vector<Point>& pois, const SearchOptions& o);

		// --- assembly --------------------------------------------------------------------------

		// The tree, then every listed robot back home to land (spec 03 §6.2 example): a T_S of
		// `tree` and a T_L (k = 0) with one landing leaf per robot (R_I). `tree` keeps its shape and
		// its affinity ids are moved to their new place.
		Protocol::Record ThenLand(const std::string& id, const Protocol::Record& tree, const std::vector<std::string>& robots);

		// A task file (.mrst) of top-level tasks.
		std::string TaskFile(const std::vector<Protocol::Record>& tasks, const std::string& comment = {});
		// A timeline (.mrsl): each task at its dispatch time.
		std::string Timeline(const std::vector<std::pair<double, Protocol::Record>>& entries, const std::string& comment = {});

		// Everything in a GeoJSON file (spec 11 §4.2): each feature's "task" property picks the
		// generator (Point: point preset, default "leds"; LineString: "path"; Polygon: "coverage"
		// or "perimeter"; MultiPoint or Point with task "search": spiral search) and its other
		// properties set the options. Ids are <issuer>.1, .2, ... in feature order. Features with
		// the same "search" group are searched in one tree. Throws JsonError.
		struct GeoJsonOptions {
			std::string issuer = "op";
			int first_id = 1;
			std::vector<std::string> land_robots;  // non-empty: every tree ends with these robots landing
		};
		std::vector<Protocol::Record> TasksFromGeoJson(const std::vector<Feature>& features, const GeoJsonOptions& o);
	}
}
