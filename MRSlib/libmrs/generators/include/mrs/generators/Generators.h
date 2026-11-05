#pragma once
// Task generators (plan §9.2, spec 11 §3): operator geometry in, protocol tasks out. Points
// become atomic tasks with a preset action; lines, areas, perimeters and points of interest
// become tree tasks (spec 03 §6) that the robots split between them.
#include <array>
#include <cstdint>
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

		// --- package delivery (spec 18 §5) ------------------------------------------------------

		struct DeliveryOptions {
			double priority = 1.0;
			double sling_length = 2.0;     // m, body to hook (closed)
			double package_height = 0.25;  // m, for the height of a pile
			double loop_height = 0.10;     // m, the loop above a package
			double approach = 1.0;         // m: the hook starts its descent this far above the top loop
			double tol_xy = 0.25, tol_z = 0.5;  // m, of the point above the pile
			double mass_min = 0.2, mass_max = 0.5;  // kg, drawn per package
			unsigned long long seed = 1;   // destinations and masses
			// Zones (spec 18 §4): wait in line for the pile and for the destination area.
			bool zones = false;
			double zone_radius = 3.0;      // m
			double hold_offset = 6.0;      // m from the zone's centre to its holding point
			std::int64_t dest_zone_base = 200;
			// Relay positions (spec 18 §6.3): with relay_range > 0, hold-position tasks on the line
			// from the operator to each destination farther than relay_spacing x relay_range.
			double relay_range = 0.0;        // m, the radio range; 0: no relay tasks
			Point operator_pos{0, 0, 0};     // ENU, the issuer's radio
			double relay_spacing = 0.8;      // links are at most this fraction of the range
			double relay_altitude = 40.0;    // m, above the robots' altitude layers (spec 06 §6)
			double relay_hold_s = 600.0;     // s a relay holds its position
			double relay_priority = 2.0;     // above the deliveries, so idle robots take them first
		};

		// A zone of the job: source s is zone s + 1, destination area a is dest_zone_base + a.
		struct ZoneDef {
			std::int64_t id = 0;
			Point centre{0, 0, 0}, hold{0, 0, 0};
			double radius = 3.0;
		};

		// One package of the manifest: what its tag says, and the pile it starts in.
		struct ManifestEntry {
			std::int64_t package = 0;
			std::size_t source = 0;  // index into the source list
			std::size_t area = 0;    // index into the destination list (1-based on the tag)
			Point dest{0, 0, 0};
			double mass = 0.3;
		};

		struct DeliveryJob {
			std::vector<ManifestEntry> manifest;  // in pile order per source, bottom first
			std::vector<Protocol::Record> tasks;  // one delivery unit per package
			std::vector<ZoneDef> zones;           // with DeliveryOptions::zones
			std::vector<Point> relays;            // relay positions, with DeliveryOptions::relay_range (tasks relay.1.., first in `tasks`)
		};

		// n packages split over the sources in turn, each with a destination drawn from `destinations`
		// (seeded). One task per package: "deliver the next package from source s" (spec 18 §5).
		// Task ids are <issuer>.<first_id + k>.
		DeliveryJob DeliveryTasks(int n, const std::vector<Point>& sources, const std::vector<Point>& destinations,
		                          const DeliveryOptions& o, const std::string& issuer = "op", int first_id = 1);
		// A delivery unit for a pile at `source` whose top loop is at `top_loop` m (spec 18 §5.1).
		// With o.zones, source_zone is the pile's zone number.
		Protocol::Record DeliveryTask(const std::string& id, const Point& source, double top_loop, const DeliveryOptions& o,
		                              std::int64_t source_zone = 0);
		// Relay positions (spec 18 §6.3): for each destination farther than spacing x range from the
		// operator, evenly spaced points on the line between them so that no link is longer than
		// spacing x range, at the relay altitude. A point within 0.3 x range of one already chosen
		// is shared.
		std::vector<Point> RelayPoints(const std::vector<Point>& destinations, const DeliveryOptions& o);
		// The manifest as CSV: package,source,area,dest_x,dest_y,dest_z,mass.
		std::string ManifestCsv(const DeliveryJob& job);

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
