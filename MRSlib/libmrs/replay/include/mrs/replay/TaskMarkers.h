#pragma once
// Task markers for the MRS dot viewer (spec 17 §3): the spatial conditions of the tasks in
// protocol files (.mrsl timelines, .mrst task files, mission headers), as points in ENU.
#include <optional>
#include <string>
#include <vector>

#include "mrs/protocol/Record.h"
#include "mrs/replay/Track.h"
#include "mrs/world/GeoReference.h"

namespace MRS {
	namespace Replay {
		enum class MarkerState { HIDDEN, OPEN, DONE, FAILED };

		struct TaskMarker {
			std::string task;       // the id the issuer reports: the root's own, p.i below a tree
			std::string condition;  // C_P3 or C_G
			double x = 0.0, y = 0.0, z = 0.0;  // ENU m
			double tol_xy = 0.0, tol_z = 0.0;  // m
			double dispatch = 0.0;  // s, from the timeline (0 for a task file)
			std::string done_by;    // filled from the track's events
			std::optional<double> done, failed;

			MarkerState StateAt(double t) const;
		};

		struct TaskSet {
			std::vector<TaskMarker> markers;
			std::optional<Protocol::Record> mission;  // the first H_M found
			std::vector<std::string> warnings;        // C_G without a geo reference, unreadable files
		};

		// Adds the markers of one parsed document. `geo` converts C_G; without it C_G is skipped
		// with a warning. A H_M in the document is kept in `set.mission` when none is set yet.
		void AddMarkers(const Protocol::Document& doc, const Environment::GeoReference* geo, TaskSet& set);
		// Reads, parses and adds a protocol file; a parse error becomes a warning.
		void AddMarkerFile(const std::string& path, const Environment::GeoReference* geo, TaskSet& set);

		// The geo reference of a H_M record.
		Environment::GeoReference GeoOf(const Protocol::Record& mission);

		// Applies the track's events: the issuer's dispatch time replaces the timeline's, and
		// done or failed events for a marker's task, or for a node above it, end it.
		void ApplyEvents(const std::vector<TrackEvent>& events, TaskSet& set);
	}
}
