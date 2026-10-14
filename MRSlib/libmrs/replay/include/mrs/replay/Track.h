#pragma once
// Track files (spec 17 §2): robot poses over time, written by a supervisor or the simulator and
// read by the MRS dot viewer. CSV with '#' comment lines that carry the mission and task events.
#include <cstddef>
#include <iosfwd>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Algorithms {
		class TaskIssuer;
	}

	namespace Replay {
		// ENU metres and radians (roll about x, pitch about y, yaw about z, applied z-y-x).
		struct Pose {
			double x = 0.0, y = 0.0, z = 0.0;
			double roll = 0.0, pitch = 0.0, yaw = 0.0;
		};

		// What happened to a task (spec 17 §2.3).
		struct TrackEvent {
			double t = 0.0;
			std::string kind;    // dispatch, done or failed
			std::string task;    // task, leaf or unit id
			std::string detail;  // done: the robot; failed: the reason; dispatch: empty
		};

		struct TrackError : std::runtime_error {
			TrackError(std::size_t line, const std::string& message);
			std::size_t line;
		};

		struct Track {
			std::vector<std::string> robots;
			std::vector<bool> oriented;              // per robot: the file has its roll, pitch and yaw
			std::vector<double> times;               // s, strictly increasing
			std::vector<std::vector<Pose>> samples;  // [sample][robot]
			std::vector<std::vector<bool>> present;  // [sample][robot]: false where the cells were empty
			std::vector<TrackEvent> events;          // in file order
			std::optional<Protocol::Record> mission; // the H_M record, if the file has one

			double Start() const { return times.empty() ? 0.0 : times.front(); }
			double End() const { return times.empty() ? 0.0 : times.back(); }
			// The last sample at or before t (0 before the first).
			std::size_t Index(double t) const;
			// The pose of robot r at t, interpolated between samples (yaw the short way round).
			// False when the robot is not present at t.
			bool At(std::size_t r, double t, Pose& out) const;
		};

		// Throws TrackError with the 1-based line of the first problem.
		Track ReadTrack(std::istream& in);
		Track ReadTrackFile(const std::string& path);

		// Writes a track. Every row has a pose for every robot named in the constructor.
		class TrackWriter {
		public:
			TrackWriter(std::ostream& out, std::vector<std::string> robots, const Protocol::Record* mission = nullptr);
			// Writes a row; a row whose time (to the millisecond) is not after the last is dropped.
			void Sample(double t, const std::vector<Pose>& poses);
			void Event(double t, const std::string& kind, const std::string& task, const std::string& detail = "");

		private:
			std::ostream& out_;
			std::size_t n_;
			std::optional<double> last_ms_;
		};

		// Turns the issuer's task states into dispatch, done and failed events, once each.
		class IssuerEvents {
		public:
			void Poll(const Algorithms::TaskIssuer& issuer, TrackWriter& writer, double t);

		private:
			struct Seen {
				bool dispatch = false, done = false, failed = false;
			};
			std::map<std::string, Seen> seen_;
		};

		// Roll, pitch and yaw of a row-major 3x3 rotation matrix (body to world).
		void RotationToRpy(const double* r, double& roll, double& pitch, double& yaw);
	}
}
