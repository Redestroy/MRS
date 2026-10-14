#include "mrs/replay/Track.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <istream>
#include <limits>
#include <ostream>
#include <sstream>

#include "mrs/algorithms/TaskIssuer.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

namespace MRS {
	namespace Replay {
		namespace {
			constexpr double kPi = 3.14159265358979323846;
			const char* const kAxes[6] = {"x", "y", "z", "roll", "pitch", "yaw"};

			std::string Trim(const std::string& s) {
				const auto a = s.find_first_not_of(" \t\r");
				if (a == std::string::npos) return "";
				const auto b = s.find_last_not_of(" \t\r");
				return s.substr(a, b - a + 1);
			}

			std::vector<std::string> SplitCsv(const std::string& line) {
				std::vector<std::string> out;
				std::string cell;
				std::istringstream s(line);
				while (std::getline(s, cell, ',')) out.push_back(Trim(cell));
				if (!line.empty() && line.back() == ',') out.push_back("");
				return out;
			}

			bool ParseNumber(const std::string& s, double& v) {
				if (s.empty()) return false;
				char* end = nullptr;
				v = std::strtod(s.c_str(), &end);
				return end == s.c_str() + s.size() && std::isfinite(v);
			}

			double Lerp(double a, double b, double f) { return a + (b - a) * f; }
			double LerpAngle(double a, double b, double f) {
				double d = std::fmod(b - a, 2.0 * kPi);
				if (d > kPi) d -= 2.0 * kPi;
				if (d < -kPi) d += 2.0 * kPi;
				return a + d * f;
			}

			// Column of a track: robot index and axis (0..5).
			struct Column {
				std::size_t robot;
				int axis;
			};

			// The header row (spec 17 §2.2).
			std::vector<Column> ReadHeader(const std::vector<std::string>& cells, std::size_t line, Track& track) {
				if (cells.empty() || cells[0] != "t") throw TrackError(line, "the header must start with the column t");
				std::vector<Column> cols;
				std::size_t bare = 0;  // columns without a robot prefix, taken as x, y, z of robots 1, 2, ...
				for (std::size_t k = 1; k < cells.size(); ++k) {
					const std::string& c = cells[k];
					const auto dot = c.rfind('.');
					std::string robot, axis;
					if (dot == std::string::npos) {
						robot = std::to_string(bare / 3 + 1);
						axis = kAxes[bare % 3];
						++bare;
					} else {
						robot = c.substr(0, dot);
						axis = c.substr(dot + 1);
					}
					const int a = static_cast<int>(std::find(kAxes, kAxes + 6, axis) - kAxes);
					if (robot.empty() || a == 6) throw TrackError(line, "column " + std::to_string(k + 1) + " (" + c + ") is not <robot>.<x|y|z|roll|pitch|yaw>");
					auto it = std::find(track.robots.begin(), track.robots.end(), robot);
					const std::size_t r = static_cast<std::size_t>(it - track.robots.begin());
					if (it == track.robots.end()) track.robots.push_back(robot);
					for (const auto& col : cols)
						if (col.robot == r && col.axis == a) throw TrackError(line, "column " + c + " appears twice");
					cols.push_back({r, a});
				}
				if (track.robots.empty()) throw TrackError(line, "the header names no robot");
				track.oriented.assign(track.robots.size(), false);
				for (std::size_t r = 0; r < track.robots.size(); ++r) {
					int mask = 0;
					for (const auto& col : cols)
						if (col.robot == r) mask |= 1 << col.axis;
					if ((mask & 7) != 7) throw TrackError(line, "robot " + track.robots[r] + " needs x, y and z columns");
					track.oriented[r] = (mask & 0x38) != 0;
				}
				return cols;
			}

			void ReadComment(const std::string& text, std::size_t line, Track& track) {
				std::istringstream s(text);
				std::string word;
				s >> word;
				if (word == "mission") {
					std::string rest;
					std::getline(s, rest);
					auto parsed = Protocol::Parse(rest);
					if (!parsed.Ok()) throw TrackError(line, "mission: " + parsed.error->message);
					for (const auto& r : parsed.document.records)
						if (r.code == "H_M") track.mission = r;
				} else if (word == "event") {
					TrackEvent e;
					std::string t;
					s >> t >> e.kind >> e.task;
					std::getline(s, e.detail);
					e.detail = Trim(e.detail);
					if (!ParseNumber(t, e.t) || e.task.empty() || (e.kind != "dispatch" && e.kind != "done" && e.kind != "failed"))
						throw TrackError(line, "event: expected '# event <t> dispatch|done|failed <task> [detail]'");
					track.events.push_back(e);
				}
				// Other comments are free text.
			}
		}

		TrackError::TrackError(std::size_t l, const std::string& message)
		    : std::runtime_error("line " + std::to_string(l) + ": " + message), line(l) {}

		std::size_t Track::Index(double t) const {
			if (times.empty() || t <= times.front()) return 0;
			auto it = std::upper_bound(times.begin(), times.end(), t);
			return static_cast<std::size_t>(it - times.begin()) - 1;
		}

		bool Track::At(std::size_t r, double t, Pose& out) const {
			if (times.empty() || r >= robots.size()) return false;
			const std::size_t i = Index(t);
			if (!present[i][r]) return false;
			const Pose& a = samples[i][r];
			if (i + 1 >= times.size() || t <= times[i] || !present[i + 1][r]) {
				out = a;
				return true;
			}
			const Pose& b = samples[i + 1][r];
			const double f = (t - times[i]) / (times[i + 1] - times[i]);
			out.x = Lerp(a.x, b.x, f);
			out.y = Lerp(a.y, b.y, f);
			out.z = Lerp(a.z, b.z, f);
			out.roll = LerpAngle(a.roll, b.roll, f);
			out.pitch = LerpAngle(a.pitch, b.pitch, f);
			out.yaw = LerpAngle(a.yaw, b.yaw, f);
			return true;
		}

		Track ReadTrack(std::istream& in) {
			Track track;
			std::vector<Column> cols;
			bool header = false;
			std::string raw;
			for (std::size_t line = 1; std::getline(in, raw); ++line) {
				const std::string text = Trim(raw);
				if (text.empty()) continue;
				if (text[0] == '#') {
					ReadComment(text.substr(1), line, track);
					continue;
				}
				const auto cells = SplitCsv(text);
				if (!header) {
					cols = ReadHeader(cells, line, track);
					header = true;
					continue;
				}
				if (cells.size() != cols.size() + 1)
					throw TrackError(line, "expected " + std::to_string(cols.size() + 1) + " cells, found " + std::to_string(cells.size()));
				double t;
				if (!ParseNumber(cells[0], t)) throw TrackError(line, "t is not a number");
				if (!track.times.empty() && t <= track.times.back()) throw TrackError(line, "t must increase");
				std::vector<Pose> poses(track.robots.size());
				std::vector<int> filled(track.robots.size(), 0), empty(track.robots.size(), 0);
				for (std::size_t k = 0; k < cols.size(); ++k) {
					const Column& c = cols[k];
					double v;
					if (cells[k + 1].empty()) {
						++empty[c.robot];
						continue;
					}
					if (!ParseNumber(cells[k + 1], v)) throw TrackError(line, "cell " + std::to_string(k + 2) + " is not a number");
					++filled[c.robot];
					double* fields[6] = {&poses[c.robot].x, &poses[c.robot].y, &poses[c.robot].z,
					                     &poses[c.robot].roll, &poses[c.robot].pitch, &poses[c.robot].yaw};
					*fields[c.axis] = v;
				}
				std::vector<bool> present(track.robots.size());
				for (std::size_t r = 0; r < present.size(); ++r) {
					if (filled[r] > 0 && empty[r] > 0) throw TrackError(line, "robot " + track.robots[r] + " has some cells empty: leave all empty or none");
					present[r] = filled[r] > 0;
				}
				track.times.push_back(t);
				track.samples.push_back(std::move(poses));
				track.present.push_back(std::move(present));
			}
			if (!header) throw TrackError(1, "no header row");
			return track;
		}

		Track ReadTrackFile(const std::string& path) {
			std::ifstream in(path, std::ios::binary);
			if (!in) throw std::runtime_error("cannot open " + path);
			try {
				return ReadTrack(in);
			} catch (const TrackError& e) {
				throw std::runtime_error(path + ": " + e.what());
			}
		}

		TrackWriter::TrackWriter(std::ostream& out, std::vector<std::string> robots, const Protocol::Record* mission)
		    : out_(out), n_(robots.size()) {
			out_ << "# MRS track 0.1 (spec 17)\n";
			if (mission) {
				Protocol::WriteOptions o;
				o.compact = true;
				std::string text = Protocol::Write(*mission, o);
				while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) text.pop_back();
				out_ << "# mission " << text << "\n";
			}
			out_ << "t";
			for (const auto& r : robots)
				for (const char* a : kAxes) out_ << "," << r << "." << a;
			out_ << "\n";
		}

		void TrackWriter::Sample(double t, const std::vector<Pose>& poses) {
			if (poses.size() != n_) throw std::invalid_argument("TrackWriter::Sample: one pose per robot");
			std::ostringstream s;
			s << std::fixed << std::setprecision(3) << t;
			const double ms = std::round(t * 1000.0);
			if (last_ms_ && ms <= *last_ms_) return;  // as written, a row must be later than the last
			for (const auto& p : poses) {
				// + 0.0 turns -0 into 0
				s << std::setprecision(3) << "," << p.x + 0.0 << "," << p.y + 0.0 << "," << p.z + 0.0;
				s << std::setprecision(4) << "," << p.roll + 0.0 << "," << p.pitch + 0.0 << "," << p.yaw + 0.0;
			}
			out_ << s.str() << "\n";
			last_ms_ = ms;
		}

		void TrackWriter::Event(double t, const std::string& kind, const std::string& task, const std::string& detail) {
			std::ostringstream s;
			s << std::fixed << std::setprecision(3) << t;
			out_ << "# event " << s.str() << " " << kind << " " << task;
			if (!detail.empty()) out_ << " " << detail;
			out_ << "\n";
		}

		void IssuerEvents::Poll(const Algorithms::TaskIssuer& issuer, TrackWriter& writer, double t) {
			for (const auto* tasks : {&issuer.Tasks(), &issuer.Units(), &issuer.Leaves()}) {
				for (const auto& [id, it] : *tasks) {
					Seen& s = seen_[id];
					if (!s.dispatch) {
						s.dispatch = true;
						writer.Event(it.dispatch, "dispatch", id);
					}
					if (it.done && !s.done) {
						s.done = true;
						writer.Event(*it.done, "done", id, it.done_by);
					}
					if (it.failed && !s.failed) {
						s.failed = true;
						writer.Event(t, "failed", id, *it.failed);
					}
				}
			}
		}

		void RotationToRpy(const double* r, double& roll, double& pitch, double& yaw) {
			yaw = std::atan2(r[3], r[0]);
			pitch = std::asin(std::clamp(-r[6], -1.0, 1.0));
			roll = std::atan2(r[7], r[8]);
		}
	}
}
