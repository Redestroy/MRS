#include "mrs/replay/TaskMarkers.h"

#include <cmath>
#include <fstream>
#include <sstream>

#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Replay {
		using Protocol::FieldType;
		using Protocol::Record;

		namespace {
			double Num(const Protocol::Field& f) { return f.type == FieldType::Int ? static_cast<double>(f.i) : f.n; }

			bool IsComplex(const Record& r) { return r.code == "T_S" || r.code == "T_L" || r.code == "T_O"; }

			bool Under(const std::string& id, const std::string& node) { return id == node || id.rfind(node + ".", 0) == 0; }

			struct Walker {
				const Environment::GeoReference* geo;
				TaskSet& set;
				double dispatch;

				void Add(const Record& c, const std::string& label) {
					TaskMarker m;
					m.task = label;
					m.condition = c.code;
					m.dispatch = dispatch;
					if (c.fields.size() < 5) return;
					if (c.code == "C_P3") {
						m.x = Num(c.fields[0]);
						m.y = Num(c.fields[1]);
						m.z = Num(c.fields[2]);
					} else {
						if (!geo) {
							set.warnings.push_back("task " + label + ": C_G needs a mission header (geo reference); skipped");
							return;
						}
						const auto e = geo->ToEnu(Num(c.fields[0]), Num(c.fields[1]), Num(c.fields[2]));
						m.x = e.x;
						m.y = e.y;
						m.z = e.z;
					}
					m.tol_xy = Num(c.fields[3]);
					m.tol_z = Num(c.fields[4]);
					for (const auto& o : set.markers)
						if (o.task == m.task && o.x == m.x && o.y == m.y && o.z == m.z && o.tol_xy == m.tol_xy && o.tol_z == m.tol_z) return;
					set.markers.push_back(m);
				}

				// Conditions anywhere under r that belong to the task `label`: not inside the children of
				// a complex task, which are tasks of their own (spec 11 §2.1: child i of p is p.i).
				void Conditions(const Record& r, const std::string& label) {
					if (r.code == "C_P3" || r.code == "C_G") Add(r, label);
					for (const auto& c : r.children) {
						if (c.kind == 'T' && IsComplex(r)) continue;
						Conditions(c, label);
					}
				}

				void Task(const Record& t, const std::string& label) {
					Conditions(t, label);
					if (!IsComplex(t)) return;
					std::size_t i = 0;
					for (const auto& f : t.fields)
						if (f.type == FieldType::Ref && t.children.at(f.ref).kind == 'T') Task(t.children[f.ref], label + "." + std::to_string(++i));
				}

				void Root(const Record& t) {
					std::string id = !t.fields.empty() && t.fields[0].type == FieldType::TaskId ? t.fields[0].s : "0";
					if (id == "0") id = "task" + std::to_string(set.markers.size() + 1);
					Task(t, id);
				}
			};
		}

		MarkerState TaskMarker::StateAt(double t) const {
			if (t < dispatch) return MarkerState::HIDDEN;
			if (done && t >= *done) return MarkerState::DONE;
			if (failed && t >= *failed) return MarkerState::FAILED;
			return MarkerState::OPEN;
		}

		Environment::GeoReference GeoOf(const Record& mission) {
			return Environment::GeoReference(Num(mission.fields.at(1)), Num(mission.fields.at(2)), Num(mission.fields.at(3)));
		}

		void AddMarkers(const Protocol::Document& doc, const Environment::GeoReference* geo, TaskSet& set) {
			std::optional<Environment::GeoReference> own;
			for (const auto& r : doc.records) {
				if (r.code != "H_M") continue;
				if (!set.mission) set.mission = r;
				if (!geo) {
					own = GeoOf(r);
					geo = &*own;
				}
			}
			Walker w{geo, set, 0.0};
			for (const auto& r : doc.records) {
				if (r.kind == 'T') {
					w.dispatch = 0.0;
					w.Root(r);
				} else if (r.code == "L_D" && !r.fields.empty()) {
					w.dispatch = Num(r.fields[0]);
					for (const auto& f : r.fields)
						if (f.type == FieldType::Ref && r.children.at(f.ref).kind == 'T') w.Root(r.children[f.ref]);
				}
			}
		}

		void AddMarkerFile(const std::string& path, const Environment::GeoReference* geo, TaskSet& set) {
			std::ifstream in(path, std::ios::binary);
			if (!in) {
				set.warnings.push_back("cannot open " + path);
				return;
			}
			std::stringstream s;
			s << in.rdbuf();
			auto parsed = Protocol::Parse(s.str());
			if (!parsed.Ok()) {
				set.warnings.push_back(path + ": byte " + std::to_string(parsed.error->offset) + ": " + parsed.error->message);
				return;
			}
			AddMarkers(parsed.document, geo, set);
		}

		void ApplyEvents(const std::vector<TrackEvent>& events, TaskSet& set) {
			for (auto& m : set.markers) {
				for (const auto& e : events) {
					if (e.kind == "dispatch" && e.task == m.task) m.dispatch = e.t;
					if (!Under(m.task, e.task)) continue;
					if (e.kind == "done" && !m.done) {
						m.done = e.t;
						m.done_by = e.detail;
					}
					if (e.kind == "failed" && !m.failed) m.failed = e.t;
				}
			}
		}
	}
}
