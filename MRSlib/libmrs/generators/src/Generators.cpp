#include "mrs/generators/Generators.h"

#include <algorithm>
#include <map>
#include <sstream>

#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

namespace MRS {
	namespace Generators {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		namespace {
			constexpr double kPi = 3.14159265358979323846;

			Record Parse(const std::string& text) {
				auto parsed = Protocol::Parse(text);
				if (!parsed.Ok()) throw JsonError("internal: generated text does not parse: " + parsed.error->message + "\n" + text);
				return parsed.document.records.at(0);
			}

			std::string Num(double v) {
				const double r = std::round(v * 1000.0) / 1000.0;  // mm
				return Protocol::FormatNumber(std::fabs(r) < 5e-4 ? 0.0 : r);
			}

			std::string P3(const Point& p, double tol_xy, double tol_z) {
				return "C_P3 " + Num(p[0]) + " " + Num(p[1]) + " " + Num(p[2]) + " " + Num(tol_xy) + " " + Num(tol_z) + " 0 -1";
			}

			// A complex node: code, id, priority, [k], start, end, children.
			Record Node(const std::string& code, const std::string& id, double priority, int k, const std::vector<Record>& children) {
				Record r;
				r.kind = 'T';
				r.code = code;
				r.fields = {Field::MakeText(FieldType::TaskId, id), Field::MakeNum(priority), Field::MakeInt(0)};
				if (code == "T_L") r.fields.push_back(Field::MakeInt(k));
				const Record null = Parse("C: C_N/\n");
				for (int c = 0; c < 2; ++c) {
					r.fields.push_back(Field::MakeRef(r.children.size()));
					r.children.push_back(null);
				}
				for (const auto& child : children) {
					r.fields.push_back(Field::MakeRef(r.children.size()));
					r.children.push_back(child);
				}
				return r;
			}

			void SetId(Record& r, const std::string& id) { r.fields.at(0) = Field::MakeText(FieldType::TaskId, id); }

			// Moves affinity ids under `from` to `to` (a tree that became a child).
			void Reroot(Record& r, const std::string& from, const std::string& to) {
				if (r.code == "R_K" && !r.fields.empty()) {
					std::string& s = r.fields[0].s;
					if (s == from || s.rfind(from + ".", 0) == 0) s = to + s.substr(from.size());
				}
				for (auto& c : r.children) Reroot(c, from, to);
			}

			// The first leaf flies to the waypoint with the robot's fly-to; later ones are level legs.
			Record FirstLeaf(const Point& p, const ChainOptions& o) {
				return Parse("T: T_A 0 1 0 C_1 C_2 A_1/\nC_1: " + P3(p, o.tol_xy, o.tol_z) + "/\nC_2: C_N/\nA_1: A_N/\n");
			}

			Record Leg(const Point& from, const Point& p, const std::string& after, const ChainOptions& o) {
				const double yaw = std::atan2(p[1] - from[1], p[0] - from[0]);
				return Parse("T: T_B 0 1 0 C_1 C_2 C_3 R_1 T_1/\nC_1: C_? ?_1/\n?_1: airborne T/\nC_2: C_N/\nC_3: " + P3(p, o.tol_xy, o.tol_z) +
				             "/\nR_1: R_K " + after + "/\nT_1: T_P 0 1 0 C_1 C_2 A_1..2/\nC_1: C_N/\nC_2: C_N/\nA_1: A_MAP any A_1 any A_2/\nA_1: A_PXY " +
				             Num(p[0]) + " " + Num(p[1]) + "/\nA_2: A_PZY " + Num(p[2]) + " " + Num(yaw) + "/\nA_2: A_N/\n");
			}

			// Splits a list into n contiguous groups of about equal weight.
			template <typename T>
			std::vector<std::vector<T>> Split(const std::vector<T>& items, const std::vector<double>& weight, int n) {
				n = std::max(1, std::min<int>(n, static_cast<int>(items.size())));
				double total = 0.0;
				for (double w : weight) total += w;
				std::vector<std::vector<T>> out(static_cast<std::size_t>(n));
				double acc = 0.0;
				std::size_t g = 0;
				for (std::size_t i = 0; i < items.size(); ++i) {
					// Leave at least one item for each group still to come.
					const std::size_t left = items.size() - i, groups_left = static_cast<std::size_t>(n) - g;
					if (g + 1 < static_cast<std::size_t>(n) && !out[g].empty() &&
					    (acc + weight[i] / 2 > total * static_cast<double>(g + 1) / n || left < groups_left))
						++g;
					out[g].push_back(items[i]);
					acc += weight[i];
				}
				return out;
			}
		}

		const char* PointActionName(PointAction a) {
			switch (a) {
			case PointAction::LEDS: return "leds";
			case PointAction::HOVER: return "hover";
			case PointAction::LAND: return "land";
			case PointAction::PICTURE: return "picture";
			case PointAction::GIMBAL: return "gimbal";
			case PointAction::RELEASE: return "release";
			}
			return "?";
		}

		bool ParsePointAction(const std::string& s, PointAction& a) {
			for (auto x : {PointAction::LEDS, PointAction::HOVER, PointAction::LAND, PointAction::PICTURE, PointAction::GIMBAL, PointAction::RELEASE})
				if (s == PointActionName(x)) {
					a = x;
					return true;
				}
			return false;
		}

		Record PointTask(const std::string& id, const Point& p, const PointOptions& o) {
			const std::string head = "T: T_A " + id + " " + Num(o.priority) + " 0 C_1 C_2 ";
			const std::string at = "C_1: " + P3(p, o.tol_xy, o.tol_z) + "/\nC_2: C_N/\n";
			const std::string d = Num(o.duration);
			switch (o.action) {
			case PointAction::LEDS:
				return Parse(head + "A_1..5/\n" + at + "A_1: A_L " + std::to_string(o.led_mask) + " " + std::to_string(o.colour) + "/\nA_2: A_W " + d +
				             "/\nA_3: A_L " + std::to_string(o.led_mask) + " 0/\nA_4: A_W " + d + "/\nA_5: A_N/\n");
			case PointAction::HOVER: return Parse(head + "A_1..2/\n" + at + "A_1: A_W " + d + "/\nA_2: A_N/\n");
			case PointAction::LAND: return Parse(head + "A_1..2/\n" + at + "A_1: A_LD " + Num(o.descent) + " 0/\nA_2: A_N/\n");
			case PointAction::PICTURE:
				return Parse(head + "A_1..4/\n" + at + "A_1: A_GMB " + Num(o.pitch) + " " + Num(o.yaw) + "/\nA_2: A_W " + Num(std::min(o.duration, 1.0)) +
				             "/\nA_3: A_CAM 1 0/\nA_4: A_N/\n");
			case PointAction::GIMBAL: return Parse(head + "A_1..2/\n" + at + "A_1: A_GMB " + Num(o.pitch) + " " + Num(o.yaw) + "/\nA_2: A_N/\n");
			case PointAction::RELEASE: {
				// Land at the point, then release: only a robot carrying the package may take it, and
				// the release starts only on the ground (JB, 2026-10-08).
				const std::string pkg = std::to_string(o.package);
				Record land = Parse("T: T_A 0 1 0 C_1 C_2 R_1 A_1..2/\n" + at + "R_1: R_C C_1/\nC_1: C_m payload.id eq " + pkg + " 0.5/\nA_1: A_LD " +
				                    Num(o.descent) + " 0/\nA_2: A_N/\n");
				Record release = Parse("T: T_A 0 1 0 C_1 C_2 R_1 A_1..3/\nC_1: C_? ?_1/\n?_1: landed T/\nC_2: C_N/\nR_1: R_K " + id + ".1/\nA_1: A_REL " +
				                       pkg + "/\nA_2: A_W 1/\nA_3: A_N/\n");
				return Node("T_S", id, o.priority, 0, {land, release});
			}
			}
			throw JsonError("unknown point action");
		}

		Record ChainTask(const std::string& id, const std::vector<Point>& wps, const ChainOptions& o) {
			if (wps.empty()) throw JsonError("a chain needs at least one waypoint");
			std::vector<Record> leaves{FirstLeaf(wps[0], o)};
			for (std::size_t i = 1; i < wps.size(); ++i) leaves.push_back(Leg(wps[i - 1], wps[i], id + "." + std::to_string(i), o));
			return Node("T_S", id, o.priority, 0, leaves);
		}

		Record PathTask(const std::string& id, const std::vector<Point>& line, const ChainOptions& o) { return ChainTask(id, line, o); }

		std::vector<std::pair<Point, Point>> CoverageTracks(const std::vector<Point>& poly, const CoverageOptions& o) {
			if (poly.size() < 3) throw JsonError("a coverage polygon has at least 3 corners");
			const double spacing = o.footprint * (1.0 - o.overlap);
			if (spacing <= 0.0) throw JsonError("coverage: footprint × (1 − overlap) must be > 0");
			double heading = o.heading;
			if (std::isnan(heading)) {
				double best = -1.0;
				for (std::size_t i = 0; i < poly.size(); ++i) {
					const Point& a = poly[i];
					const Point& b = poly[(i + 1) % poly.size()];
					const double len = std::hypot(b[0] - a[0], b[1] - a[1]);
					if (len > best) {
						best = len;
						heading = std::atan2(b[1] - a[1], b[0] - a[0]);
					}
				}
			}
			// Track frame: u along the tracks, v across.
			const double c = std::cos(heading), s = std::sin(heading);
			std::vector<std::array<double, 2>> uv;
			double vmin = 1e18, vmax = -1e18;
			for (const auto& p : poly) {
				uv.push_back({c * p[0] + s * p[1], -s * p[0] + c * p[1]});
				vmin = std::min(vmin, uv.back()[1]);
				vmax = std::max(vmax, uv.back()[1]);
			}
			const int n = std::max(1, static_cast<int>(std::ceil((vmax - vmin) / spacing - 1e-9)));
			const double first = vmin + (vmax - vmin - (n - 1) * spacing) / 2.0;  // centred
			std::vector<std::pair<Point, Point>> tracks;
			for (int k = 0; k < n; ++k) {
				const double v = first + k * spacing;
				double umin = 1e18, umax = -1e18;
				for (std::size_t i = 0; i < uv.size(); ++i) {
					const auto& a = uv[i];
					const auto& b = uv[(i + 1) % uv.size()];
					if ((a[1] - v) * (b[1] - v) > 0.0 || a[1] == b[1]) continue;
					const double u = a[0] + (b[0] - a[0]) * (v - a[1]) / (b[1] - a[1]);
					umin = std::min(umin, u);
					umax = std::max(umax, u);
				}
				if (umin > umax) continue;
				auto xy = [&](double u) { return Point{c * u - s * v, s * u + c * v, o.altitude}; };
				// Boustrophedon: every other track the other way.
				if (k % 2 == 0) tracks.push_back({xy(umin), xy(umax)});
				else tracks.push_back({xy(umax), xy(umin)});
			}
			return tracks;
		}

		Record CoverageTask(const std::string& id, const std::vector<Point>& poly, const CoverageOptions& o) {
			const auto tracks = CoverageTracks(poly, o);
			if (tracks.empty()) throw JsonError("coverage: no track crosses the polygon");
			std::vector<double> weight;
			for (const auto& t : tracks) weight.push_back(std::hypot(t.second[0] - t.first[0], t.second[1] - t.first[1]) + o.footprint);
			const auto cells = Split(tracks, weight, o.cells);
			std::vector<Record> children;
			for (std::size_t k = 0; k < cells.size(); ++k) {
				std::vector<Point> wps;
				for (auto t : cells[k]) {
					// Each track starts at the end nearer the last waypoint (the sweep alternates).
					if (!wps.empty() && std::hypot(wps.back()[0] - t.first[0], wps.back()[1] - t.first[1]) >
					                        std::hypot(wps.back()[0] - t.second[0], wps.back()[1] - t.second[1]))
						std::swap(t.first, t.second);
					wps.push_back(t.first);
					wps.push_back(t.second);
				}
				Record chain = ChainTask(id + "." + std::to_string(k + 1), wps, o.chain);
				SetId(chain, "0");
				children.push_back(chain);
			}
			return Node("T_L", id, o.chain.priority, 0, children);
		}

		Record PerimeterTask(const std::string& id, const std::vector<Point>& poly, const PerimeterOptions& o) {
			if (poly.size() < 3) throw JsonError("a perimeter polygon has at least 3 corners");
			// The closed ring, with no leg longer than `spacing`.
			std::vector<Point> ring;
			std::vector<double> weight;
			for (std::size_t i = 0; i < poly.size(); ++i) {
				const Point& a = poly[i];
				const Point& b = poly[(i + 1) % poly.size()];
				const double len = std::hypot(b[0] - a[0], b[1] - a[1]);
				const int parts = std::max(1, static_cast<int>(std::ceil(len / std::max(1.0, o.spacing))));
				for (int k = 0; k < parts; ++k) {
					const double f = static_cast<double>(k) / parts;
					ring.push_back({a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), o.altitude});
					weight.push_back(len / parts);
				}
			}
			ring.push_back({poly[0][0], poly[0][1], o.altitude});  // back to the start
			weight.push_back(0.0);
			if (o.arcs <= 1) return ChainTask(id, ring, o.chain);
			// Arcs share their end points, so the boundary is flown without gaps.
			const auto arcs = Split(std::vector<Point>(ring.begin(), ring.end() - 1), std::vector<double>(weight.begin(), weight.end() - 1), o.arcs);
			std::vector<Record> children;
			std::size_t at = 0;
			for (std::size_t k = 0; k < arcs.size(); ++k) {
				std::vector<Point> wps = arcs[k];
				at += arcs[k].size();
				wps.push_back(ring[at]);
				Record chain = ChainTask(id + "." + std::to_string(k + 1), wps, o.chain);
				SetId(chain, "0");
				children.push_back(chain);
			}
			return Node("T_L", id, o.chain.priority, 0, children);
		}

		std::vector<Point> SpiralWaypoints(const Point& c, const SearchOptions& o) {
			const double pitch = o.footprint * (1.0 - o.overlap);  // between turns
			if (pitch <= 0.0 || o.step <= 0.0) throw JsonError("search: footprint × (1 − overlap) and step must be > 0");
			const double z = c[2] > 0.0 ? c[2] : o.altitude;
			const double a = pitch / (2.0 * kPi);  // r = a·θ
			std::vector<Point> out{{c[0], c[1], z}};
			// The first turn starts half a pitch out, so the centre is covered once.
			double theta = kPi;
			while (a * theta <= o.radius + 1e-9) {
				const double r = a * theta;
				out.push_back({c[0] + r * std::cos(theta), c[1] + r * std::sin(theta), z});
				// The next point about `step` along the spiral (ds = √(r² + a²)·dθ), shortened until
				// the leg itself is at most `step` (r grows along it).
				double d = o.step / std::sqrt(r * r + a * a);
				auto leg = [&](double dt) {
					const double r2 = a * (theta + dt);
					return std::hypot(r2 * std::cos(theta + dt) - r * std::cos(theta), r2 * std::sin(theta + dt) - r * std::sin(theta));
				};
				while (leg(d) > o.step) d *= 0.98;
				theta += d;
			}
			return out;
		}

		Record SearchTask(const std::string& id, const std::vector<Point>& pois, const SearchOptions& o) {
			if (pois.empty()) throw JsonError("search: no point of interest");
			if (pois.size() == 1) return ChainTask(id, SpiralWaypoints(pois[0], o), o.chain);
			std::vector<Record> children;
			for (std::size_t k = 0; k < pois.size(); ++k) {
				Record chain = ChainTask(id + "." + std::to_string(k + 1), SpiralWaypoints(pois[k], o), o.chain);
				SetId(chain, "0");
				children.push_back(chain);
			}
			return Node("T_L", id, o.chain.priority, 0, children);
		}

		Record ThenLand(const std::string& id, const Record& tree, const std::vector<std::string>& robots) {
			if (robots.empty()) throw JsonError("ThenLand: no robots");
			Record first = tree;
			if (first.code == "T_S" || first.code == "T_L" || first.code == "T_O") Reroot(first, first.fields[0].s, id + ".1");
			SetId(first, "0");
			std::vector<Record> lands;
			for (const auto& r : robots)
				lands.push_back(Parse("T: T_A 0 1 0 C_1 C_2 R_1 A_1..2/\nC_1: C_? ?_1/\n?_1: home T/\nC_2: C_N/\nR_1: R_I " + r +
				                      "/\nA_1: A_LD 1 0/\nA_2: A_N/\n"));
			return Node("T_S", id, 1.0, 0, {first, Node("T_L", "0", 1.0, 0, lands)});
		}

		std::string TaskFile(const std::vector<Record>& tasks, const std::string& comment) {
			std::string out;
			if (!comment.empty()) out += "# " + comment + "\n";
			out += "@: MRS 0.1/\n\n";
			for (const auto& t : tasks) out += Protocol::Write(t) + "\n";
			return out;
		}

		std::string Timeline(const std::vector<std::pair<double, Record>>& entries, const std::string& comment) {
			std::string out;
			if (!comment.empty()) out += "# " + comment + "\n";
			out += "@: MRS 0.1/\n";
			for (const auto& [t, task] : entries) {
				Record l;
				l.kind = 'L';
				l.code = "L_D";
				l.fields = {Field::MakeNum(t), Field::MakeRef(0)};
				l.children = {task};
				out += "\n" + Protocol::Write(l);
			}
			return out;
		}

		std::vector<Record> TasksFromGeoJson(const std::vector<Feature>& features, const GeoJsonOptions& o) {
			std::vector<Record> out;
			int next = o.first_id;
			auto new_id = [&]() { return o.issuer + "." + std::to_string(next++); };
			auto finish = [&](Record task) {
				if (!o.land_robots.empty() && (task.code == "T_S" || task.code == "T_L")) {
					const std::string id = task.fields[0].s;
					task = ThenLand(id, task, o.land_robots);
				}
				out.push_back(std::move(task));
			};
			auto chain_opts = [](const Json& p) {
				ChainOptions c;
				c.priority = p.Number("priority", c.priority);
				c.tol_xy = p.Number("tol_xy", c.tol_xy);
				c.tol_z = p.Number("tol_z", c.tol_z);
				return c;
			};
			std::map<std::string, std::vector<Point>> groups;  // search groups
			std::vector<std::string> group_order;
			std::map<std::string, SearchOptions> group_opts;
			int next_group = 0;
			for (const auto& f : features) {
				const Json& p = f.properties;
				const std::string task = p.String("task", f.geometry == "Point" ? "leds" : f.geometry == "LineString" ? "path" : "coverage");
				const double alt = p.Number("altitude", std::numeric_limits<double>::quiet_NaN());
				auto at_alt = [&](std::vector<Point> pts, double fallback) {
					for (auto& q : pts)
						if (!std::isnan(alt)) q[2] = alt;
						else if (q[2] == 0.0) q[2] = fallback;
					return pts;
				};
				if (task == "search") {
					SearchOptions s;
					s.altitude = std::isnan(alt) ? s.altitude : alt;
					s.radius = p.Number("radius", s.radius);
					s.footprint = p.Number("footprint", s.footprint);
					s.overlap = p.Number("overlap", s.overlap);
					s.step = p.Number("step", s.step);
					s.chain = chain_opts(p);
					const std::string g = p.String("group", "#" + std::to_string(next_group++));
					if (!groups.count(g)) {
						group_order.push_back(g);
						group_opts[g] = s;
					}
					for (const auto& q : f.points) groups[g].push_back(q);
					continue;
				}
				if (f.geometry == "Point") {
					PointOptions po;
					if (!ParsePointAction(task, po.action)) throw JsonError("unknown point task \"" + task + "\"");
					po.priority = p.Number("priority", po.priority);
					po.duration = p.Number("duration", po.duration);
					po.colour = static_cast<std::int64_t>(p.Number("colour", static_cast<double>(po.colour)));
					po.pitch = p.Number("pitch", po.pitch);
					po.yaw = p.Number("yaw", po.yaw);
					po.package = static_cast<std::int64_t>(p.Number("package", 0.0));
					out.push_back(PointTask(new_id(), at_alt(f.points, 15.0)[0], po));
				} else if (f.geometry == "LineString") {
					finish(PathTask(new_id(), at_alt(f.points, 20.0), chain_opts(p)));
				} else if (task == "perimeter") {
					PerimeterOptions po;
					po.altitude = std::isnan(alt) ? po.altitude : alt;
					po.spacing = p.Number("spacing", po.spacing);
					po.arcs = static_cast<int>(p.Number("arcs", po.arcs));
					po.chain = chain_opts(p);
					finish(PerimeterTask(new_id(), f.points, po));
				} else if (task == "coverage") {
					CoverageOptions co;
					co.altitude = std::isnan(alt) ? co.altitude : alt;
					co.footprint = p.Number("footprint", co.footprint);
					co.overlap = p.Number("overlap", co.overlap);
					co.cells = static_cast<int>(p.Number("cells", co.cells));
					if (p.Has("heading")) co.heading = p.Number("heading", 0.0);
					co.chain = chain_opts(p);
					finish(CoverageTask(new_id(), f.points, co));
				} else {
					throw JsonError("unknown task \"" + task + "\" for a " + f.geometry);
				}
			}
			for (const auto& g : group_order) finish(SearchTask(new_id(), groups[g], group_opts[g]));
			return out;
		}
	}
}
