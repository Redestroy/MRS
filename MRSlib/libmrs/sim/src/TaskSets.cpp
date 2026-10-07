#include "mrs/sim/TaskSets.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

namespace MRS {
	namespace Sim {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		const char* FamilyName(Family f) {
			switch (f) {
			case Family::GRID: return "grid";
			case Family::RADIAL: return "radial";
			case Family::CLUSTER: return "cluster";
			case Family::MULTICLUSTER: return "multicluster";
			case Family::RANDOM: return "random";
			}
			return "?";
		}

		const char* DispatchName(Dispatch d) {
			switch (d) {
			case Dispatch::STATIC: return "static";
			case Dispatch::EVEN: return "even";
			case Dispatch::CLUSTERED: return "clustered";
			case Dispatch::RANDOM: return "random";
			}
			return "?";
		}

		bool ParseFamily(const std::string& s, Family& f) {
			for (Family x : {Family::GRID, Family::RADIAL, Family::CLUSTER, Family::MULTICLUSTER, Family::RANDOM})
				if (s == FamilyName(x)) {
					f = x;
					return true;
				}
			return false;
		}

		bool ParseDispatch(const std::string& s, Dispatch& d) {
			for (Dispatch x : {Dispatch::STATIC, Dispatch::EVEN, Dispatch::CLUSTERED, Dispatch::RANDOM})
				if (s == DispatchName(x)) {
					d = x;
					return true;
				}
			return false;
		}

		namespace {
			// SplitMix64: the same numbers on every platform (std distributions are not).
			class Rng {
			public:
				explicit Rng(std::uint64_t seed) : s_(seed) {}
				std::uint64_t Next() {
					std::uint64_t z = (s_ += 0x9E3779B97F4A7C15ULL);
					z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
					z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
					return z ^ (z >> 31);
				}
				double Uniform() { return static_cast<double>(Next() >> 11) * (1.0 / 9007199254740992.0); }
				double Uniform(double a, double b) { return a + (b - a) * Uniform(); }
				double Normal(double sigma) {
					const double u = std::max(Uniform(), 1e-12), v = Uniform();
					return sigma * std::sqrt(-2.0 * std::log(u)) * std::cos(6.283185307179586 * v);
				}

			private:
				std::uint64_t s_;
			};

			double Round(double v, double step) { return std::round(v / step) * step; }

			std::vector<std::array<double, 2>> Points(const GenConfig& c, Rng& rng) {
				const double h = c.half_size;
				const auto clamp = [h](double v) { return std::clamp(v, -h, h); };
				std::vector<std::array<double, 2>> p;
				switch (c.family) {
				case Family::GRID: {
					const int cols = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(c.tasks))));
					const int rows = (c.tasks + cols - 1) / cols;
					const double sx = 2 * h / cols, sy = 2 * h / rows;
					for (int r = 0; r < rows; ++r)
						for (int k = 0; k < cols && static_cast<int>(p.size()) < c.tasks; ++k) p.push_back({-h + sx * (k + 0.5), -h + sy * (r + 0.5)});
					// Grid order is not dispatch order.
					for (std::size_t i = p.size(); i > 1; --i) std::swap(p[i - 1], p[static_cast<std::size_t>(rng.Next() % i)]);
					break;
				}
				case Family::RADIAL:
					for (int k = 0; k < c.tasks; ++k) {
						const double a = (k % 8) * 0.7853981633974483 + rng.Uniform(-0.087, 0.087);
						const double r = h * rng.Uniform(0.2, 1.0);
						p.push_back({clamp(r * std::cos(a)), clamp(r * std::sin(a))});
					}
					break;
				case Family::CLUSTER: {
					const double cx = rng.Uniform(-0.6 * h, 0.6 * h), cy = rng.Uniform(-0.6 * h, 0.6 * h);
					for (int k = 0; k < c.tasks; ++k) p.push_back({clamp(cx + rng.Normal(8.0)), clamp(cy + rng.Normal(8.0))});
					break;
				}
				case Family::MULTICLUSTER: {
					std::array<std::array<double, 2>, 4> centres{};
					for (auto& ce : centres) ce = {rng.Uniform(-0.7 * h, 0.7 * h), rng.Uniform(-0.7 * h, 0.7 * h)};
					for (int k = 0; k < c.tasks; ++k) {
						const auto& ce = centres[static_cast<std::size_t>(k % 4)];
						p.push_back({clamp(ce[0] + rng.Normal(6.0)), clamp(ce[1] + rng.Normal(6.0))});
					}
					break;
				}
				case Family::RANDOM:
					for (int k = 0; k < c.tasks; ++k) p.push_back({rng.Uniform(-h, h), rng.Uniform(-h, h)});
					break;
				}
				return p;
			}

			std::vector<double> Times(const GenConfig& c, Rng& rng) {
				std::vector<double> t;
				double now = c.first;
				for (int k = 0; k < c.tasks; ++k) {
					switch (c.dispatch) {
					case Dispatch::STATIC: t.push_back(c.first); break;
					case Dispatch::EVEN: t.push_back(c.first + k * c.interval); break;
					case Dispatch::CLUSTERED: t.push_back(c.first + (k / std::max(1, c.burst)) * c.burst_gap); break;
					case Dispatch::RANDOM:
						if (k > 0) now += -c.interval * std::log(std::max(1e-12, 1.0 - rng.Uniform()));
						t.push_back(Round(now, 0.01));
						break;
					}
				}
				return t;
			}

			std::string Num(double v) { return Protocol::FormatNumber(std::fabs(v) < 5e-7 ? 0.0 : v); }
		}

		std::string GenerateTaskSet(const GenConfig& c) {
			Rng rng(c.seed * 0x2545F4914F6CDD1DULL + static_cast<std::uint64_t>(c.family) * 131 + static_cast<std::uint64_t>(c.dispatch));
			const auto points = Points(c, rng);
			std::vector<double> z;
			for (int k = 0; k < c.tasks; ++k) z.push_back(Round(rng.Uniform(c.z_min, c.z_max), 0.01));
			const auto times = Times(c, rng);
			static const long kColours[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0x00FFFF, 0xFF00FF, 0xFFFFFF, 0xFF8000};
			std::ostringstream out;
			out << "# Generated task set (spec 10 §3.2): family " << FamilyName(c.family) << ", dispatch " << DispatchName(c.dispatch) << ", "
			    << c.tasks << " tasks, seed " << c.seed << ", area ±" << Num(c.half_size) << " m, altitude " << Num(c.z_min) << " to "
			    << Num(c.z_max) << " m.\n@: MRS 0.1/\n";
			for (int k = 0; k < c.tasks; ++k) {
				const auto& p = points[static_cast<std::size_t>(k)];
				out << "\nL: L_D " << Num(times[static_cast<std::size_t>(k)]) << " T_1/\n";
				out << "T_1: T_A " << c.issuer << "." << (k + 1) << " 10 0 C_1 C_2 A_1..5/\n";
				out << "C_1: C_P3 " << Num(Round(p[0], 0.01)) << " " << Num(Round(p[1], 0.01)) << " " << Num(z[static_cast<std::size_t>(k)]) << " "
				    << Num(c.tol_xy) << " " << Num(c.tol_z) << " 0 -1/\nC_2: C_N/\n";
				out << "A_1: A_L 3 " << kColours[k % 8] << "/\nA_2: A_W 3/\nA_3: A_L 3 0/\nA_4: A_W 3/\nA_5: A_N/\n";
			}
			return out.str();
		}

		namespace {
			Protocol::Document ParseOrThrow(const std::string& text) {
				auto parsed = Protocol::Parse(text);
				if (!parsed.Ok())
					throw std::runtime_error(std::string(Protocol::ErrorClassName(parsed.error->error_class)) + " at byte " +
					                         std::to_string(parsed.error->offset) + ": " + parsed.error->message);
				return parsed.document;
			}

			// The tasks of an L_D record, as references into its children.
			std::vector<const Record*> TasksOf(const Record& l) {
				std::vector<const Record*> out;
				for (std::size_t k = 1; k < l.fields.size(); ++k)
					if (l.fields[k].type == FieldType::Ref) out.push_back(&l.children.at(l.fields[k].ref));
				return out;
			}

			const Record& Slot(const Record& r, std::size_t k) { return r.children.at(r.fields.at(k).ref); }

			// T_S id: go to the target, then wait for the release and do the task (spec 10 §3.3).
			Record OracleTask(const Record& task, double release) {
				if (task.code != "T_A" && task.code != "T_P") throw std::runtime_error("oracle form needs T_A or T_P tasks: " + task.code);
				static const Record shape = [] {
					return ParseOrThrow("T: T_S op.1 1 0 C_1 C_2 T_1 T_2/\nC_1: C_N/\nC_2: C_N/\n"
					                    "T_1: T_A 0 1 0 C_1 C_2 A_1/\nC_1: C_N/\nC_2: C_N/\nA_1: A_N/\n"
					                    "T_2: T_A 0 1 0 C_1 C_2 A_1/\nC_1: C_T 0/\nC_2: C_N/\nA_1: A_N/\n")
					    .records.at(0);
				}();
				Record root = shape;
				root.fields[0] = task.fields[0];
				root.fields[1] = task.fields[1];
				Record& go = root.children.at(root.fields[5].ref);
				Record& work = root.children.at(root.fields[6].ref);
				go.fields[1] = work.fields[1] = task.fields[1];
				go.children.at(go.fields[3].ref) = Slot(task, 3);  // the task's start condition
				Record& wait = work.children.at(work.fields[3].ref);
				wait.fields[0].i = static_cast<std::int64_t>(std::llround(release * 1000.0));
				work.children.at(work.fields[4].ref) = Slot(task, 4);  // its end condition
				// Its actions replace the placeholder A_N.
				const std::size_t first_action = 5;
				work.fields.resize(first_action);
				work.children.resize(2);
				for (std::size_t k = first_action; k < task.fields.size(); ++k) {
					if (task.fields[k].type != FieldType::Ref) continue;
					work.fields.push_back(Field::MakeRef(work.children.size()));
					work.children.push_back(Slot(task, k));
				}
				return root;
			}
		}

		std::string OracleTimeline(const std::string& timeline) {
			const auto doc = ParseOrThrow(timeline);
			Protocol::Document out;
			for (const auto& r : doc.records) {
				if (r.code != "L_D") {
					if (r.kind == '@') out.records.push_back(r);
					continue;
				}
				for (const Record* task : TasksOf(r)) {
					Record l;
					l.kind = 'L';
					l.code = "L_D";
					l.fields = {Field::MakeNum(0.0), Field::MakeRef(0)};
					l.children = {OracleTask(*task, r.fields[0].n)};
					out.records.push_back(l);
				}
			}
			return "# Oracle form (spec 10 §3.3): every task at time 0, released by C_T.\n" + Protocol::Write(out);
		}

		std::map<std::string, double> ReleaseTimes(const std::string& timeline) {
			std::map<std::string, double> out;
			for (const auto& r : ParseOrThrow(timeline).records) {
				if (r.code != "L_D") continue;
				for (const Record* task : TasksOf(r)) out[task->fields.at(0).s] = r.fields[0].n;
			}
			return out;
		}
	}
}
