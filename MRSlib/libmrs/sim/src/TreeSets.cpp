#include <cmath>
#include <utility>
#include <vector>

#include "mrs/generators/Generators.h"
#include "mrs/sim/TaskSets.h"

namespace MRS {
	namespace Sim {
		const char* TreeFamilyName(TreeFamily f) {
			switch (f) {
			case TreeFamily::COVERAGE: return "coverage";
			case TreeFamily::PERIMETER: return "perimeter";
			case TreeFamily::SEARCH: return "search";
			case TreeFamily::MIXED: return "mixed";
			}
			return "?";
		}

		bool ParseTreeFamily(const std::string& s, TreeFamily& f) {
			for (TreeFamily x : {TreeFamily::COVERAGE, TreeFamily::PERIMETER, TreeFamily::SEARCH, TreeFamily::MIXED})
				if (s == TreeFamilyName(x)) {
					f = x;
					return true;
				}
			return false;
		}

		namespace {
			// SplitMix64, as in TaskSets.cpp.
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

			private:
				std::uint64_t s_;
			};

			double Round(double v) { return std::round(v * 100.0) / 100.0; }

			// A rectangle of w × h m turned by `heading`, centred where it fits inside the area.
			std::vector<Generators::Point> Rectangle(Rng& rng, double half_size, double w, double h) {
				const double heading = rng.Uniform(0.0, 3.141592653589793);
				const double reach = 0.5 * std::hypot(w, h);
				const double cx = rng.Uniform(-half_size + reach, half_size - reach), cy = rng.Uniform(-half_size + reach, half_size - reach);
				const double c = std::cos(heading), s = std::sin(heading);
				std::vector<Generators::Point> out;
				for (const auto& [u, v] : {std::pair{-0.5, -0.5}, std::pair{0.5, -0.5}, std::pair{0.5, 0.5}, std::pair{-0.5, 0.5}})
					out.push_back({Round(cx + u * w * c - v * h * s), Round(cy + u * w * s + v * h * c), 0.0});
				return out;
			}
		}

		std::string GenerateTreeSet(const TreeGenConfig& c) {
			Rng rng(c.seed * 0x9E3779B97F4A7C15ULL + 7919ULL * static_cast<std::uint64_t>(c.family) + 1);
			std::vector<std::pair<double, Protocol::Record>> entries;
			for (int k = 0; k < c.trees; ++k) {
				const std::string id = c.issuer + "." + std::to_string(k + 1);
				const TreeFamily f = c.family == TreeFamily::MIXED ? static_cast<TreeFamily>(k % 3) : c.family;
				const double at = c.first + c.interval * k;
				const double alt = Round(rng.Uniform(10.0, 20.0));
				switch (f) {
				case TreeFamily::COVERAGE: {
					Generators::CoverageOptions o;
					o.altitude = alt;
					o.footprint = 8.0;
					o.cells = c.parts;
					entries.emplace_back(at, Generators::CoverageTask(id, Rectangle(rng, c.half_size, rng.Uniform(30, 50), rng.Uniform(30, 50)), o));
					break;
				}
				case TreeFamily::PERIMETER: {
					Generators::PerimeterOptions o;
					o.altitude = alt;
					o.spacing = 10.0;
					o.arcs = c.parts;
					entries.emplace_back(at, Generators::PerimeterTask(id, Rectangle(rng, c.half_size, rng.Uniform(40, 80), rng.Uniform(40, 80)), o));
					break;
				}
				default: {
					Generators::SearchOptions o;
					o.altitude = alt;
					o.radius = 10.0;
					o.footprint = 8.0;
					std::vector<Generators::Point> pois;
					const double m = c.half_size - o.radius;
					for (int p = 0; p < c.parts; ++p) pois.push_back({Round(rng.Uniform(-m, m)), Round(rng.Uniform(-m, m)), 0.0});
					entries.emplace_back(at, Generators::SearchTask(id, pois, o));
					break;
				}
				}
			}
			return Generators::Timeline(entries, "Generated tree-task set (spec 12 §4): family " + std::string(TreeFamilyName(c.family)) + ", " +
			                                         std::to_string(c.trees) + " trees of " + std::to_string(c.parts) + " parts, seed " +
			                                         std::to_string(c.seed) + ".");
		}
	}
}
