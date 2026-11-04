#pragma once
// Packages for the delivery demo in the test simulator (spec 18 §2.4): piles at source points,
// packages on the hook of a sling, packages placed at their destinations.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace MRS {
	namespace Sim {
		struct Package {
			enum class State { PILE, CARRIED, PLACED };
			std::int64_t id = 0;
			std::array<double, 3> pos{0, 0, 0};   // ENU, the centre of the package's bottom
			std::array<double, 3> dest{0, 0, 0};  // ENU, written on the package's tag
			std::int64_t area = 0;                // destination area number, on the tag
			double mass = 0.3;                    // kg
			double height = 0.25;                 // m
			State state = State::PILE;
			int carrier = 0;                      // robot id while CARRIED, and of the last carrier
			double t_picked = -1, t_placed = -1;  // s
			double drop_height = 0;               // m above the ground when released
		};

		// Every package of a run. One world is shared by the team's simulators.
		struct PackageWorld {
			std::vector<Package> packages;
			double loop_height = 0.10;  // m, the loop above the package's top
			double capture_xy = 0.35;   // m: the hook catches a loop this close horizontally
			double capture_z = 0.15;    // m, and vertically
			double hard_drop = 0.3;     // m: a release higher than this above the ground is a drop

			// A stack of n packages at (x, y) on the ground, ids from first_id, with the given tags.
			void AddPile(double x, double y, const std::vector<Package>& tags) {
				double z = 0.0;
				for (Package p : tags) {
					p.pos = {x, y, z};
					p.state = Package::State::PILE;
					z += p.height;
					packages.push_back(p);
				}
			}

			// The topmost pile package whose loop is within capture distance of the hook point, or nullptr.
			Package* Catch(const std::array<double, 3>& hook) {
				for (auto& p : packages) {
					if (p.state != Package::State::PILE || !Top(p)) continue;
					const double lz = p.pos[2] + p.height + loop_height;
					if (std::hypot(hook[0] - p.pos[0], hook[1] - p.pos[1]) <= capture_xy && std::fabs(hook[2] - lz) <= capture_z) return &p;
				}
				return nullptr;
			}

			// No pile package lies on top of p.
			bool Top(const Package& p) const {
				for (const auto& q : packages)
					if (&q != &p && q.state == Package::State::PILE && q.pos[2] > p.pos[2] &&
					    std::hypot(q.pos[0] - p.pos[0], q.pos[1] - p.pos[1]) < 0.2)
						return false;
				return true;
			}

			// The height of whatever is below (x, y): the top of a placed or pile package there, or the ground.
			double SupportAt(double x, double y, const Package* except) const {
				double z = 0.0;
				for (const auto& q : packages)
					if (&q != except && q.state != Package::State::CARRIED && std::hypot(q.pos[0] - x, q.pos[1] - y) < 0.2)
						z = std::max(z, q.pos[2] + q.height);
				return z;
			}

			Package* Find(std::int64_t id) {
				for (auto& p : packages)
					if (p.id == id) return &p;
				return nullptr;
			}

			long Placed() const {
				long n = 0;
				for (const auto& p : packages) n += p.state == Package::State::PLACED;
				return n;
			}
		};
	}
}
