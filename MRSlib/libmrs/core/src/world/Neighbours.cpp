#include "mrs/world/Neighbours.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace MRS {
	namespace Environment {
		namespace {
			bool Vec(const Worldview& w, const std::string& path, double t, Enu& out) {
				auto x = w.Scalar(path + ".x", t), y = w.Scalar(path + ".y", t), z = w.Scalar(path + ".z", t);
				if (!x || !y || !z) return false;
				out = {*x, *y, *z};
				return true;
			}

			// Like Vec, with an explicit max age (peer states have none of their own).
			bool VecAged(const Worldview& w, const std::string& path, double t, double max_age, Enu& out) {
				double v[3];
				const char* axis[3] = {".x", ".y", ".z"};
				for (int k = 0; k < 3; ++k) {
					auto e = w.Raw(path + axis[k]);
					if (!e || !e->valid || !std::holds_alternative<double>(e->value) || t - e->stamp > max_age) return false;
					v[k] = std::get<double>(e->value);
				}
				out = {v[0], v[1], v[2]};
				return true;
			}

			// The ids N of paths "<prefix>rN.<rest>".
			void IdsUnder(const Worldview& w, const std::string& prefix, std::map<std::string, bool>& ids) {
				for (const auto& p : w.PathsWithPrefix(prefix)) {
					const auto dot = p.find('.', prefix.size());
					if (dot != std::string::npos) ids.emplace(p.substr(prefix.size(), dot - prefix.size()), true);
				}
			}
		}

		std::vector<Neighbour> Neighbours(const Worldview& w, double t, double peer_max_age) {
			std::map<std::string, bool> ids;
			IdsUnder(w, "rel.", ids);
			IdsUnder(w, "peer.", ids);
			Enu own{}, own_vel{};
			const bool has_own = Vec(w, "pose.enu", t, own);
			const bool has_vel = Vec(w, "vel.enu", t, own_vel);
			std::vector<Neighbour> out;
			for (const auto& entry : ids) {
				const std::string& id = entry.first;
				Neighbour n;
				n.id = id;
				Enu p{};
				if (Vec(w, "rel." + id + ".enu", t, n.rel)) n.measured = true;
				else if (!has_own || !VecAged(w, "peer." + id + ".pose.enu", t, peer_max_age, p)) continue;
				else n.rel = {p.x - own.x, p.y - own.y, p.z - own.z};
				Enu v{};
				if (has_vel && VecAged(w, "peer." + id + ".vel.enu", t, peer_max_age, v))
					n.rel_vel = {v.x - own_vel.x, v.y - own_vel.y, v.z - own_vel.z};
				out.push_back(n);
			}
			return out;
		}

		Repulsion RepulsionFrom(const std::vector<Neighbour>& neighbours, const RepulsionConfig& c) {
			Repulsion r;
			for (const auto& n : neighbours) {
				const double dist = std::sqrt(n.rel.x * n.rel.x + n.rel.y * n.rel.y + n.rel.z * n.rel.z);
				if (r.nearest < 0.0 || dist < r.nearest) r.nearest = dist;
				// Look ahead to the closest approach, at most `lookahead` s.
				const double vv = n.rel_vel.x * n.rel_vel.x + n.rel_vel.y * n.rel_vel.y + n.rel_vel.z * n.rel_vel.z;
				double tau = 0.0;
				if (vv > 1e-9) tau = std::clamp(-(n.rel.x * n.rel_vel.x + n.rel.y * n.rel_vel.y + n.rel.z * n.rel_vel.z) / vv, 0.0, c.lookahead);
				const Enu d{n.rel.x + n.rel_vel.x * tau, n.rel.y + n.rel_vel.y * tau, n.rel.z + n.rel_vel.z * tau};
				const double R2 = c.radius_xy * c.radius_xy, H2 = c.radius_z * c.radius_z;
				const double s = std::sqrt((d.x * d.x + d.y * d.y) / R2 + d.z * d.z / H2);
				if (s >= 1.0) continue;
				++r.count;
				// Away from the neighbour along the ellipsoid's gradient; straight up when on top of it.
				Enu g{-d.x / R2, -d.y / R2, -d.z / H2};
				const double gl = std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
				if (gl < 1e-9) g = {0.0, 0.0, 1.0};
				else g = {g.x / gl, g.y / gl, g.z / gl};
				const double m = s < 1e-6 ? c.max : std::min(c.gain * (1.0 / s - 1.0), c.max);
				r.force.x += m * g.x;
				r.force.y += m * g.y;
				r.force.z += m * g.z;
			}
			return r;
		}
	}
}
