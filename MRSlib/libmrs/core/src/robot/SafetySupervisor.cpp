#include "mrs/robot/SafetySupervisor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MRS {
	namespace Robot {
		bool Fence::Inside(const std::array<double, 3>& p, double m) const {
			return p[0] >= xmin + m && p[0] <= xmax - m && p[1] >= ymin + m && p[1] <= ymax - m && p[2] >= 0.0 && p[2] <= zmax - m;
		}

		std::optional<Fence> FenceOf(const Environment::Worldview& w) {
			double v[5];
			const char* keys[5] = {"geofence.xmin", "geofence.xmax", "geofence.ymin", "geofence.ymax", "geofence.zmax"};
			for (int k = 0; k < 5; ++k) {
				auto e = w.Raw(keys[k]);
				if (!e || !e->valid || !std::holds_alternative<double>(e->value)) return std::nullopt;
				v[k] = std::get<double>(e->value);
			}
			return Fence{v[0], v[1], v[2], v[3], v[4]};
		}

		namespace {
			std::optional<double> RawScalar(const Environment::Worldview& w, const char* path) {
				auto e = w.Raw(path);
				if (!e || !e->valid || !std::holds_alternative<double>(e->value)) return std::nullopt;
				return std::get<double>(e->value);
			}
		}

		SafetySupervisor::SafetySupervisor(Task::IActionSink& inner, const Environment::Worldview& world, SafetyConfig config)
		    : inner_(inner), w_(world), c_(config) {}

		Device::ActionStatus SafetySupervisor::Dispatch(const Device::ActionMap& action, double t) {
			Device::ActionMap filtered = Filter(action, t);
			bool changed = false;
			for (std::size_t i = 0; i < action.entries.size(); ++i)
				if (filtered.entries[i].action.arg != action.entries[i].action.arg) changed = true;
			if (changed) ++changed_;
			return inner_.Dispatch(filtered, t);
		}

		Device::ActionMap SafetySupervisor::Filter(const Device::ActionMap& action, double t) const {
			Device::ActionMap out = action;
			const auto fence = FenceOf(w_);
			const double ground = RawScalar(w_, "home.enu.z").value_or(0.0);
			const bool airborne = w_.Bool("airborne", t).value_or(false);
			const auto x = w_.Scalar("pose.enu.x", t), y = w_.Scalar("pose.enu.y", t);
			const double m = c_.fence_margin;
			std::optional<Environment::Enu> push;
			if (c_.repulsion_lead > 0.0 && airborne) {
				const auto px = w_.Scalar("repulse.enu.x", t), py = w_.Scalar("repulse.enu.y", t), pz = w_.Scalar("repulse.enu.z", t);
				if (px && py && pz && (*px != 0.0 || *py != 0.0 || *pz != 0.0)) push = Environment::Enu{*px, *py, *pz};
			}
			for (auto& e : out.entries) {
				const std::string& code = e.action.code;
				if (code != "A_PXY" && code != "A_PZY" && code != "A_TO" && code != "A_VXY" && code != "A_VZY") continue;
				auto v = Device::UnpackReals(Device::ArgLayout::F32X2, e.action.arg);
				const auto before = v;
				if (push) {  // away from nearby robots, before the limits (spec 15 §4.3)
					if (code == "A_PXY") {
						v[0] += c_.repulsion_lead * push->x;
						v[1] += c_.repulsion_lead * push->y;
					} else if (code == "A_PZY") {
						v[0] += c_.repulsion_lead * push->z;
					} else if (code == "A_VXY") {
						v[0] += push->x;
						v[1] += push->y;
					} else if (code == "A_VZY") {
						v[0] += push->z;
					}
				}
				if (code == "A_PXY" && fence) {
					v[0] = std::clamp(v[0], fence->xmin + m, std::max(fence->xmin + m, fence->xmax - m));
					v[1] = std::clamp(v[1], fence->ymin + m, std::max(fence->ymin + m, fence->ymax - m));
				} else if (code == "A_PZY") {
					const double hi = fence ? fence->zmax - m : std::numeric_limits<double>::infinity();
					const double lo = airborne ? ground + c_.min_alt : -std::numeric_limits<double>::infinity();
					v[0] = std::clamp(v[0], lo, std::max(lo, hi));
				} else if (code == "A_TO" && fence) {
					v[0] = std::min(v[0], fence->zmax - m);
				} else if (code == "A_VXY") {
					const double speed = std::hypot(v[0], v[1]);
					if (speed > c_.max_speed_xy) {
						v[0] *= c_.max_speed_xy / speed;
						v[1] *= c_.max_speed_xy / speed;
					}
					if (fence && x && y) {
						if ((v[0] > 0 && *x > fence->xmax - m) || (v[0] < 0 && *x < fence->xmin + m)) v[0] = 0;
						if ((v[1] > 0 && *y > fence->ymax - m) || (v[1] < 0 && *y < fence->ymin + m)) v[1] = 0;
					}
				} else if (code == "A_VZY") {
					v[0] = std::clamp(v[0], -c_.max_climb, c_.max_climb);
				}
				if (v != before) e.action.arg = *Device::PackArgument(Device::ArgLayout::F32X2, v);
			}
			return out;
		}

		bool SafetySupervisor::TargetAllowed(const std::array<double, 3>& p) const {
			const auto fence = FenceOf(w_);
			if (!fence) return true;
			const double ground = RawScalar(w_, "home.enu.z").value_or(0.0);
			return fence->Inside(p, c_.fence_margin) && p[2] >= ground + c_.min_alt - 1e-9;
		}
	}
}
