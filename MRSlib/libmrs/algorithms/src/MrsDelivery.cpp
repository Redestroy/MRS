// Intermediate drops (spec 18 §7): when a robot hooks a package it cannot carry to the
// destination and then home on the battery it has, it leaves the package in the reachable drop
// area closest to the destination and posts a new delivery from there for any robot.
#include <algorithm>
#include <cmath>
#include <string>

#include "mrs/algorithms/MrsLayer.h"
#include "mrs/generators/Generators.h"

namespace MRS {
	namespace Algorithms {
		namespace {
			std::optional<std::array<double, 3>> Vec(const Environment::Worldview& w, const std::string& p, double t) {
				const auto x = w.Scalar(p + ".x", t), y = w.Scalar(p + ".y", t), z = w.Scalar(p + ".z", t);
				if (!x || !y || !z) return std::nullopt;
				return std::array<double, 3>{*x, *y, *z};
			}
			double Horizontal(const std::array<double, 3>& a, const std::array<double, 3>& b) { return std::hypot(a[0] - b[0], a[1] - b[1]); }
		}

		void MrsLayer::UpdateDelivery(double t) {
			auto& w = robot_.World();
			const auto id = w.Scalar("payload.id", t);
			const std::int64_t held = id ? static_cast<std::int64_t>(std::llround(*id)) : 0;
			if (held != held_) held_since_ = t;
			if (held == 0) planned_ = 0;
			else if (held != planned_ && !c_.drop_areas.empty() && PlanLeg(held, t)) planned_ = held;
			// Released at the leg point: the package waits there for the next robot.
			if (held == 0 && held_ != 0 && held_ == leg_package_ && leg_) {
				Generators::DeliveryOptions o;
				o.sling_length = c_.sling_length;
				const std::string task_id = self_ + "." + std::to_string(own_tasks_ + 1);
				if (!leg_abandon_) ++own_tasks_;
				// The approach clears a small stack: other legs may have left packages there too.
				if (leg_abandon_) ++stranded_;
				else PostTask(Generators::DeliveryTask(task_id, {(*leg_)[0], (*leg_)[1], (*leg_)[2]}, 1.0, o), t);
				leg_package_ = 0;
				leg_.reset();
			}
			held_ = held;
			if (held != 0 && held == leg_package_ && leg_) w.SetVec3("payload.leg", (*leg_)[0], (*leg_)[1], (*leg_)[2], t, "legs");
			else w.Invalidate("payload.leg", t);
		}

		bool MrsLayer::PlanLeg(std::int64_t package, double t) {
			auto& w = robot_.World();
			auto& res = robot_.Resources();
			const auto here = Vec(w, "pose.enu", t), dest = Vec(w, "payload.dest", t);
			const auto remaining = res.Remaining();
			leg_package_ = 0;
			leg_.reset();
			// The destination must come from this package's tag, read after it was hooked.
			const auto tag = w.Raw("payload.dest.x");
			if (!here || !dest || !remaining || !tag || tag->stamp < held_since_ - 0.5) return false;
			std::array<double, 3> home_at = {(*here)[0], (*here)[1], 0.0};
			if (auto hx = w.Raw("home.enu.x"), hy = w.Raw("home.enu.y"); hx && hy)
				home_at = {std::get<double>(hx->value), std::get<double>(hy->value), 0.0};
			// Carried mass raises hover power by about ((M + m) / M)^1.5, and the package flight is
			// slower than the empty cruise, so a loaded metre costs more than twice an empty one.
			const double mass = w.Scalar("payload.mass", t).value_or(0.0);
			const double loaded = std::pow(1.0 + mass / c_.frame_mass, 1.5);
			const double cruise = w.Scalar("layer.alt", t).value_or((*here)[2]);
			const double hover = res.Model().Energy({{c_.package_hover_s, 0.0, 0.0}}) * loaded;
			auto loaded_cruise = [&](const std::array<double, 3>& a, const std::array<double, 3>& b) {
				const double d = Horizontal(a, b);
				return res.Model().Energy({{d / c_.loaded_speed, c_.loaded_speed, 0.0}}) * loaded;
			};
			auto cost_via = [&](const std::array<double, 3>& drop) {
				const std::array<double, 3> up{(*here)[0], (*here)[1], cruise}, over{drop[0], drop[1], cruise};
				const std::array<double, 3> home_up{home_at[0], home_at[1], cruise};
				const double out = (res.PathCost(*here, up) + res.PathCost(over, drop)) * loaded + loaded_cruise(up, over) + hover;
				const double back = res.PathCost(drop, over) + res.PathCost(over, home_up) + res.PathCost(home_up, home_at);
				return c_.leg_margin * (out + back);
			};
			const double budget = *remaining - res.Reserve();
			if (cost_via(*dest) <= budget) return true;  // it can go all the way
			// The point of each drop area nearest the destination, 1 m inside its edge.
			std::optional<std::array<double, 3>> best;
			// An area must bring the package clearly closer, not just to the other side of the
			// area it is already in.
			double best_left = Horizontal(*here, *dest) - c_.leg_min_progress;
			for (const auto& a : c_.drop_areas) {
				std::array<double, 3> p{a.x, a.y, 0.0};
				const double d = std::hypot((*dest)[0] - a.x, (*dest)[1] - a.y), r = std::max(0.0, a.radius - 1.0);
				if (d > 1e-6) {
					const double s = std::min(r, d) / d;
					p = {a.x + ((*dest)[0] - a.x) * s, a.y + ((*dest)[1] - a.y) * s, 0.0};
				}
				const double left = Horizontal(p, *dest);
				if (left >= best_left || cost_via(p) > budget) continue;
				best_left = left;
				best = p;
			}
			leg_abandon_ = false;
			if (!best) {
				// Nowhere closer it can reach: put the package back down here. A robot that set out
				// from home fully charged would have `fresh` left here; if this one has about that
				// much, no robot does better, so the package stays and no new delivery is posted.
				const std::array<double, 3> here_up{(*here)[0], (*here)[1], cruise}, home_up{home_at[0], home_at[1], cruise};
				const double fresh = res.Capacity() - (res.PathCost(home_at, home_up) + res.PathCost(home_up, here_up) + res.PathCost(here_up, *here)) -
				                     res.Model().Energy({{c_.package_hover_s, 0.0, 0.0}});
				leg_abandon_ = *remaining >= fresh - 0.1 * res.Capacity();
				best = std::array<double, 3>{(*here)[0], (*here)[1], 0.0};
			}
			leg_package_ = package;
			leg_ = best;
			++legs_planned_;
			return true;
		}

		void MrsLayer::PostTask(const Protocol::Record& task, double t) {
			auto r = messenger_.Begin("M_TASK", "all", t);
			r.fields.push_back(Protocol::Field::MakeRef(r.children.size()));
			r.children.push_back(task);
			messenger_.Post(r);
			AddTaskRecord(task, t);
		}
	}
}
