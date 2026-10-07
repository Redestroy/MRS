#include "mrs/robot/Resources.h"

#include <algorithm>
#include <cmath>

namespace MRS {
	namespace Robot {
		namespace {
			constexpr double kG = 9.81;

			void Walk(const Protocol::Record& r, std::vector<std::array<double, 3>>& targets, double& hold) {
				if (r.code == "C_P3" && r.fields.size() >= 3) targets.push_back({r.fields[0].n, r.fields[1].n, r.fields[2].n});
				if ((r.code == "A_W" || r.code == "A_HD") && !r.fields.empty()) hold += std::max(0.0, r.fields[0].n);
				for (const auto& c : r.children) Walk(c, targets, hold);
			}

			std::optional<std::array<double, 3>> Vec(const Environment::Worldview& w, const std::string& base, double t, bool fresh) {
				std::array<double, 3> v{};
				const char* axes[3] = {".x", ".y", ".z"};
				for (int i = 0; i < 3; ++i) {
					if (fresh) {
						auto x = w.Scalar(base + axes[i], t);
						if (!x) return std::nullopt;
						v[i] = *x;
					} else {
						auto e = w.Raw(base + axes[i]);
						if (!e || !std::holds_alternative<double>(e->value)) return std::nullopt;
						v[i] = std::get<double>(e->value);
					}
				}
				return v;
			}
		}

		double IEnergyModel::Energy(const std::vector<FlightSegment>& plan) const {
			double wh = 0.0;
			for (const auto& s : plan) wh += Power(s) * s.duration / 3600.0;
			return wh;
		}

		QuadEnergyModel::QuadEnergyModel(QuadEnergyConfig config) : c_(config) {
			theta_ = {c_.p_hover, c_.p_hover * c_.c_v, c_.mass * kG / c_.efficiency};
			p_ = {1.0e4, 0, 0, 0, 1.0, 0, 0, 0, 1.0e2};
		}

		double QuadEnergyModel::Power(const FlightSegment& s) const {
			return theta_[0] + theta_[1] * s.speed * s.speed + theta_[2] * std::max(s.climb, 0.0);
		}

		void QuadEnergyModel::Observe(double power_w, double speed, double climb) {
			const std::array<double, 3> x = {1.0, speed * speed, std::max(climb, 0.0)};
			std::array<double, 3> px{};  // P x
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c) px[r] += p_[r * 3 + c] * x[c];
			double denom = c_.forgetting;
			for (int r = 0; r < 3; ++r) denom += x[r] * px[r];
			double err = power_w;
			for (int r = 0; r < 3; ++r) err -= theta_[r] * x[r];
			for (int r = 0; r < 3; ++r) theta_[r] += px[r] / denom * err;
			// P = (P - P x x^T P / denom) / lambda; x^T P = (P x)^T because P is symmetric.
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c) p_[r * 3 + c] = (p_[r * 3 + c] - px[r] * px[c] / denom) / c_.forgetting;
			theta_[1] = std::max(theta_[1], 0.0);
			theta_[2] = std::max(theta_[2], 0.0);
			++samples_;
		}

		ResourceManager::ResourceManager(ResourceConfig config, std::unique_ptr<IEnergyModel> model)
		    : c_(config), model_(model ? std::move(model) : std::make_unique<QuadEnergyModel>()) {}

		void ResourceManager::Update(const Environment::Worldview& w, double t) {
			const auto energy = w.Scalar("battery.energy_wh", t);
			const auto entry = w.Raw("battery.energy_wh");
			if (!energy || !entry) return;
			remaining_ = *energy;
			const auto vx = w.Scalar("vel.enu.x", t), vy = w.Scalar("vel.enu.y", t), vz = w.Scalar("vel.enu.z", t);
			const bool airborne = w.Bool("airborne", t).value_or(false);
			if (vx && vy && vz && airborne) {
				speed_sum_ += std::hypot(*vx, *vy);
				climb_sum_ += *vz;
				++window_n_;
			} else {
				window_ok_ = false;  // the window is not all flight: do not use it
			}
			if (entry->stamp == window_stamp_) return;  // no new battery reading
			if (window_start_ < 0.0) {
				window_start_ = window_stamp_ = entry->stamp;
				window_energy_ = *energy;
				speed_sum_ = climb_sum_ = 0.0;
				window_n_ = 0;
				window_ok_ = true;
				return;
			}
			window_stamp_ = entry->stamp;
			const double dt = entry->stamp - window_start_;
			if (dt < c_.sample_period - 1e-6) return;
			const double power = (window_energy_ - *energy) * 3600.0 / dt;
			if (window_ok_ && window_n_ > 0) {
				const FlightSegment mean{dt, speed_sum_ / window_n_, climb_sum_ / window_n_};
				// A jump in the reading (a swap, a recalibrated gauge) is not flight power.
				const double expected = model_->Power(mean);
				if (power > 0.25 * expected && power < 4.0 * expected) model_->Observe(power, mean.speed, mean.climb);
			}
			window_start_ = entry->stamp;
			window_energy_ = *energy;
			speed_sum_ = climb_sum_ = 0.0;
			window_n_ = 0;
			window_ok_ = true;
		}

		std::vector<FlightSegment> ResourceManager::Path(const std::array<double, 3>& a, const std::array<double, 3>& b) const {
			std::vector<FlightSegment> plan;
			const double dz = b[2] - a[2];
			if (dz > 0.0) plan.push_back({dz / c_.climb_rate, 0.0, c_.climb_rate});
			else if (dz < 0.0) plan.push_back({-dz / c_.climb_rate, 0.0, -c_.climb_rate});
			const double d = std::hypot(b[0] - a[0], b[1] - a[1]);
			if (d > 0.0) plan.push_back({d / c_.cruise_speed, c_.cruise_speed, 0.0});
			return plan;
		}

		double ResourceManager::PathCost(const std::array<double, 3>& a, const std::array<double, 3>& b) const {
			return model_->Energy(Path(a, b));
		}

		std::vector<std::array<double, 3>> ResourceManager::Targets(const Task::Task& task) {
			std::vector<std::array<double, 3>> targets;
			double hold = 0.0;
			Walk(task.ToRecord(), targets, hold);
			return targets;
		}

		double ResourceManager::HoldTime(const Task::Task& task) {
			std::vector<std::array<double, 3>> targets;
			double hold = 0.0;
			Walk(task.ToRecord(), targets, hold);
			return hold;
		}

		std::optional<double> ResourceManager::HomeCost(const Environment::Worldview& w, double t) const {
			const auto here = Vec(w, "pose.enu", t, true);
			const auto home = Vec(w, "home.enu", t, false);
			if (!here || !home) return std::nullopt;
			const std::array<double, 3> above_home = {(*home)[0], (*home)[1], (*here)[2]};
			const double descent = std::max(0.0, (*here)[2] - (*home)[2]) / c_.descent_rate;
			return PathCost(*here, above_home) + model_->Energy({{descent, 0.0, -c_.descent_rate}});
		}

		std::optional<double> ResourceManager::TaskCost(const Task::Task& task, const Environment::Worldview& w, double t) const {
			auto here = Vec(w, "pose.enu", t, true);
			const auto home = Vec(w, "home.enu", t, false);
			if (!here || !home) return std::nullopt;
			double wh = 0.0;
			std::array<double, 3> at = *here;
			for (const auto& target : Targets(task)) {
				wh += PathCost(at, target);
				at = target;
			}
			wh += model_->Energy({{HoldTime(task), 0.0, 0.0}});
			const std::array<double, 3> above_home = {(*home)[0], (*home)[1], at[2]};
			wh += PathCost(at, above_home);
			wh += model_->Energy({{std::max(0.0, at[2] - (*home)[2]) / c_.descent_rate, 0.0, -c_.descent_rate}});
			return wh;
		}

		bool ResourceManager::Feasible(const Task::Task& task, const Environment::Worldview& w, double t) const {
			const auto cost = TaskCost(task, w, t);
			const auto remaining = w.Scalar("battery.energy_wh", t);  // this tick's reading
			if (!cost || !remaining) return true;
			return *cost + Reserve() <= *remaining;
		}
	}
}
