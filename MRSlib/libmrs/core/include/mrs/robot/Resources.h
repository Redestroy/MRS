#pragma once
// Resources and energy (spec 08 §6): an energy model with online calibration, and the
// resource manager that estimates what tasks cost.
#include <array>
#include <memory>
#include <optional>
#include <vector>

#include "mrs/task/Task.h"
#include "mrs/world/Worldview.h"

namespace MRS {
	namespace Robot {
		// A duration at a horizontal speed and a climb rate; hover is speed 0, climb 0.
		struct FlightSegment {
			double duration = 0.0;  // s
			double speed = 0.0;     // m/s, horizontal
			double climb = 0.0;     // m/s, positive up
		};

		class IEnergyModel {
		public:
			virtual ~IEnergyModel() = default;
			virtual double Power(const FlightSegment& s) const = 0;  // W
			virtual double Energy(const std::vector<FlightSegment>& plan) const;  // Wh
			// One calibration sample: mean power over a second, with the mean speed and climb.
			virtual void Observe(double power_w, double speed, double climb) { (void)power_w, (void)speed, (void)climb; }
		};

		struct QuadEnergyConfig {
			double p_hover = 90.0;   // W
			double c_v = 0.004;      // s^2/m^2
			double mass = 0.9;       // kg
			double efficiency = 0.6;
			double forgetting = 0.99;
		};

		// P(v, v_z) = P_hover (1 + c_v v^2) + k_climb max(v_z, 0), with k_climb = m g / eta to
		// start. P_hover, c_v and k_climb are calibrated by recursive least squares on the
		// regressors (1, v^2, max(v_z, 0)).
		class QuadEnergyModel : public IEnergyModel {
		public:
			explicit QuadEnergyModel(QuadEnergyConfig config = {});
			double Power(const FlightSegment& s) const override;
			void Observe(double power_w, double speed, double climb) override;

			double HoverPower() const { return theta_[0]; }
			double SpeedCoefficient() const { return theta_[0] != 0.0 ? theta_[1] / theta_[0] : 0.0; }
			double ClimbCoefficient() const { return theta_[2]; }  // W per m/s
			int Samples() const { return samples_; }

		private:
			QuadEnergyConfig c_;
			std::array<double, 3> theta_;  // P_hover, P_hover * c_v, k_climb
			std::array<double, 9> p_;      // covariance, row major
			int samples_ = 0;
		};

		struct ResourceConfig {
			double cruise_speed = 6.0;   // m/s: max_speed_xy * 0.75
			double climb_rate = 2.0;     // m/s: max_climb
			double descent_rate = 0.7;   // m/s: the landing rate
			double reserve_fraction = 0.15;
			double sample_period = 1.0;  // s between calibration samples
		};

		class ResourceManager {
		public:
			explicit ResourceManager(ResourceConfig config = {}, std::unique_ptr<IEnergyModel> model = nullptr);

			// Reads battery.energy_wh. Over windows of at least sample_period between battery
			// readings spent airborne, feeds the model the measured power (the drop in energy)
			// with the mean speed and climb from vel.enu.
			void Update(const Environment::Worldview& w, double t);

			void SetCapacity(double wh) { capacity_ = wh; }
			double Capacity() const { return capacity_; }
			double Reserve() const { return capacity_ * c_.reserve_fraction; }
			std::optional<double> Remaining() const { return remaining_; }

			// Straight flight from a to b: climb (or descend) and cross at cruise speed (Wh).
			std::vector<FlightSegment> Path(const std::array<double, 3>& a, const std::array<double, 3>& b) const;
			double PathCost(const std::array<double, 3>& a, const std::array<double, 3>& b) const;
			// The targets of a task's position conditions (C_P3), in record order.
			static std::vector<std::array<double, 3>> Targets(const Task::Task& task);
			// The hover time a task asks for: its A_W and A_HD durations (s).
			static double HoldTime(const Task::Task& task);
			// cost(here -> task) + cost(task) + cost(task -> home) + landing, in Wh. nullopt without
			// a position or home.
			std::optional<double> TaskCost(const Task::Task& task, const Environment::Worldview& w, double t) const;
			// The cost from here to home, and the landing (Wh).
			std::optional<double> HomeCost(const Environment::Worldview& w, double t) const;
			// TaskCost + reserve <= remaining (spec 08 §6). True when nothing is known.
			bool Feasible(const Task::Task& task, const Environment::Worldview& w, double t) const;

			IEnergyModel& Model() { return *model_; }
			const IEnergyModel& Model() const { return *model_; }

		private:
			ResourceConfig c_;
			std::unique_ptr<IEnergyModel> model_;
			double capacity_ = 0.0;
			std::optional<double> remaining_;
			// Calibration window.
			double window_start_ = -1.0, window_stamp_ = -1.0, window_energy_ = 0.0;
			double speed_sum_ = 0.0, climb_sum_ = 0.0;
			int window_n_ = 0;
			bool window_ok_ = true;
		};
	}
}
