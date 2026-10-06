#include "mrs/task/Function.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MRS {
	namespace Task {
		void FunctionRegistry::Register(std::string key, RegistryFunction f, std::size_t n_coefficients,
		                                std::vector<std::string> fields_read) {
			entries_[std::move(key)] = Entry{std::move(f), n_coefficients, std::move(fields_read)};
		}

		bool FunctionRegistry::Has(const std::string& key) const { return entries_.count(key) > 0; }

		std::size_t FunctionRegistry::CoefficientCount(const std::string& key) const { return entries_.at(key).n; }

		const std::vector<std::string>& FunctionRegistry::FieldsRead(const std::string& key) const { return entries_.at(key).fields; }

		std::optional<double> FunctionRegistry::Call(const std::string& key, const Environment::Worldview& w, double t,
		                                             const std::vector<double>& k) const {
			auto it = entries_.find(key);
			if (it == entries_.end()) return std::nullopt;
			for (const auto& field : it->second.fields)
				if (!w.IsFresh(field, t)) return std::nullopt;
			const double v = it->second.f(w, t, k);
			if (!std::isfinite(v)) return std::nullopt;
			return v;
		}

		namespace {
			// Reads a field the registry has already checked for freshness.
			double Get(const Environment::Worldview& w, const char* path, double t) { return w.Scalar(path, t).value_or(0.0); }

			double HorizontalDistance(const Environment::Worldview& w, double t) {
				return std::hypot(Get(w, "target.x", t) - Get(w, "pose.enu.x", t), Get(w, "target.y", t) - Get(w, "pose.enu.y", t));
			}

			// layered_x / layered_y (spec 02 §8): go to the target's coordinate once at the
			// layer altitude or near the target; otherwise hold the current one (climb in place).
			RegistryFunction Layered(const char* axis_target, const char* axis_pose) {
				return [axis_target, axis_pose](const Environment::Worldview& w, double t, const std::vector<double>& k) {
					const double alt_tol = k.at(0), switch_radius = k.at(1);
					const bool at_layer = std::fabs(Get(w, "pose.enu.z", t) - Get(w, "layer.alt", t)) <= alt_tol;
					return (at_layer || HorizontalDistance(w, t) <= switch_radius) ? Get(w, axis_target, t) : Get(w, axis_pose, t);
				};
			}

			RegistryFunction VelocityTo(const char* axis_target, const char* axis_pose) {
				return [axis_target, axis_pose](const Environment::Worldview& w, double t, const std::vector<double>& k) {
					const double gain = k.at(0), limit = k.at(1);
					return std::clamp(gain * (Get(w, axis_target, t) - Get(w, axis_pose, t)), -limit, limit);
				};
			}
		}

		void RegisterUavFunctions(FunctionRegistry& r) {
			const std::vector<std::string> pose_target = {"pose.enu", "target.x", "target.y", "target.z"};
			const std::vector<std::string> pose_target_xy = {"pose.enu", "target.x", "target.y"};
			r.Register(
			    "dist_to_target",
			    [](const Environment::Worldview& w, double t, const std::vector<double>&) {
				    const double dz = Get(w, "target.z", t) - Get(w, "pose.enu.z", t);
				    return std::hypot(HorizontalDistance(w, t), dz);
			    },
			    0, pose_target);
			r.Register(
			    "dist_to_target_xy", [](const Environment::Worldview& w, double t, const std::vector<double>&) { return HorizontalDistance(w, t); },
			    0, pose_target_xy);
			r.Register(
			    "yaw_to_target",
			    [](const Environment::Worldview& w, double t, const std::vector<double>&) {
				    if (HorizontalDistance(w, t) < 1.0) return Get(w, "att.yaw", t);
				    return std::atan2(Get(w, "target.y", t) - Get(w, "pose.enu.y", t), Get(w, "target.x", t) - Get(w, "pose.enu.x", t));
			    },
			    0, {"pose.enu", "target.x", "target.y", "att.yaw"});
			r.Register("vel_to_target_x", VelocityTo("target.x", "pose.enu.x"), 2, {"pose.enu", "target.x"});
			r.Register("vel_to_target_y", VelocityTo("target.y", "pose.enu.y"), 2, {"pose.enu", "target.y"});
			r.Register("vel_to_target_z", VelocityTo("target.z", "pose.enu.z"), 2, {"pose.enu", "target.z"});
			r.Register("layered_x", Layered("target.x", "pose.enu.x"), 2, {"pose.enu", "target.x", "target.y", "layer.alt"});
			r.Register("layered_y", Layered("target.y", "pose.enu.y"), 2, {"pose.enu", "target.x", "target.y", "layer.alt"});
			r.Register(
			    "layered_z",
			    [](const Environment::Worldview& w, double t, const std::vector<double>& k) {
				    return HorizontalDistance(w, t) > k.at(0) ? Get(w, "layer.alt", t) : Get(w, "target.z", t);
			    },
			    1, {"pose.enu", "target.x", "target.y", "target.z", "layer.alt"});
		}

		LinearFunction::LinearFunction(std::vector<double> k, double c, std::vector<std::string> fields)
		    : Function("F_L"), k_(std::move(k)), c_(c), fields_(std::move(fields)) {}

		std::optional<double> LinearFunction::Evaluate(const Environment::Worldview& w, double t) const {
			double v = c_;
			for (std::size_t i = 0; i < k_.size(); ++i) {
				auto x = w.Scalar(fields_[i], t);
				if (!x) return std::nullopt;
				v += k_[i] * *x;
			}
			return v;
		}

		void LinearFunction::Fields(std::vector<std::string>& out) const { out.insert(out.end(), fields_.begin(), fields_.end()); }

		RegistryCall::RegistryCall(std::string key, std::vector<double> k, const FunctionRegistry& registry)
		    : Function("F_X"), key_(std::move(key)), k_(std::move(k)), registry_(&registry) {}

		std::optional<double> RegistryCall::Evaluate(const Environment::Worldview& w, double t) const {
			return registry_->Call(key_, w, t, k_);
		}

		void RegistryCall::Fields(std::vector<std::string>& out) const {
			if (registry_->Has(key_)) {
				const auto& f = registry_->FieldsRead(key_);
				out.insert(out.end(), f.begin(), f.end());
			}
		}

		CombinedFunction::CombinedFunction(std::string code, std::vector<FunctionPtr> children)
		    : Function(std::move(code)), children_(std::move(children)) {}

		std::optional<double> CombinedFunction::Evaluate(const Environment::Worldview& w, double t) const {
			const bool sum = Code() == "F_S";
			double v = sum ? 0.0 : 1.0;
			for (const auto& child : children_) {
				auto x = child->Evaluate(w, t);
				if (!x) return std::nullopt;
				v = sum ? v + *x : v * *x;
			}
			return v;
		}

		void CombinedFunction::Fields(std::vector<std::string>& out) const {
			for (const auto& child : children_) child->Fields(out);
		}

		FunctionPtr CombinedFunction::Clone() const {
			std::vector<FunctionPtr> copy;
			for (const auto& child : children_) copy.push_back(child->Clone());
			return std::make_unique<CombinedFunction>(Code(), std::move(copy));
		}

		ClampFunction::ClampFunction(double lo, double hi, FunctionPtr child) : Function("F_C"), lo_(lo), hi_(hi), child_(std::move(child)) {}

		std::optional<double> ClampFunction::Evaluate(const Environment::Worldview& w, double t) const {
			auto x = child_->Evaluate(w, t);
			if (!x) return std::nullopt;
			return std::clamp(*x, lo_, hi_);
		}
	}
}
