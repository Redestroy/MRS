#include "mrs/task/TaskAction.h"

#include <cmath>
#include <limits>

namespace MRS {
	namespace Task {
		TaskAction TaskAction::Leaf(Device::Action action) {
			TaskAction a;
			a.code_ = action.code;
			a.action_ = std::move(action);
			return a;
		}

		TaskAction TaskAction::Map(std::vector<std::pair<std::string, TaskAction>> entries) {
			TaskAction a;
			a.code_ = "A_MAP";
			a.entries_ = std::move(entries);
			return a;
		}

		TaskAction TaskAction::Parametric(std::string code, std::vector<std::shared_ptr<const Function>> functions) {
			TaskAction a;
			a.code_ = "A_FN";
			a.produced_ = std::move(code);
			a.functions_ = std::move(functions);
			return a;
		}

		bool TaskAction::ContainsParametric() const {
			if (IsParametric()) return true;
			for (const auto& e : entries_)
				if (e.second.ContainsParametric()) return true;
			return false;
		}

		double TaskAction::WaitSeconds() const { return Device::UnpackReals(Device::ArgLayout::F64, action_.arg).at(0); }

		std::optional<Device::ActionMap> TaskAction::Resolve(const Environment::Worldview& w, double t) const {
			Device::ActionMap out;
			if (IsMap()) {
				out.combined = true;
				for (const auto& [target, entry] : entries_) {
					auto resolved = entry.Resolve(w, t);
					if (!resolved) return std::nullopt;
					out.entries.push_back({target, resolved->entries.at(0).action});
				}
				return out;
			}
			if (!IsParametric()) {
				out.entries.push_back({"any", action_});
				return out;
			}
			const auto layout = Device::FindAction(produced_)->layout;
			std::vector<double> values;
			for (const auto& f : functions_) {
				auto v = f->Evaluate(w, t);
				if (!v) return std::nullopt;
				values.push_back(*v);
			}
			std::optional<std::uint64_t> arg;
			if (Device::IsIntegerLayout(layout)) {
				// Round half away from zero, then saturate to the layout's range (spec 02 §6).
				const double lo = layout == Device::ArgLayout::U32X2 ? 0.0
				                  : layout == Device::ArgLayout::I32X2 ? static_cast<double>(std::numeric_limits<std::int32_t>::min())
				                                                         : -9.2233720368547748e18;
				const double hi = layout == Device::ArgLayout::U32X2 ? static_cast<double>(std::numeric_limits<std::uint32_t>::max())
				                  : layout == Device::ArgLayout::I32X2 ? static_cast<double>(std::numeric_limits<std::int32_t>::max())
				                                                         : 9.2233720368547748e18;
				std::vector<std::int64_t> ints;
				for (double v : values) {
					double r = std::round(v);
					if (r < lo) r = lo;
					if (r > hi) r = hi;
					ints.push_back(r >= 9.2233720368547748e18 ? std::numeric_limits<std::int64_t>::max() : static_cast<std::int64_t>(r));
				}
				arg = Device::PackArgument(layout, ints);
			} else {
				arg = Device::PackArgument(layout, values);
			}
			if (!arg) return std::nullopt;
			out.entries.push_back({"any", Device::Action{produced_, *arg}});
			return out;
		}

		void TaskAction::Fields(std::vector<std::string>& out) const {
			for (const auto& f : functions_) f->Fields(out);
			for (const auto& e : entries_) e.second.Fields(out);
		}

		void TaskAction::ActionCodes(std::vector<std::string>& out) const {
			if (IsMap()) {
				for (const auto& e : entries_) e.second.ActionCodes(out);
			} else if (IsParametric()) {
				out.push_back(produced_);
			} else if (code_ != "A_N" && code_ != "A_W" && code_ != "A_I") {
				out.push_back(code_);
			}
		}
	}
}
