#include "mrs/world/Worldview.h"

namespace MRS {
	namespace Environment {
		Worldview::Worldview() {
			// Spec 05 §4.1, minimum UAV field set.
			max_age_ = {{"geo.position", 1.0}, {"pose.enu", 0.5}, {"alt.agl", 0.5}, {"alt.amsl", 0.5}, {"att", 0.2},
			            {"heading", 0.2}, {"vel.enu", 0.5}, {"acc.enu", 0.5}, {"rate.body", 0.2}, {"battery", 2.0}};
		}

		void Worldview::SetScalar(const std::string& path, double value, double stamp) { fields_[path] = {value, stamp, true}; }
		void Worldview::SetBool(const std::string& path, bool value, double stamp) { fields_[path] = {value, stamp, true}; }
		void Worldview::SetId(const std::string& path, const std::string& value, double stamp) { fields_[path] = {value, stamp, true}; }

		void Worldview::SetVec3(const std::string& path, double x, double y, double z, double stamp) {
			SetScalar(path + ".x", x, stamp);
			SetScalar(path + ".y", y, stamp);
			SetScalar(path + ".z", z, stamp);
		}

		void Worldview::Invalidate(const std::string& path, double stamp) {
			auto it = fields_.find(path);
			if (it != fields_.end()) {
				it->second.valid = false;
				it->second.stamp = stamp;
				return;
			}
			bool any = false;
			for (auto& [p, entry] : fields_)
				if (p.size() > path.size() && p.compare(0, path.size(), path) == 0 && p[path.size()] == '.') {
					entry.valid = false;
					entry.stamp = stamp;
					any = true;
				}
			if (!any) fields_[path] = {0.0, stamp, false};
		}

		void Worldview::Erase(const std::string& path) { fields_.erase(path); }

		void Worldview::SetMaxAge(const std::string& path, double max_age) { max_age_[path] = max_age; }

		double Worldview::MaxAge(const std::string& path) const {
			std::string p = path;
			while (true) {
				auto it = max_age_.find(p);
				if (it != max_age_.end()) return it->second;
				auto dot = p.rfind('.');
				if (dot == std::string::npos) return kNoMaxAge;
				p.resize(dot);
			}
		}

		const WorldFieldEntry* Worldview::Fresh(const std::string& path, double t) const {
			auto it = fields_.find(path);
			if (it == fields_.end() || !it->second.valid) return nullptr;
			if (t - it->second.stamp > MaxAge(path)) return nullptr;
			return &it->second;
		}

		std::optional<double> Worldview::Scalar(const std::string& path, double t) const {
			const auto* e = Fresh(path, t);
			if (!e || !std::holds_alternative<double>(e->value)) return std::nullopt;
			return std::get<double>(e->value);
		}

		std::optional<bool> Worldview::Bool(const std::string& path, double t) const {
			const auto* e = Fresh(path, t);
			if (!e || !std::holds_alternative<bool>(e->value)) return std::nullopt;
			return std::get<bool>(e->value);
		}

		std::optional<std::string> Worldview::Id(const std::string& path, double t) const {
			const auto* e = Fresh(path, t);
			if (!e || !std::holds_alternative<std::string>(e->value)) return std::nullopt;
			return std::get<std::string>(e->value);
		}

		bool Worldview::IsFresh(const std::string& path, double t, double max_age) const {
			auto fresh = [&](const std::string& p, const WorldFieldEntry& e) {
				const double age = max_age < 0 ? MaxAge(p) : max_age;
				return e.valid && t - e.stamp <= age;
			};
			auto it = fields_.find(path);
			if (it != fields_.end()) return fresh(path, it->second);
			bool any = false;
			for (auto c = fields_.lower_bound(path + "."); c != fields_.end(); ++c) {
				if (c->first.compare(0, path.size() + 1, path + ".") != 0) break;
				if (!fresh(c->first, c->second)) return false;
				any = true;
			}
			return any;
		}

		bool Worldview::Has(const std::string& path) const {
			if (fields_.count(path)) return true;
			auto c = fields_.lower_bound(path + ".");
			return c != fields_.end() && c->first.compare(0, path.size() + 1, path + ".") == 0;
		}

		std::optional<WorldFieldEntry> Worldview::Raw(const std::string& path) const {
			auto it = fields_.find(path);
			if (it == fields_.end()) return std::nullopt;
			return it->second;
		}

		void Worldview::Restore(const std::string& path, const std::optional<WorldFieldEntry>& entry) {
			if (entry) fields_[path] = *entry;
			else fields_.erase(path);
		}

		std::vector<std::string> Worldview::PathsWithPrefix(const std::string& prefix) const {
			std::vector<std::string> out;
			for (auto c = fields_.lower_bound(prefix); c != fields_.end() && c->first.compare(0, prefix.size(), prefix) == 0; ++c)
				out.push_back(c->first);
			return out;
		}
	}
}
