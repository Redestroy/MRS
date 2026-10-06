#include "mrs/world/Worldview.h"

#include <algorithm>

namespace MRS {
	namespace Environment {
		Worldview::Worldview() {
			// Spec 05 §4.1, minimum UAV field set.
			max_age_ = {{"geo.position", 1.0}, {"pose.enu", 0.5}, {"alt.agl", 0.5}, {"alt.amsl", 0.5}, {"att", 0.2},
			            {"heading", 0.2}, {"vel.enu", 0.5}, {"acc.enu", 0.5}, {"rate.body", 0.2}, {"battery", 2.0}};
		}

		void Worldview::SetScalar(const std::string& path, double value, double stamp, const std::string& source) {
			fields_[path] = {value, stamp, true, source};
			auto it = history_.find(path);
			if (it == history_.end()) it = history_.emplace(path, TimeSeries(history_capacity_)).first;
			it->second.Add(stamp, value);
		}
		void Worldview::SetBool(const std::string& path, bool value, double stamp, const std::string& source) {
			fields_[path] = {value, stamp, true, source};
		}
		void Worldview::SetId(const std::string& path, const std::string& value, double stamp, const std::string& source) {
			fields_[path] = {value, stamp, true, source};
		}

		void Worldview::SetVec3(const std::string& path, double x, double y, double z, double stamp, const std::string& source) {
			SetScalar(path + ".x", x, stamp, source);
			SetScalar(path + ".y", y, stamp, source);
			SetScalar(path + ".z", z, stamp, source);
		}

		void Worldview::SetComponents(const std::string& path, const std::vector<std::string>& names, const std::vector<double>& values,
		                              double stamp, const std::string& source) {
			if (names.empty()) {
				if (!values.empty()) SetScalar(path, values[0], stamp, source);
				return;
			}
			for (std::size_t k = 0; k < names.size() && k < values.size(); ++k) SetScalar(path + "." + names[k], values[k], stamp, source);
		}

		void Worldview::Offer(const std::string& path, const std::string& source, const std::vector<std::string>& names,
		                      const std::vector<double>& values, double stamp) {
			auto& c = offers_[path][source];
			if (c.values.size() && stamp < c.stamp) return;  // an older value than the source's latest
			c = {names, values, stamp};
			Select(path, stamp);
		}

		void Worldview::OfferScalar(const std::string& path, const std::string& source, double value, double stamp) {
			Offer(path, source, {}, {value}, stamp);
		}

		void Worldview::OfferVec3(const std::string& path, const std::string& source, double x, double y, double z, double stamp) {
			Offer(path, source, {"x", "y", "z"}, {x, y, z}, stamp);
		}

		void Worldview::SetSourceOrder(const std::string& path, std::vector<std::string> sources) {
			source_order_[path] = std::move(sources);
		}

		const std::vector<std::string>& Worldview::SourceOrder(const std::string& path) const {
			static const std::vector<std::string> none;
			auto it = source_order_.find(path);
			return it == source_order_.end() ? none : it->second;
		}

		void Worldview::Select(const std::string& path, double t) {
			auto it = offers_.find(path);
			if (it == offers_.end() || it->second.empty()) return;
			const auto& candidates = it->second;
			// Rank: the source order first, then the other sources by name (std::map order).
			std::vector<const std::string*> ranked;
			for (const auto& name : SourceOrder(path))
				if (candidates.count(name)) ranked.push_back(&candidates.find(name)->first);
			for (const auto& c : candidates)
				if (std::find_if(ranked.begin(), ranked.end(), [&](const std::string* r) { return *r == c.first; }) == ranked.end())
					ranked.push_back(&c.first);

			const double max_age = MaxAge(path);
			const std::string* chosen = nullptr;
			for (const std::string* name : ranked)
				if (t - candidates.at(*name).stamp <= max_age) {
					chosen = name;
					break;
				}
			if (!chosen)  // nothing fresh: keep the newest, which reads as stale
				for (const std::string* name : ranked)
					if (!chosen || candidates.at(*name).stamp > candidates.at(*chosen).stamp) chosen = name;

			const Candidate& c = candidates.at(*chosen);
			const auto current = fields_.find(c.names.empty() ? path : path + "." + c.names[0]);
			const bool same = current != fields_.end() && selected_[path] == *chosen && current->second.stamp == c.stamp;
			selected_[path] = *chosen;
			if (!same) SetComponents(path, c.names, c.values, c.stamp, *chosen);
		}

		void Worldview::RefreshSources(double t) {
			for (const auto& entry : offers_) Select(entry.first, t);
		}

		std::string Worldview::SelectedSource(const std::string& path) const {
			auto it = selected_.find(path);
			return it == selected_.end() ? std::string() : it->second;
		}

		std::vector<std::string> Worldview::OfferedSources(const std::string& path) const {
			std::vector<std::string> out;
			auto it = offers_.find(path);
			if (it != offers_.end())
				for (const auto& c : it->second) out.push_back(c.first);
			return out;
		}

		const TimeSeries* Worldview::History(const std::string& path) const {
			auto it = history_.find(path);
			return it == history_.end() ? nullptr : &it->second;
		}

		void Worldview::PutObject(SemanticObject object) {
			const std::string id = object.id;
			objects_[id] = std::move(object);
		}

		const SemanticObject* Worldview::Object(const std::string& id) const {
			auto it = objects_.find(id);
			return it == objects_.end() ? nullptr : &it->second;
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
			if (!any) fields_[path] = {0.0, stamp, false, {}};
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
