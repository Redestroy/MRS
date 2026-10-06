#include "mrs/world/ProcessorCatalog.h"

#include <algorithm>

namespace MRS {
	namespace Environment {
		const ProcessorCatalog& ProcessorCatalog::Default() {
			static const ProcessorCatalog catalog({
				{"Clock", {}, {}, {"time"}, {}},
				{"GnssProcessor", {"V_GEO"}, {}, {"geo.position"}, {}},
				{"GeoToLocalProcessor", {}, {"geo.position"}, {"pose.enu"}, {}},
				{"LocalPositionProcessor", {"V_POS3"}, {}, {"pose.enu"}, {}},
				{"AltitudeProcessor", {"V_BARO", "V_GEO", "V_RNG"}, {}, {"alt.amsl", "alt.agl"}, {}},
				{"AttitudeProcessor", {"V_ATT"}, {}, {"att", "heading"}, {{"V_RATE", "rate.body"}}},
				{"KinematicsEstimator", {}, {"pose.enu"}, {"vel.enu", "acc.enu"}, {}},
				{"BatteryProcessor", {"V_BAT"}, {}, {"battery", "battery.low", "battery.critical"}, {}},
				{"FlightStateProcessor", {}, {"alt.agl", "vel.enu"}, {"airborne", "landed", "armed", "home"}, {}},
				{"SafetySupervisor", {}, {"pose.enu"}, {"geofence.inside"}, {}},
				{"PeerStateProcessor", {"V_PEER"}, {}, {"peer"}, {}},
			});
			return catalog;
		}

		CatalogResolution ProcessorCatalog::Resolve(const std::set<std::string>& views) const {
			CatalogResolution r;
			std::vector<bool> done(entries_.size(), false);
			bool changed = true;
			while (changed) {
				changed = false;
				for (std::size_t k = 0; k < entries_.size(); ++k) {
					if (done[k]) continue;
					const auto& d = entries_[k];
					const bool subscribed = d.subscriptions.empty() ||
					                        std::any_of(d.subscriptions.begin(), d.subscriptions.end(),
					                                    [&](const std::string& v) { return views.count(v) != 0; });
					const bool needs_met = std::all_of(d.needs.begin(), d.needs.end(),
					                                   [&](const std::string& f) { return r.fields.count(f) != 0; });
					if (!subscribed || !needs_met) continue;
					done[k] = true;
					const bool duplicate = std::any_of(d.provides.begin(), d.provides.end(),
					                                   [&](const std::string& f) { return r.fields.count(f) != 0; });
					if (duplicate) continue;  // another processor already provides it
					r.active.push_back(d.name);
					r.fields.insert(d.provides.begin(), d.provides.end());
					for (const auto& opt : d.optional)
						if (views.count(opt.first)) r.fields.insert(opt.second);
					changed = true;
				}
			}
			return r;
		}
	}
}
