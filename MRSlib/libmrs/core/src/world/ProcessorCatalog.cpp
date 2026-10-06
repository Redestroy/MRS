#include "mrs/world/ProcessorCatalog.h"

#include <algorithm>

#include "mrs/world/Processor.h"
#include "mrs/world/UavProcessors.h"

namespace MRS {
	namespace Environment {
		ProcessorCatalog ProcessorCatalog::FromProcessors(const std::vector<const IViewProcessor*>& processors) {
			ProcessorCatalog catalog;
			for (const IViewProcessor* p : processors) {
				ProcessorDescriptor d{p->Name(), p->Subscriptions(), p->Needs(), p->Provides(), p->Optional(), {}};
				for (const auto& o : p->Offers()) d.offers.emplace_back(o.field, o.source);
				catalog.Add(std::move(d));
			}
			return catalog;
		}

		const ProcessorCatalog& ProcessorCatalog::Default() {
			static const ProcessorCatalog catalog = [] {
				const auto processors = MakeUavProcessors();
				std::vector<const IViewProcessor*> view;
				for (const auto& p : processors) view.push_back(p.get());
				return FromProcessors(view);
			}();
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
					r.active.push_back(d.name);
					r.fields.insert(d.provides.begin(), d.provides.end());
					for (const auto& o : d.offers) r.fields.insert(o.first);
					for (const auto& opt : d.optional)
						if (views.count(opt.first)) r.fields.insert(opt.second);
					changed = true;
				}
			}
			return r;
		}
	}
}
