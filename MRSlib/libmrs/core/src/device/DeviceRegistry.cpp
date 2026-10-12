#include "mrs/device/DeviceRegistry.h"

#include "mrs/BuildError.h"

namespace MRS {
	namespace Device {
		void DeviceRegistry::Add(const std::string& type_key, Factory factory, Params defaults, std::vector<std::string> identities) {
			Entry entry{std::move(factory), std::move(defaults), {}};
			for (const auto& text : identities) {
				auto pattern = IdentityPattern::Parse(text);
				if (!pattern) throw BuildError("type " + type_key + ": \"" + text + "\" is not an identity pattern");
				entry.identities.push_back(*pattern);
			}
			entries_[type_key] = std::move(entry);
		}

		std::unique_ptr<DeviceNode> DeviceRegistry::Create(const std::string& type_key, const std::string& name,
		                                                   const Params& params) const {
			auto it = entries_.find(type_key);
			if (it == entries_.end()) throw BuildError("node " + name + ": unknown device type " + type_key);
			Params merged = it->second.defaults;
			merged.Merge(params);
			auto node = it->second.factory();
			node->Configure(name, type_key, std::move(merged));
			return node;
		}

		std::string DeviceRegistry::Match(const Identity& id) const {
			std::string best;
			int best_spec = -1;
			bool tie = false;
			for (const auto& entry : entries_)
				for (const auto& pattern : entry.second.identities) {
					if (!pattern.Matches(id)) continue;
					const int spec = pattern.Specificity();
					if (spec > best_spec) {
						best = entry.first;
						best_spec = spec;
						tie = false;
					} else if (spec == best_spec && entry.first != best) {
						tie = true;
					}
				}
			if (tie) throw BuildError("identity " + id.ToString() + " matches several device types equally (" + best + " and others)");
			return best;
		}
	}
}
