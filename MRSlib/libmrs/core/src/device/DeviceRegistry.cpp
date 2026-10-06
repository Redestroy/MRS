#include "mrs/device/DeviceRegistry.h"

#include "mrs/BuildError.h"

namespace MRS {
	namespace Device {
		void DeviceRegistry::Add(const std::string& type_key, Factory factory, Params defaults) {
			entries_[type_key] = Entry{std::move(factory), std::move(defaults)};
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
	}
}
