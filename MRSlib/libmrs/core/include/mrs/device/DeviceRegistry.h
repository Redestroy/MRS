#pragma once
// Type key -> device class (spec 04 §5), with the identities each class declares (spec 16 §3.2).
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "mrs/device/DeviceNode.h"
#include "mrs/device/Identity.h"

namespace MRS {
	namespace Device {
		class DeviceRegistry {
		public:
			using Factory = std::function<std::unique_ptr<DeviceNode>()>;

			// Type keys are <device>.<platform> or <device>.default. `defaults` are merged under the
			// definition file's parameters, so a platform key can carry platform device names.
			// `identities` are identity patterns of devices this key describes. Throws BuildError for a bad pattern.
			template <class T>
			void Register(const std::string& type_key, Params defaults = {}, std::vector<std::string> identities = {}) {
				static_assert(std::is_base_of<DeviceNode, T>::value, "T must derive from DeviceNode");
				Add(type_key, [] { return std::unique_ptr<DeviceNode>(new T()); }, std::move(defaults), std::move(identities));
			}
			void Add(const std::string& type_key, Factory factory, Params defaults = {}, std::vector<std::string> identities = {});

			bool Has(const std::string& type_key) const { return entries_.count(type_key) != 0; }
			// Creates and configures a node. Throws BuildError for an unknown key.
			std::unique_ptr<DeviceNode> Create(const std::string& type_key, const std::string& name, const Params& params) const;

			// The type key whose most specific identity pattern matches, or empty. Throws BuildError when two keys
			// match with the same specificity.
			std::string Match(const Identity& id) const;

		private:
			struct Entry {
				Factory factory;
				Params defaults;
				std::vector<IdentityPattern> identities;
			};
			std::map<std::string, Entry> entries_;
		};

		// The platform-independent keys: head.default and group.default (spec 16 §6.1).
		void RegisterBaseDevices(DeviceRegistry& registry);
	}
}
