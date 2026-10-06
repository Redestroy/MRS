#pragma once
// Type key -> device class (spec 04 §5).
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <type_traits>

#include "mrs/device/DeviceNode.h"

namespace MRS {
	namespace Device {
		class DeviceRegistry {
		public:
			using Factory = std::function<std::unique_ptr<DeviceNode>()>;

			// Type keys are <device>.<platform> or <device>.default. `defaults` are merged under the
			// definition file's parameters, so a platform key can carry platform device names.
			template <class T>
			void Register(const std::string& type_key, Params defaults = {}) {
				static_assert(std::is_base_of<DeviceNode, T>::value, "T must derive from DeviceNode");
				Add(type_key, [] { return std::unique_ptr<DeviceNode>(new T()); }, std::move(defaults));
			}
			void Add(const std::string& type_key, Factory factory, Params defaults = {});

			bool Has(const std::string& type_key) const { return entries_.count(type_key) != 0; }
			// Creates and configures a node. Throws BuildError for an unknown key.
			std::unique_ptr<DeviceNode> Create(const std::string& type_key, const std::string& name, const Params& params) const;

		private:
			struct Entry {
				Factory factory;
				Params defaults;
			};
			std::map<std::string, Entry> entries_;
		};
	}
}
