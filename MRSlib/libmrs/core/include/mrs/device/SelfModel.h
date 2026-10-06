#pragma once
// What the robot can do, from its available nodes (spec 04 §7).
#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include "mrs/device/DeviceTree.h"
#include "mrs/protocol/Record.h"
#include "mrs/task/Task.h"
#include "mrs/world/ProcessorCatalog.h"

namespace MRS {
	namespace Device {
		struct SelfModel {
			std::string robot_type;
			std::int64_t id = 0;
			std::vector<std::string> roles;
			std::vector<Capability> capabilities;  // of available nodes, in tree order, with their node names
			std::set<std::string> actions;         // K_A codes
			std::set<std::string> views;           // K_V codes, plus V_PEER with any K_M
			std::set<std::string> fields;          // worldview fields the robot can provide
			std::vector<std::string> processors;   // catalog processors that are active

			static SelfModel Build(const DeviceTree& tree,
			                       const Environment::ProcessorCatalog& catalog = Environment::ProcessorCatalog::Default());

			bool Accepts(const std::string& action) const { return actions.count(action) != 0; }
			bool Provides(const std::string& field) const { return fields.count(field) != 0; }
			double Capacity(const std::string& resource) const;

			Task::CapabilityProfile ToProfile() const;
			// The PROFILE message of spec 06 §4.
			Protocol::Record ToProfileMessage(const std::string& mission, std::int64_t seq, double stamp) const;
		};
	}
}
