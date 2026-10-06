#pragma once
// Builds a robot's device side from its definition: tree, ports, blocks and self model (plan §5).
#include <string>
#include <string_view>
#include <vector>

#include "mrs/device/BlockBuilder.h"
#include "mrs/device/DeviceTree.h"
#include "mrs/device/SelfModel.h"
#include "mrs/port/PortManager.h"

namespace MRS {
	namespace Device {
		// A node left out of the robot (plan §7.3 FAULT event).
		struct Fault {
			std::string node;
			std::string reason;
		};

		struct Robot {
			DeviceTree tree;
			Port::PortReport ports;
			std::vector<Fault> faults;
			Blocks blocks;
			SelfModel self;
		};

		// 1. Builds the tree. 2. Assigns ports (port map, address, scan). 3. Opens them; a node with an
		// unresolved or unopened port, or one that fails to start, is unavailable. 4. Propagates
		// unavailability to dependants. 5. Builds the blocks and the self model.
		// Throws BuildError for definition errors; unavailable nodes are faults, not errors.
		Robot BuildRobot(const Protocol::Document& definition, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const Port::PortMap* port_map = nullptr,
		                 const Environment::ProcessorCatalog& catalog = Environment::ProcessorCatalog::Default());
		Robot BuildRobot(std::string_view definition_text, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const Port::PortMap* port_map = nullptr,
		                 const Environment::ProcessorCatalog& catalog = Environment::ProcessorCatalog::Default());
	}
}
