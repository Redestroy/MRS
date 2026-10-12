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

		// Optional inputs of BuildRobot.
		struct BuildOptions {
			const Port::PortMap* port_map = nullptr;      // the robot's .mrsp
			const DeviceLibrary* library = nullptr;       // parts and library identity maps (spec 16 §6)
			const Environment::ProcessorCatalog* catalog = nullptr;  // null: the default catalog
		};

		struct Robot {
			DeviceTree tree;
			Port::PortReport ports;
			std::vector<Fault> faults;
			Blocks blocks;
			SelfModel self;
			std::vector<Port::IPortProvider*> providers;  // the available provider joints, in tree order (spec 16 §4)
		};

		// Per-tick work of the provider joints, before sensors are sampled (spec 16 §4.1).
		void UpdateProviders(Robot& robot, double t);

		// 1. Builds the tree. 2. Assigns ports (port map, address, scan). 3. Opens them; a node with an
		// unresolved or unopened port, or one that fails to start, is unavailable. 4. Propagates
		// unavailability to dependants. 5. Builds the blocks and the self model.
		// Throws BuildError for definition errors; unavailable nodes are faults, not errors.
		Robot BuildRobot(const Protocol::Document& definition, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const BuildOptions& options);
		Robot BuildRobot(std::string_view definition_text, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const BuildOptions& options);
		Robot BuildRobot(const Protocol::Document& definition, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const Port::PortMap* port_map = nullptr,
		                 const Environment::ProcessorCatalog& catalog = Environment::ProcessorCatalog::Default());
		Robot BuildRobot(std::string_view definition_text, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const Port::PortMap* port_map = nullptr,
		                 const Environment::ProcessorCatalog& catalog = Environment::ProcessorCatalog::Default());
	}
}
