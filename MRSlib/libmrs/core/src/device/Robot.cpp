#include "mrs/device/Robot.h"

#include "mrs/BuildError.h"
#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Device {
		Robot BuildRobot(std::string_view definition_text, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const Port::PortMap* port_map, const Environment::ProcessorCatalog& catalog) {
			auto result = Protocol::Parse(definition_text);
			if (!result.Ok())
				throw BuildError("definition: " + std::string(Protocol::ErrorClassName(result.error->error_class)) + " at byte " +
				                 std::to_string(result.error->offset) + ": " + result.error->message);
			return BuildRobot(result.document, registry, platform, port_map, catalog);
		}

		Robot BuildRobot(const Protocol::Document& definition, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const Port::PortMap* port_map, const Environment::ProcessorCatalog& catalog) {
			Robot robot;
			robot.tree = TreeBuilder(registry).Build(definition);
			const auto nodes = robot.tree.Nodes();

			std::vector<Port::NodePorts> requirements;
			for (DeviceNode* node : nodes)
				if (!node->PortRequirements().empty()) requirements.push_back({node->Name(), node->PortRequirements()});
			Port::PortManager manager(platform);
			robot.ports = manager.Assign(requirements, port_map);

			for (const auto& u : robot.ports.unresolved)
				robot.tree.Find(u.node)->SetUnavailable("port " + u.requirement + ": " + u.reason);
			for (const auto& a : robot.ports.assigned) {
				DeviceNode* node = robot.tree.Find(a.node);
				if (!node->Available()) continue;
				auto port = manager.Open(a);
				if (!port) {
					node->SetUnavailable("port " + a.requirement + ": " + Port::PortTypeName(a.type) + " \"" + a.address +
					                     "\" did not open");
					continue;
				}
				node->AttachPort(a.requirement, std::move(port));
			}
			for (DeviceNode* node : nodes)
				if (node->Available() && !node->Start()) node->SetUnavailable(node->FaultReason().empty() ? "did not start" : node->FaultReason());

			// Dependants of unavailable nodes are unavailable too, to a fixed point.
			bool changed = true;
			while (changed) {
				changed = false;
				for (DeviceNode* node : nodes) {
					if (!node->Available()) continue;
					for (const DeviceNode* dep : node->DependsOn())
						if (!dep->Available()) {
							node->SetUnavailable("depends on " + dep->Name() + ", which is unavailable");
							changed = true;
							break;
						}
				}
			}

			for (DeviceNode* node : nodes)
				if (!node->Available()) robot.faults.push_back({node->Name(), node->FaultReason()});
			robot.blocks = BuildBlocks(robot.tree);
			robot.self = SelfModel::Build(robot.tree, catalog);
			return robot;
		}
	}
}
