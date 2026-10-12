#include "mrs/device/Robot.h"

#include <map>
#include <optional>

#include "mrs/BuildError.h"
#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Device {
		Robot BuildRobot(std::string_view definition_text, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const BuildOptions& options) {
			auto result = Protocol::Parse(definition_text);
			if (!result.Ok())
				throw BuildError("definition: " + std::string(Protocol::ErrorClassName(result.error->error_class)) + " at byte " +
				                 std::to_string(result.error->offset) + ": " + result.error->message);
			return BuildRobot(result.document, registry, platform, options);
		}

		Robot BuildRobot(std::string_view definition_text, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const Port::PortMap* port_map, const Environment::ProcessorCatalog& catalog) {
			BuildOptions options;
			options.port_map = port_map;
			options.catalog = &catalog;
			return BuildRobot(definition_text, registry, platform, options);
		}

		Robot BuildRobot(const Protocol::Document& definition, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const Port::PortMap* port_map, const Environment::ProcessorCatalog& catalog) {
			BuildOptions options;
			options.port_map = port_map;
			options.catalog = &catalog;
			return BuildRobot(definition, registry, platform, options);
		}

		Robot BuildRobot(const Protocol::Document& definition, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const BuildOptions& options) {
			const Port::PortMap* port_map = options.port_map;
			const auto& catalog = options.catalog ? *options.catalog : Environment::ProcessorCatalog::Default();
			Robot robot;
			robot.tree = TreeBuilder(registry, options.library).Build(definition);
			const auto nodes = robot.tree.Nodes();

			// Each node's requirements, with the provider joints above it, nearest first (spec 16 §4.2).
			std::vector<Port::NodePorts> requirements;
			std::map<const DeviceNode*, std::size_t> index;
			for (DeviceNode* node : nodes) {
				Port::NodePorts np{node->Name(), node->PortRequirements(), {}};
				for (DeviceNode* up = node->Parent(); up; up = up->Parent())
					if (dynamic_cast<ProviderJoint*>(up)) np.providers.push_back(up->Name());
				index[node] = requirements.size();
				requirements.push_back(std::move(np));
			}
			// Port map entries may name nodes by a short name (spec 16 §5); assignment uses full names.
			std::optional<Port::PortMap> resolved_map;
			if (port_map) {
				resolved_map = *port_map;
				for (auto& e : resolved_map->entries) {
					const auto matches = robot.tree.Matches(e.node);
					if (matches.size() > 1) throw BuildError("port map: node name " + e.node + " is ambiguous");
					if (matches.size() == 1) e.node = matches[0]->Name();
				}
			}
			const Port::PortMap* map = resolved_map ? &*resolved_map : nullptr;
			Port::PortManager manager(platform);
			manager.CheckMap(requirements, map);

			// In tree order: a provider joint gets its ports and starts before its subtree is assigned.
			for (DeviceNode* node : nodes) {
				const auto& np = requirements[index[node]];
				for (const auto& name : np.providers) {
					const DeviceNode* provider = robot.tree.Find(name);
					if (provider && !provider->Available()) {
						node->SetUnavailable("provider " + name + " is unavailable");
						break;
					}
				}
				if (!node->Available() || np.requirements.empty()) {
					if (node->Available()) {
						if (auto* joint = dynamic_cast<ProviderJoint*>(node)) {
							if (!joint->Start()) joint->SetUnavailable(joint->FaultReason().empty() ? "did not start" : joint->FaultReason());
							else manager.AddProvider(joint->Name(), *joint);
						}
					}
					continue;
				}
				Port::PortReport report;
				manager.AssignNode(np, map, report);
				for (const auto& u : report.unresolved) node->SetUnavailable("port " + u.requirement + ": " + u.reason);
				for (auto& a : report.assigned) {
					if (node->Available()) {
						auto port = manager.Open(a);
						if (!port)
							node->SetUnavailable("port " + a.requirement + ": " + Port::PortTypeName(a.type) + " \"" + a.address +
							                     "\" did not open");
						else
							node->AttachPort(a.requirement, std::move(port));
					}
					robot.ports.assigned.push_back(std::move(a));
				}
				for (auto& u : report.unresolved) robot.ports.unresolved.push_back(std::move(u));
				if (auto* joint = dynamic_cast<ProviderJoint*>(node); joint && joint->Available()) {
					if (!joint->Start()) joint->SetUnavailable(joint->FaultReason().empty() ? "did not start" : joint->FaultReason());
					else manager.AddProvider(joint->Name(), *joint);
				}
			}
			for (DeviceNode* node : nodes)
				if (node->Available() && !dynamic_cast<ProviderJoint*>(node) && !node->Start())
					node->SetUnavailable(node->FaultReason().empty() ? "did not start" : node->FaultReason());
			for (DeviceNode* node : nodes)
				if (auto* joint = dynamic_cast<ProviderJoint*>(node); joint && joint->Available()) robot.providers.push_back(joint);

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

		void UpdateProviders(Robot& robot, double t) {
			for (Port::IPortProvider* p : robot.providers) p->Update(t);
		}
	}
}
