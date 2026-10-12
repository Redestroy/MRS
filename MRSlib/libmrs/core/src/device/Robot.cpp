#include "mrs/device/Robot.h"

#include <map>
#include <set>
#include <tuple>
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

		namespace {
			const Port::PortInfo* Scanned(Port::PortManager& manager, const std::string& provider, const Port::PortType& type,
			                              const std::string& address) {
				for (const auto& p : manager.Scan(provider))
					if (p.type == type && p.address == address) return &p;
				return nullptr;
			}

			std::string TypeOf(const DeviceNode& node) { return node.Part().empty() ? node.TypeKey() : kPartPrefix + node.Part(); }

			// One build. `identity_params` go to TreeBuilder; `updates` gets the parameters of updated devices.
			Robot BuildOnce(const Protocol::Document& definition, const DeviceRegistry& registry, Port::IPlatform& platform,
			                const BuildOptions& options, const std::map<std::string, Params>& identity_params,
			                std::map<std::string, Params>& updates) {
			const Port::PortMap* port_map = options.port_map;
			const auto& catalog = options.catalog ? *options.catalog : Environment::ProcessorCatalog::Default();
			std::vector<const IdentityMap*> library_maps;
			if (options.library) library_maps = options.library->Maps();
			const IdentityResolver resolver(registry, options.identity_map, library_maps);
			Robot robot;
			TreeBuilder builder(registry, options.library);
			builder.SetIdentityParams(identity_params);
			robot.tree = builder.Build(definition);
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
				// The boot check (spec 16 §7.3): the identity the port map saved against what is there now.
				auto saved = [&](const std::string& requirement) -> std::string {
					const Port::PortAssignment* e = map ? map->Find(node->Name(), requirement) : nullptr;
					return e ? e->params.Text("identity") : std::string();
				};
				for (const auto& u : report.unresolved) {
					const std::string expected = saved(u.requirement);
					if (!expected.empty() && u.reason.find("is not present") != std::string::npos)
						robot.check.push_back({"missing", node->Name(), expected, ""});
					node->SetUnavailable("port " + u.requirement + ": " + u.reason);
				}
				for (const auto& a : report.assigned) {
					const std::string expected = saved(a.requirement);
					if (expected.empty() || expected == a.identity) continue;
					const auto exp = Identity::Parse(expected), found = Identity::Parse(a.identity);
					if (exp && found && exp->vendor == found->vendor && exp->model == found->model && exp->rev == found->rev) {
						robot.check.push_back({"swapped", node->Name(), expected, a.identity});
						continue;
					}
					std::optional<IdentityMatch> match;
					try {
						const auto* info = Scanned(manager, a.provider, a.type, a.address);
						match = resolver.Resolve(a.identity, info ? info->descriptor : std::string());
					} catch (const BuildError&) {
						match.reset();
					}
					if (match && !match->Ignored() && match->type == TypeOf(*node)) {
						robot.check.push_back({"updated", node->Name(), expected, a.identity});
						if (!match->params.Empty()) updates[node->Name()] = match->params;
					} else {
						robot.check.push_back({"changed", node->Name(), expected, a.identity});
						node->SetUnavailable("port " + a.requirement + ": device changed from " + expected + " to " +
						                     (a.identity.empty() ? std::string("an unknown device") : a.identity));
					}
				}
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

			// Devices that are there and known but used by no node (spec 16 §7.3, `extra`).
			std::set<std::tuple<std::string, Port::PortType, std::string>> used;
			for (const auto& a : robot.ports.assigned) used.insert({a.provider, a.type, a.address});
			for (const auto& provider : manager.ProviderNames())
				for (const auto& p : manager.Scan(provider)) {
					if (used.count({provider, p.type, p.address})) continue;
					std::optional<IdentityMatch> match;
					try {
						match = resolver.Resolve(p.identity, p.descriptor);
					} catch (const BuildError&) {
						match.reset();
					}
					if (match && !match->Ignored()) robot.check.push_back({"extra", provider, p.address, p.identity});
				}

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

		Robot BuildRobot(const Protocol::Document& definition, const DeviceRegistry& registry, Port::IPlatform& platform,
		                 const BuildOptions& options) {
			std::map<std::string, Params> updates;
			{
				Robot robot = BuildOnce(definition, registry, platform, options, {}, updates);
				if (updates.empty()) return robot;
			}
			// An updated device's own information takes effect (spec 16 §7.3): the first build is released
			// and the robot is built again with it.
			std::map<std::string, Params> again;
			return BuildOnce(definition, registry, platform, options, updates, again);
		}

		Protocol::Record CheckFinding::ToRecord() const {
			Protocol::Record rec;
			rec.kind = 'I';
			rec.code = "I_R";
			rec.fields = {Protocol::Field::MakeText(Protocol::FieldType::Id, result), Protocol::Field::MakeText(Protocol::FieldType::Id, node),
			              Protocol::Field::MakeText(Protocol::FieldType::Str, expected), Protocol::Field::MakeText(Protocol::FieldType::Str, found)};
			return rec;
		}

		void UpdateProviders(Robot& robot, double t) {
			for (Port::IPortProvider* p : robot.providers) p->Update(t);
		}
	}
}
