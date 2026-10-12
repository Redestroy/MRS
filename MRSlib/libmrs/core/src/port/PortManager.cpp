#include "mrs/port/PortManager.h"

#include <map>
#include <optional>
#include <utility>

#include "mrs/BuildError.h"
#include "mrs/device/Identity.h"
#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Port {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		PortMap PortMap::FromDocument(const Protocol::Document& doc) {
			PortMap map;
			for (const Record& rec : doc.records) {
				if (rec.kind == '@') continue;
				if (rec.code != "P_A") throw BuildError("a port map holds only P_A records, found " + rec.code);
				PortAssignment a;
				a.node = rec.fields.at(0).s;
				a.requirement = rec.fields.at(1).s;
				a.type = *ParsePortType(rec.fields.at(2).s);
				a.address = rec.fields.at(3).s;
				a.params = Device::Params::FromFields(rec.fields, 4);
				a.source = AssignmentSource::PortMap;
				if (map.Find(a.node, a.requirement))
					throw BuildError("port map: two entries for " + a.node + "." + a.requirement);
				map.entries.push_back(std::move(a));
			}
			return map;
		}

		PortMap PortMap::Parse(std::string_view text) {
			auto result = Protocol::Parse(text);
			if (!result.Ok())
				throw BuildError("port map: " + std::string(Protocol::ErrorClassName(result.error->error_class)) + " at byte " +
				                 std::to_string(result.error->offset) + ": " + result.error->message);
			return FromDocument(result.document);
		}

		const PortAssignment* PortMap::Find(const std::string& node, const std::string& requirement) const {
			for (const auto& e : entries)
				if (e.node == node && e.requirement == requirement) return &e;
			return nullptr;
		}

		namespace {
			std::string Where(const std::string& node, const std::string& requirement) { return node + "." + requirement; }

			std::string PortName(PortType type, const std::string& address) {
				return PortTypeName(type) + " \"" + address + "\"";
			}
		}

		PortManager::PortManager(IPlatform& platform) { providers_[kPlatformProvider] = &platform; }

		void PortManager::AddProvider(const std::string& name, IPortProvider& provider) { providers_[name] = &provider; }

		const std::vector<PortInfo>& PortManager::Scan(const std::string& provider) {
			auto it = scans_.find(provider);
			if (it != scans_.end()) return it->second;
			std::vector<PortInfo> ports;
			if (auto p = providers_.find(provider); p != providers_.end()) ports = p->second->Scan();
			for (auto& info : ports) info.provider = provider;
			return scans_[provider] = std::move(ports);
		}

		std::unique_ptr<Port> PortManager::Open(const PortAssignment& assignment) {
			auto it = providers_.find(assignment.provider);
			return it == providers_.end() ? nullptr : it->second->Open(assignment);
		}

		void PortManager::CheckMap(const std::vector<NodePorts>& nodes, const PortMap* map) const {
			// Every map entry must name an existing requirement of the same type.
			if (!map) return;
			for (const auto& e : map->entries) {
				const PortRequirement* found = nullptr;
				for (const auto& n : nodes)
					if (n.node == e.node)
						for (const auto& r : n.requirements)
							if (r.name == e.requirement) found = &r;
				if (!found) throw BuildError("port map: " + Where(e.node, e.requirement) + " is not a port requirement of the robot");
				if (found->type != e.type)
					throw BuildError("port map: " + Where(e.node, e.requirement) + " is " + PortTypeName(found->type) + ", not " +
					                 PortTypeName(e.type));
			}
		}

		PortReport PortManager::Assign(const std::vector<NodePorts>& nodes, const PortMap* map) {
			CheckMap(nodes, map);
			uses_.clear();  // a fresh assignment
			PortReport report;
			for (const auto& n : nodes) AssignNode(n, map, report);
			return report;
		}

		void PortManager::AssignNode(const NodePorts& n, const PortMap* map, PortReport& report) {
			std::vector<std::string> chain = n.providers;
			chain.push_back(kPlatformProvider);
			for (const auto& r : n.requirements) {
				auto unresolved = [&](std::string reason) { report.unresolved.push_back({n.node, r.name, std::move(reason)}); };

				// The nearest provider that offers the type (spec 16 §4.2).
				std::string provider;
				bool missing = false;
				for (const auto& name : chain) {
					auto it = providers_.find(name);
					if (it == providers_.end()) {
						missing = true;
						provider = name;
						break;
					}
					if (it->second->Offers(r.type)) {
						provider = name;
						break;
					}
				}
				if (provider.empty())
					throw BuildError(Where(n.node, r.name) + ": no provider offers port type " + PortTypeName(r.type));
				if (missing) {
					unresolved("provider " + provider + " is unavailable");
					continue;
				}

				PortAssignment a;
				a.node = n.node;
				a.requirement = r.name;
				a.type = r.type;
				a.exclusive = r.exclusive;
				a.params = r.params;
				a.provider = provider;

				if (const PortAssignment* e = map ? map->Find(n.node, r.name) : nullptr) {
					const std::string via = e->params.Text("via", kPlatformProvider);
					if (via != provider)
						throw BuildError("port map: " + Where(n.node, r.name) + " says via " + via + ", but its provider is " + provider);
					a.address = e->address;
					a.params.Merge(e->params);
					a.source = AssignmentSource::PortMap;
				} else if (r.address != "any") {
					a.address = r.address;
					a.source = AssignmentSource::Address;
				} else {
					const std::string identity = r.params.Text("identity");
					if (identity.empty()) {
						unresolved("address any needs a port map entry or an identity to scan for");
						continue;
					}
					std::vector<const PortInfo*> matches;
					for (const auto& p : Scan(provider))
						if (p.type == r.type && Device::IdentityMatches(identity, p.identity)) matches.push_back(&p);
					if (matches.size() != 1) {
						unresolved(matches.empty() ? "no scanned " + PortTypeName(r.type) + " port reports identity " + identity
						                           : std::to_string(matches.size()) + " scanned ports report identity " + identity);
						continue;
					}
					a.address = matches[0]->address;
					a.source = AssignmentSource::Scan;
				}

				// A port the scan does not list is not there.
				bool type_listed = false, present = false;
				for (const auto& p : Scan(provider)) {
					if (p.type != a.type) continue;
					type_listed = true;
					if (p.address == a.address) present = true;
				}
				if (type_listed && !present) {
					unresolved("port " + PortName(a.type, a.address) + " is not present");
					continue;
				}

				// Exclusive ports are exclusive per provider (spec 16 §4.2).
				auto& list = uses_[{provider, a.type, a.address}];
				for (const auto& u : list)
					if (u.exclusive || a.exclusive)
						throw BuildError("port " + PortName(a.type, a.address) + " is given to both " + u.where + " and " +
						                 Where(a.node, a.requirement));
				list.push_back({Where(a.node, a.requirement), a.exclusive});
				report.assigned.push_back(std::move(a));
			}
		}

		Protocol::Document PortManager::WritePortMap(const PortReport& report) {
			Protocol::Document doc;
			Record version;
			version.kind = '@';
			version.fields = {Field::MakeText(FieldType::Id, "MRS"), Field::MakeNum(0.1)};
			doc.records.push_back(version);
			for (const auto& a : report.assigned) {
				Record rec;
				rec.kind = 'P';
				rec.code = "P_A";
				rec.fields = {Field::MakeText(FieldType::Id, a.node), Field::MakeText(FieldType::Id, a.requirement),
				              Field::MakeText(FieldType::Id, PortTypeName(a.type)), Field::MakeText(FieldType::Str, a.address)};
				Device::Params params;
				for (const auto& p : a.params.Items())
					if (p.key != "identity" && p.key != "req" && p.key != "via") params.Add(p.key, p.value);
				if (a.provider != kPlatformProvider) params.Add("via", Device::IdValue(a.provider));
				for (auto& f : params.ToFields()) rec.fields.push_back(std::move(f));
				doc.records.push_back(std::move(rec));
			}
			return doc;
		}
	}
}
