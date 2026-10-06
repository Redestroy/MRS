#include "mrs/port/PortManager.h"

#include <map>
#include <optional>
#include <utility>

#include "mrs/BuildError.h"
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
				return std::string(PortTypeName(type)) + " \"" + address + "\"";
			}
		}

		PortReport PortManager::Assign(const std::vector<NodePorts>& nodes, const PortMap* map) {
			// Every map entry must name an existing requirement of the same type.
			if (map) {
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

			std::optional<std::vector<PortInfo>> scan;
			auto scanned = [&]() -> const std::vector<PortInfo>& {
				if (!scan) scan = platform_.Scan();
				return *scan;
			};

			PortReport report;
			struct Use {
				std::string where;
				bool exclusive;
			};
			std::map<std::pair<PortType, std::string>, std::vector<Use>> uses;

			for (const auto& n : nodes) {
				for (const auto& r : n.requirements) {
					PortAssignment a;
					a.node = n.node;
					a.requirement = r.name;
					a.type = r.type;
					a.exclusive = r.exclusive;
					a.params = r.params;
					auto unresolved = [&](std::string reason) { report.unresolved.push_back({n.node, r.name, std::move(reason)}); };

					if (const PortAssignment* e = map ? map->Find(n.node, r.name) : nullptr) {
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
						for (const auto& p : scanned())
							if (p.type == r.type && p.identity == identity) matches.push_back(&p);
						if (matches.size() != 1) {
							unresolved(matches.empty() ? "no scanned " + std::string(PortTypeName(r.type)) + " port reports identity " + identity
							                           : std::to_string(matches.size()) + " scanned ports report identity " + identity);
							continue;
						}
						a.address = matches[0]->address;
						a.source = AssignmentSource::Scan;
					}

					// A port the scan does not list is not there.
					bool type_listed = false, present = false;
					for (const auto& p : scanned()) {
						if (p.type != a.type) continue;
						type_listed = true;
						if (p.address == a.address) present = true;
					}
					if (type_listed && !present) {
						unresolved("port " + PortName(a.type, a.address) + " is not present");
						continue;
					}

					auto& list = uses[{a.type, a.address}];
					for (const auto& u : list)
						if (u.exclusive || a.exclusive)
							throw BuildError("port " + PortName(a.type, a.address) + " is given to both " + u.where + " and " +
							                 Where(a.node, a.requirement));
					list.push_back({Where(a.node, a.requirement), a.exclusive});
					report.assigned.push_back(std::move(a));
				}
			}
			return report;
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
					if (p.key != "identity" && p.key != "req") params.Add(p.key, p.value);
				for (auto& f : params.ToFields()) rec.fields.push_back(std::move(f));
				doc.records.push_back(std::move(rec));
			}
			return doc;
		}
	}
}
