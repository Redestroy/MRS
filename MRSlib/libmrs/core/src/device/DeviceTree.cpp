#include "mrs/device/DeviceTree.h"

#include <map>
#include <set>

#include "mrs/BuildError.h"
#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Device {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		bool NameMatches(const std::string& full, const std::string& ref) {
			if (full == ref) return true;
			return !ref.empty() && full.size() > ref.size() && full.compare(full.size() - ref.size(), ref.size(), ref) == 0 &&
			       full[full.size() - ref.size() - 1] == '.';
		}

		std::vector<DeviceNode*> DeviceTree::Matches(const std::string& ref) const {
			std::vector<DeviceNode*> out;
			for (DeviceNode* node : Nodes()) {
				if (node->Name() == ref) return {node};
				if (NameMatches(node->Name(), ref)) out.push_back(node);
			}
			return out;
		}

		DeviceNode* DeviceTree::Find(const std::string& ref) const {
			const auto matches = Matches(ref);
			return matches.size() == 1 ? matches[0] : nullptr;
		}

		std::vector<DeviceNode*> DeviceTree::Nodes() const {
			std::vector<DeviceNode*> out;
			if (!head_) return out;
			std::function<void(DeviceNode&)> walk = [&](DeviceNode& n) {
				out.push_back(&n);
				for (const auto& child : n.Children()) walk(*child);
			};
			walk(*head_);
			return out;
		}

		DeviceTree TreeBuilder::Build(std::string_view mrsd_text) const {
			auto result = Protocol::Parse(mrsd_text);
			if (!result.Ok())
				throw BuildError("definition: " + std::string(Protocol::ErrorClassName(result.error->error_class)) + " at byte " +
				                 std::to_string(result.error->offset) + ": " + result.error->message);
			return Build(result.document);
		}

		DeviceTree TreeBuilder::Build(const Protocol::Document& definition) const {
			const Record* head = nullptr;
			for (const Record& rec : definition.records) {
				if (rec.kind == '@') continue;
				if (rec.kind != 'D') throw BuildError("a definition file holds D records, found " + rec.code);
				if (rec.code != "D_H") throw BuildError("the root of a device tree must be D_H, found " + rec.code);
				if (head) throw BuildError("a device tree has exactly one D_H");
				head = &rec;
			}
			if (!head) throw BuildError("the definition has no D_H");

			std::unique_ptr<DeviceNode> root = BuildNode(*head);
			auto* head_node = dynamic_cast<HeadNode*>(root.get());
			if (!head_node) throw BuildError("type " + head->fields.at(1).s + " is not a head node");
			root.release();
			DeviceTree tree{std::unique_ptr<HeadNode>(head_node)};

			// Full names (spec 16 §5): a node inside a complex device is named after it.
			std::function<void(DeviceNode&, const std::string&)> place = [&](DeviceNode& n, const std::string& scope) {
				if (n.LocalName().find('.') != std::string::npos)
					throw BuildError("node name " + n.LocalName() + " contains '.', which only full names may have");
				n.SetFullName(scope.empty() ? n.LocalName() : scope + "." + n.LocalName());
				const std::string inner = n.Kind() == NodeKind::Complex ? n.Name() : scope;
				for (const auto& child : n.Children()) place(*child, inner);
			};
			place(tree.Head(), "");

			std::set<std::string> names;
			for (DeviceNode* node : tree.Nodes()) {
				if (node != &tree.Head() && node->Kind() == NodeKind::Head) throw BuildError("only the root may be D_H: " + node->Name());
				if (!names.insert(node->Name()).second) throw BuildError("node name " + node->Name() + " is used twice");
			}
			return tree;
		}

		namespace {
			// Every D sub-record below `rec` whose local name is `name`.
			void FindByName(Record& rec, const std::string& name, std::vector<Record*>& out) {
				for (auto& child : rec.children) {
					if (child.kind != 'D') continue;
					if (child.fields.at(0).s == name) out.push_back(&child);
					FindByName(child, name, out);
				}
			}

			// Replaces the parameter slots of a D record, keeping its other slots.
			void SetParams(Record& rec, const Params& params) {
				std::size_t next = 0;
				Params::FromFields(rec.fields, 2, &next);
				std::vector<Field> fields(rec.fields.begin(), rec.fields.begin() + 2);
				for (auto& f : params.ToFields()) fields.push_back(std::move(f));
				fields.insert(fields.end(), rec.fields.begin() + static_cast<std::ptrdiff_t>(next), rec.fields.end());
				rec.fields = std::move(fields);
			}
		}

		Record TreeBuilder::ExpandPart(const Record& include, int depth) const {
			const std::string& name = include.fields.at(0).s;
			const std::string part = include.fields.at(1).s.substr(std::string(kPartPrefix).size());
			if (include.code != "D_X") throw BuildError("node " + name + ": a part is included on a D_X record, not " + include.code);
			if (depth > 16) throw BuildError("node " + name + ": parts nested too deep (a part includes itself?)");
			const Record* source = library_ ? library_->Part(part) : nullptr;
			if (!source) throw BuildError("node " + name + ": unknown part " + part);

			Record out = *source;
			out.fields.at(0).s = name;
			std::size_t next = 0;
			const Params given = Params::FromFields(include.fields, 2, &next);
			Params root = Params::FromFields(out.fields, 2);
			Params root_over;
			std::vector<std::pair<std::string, Param>> child_over;  // local name, parameter
			for (const auto& p : given.Items()) {
				const auto dot = p.key.find('.');
				if (dot == std::string::npos) root_over.Add(p.key, p.value);
				else child_over.push_back({p.key.substr(0, dot), {p.key.substr(dot + 1), p.value}});
			}
			root.Merge(root_over);
			SetParams(out, root);
			// <local>.<key> sets the parameter on that node of the part.
			std::map<Record*, Params> per_child;
			for (const auto& c : child_over) {
				std::vector<Record*> found;
				FindByName(out, c.first, found);
				if (found.size() != 1)
					throw BuildError("node " + name + ": " + c.first + "." + c.second.key + (found.empty() ? " names no node of part " : " is ambiguous in part ") + part);
				per_child[found[0]].Add(c.second.key, c.second.value);
			}
			for (auto& entry : per_child) {
				Params p = Params::FromFields(entry.first->fields, 2);
				p.Merge(entry.second);
				SetParams(*entry.first, p);
			}
			// The include's own K, P_R and D sub-records follow the part's.
			for (std::size_t k = next; k < include.fields.size(); ++k) {
				if (include.fields[k].type != FieldType::Ref) continue;
				out.children.push_back(include.children.at(include.fields[k].ref));
				out.fields.push_back(Field::MakeRef(out.children.size() - 1));
			}
			return out;
		}

		std::unique_ptr<DeviceNode> TreeBuilder::BuildNode(const Record& rec, int depth) const {
			const std::string& type = rec.fields.at(1).s;
			if (type.rfind(kPartPrefix, 0) == 0) {
				std::size_t next = 0;
				Params given = Params::FromFields(rec.fields, 2, &next);
				auto node = BuildNode(ExpandPart(rec, depth), depth + 1);
				node->SetPart(type.substr(std::string(kPartPrefix).size()), std::move(given));
				return node;
			}
			const NodeKind kind = *NodeKindFromCode(rec.code);
			const std::string& name = rec.fields.at(0).s;
			std::size_t next = 0;
			Params params = Params::FromFields(rec.fields, 2, &next);

			std::unique_ptr<DeviceNode> node;
			if (kind == NodeKind::Joint) {
				node.reset(new JointNode());
				node->Configure(name, type, std::move(params));
			} else {
				node = registry_.Create(type, name, params);
				if (node->Kind() != kind)
					throw BuildError("node " + name + ": " + type + " is a " + NodeCode(node->Kind()) + ", not a " + rec.code);
			}

			std::vector<const Record*> children;
			for (std::size_t k = next; k < rec.fields.size(); ++k) {
				if (rec.fields[k].type != FieldType::Ref) continue;
				const Record& sub = rec.children.at(rec.fields[k].ref);
				if (sub.kind == 'P') {
					Port::PortRequirement over;
					over.type = *Port::ParsePortType(sub.fields.at(0).s);
					over.address = sub.fields.at(1).s;
					over.exclusive = sub.fields.at(2).b;
					over.params = Params::FromFields(sub.fields, 3);
					const std::string req = over.params.Text("req");
					node->OverridePort(over, req);
				} else if (sub.kind == 'K') {
					node->OverrideCapability(*Capability::FromRecord(sub));
				} else if (sub.kind == 'D') {
					children.push_back(&sub);
				}
			}

			if (auto* complex = dynamic_cast<ComplexDevice*>(node.get())) complex->Expand(registry_);
			for (const Record* child : children) node->AddChild(BuildNode(*child, depth));
			if (auto* joint = dynamic_cast<JointNode*>(node.get())) joint->CheckChildren();
			return node;
		}
	}
}
