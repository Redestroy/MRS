#include "mrs/device/DeviceTree.h"

#include <set>

#include "mrs/BuildError.h"
#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Device {
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

		std::unique_ptr<DeviceNode> TreeBuilder::BuildNode(const Record& rec) const {
			const NodeKind kind = *NodeKindFromCode(rec.code);
			const std::string& name = rec.fields.at(0).s;
			const std::string& type = rec.fields.at(1).s;
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
			for (const Record* child : children) node->AddChild(BuildNode(*child));
			if (auto* joint = dynamic_cast<JointNode*>(node.get())) joint->CheckChildren();
			return node;
		}
	}
}
