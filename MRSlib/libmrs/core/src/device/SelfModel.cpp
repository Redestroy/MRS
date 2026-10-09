#include "mrs/device/SelfModel.h"

namespace MRS {
	namespace Device {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		SelfModel SelfModel::Build(const DeviceTree& tree, const Environment::ProcessorCatalog& catalog) {
			SelfModel m;
			const HeadNode& head = tree.Head();
			m.robot_type = head.RobotType();
			m.id = head.RobotId();
			m.roles = head.Roles();
			bool messages = false;
			for (DeviceNode* node : tree.Nodes()) {
				if (!node->Available()) continue;
				for (const auto& c : node->Capabilities()) {
					m.capabilities.push_back(c);
					switch (c.kind) {
					case CapabilityKind::Action: m.actions.insert(c.code); break;
					case CapabilityKind::View: m.views.insert(c.code); break;
					case CapabilityKind::Message: messages = true; break;
					case CapabilityKind::Capacity: break;
					}
				}
			}
			// Peer messages decode to views: STATE to V_PEER, shared world fields and detections to
			// V_FLD and V_DET (spec 15 §2.4).
			if (messages) m.views.insert({"V_PEER", "V_FLD", "V_DET"});
			auto resolution = catalog.Resolve(m.views);
			m.fields = std::move(resolution.fields);
			m.processors = std::move(resolution.active);
			return m;
		}

		double SelfModel::Capacity(const std::string& resource) const {
			double total = 0.0;
			for (const auto& c : capabilities)
				if (c.kind == CapabilityKind::Capacity && c.code == resource) total += c.capacity;
			return total;
		}

		Task::CapabilityProfile SelfModel::ToProfile() const {
			Task::CapabilityProfile p;
			p.actions = actions;
			p.fields = fields;
			p.roles.insert(roles.begin(), roles.end());
			p.robot = "r" + std::to_string(id);
			return p;
		}

		Record SelfModel::ToProfileMessage(const std::string& mission, std::int64_t seq, double stamp) const {
			Record rec;
			rec.kind = 'M';
			rec.code = "M_PROFILE";
			rec.fields = {Field::MakeNum(0.1),
			              Field::MakeText(FieldType::Id, mission),
			              Field::MakeText(FieldType::Id, "r" + std::to_string(id)),
			              Field::MakeText(FieldType::Id, "all"),
			              Field::MakeInt(seq),
			              Field::MakeNum(stamp),
			              Field::MakeText(FieldType::Id, robot_type),
			              Field::MakeInt(static_cast<std::int64_t>(roles.size()))};
			for (const auto& role : roles) rec.fields.push_back(Field::MakeText(FieldType::Id, role));
			for (const auto& c : capabilities) {
				rec.fields.push_back(Field::MakeRef(rec.children.size()));
				rec.children.push_back(c.ToRecord());
			}
			return rec;
		}
	}
}
