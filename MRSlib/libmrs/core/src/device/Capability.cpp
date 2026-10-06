#include "mrs/device/Capability.h"

namespace MRS {
	namespace Device {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		Capability Capability::Act(std::string code, std::map<std::string, double> limits) {
			return {CapabilityKind::Action, std::move(code), 0.0, std::move(limits), {}};
		}
		Capability Capability::ViewOf(std::string code, std::map<std::string, double> limits) {
			return {CapabilityKind::View, std::move(code), 0.0, std::move(limits), {}};
		}
		Capability Capability::Messages(std::string mode, std::map<std::string, double> limits) {
			return {CapabilityKind::Message, std::move(mode), 0.0, std::move(limits), {}};
		}
		Capability Capability::Store(std::string resource, double capacity, std::map<std::string, double> limits) {
			return {CapabilityKind::Capacity, std::move(resource), capacity, std::move(limits), {}};
		}

		const char* CapabilityCode(CapabilityKind kind) {
			switch (kind) {
			case CapabilityKind::Action: return "K_A";
			case CapabilityKind::View: return "K_V";
			case CapabilityKind::Message: return "K_M";
			case CapabilityKind::Capacity: return "K_Q";
			}
			return "K_A";
		}

		Record Capability::ToRecord() const {
			Record rec;
			rec.kind = 'K';
			rec.code = CapabilityCode(kind);
			switch (kind) {
			case CapabilityKind::Action:
			case CapabilityKind::View: rec.fields.push_back(Field::MakeText(FieldType::Code, code)); break;
			case CapabilityKind::Message: rec.fields.push_back(Field::MakeText(FieldType::Id, code)); break;
			case CapabilityKind::Capacity:
				rec.fields.push_back(Field::MakeText(FieldType::Id, code));
				rec.fields.push_back(Field::MakeNum(capacity));
				break;
			}
			rec.fields.push_back(Field::MakeInt(static_cast<std::int64_t>(limits.size())));
			for (const auto& limit : limits) {
				rec.fields.push_back(Field::MakeText(FieldType::Id, limit.first));
				rec.fields.push_back(Field::MakeNum(limit.second));
			}
			return rec;
		}

		std::optional<Capability> Capability::FromRecord(const Record& rec) {
			Capability c;
			std::size_t k = 1;
			if (rec.code == "K_A") c.kind = CapabilityKind::Action;
			else if (rec.code == "K_V") c.kind = CapabilityKind::View;
			else if (rec.code == "K_M") c.kind = CapabilityKind::Message;
			else if (rec.code == "K_Q") {
				c.kind = CapabilityKind::Capacity;
				c.capacity = rec.fields.at(1).n;
				k = 2;
			} else {
				return std::nullopt;
			}
			c.code = rec.fields.at(0).s;
			const auto n = static_cast<std::size_t>(rec.fields.at(k).i);
			for (std::size_t j = 0; j < n; ++j) c.limits[rec.fields.at(k + 1 + 2 * j).s] = rec.fields.at(k + 2 + 2 * j).n;
			return c;
		}
	}
}
