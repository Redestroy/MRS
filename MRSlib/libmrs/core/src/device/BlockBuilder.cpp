#include "mrs/device/BlockBuilder.h"

#include <set>

#include "mrs/BuildError.h"

namespace MRS {
	namespace Device {
		std::vector<View> SensorBlock::Sample(double t) const {
			std::vector<View> out;
			for (DeviceNode* node : sources_) {
				if (auto* sensor = dynamic_cast<Sensor*>(node)) sensor->Sample(t, out);
				else if (auto* storage = dynamic_cast<StorageDevice*>(node)) storage->Sample(t, out);
			}
			return out;
		}

		ActuatorBlock::ActuatorBlock(std::vector<Actuator*> actuators) : actuators_(std::move(actuators)) {
			for (Actuator* a : actuators_)
				for (const auto& c : a->Capabilities())
					if (c.kind == CapabilityKind::Action) by_code_[c.code].push_back(a);
			for (const auto& entry : by_code_) {
				const auto& list = entry.second;
				if (list.size() == 1) {
					default_[entry.first] = list[0];
					continue;
				}
				Actuator* chosen = nullptr;
				for (Actuator* a : list) {
					if (!a->IsDefault()) continue;
					if (chosen)
						throw BuildError("both " + chosen->Name() + " and " + a->Name() + " are default T for " + entry.first);
					chosen = a;
				}
				if (chosen) default_[entry.first] = chosen;  // otherwise the code has no `any` route
			}
		}

		Actuator* ActuatorBlock::Route(const ActionMapEntry& entry) const {
			const std::string& code = entry.action.code;
			if (code == "A_N" || code == "A_W" || code == "A_I") return nullptr;
			if (entry.target == "any") {
				auto it = default_.find(code);
				return it == default_.end() ? nullptr : it->second;
			}
			auto it = by_code_.find(code);
			if (it == by_code_.end()) return nullptr;
			for (Actuator* a : it->second)
				if (a->Name() == entry.target) return a;
			return nullptr;
		}

		ActionStatus ActuatorBlock::Dispatch(const ActionMap& action, double t) {
			// The whole map is checked before any entry is applied.
			std::vector<Actuator*> routes;
			for (const auto& entry : action.entries) {
				Actuator* a = Route(entry);
				if (!a) return ActionStatus::REJECTED;
				routes.push_back(a);
			}
			bool failed = false, running = false;
			for (std::size_t k = 0; k < routes.size(); ++k) {
				switch (routes[k]->Apply(action.entries[k].action, t)) {
				case ActionStatus::FAILED:
				case ActionStatus::REJECTED: failed = true; break;
				case ActionStatus::RUNNING: running = true; break;
				case ActionStatus::DONE: break;
				}
			}
			if (failed) return ActionStatus::FAILED;
			return running ? ActionStatus::RUNNING : ActionStatus::DONE;
		}

		void ActuatorBlock::Update(double t) {
			for (Actuator* a : actuators_) a->Update(t);
		}

		bool CommBlock::Send(const std::string& message, const std::string& recipient) {
			bool sent = false;
			for (auto* d : devices_) sent = d->Send(message, recipient) || sent;
			return sent;
		}

		std::vector<std::string> CommBlock::Receive() {
			std::vector<std::string> out;
			for (auto* d : devices_)
				for (auto& m : d->Receive()) out.push_back(std::move(m));
			return out;
		}

		double StorageBlock::Capacity(const std::string& resource) const {
			double total = 0.0;
			for (auto* d : devices_)
				for (const auto& c : d->Capabilities())
					if (c.kind == CapabilityKind::Capacity && c.code == resource) total += c.capacity;
			return total;
		}

		Blocks BuildBlocks(const DeviceTree& tree) {
			Blocks blocks;
			std::vector<Actuator*> actuators;
			for (DeviceNode* node : tree.Nodes()) {
				if (!node->Available()) continue;
				bool has_views = false;
				for (const auto& c : node->Capabilities())
					if (c.kind == CapabilityKind::View) has_views = true;
				switch (node->Kind()) {
				case NodeKind::Sensor: blocks.sensors.Add(node); break;
				case NodeKind::Actuator: actuators.push_back(static_cast<Actuator*>(node)); break;
				case NodeKind::Communication: blocks.comms.Add(static_cast<CommunicationDevice*>(node)); break;
				case NodeKind::Storage:
					blocks.storage.Add(static_cast<StorageDevice*>(node));
					if (has_views) blocks.sensors.Add(node);
					break;
				default: break;
				}
			}
			blocks.actuators = ActuatorBlock(std::move(actuators));
			return blocks;
		}
	}
}
