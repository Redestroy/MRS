#pragma once
// Blocks: the available edge nodes grouped by capability (plan §5). The controller talks to blocks,
// not to single nodes.
#include <map>
#include <string>
#include <vector>

#include "mrs/device/DeviceTree.h"
#include "mrs/task/TaskExecutor.h"

namespace MRS {
	namespace Device {
		class SensorBlock {
		public:
			void Add(DeviceNode* node) { sources_.push_back(node); }
			// Every new view at time t, in tree order.
			std::vector<View> Sample(double t) const;
			const std::vector<DeviceNode*>& Sources() const { return sources_; }

		private:
			std::vector<DeviceNode*> sources_;  // sensors, and storage devices that declare views
		};

		// Action dispatch (spec 04 §4.2).
		class ActuatorBlock : public Task::IActionSink {
		public:
			// Throws BuildError when two actuators are `default T` for the same code.
			explicit ActuatorBlock(std::vector<Actuator*> actuators = {});

			Device::ActionStatus Dispatch(const ActionMap& action, double t) override;
			// Control step of every actuator, after dispatch.
			void Update(double t);

			// The actuator an entry goes to, or null when it has no route.
			Actuator* Route(const ActionMapEntry& entry) const;
			const std::vector<Actuator*>& Actuators() const { return actuators_; }

		private:
			std::vector<Actuator*> actuators_;
			std::map<std::string, std::vector<Actuator*>> by_code_;
			std::map<std::string, Actuator*> default_;  // code -> its `any` route
		};

		class CommBlock {
		public:
			void Add(CommunicationDevice* device) { devices_.push_back(device); }
			// Sends on every device. Returns true if at least one accepted it.
			bool Send(const std::string& message, const std::string& recipient = "all");
			std::vector<std::string> Receive();
			bool Empty() const { return devices_.empty(); }

		private:
			std::vector<CommunicationDevice*> devices_;
		};

		class StorageBlock {
		public:
			void Add(StorageDevice* device) { devices_.push_back(device); }
			// Total capacity of a resource over the available storage devices.
			double Capacity(const std::string& resource) const;
			const std::vector<StorageDevice*>& Devices() const { return devices_; }

		private:
			std::vector<StorageDevice*> devices_;
		};

		struct Blocks {
			SensorBlock sensors;
			ActuatorBlock actuators;
			CommBlock comms;
			StorageBlock storage;
		};

		// Groups the available edge nodes of the tree.
		Blocks BuildBlocks(const DeviceTree& tree);
	}
}
