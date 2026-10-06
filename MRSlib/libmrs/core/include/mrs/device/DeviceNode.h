#pragma once
// Device tree nodes, layer 1 (spec 04 §1).
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "mrs/device/Action.h"
#include "mrs/device/Capability.h"
#include "mrs/device/Params.h"
#include "mrs/port/Port.h"

namespace MRS {
	namespace Device {
		enum class NodeKind { Head, Joint, Complex, Sensor, Actuator, Communication, Storage };

		const char* NodeCode(NodeKind kind);  // "D_H", ...
		std::optional<NodeKind> NodeKindFromCode(std::string_view code);

		class DeviceRegistry;

		class DeviceNode {
		public:
			explicit DeviceNode(NodeKind kind) : kind_(kind) {}
			virtual ~DeviceNode() = default;
			DeviceNode(const DeviceNode&) = delete;
			DeviceNode& operator=(const DeviceNode&) = delete;

			NodeKind Kind() const { return kind_; }
			const std::string& Name() const { return name_; }
			const std::string& TypeKey() const { return type_key_; }
			const Params& GetParams() const { return params_; }

			// Called once by the registry. Reads the parameters, then OnConfigure() declares
			// the class's capabilities and port requirements. Throws BuildError on bad parameters.
			void Configure(std::string name, std::string type_key, Params params);

			// Capabilities as declared, with K overrides applied (spec 04 §4.1).
			const std::vector<Capability>& Capabilities() const { return capabilities_; }
			// Replaces the limits of a declared capability. Throws BuildError if the class does not declare it.
			void OverrideCapability(const Capability& over);
			bool HasCapability(CapabilityKind kind, const std::string& code) const;

			// Port requirements as declared, with P_R overrides applied (spec 04 §6.1).
			const std::vector<Port::PortRequirement>& PortRequirements() const { return ports_; }
			// Replaces the requirement named by `req`, or the only one of that type. Throws BuildError otherwise.
			void OverridePort(const Port::PortRequirement& over, const std::string& req);

			void AttachPort(const std::string& requirement, std::unique_ptr<Port::Port> port);
			Port::Port* GetPort(const std::string& requirement) const;

			// Availability after port assignment (spec 04 §6.3).
			bool Available() const { return available_; }
			const std::string& FaultReason() const { return fault_; }
			void SetUnavailable(std::string reason);
			// Nodes this one cannot work without (the fcu needs its motors).
			virtual std::vector<const DeviceNode*> DependsOn() const { return {}; }
			// Called once the ports are attached. Returns false (with SetUnavailable) if the device cannot start.
			virtual bool Start() { return true; }

			// Tree links.
			DeviceNode* Parent() const { return parent_; }
			const std::vector<std::unique_ptr<DeviceNode>>& Children() const { return children_; }
			DeviceNode& AddChild(std::unique_ptr<DeviceNode> child);
			// True for nodes a complex device created (spec 04 §3).
			bool IsVirtual() const { return virtual_; }
			void SetVirtual(bool v) { virtual_ = v; }

		protected:
			virtual void OnConfigure() {}
			void Declare(Capability capability);
			void DeclarePort(std::string name, Port::PortType type, std::string address, bool exclusive = true, Params params = {});
			// The rate_hz parameter as port parameters, if set.
			Params RateParams() const;

		private:
			NodeKind kind_;
			std::string name_;
			std::string type_key_;
			Params params_;
			std::vector<Capability> capabilities_;
			std::vector<Port::PortRequirement> ports_;
			std::vector<std::pair<std::string, std::unique_ptr<Port::Port>>> open_ports_;
			bool available_ = true;
			std::string fault_;
			DeviceNode* parent_ = nullptr;
			std::vector<std::unique_ptr<DeviceNode>> children_;
			bool virtual_ = false;
		};

		// Edge: produces views.
		class Sensor : public DeviceNode {
		public:
			Sensor() : DeviceNode(NodeKind::Sensor) {}
			// Appends the views that have a new sample at time t.
			virtual void Sample(double t, std::vector<View>& out) = 0;
		};

		// Edge: executes actions (spec 03 §8.2, spec 04 §4.2).
		class Actuator : public DeviceNode {
		public:
			Actuator() : DeviceNode(NodeKind::Actuator) {}
			virtual ActionStatus Apply(const Action& action, double t) = 0;
			// Runs every control tick, after dispatch.
			virtual void Update(double t) { (void)t; }
			// Parameter `default T`: gets `any` actions whose code several actuators accept.
			bool IsDefault() const { return GetParams().Bool("default", false); }
		};

		// Edge: sends and receives messages (spec 06).
		class CommunicationDevice : public DeviceNode {
		public:
			CommunicationDevice() : DeviceNode(NodeKind::Communication) {}
			// `recipient` is a robot id such as "r2", or "all" for broadcast.
			virtual bool Send(const std::string& message, const std::string& recipient) = 0;
			virtual std::vector<std::string> Receive() = 0;
		};

		// Edge: stores a resource. May also produce views (the battery produces V_BAT).
		class StorageDevice : public DeviceNode {
		public:
			StorageDevice() : DeviceNode(NodeKind::Storage) {}
			virtual void Sample(double t, std::vector<View>& out) {
				(void)t;
				(void)out;
			}
		};

		// Root (spec 04 §2.3).
		class HeadNode : public DeviceNode {
		public:
			HeadNode() : DeviceNode(NodeKind::Head) {}
			const std::string& RobotType() const { return robot_type_; }
			const std::vector<std::string>& Roles() const { return roles_; }
			std::int64_t RobotId() const { return id_; }

		protected:
			void OnConfigure() override;

		private:
			std::string robot_type_;
			std::vector<std::string> roles_;
			std::int64_t id_ = 0;
		};

		// A link with an interface and connection rules (spec 04 §2.2). Not in the registry:
		// the `type` slot of D_J is the interface kind.
		class JointNode : public DeviceNode {
		public:
			JointNode() : DeviceNode(NodeKind::Joint) {}
			const std::string& Interface() const { return TypeKey(); }
			const std::string& Frame() const { return frame_; }
			// Throws BuildError if the children break the rules.
			void CheckChildren() const;

		protected:
			void OnConfigure() override;

		private:
			std::int64_t max_children_ = 0;
			std::set<std::string> accepts_;
			std::string frame_;
		};

		// One physical block represented as a branch of virtual nodes (spec 04 §3).
		class ComplexDevice : public DeviceNode {
		public:
			ComplexDevice() : DeviceNode(NodeKind::Complex) {}
			// Creates the virtual children. Called once by the tree builder, before explicit children are added.
			virtual void Expand(const DeviceRegistry& registry) = 0;

		protected:
			// The platform part of the type key: "webots" for "quadrotor.webots".
			std::string Platform() const;
			DeviceNode& AddVirtual(const DeviceRegistry& registry, const std::string& type_key, const std::string& name, Params params);
		};
	}
}
