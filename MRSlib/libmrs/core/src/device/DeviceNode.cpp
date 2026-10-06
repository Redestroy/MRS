#include "mrs/device/DeviceNode.h"

#include "mrs/BuildError.h"
#include "mrs/device/DeviceRegistry.h"

namespace MRS {
	namespace Device {
		namespace {
			struct KindCode {
				NodeKind kind;
				const char* code;
			};
			const KindCode kCodes[] = {{NodeKind::Head, "D_H"},     {NodeKind::Joint, "D_J"},    {NodeKind::Complex, "D_X"},
			                           {NodeKind::Sensor, "D_S"},   {NodeKind::Actuator, "D_A"}, {NodeKind::Communication, "D_C"},
			                           {NodeKind::Storage, "D_M"}};
		}

		const char* NodeCode(NodeKind kind) {
			for (const auto& k : kCodes)
				if (k.kind == kind) return k.code;
			return "D_S";
		}

		std::optional<NodeKind> NodeKindFromCode(std::string_view code) {
			for (const auto& k : kCodes)
				if (code == k.code) return k.kind;
			return std::nullopt;
		}

		void DeviceNode::Configure(std::string name, std::string type_key, Params params) {
			name_ = std::move(name);
			type_key_ = std::move(type_key);
			params_ = std::move(params);
			try {
				OnConfigure();
			} catch (const BuildError& e) {
				throw BuildError("node " + name_ + ": " + e.what());
			}
		}

		void DeviceNode::Declare(Capability capability) {
			capability.node = name_;
			capabilities_.push_back(std::move(capability));
		}

		void DeviceNode::DeclarePort(std::string name, Port::PortType type, std::string address, bool exclusive, Params params) {
			ports_.push_back({std::move(name), type, std::move(address), exclusive, std::move(params)});
		}

		Params DeviceNode::RateParams() const {
			Params p;
			if (params_.Has("rate_hz")) p.Add("rate_hz", NumValue(params_.Num("rate_hz", 0.0)));
			return p;
		}

		bool DeviceNode::HasCapability(CapabilityKind kind, const std::string& code) const {
			for (const auto& c : capabilities_)
				if (c.kind == kind && c.code == code) return true;
			return false;
		}

		void DeviceNode::OverrideCapability(const Capability& over) {
			for (auto& c : capabilities_) {
				if (!c.SameAs(over)) continue;
				for (const auto& limit : over.limits) c.limits[limit.first] = limit.second;
				if (over.kind == CapabilityKind::Capacity) c.capacity = over.capacity;
				return;
			}
			throw BuildError("node " + name_ + ": " + CapabilityCode(over.kind) + " " + over.code +
			                 " is not a capability of " + type_key_ + " (a K record only overrides limits)");
		}

		void DeviceNode::OverridePort(const Port::PortRequirement& over, const std::string& req) {
			Port::PortRequirement* target = nullptr;
			int same_type = 0;
			for (auto& r : ports_) {
				if (!req.empty()) {
					if (r.name == req) target = &r;
				} else if (r.type == over.type) {
					target = &r;
					++same_type;
				}
			}
			if (!target || same_type > 1)
				throw BuildError("node " + name_ + ": P_R " + Port::PortTypeName(over.type) +
				                 (req.empty() ? (same_type > 1 ? " matches several port requirements; name one with req"
				                                               : " matches no port requirement")
				                              : " names no port requirement " + req));
			if (target->type != over.type)
				throw BuildError("node " + name_ + ": port requirement " + target->name + " is " + Port::PortTypeName(target->type));
			// Address and exclusive flag are replaced; parameters replace those of the same key.
			target->address = over.address;
			target->exclusive = over.exclusive;
			Params kept;
			for (const auto& p : over.params.Items())
				if (p.key != "req") kept.Add(p.key, p.value);
			target->params.Merge(kept);
		}

		void DeviceNode::AttachPort(const std::string& requirement, std::unique_ptr<Port::Port> port) {
			for (auto& p : open_ports_)
				if (p.first == requirement) {
					p.second = std::move(port);
					return;
				}
			open_ports_.emplace_back(requirement, std::move(port));
		}

		Port::Port* DeviceNode::GetPort(const std::string& requirement) const {
			for (const auto& p : open_ports_)
				if (p.first == requirement) return p.second.get();
			return nullptr;
		}

		void DeviceNode::SetUnavailable(std::string reason) {
			if (!available_) return;
			available_ = false;
			fault_ = std::move(reason);
		}

		DeviceNode& DeviceNode::AddChild(std::unique_ptr<DeviceNode> child) {
			child->parent_ = this;
			children_.push_back(std::move(child));
			return *children_.back();
		}

		void HeadNode::OnConfigure() {
			robot_type_ = GetParams().Text("robot_type");
			roles_ = GetParams().Texts("role");
			id_ = GetParams().Int("id", 0);
			if (robot_type_.empty()) throw BuildError("head needs a robot_type");
			if (id_ < 1) throw BuildError("head needs an id >= 1");
		}

		void JointNode::OnConfigure() {
			static const std::set<std::string> interfaces = {"mech", "bus", "elec", "mech_bus", "mech_bus_elec"};
			if (!interfaces.count(Interface())) throw BuildError("unknown joint interface " + Interface());
			max_children_ = GetParams().Int("max_children", 0);
			for (const auto& code : GetParams().Texts("accepts")) {
				if (!NodeKindFromCode(code) || code == "D_H") throw BuildError("accepts " + code + " is not a child node code");
				accepts_.insert(code);
			}
			frame_ = GetParams().Text("frame");
		}

		void JointNode::CheckChildren() const {
			if (max_children_ > 0 && static_cast<std::int64_t>(Children().size()) > max_children_)
				throw BuildError("joint " + Name() + " has " + std::to_string(Children().size()) + " children, max_children is " +
				                 std::to_string(max_children_));
			if (accepts_.empty()) return;
			for (const auto& child : Children())
				if (!accepts_.count(NodeCode(child->Kind())))
					throw BuildError("joint " + Name() + " does not accept " + NodeCode(child->Kind()) + " " + child->Name());
		}

		std::string ComplexDevice::Platform() const {
			const auto dot = TypeKey().rfind('.');
			return dot == std::string::npos ? std::string("default") : TypeKey().substr(dot + 1);
		}

		DeviceNode& ComplexDevice::AddVirtual(const DeviceRegistry& registry, const std::string& type_key, const std::string& name,
		                                      Params params) {
			auto node = registry.Create(type_key, name, params);
			node->SetVirtual(true);
			return AddChild(std::move(node));
		}
	}
}
