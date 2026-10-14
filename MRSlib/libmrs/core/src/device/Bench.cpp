#include "mrs/device/Bench.h"

#include "mrs/BuildError.h"

namespace MRS {
	namespace Device {
		const char* TimeoutModeName(TimeoutMode mode) {
			switch (mode) {
			case TimeoutMode::Default: return "default";
			case TimeoutMode::Safe: return "safe";
			case TimeoutMode::Continue: return "continue";
			}
			return "default";
		}

		std::optional<TimeoutMode> TimeoutModeFromName(std::string_view name) {
			if (name == "default") return TimeoutMode::Default;
			if (name == "safe") return TimeoutMode::Safe;
			if (name == "continue") return TimeoutMode::Continue;
			return std::nullopt;
		}

		bool IsMotionCode(const std::string& code) {
			static const std::set<std::string> codes = {"A_MOT", "A_TO", "A_LD", "A_HD", "A_PXY", "A_PZY", "A_VXY", "A_VZY", "A_GMB"};
			return codes.count(code) != 0;
		}

		BenchSettings BenchSettings::FromParams(const Params& params) {
			BenchSettings s;
			s.timeout = params.Num("bench_timeout", s.timeout);
			if (s.timeout < 0) throw BuildError("bench_timeout must not be negative");
			if (params.Has("bench_on_timeout")) {
				const auto mode = TimeoutModeFromName(params.Text("bench_on_timeout"));
				if (!mode) throw BuildError("bench_on_timeout must be default, safe or continue");
				s.on_timeout = *mode;
			}
			for (const auto& code : params.Texts("allow")) s.allow.insert(code);
			return s;
		}

		Bench::Bench(Robot robot, Port::IPlatform& platform, BenchSettings settings)
		    : robot_(std::move(robot)), platform_(&platform), settings_(std::move(settings)) {
			if (!robot_.bench) throw BuildError("a bench needs a robot built with the bench flag (spec 16 §8.1)");
			if (settings_.timeout < 0) throw BuildError("the bench timeout must not be negative");
		}

		Bench Bench::Build(std::string_view definition, const DeviceRegistry& registry, Port::IPlatform& platform,
		                   BuildOptions options, BenchSettings settings) {
			options.bench = true;
			return Bench(BuildRobot(definition, registry, platform, options), platform, std::move(settings));
		}

		DeviceNode* Bench::Node(const std::string& name) const {
			const auto matches = robot_.tree.Matches(name);
			if (matches.size() > 1) throw BuildError("node name " + name + " is ambiguous");
			if (matches.empty()) throw BuildError("no node " + name);
			return matches[0];
		}

		namespace {
			// Each value of the argument within the K_A's `min` and `max`, where given (§8.3).
			bool WithinLimits(const DeviceNode& node, const Action& action) {
				const auto info = FindAction(action.code);
				if (!info) return false;
				std::vector<double> values;
				if (IsIntegerLayout(info->layout)) {
					for (auto v : UnpackIntegers(info->layout, action.arg)) values.push_back(static_cast<double>(v));
				} else {
					values = UnpackReals(info->layout, action.arg);
				}
				for (const auto& c : node.Capabilities()) {
					if (c.kind != CapabilityKind::Action || c.code != action.code) continue;
					const auto min = c.limits.find("min"), max = c.limits.find("max");
					for (double v : values) {
						if (min != c.limits.end() && v < min->second) return false;
						if (max != c.limits.end() && v > max->second) return false;
					}
				}
				return true;
			}
		}

		ActionStatus Bench::Apply(const std::string& node, const Action& action, std::optional<double> timeout,
		                          std::optional<TimeoutMode> on_timeout) {
			if (timeout && *timeout < 0) return ActionStatus::REJECTED;
			Actuator* actuator = robot_.blocks.actuators.Route({node, action});
			if (!actuator) return ActionStatus::REJECTED;
			if (IsMotionCode(action.code) && !settings_.allow.count(action.code)) return ActionStatus::REJECTED;
			if (!WithinLimits(*actuator, action)) return ActionStatus::REJECTED;
			const double t = Time();
			const ActionStatus status = actuator->Apply(action, t);
			if (status == ActionStatus::REJECTED) return status;
			const double after = timeout ? *timeout : settings_.timeout;
			commands_[actuator] = {action, after > 0 ? t + after : -1.0, on_timeout ? *on_timeout : settings_.on_timeout, false};
			return status;
		}

		ActionStatus Bench::Apply(const std::string& node, const Protocol::Record& rec, std::optional<double> timeout,
		                          std::optional<TimeoutMode> on_timeout) {
			const auto info = rec.kind == 'A' ? FindAction(rec.code) : std::nullopt;
			if (!info) return ActionStatus::REJECTED;
			std::optional<std::uint64_t> arg;
			if (IsIntegerLayout(info->layout)) {
				std::vector<std::int64_t> v;
				for (const auto& f : rec.fields) v.push_back(f.i);
				arg = PackArgument(info->layout, v);
			} else {
				std::vector<double> v;
				for (const auto& f : rec.fields) v.push_back(f.n);
				arg = PackArgument(info->layout, v);
			}
			if (!arg) return ActionStatus::REJECTED;
			return Apply(node, Action{rec.code, *arg}, timeout, on_timeout);
		}

		std::vector<View> Bench::Sample(const std::string& name) {
			DeviceNode* node = Node(name);
			std::vector<View> out;
			auto it = sampled_.find(node);
			if (it != sampled_.end()) {
				out = std::move(it->second);
				sampled_.erase(it);
			}
			if (!node->Available()) return out;
			if (auto* sensor = dynamic_cast<Sensor*>(node)) sensor->Sample(Time(), out);
			else if (auto* storage = dynamic_cast<StorageDevice*>(node)) storage->Sample(Time(), out);
			return out;
		}

		std::vector<BenchNode> Bench::Nodes() const {
			std::vector<BenchNode> out;
			for (const DeviceNode* n : robot_.tree.Nodes()) {
				const auto* joint = dynamic_cast<const JointNode*>(n);
				out.push_back({n->Name(), NodeCode(n->Kind()), joint ? joint->Interface() : n->TypeKey(), n->Available()});
			}
			return out;
		}

		int Bench::OpenRaw(const std::string& provider, Port::PortType type, const std::string& address, const Params& params) {
			Port::IPortProvider* source = nullptr;
			if (provider == Port::kPlatformProvider) {
				source = platform_;
			} else {
				for (Port::IPortProvider* p : robot_.providers)
					if (auto* node = dynamic_cast<DeviceNode*>(p); node && node->Name() == provider) source = p;
			}
			if (!source) throw BuildError("no available provider " + provider);
			for (const auto& a : robot_.ports.assigned)
				if (a.provider == provider && a.type == type && a.address == address && a.exclusive)
					throw BuildError("port " + Port::PortTypeName(type) + " \"" + address + "\" is held by " + a.node);
			Port::PortAssignment a;
			a.node = "bench";
			a.requirement = "raw";
			a.type = type;
			a.address = address;
			a.exclusive = false;
			a.params = params;
			a.provider = provider;
			a.source = Port::AssignmentSource::Address;
			auto port = source->Open(a);
			if (!port) return -1;
			const int handle = next_handle_++;
			raw_[handle] = std::move(port);
			return handle;
		}

		Port::Port* Bench::Raw(int handle) const {
			auto it = raw_.find(handle);
			return it == raw_.end() ? nullptr : it->second.get();
		}

		void Bench::Stop() {
			const double t = Time();
			for (Actuator* a : robot_.blocks.actuators.Actuators()) a->Safe(t);
			commands_.clear();
		}

		bool Bench::Step() {
			const bool running = platform_->Step();
			const double t = Time();
			UpdateProviders(robot_, t);
			for (DeviceNode* node : robot_.blocks.sensors.Sources()) {
				auto& views = sampled_[node];
				if (auto* sensor = dynamic_cast<Sensor*>(node)) sensor->Sample(t, views);
				else if (auto* storage = dynamic_cast<StorageDevice*>(node)) storage->Sample(t, views);
			}
			for (auto& [actuator, command] : commands_) {
				if (command.continuing) {
					actuator->Apply(command.action, t);
					continue;
				}
				if (command.deadline < 0 || t + 1e-9 < command.deadline) continue;
				command.deadline = -1;
				switch (command.mode) {
				case TimeoutMode::Default: actuator->OnTimeout(t); break;
				case TimeoutMode::Safe: actuator->Safe(t); break;
				case TimeoutMode::Continue:
					command.continuing = true;
					actuator->Apply(command.action, t);
					break;
				}
			}
			robot_.blocks.actuators.Update(t);
			return running;
		}
	}
}
