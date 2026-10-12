#pragma once
// Bench mode (spec 16 §8): a robot, or part of one, whose devices are commanded directly.
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "mrs/device/Robot.h"

namespace MRS {
	namespace Device {
		// What happens when an actuator's command timeout runs out (spec 16 §8.4).
		enum class TimeoutMode { Default, Safe, Continue };
		const char* TimeoutModeName(TimeoutMode mode);  // "default", "safe", "continue"
		std::optional<TimeoutMode> TimeoutModeFromName(std::string_view name);

		// The motion codes of spec 16 §8.3: rejected unless allowed.
		bool IsMotionCode(const std::string& code);

		struct BenchSettings {
			double timeout = 5.0;                         // s; 0: none
			TimeoutMode on_timeout = TimeoutMode::Default;
			std::set<std::string> allow;                  // motion codes the bench accepts

			// From parameters bench_timeout, bench_on_timeout and allow (one id per code). Throws BuildError
			// on a bad value.
			static BenchSettings FromParams(const Params& params);
		};

		// A node of the bench tree, for Nodes().
		struct BenchNode {
			std::string name;  // full name
			std::string kind;  // D_H, D_J, ...
			std::string type;  // type key, or interface for a joint
			bool available = true;
		};

		class Bench {
		public:
			// Takes a robot built with BuildOptions::bench. Throws BuildError otherwise.
			Bench(Robot robot, Port::IPlatform& platform, BenchSettings settings = {});
			// Builds the robot with the bench flag set.
			static Bench Build(std::string_view definition, const DeviceRegistry& registry, Port::IPlatform& platform,
			                   BuildOptions options = {}, BenchSettings settings = {});

			Bench(Bench&&) = default;
			Bench(const Bench&) = delete;
			Bench& operator=(const Bench&) = delete;

			// One action for the actuator `node` (or `any`, routed by its code). REJECTED for a node with no
			// route, a motion code that is not allowed, or a value outside a `min` or `max` limit of the
			// node's K_A. Starts the actuator's command timeout (§8.4).
			ActionStatus Apply(const std::string& node, const Action& action, std::optional<double> timeout = std::nullopt,
			                   std::optional<TimeoutMode> on_timeout = std::nullopt);
			// The same from an A record (a leaf action; A_MAP and A_FN are not bench commands).
			ActionStatus Apply(const std::string& node, const Protocol::Record& action, std::optional<double> timeout = std::nullopt,
			                   std::optional<TimeoutMode> on_timeout = std::nullopt);

			// The node's new views: those the last steps sampled, then what it has now.
			std::vector<View> Sample(const std::string& node);
			std::vector<BenchNode> Nodes() const;
			const std::vector<Fault>& Faults() const { return robot_.faults; }
			const SelfModel& Self() const { return robot_.self; }
			const std::vector<CheckFinding>& Check() const { return robot_.check; }

			// A port no node holds, for testing wiring (§8.2). `provider` is `platform` or a provider joint.
			// Returns a handle, or throws BuildError for an unknown provider or a port a node holds
			// exclusively. Returns -1 when the provider cannot open it.
			int OpenRaw(const std::string& provider, Port::PortType type, const std::string& address, const Params& params = {});
			Port::Port* Raw(int handle) const;
			void CloseRaw(int handle) { raw_.erase(handle); }

			// Every actuator to its safe state; command timeouts are cleared.
			void Stop();
			// One platform step: providers' Update, sensor sampling, actuators' Update, then the timeouts.
			// Returns the platform's Step().
			bool Step();
			double Time() const { return platform_->Time(); }

			const Robot& GetRobot() const { return robot_; }
			const BenchSettings& Settings() const { return settings_; }

		private:
			struct Command {
				Action action;
				double deadline = -1;  // platform time; negative: no timeout
				TimeoutMode mode = TimeoutMode::Default;
				bool continuing = false;
			};
			DeviceNode* Node(const std::string& name) const;

			Robot robot_;
			Port::IPlatform* platform_;
			BenchSettings settings_;
			std::map<Actuator*, Command> commands_;
			std::map<DeviceNode*, std::vector<View>> sampled_;
			std::map<int, std::unique_ptr<Port::Port>> raw_;
			int next_handle_ = 1;
		};
	}
}
