#pragma once
// Port layer 0 (plan §4, spec 04 §6): typed connections to hardware or a simulator.
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mrs/device/Params.h"

namespace MRS {
	namespace Port {
		enum class PortType { GPIO, PWM, ADC, UART, I2C, SPI, CAN, UDP, TCP, MAVLINK, SIM };

		const char* PortTypeName(PortType type);
		std::optional<PortType> ParsePortType(std::string_view name);

		// A port a device node needs (spec 04 §6.1). `name` is local to the node.
		struct PortRequirement {
			std::string name;
			PortType type = PortType::SIM;
			std::string address = "any";
			bool exclusive = true;
			Device::Params params;  // rate_hz, baud, identity, ...
		};

		// What a port scan reports about one port.
		struct PortInfo {
			PortType type = PortType::SIM;
			std::string address;
			std::string identity;  // what the device on it says it is; empty if unknown
		};

		enum class AssignmentSource { PortMap, Address, Scan };

		// A requirement with the port it got (spec 04 §6.3). Also one P_A record of a port map.
		struct PortAssignment {
			std::string node;
			std::string requirement;
			PortType type = PortType::SIM;
			std::string address;
			bool exclusive = true;
			Device::Params params;
			AssignmentSource source = AssignmentSource::PortMap;
		};

		// An open port. Implementations override what their type supports; the rest fail.
		class Port {
		public:
			explicit Port(PortAssignment assignment) : assignment_(std::move(assignment)) {}
			virtual ~Port() = default;
			Port(const Port&) = delete;
			Port& operator=(const Port&) = delete;

			const PortAssignment& Assignment() const { return assignment_; }

			// Sampled values. Returns true and fills `values` only when a new sample is ready.
			virtual bool Read(std::vector<double>& values) {
				(void)values;
				return false;
			}
			// Output values (motor speed, LED state, PWM duty). Returns false if refused.
			virtual bool Write(const std::vector<double>& values) {
				(void)values;
				return false;
			}
			// Byte streams and packet links.
			virtual bool Send(const std::string& data) {
				(void)data;
				return false;
			}
			virtual std::optional<std::string> Receive() { return std::nullopt; }

		private:
			PortAssignment assignment_;
		};

		// The platform the robot runs on: Webots, a mock, or real hardware.
		class IPlatform {
		public:
			virtual ~IPlatform() = default;
			// The ports that are present, with identities where the platform can tell.
			virtual std::vector<PortInfo> Scan() = 0;
			// Opens an assigned port with its parameters; null if it cannot be opened.
			virtual std::unique_ptr<Port> Open(const PortAssignment& assignment) = 0;
			// Mission time, s.
			virtual double Time() const = 0;
			// Advances one control period. Returns false when the platform stops.
			virtual bool Step() = 0;
		};
	}
}
