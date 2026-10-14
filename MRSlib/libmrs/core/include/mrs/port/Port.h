#pragma once
// Port layer 0 (plan §4, spec 04 §6): typed connections to hardware or a simulator.
#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mrs/device/Params.h"

namespace MRS {
	namespace Port {
		// A port type is a name (spec 16 §4.5): upper-case letters and digits, starting with a letter, at most
		// 15 characters. The names of spec 04 §6.1 are built in; a provider may offer others (ROS2, ...).
		class PortType {
		public:
			constexpr PortType() : name_{'S', 'I', 'M'} {}

			static const PortType GPIO, PWM, ADC, UART, I2C, SPI, CAN, UDP, TCP, MAVLINK, SIM;

			// Nothing for a string that is not a valid port type name.
			static std::optional<PortType> FromName(std::string_view name);
			std::string Name() const { return std::string(name_.data()); }
			// True for the names of spec 04 §6.1.
			bool BuiltIn() const;

			friend bool operator==(const PortType& a, const PortType& b) { return a.name_ == b.name_; }
			friend bool operator!=(const PortType& a, const PortType& b) { return !(a == b); }
			friend bool operator<(const PortType& a, const PortType& b) { return a.name_ < b.name_; }

		private:
			// Only for names known to be valid: the built-in constants and FromName.
			constexpr explicit PortType(const char* name) : name_{} {
				for (std::size_t k = 0; name[k] != '\0' && k + 1 < name_.size(); ++k) name_[k] = name[k];
			}
			std::array<char, 16> name_;
		};

		inline const PortType PortType::GPIO{"GPIO"}, PortType::PWM{"PWM"}, PortType::ADC{"ADC"}, PortType::UART{"UART"},
		    PortType::I2C{"I2C"}, PortType::SPI{"SPI"}, PortType::CAN{"CAN"}, PortType::UDP{"UDP"}, PortType::TCP{"TCP"},
		    PortType::MAVLINK{"MAVLINK"}, PortType::SIM{"SIM"};

		inline std::string PortTypeName(const PortType& type) { return type.Name(); }
		inline std::optional<PortType> ParsePortType(std::string_view name) { return PortType::FromName(name); }

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
			PortInfo() = default;
			PortInfo(PortType t, std::string a, std::string id = {}, std::string desc = {})
			    : type(t), address(std::move(a)), identity(std::move(id)), descriptor(std::move(desc)) {}

			PortType type = PortType::SIM;
			std::string address;
			std::string identity;    // what the device on it says it is; empty if unknown (spec 16 §2.1)
			std::string provider;    // the provider that reported it; set by the port manager (spec 16 §4.3)
			std::string descriptor;  // the device's descriptor text, empty if none (spec 16 §2.2)
		};

		// The name of the root provider (spec 16 §4.1).
		inline constexpr const char* kPlatformProvider = "platform";

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
			std::string provider = kPlatformProvider;  // the provider that opens it (spec 16 §4.2)
			std::string identity;  // what the provider's scan reported at the address, if anything (spec 16 §7.3)
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

		// Something that scans and opens ports: the platform, or a joint for its subtree (spec 16 §4.1).
		class IPortProvider {
		public:
			virtual ~IPortProvider() = default;
			// The ports that are present, with identities where the provider can tell.
			virtual std::vector<PortInfo> Scan() = 0;
			// Opens an assigned port with its parameters; null if it cannot be opened.
			virtual std::unique_ptr<Port> Open(const PortAssignment& assignment) = 0;
			// The port types it offers. Empty: the built-in types (what every platform of version 0.1 offers).
			virtual std::vector<PortType> PortTypes() const { return {}; }
			// Per-tick work, before sensors are sampled. The platform steps in Step() instead.
			virtual void Update(double t) { (void)t; }

			bool Offers(const PortType& type) const;
		};

		// The platform the robot runs on: Webots, a mock, or real hardware. It is the root provider.
		class IPlatform : public IPortProvider {
		public:
			// Mission time, s.
			virtual double Time() const = 0;
			// Advances one control period. Returns false when the platform stops.
			virtual bool Step() = 0;
		};
	}
}
