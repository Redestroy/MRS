#include "mrs/port/Port.h"

namespace MRS {
	namespace Port {
		namespace {
			struct Name {
				PortType type;
				const char* name;
			};
			const Name kNames[] = {{PortType::GPIO, "GPIO"}, {PortType::PWM, "PWM"},         {PortType::ADC, "ADC"},
			                       {PortType::UART, "UART"}, {PortType::I2C, "I2C"},         {PortType::SPI, "SPI"},
			                       {PortType::CAN, "CAN"},   {PortType::UDP, "UDP"},         {PortType::TCP, "TCP"},
			                       {PortType::MAVLINK, "MAVLINK"}, {PortType::SIM, "SIM"}};
		}

		const char* PortTypeName(PortType type) {
			for (const auto& n : kNames)
				if (n.type == type) return n.name;
			return "SIM";
		}

		std::optional<PortType> ParsePortType(std::string_view name) {
			for (const auto& n : kNames)
				if (name == n.name) return n.type;
			return std::nullopt;
		}
	}
}
