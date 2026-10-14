#include "mrs/port/Port.h"

#include <algorithm>

namespace MRS {
	namespace Port {
		std::optional<PortType> PortType::FromName(std::string_view name) {
			if (name.empty() || name.size() > 15 || !(name[0] >= 'A' && name[0] <= 'Z')) return std::nullopt;
			for (char c : name)
				if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return std::nullopt;
			return PortType(std::string(name).c_str());
		}

		bool PortType::BuiltIn() const {
			for (const PortType* t : {&GPIO, &PWM, &ADC, &UART, &I2C, &SPI, &CAN, &UDP, &TCP, &MAVLINK, &SIM})
				if (*this == *t) return true;
			return false;
		}

		bool IPortProvider::Offers(const PortType& type) const {
			const auto types = PortTypes();
			if (types.empty()) return type.BuiltIn();
			return std::find(types.begin(), types.end(), type) != types.end();
		}
	}
}
