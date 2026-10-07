#pragma once
// UAV device classes (spec 04 §3). They talk only to Ports, so the same classes serve every
// platform; a registry key such as gnss.webots only adds that platform's default device names.
#include <array>
#include <map>
#include <string>
#include <vector>

#include "mrs/device/DeviceNode.h"
#include "mrs/device/DeviceRegistry.h"
#include "mrs/device/uav/FlightControlUnit.h"

namespace MRS {
	namespace Device {
		namespace Uav {
			// One rotor: A_MOT, angular speed rad/s, written to its port.
			class RotorMotor : public Actuator {
			public:
				ActionStatus Apply(const Action& action, double t) override;
				double Speed() const { return speed_; }

			protected:
				void OnConfigure() override;

			private:
				double speed_ = 0.0;
			};

			// Attitude (V_ATT) and body rates (V_RATE); V_ACC when an accelerometer is named.
			class Imu : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;

			protected:
				void OnConfigure() override;
			};

			// V_GEO (frame wgs84) or V_POS3 (frame local).
			class Gnss : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;

			protected:
				void OnConfigure() override;

			private:
				bool local_ = false;
			};

			// V_MAG: heading = atan2(n_y, n_x) of the body-frame north vector, wrapped to [0, 2π).
			class Compass : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;
				static double Heading(double north_x, double north_y);

			protected:
				void OnConfigure() override;
			};

			// V_BARO: altitude AMSL.
			class Barometer : public Sensor {
			public:
				void Sample(double t, std::vector<View>& out) override;

			protected:
				void OnConfigure() override;
			};

			// A_L: bit k of the mask sets LED k to the colour (0 is off).
			class LedArray : public Actuator {
			public:
				ActionStatus Apply(const Action& action, double t) override;
				std::size_t Count() const { return count_; }

			protected:
				void OnConfigure() override;

			private:
				std::size_t count_ = 0;
			};

			// Energy store, K_Q energy_wh, and V_BAT from its port (energy Wh, optional voltage V).
			class Battery : public StorageDevice {
			public:
				void Sample(double t, std::vector<View>& out) override;

			protected:
				void OnConfigure() override;

			private:
				double capacity_wh_ = 0.0;
				double voltage_ = 0.0;
			};

			// Broadcast radio: an emitter port and a receiver port.
			class Radio : public CommunicationDevice {
			public:
				bool Send(const std::string& message, const std::string& recipient) override;
				std::vector<std::string> Receive() override;

			protected:
				void OnConfigure() override;
			};

			// quadrotor.*: expands to m_fl, m_fr, m_rl, m_rr, fcu and imu (spec 04 §3).
			class Quadrotor : public ComplexDevice {
			public:
				void Expand(const DeviceRegistry& registry) override;
			};

			// head.default, fcu.default and the .webots keys of spec 04 §3.1.
			void RegisterUavDevices(DeviceRegistry& registry);
		}
	}
}
