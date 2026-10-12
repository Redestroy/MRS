#pragma once
// The ArduPilot autopilot as a provider joint (spec 16 §4.6), and the host platform it hangs from.
// The joint talks MAVLink through one link port of its own provider and offers the autopilot's
// MAVLink ports (spec 14 §3) to the nodes below it.
#include <memory>
#include <string>
#include <vector>

#include "mrs/device/DeviceNode.h"
#include "mrs/platform/ArduPilotPlatform.h"
#include "mrs/platform/Net.h"

namespace MRS {
	namespace Platform {
		// A byte link over a port's Send and Receive.
		class PortByteLink : public Net::IByteLink {
		public:
			explicit PortByteLink(Port::Port& port) : port_(port) {}
			bool Write(const std::uint8_t* data, std::size_t size) override;
			std::size_t Read(std::uint8_t* data, std::size_t capacity) override;
			void Wait(double) override {}
			bool Connected() const override { return true; }
			std::string Describe() const override;

		private:
			Port::Port& port_;
			std::string pending_;
		};

		// A port over a byte link: Send writes the bytes, Receive returns what has arrived.
		class ByteLinkPort : public ::MRS::Port::Port {
		public:
			ByteLinkPort(::MRS::Port::PortAssignment a, std::unique_ptr<Net::IByteLink> link)
			    : ::MRS::Port::Port(std::move(a)), link_(std::move(link)) {}
			bool Send(const std::string& data) override;
			std::optional<std::string> Receive() override;

		private:
			std::unique_ptr<Net::IByteLink> link_;
		};

		// autopilot.mavlink. Parameters: link (the link port's address, default "127.0.0.1:5760"), link_type
		// (TCP, UDP or UART; default TCP), system_id (245), target_system (0: the first heard), stream_hz (10),
		// period (s per tick, 0.02). Its ports are those of spec 14 §3, type MAVLINK.
		class AutopilotJoint : public Device::ProviderJoint {
		public:
			bool Start() override;
			std::vector<Port::PortInfo> Scan() override;
			std::unique_ptr<Port::Port> Open(const Port::PortAssignment& assignment) override;
			std::vector<Port::PortType> PortTypes() const override { return {Port::PortType::MAVLINK}; }
			// Pumps the link and sends heartbeats and stream requests.
			void Update(double t) override;

			// The MAVLink engine; null before Start.
			ArduPilotPlatform* Engine() const { return engine_.get(); }

		protected:
			void OnJointConfigure() override;

		private:
			std::unique_ptr<ArduPilotPlatform> engine_;
		};

		struct HostConfig {
			double period = 0.02;              // s per Step
			ClockMode clock = ClockMode::WALL;  // WALL or MANUAL
			double epoch = -1.0;               // unix s of mission time 0; below 0: the last UTC midnight
		};

		// The companion computer as the root platform (spec 16 §4.6). Ports: TCP "HOST:PORT" (client),
		// UDP "HOST:PORT" (send and read replies) or "in:PORT" (listen). It cannot list its ports, so its scan is
		// empty. Robots on it hang their autopilot from an autopilot.mavlink joint.
		class HostPlatform : public Port::IPlatform {
		public:
			explicit HostPlatform(HostConfig config = {});
			std::vector<Port::PortInfo> Scan() override { return {}; }
			std::unique_ptr<Port::Port> Open(const Port::PortAssignment& assignment) override;
			std::vector<Port::PortType> PortTypes() const override { return {Port::PortType::TCP, Port::PortType::UDP}; }
			double Time() const override { return time_; }
			bool Step() override;
			void Stop() { stop_ = true; }

		private:
			double Now() const;
			HostConfig c_;
			double time_ = 0.0, wall0_ = 0.0, epoch_offset_ = 0.0;
			bool stop_ = false;
		};
	}
}
