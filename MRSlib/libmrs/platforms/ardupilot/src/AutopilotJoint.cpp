#include "mrs/platform/AutopilotJoint.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "mrs/BuildError.h"

namespace MRS {
	namespace Platform {
		namespace P = ::MRS::Port;

		bool PortByteLink::Write(const std::uint8_t* data, std::size_t size) {
			return port_.Send(std::string(reinterpret_cast<const char*>(data), size));
		}

		std::size_t PortByteLink::Read(std::uint8_t* data, std::size_t capacity) {
			if (pending_.empty())
				if (auto bytes = port_.Receive()) pending_ = std::move(*bytes);
			const std::size_t n = std::min(capacity, pending_.size());
			std::memcpy(data, pending_.data(), n);
			pending_.erase(0, n);
			return n;
		}

		std::string PortByteLink::Describe() const {
			const auto& a = port_.Assignment();
			return P::PortTypeName(a.type) + " \"" + a.address + "\" via " + a.provider;
		}

		bool ByteLinkPort::Send(const std::string& data) {
			return link_->Write(reinterpret_cast<const std::uint8_t*>(data.data()), data.size());
		}

		std::optional<std::string> ByteLinkPort::Receive() {
			std::uint8_t buf[4096];
			const std::size_t n = link_->Read(buf, sizeof(buf));
			if (n == 0) return std::nullopt;
			return std::string(reinterpret_cast<const char*>(buf), n);
		}

		void AutopilotJoint::OnJointConfigure() {
			const auto type = P::PortType::FromName(GetParams().Text("link_type", "TCP"));
			if (!type) throw BuildError("link_type must be a port type name");
			DeclarePort("link", *type, GetParams().Text("link", "127.0.0.1:5760"));
		}

		bool AutopilotJoint::Start() {
			P::Port* link = GetPort("link");
			if (!link) return false;
			ArduPilotConfig c;
			c.clock = ClockMode::MANUAL;  // the robot's platform keeps the time
			c.system_id = static_cast<int>(GetParams().Int("system_id", 245));
			c.target_system = static_cast<int>(GetParams().Int("target_system", 0));
			c.stream_hz = GetParams().Num("stream_hz", 10.0);
			c.period = GetParams().Num("period", 0.02);
			engine_ = std::make_unique<ArduPilotPlatform>(c, std::make_unique<PortByteLink>(*link));
			return true;
		}

		std::vector<P::PortInfo> AutopilotJoint::Scan() { return engine_ ? engine_->Scan() : std::vector<P::PortInfo>{}; }

		std::unique_ptr<P::Port> AutopilotJoint::Open(const P::PortAssignment& assignment) {
			return engine_ ? engine_->Open(assignment) : nullptr;
		}

		void AutopilotJoint::Update(double) {
			if (engine_) engine_->Step();
		}

		namespace {
			double SteadySeconds() {
				return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
			}
			double UnixSeconds() {
				return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
			}
		}

		HostPlatform::HostPlatform(HostConfig config) : c_(config) {
			wall0_ = SteadySeconds();
			if (c_.clock == ClockMode::WALL) {
				const double unix_now = UnixSeconds();
				const double epoch = c_.epoch >= 0.0 ? c_.epoch : std::floor(unix_now / 86400.0) * 86400.0;
				epoch_offset_ = unix_now - epoch;
			}
			time_ = Now();
		}

		double HostPlatform::Now() const {
			return c_.clock == ClockMode::WALL ? epoch_offset_ + (SteadySeconds() - wall0_) : time_;
		}

		std::unique_ptr<P::Port> HostPlatform::Open(const P::PortAssignment& a) {
			std::string url;
			if (a.type == P::PortType::TCP) url = "tcp:" + a.address;
			else if (a.type == P::PortType::UDP) url = a.address.rfind("in:", 0) == 0 ? "udpin:" + a.address.substr(3) : "udp:" + a.address;
			else return nullptr;
			try {
				return std::make_unique<ByteLinkPort>(a, Net::OpenLink(url));
			} catch (const std::exception&) {
				return nullptr;
			}
		}

		bool HostPlatform::Step() {
			if (stop_) return false;
			if (c_.clock == ClockMode::WALL) {
				const double target = time_ + c_.period;
				const double now = Now();
				if (now < target) std::this_thread::sleep_for(std::chrono::duration<double>(target - now));
				time_ = std::max(target, Now() - c_.period);  // a slow tick catches up instead of bursting
			} else {
				time_ += c_.period;
			}
			return !stop_;
		}
	}
}
