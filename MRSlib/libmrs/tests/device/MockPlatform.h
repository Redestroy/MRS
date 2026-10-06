#pragma once
// A platform for device tests: named ports with scripted samples, recorded writes and messages.
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mrs/port/Port.h"

namespace MRS {
	namespace Test {
		struct MockDevice {
			std::string identity;
			std::vector<double> sample;  // what Read returns
			bool fresh = false;          // Read succeeds once per Push
			std::vector<std::vector<double>> writes;
			std::vector<std::string> sent;
			std::deque<std::string> inbox;
			bool opens = true;
			int open_count = 0;
			Device::Params opened_with;

			void Push(std::vector<double> values) {
				sample = std::move(values);
				fresh = true;
			}
		};

		class MockPort : public ::MRS::Port::Port {
		public:
			MockPort(::MRS::Port::PortAssignment a, std::shared_ptr<MockDevice> d)
			    : ::MRS::Port::Port(std::move(a)), device_(std::move(d)) {}
			bool Read(std::vector<double>& values) override {
				if (!device_->fresh) return false;
				device_->fresh = false;
				values = device_->sample;
				return true;
			}
			bool Write(const std::vector<double>& values) override {
				device_->writes.push_back(values);
				return true;
			}
			bool Send(const std::string& data) override {
				device_->sent.push_back(data);
				return true;
			}
			std::optional<std::string> Receive() override {
				if (device_->inbox.empty()) return std::nullopt;
				std::string m = device_->inbox.front();
				device_->inbox.pop_front();
				return m;
			}

		private:
			std::shared_ptr<MockDevice> device_;
		};

		class MockPlatform : public Port::IPlatform {
		public:
			MockDevice& Add(Port::PortType type, const std::string& address, std::string identity = {}) {
				auto& d = devices_[{type, address}];
				d = std::make_shared<MockDevice>();
				d->identity = std::move(identity);
				return *d;
			}
			void Remove(Port::PortType type, const std::string& address) { devices_.erase({type, address}); }
			MockDevice& Get(const std::string& address, Port::PortType type = Port::PortType::SIM) { return *devices_.at({type, address}); }
			int scans = 0;

			// The Webots Mavic 2 Pro devices, plus an emitter and receiver in bodySlot.
			static MockPlatform Mavic() {
				MockPlatform p;
				for (const char* name : {"front left propeller", "front right propeller", "rear left propeller", "rear right propeller",
				                         "inertial unit", "gyro", "gps", "compass", "front left led", "front right led", "battery",
				                         "emitter", "receiver", "camera"})
					p.Add(Port::PortType::SIM, name, name);
				return p;
			}

			std::vector<Port::PortInfo> Scan() override {
				++scans;
				std::vector<Port::PortInfo> out;
				for (const auto& d : devices_) out.push_back({d.first.first, d.first.second, d.second->identity});
				return out;
			}
			std::unique_ptr<Port::Port> Open(const Port::PortAssignment& a) override {
				auto it = devices_.find({a.type, a.address});
				if (it == devices_.end() || !it->second->opens) return nullptr;
				++it->second->open_count;
				it->second->opened_with = a.params;
				return std::make_unique<MockPort>(a, it->second);
			}
			double Time() const override { return time_; }
			bool Step() override {
				time_ += 0.008;
				return true;
			}

		private:
			std::map<std::pair<Port::PortType, std::string>, std::shared_ptr<MockDevice>> devices_;
			double time_ = 0.0;
		};
	}
}
