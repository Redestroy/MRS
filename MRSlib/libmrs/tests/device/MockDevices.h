#pragma once
// Generic devices for tree tests (spec 16): a motor that writes its speed to a port and an encoder that
// reads one. Registered as motor.mock and encoder.mock with the base keys.
#include <map>
#include <string>
#include <vector>

#include "MockPlatform.h"
#include "mrs/BuildError.h"
#include "mrs/device/DeviceRegistry.h"

namespace MRS {
	namespace Test {
		class MockMotor : public Device::Actuator {
		public:
			Device::ActionStatus Apply(const Device::Action& action, double) override {
				if (action.code != "A_MOT") return Device::ActionStatus::REJECTED;
				Port::Port* port = GetPort("motor");
				const double speed = Device::UnpackReals(Device::ArgLayout::F64, action.arg).at(0);
				if (!port || !port->Write({speed})) return Device::ActionStatus::FAILED;
				++applied;
				return Device::ActionStatus::DONE;
			}
			void Safe(double) override {
				++safes;
				if (Port::Port* port = GetPort("motor")) port->Write({0.0});
			}
			void OnTimeout(double t) override {
				++timeouts;
				Actuator::OnTimeout(t);
			}
			int applied = 0, safes = 0, timeouts = 0;

		protected:
			void OnConfigure() override {
				Declare(Device::Capability::Act("A_MOT", {{"max", GetParams().Num("max_speed", 100.0)}, {"min", -GetParams().Num("max_speed", 100.0)}}));
				Device::Params port;
				if (GetParams().Has("identity")) port.Add("identity", *GetParams().Get("identity"));
				const auto type = Port::PortType::FromName(GetParams().Text("port_type", "SIM"));
				if (!type) throw BuildError("bad port_type");
				DeclarePort("motor", *type, GetParams().Text("device", "any"), true, port);
			}
		};

		class MockEncoder : public Device::Sensor {
		public:
			void Sample(double t, std::vector<Device::View>& out) override {
				std::vector<double> v;
				Port::Port* port = GetPort("encoder");
				if (port && port->Read(v) && !v.empty()) out.push_back({"V_RNG", t, {v[0]}, {}, {}});
			}

		protected:
			void OnConfigure() override {
				Declare(Device::Capability::ViewOf("V_RNG"));
				DeclarePort("encoder", Port::PortType::SIM, GetParams().Text("device", "any"));
			}
		};

		// The devices on each mock bus, by bus name (parameter `bus` of bus.mock).
		inline std::map<std::string, MockPlatform>& Buses() {
			static std::map<std::string, MockPlatform> buses;
			return buses;
		}

		// A provider joint (spec 16 §4): its link is a UART port of its own provider, and it scans and opens
		// the ports of the mock bus named by `bus`, offering the types in `types` (default CAN).
		class MockBus : public Device::ProviderJoint {
		public:
			std::vector<Port::PortInfo> Scan() override { return Bus().Scan(); }
			std::unique_ptr<Port::Port> Open(const Port::PortAssignment& a) override { return Bus().Open(a); }
			std::vector<Port::PortType> PortTypes() const override { return types_; }
			void Update(double) override { ++updates; }
			bool Start() override {
				if (!GetPort("link")) return false;
				if (GetParams().Bool("fail", false)) {
					SetUnavailable("bus does not answer");
					return false;
				}
				return true;
			}
			int updates = 0;

		protected:
			void OnJointConfigure() override {
				DeclarePort("link", Port::PortType::UART, GetParams().Text("link", "any"));
				for (const auto& t : GetParams().Texts("types")) types_.push_back(*Port::PortType::FromName(t));
				if (types_.empty()) types_.push_back(Port::PortType::CAN);
			}

		private:
			MockPlatform& Bus() const { return Buses()[GetParams().Text("bus")]; }
			std::vector<Port::PortType> types_;
		};

		inline Device::DeviceRegistry MockRegistry() {
			Device::DeviceRegistry r;
			Device::RegisterBaseDevices(r);
			r.Register<MockMotor>("motor.mock", {}, {"acme:motor"});
			r.Register<MockEncoder>("encoder.mock", {}, {"acme:encoder"});
			r.Register<MockBus>("bus.mock", {}, {"acme:canhub"});
			return r;
		}
	}
}
