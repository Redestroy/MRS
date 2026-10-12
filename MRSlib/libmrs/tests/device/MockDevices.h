#pragma once
// Generic devices for tree tests (spec 16): a motor that writes its speed to a port and an encoder that
// reads one. Registered as motor.mock and encoder.mock with the base keys.
#include <string>
#include <vector>

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
				return Device::ActionStatus::DONE;
			}

		protected:
			void OnConfigure() override {
				Declare(Device::Capability::Act("A_MOT", {{"max", GetParams().Num("max_speed", 100.0)}, {"min", -GetParams().Num("max_speed", 100.0)}}));
				Device::Params port;
				if (GetParams().Has("identity")) port.Add("identity", *GetParams().Get("identity"));
				DeclarePort("motor", Port::PortType::SIM, GetParams().Text("device", "any"), true, port);
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

		inline Device::DeviceRegistry MockRegistry() {
			Device::DeviceRegistry r;
			Device::RegisterBaseDevices(r);
			r.Register<MockMotor>("motor.mock", {}, {"acme:motor"});
			r.Register<MockEncoder>("encoder.mock", {}, {"acme:encoder"});
			return r;
		}
	}
}
