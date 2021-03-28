#pragma once
#include "Device.hpp"
#include <webots/Device.hpp>
#include <webots/Node.hpp>

namespace MRS {
	namespace Drivers {
		namespace Webots {
			class Device_d_Webots : public Device::Device{
			private:
				webots::Device* wb_device_ptr;
				std::string * name;
				std::string * model;
			public:
				Device_d_Webots();
				Device_d_Webots(webots::Device*);
				~Device_d_Webots();
				MRS::Device::DeviceType GetDeviceTypeFromNodeType(int);
			};
		}
	}
}