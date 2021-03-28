#pragma once
#include "Include/Device.hpp"
namespace MRS {
	namespace Device {
		class __declspec(dllexport) Device_p
		{
			Device device;
		public:
			Device_p();
			Device_p(Device*);
			~Device_p();
			Device* GetDevice();
			void SetDevice(Device *);
		};
	}
}

