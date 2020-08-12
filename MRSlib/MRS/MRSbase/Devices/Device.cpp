#include "Device.hpp"
namespace MRS {
	void Device::SetDeviceType(DeviceType type){
		this->type = type;
	}

	DeviceType Device::GetDeviceType() {
		return type;
	}
}