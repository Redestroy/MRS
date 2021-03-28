//#include "pch.h"
#include "Device.hpp"
#include "../Include/Device.hpp"

namespace MRS {
	namespace Device {
		int Device::device_count = 0;
		Device::Device() : tags()
		{
			deviceId = device_count;
			type = DeviceType::Null;
			device_count++;
			initialized = false;
		}

		Device::Device(DeviceType type) : Device()
		{
			this->type = type;
		}

		Device::~Device()
		{
			device_count--;
		}

		void Device::SetDeviceType(DeviceType type) {
			this->type = type;
		}

		DeviceType Device::GetDeviceType() {
			return type;
		}
		void Device::Init()
		{
			initialized = true;
		}

		void Device::DeInit()
		{
			initialized = false;
		}

		bool Device::IsInitialized()
		{
			return initialized;
		}

		void Device::AddTag(std::string tag, std::string value)
		{
			tags.emplace(tag, value);
		}

		std::string Device::GetTag(std::string tag)
		{
			if (tags.at(tag) != "") {
				return tags.at(tag);
			}
			return std::string();
		}
	}
}