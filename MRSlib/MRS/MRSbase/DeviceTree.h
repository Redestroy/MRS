
#include "Device.hpp"
#include "tree.h"
namespace MRSbase {
	class DeviceTree :
		public tree<Device>
	{
		DeviceTree();
		~DeviceTree();
		
		Device* GetDevice(DeviceType, long deviceId);

	};
}

