#pragma once
#include "Device.hpp"
#include <deque>			//might need to implament seperatly
namespace MRS {
	namespace Device {
		class
#ifdef BUILD_DLL
			__declspec(dllexport)
#endif // BUILD_DLL
			 DeviceBlock {
		private:
			std::deque<Device*> block;
			size_t size;
		public:
			DeviceBlock();
			~DeviceBlock();
		};
	}
}