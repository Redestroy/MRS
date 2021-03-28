#pragma once
#include "Device.hpp"
#include <deque>			//might need to implament seperatly
namespace MRS {
	namespace Device {
		class __declspec(dllexport) Agent {
		private:
		//sensor block
		//actuator block
		public:
			virtual void * Update() = 0;
			virtual void * DoAction() = 0;
		};
	}
}