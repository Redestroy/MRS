#pragma once
//#include "pch.h"
#include <map>
#include <string>
namespace MRS {
	namespace Device {
		enum class DeviceType {
			// Basic types: types do not contain child devices
			Null,
			SENSOR,			// Obtains data from environment 
			ACTUATOR,		// Excecutes actions
			POWER,			// Provides power to the system 
			MEMORY,			// Stores data
			LINK,			// Links 2 or more devices
			SENDER,			// Sends messeges to other devices
			RECIEVER,		// Recieves messages from other devices

			// Advanced types: contain child devices
			COMUNICATOR,	// Container for atleast one Sender and one reciever
			COMPLEX,
			HEAD
		};

		class
#ifdef BUILD_DLL
			__declspec(dllexport)
#endif // BUILD_DLL
			Device {
		private:
			static int device_count;
			DeviceType type;
			long deviceId;
			bool initialized;
			std::map<std::string, std::string> tags;
		protected:
			void SetDeviceType(DeviceType);
		public:
			Device();
			Device(DeviceType);
			~Device();
			DeviceType GetDeviceType();
			virtual void Init();
			virtual void DeInit();
			bool IsInitialized();
			void AddTag(std::string tag, std::string value);
			std::string GetTag(std::string tag);
		};
	}
}