#pragma once
namespace MRSbase {
	enum DeviceType {
		// Basic types: types do not contain child devices
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

	class Device {
	private:
		DeviceType type;
		long deviceId;
	protected:
		void SetDeviceType(DeviceType);
	public:
		DeviceType GetDeviceType();
	};
}