#include "UDPSensor.h"
namespace MRS {
	namespace Device {

		UDPSensor::UDPSensor() : UDPSensor(nullptr)
		{
		}

		UDPSensor::UDPSensor(UDPController* ptr)
		{
			udpController_ptr = ptr;
			//AddTag("ComType","UDPSensor");
		}

		UDPSensor::~UDPSensor()
		{
		}

		void UDPSensor::SetUDPController(UDPController* ptr)
		{
			udpController_ptr = ptr;
		}

		UDPController* UDPSensor::GetUDPController()
		{
			return udpController_ptr;
		}

		void UDPSensor::OnPacketReceived(std::string payload)
		{
			std::cout << "Packet: " << payload << "\n";
		}

		void UDPSensor::OnUpdate(long delta_time)
		{
			std::cout << "Updating UDP sensor \n";
		}
	}
}
