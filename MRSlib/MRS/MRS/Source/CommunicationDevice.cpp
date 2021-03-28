#include "../Include/CommunicationDevice.hpp"
namespace MRS {
	namespace Device {
		CommunicationDevice::CommunicationDevice()
		{
		}

		CommunicationDevice::~CommunicationDevice()
		{
		}

		void CommunicationDevice::Init()
		{
			Device::Init();
		}

		void CommunicationDevice::DeInit()
		{
			Device::DeInit();
		}

		void CommunicationDevice::SetEmitter(Emitter* emitter)
		{
			this->emitter = emitter;
		}

		Emitter* CommunicationDevice::GetEmitter()
		{
			return emitter;
		}

		void CommunicationDevice::SetReceiver(Receiver* receiver)
		{
			this->receiver = receiver;
		}

		Receiver* CommunicationDevice::GetReceiver()
		{
			return receiver;
		}
	}
}