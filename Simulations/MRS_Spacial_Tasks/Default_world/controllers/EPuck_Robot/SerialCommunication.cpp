#include "SerialCommunication.h"

int SerialCommunication::DEVICE_NUMBER = 0;

SerialCommunication::SerialCommunication() : CommunicationDevice(DEVICE_NUMBER)
{
	emiter = nullptr;
	reciever = nullptr;
	isMaster = false;
	DEVICE_NUMBER++;
}

SerialCommunication::SerialCommunication(webots::Robot * robot) : CommunicationDevice(DEVICE_NUMBER)
{
	emiter = robot->getEmitter("emitter");
	reciever = robot->getReceiver("receiver");
	TIME_STEP = 4;
	isMaster = false;
	DEVICE_NUMBER++;
	emiter->setChannel(3);
	reciever->enable(TIME_STEP);
	//printf("Type reciever: %s \n", reciever->);
	reciever->setChannel(3);
	packet = new std::string("");
	SetID(DEVICE_NUMBER);
}


SerialCommunication::~SerialCommunication()
{
	delete packet;
}

bool SerialCommunication::SwitchToMaster()
{
	isMaster = true;
	return isMaster;
}

bool SerialCommunication::SwitchToSlave()
{
	isMaster = false;
	return isMaster;
}

void SerialCommunication::FindNearbyDevices(const char * deviceList)
{
	// send discovery notice
	// for discovery time
	//		if(time >= i*delta) resend discovery notice
	//		if(queue>0) Add to device list
}

void SerialCommunication::ConnectTo(long id)
{
	// change method to int
	// if(isMaster)
	// brodcast pair request
	// wait for ack
	// if ack setChannel to id
	// send handshake
	// if ok return 1
	// else return -1
	// else return -1
	// else
	// setChannel to local ID
	// wait for handshake
	// if ok return 1
	// else return -1
}

void SerialCommunication::Disconnect()
{
	// sendNotice
	// if !isMaster wait for ack
	// disconnect
	//
}

void SerialCommunication::SendMessage(const char * message, int size)
{
	emiter->send((void *)message, size);
}

void SerialCommunication::SendMessage(char * message, int size)
{
	emiter->send((void *)message, size);
}

void SerialCommunication::WaitForReply(const char *)
{
	//not sure, might be obsolete
}

int SerialCommunication::RequestDataCode(int)
{
	//add later
	return 0;
}

int SerialCommunication::GetNumberOfPackets()
{
	return reciever->getQueueLength();
}

std::string * SerialCommunication::ReturnLastPacket()
{
	if (reciever->getQueueLength() > 0) {
		packet->clear();
		packet->append(std::string((char *)reciever->getData(), reciever->getDataSize()));
		reciever->nextPacket();
		return packet;
	}
	else return nullptr;
}

void SerialCommunication::Update()
{
	//also nothing to do here
}

void SerialCommunication::SwitchChannel(int ch)
{
	emiter->setChannel(ch);
	reciever->setChannel(ch);
	channel = ch;
}

int SerialCommunication::GetChannel()
{
	return channel;
}

void SerialCommunication::SetID(int idNum)
{
	id = idNum;
}

int SerialCommunication::GetID()
{
	return id;
}
