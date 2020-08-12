#pragma once
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include "CommunicationDevice.h"
class SerialCommunication : public CommunicationDevice
{
	webots::Emitter * emiter;
	webots::Receiver * reciever;
	bool isMaster;
	int channel;
	int TIME_STEP;
	static int DEVICE_NUMBER;
	std::string * packet;
public:
	SerialCommunication();
	SerialCommunication(webots::Robot *);
	~SerialCommunication();
	bool SwitchToMaster();
	bool SwitchToSlave();
	void FindNearbyDevices(const char *);
	void ConnectTo(long id);
	void Disconnect();
	void SendMessage(char *, int);
	void SendMessage(const char *, int);
	void WaitForReply(const char *);
	int RequestDataCode(int);
	int GetNumberOfPackets();
	std::string * ReturnLastPacket();
	//void OnTaskRecieved(TaskList *);
	void Update();

	void SwitchChannel(int);
	int GetChannel();

	void SetID(int);
	int GetID();
};

