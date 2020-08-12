#pragma once
#ifndef COMMUNICATIONDEVICE
#define COMMUNICATIONDEVICE
class CommunicationDevice 
{
protected:
	int id;
public:
	CommunicationDevice();
	CommunicationDevice(int);
	~CommunicationDevice();
	virtual bool SwitchToMaster() = 0;
	virtual bool SwitchToSlave() = 0;
	virtual void FindNearbyDevices(const char *) = 0;
	virtual void ConnectTo(long) = 0;
	virtual void Disconnect() = 0;
	virtual void SendMessage(char *, int) = 0;
	virtual void WaitForReply(const char *) = 0;
	virtual int RequestDataCode(int) = 0;
};
#endif

