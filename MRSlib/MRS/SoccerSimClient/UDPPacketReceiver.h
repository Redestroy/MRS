#pragma once
#include <string>
class UDPPacketReceiver
{
public:
	virtual void OnPacketReceived(std::string) = 0;
};

