#include "UDPController.h"
#include <iostream>
#define MAX_LENGTH 1024

std::chrono::steady_clock::duration UDPController::DEFAULT_UPDATE_TIMEOUT = std::chrono::milliseconds(60);

void UDPController::run(std::chrono::steady_clock::duration timeout)
{
		// Restart the io_context, as it may have been left in the "stopped" state
		// by a previous operation.
		io_context.restart();

		// Block until the asynchronous operation has completed, or timed out. If
		// the pending asynchronous operation is a composed operation, the deadline
		// applies to the entire operation, rather than individual operations on
		// the socket.
		io_context.run_for(timeout);

		// If the asynchronous operation completed successfully then the io_context
		// would have been stopped due to running out of work. If it was not
		// stopped, then the io_context::run_for call must have timed out.
		if (!io_context.stopped())
		{
			// Cancel the outstanding asynchronous operation.
			s.cancel();

			// Run the io_context again until the operation completes.
			io_context.run();
		}
	
}

void UDPController::handle_receive(const boost::system::error_code& error, std::size_t length, boost::system::error_code* out_error, std::size_t* out_length)
{
	*out_error = error;
	*out_length = length;
}

UDPController::UDPController() : UDPController("127.0.0.1", 6000)
{
}

UDPController::UDPController(std::string ip_address, int port) : io_context(), endpoint_(boost::asio::ip::address::from_string(ip_address), port), s(io_context)
{
	current_packet = "";
	try {
		s.open(boost::asio::ip::udp::v4());
	}
	catch (...) {
		std::cout << "Could not open udp port\n";
	}
	//Controller::SetProtocol(MRS::Device::PROTOCOL_TYPE::SIMPLE_MRS);
}

void UDPController::AttatchSignal(UDPPacketReceiver* sensor)
{
	PacketReceived.connect(bind(&UDPPacketReceiver::OnPacketReceived, sensor, _1));
}

void UDPController::SendCommand(std::string cmd_template, std::string args)
{
	char packet[MAX_LENGTH];
	std::cout << "Command input: " << cmd_template << " ; " << args << "\n";
	std::cout << "Command generated:" << MRS::Device::ProtocolParser::ParseArgsIntoString(MRS::Device::PROTOCOL_TYPE::SIMPLE_MRS, cmd_template, args) << "\n";
	strncpy(packet, MRS::Device::ProtocolParser::ParseArgsIntoString(MRS::Device::PROTOCOL_TYPE::SIMPLE_MRS, cmd_template, args).c_str(), MAX_LENGTH);
	std::cout << "Command sent:"<< packet <<"\n";
	size_t length = std::strlen(packet);
	if (s.is_open()) {
		s.send_to(boost::asio::buffer(packet, length), endpoint_);
	}
}

std::string UDPController::GetPacket()
{
	boost::asio::ip::udp::endpoint sender_endpoint;
	char reply[MAX_LENGTH];
	int reply_length = 0;
	std::cout << "Waiting for packet:\n";
	if (s.is_open()) {
		reply_length = s.receive_from(boost::asio::buffer(reply, MAX_LENGTH), sender_endpoint);
	}
	reply[reply_length] = '\0';
	return std::string(reply);
}

std::string UDPController::GetPacket(std::chrono::steady_clock::duration timeout)
{
	std::size_t length = 0;
	buffer[0] = '\0';
	std::cout << "Waiting for packet2:\n";
	s.async_receive(boost::asio::buffer(buffer),
		std::bind(&UDPController::handle_receive, std::placeholders::_1, std::placeholders::_2, &error, &length));

	// Run the operation until it completes, or until the timeout.
	run(timeout);
	buffer[length] = '\0';
	std::string packet = buffer;
	std::cout << "Response: "<< packet <<"\n";
	return packet;
}

void UDPController::OnUpdate(long delta)
{
	current_packet.clear();
	current_packet = GetPacket(DEFAULT_UPDATE_TIMEOUT);

	PacketReceived(current_packet);
}
