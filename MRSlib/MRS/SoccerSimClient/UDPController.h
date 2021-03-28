#pragma once
#include <boost/version.hpp>
#include "UDPPacketReceiver.h"
#include <boost/asio.hpp>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <string>
#include <MRS/Include/Controller.hpp>
#include <MRS/Include/ProtocolParser.hpp>
#include <chrono>


class UDPController :
	public MRS::Device::Controller
{
	private:
		boost::asio::io_context io_context;
		boost::asio::ip::udp::endpoint endpoint_;
		boost::asio::ip::udp::socket s;
		std::string current_packet;
		boost::system::error_code error;
		static std::chrono::steady_clock::duration DEFAULT_UPDATE_TIMEOUT;
		char buffer[4000];

		boost::signal<void (std::string)> PacketReceived;
        void run(std::chrono::steady_clock::duration timeout);

		static void handle_receive(
			const boost::system::error_code& error, std::size_t length,
			boost::system::error_code* out_error, std::size_t* out_length);
	public:
		UDPController();
		UDPController(std::string ip_address, int port);
		void AttatchSignal(UDPPacketReceiver *);
		virtual void SendCommand(std::string cmd_template, std::string args);
		std::string GetPacket();
		std::string GetPacket(std::chrono::steady_clock::duration timeout);
		void OnUpdate(long delta);
};

