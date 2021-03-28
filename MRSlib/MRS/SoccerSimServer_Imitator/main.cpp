#include <cstdio>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include  "udp_server.h"

#define DEF_SYNCHRONOUS

enum { max_length = 1024 };
int main(int argc, char* argv[])
{
    printf("hello from SoccerSimServer_Imitator!\n");
#ifdef DEF_SYNCHRONOUS
    try
    {
        if (argc != 3)
        {
            std::cerr << "Usage: blocking_udp_echo_client <host> <port>\n";
            printf("Arg echo: arg1 : %s; \n \t arg2: %s \n \t arg3: %s\n", argv[0], argv[1], argv[2]);
            return 1;
        }
        printf("Arg echo: arg1 : %s; \n \t arg2: %s \n \t arg3: %s\n", argv[0], argv[1], argv[2]);
        std::string IP_ADDRESS_str = argv[1];
        unsigned short port = (unsigned short)std::stoi(argv[2]);

        boost::asio::io_service io_context;
        boost::asio::ip::udp::endpoint endpoint_(boost::asio::ip::udp::v4(), port);
        boost::asio::ip::udp::socket s(io_context, endpoint_);

        for (;;) {
            boost::array<char, 1028> recv_buf;
            boost::asio::ip::udp::endpoint remote_endpoint;
            boost::system::error_code error;
            
            size_t reply_length = s.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);
            recv_buf.data()[1027] = '\0';
            std::cout << "I recieved this: ";
            std::cout.write(recv_buf.data(), reply_length);
            std::cout<< "\n";
            std::string message = "Reply from server imitator\n";
            if (error && error != boost::asio::error::message_size) throw boost::system::system_error(error);
            boost::system::error_code ignored_error;
            s.send_to(boost::asio::buffer(message), remote_endpoint, 0, ignored_error);
        }
    }
    catch (std::exception & e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }
#else
    try
    {
        if (argc != 3)
        {
            std::cerr << "Usage: blocking_udp_echo_client <host> <port>\n";
            printf("Arg echo: arg1 : %s; \n \t arg2: %s \n \t arg3: %s", argv[0], argv[1], argv[2]);
            return 1;
        }
        printf("Arg echo: arg1 : %s; \n \t arg2: %s \n \t arg3: %s", argv[0], argv[1], argv[2]);
        std::string IP_ADDRESS_str = argv[1];
        unsigned short port = (unsigned short)std::stoi(argv[2]);

        boost::asio::io_service io_context;
        boost::asio::ip::udp::endpoint endpoint_(boost::asio::ip::udp::v4(), port);
        udp_server s(io_context, endpoint_); //need a server class
        io_context.run();

        for (;;) {
            boost::array<char, 1> recv_buf;
            boost::asio::ip::udp::endpoint remote_endpoint;
            boost::system::error_code error;
            socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);
            std::string message = "Reply from server imitator\n";
            if (error && error != boost::asio::error::message_size) throw boost::system::system_error(error);
            boost::system::error_code ignored_error;
            socket.send_to(boost::asio::buffer(message), remote_endpoint, 0, ignored_error);
        }
    }
    catch (std::exception & e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }
#endif
    return 0;
}