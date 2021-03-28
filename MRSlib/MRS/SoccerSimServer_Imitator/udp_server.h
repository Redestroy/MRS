#pragma once
#ifndef UDP_SERVER_CLASS_f_LIBBOOST
#define UDP_SERVER_CLASS_f_LIBBOOST

#include <boost/asio.hpp>
class udp_server
{
    /*
public:
    udp_server(boost::asio::io_service& io_service)
        : boost::asio::ip::udp::(io_service, boost::asio::ip::udp::endpoint(udp::v4(), 13))
    {
        start_receive();
    }

private:
    void start_receive()
    {
        socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            boost::bind(&udp_server::handle_receive, this,
                boost::asio::placeholders::error));
    }

    void handle_receive(const boost::system::error_code& error)
    {
        if (!error || error == boost::asio::error::message_size)
        {
            boost::shared_ptr<std::string> message(
                new std::string("this is string"));

            socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
                boost::asio::bind(&udp_server::handle_send, this, message));

            start_receive();
        }
    }

    void handle_send(boost::shared_ptr<std::string> message)
    {
    }
    
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint remote_endpoint_;
    boost::array<char, 1> recv_buffer_;*/
    
};

#endif // !1