#ifndef ZEROMQOBJECT_H
#define ZEROMQOBJECT_H

#include <zmq.hpp>
#include <iostream>
#include <string>
#include <memory>
#include <thread>

#include <ReloPush/BinaryString.h>

class zeromp_object{
public:
    zmq::context_t context;
    zmq::socket_t socket;


    zeromp_object()
    {
        context = zmq::context_t(1);
        socket = zmq::socket_t(context, zmq::socket_type::req);
    }
    void connect(std::string ip_str = "tcp://localhost:5555")
    {
        socket.connect(ip_str);
    }

    int send(std::string msg_str)
    {
        // Create a ZeroMQ message and copy the string data into it.
        zmq::message_t request(msg_str.size());
        memcpy(request.data(), msg_str.c_str(), msg_str.size());

        std::cout << "Sending message: " << msg_str << std::endl;

        // Send the message.
        socket.send(request, zmq::send_flags::none);
        // todo: handle exceptions
        return 0;
    }
    std::string wait_for_response()
    {
        // Wait for the reply from the server.
        zmq::message_t reply;
        socket.recv(reply, zmq::recv_flags::none);
        // Convert the reply to a std::string.
        std::string reply_str(static_cast<char*>(reply.data()), reply.size());
        std::cout << "Received reply: " << reply_str << std::endl;

        return reply_str;
    }


    std::string send_and_wait(std::string msg_str)
    {
        send(msg_str);
        return wait_for_response();
    }
};

////////////////// String <-> Binary ////////////////
std::string float2binarystr(float f_in)
{
    std::string message(reinterpret_cast<char*>(&f_in), sizeof(float));
    return message;
}

float binarystr2float(std::string str_in)
{
    float receivedValue;
    memcpy(&receivedValue, str_in.data(), sizeof(float));
    return receivedValue;
}

std::string bool2binarystr(bool b_in)
{
    std::string out_str;
    if(b_in)
        out_str="t";
    else
        out_str="f";

    return out_str;
}

bool binarystr2bool(std::string str_in)
{
    if(str_in=="t")
        return true;
    else if(str_in=="f")
        return false;
    else {
        //??????
    }
}

#endif // ZEROMQOBJECT_H
