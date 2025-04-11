#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <zmq.hpp>
#include <iostream>
#include <string>
#include <memory>

#include <BinaryString.h>
#include <batchInstanceParcer.hpp>

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

namespace ReloPush {

/*
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
    */

    class trajectory_elem
    {
    public:
        float x;
        float y;
        float yaw;
        float ref_vel; //reference velocity
        float time;
        bool is_pushing;

        trajectory_elem()
        {
           x=0;
           y=0;
           yaw=0;
           ref_vel=0;
           time=-1;
           is_pushing = false;
        }
        trajectory_elem(std::string& serialized_waypoint)
        {
            deserialize(serialized_waypoint);
        }


        trajectory_elem(float x_in, float y_in, float yaw_in, float ref_vel_in, float time_in, bool is_pushing_in)
            : x(x_in), y(y_in), yaw(yaw_in), ref_vel(ref_vel_in), time(time_in), is_pushing(is_pushing_in)
        {}

        std::string serialize()
        {
            std::string var_delim = ",,,";
            std::string temp_str=""; // string for one waypoint
            temp_str += float2binarystr(x);
            temp_str += var_delim;
            temp_str += float2binarystr(y);
            temp_str += var_delim;
            temp_str += float2binarystr(yaw);
            temp_str += var_delim;
            temp_str += float2binarystr(ref_vel);
            temp_str += var_delim;
            temp_str += float2binarystr(time);
            temp_str += var_delim;
            temp_str += bool2binarystr(is_pushing);

            return temp_str;
        }

        void deserialize(std::string& str_pose)
        {
            std::string header_delim = "!!!";
            std::string elem_delim = ";;;";
            std::string var_delim = ",,,";

            // split variables
            auto var_sp = split(str_pose,var_delim); // x y yaw vel time
            if(var_sp.size()==6)
            {
                float x_in = binarystr2float(var_sp[0]);
                float y_in = binarystr2float(var_sp[1]);
                float yaw_in = binarystr2float(var_sp[2]);
                float vel_in = binarystr2float(var_sp[3]);
                float time_in = binarystr2float(var_sp[4]);
                bool is_pushing_in = binarystr2bool(var_sp[5]);

                x=x_in; y=y_in; yaw=yaw_in; ref_vel=vel_in; time=time_in; is_pushing=is_pushing_in;
            }
            else
            {
                std::cout << "Cannot deserialize" << std::endl;
            }
        }

        // Print function for trajectory_elem
        void print() const {
            std::cout << "(x=" << x
                      << ", y=" << y
                      << ", yaw=" << yaw
                      << ", ref_vel=" << ref_vel
                      << ", time=" << time << ")";
        }

    };

    class trajectory
    {
    public:
        float time_zero=0;
        std::string header_delim = "!!!";
        std::string elem_delim = ";;;";
        std::string var_delim = ",,,";
        std::string header = "t"; // header for trajectory

        std::shared_ptr<std::vector<trajectory_elem>> trajectory_points;

        trajectory()
        {
            std::vector<trajectory_elem> traj(0);
            trajectory_points = std::make_shared<std::vector<trajectory_elem>>(traj);
        }

        trajectory(std::string& serialized_trajectory_str)
        {
            std::vector<trajectory_elem> traj(0);
            trajectory_points = std::make_shared<std::vector<trajectory_elem>>(traj);
            deserialize(serialized_trajectory_str);
        }


        void append_waypoint(trajectory_elem wpt)
        {
            trajectory_points->push_back(wpt);
        }

        void augment_trajectory(trajectory traj_in)
        {
            if(trajectory_points->size()>0)
            {
                float time_off = trajectory_points->back().time + 0.5; // todo: calculate time offset
                for(size_t n=0; n<traj_in.trajectory_points->size(); n++)
                {
                    auto temp = traj_in.trajectory_points->at(n);
                    temp.time += time_off; // apply time
                    append_waypoint(temp);
                }
            }
            else // current trajectory is empty
            {
                trajectory_points = traj_in.trajectory_points;
            }
        }

        std::string serialize()
        {
            // header!time_zero;x,y,yaw,vel,time,is_pushing;...;
            std::string out_str = header + header_delim + float2binarystr(time_zero) + elem_delim; // init with time_zero

            size_t traj_size = trajectory_points->size();
            for(size_t n=0; n<traj_size; n++)
            {
                std::string temp_str=""; // string for one waypoint
                temp_str += float2binarystr(trajectory_points->at(n).x);
                temp_str += var_delim;
                temp_str += float2binarystr(trajectory_points->at(n).y);
                temp_str += var_delim;
                temp_str += float2binarystr(trajectory_points->at(n).yaw);
                temp_str += var_delim;
                temp_str += float2binarystr(trajectory_points->at(n).ref_vel);
                temp_str += var_delim;
                temp_str += float2binarystr(trajectory_points->at(n).time);
                temp_str += var_delim;
                temp_str += bool2binarystr(trajectory_points->at(n).is_pushing);


                // append to the output string
                out_str += temp_str;
                if(n!=traj_size-1)
                    out_str += elem_delim;
            }

            return out_str;
        }

        void deserialize(std::string& str_traj)
        {
            // split header
            auto header_sp = split(str_traj,header_delim);
            if(header_sp[0] == "t") // trajectory
            {
                // split elements
                auto elem_sp = split(header_sp[1],elem_delim);
                // first elem is time_zero
                auto time_zero_str = elem_sp[0];
                time_zero = binarystr2float(time_zero_str);

                // parse trajectory
                size_t elem_size = elem_sp.size();
                for(size_t n=1; n<elem_size; n++)
                {
                    // split variables
                    auto var_sp = split(elem_sp[n],var_delim); // x y yaw vel time
                    float x_in = binarystr2float(var_sp[0]);
                    float y_in = binarystr2float(var_sp[1]);
                    float yaw_in = binarystr2float(var_sp[2]);
                    float vel_in = binarystr2float(var_sp[3]);
                    float time_in = binarystr2float(var_sp[4]);
                    bool is_pushing_in = binarystr2bool(var_sp[5]);

                    trajectory_elem temp_elem(x_in,y_in,yaw_in,vel_in,time_in,is_pushing_in);
                    append_waypoint(temp_elem);
                }
            }
            else
            {
                //unknown header
            }
        }

        // Print function for trajectory
        void print() const {
            std::cout << "Trajectory:" << std::endl;
            std::cout << "Time Zero: " << time_zero << std::endl;
            std::cout << "Waypoints:" << std::endl;
            for (size_t i = 0; i < trajectory_points->size(); ++i)
            {
                std::cout << "Waypoint " << i << ": ";
                trajectory_points->at(i).print();
                std::cout << std::endl;
            }
        }
    };
}


#endif // TRAJECTORY_HPP
