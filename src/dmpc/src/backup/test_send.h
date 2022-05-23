
#include <iostream>
#include <eigen3/Eigen/Core>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <typeinfo>
#include <chrono>
#include <exchange/MsgArray.h>
#include <eigen3/Eigen/Dense>
#include "mpc.h"
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

class Optimised : private MPC
{
    private:
        // Initialize the node handle
        ros::NodeHandle n;
        // Initialize the publisher
        ros::Publisher pub;
        // Initialize the subscriber
        ros::Subscriber sub;
        // Initialize the subscriber for when there is another adjacent car
        ros::Subscriber sub2;
        // Initialize the time horizon
        int N;
        // Create the message published by this node
        exchange::MsgArray msg;
        // Create the message which stores data when there are two adjacent cars
        exchange::MsgArray stored_msg;
        // Create the message which stores extra data when there are two adjacent cars
        // Used in the case when one of the messages is delayed and from one car two messages are received
        exchange::MsgArray extra_stored_msg;
        // Car number
        int carNum;


        std::chrono::_V2::system_clock::time_point t1 = high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point t2 = high_resolution_clock::now();






    public:

        // constructor and setup both subscriber and publisher and publish first message
        Optimise(int CarNumber, int totalCars, float IniDist, float IniVel):MPC()
        {
            //// Variables for the controller and the vehicle model car
            // Car number
            carNum = CarNumber;
            // Timestep
            float t = 0.1;
            // Time horizon
            N = 20;
            // Maximum and Minimum accelerations
            float accMax = 3;
            float accMin = -3;

            // Step size for updating the constrains
            float stepCon = 1;
            // Step size for updating the input
            float stepU = 0.1;
            // Weight for the difference between current velocity and reference velocity
            float Qw = 1;
            // Weight on the input
            float Rw = 1;
            // Distance between two adjacent cars
            float s = 0.2;
            // Reference velocity. For the leading car lower than the following cars, so followers can catch up.
            // If it is the first car set it to a lower velocity then if its a follower
            // The constrain is also set to this reference velocity
            float vR;
            float velMax;
            if (carNum == 1)
            {
                vR = 2;
                velMax = vR;
            }
            else {
                vR = 3;
                velMax = vR;
            }
            // queue size for the publisher and subscriber
            int queueSize = 5;
            // Initialize the MPC class, which first only sets up its subclass vehicle
            setup(t,N,accMax,accMin,velMax,Qw,Rw);
            // Setup the MPC class
            setup_mpc(s,IniDist,IniVel,stepCon,stepU,vR);
            // Precalculate values
            preOptim();

            //// Setup the publishers and subscribers
            // Naming setup for the publishers and subscribers
            if (carNum == 1) // Check if it is the first car
            {
                // Names for the subscriber and publisher
                std::string pubName = "carid1";
                std::string subName = "carid2";
                // Setup publisher node
                pub = n.advertise<exchange::MsgArray>(pubName, queueSize);
                // Setup subscriber node
                sub = n.subscribe(subName, queueSize, &Optimise::CallbackOneAdjacent,this, ros::TransportHints().unreliable().tcpNoDelay());

            } else if (carNum == totalCars) // Check if it is the last car
            {
                // Names for the subscriber and publisher
                std::string pubName = "carid"+std::to_string(carNum);
                std::string subName = "carid"+std::to_string(carNum-1);
                // Setup publisher node
                pub = n.advertise<exchange::MsgArray>(pubName, queueSize);
                // Setup subscriber node
                sub = n.subscribe(subName, queueSize, &Optimise::CallbackOneAdjacent,this, ros::TransportHints().unreliable().tcpNoDelay());

            } else // All cars in the middle
            {
                // Names for the subscriber and publisher
                std::string pubName = "carid"+std::to_string(carNum);
                std::string subName_front = "carid"+std::to_string(carNum-1);
                std::string subName_back = "carid"+std::to_string(carNum+1);
                // Setup publisher node
                pub = n.advertise<exchange::MsgArray>(pubName, queueSize);
                // Setup subscriber nodes
                sub = n.subscribe(subName_front, queueSize, &Optimise::CallbackTwoAdjacent,this, ros::TransportHints().unreliable().tcpNoDelay());
                sub2 = n.subscribe(subName_back, queueSize, &Optimise::CallbackTwoAdjacent,this, ros::TransportHints().unreliable().tcpNoDelay());
            }



            std::cout << "Subscriber and publisher started, waiting for ROS to set everything up" <<std::endl;
            std::cout << "Waiting 5 seconds" <<std::endl;
            // Wait 5 seconds
            ros::Rate r(0.2);
            r.sleep();

            std::cout << "Starting up continuous controller" <<std::endl;

            // Initialise storing messages
            // If the number is zero it means there is no data stored in the message
            stored_msg.number = 0;
            extra_stored_msg.number = 0;


            //// Create the first message and
            // Set the car number
            msg.number = carNum;
            // Set the iteration number
            msg.iter = 1;
            // Create data vector
            for (int i=0; i<N; ++i)
            {
                msg.data.push_back(return_xN(i));
            }
            // Publish message
            pub.publish(msg);

            std::cout << "Startup complete" << std::endl;

            // Start the subscriber spinner
            ros::spin();

        }

        // function for callback when there is only one adjacent car
        void CallbackOneAdjacent(const exchange::MsgArray& receive_msg)
        {
            // Store incoming data
            std::vector<float> data = receive_msg.data;
            Eigen::Map<Eigen::VectorXf> received(data.data(), N);

            //// Process incoming data
            // Check the number of the car and which optimization to run
            // If it is the leading car call minimize_front,
            // else it will be last car and call minimize_back
            if (carNum == 1)
            {
                minimize_front(received);
            } else {
                minimize_back(received);
            }


            //// create new message
            // Check if the maximum iteration number reached
            if (receive_msg.iter < 200)
            {
                // If this number is not yet reached, add one to the iteration number
                msg.iter = receive_msg.iter + 1;
            } else {
                // Reset the iteration number to zero
                msg.iter = 0;
                // Store the new position
                update_system();

                // Clock to check algorithm timer
                t2 = high_resolution_clock::now();
                duration<double, std::micro> ms_double = t2 - t1;
                std::cout << ms_double.count() << "us" << std::endl;
                t1 = high_resolution_clock::now();

                Eigen::Vector2f temp = return_x0();
                std::cout << "car1 dist:" << temp(0) << " vel: " << temp(1) << std::endl;
            }

            // Create data vector
            // Clear the data in the array for the use of the push back function
            msg.data.clear();
            for (int i=0; i<N; ++i)
            {
                msg.data.push_back(return_xN(i));
            }
            // Publish message
            pub.publish(msg);

        }

        // function for callback when there are two adjacent cars
        void CallbackTwoAdjacent(const exchange::MsgArray& receive_msg)
        {
	    //// First check if both messages are in
            // If previous message has same iteration number then we can start the optimization
            // else we have to store the data and wait for next message
            if ((stored_msg.iter == receive_msg.iter) && (stored_msg.number != 0))
            {
                // Store incoming data
                std::vector<float> data = receive_msg.data;
                Eigen::Map<Eigen::VectorXf> received1(data.data(), N);
                std::vector<float> data2 = stored_msg.data;
                Eigen::Map<Eigen::VectorXf> received2(data2.data(), N);


                // Check if a extra message was stored and if yes then push it to normal message
                // If no extra message then set the number to zero of the stored message
                if (extra_stored_msg.number !=0)
                {
                    stored_msg = extra_stored_msg;
                    extra_stored_msg.number = 0;
                } else
                {
                    stored_msg.number = 0;
                }

                //// Process incoming data
                if (receive_msg.number < carNum)
                {
                    minimize(received1,received2);
                } else
                {
                    minimize(received2,received1);
                }


                //// create new message
                // Check if the maximum iteration number reached
                if (receive_msg.iter < 200)
                {
                    // If this number is not yet reached, add one to the iteration number
                    msg.iter = receive_msg.iter + 1;
                } else {
                    // Reset the iteration number to zero
                    msg.iter = 0;
                    // Store the new position
                    update_system();

                    // Clock to check algorithm timer
                    t2 = high_resolution_clock::now();
                    duration<double, std::micro> ms_double = t2 - t1;
                    std::cout << ms_double.count() << "us" << std::endl;
                    t1 = high_resolution_clock::now();

                    Eigen::Vector2f temp = return_x0();
                    std::cout << "car dist:" << temp(0) << " vel: " << temp(1) << std::endl;
                }

                // Create data vector
                // Clear the data in the array for the use of the push back function
                msg.data.clear();
                for (int i=0; i<N; ++i)
                {
                    msg.data.push_back(return_xN(i));
                }
                // Publish message
                pub.publish(msg);

            } else if (stored_msg.number == 0)
            {
                // Still need to wait for next message so only store the data
                stored_msg = receive_msg;
            } else if (stored_msg.number == receive_msg.number)
            {
                // Already one message stored from this car and still waiting for other car, so it is stored in extra data
                extra_stored_msg = receive_msg;

            } else
            {
                std::cout << " Something wrong!!!!!!!!" << std::endl;
            }

        }

};

