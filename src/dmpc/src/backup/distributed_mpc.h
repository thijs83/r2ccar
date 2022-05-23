
#include <iostream>
#include <eigen3/Eigen/Core>
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <typeinfo>
#include <chrono>
#include <dmpc/MsgArray.h>
#include <eigen3/Eigen/Dense>
#include "mpc.h"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

class Optimise : private MPC
{
    private:
        ros::NodeHandle n;						// Initialize the node handle
        ros::Publisher pub;						// Initialize the publisher
        ros::Publisher pub_vel;					// Initialize the velocity publisher
        ros::Subscriber sub;					// Initialize the subscriber
        ros::Subscriber sub2;					// Initialize the subscriber for when there is another adjacent car
        ros::Subscriber sub_vel;				// Initialize the subscriber for velocity measurement
        ros::Subscriber sub_dist;				// Initialize the subscriber for distance measurement
        dmpc::MsgArray msg;					// Create the message published by this node
        dmpc::MsgArray stored_msg;			// Create the message which stores data when there are two adjacent cars
        // Create the message which stores extra data when there are two adjacent cars
        // Used in the case when one of the messages is delayed and from one car two messages are received
        dmpc::MsgArray extra_stored_msg;
        
        int carNum;								// Car number
        int N;									// Initialize the time horizon
        float last_vel0;							// Storing velocity data

        std::chrono::_V2::system_clock::time_point t1 = high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point t2 = high_resolution_clock::now();


    public:

        // constructor and setup both subscribers and publisher
        Optimise(int CarNumber, int totalCars, float IniDist, float IniVel):MPC()
        {
            //// Variables for the controller and the vehicle model car
            carNum = CarNumber;			// Car number
            float t = 0.1;				// Timestep
            N = 20;						// Time horizon
            float accMax = 3;			// Maximum and Minimum accelerations
            float accMin = -3;			// Minimum accelerations
            float stepCon = 1;			// Step size for updating the constrains
            float stepU = 0.1;			// Step size for updating the input
            float Qw = 1;				// Weight for the difference between current velocity and reference velocity
            float Rw = 1;				// Weight on the input
            float s = 0.3;				// safe distance between two adjacent cars
			last_vel0 = IniVel;
            
            // Reference velocity. For the leading car lower than the following cars, so followers can catch up.
            // If it is the first car set it to a lower velocity then if its a follower
            // The constrain is also set to this reference velocity
            float vR;
            float velMax;
            if (carNum == 1){
                vR = 0.8;
                velMax = vR;
            } else {
                vR = 2;
                velMax = vR;
            }
            int queueSize = 5;			// queue size for the publisher and subscriber
            
            setup(t,N,accMax,accMin,velMax,Qw,Rw);			// Initialize the MPC class, which first only sets up its subclass vehicle
            setup_mpc(s,IniDist,IniVel,stepCon,stepU,vR);	// Setup the MPC class
            preOptim();										// Precalculate values

            //// Setup the publishers and subscribers
            if (carNum == 1){ 						// Check if it is the first car
                std::string pubName = "carid1";
                std::string subName = "carid2";
                pub = n.advertise<dmpc::MsgArray>(pubName, queueSize);
                sub = n.subscribe(subName, queueSize, &Optimise::CallbackOneAdjacent,this, ros::TransportHints().unreliable().tcpNoDelay());
            } else if (carNum == totalCars){ 		// Check if it is the last car
                std::string pubName = "carid"+std::to_string(carNum);
                std::string subName = "carid"+std::to_string(carNum-1);
                pub = n.advertise<dmpc::MsgArray>(pubName, queueSize);
                sub = n.subscribe(subName, queueSize, &Optimise::CallbackOneAdjacent,this, ros::TransportHints().unreliable().tcpNoDelay());
            } 

			// setup velocity reference publisher
			std::string pubNameVel = "vel_ref"+std::to_string(carNum);
			pub_vel = n.advertise<std_msgs::Float32>(pubNameVel, 1);
			
			// setup velocity subscriber
			std::string subVelName = "velocity"+std::to_string(carNum);
			sub_vel = n.subscribe(subVelName, 1, &Optimise::CallbackVelocity, this);
			
			// setup distance subscriber for following cars
			if (carNum > 1){
				//std::string subDistName = "distance"+std::to_string(carNum);
				sub_dist = n.subscribe("distance", 1, &Optimise::CallbackDistance, this);
			}

            std::cout << "Subscriber and publisher started, waiting for ROS to set everything up" <<std::endl;
            std::cout << "Waiting 5 seconds" <<std::endl;
            ros::Rate r(0.2);					// Wait 5 seconds
            r.sleep();
            std::cout << "Starting up continuous controller" <<std::endl;

            // Initialise storing messages
            stored_msg.number = 0;				// If the number is zero it means there is no data stored in the message
            extra_stored_msg.number = 0;		// If the number is zero it means there is no data stored in the message

            //// Create message data
            msg.number = carNum;				// Set the car number
            msg.safe = s;						// Set the safe distance for this car
            
            std::cout << "Startup complete" << std::endl;
            ros::spin();						// Start the subscriber spinner

        }
        
        // function for callback when there is distance measurement available
        void CallbackDistance(const std_msgs::Float32& receive_dist) {
			t1 = high_resolution_clock::now();

			dist0 = receive_dist.data;
			vel0 = last_vel0;
			preOptim();							// Pre optimise
			msg.dist = dist0;					// Set initial distance
			msg.iter = 1;						// Set the iteration number
			msg.data.clear();
			for (int i=0; i<N; ++i){			// Set data vector
				msg.data.push_back(distChange(i));
			}
			pub.publish(msg);					// Publish message
		}
        
        // function callback for storing the new velocity
        void CallbackVelocity(const std_msgs::Float32& receive_vel) {
			last_vel0 = receive_vel.data;
		}
        

        // function for callback when there is only one adjacent car
        void CallbackOneAdjacent(const dmpc::MsgArray& receive_msg)
        {
            // first car has no incoming distance data so it waits for follower to send data then sends extra msg
            if ((carNum == 1) && (receive_msg.iter == 1)){
				t1 = high_resolution_clock::now();
				vel0 = last_vel0;
				preOptim();
				msg.iter = 1;					// Set initial distance
				msg.dist = dist0;					// Set initial distance
				msg.data.clear();
				for (int i=0; i<N; ++i){			// Set data vector
					msg.data.push_back(distChange(i));
				}
				pub.publish(msg);					// Publish message
            }
            
            // Store incoming data
            std::vector<float> data = receive_msg.data;
            Eigen::Map<Eigen::VectorXf> received(data.data(), N);

            //// Process incoming data
            // Check the number of the car and which optimization to run
            // If it is the leading car call minimize_front,
            // else it will be last car and call minimize_back
            if (carNum == 1)
            {
                received = received.array() + receive_msg.safe - receive_msg.dist;
                minimize_front(received);
            } else {
                minimize_back(received);
            }


            //// create new message
            // Check if the maximum iteration number reached
            if (msg.iter < 100)
            {
                // If this number is not yet reached, add one to the iteration number
                msg.iter ++;
                
                std::cout << "msg iter" << msg.iter << " receive" << receive_msg.iter << std::endl;
                
                // Create data vector
				// Clear the data in the array for the use of the push back function
				msg.data.clear();
				for (int i=0; i<N; ++i){
					msg.data.push_back(distChange(i));
				}
				// Publish message
				pub.publish(msg);
				
				// Constrain update
				constrain_update();
                
            } else {
				msg.iter ++;
				msg.data.clear();
				for (int i=0; i<N; ++i){
					msg.data.push_back(distChange(i));
				}
				// Publish message
				pub.publish(msg);
				

                // Clock to check algorithm timer
                t2 = high_resolution_clock::now();
                duration<double, std::micro> ms_double = t2 - t1;
                std::cout << ms_double.count() << "us" << std::endl;
                

                //std::cout << "car input:" << return_u().transpose() << std::endl;
                
                std_msgs::Float32 msg_vel;
                msg_vel.data = vel0 + t*u(0);
                pub_vel.publish(msg_vel);
                
				vel0 = vel0 + t*u(0);
                //if (carNum == 1)
				//{
				//	dist0 = dist0;
				//} else {
				//	dist0 = dist0 + received(0) - distChange(0); 
				//}
                std::cout << "vel: " << vel0 << " dist: " << dist0 << std::endl;
            }

            

        }

        

};

