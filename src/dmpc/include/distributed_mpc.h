
#include <iostream>
#include <eigen3/Eigen/Core>
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
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
        ros::Publisher pub_iter;				// Initialize the iteration publisher
        ros::Publisher pub_time;				// Initialize the optimisation time  publisher
        ros::Publisher pub_calctime;			// Initialize the calculation time publisher
        ros::Subscriber sub;					// Initialize the subscriber
        ros::Subscriber sub2;					// Initialize the subscriber for when there is another adjacent car
        ros::Subscriber sub_dist;				// Initialize the subscriber for distance measurement
        ros::Subscriber sub_start;				// Initialise the subscriber for starting the DMPC
        dmpc::MsgArray msg;					    // Create the message published by this node

        // Messages to be stored when two adjecent cars are present
        dmpc::MsgArray CarFront;
        dmpc::MsgArray CarRear;
        dmpc::MsgArray CarFrontExtra;
        dmpc::MsgArray CarRearExtra;

        int carNum;								// Car number
        int totalcar;                          // Total cars
        int N;									// Initialize the time horizon
        float last_dist0;						// Storing velocity data
        bool mpc;							    // If DMPC is running

        std::chrono::_V2::system_clock::time_point t1 = high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point t2 = high_resolution_clock::now();

        std::chrono::_V2::system_clock::time_point t1_calc = high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point t2_calc = high_resolution_clock::now();
        duration<double, std::milli> opti_calctime = 0ms;   // Calculate the optimisation time

        float dist_change;


    public:

        // constructor and setup both subscribers and publisher
        Optimise(int CarNumber, int totalCars, float IniDist, float IniVel):MPC()
        {
            //// Variables for the controller and the vehicle model car
            carNum = CarNumber;			// Car number
            totalcar = totalCars;
            float t = 0.1;				// Timestep
            N = 40;						// Time horizon
            float accMax = 3;			// Maximum and Minimum accelerations
            float accMin = -3;			// Minimum accelerations
            float stepCon = 1;			// Step size for updating the constrains
            float stepU = 0.1;			// Step size for updating the input
            float Qw = 1;				// Weight for the difference between current velocity and reference velocity
            float Rw = 1;				// Weight on the input
            float s = 0.25;				// safe distance between two adjacent cars
            last_dist0 = IniDist;		// Set first distance for startup
            mpc = false;                // Set controller to off

            // Reference velocity. For the leading car lower than the following cars, so followers can catch up.
            // If it is the first car set it to a lower velocity then if its a follower
            // The constrain is also set to this reference velocity
            float vR;
            float velMax;
            if (carNum == 1){
                vR = 1;
                velMax = vR;
            } else {
                vR = 1.5;
                velMax = vR;
            }
            int queueSize = 5;			// queue size for the publisher and subscriber

            setup(t,N,accMax,accMin,velMax,Qw,Rw);			// Initialize the MPC class, which first only sets up its subclass vehicle
            setup_mpc(s,IniDist,IniVel,stepCon,stepU,vR);	// Setup the MPC class
            preOptim();										// Precalculate values

            //// Setup the publishers and subscribers
            if (carNum == 1) 						// Check if it is the first car
            {
				std::string pubName = "carid1";
                std::string subName = "carid2";
                pub = n.advertise<dmpc::MsgArray>(pubName, queueSize);
                sub = n.subscribe(subName, queueSize, &Optimise::CallbackOneAdjacent,this, ros::TransportHints().unreliable().tcpNoDelay());
            } else if (carNum == totalCars) 		// Check if it is the last car
            {
                std::string pubName = "carid"+std::to_string(carNum);
                std::string subName = "carid"+std::to_string(carNum-1);
                pub = n.advertise<dmpc::MsgArray>(pubName, queueSize);
                sub = n.subscribe(subName, queueSize, &Optimise::CallbackOneAdjacent,this, ros::TransportHints().unreliable().tcpNoDelay());
            } else {										// All cars in the middle
                std::string pubName = "carid"+std::to_string(carNum);
                std::string subName_front = "carid"+std::to_string(carNum-1);
                std::string subName_back = "carid"+std::to_string(carNum+1);
                pub = n.advertise<dmpc::MsgArray>(pubName, queueSize);
                sub = n.subscribe(subName_front, queueSize, &Optimise::CallbackTwoAdjacent,this, ros::TransportHints().unreliable().tcpNoDelay());
                sub2 = n.subscribe(subName_back, queueSize, &Optimise::CallbackTwoAdjacent,this, ros::TransportHints().unreliable().tcpNoDelay());
            }

			// setup velocity reference publisher
			std::string pubNameVel = "vel_ref"+std::to_string(carNum);
			pub_vel = n.advertise<std_msgs::Float32>(pubNameVel, 1);

			// setup final iterations publisher
			std::string pubNameIter = "iterations"+std::to_string(carNum);
			pub_iter = n.advertise<std_msgs::Float32>(pubNameIter, 1);

			// setup optimisation time publisher
			std::string pubNameTime = "opti_time"+std::to_string(carNum);
			pub_time = n.advertise<std_msgs::Float32>(pubNameTime, 1);

			// setup calculation time publisher, only used when there are two cars
			if (totalcar == 2)
			{
				std::string pubNameCalcTime = "opti_calctime"+std::to_string(carNum);
				pub_calctime = n.advertise<std_msgs::Float32>(pubNameCalcTime, 1);
			}

			// setup start subscriber
			sub_start = n.subscribe("mpc_start", 1, &Optimise::CallbackStart, this);

			// setup distance subscriber for following cars
			if (carNum > 1)
			{
				std::string subDistName = "distance"+std::to_string(carNum);
				sub_dist = n.subscribe(subDistName, 1, &Optimise::CallbackDistance, this);
			}

            std::cout << "Waiting 5 seconds to be sure ros is correctly set up" <<std::endl;
            ros::Rate r(0.2);					// Wait 5 seconds
            r.sleep();


            // Initialise storing messages, if zero then the message is empty
            CarFront.number = 0;
            CarRear.number = 0;
            CarFrontExtra.number = 0;
            CarRearExtra.number = 0;

            //// Create message data
            msg.number = carNum;							// Set the car number
            msg.safe = s;									// Set the safe distance for this car
            msg.iter = 1;									// Set the iteration number
			msg.dist = dist0;								// Set initial distance for optimisation
			msg.data.clear();
			for (int i=0; i<N; ++i)							// Set data vector
			{
				msg.data.push_back(distChange(i));
			}


			std::cout << "Waiting for starting message for controller" <<std::endl;

			ros::spin();

        }



        // function callback for storing the new velocity
        void CallbackStart(const std_msgs::Bool& start)
        {
		std::cout<< " starting" << std::endl;
		if ((start.data) && (!mpc))
            {
                pub.publish(msg);								// Publish message
                t1 = high_resolution_clock::now();				// Start time to measure optimisation time
                mpc = true;
            }
        }




        // function for callback when there is distance measurement available
        void CallbackDistance(const std_msgs::Float32& receive_dist)
        {
			last_dist0 = receive_dist.data;
		}




        // Used to reset the optimisation
		void resetOptim(bool send) {

			vel0 = vel0 + t*u(0);                               // Update velocity

			if (carNum > 1)                                     // Only first car doesnt have distance update
			{
				// Uncommend the below if you want to update DMPC using external distance measurement
				dist0 = last_dist0;

				// Uncommend the below to use DMPC prediction for distance
				//dist0 = dist0 - t*vel0 + dist_change;
			}


			t2 = high_resolution_clock::now();                  // End time of optimisation
			duration<double, std::milli> opti_time = t2 - t1;   // Calculate the optimisation time
			t1 = high_resolution_clock::now();				    // Start time to measure optimisation time


			// Check the calculation time
            if (totalcar == 2)
            {
                t2_calc = high_resolution_clock::now();             // End calc timer
                opti_calctime += t2_calc - t1_calc;                 // Calculate the timing difference and add it to calc time
                t1_calc = high_resolution_clock::now();             // Restart calc timer
                std_msgs::Float32 msg_opticalc;
                msg_opticalc.data = opti_calctime.count();
                pub_calctime.publish(msg_opticalc);

                std::cout << "calculation time:" << opti_calctime.count() << "ms" << std::endl;
                opti_calctime = 0ms;                                    // set timer to zero
            }


			std_msgs::Float32 msg_vel;						    // Create velocity reference message
			msg_vel.data = vel0;					            // Store new velocity reference for next time step
			std_msgs::Float32 msg_iter;
			msg_iter.data = msg.iter;
			pub_iter.publish(msg_iter);                         // Publish iteration number for information purposes
			std_msgs::Float32 msg_optitime;
			msg_optitime.data = opti_time.count();
			pub_time.publish(msg_optitime);                     // Optimisation time in ms
			pub_vel.publish(msg_vel);                           // Publish velocity reference for low-level controller


			std::cout << "iter:" << msg.iter << " duration: " << opti_time.count() << "ms,  vel: " << vel0 << " dist: " << dist0 << std::endl;


			msg.iter = 0;									// Set the iteration number
			msg.dist = dist0;								// Set initial distance for optimisation
			preOptim();

			if (send)
			{
				msg.data.clear();										// Clear the data in the array for the use of the push back function
				for (int i=0; i<N; ++i)
				{
					msg.data.push_back(distChange(i));
				}
				// Check if the controller is on
				if (mpc)
                {
                    pub.publish(msg);										// Publish message
                    t2_calc = high_resolution_clock::now();             // End calc timer
                    opti_calctime += t2_calc - t1_calc;                           // Calculate the timing difference and add it to calc time
                }
			}
		}


        // function for callback when there is only one adjacent car
        void CallbackOneAdjacent(const dmpc::MsgArray& receive_msg)
        {
			if (totalcar == 2)
            {
                t1_calc = high_resolution_clock::now();             // Restart calc timer                                    // set timer to zero
            }

			// Check if time in other controller already passed the t seconds (iteration is then set to zero) and thus received 0
			if ( (receive_msg.iter == 0) && (msg.iter > 5) )
			{
				// Uncommend below if you want to use predictions for updating distance
				//dist_change = receive_msg.data[0];

				bool send = false;                                      // Set send to false since message is send after optimisation
				resetOptim(send);                                       // Reset optimiser without sending DMPC message
			}

            std::vector<float> data = receive_msg.data;                 // Convert data to be later used
            Eigen::Map<Eigen::VectorXf> received(data.data(), N);

            // Process incoming data
            if (carNum == 1)											// If it is the leading car call minimize_front,
            {
                received = received.array() + receive_msg.safe - receive_msg.dist;
                minimize_front(received);
            } else 														// else it will be last car and call minimize_back
            {
                minimize_back(received);
            }

            // Clock to check algorithm timer
            t2 = high_resolution_clock::now();
            duration<double, std::milli> ms_double = t2 - t1;

            if ( (msg.iter > 5) && (ms_double.count() > t*1000) )				// Check if the maximum iteration number reached
            {
                // Uncommend below if you want to use predictions for updating distance
                //dist_change = receive_msg.data[0];

                bool send = true;
                resetOptim(send);
            } else
            {
                msg.iter++;												// Increment iteration number
                msg.data.clear();										// Clear the data in the array for the use of the push back function
                for (int i=0; i<N; ++i)
                {
                    msg.data.push_back(distChange(i));
                }
                // Check if the controller is on
				if (mpc)
                {
                    pub.publish(msg);									// Publish message

                    if (totalcar == 2)
                    {
                        t2_calc = high_resolution_clock::now();             // End calc timer
                        opti_calctime += t2_calc - t1_calc;                 // Calculate the timing difference and add it to calc time
                    }

                }
                constrain_update();										// Constrain update for next iteration
            }
        }

        // function for callback when there are two adjacent cars
        void CallbackTwoAdjacent(const dmpc::MsgArray& receive_msg)
        {
			// First we check if it is front car message or rear and store the message
			if (receive_msg.number < msg.number)
            {
                if (CarFront.number == 0)
                {
                    CarFront = receive_msg;
                } else
                {
                    CarFrontExtra = receive_msg;
                }
            } else
            {
                if (CarRear.number == 0)
                {
                    CarRear = receive_msg;
                } else
                {
                    CarRearExtra = receive_msg;
                }
            }

			// If from both front and rear messages have arrived, do the optimisation
            if ((CarFront.number != 0) && (CarRear.number != 0))
            {

                // Clock to check algorithm timer
                t2 = high_resolution_clock::now();
                duration<double, std::milli> ms_double = t2 - t1;

                // Check if time in other controller already passed the t seconds (iteration is then set to zero) and thus received 0
                if ( ( (CarFront.iter == 0) || (CarRear.iter == 0) ) && (ms_double.count() > t*500) )
                {
                    dist_change = CarFront.data[0];
                    bool send = false;
                    resetOptim(send);
                }


                // Convert incoming data for process
                std::vector<float> data = CarFront.data;
                Eigen::Map<Eigen::VectorXf> frontt(data.data(), N);
                std::vector<float> data2 = CarRear.data;
                Eigen::Map<Eigen::VectorXf> rearr(data2.data(), N);


                // Proces incoming data
                rearr = rearr.array() + CarRear.safe - CarRear.dist;
                minimize(frontt,rearr);




                if ( (msg.iter > 5) && (ms_double.count() > t*1000) )				// Check if the maximum iteration number reached
                {
                    // Uncommend below if you want to use predictions for updating distance
                    //dist_change = CarFront.data[0];

                    bool send = true;
                    resetOptim(send);
                } else
                {
                    msg.iter++;												// Increment iteration number
                    msg.data.clear();										// Clear the data in the array for the use of the push back function
                    for (int i=0; i<N; ++i)
                    {
                        msg.data.push_back(distChange(i));
                    }
                    // Check if the controller is on
                    if (mpc)
                    {
                        pub.publish(msg);										// Publish message
                    }
                    constrain_update();										// Constrain update for next iteration
                }


                // Check if a extra message were stored, if yes then push it to normal storage
                // Change empty message number to 0
                if (CarFrontExtra.number !=0) {
                    CarFront = CarFrontExtra;
                    CarFrontExtra.number = 0;
                } else {
                    CarFront.number = 0;
                }
                if (CarRearExtra.number !=0) {
                    CarRear = CarRearExtra;
                    CarRearExtra.number = 0;
                } else {
                    CarRear.number = 0;
                }
            }
        }




};

