#include <iostream>
#include <eigen3/Eigen/Core>
#include "ros/ros.h"
#include <string>
#include <std_msgs/Float64.h>
#include <ctime>
#include "distributed_mpc.h"

int main(int argc, char** argv)
{
    // initialise a ros node
    ros::init(argc, argv, "mpc4");

    int totalCars = 4;
    int carnumber = 4;
    float dist = 5;
    float vel = 0;

    // Create sub/pub object
    Optimise Optimise_Node1(carnumber, totalCars, dist, vel);


    std::cout << "Ending program" << std::endl;


    return 0;
}




