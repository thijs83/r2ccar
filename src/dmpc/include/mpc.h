#include <iostream>
#include <eigen3/Eigen/Core>
#include "vehicle.h"
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

class MPC : public Vehicle
{
public:
    float muCon;
    float muU;
    float dist0;
    float vel0;
    Eigen::VectorXf u;
    Eigen::VectorXf distN;
    Eigen::VectorXf velN;
    Eigen::VectorXf lambAccMax;
    Eigen::VectorXf lambAccMin;
    Eigen::VectorXf lambVMax;
    Eigen::VectorXf lambDist_front;
    Eigen::VectorXf lambDist_back;
    Eigen::VectorXf vRef;
    Eigen::VectorXf f;
    Eigen::VectorXf grad;
    Eigen::VectorXf distChange;
    float safe_dist;


    MPC():Vehicle(){
    }

    // initialize the MPC class which only sets up the vehicle subclass
    void setup(float t, int N, float accMax, float accMin, float velMax, float Qw, float Rw)
    {
        //// Create initial vector for time horizon N
        // Input vector
        u = Eigen::VectorXf::Zero(N);
        // Lambda vectors setup
        lambAccMax = Eigen::VectorXf::Zero(N);
        lambAccMin = Eigen::VectorXf::Zero(N);
        lambVMax = Eigen::VectorXf::Zero(N);
        lambDist_front = Eigen::VectorXf::Zero(N);
        lambDist_back = Eigen::VectorXf::Zero(N);
        // Setup Reference velocity vector
        vRef = Eigen::VectorXf::Ones(N);
        // Setup gradient vector which is reused for every gradient calculation
        grad = Eigen::VectorXf::Zero(N);
        // Setup vector for storing change in distance in time horizon
		distChange = Eigen::VectorXf::Zero(N);
        // Setup the vehicle subclass
        setup_vehicle(t,N,accMax,accMin,velMax,Qw,Rw);
    }

    // Initialize the MPC class
    void setup_mpc(float s, float distance0, float velocity0, float stepCon, float stepU, float vR)
    {
        // Multiply the initialized velocity reference vector with the reference velocity value
        vRef = vRef*vR;
        // Gradient step size for the constrains
        muCon = stepCon;
        // Gradient step size for the input
        muU = stepU;
        // Safe distance between two vehicles
        safe_dist = s;
        // Store the initial state
        dist0 = distance0, 
        vel0 = velocity0;
    }


    void preOptim()
    {
        // Calculate the value for the vector f since it stays the same for one optimization step
        f = vel0 - vRef.array();
        f = (f.transpose() * E * Q).transpose();
        // Calculate for first iteration the change in distance in time horizon
        distChange = C*vel0 + D*u;
    }
    
    void constrain_update(){
		//// Update dual variables that don't need other vehicle information
        // Gradient of constrain for maximum acceleration and update lambda values
        grad = u.array() - Amax;
        lambAccMax +=  muCon * grad;
        // Gradient of constrain for minimum acceleration and update lambda values
        grad = -1*u.array() + Amin;
        lambAccMin +=  muCon * grad;
        // Gradient of constrain for maximum velocity and update lambda values
        grad = (E*u).array() + vel0 - Vmax;
        lambVMax +=  muCon * grad;
        
        // Remove all negative from local dual variables
        lambAccMax = (lambAccMax.array() < 0).select(0, lambAccMax);
        lambAccMin = (lambAccMin.array() < 0).select(0, lambAccMin);
        lambVMax = (lambVMax.array() < 0).select(0, lambVMax);
	}
	

    // Call this function if it is a car in the middle of the platoon
    void minimize(const Eigen::VectorXf& x_front, const Eigen::VectorXf& x_back)
    {
        //// First update the coupled constrains and then update the primal
        // Gradient of constrain for rear distance and update lambda values
        grad = x_back - distChange;
        lambDist_back +=  muCon * grad;
        // Gradient of constrain for front distance and update lambda values
        grad = (distChange - x_front).array() + safe_dist - dist0;
        lambDist_front +=  muCon * grad;

        // Remove all negative from local dual variables
        lambDist_front = (lambDist_front.array() < 0).select(0, lambDist_front);
        lambDist_back = (lambDist_back.array() < 0).select(0, lambDist_back);
        
        // Update the primal
        grad = H*u + lambAccMax - lambAccMin + ET*lambVMax + f + DT*(lambDist_front - lambDist_back);
        u -=  muU * grad;

        // Update distance vector for time horizon
        distChange = C*vel0 + D*u;
    }

    // Call this function if it is the leading car
    void minimize_front(const Eigen::VectorXf& x_back)
    {
        //// Update dual variables
        // Gradient of constrain for distance and update lambda values
        grad = x_back - distChange;
        lambDist_back +=  muCon * grad;

        // Remove all negative from local dual variables
        lambDist_back = (lambDist_back.array() < 0).select(0, lambDist_back);
        
        // Update the primal
        grad = H*u + lambAccMax - lambAccMin + ET*lambVMax + f - DT*lambDist_back;
        u -=  muU * grad;
        
        // Update distance vector for time horizon
        distChange = C*vel0 + D*u;
    }

    // Call this function if it is the last car
    void minimize_back(const Eigen::VectorXf& x_front)
    {
        //// Update dual variables
        // Gradient of constrain for distance and update lambda values
        grad = (distChange - x_front).array() + safe_dist - dist0;
        lambDist_front +=  muCon * grad;

        // Remove all negative from local dual variables
        lambDist_front = (lambDist_front.array() < 0).select(0, lambDist_front);
        
        // Update the primal
        grad = H*u + lambAccMax - lambAccMin + ET*lambVMax + f + DT*lambDist_front;
        u -=  muU * grad;

        // Update distance vector for time horizon
        distChange = C*vel0 + D*u;
    }

    // Function to return the distances in the time horizon N (xN)

    Eigen::VectorXf return_u()
    {
        return u;
    }

};
