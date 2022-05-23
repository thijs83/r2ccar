
#include <iostream>
#include <eigen3/Eigen/Core>


class Vehicle
{
public:
    // Initialize members of the class
    int N;					// Time Horizon
    float t;				// Time step
    float Amax;				// Maximum acceleration
    float Amin;				// Minimum acceleration
    float Vmax;				// Maximum velocity
    float Q;				// Weight on velocity tracking (weight on all diagonal entries the same so one value used to multiply all entris from matrix)
    float R;				// Weight on input (weight on all diagonal entries the same so one value used to multiply all entris from matrix)
    Eigen::VectorXf C; //adist 		// C matrix used for calculating distance in time horizon
    Eigen::MatrixXf D;  //bdist		// D matrix used for calculating distance in time horizon
    Eigen::MatrixXf DT;				// D^T
    Eigen::MatrixXf E; //bvel		// E matrix used for calculating velocity in time horizon
    Eigen::MatrixXf ET;				// E^T
    Eigen::MatrixXf H;				// H matrix for calculating the gradient of the cost function

    Vehicle(){
    }

    // Constructor for the class
    void setup_vehicle(float timestep, int timeHorizon, float accMax, float accMin, float velMax, float Qw, float Rw)
    {
        t = timestep;
        N = timeHorizon;
        Amax = accMax;
        Amin = accMin;
        Vmax = velMax;
        Q = Qw;
        R = Rw;

        // Construct the system matrices for time horizon N
        C = Eigen::VectorXf::Ones(N,1);
        D = Eigen::MatrixXf::Zero(N,N);
        E = Eigen::MatrixXf::Zero(N,N);

        // Create temporary value for iteration and create C matrix
        
        

        // Create temporary values for iteration and create Bdist and Bvel matrices
        //float temp = t*t;
        float temp1 = t;
        float temp2 = t*t;
        int init = 1;
        for (int j=0; j<N; ++j) {
			C(j,0) = temp1;
            temp1 += t;
            E(init-1,j) = t;
            for (int i=init; i<N; ++i) {
                D(i,j) = temp2;
                temp2 += t*t;
                E(i,j) = t;
            }
            init += 1;
            temp2 = t*t;
        }

        //Precalculate transpose of Bvel
        ET = E.transpose();
        //Precalculate transpose of Bdist
        DT = D.transpose();

        // create H matrix
        Eigen::VectorXf tempVec = Eigen::VectorXf::Ones(N)*R;
        Eigen::MatrixXf matR = tempVec.asDiagonal();
        H = ((ET * E * Q) + matR );
    }

};
