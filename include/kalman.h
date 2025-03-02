#ifndef KALMAN
#define KALMAN

#include <BasicLinearAlgebra.h>
#include "imu.h"
using namespace BLA;

/*
    Reference: Dan Simon Optimal State Estimations, p.409

    Q: Covariance of process noise 
    R: Covariance of measurement noise
    P: Covariance of estimation error

*/

class AttitudeKalman {
    public:
        BLA::Matrix<3,1,float> x = {0.0, 0.0, 0.0};
        BLA::Matrix<3,3,float> Q = {
            0.2f, 0.0f, 0.0f,
            0.0f, 0.2f, 0.0f,
            0.0f, 0.0f, 0.2f
        }; 
        BLA::Matrix<3,3,float> R = {
            0.05f, 0.0f, 0.0f,
            0.0f, 0.05f, 0.0f,
            0.0f, 0.0f, 0.05f
        };
        BLA::Matrix<3,3,float> P = {
            4.0f, 0.0f, 0.0f,
            0.0f, 4.0f, 0.0f,
            0.0f, 0.0f, 4.0f
        };
        BLA::Matrix<3,3,float> K;
        //BLA::Matrix<3,3,float> F;  F is unnecessary, since F = Identity
        //BLA::Matrix<3,3,float> L;  L = Identity (not sure, could be beneficial to model)
        BLA::Matrix<3,3,float> H = {
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        };
        //BLA::Matrix<3,3,float> M;      // M = Identity
        BLA::Matrix<3,1,float> h = {0, 0, 0};
        AttitudeKalman();
        void predict(ConvertedImuData gyros);
        void update(ConvertedImuData accels);
        ConvertedImuData read();
};

class PositionKalman {
    public:
        PositionKalman(BLA::Matrix<6,6> Q, BLA::Matrix<6,6> R);
};




#endif