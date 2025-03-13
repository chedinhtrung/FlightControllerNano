#include "kalman.h"
#include "config.h"
#include "debugger.h"

/*
    Reference: Dan Simon Optimal State Estimations, p.409
*/

AttitudeKalman::AttitudeKalman(){
}


void AttitudeKalman::predict(ConvertedImuData gyros){
    float sin_phi = sinf(x(0,0));
    float cos_phi = cosf(x(0,0));
    float sin_theta = sinf(x(1,0));
    float cos_theta = cosf(x(1,0));
    if (abs(cos_theta) < 0.01){ cos_theta = 0.01/cos_theta*abs(cos_theta);}    // Prevent singularity
    float tan_theta = sin_theta/cos_theta;


    // Equation 13.47
    
    P += Q;   // F = Identity, L = Identity

    x(0,0) += (gyros.x + gyros.y*sin_phi*tan_theta + gyros.z*cos_phi*tan_theta)*DT*1e-3;
    x(1,0) += (          gyros.y*cos_phi           + gyros.z*-sin_phi         )*DT*1e-3;
    x(2,0) += (          gyros.y*sin_phi/cos_theta + gyros.z*cos_phi/cos_theta)*DT*1e-3;

    
    // Equation 13.48
    // Recalculate trigs (not sure if necessary)
    /*
    sin_phi = sinf(deg_to_rad(x(0,0)));
    cos_phi = cosf(deg_to_rad(x(0,0)));
    sin_theta = sinf(deg_to_rad(x(1,0)));
    cos_theta = cosf(deg_to_rad(x(1,0)));
    tan_theta = sin_theta/cos_theta;
    */

    H(0,1) =                    cos_theta;
    H(1,0) = -cos_theta*cos_phi;
    H(1,1) =                    sin_theta*sin_phi;
    H(2,0) = cos_theta*sin_phi;
    H(2,1) =                    sin_theta*cos_phi;

    // Calculate h for equation 13.49
    h(0,0) = sin_theta;
    h(1,0) = -cos_theta*sin_phi;
    h(2,0) = -cos_theta*cos_phi;
}

void AttitudeKalman::update_roll_pitch(ConvertedImuData accels){
    float a_norm = abs(accels.x*accels.x + accels.y*accels.y + accels.z*accels.z - 1.0);
    float meas_covar = a_norm + 0.004;
    R(0,0) = meas_covar;
    R(1,1) = meas_covar;
    R(2,2) = meas_covar;

    BLA::Matrix<3,1,float> y;
    y(0,0) = accels.x;
    y(1,0) = accels.y;
    y(2,0) = accels.z;

    BLA::Matrix<3, 3, float> I = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
    // Equation 13.49
    BLA::Matrix<3,3,float> I_KH = (I - K * H);
    K = P * ~H * BLA::Inverse(H * P * ~H + R);
    x += K * (y - h);
    P = I_KH*P*(~I_KH) + K*R*~K;        // Follows Eq. 5.19 p.129, remark: This form is more stable. Empirically confirmed to be more stable
}

ConvertedImuData AttitudeKalman::read_euler(){
    ConvertedImuData data;
    data.x = x(0,0);
    data.y = x(1,0);
    data.z = x(2,0);
    return data;
}
