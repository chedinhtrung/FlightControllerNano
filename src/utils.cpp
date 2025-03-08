#include "utils.h"

float deg_to_rad(float deg){
    return deg/180*M_PI;
}

float rad_to_deg(float rad){
    return rad*180/M_PI;
}

void rad_to_deg(ConvertedImuData& data) {
    data.x *= 180/M_PI;
    data.y *= 180/M_PI;
    data.z *= 180/M_PI;
}

void deg_to_rad(ConvertedImuData& data){
    data.x /= 180/M_PI;
    data.y /= 180/M_PI;
    data.z /= 180/M_PI;
}