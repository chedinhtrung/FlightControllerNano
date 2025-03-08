#ifndef UTILS
#define UTILS
#include "imu.h"

float deg_to_rad(float deg);
float rad_to_deg(float rad);

void deg_to_rad(ConvertedImuData& data);
void rad_to_deg(ConvertedImuData& data);

#endif