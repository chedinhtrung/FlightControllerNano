#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "kalman.h"
#include "debugger.h"

Imu imu = Imu();
AttitudeKalman attitude_kf = AttitudeKalman();

void setup(){
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    delay(3000);
    imu.setup();
}

void loop(){
    ImuData data = imu.read();
    attitude_kf.predict(data.gyro);
    attitude_kf.update(data.accel);
    ConvertedImuData euler = attitude_kf.read();
    ConvertedImuData d;
    d.x = atanf(data.accel.y/data.accel.z)*180/M_PI;
    d.y = atanf(data.accel.x/sqrt(data.accel.y*data.accel.y + data.accel.z*data.accel.z))*180/M_PI; 
    debug("euler: ", euler);
    delay(DT);
}