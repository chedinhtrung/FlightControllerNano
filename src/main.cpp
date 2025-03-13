#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "kalman.h"
#include "debugger.h"
#include "utils.h"
#include "opticalflow.h"


Imu imu = Imu();
AttitudeKalman attitude_kf = AttitudeKalman();
OpticalFlow of = OpticalFlow();

void setup(){
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    delay(3000);
    imu.setup();
    delay(200);
}

unsigned long last_mainloop_update;

void loop(){
    if (micros()-last_mainloop_update < DT*1e3) {return;}
    ImuData data = imu.read();
    attitude_kf.predict(data.gyro);
    attitude_kf.update_roll_pitch(data.accel);
    ConvertedImuData euler = attitude_kf.read_euler();
    ConvertedImuData d;
    d.x = atanf(data.accel.y/data.accel.z)*180/M_PI;
    d.y = atanf(data.accel.x/sqrt(data.accel.y*data.accel.y + data.accel.z*data.accel.z))*180/M_PI;
    rad_to_deg(euler); 
    //debug("euler: ", euler);
}

void serialEvent3(){
    if (of.micolink_decode((uint8_t)Serial3.read())) {
        Serial.printf("h: %i, fx: %i, fy: %i, qf: %i", of.payload.distance, of.payload.flow_vel_x, 
                        of.payload.flow_vel_y, of.payload.flow_quality);
        Serial.println();
    };
}