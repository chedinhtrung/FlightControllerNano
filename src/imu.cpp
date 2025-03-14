#include "imu.h"
#include "Wire.h"
#include <Arduino.h>
#include "debugger.h"
#include "utils.h"

void Imu::setup(){
    // Manual accel ffset values
    offset.accel.z = -0.07;
    offset.accel.y = 0.02;
    offset.accel.x = -0.015;
    // Wake up imu
    Wire.beginTransmission(IMUADDR);
    Wire.write(0x6B); 
    Wire.write(0x03);                   // Set PLL clock to use Z Axis Gyro as reference. also resets imu
    Wire.endTransmission();
    delay(50);

    // Configure Gyroscope
    Wire.beginTransmission(IMUADDR);         
    Wire.write(0x1A);                        // Register 1A for Low Pass Filter (both gyro and accelerometer)
    Wire.write(0x01);                        // Set to 20Hz bandwidth
    Wire.endTransmission();
    delay(50);
 
    Wire.beginTransmission(IMUADDR);       
    Wire.write(0x1B);                       // Set sensitivity at Register 0x1B
    Wire.write(0x08);                        // Sensitivity at +- 500 degrees/s. 
    Wire.endTransmission();                 // IMPORTANT: This sets 65.5 bits/degree/s. Divide read value by 65.5!
    delay(50);

    // Configure Accelerometer
    Wire.beginTransmission(IMUADDR); 
    Wire.write(0x1C);                       // Set sensitivity at Register 0x1C
    Wire.write(0x08);                       // Set sensitivity at +-4g (non aggressive flying)
    Wire.endTransmission();
    delay(50);

    Wire.beginTransmission(IMUADDR);        // Set bypass to access magnetometer at 0x0C
    Wire.write(0x37);
    Wire.endTransmission();
    Wire.requestFrom(IMUADDR, 1);
    byte intpincfg = Wire.read();
    intpincfg |= 0x02;
    Wire.beginTransmission(IMUADDR);
    Wire.write(0x37);
    Wire.write(intpincfg);
    Wire.endTransmission();
    delay(50);

    calibrate_gyro();
}

ImuData Imu::read(){
    //unsigned long start = micros();
    ImuData data;

    // Request accelerometer data
    Wire.beginTransmission(IMUADDR);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(IMUADDR, 14);

    //Read accelerometer data as bits
    int16_t accelX = Wire.read()<<8 | Wire.read();
    int16_t accelY = Wire.read()<<8 | Wire.read();
    int16_t accelZ = Wire.read()<<8 | Wire.read();

    Wire.read(); Wire.read();      // Temperature data, not needed

    // Read gyro data
    int16_t gyroX = Wire.read()<<8 | Wire.read();
    int16_t gyroY = Wire.read()<<8 | Wire.read();
    int16_t gyroZ = Wire.read()<<8 | Wire.read();
    //Serial.println(micros() - start);

    // Calculate angular velocities  
    // NOTE: has to remap to actual x y z of drone due to imu mounting position!
    data.gyro.x = -(float)gyroX/65.5;          
    data.gyro.y = (float)gyroY/65.5;             
    data.gyro.z = -(float)gyroZ/65.5;     
    
    deg_to_rad(data.gyro);

    data.gyro.x -= offset.gyro.x;
    data.gyro.y -= offset.gyro.y;
    data.gyro.z -= offset.gyro.z;

    // Calculate accelerometer data
    float AccX = -(float)accelX/8192.0 - offset.accel.x;
    float AccY = ((float)accelY/8192.0) - offset.accel.y;
    float AccZ = -(float)accelZ/8192.0 - offset.accel.z;

    data.accel.x = AccX;
    data.accel.y = AccY;
    data.accel.z = AccZ;  
    
    return data;
}

void Imu::calibrate_gyro(){
    float x_rot_rate_offset = 0.0;
    float y_rot_rate_offset = 0.0;
    float z_rot_rate_offset = 0.0;

    for (int i=0; i<1000; i++){
        ImuData data = read();
        x_rot_rate_offset += data.gyro.x;
        y_rot_rate_offset += data.gyro.y;
        z_rot_rate_offset += data.gyro.z;
        delay(1);
    }

    offset.gyro.x = x_rot_rate_offset/1000.0;       // convention: right roll = positive
    offset.gyro.y = y_rot_rate_offset/1000.0;    // convention: up pitch = positive
    offset.gyro.z = z_rot_rate_offset/1000.0;        // convention: right yaw = positive
}