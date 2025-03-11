
#include <Wire.h>
#include "datastructures.h"
#include "motor.h"
#include "imu.h"
#include "dps310.h"
#include "kalman.h"
#include "model.h"
#include "pid.h"
#include <PulsePosition.h>
#include "receiver.h"
#include "TinyGPS++.h"

#include <Servo.h>

Servo gimbal; 

#define STARTBYTE 0x02

static Model model;
static Report report;

Kalman rollkalmanfilter = Kalman(6, 3, 6);
Kalman pitchkalmanfilter = Kalman(6, 3, 6);

PulsePositionInput control_in(RISING);
Receiver receiver = Receiver();
Imu imu = Imu();
Dps310Altimeter alt = Dps310Altimeter();
TinyGPSPlus gps;

PID pitchratepid = PID(0.00066, 0.0002, 0.0025*1e-3); 
PID rollratepid = PID(0.0006, 0.0002, 0.0025*1e-3); 
PID yawratepid = PID(0.0044, 0.00017, 0);
//PID anglepid = PID();
/*
//Baseline
PID pitchratepid = PID(0.000137, 0.00015, 0.00204*1e-3); 
PID rollratepid = PID(0.000137, 0.00015, 0.00204*1e-3); 
PID yawratepid = PID(0.0018, 0.00018, 0);
*/

Motor motor;

unsigned long last_active = micros();
unsigned long last_compass = micros();
unsigned long last_report = micros();
unsigned long last_altupdate = micros();
unsigned long last_gimbal = micros();

double h_alpha = 0.97;
double h = 0;
double vert_speed = 0;

void clearI2C() {
  Wire.begin();
  
  // Toggle SCL and SDA pins to clear any stuck state
  pinMode(SCL, OUTPUT);
  pinMode(SDA, OUTPUT);
  
  // Generate 9 clock pulses to reset any stuck devices
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delay(1);
    digitalWrite(SCL, HIGH);
    delay(1);
  }
  
  // Send a stop signal
  digitalWrite(SDA, LOW);
  delay(1);
  digitalWrite(SCL, HIGH);
  delay(1);
  digitalWrite(SDA, HIGH);

}


void setup() {
  gimbal.attach(8);
  gimbal.write(0);

  delay(4000);

  Wire.begin();
  Wire.setClock(400000);
  
  delay(1000);
  imu.setup();
  delay(200);

  analogWriteFrequency(FR, 2000);
  analogWriteFrequency(FL, 2000);
  analogWriteFrequency(BR, 2000);
  analogWriteFrequency(BL, 2000);
  analogWriteResolution(12);

  Serial.begin(115200);
  Serial4.begin(115200);

  //clearI2C();
  delay(100);
  
  //while (!Serial1){}                    // block until RPi goes online
  //comp.setup();
  //delay(200);
  //comp.calibrate();

  // Initial read using accelrometer at stand still
  ImuData imudata = imu.read();
  model.angle = imudata.angle;

  //also update initial trigonometric cache values
  model.sin_pitch = sinf(model.angle.pitch*PI/180);
  model.cos_pitch = cosf(model.angle.pitch*PI/180);
  model.sin_roll = sinf(model.angle.roll*PI/180);
  model.cos_roll = cosf(model.angle.roll*PI/180);
  model.tan_pitch = model.sin_pitch/model.cos_pitch;   

  delay(100);
  alt.setup();
  //delay(100);
  
  //warning: null division (gymbal lock)
  //yaw = comp.getHeadingCompensated(rollkalmanfilter.x_hat, pitchkalmanfilter.x_hat);
  ReceiverData data = receiver.recv();    // Lock until throttle is 0 first.
  
  while (data.ThrottleIn > 1050){
    data = receiver.recv();
  }

  
  
}
void loop() {
  last_active = micros();
  
  ImuData imudata = imu.read();

  model.predict(imudata.angle_rate);

  pitchkalmanfilter.bypass_predict();
  pitchkalmanfilter.correct(imudata.angle.pitch, model.angle.pitch);
  pitchkalmanfilter.update();

  rollkalmanfilter.bypass_predict();
  rollkalmanfilter.correct(imudata.angle.roll, model.angle.roll);
  rollkalmanfilter.update();

  model.coord_predict(imudata.accel);

  ReceiverData rd = receiver.recv();
  rd = receiver.to_anglemode(rd);       //IMPORTANT: forgetting this line will cause drone to fly away

  // calculate errors
  
  double pitch_error = rd.PitchIn - model.angle.pitch;
  double roll_error = rd.RollIn - model.angle.roll;

  double pitchrate_target, rollrate_target;

  // use square root curve to calculate desired rates
  if (pitch_error >= 0){
    pitchrate_target = pitch_error/(0.025*sqrt(pitch_error) + 0.45);
  } else {
    pitchrate_target = pitch_error/(0.025*sqrt(-pitch_error) + 0.45);
  }

  if (roll_error >= 0){
    rollrate_target = roll_error/(0.018*sqrt(roll_error) + 0.45);;
  } else {
    rollrate_target = roll_error/(0.018*sqrt(-roll_error) + 0.45);
  }
  

  double yawrate_target = 2.8*rd.YawIn;

  Angle euler_rate_target;
  euler_rate_target.pitch = pitchrate_target;
  euler_rate_target.roll = rollrate_target;
  euler_rate_target.yaw = yawrate_target;
  
  // convert to desired body rate

  Angle body_rate_target = model.euler_to_body(euler_rate_target);

  // feed the error to pid
  double pitchadjust = pitchratepid.calculate(body_rate_target.pitch - imudata.angle_rate.pitch);
  double rolladjust = rollratepid.calculate(body_rate_target.roll - imudata.angle_rate.roll);
  double yawadjust = yawratepid.calculate(body_rate_target.yaw - imudata.angle_rate.yaw);

  //Map to motor output

  float fl = rd.ThrottleIn + pitchadjust + rolladjust - yawadjust;
  float fr = rd.ThrottleIn + pitchadjust - rolladjust + yawadjust;
  float bl = rd.ThrottleIn - pitchadjust + rolladjust + yawadjust;
  float br = rd.ThrottleIn - pitchadjust - rolladjust - yawadjust;

  // Output to motor, lock until throttle is not 0
  // Danger: forgetting to convert rd.ThrottleIn to percentage will lead to flyaway
  
  if (rd.ThrottleIn > 0.01 && rd.ThrottleIn <= 1.0){
    motor.set_motor(fl, fr, bl, br);
  } else {
    motor.set_motor(0.0f, 0.0f, 0.0f, 0.0f);
    pitchratepid.reset();
    rollratepid.reset();
    yawratepid.reset();
    //alt.filter.reset();
  }

    // actuate gimbal
    
  if (micros() - last_gimbal > 0.04*1e6){
    int compens = (int) model.angle.pitch;
    int gimbal_angle = (int)((rd.AuxChannel6In-1000)/2000 * 360);
    if (gimbal_angle < 10){
      gimbal_angle = 10;
    }
    if (compens > 70){
      compens = 70;
    }
    int final_angle = gimbal_angle + compens;
    if (final_angle < 10){
      final_angle = 10;
    }
    gimbal.write(final_angle);
    last_gimbal = micros(); 
  }
    

  while(micros() - last_active < DT*1000.0){}

}