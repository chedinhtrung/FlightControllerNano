#include "motor.h"
#include "Arduino.h"

void Motor::set_motor_raw(int fl, int fr, int bl, int br) {
    analogWrite(2, fl);
    analogWrite(A8, fr);
    analogWrite(5, bl);
    analogWrite(11, br);
}

void Motor::set_motor(float fl, float fr, float bl, float br){
    /*
        FrontLeft  ---- ^^^^^^ ---- FrontRight


        BackLeft   ---- ^^^^^^ ---- BackRight
    
    */

    /*
        0%   ---> 1024 ---> 1000ms
        100% ---> 4095 ---> 2000ms
    */

    //fr += 0.019;  // Motor trimming 

    if (fr > 1){fr = 0.01;}
    else if (fr < 0){fr = 0;}

    if (fl > 1){fl = 0.01;}
    else if (fl < 0){fl = 0;}

    if (br > 1){br = 0.01;}
    else if (br < 0){br = 0;}
    
    if (bl > 1){bl = 0.01;}
    else if (bl < 0){bl = 0;}

    int mfl = (int)(fl*1023 + 1024);
    int mfr = (int)(fr*1023 + 1024); 
    int mbl = (int)(bl*1023 + 1024);
    int mbr = (int)(br*1023 + 1024);   

    motor_report.bl = (uint16_t)mbl;
    motor_report.fl = (uint16_t)mfl;
    motor_report.fr = (uint16_t)mfr;
    motor_report.br = (uint16_t)mbr;

    set_motor_raw(mfl, mfr, mbl, mbr);
}