#ifndef MOTOR 
#define MOTOR
#include <Arduino.h>

#define FL_PIN 3
#define FR_PIN 4
#define BL_PIN 5
#define BR_PIN 6

struct RawMotor {
    uint16_t fl;
    uint16_t fr;
    uint16_t bl;
    uint16_t br;
};

struct PercentMotor {
    float fl;
    float fr;
    float bl;
    float br;
};

class Motor {
    public:
        Motor();
        void setup();
        void set_motor_raw(int m1, int m2, int m3, int m4);
        void set_motor(float fl, float fr, float bl, float br);
};

/*
void set_motor(int m1, int m2, int m3, int m4);
void set_motor(float fl, float fr, float bl, float br);
*/

#endif 