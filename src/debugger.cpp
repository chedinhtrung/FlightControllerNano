#include "debugger.h"
#include <cstdarg>
#include <string>

void debug(const char* label, ConvertedImuData data){
    Serial.print(label);
    Serial.print(" x: ");
    Serial.print(data.x, 3);
    Serial.print(" y: ");
    Serial.print(data.y, 3);
    Serial.print(" z: ");
    Serial.println(data.z, 3);
}