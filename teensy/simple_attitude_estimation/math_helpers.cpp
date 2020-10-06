/*  
Math helpers implementation 
*/
#include "Arduino.h"
#include "math_helpers.h"

float constrain_angle(float x)
{
    x = fmod(x + PI, 2*PI);
    if (x < 0)
        x += 2*PI;
    return x - PI;
}