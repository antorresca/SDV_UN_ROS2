#include <Arduino.h>

double getAbs(double val)
{
    if(val > 0.0)
        return val;
    else
        return -val;
}