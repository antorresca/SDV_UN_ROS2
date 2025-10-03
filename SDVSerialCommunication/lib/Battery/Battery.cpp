#include <Arduino.h>
#include <float.h>
#include "Battery.h"


Battery::Battery(/* args */)
{
    vcells[0] = &vcell_1;
    vcells[1] = &vcell_2;
    vcells[2] = &vcell_3;
    vcells[3] = &vcell_4;
    for(uint8_t i = 0; i < n_cells; i++)
    {
        vcells[i]->f = 0.0;
    }
}

void Battery::buffer2vcells(uint8_t buffer_array[])
{
    uint8_t index = 0;
    for (uint8_t i = 0; i < n_cells; i++)
    {
        for (uint8_t j = 0; j < 4; j++)
        {
            vcells[i]->b[j] = buffer_array[index];
            index++;
        }
    }
}

void Battery::print_vcells()
{
    for(uint8_t i = 0; i < 4; i++)
    {
        Serial.print((*vcells[i]).f, 7);
        if(i < 3)
        {
            Serial.print(" ");
        }
    }
}

