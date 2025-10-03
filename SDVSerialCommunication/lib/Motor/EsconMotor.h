#include <Arduino.h>
#include "Motor.h"
#include "motor_model.h"

#pragma once

class EsconMotor : public Motor
{
public:
    EsconMotor();
    EsconMotor(
        String name,
        byte pwm_pin,
        byte enable_pin,
        byte direction_pin,
        byte speed_pin,
        byte current_pin,
        bool inverted);
    double getSpeed();
    void printStatus();
    void setSpeed(bool enable, double duty_cycle);
    void stopMotor();
    void update();
    void updateOdometry();

private:
    byte pwm_pin = 0;
    byte enable_pin = 0;
    byte direction_pin;
    byte speed_pin = 0;
    byte current_pin = 0;

    double speed_analog = 0.0;
    double current_analog = 0.0;
};