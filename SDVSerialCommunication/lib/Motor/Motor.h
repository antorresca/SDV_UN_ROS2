#include <Arduino.h>
#include "motor_model.h"

#pragma once

class Motor
{
public:
    Motor();
    void setInvertedDirection(bool inverted);
    double getSpeed();
    void setSpeed(bool enable, double duty_cycle);
    void stopMotor();
    void printStatus();
    void update();
    void updateOdometry();
    uint8_t getMotorModel();

protected:
    MotorModel motor_model = MotorModel::NONE;
    String name = "";
    bool motor_enable = false;
    bool attached_encoder = false;
    bool inverted_direction = false;
    bool direction = false;
    byte pwm_val = 0;
    double duty_cycle = 0.0;
    
private:
};