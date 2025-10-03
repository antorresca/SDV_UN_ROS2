#include <Arduino.h>
#include "Motor.h"
#include "motor_model.h"

Motor::Motor()
{
}

void Motor::setInvertedDirection(bool inverted)
{
    inverted_direction = inverted;
}

double Motor::getSpeed() { return 0.0; }

void Motor::setSpeed(bool enable, double duty_cycle) {}

void Motor::stopMotor() {}

void Motor::printStatus() {}

void update() {}


uint8_t Motor::getMotorModel() 
{
    return motor_model;
}