#include <Arduino.h>
#include "FourDriveController.h"

FourDriveController::FourDriveController()
{
}

FourDriveController::FourDriveController(
    PololuMotor *back_left_motor,
    PololuMotor *back_right_motor,
    PololuMotor *front_left_motor,
    PololuMotor *front_right_motor)
{
    FourDriveController::back_left_motor = back_left_motor;
    FourDriveController::back_right_motor = back_right_motor;
    FourDriveController::front_left_motor = front_left_motor;
    FourDriveController::front_right_motor = front_right_motor;

    motors[0] = FourDriveController::back_left_motor;
    motors[1] = FourDriveController::back_right_motor;
    motors[2] = FourDriveController::front_left_motor;
    motors[3] = FourDriveController::front_right_motor;
}

int FourDriveController::setSpeeds(bool enabled, double values[])
{
    double duty_cycle_back_l = values[0];
    double duty_cycle_back_r = values[1];
    double duty_cycle_front_l = values[2];
    double duty_cycle_front_r = values[3];

    back_left_motor->setSpeed(enabled, duty_cycle_back_l);
    back_right_motor->setSpeed(enabled, duty_cycle_back_r);
    front_left_motor->setSpeed(enabled, duty_cycle_front_l);
    front_right_motor->setSpeed(enabled, duty_cycle_front_r);

    return 0;
}

void FourDriveController::stopMotors()
{
    for (uint8_t i = 0; i < n_motors; i++)
        motors[i]->stopMotor();
}

void FourDriveController::printMotorsStatus()
{
    for (uint8_t i = 0; i < n_motors; i++)
    {
        motors[i]->printStatus();
        if(i != n_motors - 1) Serial.print(" ");
    }
}

void FourDriveController::printSpeeds()
{
    for (uint8_t i = 0; i < n_motors; i++)
    {
        Serial.print(motors[i]->getSpeed(), 3);
        Serial.print(" ");
    }
}

void FourDriveController::updateController(unsigned long time)
{
    // This controller updates motors every 100ms
    if (time - last_motor_update_stamp >= 100)
    {
        // Send update signal to motors
        last_motor_update_stamp = time;
        for (uint8_t i = 0; i < n_motors; i++)
            motors[i]->update();
    }

    // This controller motor driver every 500ms
    if (time - last_driver_sensor_update_stamp >= 500)
    {
        last_driver_sensor_update_stamp = time;
        update_task();
    }
}

void FourDriveController::updateOdometry()
{
    for (uint8_t i = 0; i < n_motors; i++)
        motors[i]->updateOdometry();
}


void FourDriveController::attachUpdateTask(void (*function)(void))
{
    update_task = function;
}

void FourDriveController::setMotorStatus(uint8_t data[])
{
    // Data array must contain 24 bytes. 6 bytes for every motor
    for (uint8_t i = 0; i < n_motors; i++)
    {
        uint8_t d[6];
        for(uint8_t j = 0; j < 6; j++) d[j] = data[j + (6 * i)];
        motors[i]->setMotorStatus(d);
    }
}


uint8_t FourDriveController::getMotorModel() 
{
    return motors[0]->getMotorModel();
}
