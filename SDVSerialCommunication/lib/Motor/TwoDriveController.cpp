#include <Arduino.h>
#include "TwoDriveController.h"
#include "EsconMotor.h"

TwoDriveController::TwoDriveController()
{
}

TwoDriveController::TwoDriveController(EsconMotor *left_motor, EsconMotor *right_motor)
{
    TwoDriveController::left_motor = left_motor;
    TwoDriveController::right_motor = right_motor;

    motors[0] = TwoDriveController::left_motor;
    motors[1] = TwoDriveController::right_motor;
}

int TwoDriveController::setSpeeds(bool enabled, double values[])
{
    double duty_cycle_L = values[0];
    double duty_cycle_R = values[1];

    /*
    In "serial_sdvun_node" application, min value of every wheel is between -40 and 40 (cm/s)
    Received value will be constrined between 10% and 90% of duty cicle
    Duty cycle has to been converted to 8-bit value (0-255)
    */
    if (abs(duty_cycle_L) > 90.0 or abs(duty_cycle_L) < 10.0)
        return -1;
    if (abs(duty_cycle_R) > 90.0 or abs(duty_cycle_R) < 10.0)
        return -1;

    left_motor->setSpeed(enabled, duty_cycle_L);
    right_motor->setSpeed(enabled, duty_cycle_R);

    return 0;
}

void TwoDriveController::stopMotors()
{
    left_motor->stopMotor();
    right_motor->stopMotor();
}

void TwoDriveController::printMotorsStatus()
{
    left_motor->printStatus();
    Serial.print(" ");
    right_motor->printStatus();
}

void TwoDriveController::updateController(unsigned long time)
{
    // This controller updates motors every 100ms
    if (time - last_motor_update_stamp >= 100)
    {
        // Send update signal to motors
        last_motor_update_stamp = time;
        for (uint8_t i = 0; i < n_motors; i++)
            motors[i]->update();
    }
}

void TwoDriveController::updateOdometry()
{
    left_motor->updateOdometry();
    right_motor->updateOdometry();
}

void TwoDriveController::printSpeeds()
{
    for (uint8_t i = 0; i < n_motors; i++)
    {
        Serial.print(motors[i]->getSpeed());
        Serial.print(" ");
    }
}

void TwoDriveController::attachUpdateTask(void (*function)(void))
{
    update_task = function;
}

void TwoDriveController::setMotorStatus(uint8_t data[])
{

}

uint8_t TwoDriveController::getMotorModel() 
{
    return motors[0]->getMotorModel();
}