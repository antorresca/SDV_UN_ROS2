#include <Arduino.h>
#include "PololuMotor.h"


class FourDriveController
{
public:
    FourDriveController();
    FourDriveController(
        PololuMotor *back_right_motor,
        PololuMotor *back_left_motor,
        PololuMotor *front_right_motor,
        PololuMotor *front_left_motor
    );
    int setSpeeds(bool enabled, double values[]);
    void printMotorsStatus();
    void stopMotors();
    void printSpeeds();
    void attachUpdateTask(void (*)(void));
    void updateController(unsigned long time);
    void (*update_task)(void);
    void updateOdometry();
    void setMotorStatus(uint8_t data[]);
    uint8_t getMotorModel();

    PololuMotor *back_right_motor;
    PololuMotor *back_left_motor;
    PololuMotor *front_right_motor;
    PololuMotor *front_left_motor;
    uint8_t n_motors = 4;

    unsigned long last_motor_update_stamp = 0;
    unsigned long last_driver_sensor_update_stamp = 0;

private:
    PololuMotor *motors[4];
};