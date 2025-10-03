#include <Arduino.h>
#include "EsconMotor.h"

class TwoDriveController
{
public:
    TwoDriveController();
    TwoDriveController(EsconMotor *left_motor, EsconMotor *right_motor);
    int setSpeeds(bool enabled, double values[]);
    void attachUpdateTask(void (*)(void));
    void printMotorsStatus();
    void printSpeeds();
    void stopMotors();
    void (*update_task)(void);
    void updateController(unsigned long time);
    void updateOdometry();
    void setMotorStatus(uint8_t data[]);
    uint8_t getMotorModel();

    EsconMotor *left_motor;
    EsconMotor *right_motor;
    EsconMotor *motors[2];
    
    byte n_motors = 2;
    unsigned long last_motor_update_stamp = 0;

private:

};