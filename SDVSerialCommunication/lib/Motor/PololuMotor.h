#include <Arduino.h>
#include <PID_v1.h>
#include <Encoder.h>
#include "Motor.h"
#include "motor_model.h"

#pragma once

#define PULSES_PER_REV  1920.0  // 64*30 = 1920
#define CICLES_PER_SEC  10.0
#define SAMPLE_TIME     100    // 10 cicles => 100 ms every cicle

//------------------------------------------------------------------------------
// Structures
//------------------------------------------------------------------------------

union MotorCurrent
{
    float f;
    byte b[4];
};

struct MotorStatus
{
    MotorCurrent current;
    bool half_bridge_a;
    bool half_bridge_b;
};

enum DriverStatus
{
    ENABLE = 0,
    DISABLE = 1,
};

class PololuMotor : public Motor
{
public:
    PololuMotor();
    PololuMotor(
        String name,
        byte pwm_pin, 
        byte direction_input_a_pin, 
        byte direction_input_b_pin, 
        byte encoder_a_pin,
        byte encoder_b_pin,
        bool inverted);
    double getSpeed();
    void printStatus();
    void setPID(double kp, double ki, double kd);
    void setSpeed(bool enable, double rps);
    void stopMotor();
    void update();
    void updateOdometry();
    void setMotorStatus(uint8_t data[]);

private:

    double ref_rps = 0;  // Motor speed reference
    byte pwm_pin = 0;
    byte direction_a_pin = 0;
    byte direction_b_pin = 0;
    byte encoder_a_pin = 0;
    byte encoder_b_pin = 0;
    byte dir_a = LOW;
    byte dir_b = HIGH;

    PID *motor_pid;
    double Kp = 8; 
    double Ki = 30;
    double Kd = 0.2;
    double Setpoint;
    double Input;
    double Output;
    double motor_speed_rps = 0.0;
    MotorStatus motor_status;

    Encoder *motor_encoder;
    long prev_enc_count = 0;
};