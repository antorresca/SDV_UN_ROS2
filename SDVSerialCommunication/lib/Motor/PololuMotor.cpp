#include <Arduino.h>
#include <PID_v1.h>
#include <Encoder.h>
#include "PololuMotor.h"
#include "motor_model.h"
#include "tools.h"


PololuMotor::PololuMotor() {}

PololuMotor::PololuMotor(
    String name,
    byte pwm_pin,
    byte direction_a_pin,
    byte direction_b_pin,
    byte encoder_a_pin,
    byte encoder_b_pin,
    bool inverted)
{
    // Set motor model
    motor_model = MotorModel::POLOLU;
    PololuMotor::name = name;

    // Save pins
    PololuMotor::pwm_pin = pwm_pin;
    PololuMotor::direction_a_pin = direction_a_pin;
    PololuMotor::direction_b_pin = direction_b_pin;
    PololuMotor::encoder_a_pin = encoder_a_pin;
    PololuMotor::encoder_b_pin = encoder_b_pin;
    inverted_direction = inverted;

    // Set encoder status
    attached_encoder = true;

    // Set inputs and outputs
    pinMode(pwm_pin, OUTPUT);
    pinMode(direction_a_pin, OUTPUT);
    pinMode(direction_b_pin, OUTPUT);
    pinMode(encoder_a_pin, INPUT);
    pinMode(encoder_b_pin, INPUT);

    // Encoder object
    motor_encoder = new Encoder(encoder_a_pin, encoder_b_pin);

    // PID object
    motor_pid = new PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
    motor_pid->SetMode(AUTOMATIC);
    motor_pid->SetOutputLimits(0, 100);
    motor_pid->SetSampleTime(SAMPLE_TIME);
}

void PololuMotor::setSpeed(bool enable, double rps)
{
    // Check speed value
    if(rps > 5.0 || rps < -5.0)
    {
        Serial.print("\u001b[31mWrong value for " + name + ": rps = ");
        Serial.print(rps, 2);
        Serial.print("\u001b[0m \n\r");
        return;
    }

    // Storing values
    motor_enable = enable;
    ref_rps = rps;
}

void PololuMotor::stopMotor()
{
    pwm_val = 0.0;
    duty_cycle = 0.0;
    ref_rps = 0.0;
    motor_enable = false;
    dir_a = LOW;
    dir_b = LOW;
    //analogWrite(pwm_pin, pwm_val);
    digitalWrite(direction_a_pin, dir_a);
    digitalWrite(direction_b_pin, dir_b);
}

double PololuMotor::getSpeed()
{
    //return motor_encoder->getCounts();
    return inverted_direction ? -1 * motor_speed_rps : motor_speed_rps;
}

void PololuMotor::update()
{

    // Calculate motor speed
    long current_enc_count = motor_encoder->getCounts();
    motor_speed_rps = -((double)(current_enc_count - prev_enc_count) * CICLES_PER_SEC) / PULSES_PER_REV;
    prev_enc_count = current_enc_count;

    // Update PID
    Setpoint = getAbs(ref_rps);
    Input = getAbs(motor_speed_rps);
    motor_pid->Compute();
    
    // Change output to PWM
    pwm_val = map(Output, 0, 100, 0, 255);

    // Motor direction: 2 pins controls the motor direction
    bool dir = false;
    if (ref_rps < 0)
        dir = true;
    else
        dir = false;

    if (inverted_direction)
        direction = !dir;
    else
        direction = dir;

    if (direction)
    {
        dir_a = HIGH;
        dir_b = LOW;
    }
    else
    {
        dir_a = LOW;
        dir_b = HIGH;
    }

    // Check if motor will be disable
    if (!motor_enable)
    {
        dir_a = LOW;
        dir_b = LOW;
    }

    // Check if rps is 0.0: if true, disable motor in Vcc break mode
    if(ref_rps == 0.0)
    {
        bool enab = LOW;
        if(motor_enable)
        {
            enab = HIGH;
            pwm_val = 255;
        }
        dir_a = enab;
        dir_b = enab;
        Output = 0.0;
    }
    digitalWrite(direction_a_pin, dir_a);
    digitalWrite(direction_b_pin, dir_b);

    if(motor_enable)
    {
        analogWrite(pwm_pin, getAbs(pwm_val));
    }
    else
    {
        analogWrite(pwm_pin, 0);
    }

    /*
    Serial.print("\n\r");
    Serial.print("Name = ");
    Serial.print(name);
    Serial.print(", ref = ");
    Serial.print(ref_rps);
    Serial.print(", curr = ");
    Serial.print(motor_speed_rps);
    Serial.print(", output = ");
    Serial.print(Output);
    Serial.print(", pwm = ");
    Serial.print(pwm_val);
    */

}

void PololuMotor::updateOdometry()
{
    // Odometry is meassured with encoder and its updated with interruptions
}

void PololuMotor::setPID(double kp, double ki, double kd)
{
}

void PololuMotor::setMotorStatus(uint8_t data[])
{
    // Data array muts contains 6 bytes
    // [0-3]: current
    // [4]: half-bridge A
    // [5]: half-bridge B
    for(uint8_t i = 0; i < 4; i++) motor_status.current.b[i] = data[i];
    motor_status.half_bridge_a = data[4];
    motor_status.half_bridge_b = data[5];
}

void PololuMotor::printStatus()
{
    Serial.print(motor_status.current.f, 7);
    Serial.print(" ");
    Serial.print(motor_status.half_bridge_a);
    Serial.print(" ");
    Serial.print(motor_status.half_bridge_b);
    /*
    Serial.print("\n\r");
    Serial.print("Name = ");
    Serial.print(name);
    Serial.print(", pwm_pin = ");
    Serial.print(pwm_pin);
    Serial.print(", inv_dir = ");
    Serial.print(inverted_direction);
    Serial.print(", enab = ");
    Serial.print(motor_enable);
    Serial.print(", pwm = ");
    Serial.print(pwm_val);
    Serial.print(", duty_cycle = ");
    Serial.print(duty_cycle);
    Serial.print(", dir = ");
    Serial.print(direction);
    Serial.print(", dir_a = ");
    Serial.print(dir_a);
    Serial.print(", dir_b = ");
    Serial.print(dir_b);
    Serial.print(", pin_dir_a = ");
    Serial.print(direction_a_pin);
    Serial.print(", pin_dir_b = ");
    Serial.print(direction_b_pin);
    */
}