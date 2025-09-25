#include <Arduino.h>
#include "EsconMotor.h"
#include "motor_model.h"

EsconMotor::EsconMotor() {}
EsconMotor::EsconMotor(
    String name,
    byte pwm_pin,
    byte enable_pin,
    byte direction_pin,
    byte speed_pin,
    byte current_pin,
    bool inverted)
{
    // Set motor model
    motor_model = MotorModel::ESCON;
    EsconMotor::name = name;

    // Save pins
    EsconMotor::pwm_pin = pwm_pin;
    EsconMotor::enable_pin = enable_pin;
    EsconMotor::direction_pin = direction_pin;
    EsconMotor::speed_pin = speed_pin;
    EsconMotor::current_pin = current_pin;

    // Set encoder status
    attached_encoder = false;

    // Set inputs and outputs
    pinMode(pwm_pin, OUTPUT);
    pinMode(enable_pin, OUTPUT);
    pinMode(direction_pin, OUTPUT);
    pinMode(speed_pin, INPUT);
    pinMode(current_pin, INPUT);

    // Set direction
    setInvertedDirection(inverted);
}

double EsconMotor::getSpeed()
{
    return speed_analog;
}

void EsconMotor::setSpeed(bool enable, double duty_cycle)
{
    EsconMotor::motor_enable = enable;
    EsconMotor::duty_cycle = duty_cycle;

    // Motor direction
    bool dir = false;
    if (duty_cycle < 0)
        dir = true;
    else
        dir = false;

    if (inverted_direction)
    {
        direction = !dir;
    }
    else
    {
        direction = dir;
    }
}

void EsconMotor::stopMotor()
{    
    setSpeed(false, 10.0);
}

void EsconMotor::update()
{
    if(!motor_enable)
    {
        digitalWrite(enable_pin, LOW);
        digitalWrite(direction_pin, LOW);
    }
    else
    {
        digitalWrite(enable_pin, motor_enable);
        digitalWrite(direction_pin, direction);
    }

    // Motor PWM
    pwm_val = map(abs(duty_cycle), 0, 100, 0, 255);
    analogWrite(pwm_pin, pwm_val);

    // Read Motor Current
    current_analog = analogRead(current_pin);
}

void EsconMotor::updateOdometry()
{
    // Odometry is meassured using an analog input.
    speed_analog = analogRead(speed_pin);
}

void EsconMotor::printStatus()
{
    Serial.print(current_analog, 7);
    /*
    Serial.print("\n\r");
    Serial.print("Name = ");
    Serial.print(name);
    Serial.print(", pwm_pin = ");
    Serial.print(pwm_pin);
    Serial.print(", inverted_direction = ");
    Serial.print(inverted_direction);
    Serial.print(", enable = ");
    Serial.print(motor_enable);
    Serial.print(", pwm = ");
    Serial.print(pwm_val);
    Serial.print(", duty_cycle = ");
    Serial.print(duty_cycle);
    Serial.print(", direction = ");
    Serial.print(direction);
    */
}