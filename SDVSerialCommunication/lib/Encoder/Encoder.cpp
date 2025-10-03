#include <Arduino.h>
#include "Encoder.h"

uint8_t Encoder::encoders;
Encoder *Encoder::instances[4];

Encoder::Encoder() {}

Encoder::Encoder(uint8_t enc_a_pin, uint8_t enc_b_pin)
{
    // Configuring pins
    Encoder::enc_a_pin = enc_a_pin;
    Encoder::enc_b_pin = enc_b_pin;
    pinMode(enc_a_pin, INPUT_PULLUP);
    pinMode(enc_b_pin, INPUT_PULLUP);

    // Storing instance in static array
    storeInstance();

    // Attaching interrupts
    attachIsrEnc(encoders, enc_a_pin);
    attachIsrEnc(encoders, enc_b_pin);
    encoders++;
}

void Encoder::read_enc()
{
    // Read encoder pins
    curr_enc_a_val = digitalRead(enc_a_pin);
    curr_enc_b_val = digitalRead(enc_b_pin);

    // Obtains a number with previus and current bit values
    inst_enc_val = (prev_enc_a_val << 3) | (prev_enc_b_val << 2) | (curr_enc_a_val << 1) | curr_enc_b_val;

    // Search for an increment value using obtained number
    // http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
    encoderVal = encoderVal + lookup_enc[inst_enc_val];

    // Save encoder values to prev variables
    prev_enc_a_val = curr_enc_a_val;
    prev_enc_b_val = curr_enc_b_val;
}

long Encoder::getCounts()
{
    return encoderVal;
}

void Encoder::storeInstance()
{
    instances[encoders] = this;
}

void Encoder::attachIsrEnc(uint8_t which, uint8_t pin)
{
    switch (which)
    {
    case 0:
        attachInterrupt(pin, Encoder::enc_isr_0, CHANGE);
        break;
    case 1:
        attachInterrupt(pin, Encoder::enc_isr_1, CHANGE);
        break;
    case 2:
        attachInterrupt(pin, Encoder::enc_isr_2, CHANGE);
        break;
    case 3:
        attachInterrupt(pin, Encoder::enc_isr_3, CHANGE);
        break;
    default:
        break;
    }
}