#include <Arduino.h>
#pragma once

class Encoder
{
public:
    Encoder();
    Encoder(uint8_t enc_a_pin, uint8_t enc_b_pin);
    long getCounts();

private:
    static uint8_t encoders;
    static Encoder *instances[];
    int lookup_enc[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    uint8_t enc_a_pin;
    uint8_t enc_b_pin;
    volatile long encoderVal = 0;
    volatile uint8_t inst_enc_val = 0;
    volatile uint8_t prev_enc_a_val = 0;
    volatile uint8_t prev_enc_b_val = 0;
    volatile uint8_t curr_enc_a_val = 0;
    volatile uint8_t curr_enc_b_val = 0;

    void storeInstance();
    void read_enc();

    // Glue functions
    void attachIsrEnc(uint8_t which, uint8_t pin);

    static void enc_isr_0(){instances[0]->read_enc();};
    static void enc_isr_1(){instances[1]->read_enc();};
    static void enc_isr_2(){instances[2]->read_enc();};
    static void enc_isr_3(){instances[3]->read_enc();};

};