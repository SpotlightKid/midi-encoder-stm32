#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ENC_ACCEL_THRESHOLD 5

typedef struct RotaryEncoder {
    uint32_t port;
    uint8_t pin_a;
    uint8_t pin_b;
    uint16_t min_value;
    uint16_t max_value;
    uint8_t pulses_per_dedent;
    uint8_t acceleration;
    uint8_t cur_accel;
    uint8_t readings;
    uint16_t counter;
    uint16_t value;
    uint8_t controller;
    bool dirty;
} RotaryEncoder;

void init_encoder(RotaryEncoder* enc);
int8_t read_encoder(RotaryEncoder* enc);
void read_encoders(RotaryEncoder* enc, uint8_t num_enc);

#endif  /* ROTARY_ENCODER_H */
