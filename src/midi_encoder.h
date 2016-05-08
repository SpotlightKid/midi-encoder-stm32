#ifndef MIDI_ENCODER_H
#define MIDI_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ENC_ACCEL_THRESHOLD 5
#define NUM_ENCODERS 2

struct RotaryEncoder {
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
};

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);
static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue);
static void usbmidi_send_event(usbd_device *usbd_dev, uint8_t ctrl, uint8_t value);

static void systick_setup(void);
static void read_encoders(void);
static int8_t read_encoder(struct RotaryEncoder* enc);

#endif  /* MIDI_ENCODER_H */
