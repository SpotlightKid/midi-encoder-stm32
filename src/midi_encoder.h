#ifndef MIDI_ENCODER_H
#define MIDI_ENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include <libopencm3/usb/usbd.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ENC_ACCEL_THRESHOLD 5
#define NUM_ENCODERS 2

/* monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering
 */
volatile uint32_t system_millis;

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

const struct RotaryEncoder encoder_tmpl = {
    .port = GPIOA,
    .pin_a = 0,
    .pin_b = 1,
    .min_value = 0,
    .max_value = 127,
    .pulses_per_dedent = 4,
    .acceleration = 20,
    .cur_accel = 0,
    .readings = 0,
    .counter = 0,
    .value = 0,
    .controller = 1,
    .dirty = false
};

struct RotaryEncoder encoders[NUM_ENCODERS];

static int8_t enc_states[] = {
    0,  /* 00 00 */
    -1, /* 00 01 */
    1,  /* 00 10 */
    0,  /* 00 11 */
    1,  /* 01 00 */
    0,  /* 01 01 */
    0,  /* 01 10 */
    -1, /* 01 11 */
    -1, /* 10 00 */
    0,  /* 10 01 */
    0,  /* 10 10 */
    1,  /* 10 11 */
    0,  /* 11 00 */
    1,  /* 11 01 */
    -1, /* 11 10 */
    0   /* 11 11 */
};


static char usb_serial_number[25]; /* 12 bytes of desig and a \0 */

static const char * usb_strings[] = {
    "libopencm3.org",
    "MIDI demo",
    usb_serial_number
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

/* SysEx identity message, preformatted with correct USB framing information */
const uint8_t sysex_identity[] = {
    0x04,   /* USB Framing (3 byte SysEx) */
    0xf0,   /* SysEx start */
    0x7e,   /* non-realtime */
    0x00,   /* Device ID 0 */
    0x04,   /* USB Framing (3 byte SysEx) */
    0x7d,   /* Educational/prototype manufacturer ID */
    0x66,   /* Device Family code (LSB) */
    0x66,   /* Device Family code (MSB) */
    0x04,   /* USB Framing (3 byte SysEx) */
    0x51,   /* Model number (LSB) */
    0x19,   /* Model number (MSB) */
    0x00,   /* Version number (byte 1) */
    0x04,   /* USB Framing (3 byte SysEx) */
    0x00,   /* Version number (byte 2) */
    0x00,   /* Version number (byte 3) */
    0x03,   /* Version number (byte 4) */
    0x05,   /* USB Framing (1 byte SysEx) */
    0xf7,   /* SysEx end */
    0x00,   /* Padding */
    0x00,   /* Padding */
};

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);
static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue);
static void usbmidi_send_event(usbd_device *usbd_dev, uint8_t ctrl, uint8_t value);

static void systick_setup(void);
static void read_encoders(void);
static int8_t read_encoder(struct RotaryEncoder* enc);

#endif  /* MIDI_ENCODER_H */
