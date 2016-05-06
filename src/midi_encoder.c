/*
 * Copyright (C) 2016 Christopher Arndt <chris@chrisarndt.de>
 *
 */

#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "midi_encoder.h"
#include "descriptors.h"


/* Called when systick fires */
void sys_tick_handler(void) {
    system_millis++;
    read_encoders();
}

static void read_encoders(void) {
    int8_t enc_incr;
    int16_t value;
    struct RotaryEncoder* enc = encoders;

    for (uint8_t i = 0; i < NUM_ENCODERS; i++, enc++) {
        enc_incr = read_encoder(enc);

        if (enc->cur_accel)
            enc->cur_accel--;

        if (enc_incr) {
            enc->cur_accel = MIN(255, enc->cur_accel + enc->acceleration);
            enc->counter = MIN(
                enc->max_value * enc->pulses_per_dedent,
                MAX(enc->min_value * enc->pulses_per_dedent,
                    enc->counter + (1 + (enc->cur_accel >> ENC_ACCEL_THRESHOLD)) * enc_incr));
            value = enc->counter / enc->pulses_per_dedent;

            if (enc->value != value) {
                enc->dirty = true;
                enc->value = value;
            }
        }
    }
}

/* returns change in encoder state (-1,0,1) */
static int8_t read_encoder(struct RotaryEncoder *enc) {
    uint16_t reading;

    reading = GPIO_IDR(enc->port);
    // shift previous state 2 bits to left
    enc->readings <<= 2;

    // Add current state to bit 0 and 1
    if (reading & (1 << enc->pin_a))
        enc->readings |= 1;
    if (reading & (1 << enc->pin_b))
        enc->readings |= 2;

    return (enc_states[(enc->readings & 0x0f)]);
}

/* Set up a timer to create 1mS ticks. */
static void systick_setup(void) {
    /* clock rate / 1000 to get 1mS interrupt rate */
    systick_set_reload(168000);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    /* this done last */
    systick_interrupt_enable();
}

/* USB stuff */

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
    (void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    /* This implementation treats any message from the host as a SysEx
     * identity request. This works well enough providing the host
     * packs the identify request in a single 8 byte USB message.
     */
    if (len) {
        while (usbd_ep_write_packet(usbd_dev, 0x81, sysex_identity,
                                    sizeof(sysex_identity)) == 0);
    }
}

static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue) {
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
                  usbmidi_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
}

static void usbmidi_send_event(usbd_device *usbd_dev, uint8_t ctrl, uint8_t value) {
    char buf[4] = {
        0x0B, /* USB framing: virtual cable 0, note on */
        0xB0, /* MIDI command: control change*/
        ctrl & 0x7F,
        value & 0x7F
    };

    while (usbd_ep_write_packet(usbd_dev, 0x81, buf, sizeof(buf)) == 0);
}

/* Encoder handling */

int main(void) {
    usbd_device *usbd_dev;
    struct RotaryEncoder* enc;

    /* Clocks setup */
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_OTGFS);

    /* USB pins */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO9 | GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

    desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));

    /* Encoder pins */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,
                    GPIO0 | GPIO1 | GPIO2 | GPIO3);

    /* Setup GPIO pin GPIO12 on GPIO port D for LED. */
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);

    /* Initialize USB driver */
    usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
                         usb_strings, 3,
                         usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbd_dev, usbmidi_set_config);

    /* Configure encoders */
    encoders[0] = encoder_tmpl;
    encoders[0].port = GPIOA;
    encoders[0].pin_a = 0;
    encoders[0].pin_b = 1;
    encoders[0].controller = 1;

    encoders[1] = encoder_tmpl;
    encoders[1].port = GPIOA;
    encoders[1].pin_a = 2;
    encoders[1].pin_b = 3;
    encoders[1].controller = 7;

    /* Start systick timer */
    systick_setup();

    /* event loop (polling) */
    while (1) {
        usbd_poll(usbd_dev);

        enc = encoders;
        for (uint8_t i = 0; i < NUM_ENCODERS; i++, enc++) {
            if (enc->dirty) {
                usbmidi_send_event(usbd_dev, enc->controller, enc->value);
                enc->dirty = false;
            }
        }
    }
}
