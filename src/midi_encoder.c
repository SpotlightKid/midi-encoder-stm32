/*
 * Copyright (C) 2016 Christopher Arndt <chris@chrisarndt.de>
 *
 */

#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>

#include "descriptors.h"
#include "rotary_encoder.h"
#include "midi_encoder.h"
#include "hd44780.h"
#include "sleep.h"

/* monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering
 */
volatile uint32_t system_millis;

static char usb_serial_number[25]; /* 12 bytes of desig and a \0 */

static const char * usb_strings[] = {
    "libopencm3.org",
    "MIDI demo",
    usb_serial_number
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

/* array of rotary encoder structs */
#define NUM_ENCODERS 2
RotaryEncoder encoders[NUM_ENCODERS];

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

/* Called when systick fires */
void sys_tick_handler(void) {
    system_millis++;
    read_encoders(encoders, NUM_ENCODERS);
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

/* sleep for delay milliseconds */
static void msleep(uint32_t delay) {
    uint32_t wake = system_millis + delay;
    while (wake > system_millis);
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
    HD44780 lcd;

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
    init_encoder(&encoders[0]);
    encoders[0].port = GPIOA;
    encoders[0].pin_a = 0;
    encoders[0].pin_b = 1;
    encoders[0].controller = 1;

    init_encoder(&encoders[1]);
    encoders[1].port = GPIOA;
    encoders[1].pin_a = 2;
    encoders[1].pin_b = 3;
    encoders[1].controller = 7;

    init_HD44780(&lcd);
    lcd_init(&lcd);

    lcd_clear(&lcd);
    lcd_write(&lcd, " MIDI Encoders  ");

    /* Start systick timer */
    systick_setup();

    msleep(2000);

    for (uint8_t i = 0; i < 40; i++) {
        lcd_scroll(&lcd, LCD_MOVERIGHT);
        msleep(250);
    }

    /* event loop */
    while (1) {
        usbd_poll(usbd_dev);

        for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
            if (encoders[i].dirty) {
                usbmidi_send_event(usbd_dev, encoders[i].controller,
                                   encoders[i].value);
                encoders[i].dirty = false;
            }
        }
    }
}
