#ifndef MIDI_ENCODER_H
#define MIDI_ENCODER_H

#include <stdint.h>

static void systick_setup(void);
static void msleep(uint32_t delay);
static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);
static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue);
static void usbmidi_send_event(usbd_device *usbd_dev, uint8_t ctrl, uint8_t value);

#endif  /* MIDI_ENCODER_H */
