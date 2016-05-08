/*
 * HD44780 display controller library for linopencm3
 *
 */

#ifndef HD44780_H
#define HD44780_H

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>


/* Timing constants */
#define E_PULSE 1
#define E_DELAY 50

/* Command / character mode */
#define LCD_CHR 1
#define LCD_CMD 0

/* LCD commands */
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

/* flags for display entry mode */
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

/* flags for display on/off control */
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

/* flags for display/cursor shift */
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

/* flags for function set */
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5X10DOTS 0x04
#define LCD_5X8DOTS 0x00


/* Interface to a HD44780 LCD controller in 4-bit mode. */
struct HD44780 {
    uint32_t port;
    uint8_t pin_RS;
    uint8_t pin_EN;
    uint8_t pin_D4;
    uint8_t pin_D5;
    uint8_t pin_D6;
    uint8_t pin_D7;
    // Maximum characters per line
    uint8_t width;
    // Number of display rows
    uint8_t lines;
    // State of display on/off, underscore & blinking cursor control
    uint8_t displaycontrol;
    // State of text flow direction and auto-scrolling
    uint8_t displaymode;
    uint8_t display_functions;
    // memory offsets for display lines
    uint8_t row_offsets[4];
};

struct HD44780 HD44780_Init = {
    .port = GPIOD,
    .pin_RS = GPIO2,
    .pin_EN = GPIO1,
    .pin_D4 = GPIO6,
    .pin_D5 = GPIO5,
    .pin_D6 = GPIO4,
    .pin_D7 = GPIO3,
    .width = 16,
    .lines = 2,
    .displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF,
    .displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT,
    .display_functions = LCD_4BITMODE | LCD_1LINE | LCD_5X8DOTS,
    .row_offsets = {0x00, 0x40, 0x14, 0x54}
};


void lcd_clear(struct HD44780* lcd);
void lcd_create_char(struct HD44780* lcd, uint8_t location, char* charmap);
void lcd_home(struct HD44780* lcd);
void lcd_init(struct HD44780* lcd);
void lcd_place_cursor(struct HD44780* lcd, uint8_t col, uint8_t row);
void lcd_pulse_enable(struct HD44780* lcd);
void lcd_send_byte(struct HD44780* lcd, char byte, uint8_t mode);
void lcd_send_command(struct HD44780* lcd, char command);
static void lcd_send_nibble(struct HD44780* lcd, char nibble);
void lcd_set_autoscroll(struct HD44780* lcd, bool enable);
void lcd_set_blink(struct HD44780* lcd, bool enable);
void lcd_set_cursor(struct HD44780* lcd, bool enable) ;
void lcd_set_direction(struct HD44780* lcd, uint8_t direction);
void lcd_set_display(struct HD44780* lcd, bool enable);
void lcd_set_scroll(struct HD44780* lcd, uint8_t direction);
void lcd_write(struct HD44780* lcd, char* message);

#endif /* HD44780_H */
