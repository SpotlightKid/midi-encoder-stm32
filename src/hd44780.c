/*
 * HD44780 display controller library for linopencm3
 *
 *
 *
*/

#include <assert.h>
#include <stdbool.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "hd44780.h"
#include "sleep.h"


void init_HD44780(HD44780 * lcd) {
    lcd->port = GPIOD;
    lcd->pin_RS = GPIO2;
    lcd->pin_EN = GPIO1;
    lcd->pin_D4 = GPIO6;
    lcd->pin_D5 = GPIO5;
    lcd->pin_D6 = GPIO4;
    lcd->pin_D7 = GPIO3;
    lcd->width = 16;
    lcd->lines = 2;
    lcd->displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    lcd->displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    lcd->display_functions = LCD_4BITMODE | LCD_1LINE | LCD_5X8DOTS;
    lcd->row_offsets[0] = 0x00;
    lcd->row_offsets[1] = 0x40;
    lcd->row_offsets[2] = 0x14;
    lcd->row_offsets[3] = 0x54;
};


/*
 * Initialize the display.
 *
 *  When the display powers up, it is configured as follows:
 *
 * 1. Display clear
 * 2. Function set:
 *    DL = 1 --> 8-bit interface data
 *    N =  0 --> 1-line display
 *    F =  0 --> 5x8 dot character font
 * 3. Display on/off control:
 *    D = 0 --> Display off
 *    C = 0 --> Cursor off
 *    B = 0 --> Blinking off
 * 4. Entry mode set:
 *    I/D = 1 --> Increment by 1
 *    S =   0 --> No shift
 *
 */
void lcd_init(HD44780* lcd) {
    usleep(50000);

    // enable periphal clock for GPIO port where LCD lines are attached
    rcc_periph_clock_enable(lcd->port);

    // set up GPIO pins on that port
    gpio_mode_setup(lcd->port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
                    lcd->pin_RS | lcd->pin_EN | lcd->pin_D4 | lcd->pin_D5 |
                    lcd->pin_D6 | lcd->pin_D7);

    // Pull RS and EN low to begin commands
    gpio_clear(lcd->port, lcd->pin_RS | lcd->pin_EN);

    // Put the LCD into 4 bit or 8 bit mode.
    // This is according to the hitachi HD44780 datasheet figure 24, p. 46
    // We start in 8bit mode, try to set 4 bit mode.
    lcd_send_nibble(lcd, 0x03);
    usleep(40);

    // Second try
    lcd_send_nibble(lcd, 0x03);
    usleep(20);

    // Third go!
    lcd_send_nibble(lcd, 0x03);
    usleep(20);

    // Finally, set to 4-bit interface
    lcd_send_nibble(lcd, 0x02);

    // Finally, set # lines, font size, etc.

    if (lcd->lines > 1)
        lcd->display_functions |= LCD_2LINE;

    lcd_send_command(lcd, LCD_FUNCTIONSET | lcd->display_functions);

    // Turn the display on with no cursor or blinking default
    lcd_send_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);

    // Clear the display
    lcd_clear(lcd);

    // Set the entry mode; initialize to default text direction
    // (for romanic languages)
    lcd_send_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}


void lcd_clear(HD44780* lcd) {
    /* Clear the display. */
    lcd_send_command(lcd, LCD_CLEARDISPLAY);
    usleep(2000);
}


void lcd_home(HD44780* lcd) {
    lcd_send_command(lcd, LCD_RETURNHOME);
    usleep(2000);
}


void lcd_set_display(HD44780* lcd, bool enable) {
    /* Turn the display on or off. */
    if (enable)
        lcd->displaycontrol |= LCD_DISPLAYON;
    else
        lcd->displaycontrol &= ~LCD_DISPLAYON;

    lcd_send_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}


void lcd_set_cursor(HD44780* lcd, bool enable) {
    /* Turn the underscore cursor on or off. */
    if (enable)
        lcd->displaycontrol |= LCD_CURSORON;
    else
        lcd->displaycontrol &= ~LCD_CURSORON;

    lcd_send_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}


void lcd_set_blink(HD44780* lcd, bool enable) {
    /* Turn the blinking cursor on or off. */
    if (enable)
        lcd->displaycontrol |= LCD_BLINKON;
    else
        lcd->displaycontrol &= ~LCD_BLINKON;

    lcd_send_command(lcd, LCD_DISPLAYCONTROL | lcd->displaycontrol);
}


void lcd_set_scroll(HD44780* lcd, uint8_t direction) {
    assert(direction == LCD_MOVERIGHT || direction == LCD_MOVELEFT);

    /* Scroll the display without changing the RAM. */
    lcd_send_command(lcd, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | direction);
}


void lcd_set_direction(HD44780* lcd, uint8_t direction) {
    assert(direction == LCD_ENTRYRIGHT || direction == LCD_ENTRYLEFT);

    /* Set text flow direction to left-to-right or right-to-left. */
    if (direction == LCD_ENTRYLEFT)
        lcd->displaymode |= LCD_ENTRYLEFT;
    else
        lcd->displaymode &= ~LCD_ENTRYLEFT;

    lcd_send_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}


/*
 * Turn autoscrolling on or off.
 *
 * When autoscrolling is on, when a character is written, the existing
 * characters to the right of it (or left, depending on the text flow
 * direction set with the ``direction()`` method), are scrolled one
 * position to the right reps. left, to make space for it.
 *
 */
void lcd_set_autoscroll(HD44780* lcd, bool enable) {
    if (enable)
        lcd->displaymode |= LCD_ENTRYSHIFTINCREMENT;
    else
        lcd->displaymode &= ~LCD_ENTRYSHIFTINCREMENT;

    lcd_send_command(lcd, LCD_ENTRYMODESET | lcd->displaymode);
}


void lcd_place_cursor(HD44780* lcd, uint8_t col, uint8_t row) {
    /* Set the cursor the given column and row. */
    lcd_send_command(lcd, LCD_SETDDRAMADDR | (col + lcd->row_offsets[row]));
}


/*
 * Send a byte to the data pins.
 *
 * byte = data
 * mode = LCD_CMD  for command
 *        LCD_CHAR for character
 *
 */
void lcd_send_byte(HD44780* lcd, char byte, uint8_t mode) {
    assert(mode == LCD_CHR || mode == LCD_CMD);

    if (mode == LCD_CMD)
        gpio_clear(lcd->port, lcd->pin_RS);
    else
        gpio_set(lcd->port, lcd->pin_RS);

    lcd_send_nibble(lcd, byte >> 4);
    lcd_send_nibble(lcd, byte);
}


void lcd_send_command(HD44780* lcd, char command) {
    lcd_send_byte(lcd, command, LCD_CMD);
}


/* Write message to given row. */
void lcd_write(HD44780* lcd, char* message) {
    while (*message)
        lcd_send_byte(lcd, *message++, LCD_CHR);
}


/*
 * Create a custom character in given memory location.
 *
 * Allows us to fill the first 8 CGRAM locations with custom characters.
 * There are 8 locations, 0-7
 *
 */
void lcd_create_char(HD44780* lcd, uint8_t location, char* charmap) {
    lcd_send_command(lcd, LCD_SETCGRAMADDR | ((location & 0x7) << 3));

    while(*charmap)
        lcd_send_byte(lcd, *charmap++, LCD_CHR);
}


/* internal helper methods */
/* Pulse the EN pin, by setting it low, then high, then low again. */
void lcd_pulse_enable(HD44780* lcd) {
    gpio_clear(lcd->port, lcd->pin_EN);
    usleep(E_PULSE);
    gpio_set(lcd->port, lcd->pin_EN);
    usleep(E_PULSE);
    gpio_clear(lcd->port, lcd->pin_EN);
    usleep(E_DELAY);
}



/* Send a nibble (4 bits) by setting data pins D4-7 and pulsing EN. */
void lcd_send_nibble(HD44780* lcd, char nibble) {
    if (nibble & 1)
        gpio_set(lcd->port, lcd->pin_D4);
    else
        gpio_clear(lcd->port, lcd->pin_D4);

    if (nibble >> 1 & 1)
        gpio_set(lcd->port, lcd->pin_D5);
    else
        gpio_clear(lcd->port, lcd->pin_D5);

    if (nibble >> 2 & 1)
        gpio_set(lcd->port, lcd->pin_D6);
    else
        gpio_clear(lcd->port, lcd->pin_D6);

    if (nibble >> 3 & 1)
        gpio_set(lcd->port, lcd->pin_D7);
    else
        gpio_clear(lcd->port, lcd->pin_D7);

    lcd_pulse_enable(lcd);
}

