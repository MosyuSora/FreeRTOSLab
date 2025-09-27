/* Add the pragma while debugging. */
#pragma GCC optimize ("O0")

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <ctype.h>
#include "ht16k33.h"

#ifndef I2C_PORT
#define I2C_PORT    i2c0
#endif

#define HT16K33_ADDRESS 0x70

// How many digits are on our display.
#define NUM_DIGITS 4

// commands
#define HT16K33_SYSTEM_STANDBY  0x20
#define HT16K33_SYSTEM_RUN      0x21

#define HT16K33_SET_ROW_INT     0xA0

#define HT16K33_BRIGHTNESS      0xE0

// Display on/off/blink
#define HT16K33_DISPLAY_SETUP   0x80
// OR/clear these to display setup register
#define HT16K33_DISPLAY_OFF     0x0
#define HT16K33_DISPLAY_ON      0x1
#define HT16K33_BLINK_2HZ       0x2
#define HT16K33_BLINK_1HZ       0x4
#define HT16K33_BLINK_0p5HZ     0x6

#define a 1
#define b 2
#define c 4
#define d 8
#define e 16
#define f 32
#define g 64
#define DP 128

void ht16k33_clear_all(void);

// Converts a character to the bit pattern needed to display the right segments.
// These are pretty standard for 14segment LED's
uint16_t char_to_pattern(char ch) {
    // Map, "A" to "Z"
    int16_t alpha[] = {
    0xF7,0x128F,0x39,0x120F,0xF9,0xF1,0xBD,0xF6,0x1209,0x1E,0x2470,0x38,0x536,0x2136,
    0x3F,0xF3,0x203F,0x20F3,0x18D,0x1201,0x3E,0xC30,0x2836,0x2D00,0x1500,0xC09
    };

    // Map, "0" to "9"
    //int16_t num[] = {0xC3F,0x406,0xDB,0x8F,0xE6,0xED,0xFD,0x1401,0xFF,0xE7};
    int16_t num[] = {0x3F, 0x06, 0x5B, 0X4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

    if (isalpha(ch))
        return alpha[toupper(ch) - 'A'];
    
    if (isdigit(ch))
        return num[ch - '0'];
    
    if (ch == '-') {
        return (1 << 6);
    }

    return 0;
}
/*-----------------------------------------------------------*/

/* Quick helper function for single byte transfers */
void i2c_write_byte(uint8_t val, uint8_t address) {
    i2c_write_blocking(I2C_PORT, address, &val, 1, false);
}
/*-----------------------------------------------------------*/

void ht16k33_init() {
    i2c_write_byte(HT16K33_SYSTEM_RUN, HT16K33_ADDRESS);
    i2c_write_byte(HT16K33_SET_ROW_INT, HT16K33_ADDRESS);
    i2c_write_byte(HT16K33_DISPLAY_SETUP | HT16K33_DISPLAY_ON, HT16K33_ADDRESS);
    ht16k33_clear_all();
}
/*-----------------------------------------------------------*/

// Send a specific binary value to the specified digit
static inline void ht16k33_display_set(int position, uint16_t bin) {
    uint8_t buf[3];
    buf[0] = position * 2;
    buf[1] = bin & 0xff;
    buf[2] = bin >> 8;
    i2c_write_blocking(I2C_PORT, HT16K33_ADDRESS, buf, count_of(buf), false);

}
/*-----------------------------------------------------------*/

void ht16k33_display_char(int position, char ch) {
    ht16k33_display_set(position, char_to_pattern(ch));
}
/*-----------------------------------------------------------*/

void ht16k33_display_string(char *str) {
    int digit = 0;
    char* prev = NULL;

    while (*str && digit <= NUM_DIGITS) {

        if (*str == '.') {
            /* A decimal point is encoded into the previous character. 
               Special case if the decimal point is the first character of the string. */
            if (digit == 0 || *prev == ' ') {
                ht16k33_display_set(digit++, 0x80);
            } else {
                char tmp = char_to_pattern(*prev) | 0x80;
                ht16k33_display_set(digit - 1, tmp);
            }
            
        } else {
            ht16k33_display_char(digit++, *str);
        }

        prev = str;
        str++;
    }
}
/*-----------------------------------------------------------*/

void ht16k33_scroll_string(char *str, int interval_ms) {
    int l = strlen(str);

    if (l <= NUM_DIGITS) {
        ht16k33_display_string(str);
    }
    else {
        for (int i = 0; i < l - NUM_DIGITS + 1; i++) {
            ht16k33_display_string(&str[i]);
            sleep_ms(interval_ms);
        }
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief Set the brightness of the 7-segment display.
 * 
 * @param bright Brightness, between 0 and 15. Note that 0 is not 'off'. 
 */
void ht16k33_set_brightness(int bright) {
    i2c_write_byte(HT16K33_BRIGHTNESS | (bright <= 15 ? bright : 15), HT16K33_ADDRESS);
}
/*-----------------------------------------------------------*/

void ht16k33_set_blink(int blink) {
    int s = 0;
    switch (blink) {
        default: break;
        case 1: s = HT16K33_BLINK_2HZ; break;
        case 2: s = HT16K33_BLINK_1HZ; break;
        case 3: s = HT16K33_BLINK_0p5HZ; break;
    }

    i2c_write_byte(HT16K33_DISPLAY_SETUP | HT16K33_DISPLAY_ON | s, HT16K33_ADDRESS);
}
/*-----------------------------------------------------------*/

void ht16k33_clear_all() {
    ht16k33_display_set(0, 0);
    ht16k33_display_set(1, 0);
    ht16k33_display_set(2, 0);
    ht16k33_display_set(3, 0);
    return;
}
/*-----------------------------------------------------------*/