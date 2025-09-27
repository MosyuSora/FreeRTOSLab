#ifndef HT16K33_H
#define HT16K33_H

void ht16k33_init();

void ht16k33_display_string(char *str);

void ht16k33_scroll_string(char *str, int interval_ms);

void ht16k33_set_brightness(int bright);

void ht16k33_set_blink(int blink);

void ht16k33_clear_all();

void ht16k33_display_char(int position, char ch);

#endif /* HT16K33_H */