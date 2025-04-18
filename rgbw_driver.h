#pragma once
#include <stdint.h>

// initialize driver
int rgbw_init(void);

// Set color in format rgbw (in the range of (0-255))
int rgbw_set_color(uint8_t r,uint8_t g,uint8_t b,uint8_t w);

// enable/disable all channel
int rgb_enable_all(void);
int rgb_disable_all(void);

//channel management
int rgbw_set_channel(uint8_t channel, uint8_t brightness);
