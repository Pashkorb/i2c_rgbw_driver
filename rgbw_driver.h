#pragma once
#include <stdint.h>

/*  
    @brief initialization function
    @param i2c_addr - I2C address of the RGBW controller (default: 0x44)
    @param reg_red_addr - register address for red channel (default: 0x02)
    @param reg_green_addr - register address for green channel (default: 0x03)
    @param reg_blue_addr - register address for blue channel (default: 0x04)
    @param reg_white_addr - register address for white channel (default: 0x05)
    @return 0 on success, or an error code on failure.
     
*/
int rgbw_init(uint8_t i2c_addr, uint8_t reg_red_addr, uint8_t reg_green_addr,
    uint8_t reg_blue_addr,uint8_t reg_white_addr);

/*
    @brief Set color in format rgbw (in the range of (0-255))
    @param r - red channel value (0-255)
    @param g - green channel value (0-255)
    @param b - blue channel value (0-255)
    @param w - white channel value (0-255)
    @param error_mask - pointer to a variable to store the error mask (optional)
    @return 0 on success, or an error code on failure.
*/
int rgbw_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t *error_mask);




/*
    @brief Set brightness of a specific channel (0-255)
    @param channel - channel to set (REG_RED, REG_GREEN, REG_BLUE, REG_WHITE)
    @return 0 on success, or an error code on failure.
*/
int rgbw_set_channel(uint8_t channel, uint8_t brightness){
    uint8_t offset = find_offset(channel);

/*
    @brief set brightness of a red channel (0-255)
    @param r - red channel value (0-255)
    @return 0 on success, or an error code on failure.
*/
int rgbw_set_red(uint8_t r);

/*
    @brief set brightness of a green channel (0-255)
    @param g - green channel value (0-255)
    @return 0 on success, or an error code on failure.
*/
int rgbw_set_green(uint8_t g);

/*
    @brief set brightness of a blue channel (0-255)
    @param b - blue channel value (0-255)
    @return 0 on success, or an error code on failure.
*/
int rgbw_set_blue(uint8_t b);

/*
    @brief set brightness of a blue channel (0-255)
    @param w - white channel value (0-255)
    @return 0 on success, or an error code on failure.
*/
int rgbw_set_white(uint8_t w);

/*
    @brief disable red channel
    @return 0 on success, or an error code on failure.
*/
int rgbw_disable_red(void);

/*
    @brief disable green channel
    @return 0 on success, or an error code on failure.
*/
int rgbw_disable_green(void);

/*
    @brief disable blue channel
    @return 0 on success, or an error code on failure.
*/
int rgbw_disable_blue(void);

/*
    @brief disable white channel
    @return 0 on success, or an error code on failure.
*/
int rgbw_disable_white(void);

/*
    @brief enable red channel
    @return 0 on success, or an error code on failure.
*/
int rgbw_enable_red(void);

/*
    @brief enable green channel
    @return 0 on success, or an error code on failure.
*/ 
int rgbw_enable_green(void);

/*
    @brief enable blue channel
    @return 0 on success, or an error code on failure.
*/ 
int rgbw_enable_blue(void);

/*
    @brief enable white channel
    @return 0 on success, or an error code on failure.
*/ 
int rgbw_enable_white(void);

/*
    @brief enavle all channels
    @return 0 on success, or an error code on failure.
*/ 
int rgbw_enable_all(void);

/*
    @brief disable all channels
    @return 0 on success, or an error code on failure.
*/
int rgbw_disable_all(void);

/*
    @brief get current LEDOUT value
    @return current LEDOUT value
    @note This function is used to read the current LEDOUT value from the RGBW controller.
    It returns the current LEDOUT value as an integer.
*/
int rgbw_read_ledout(void);

/*
    @brief deinitialize the RGBW controller
    @return 0 on success, or an error code on failure.
*/
int rgbw_deinit(void);

/*
    @brief check if the RGBW controller is initialized
    @return 1 if initialized, 0 otherwise
*/
int rgbw_is_initialized(void);
