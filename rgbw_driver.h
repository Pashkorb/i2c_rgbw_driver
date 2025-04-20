#pragma once

#include <stdint.h>

/* Driver error codes */
#define RGBW_OK                      0   // Success
#define RGBW_ERROR_SB               -1   // Start condition not generated (I2C bus issue)
#define RGBW_ERROR_TXE              -2   // Data register not ready for transmission
#define RGBW_ERROR_ADDR             -3   // Address transmission failed
#define RGBW_ERROR_CHANNEL_DISABLE  -4   // Attempt to set disabled channel
#define RGBW_ERROR_CHANNEL_RANGE    -5   // Invalid channel specified
#define RGBW_ERROR_TRANSFER         -6   // Data transfer not completed
#define RGBW_ERROR_ACK_FAIL        -7   // No acknowledgment received
#define RGBW_ERROR_REINIT          -8   // Reinitialization attempted while active
#define RGBW_ERROR_RXNE            -9   // Receive buffer not ready
#define RGBW_ERROR_BUS_BUSY        -10  // I2C bus unavailable (timeout)
#define RGBW_ERROR_NOT_INIT        -11  // Driver not initialized
#define RGBW_ERROR_BUS_ERROR       -12  // I2C bus error (signal fault)
#define RGBW_ERROR_ARB_LOST       -13  // I2C arbitration lost (multi-master conflict)

/* Default configuration values */
#define RGBW_DEFAULT_ADDR      0x44
#define RGBW_DEFAULT_RED_REG   0x02
#define RGBW_DEFAULT_GREEN_REG 0x03
#define RGBW_DEFAULT_BLUE_REG  0x04
#define RGBW_DEFAULT_WHITE_REG 0x05
#define RGBW_USE_DEFAULT       0xFF  // Use default register value

/**
  * @brief  Initialize RGBW driver
  * @param  i2c_addr: I2C address (use RGBW_USE_DEFAULT for 0x44)
  * @param  reg_red_addr: Red PWM register (use RGBW_USE_DEFAULT for 0x02)
  * @param  reg_green_addr: Green PWM register (use RGBW_USE_DEFAULT for 0x03)
  * @param  reg_blue_addr: Blue PWM register (use RGBW_USE_DEFAULT for 0x04)
  * @param  reg_white_addr: White PWM register (use RGBW_USE_DEFAULT for 0x05)
  * @retval RGBW_OK on success, error code otherwise
  */
int rgbw_init(uint8_t i2c_addr, uint8_t reg_red_addr, uint8_t reg_green_addr,
              uint8_t reg_blue_addr, uint8_t reg_white_addr);

/**
  * @brief  Set RGBW color values
  * @param  r: Red intensity (0-255)
  * @param  g: Green intensity (0-255)
  * @param  b: Blue intensity (0-255)
  * @param  w: White intensity (0-255)
  * @param  error_mask: Optional bitmask for per-channel errors (LSB-first: bit0-red, bit1-green, etc)
  * @retval RGBW_OK on success, first error code encountered otherwise
  */
int rgbw_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t *error_mask);

// Individual color control functions
int rgbw_set_red(uint8_t r);
int rgbw_set_green(uint8_t g);
int rgbw_set_blue(uint8_t b);
int rgbw_set_white(uint8_t w);

// Channel control functions
int rgbw_disable_red(void);
int rgbw_disable_green(void);
int rgbw_disable_blue(void);
int rgbw_disable_white(void);
int rgbw_enable_red(void);
int rgbw_enable_green(void);
int rgbw_enable_blue(void);
int rgbw_enable_white(void);

/**
  * @brief  Enable all channels in PWM mode
  * @retval RGBW_OK on success, error code otherwise
  */
int rgbw_enable_all(void);

/**
  * @brief  Disable all channels
  * @retval RGBW_OK on success, error code otherwise
  */
int rgbw_disable_all(void);

/**
  * @brief  Read LEDOUT register
  * @param  ledout: Pointer to store the LEDOUT register value (0-255)
  * @return RGBW_OK on success, error code otherwise
  */
int8_t rgbw_read_ledout(uint8_t *ledout);

/**
  * @brief  Deinitialize driver and release resources
  * @retval RGBW_OK on success, error code otherwise
  */
int rgbw_deinit(void);

/**
  * @brief  Check driver initialization status
  * @retval 1 if initialized, 0 otherwise
  */
int rgbw_is_initialized(void);
