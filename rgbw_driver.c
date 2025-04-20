#include "rgbw_driver.h"
#include <stdint.h>

// Driver errors
#define ERROR_SB -1
#define ERROR_TXE -2
#define ERROR_ADDR -3
#define ERROR_CHANNEL_DISABLE -4
#define ERROR_CHANNEL_NOT_IN_RANGE -5
#define ERROR_TRANSFER -6
#define ERROR_ACK_FAIL -7
#define ERROR_REINITIALIZATION -8
#define ERROR_RXNE -9
#define ERROR_BUSY_BUS -10
#define ERROR_NOT_INITIALIZED -11
#define ERROR_BUS_ERROR -12
#define ERROR_ARBITRATION_LOST -13

/* I2C Control Register 1 (CR1) */
#define I2C_CR1_START       (1 << 8)   // Bit 8: START generation 
#define I2C_CR1_STOP        (1 << 9)   // Bit 9: STOP generation 
#define I2C_CR1_ACK         (1 << 10)  // Bit 10: Acknowledge enable 
#define I2C_CR1_PE          (1 << 0)   // Bit 0: Peripheral enable

/* I2C Status Register 1 (SR1) */
#define I2C_SR1_SB          (1 << 0)   // Bit 0: Start bit (Master mode) 
#define I2C_SR1_ADDR        (1 << 1)   // Bit 1: Address sent (master)/matched (slave)
#define I2C_SR1_BTF         (1 << 2)   // Bit 2: Byte Transfer Finished 
#define I2C_SR1_RXNE        (1 << 6)   // Bit 6: Receive data register not empty 
#define I2C_SR1_TXE         (1 << 7)   // Bit 7: Transmit data register empty 
#define I2C_SR1_BERR        (1 << 8)   // Bit 8: Bus error
#define I2C_SR1_ARLO        (1 << 9)   // Bit 9: Arbitration lost (master mode)
#define I2C_SR1_ACK_FAIL    (1 << 10)  // Bit 10: Acknowledge failure 

/* I2C Status Register 2 (SR2) */
#define I2C_SR2_BUSY        (1 << 1)   /* Bit 1: Bus busy */

// base addresses for peripherals
#define RCC_BASE 0x40023800 // RCC register base address
#define I2C1_BASE 0x40005400 // I2C1 register base address
#define GPIOB_BASE 0x40020400 // GPIOB register base address
#define REG_LEDOUT 0x08 // LED output control register

// I2C timeout value
#define I2C_TIMEOUT 100000 // I2C operation timeout counter

// Channel control modes in the LEDOUT register
#define LED_MODE_OFF     0x00  // Channel disabled (LED off)
#define LED_MODE_PWM     0x02  // Channel in PWM control mode (LED on)

// Configuration LEDOUT (for 4 channels R, G, B, W)
#define LEDOUT_ALL_OFF   0x00   // 0b00000000 - all channels disabled
#define LEDOUT_ALL_PWM   0xAA   // 0b10101010 - all channels in PWM mode

// configuration I2C Control and Status Registers
#define I2C1_CR1 (*(volatile uint32_t*)(I2C1_BASE+0x00)) // I2C Control Register 1
#define I2C1_CR2 (*(volatile uint32_t*)(I2C1_BASE+0x04)) // I2C Control Register 2
#define I2C_CCR (*(volatile uint32_t*)(I2C1_BASE+0x1C)) // Clock Control Register
#define I2C_TRISE (*(volatile uint32_t*)(I2C1_BASE+0x20)) // Rise Time Register
#define I2C1_SR1 (*(volatile uint32_t*)(I2C1_BASE+0x14)) // Status Register 1
#define I2C1_SR2 (*(volatile uint32_t*)(I2C1_BASE+0x18)) // Status Register 2
#define I2C1_DR (*(volatile uint32_t*)(I2C1_BASE+0x10)) // Data Register

// I2C Timing Configuration
#define I2C_FREQ_MHZ 36                                               // Core clock frequency for I2C1 (MHz)
#define I2C_SPEED_HZ 100000                                           // I2C bus speed (100kHz standard mode)
#define I2C_CCR_VAL ((I2C_FREQ_MHZ * 1000000U) / (2 * I2C_SPEED_HZ))  // Clock control value
#define CALC_I2C_TRISE (I2C_FREQ_MHZ + 1) // Rise time calculation (max 1000ns)

// Reset and Clock Control (RCC) Registers
#define RCC_APB1ENR (*(volatile uint32_t*)(RCC_BASE + 0x40)) // APB1 Peripheral Clock Enable
#define RCC_AHB1ENR (*(volatile uint32_t*)(RCC_BASE + 0x30)) // AHB1 Peripheral Clock Enable

//configuration of GPIO port B pins
#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00)) // Port Mode Register
#define GPIOB_OTYPER    (*(volatile uint32_t*)(GPIOB_BASE + 0x04)) // Output Type Register
#define GPIOB_OSPEEDR   (*(volatile uint32_t*)(GPIOB_BASE + 0x08)) // Output Speed Register
#define GPIOB_PUPDR     (*(volatile uint32_t*)(GPIOB_BASE + 0x0C)) // Pull-up/Pull-down Register
#define GPIOB_AFRL      (*(volatile uint32_t*)(GPIOB_BASE + 0x20)) // Alternate Function Low Register

#define RGBW_USE_DEFAULT 0xFF // Default value for uninitialized registers

// Default I2C device address and register mapping
static uint8_t LED_I2C_ADDR =   0x44; // Default I2C slave address (7-bit)
static uint8_t REG_RED      =   0x02; // Red channel PWM register
static uint8_t REG_GREEN    =   0x03; // Green channel PWM register
static uint8_t REG_BLUE     =   0x04; // Blue channel PWM register
static uint8_t REG_WHITE    =   0x05; // White channel PWM register

// Register LEDOUT
static uint8_t ledout =0x00; // Initialized to all channels off

static int is_initialized = 0; // Initialization status flag

static inline int wait_flag(uint32_t flag, uint32_t *reg, int8_t error_code){
    int timer = I2C_TIMEOUT;
    while (!(*reg & flag) && (timer-- > 0)) {
        if (timer == 0) {
            return error_code; // Timeout error
        }
    }
    return 0; // Success
}
static inline int wait_flag_clear(uint32_t flag, uint32_t *reg, int8_t error_code){
    int timer = I2C_TIMEOUT;
    while ((*reg & flag) && (timer-- > 0)) {
        if (timer == 0) {
            return error_code; // Timeout error
        }
    }
    return 0; // Success
}

// inline function to set brightness for a specific channel
static inline int i2c_set_brightness_for_channel(uint8_t channel, uint8_t brightness){
    if(!is_initialized){
        return ERROR_NOT_INITIALIZED; 
    }

    int8_t offset = find_offset(channel);
    if (offset < 0) {
        return offset; 
    }
    uint8_t mode = (ledout >> offset) & 0x03;

    if (mode != LED_MODE_PWM)
        return ERROR_CHANNEL_DISABLE;
    uint8_t send_data[1]={brightness};
    uint8_t addrs[1]={channel};
    return _i2c_write_reg(send_data,NULL, 1, addrs);
}

// inline function to enable a specific channel
static inline int i2c_enable_channel(uint8_t channel) {
    if(!is_initialized){
        return ERROR_NOT_INITIALIZED; 
    }
    int8_t offset = find_offset(channel);
    if (offset < 0) {
        return ERROR_CHANNEL_NOT_IN_RANGE; 
    }
    uint8_t new_ledout = ledout | (0x02 << offset); // Set the channel to PWM mode
    uint8_t send_data[1] = {new_ledout};
    uint8_t addrs[1] = {REG_LEDOUT};
    
    int8_t ret= _i2c_write_reg(send_data, NULL, 1, addrs);
    if(ret == 0){
        ledout = new_ledout; // Update the ledout variable
    }
    return ret;
}
 
// inline function to disable a specific channel
static inline int i2c_disable_channel(uint8_t channel) {
    if(!is_initialized){
        return ERROR_NOT_INITIALIZED; 
    }
    int8_t offset = find_offset(channel);
    if (offset < 0) {
        return ERROR_CHANNEL_NOT_IN_RANGE; 
    }
    uint8_t new_ledout = ledout & ~(0x03 << offset); // Set the channel to OFF mode
    uint8_t send_data[1] = {new_ledout};
    uint8_t addrs[1] = {REG_LEDOUT};
    int8_t ret= _i2c_write_reg(send_data, NULL, 1, addrs);
    if(ret == 0){
        ledout = new_ledout; // Update the ledout variable
    }
    return ret;
}

//find offset of channel
// channel - channel to find offset for (REG_RED, REG_GREEN, REG_BLUE, REG_WHITE)
static inline int find_offset (uint32_t channel){
    if (channel == REG_RED) {
        return 0;
    } else if (channel == REG_GREEN) {
        return 2;
    } else if (channel == REG_BLUE) {
        return 4;
    } else if (channel == REG_WHITE) {
        return 6;
    } else {
        return ERROR_CHANNEL_NOT_IN_RANGE; 
    }
}

// Send STOP in I2C bus
static inline int i2c_generate_stop(void) {
    I2C1_CR1 |= I2C_CR1_STOP;

    if(I2C1_SR1 & I2C_SR1_BERR) { // Check for bus error
        I2C1_SR1 &= ~I2C_SR1_BERR; // Clear bus error flag
        return ERROR_BUS_ERROR; 
    }

    if (I2C1_SR1 & I2C_SR1_ARLO) {
        I2C1_SR1 &= ~I2C_SR1_ARLO;
        return ERROR_ARBITRATION_LOST; 
    }

    return 0; // Success
}

// Send START in I2C bus
static int i2c_generate_start(void) {
    I2C1_CR1 &= ~I2C_CR1_START;
    // Check for bus not busy
    int ret =0;
    ret = wait_flag_clear(I2C_SR2_BUSY, &I2C1_SR2, ERROR_BUSY_BUS); // Wait for bus not busy
    if (ret != 0) {
        return ret; 
    }

    // Generation START condition
    I2C1_CR1 |= I2C_CR1_START; // Старт-бит

    // Start Bit await
    ret = wait_flag(I2C_SR1_SB, &I2C1_SR1, ERROR_SB); // Wait for start bit
    if (ret != 0) {
        return ret; 
    }

    if(I2C1_SR1 & I2C_SR1_BERR) { // Check for bus error
        I2C1_SR1 &= ~I2C_SR1_BERR; // Clear bus error flag
        return ERROR_BUS_ERROR; 
    }

    if (I2C1_SR1 & I2C_SR1_ARLO) {
        I2C1_SR1 &= ~I2C_SR1_ARLO;
        return ERROR_ARBITRATION_LOST; 
    }

    return 0; // Success
    
}
// Send multiple data to I2C bus
static int8_t i2c_send_multiple_data(uint8_t data[],uint8_t *error_mask, uint8_t count, uint8_t start_addr[]){
    int first_error = 0; 
    I2C1_DR = LED_I2C_ADDR << 1; // write bit
    uint8_t mask = 0;
    int ret =0;
    // ADDR  flag await
    ret = wait_flag(I2C_SR1_ADDR, &I2C1_SR1, ERROR_ADDR); // Wait for ADDR flag
    if (ret != 0) {
        return ret; 
    }
    (void)I2C1_SR1; 
    (void)I2C1_SR2; 

    // Check for ACK Failure 
    if (I2C1_SR1 & I2C_SR1_ACK_FAIL) { I2C1_SR1 &= ~I2C_SR1_ACK_FAIL; return ERROR_ACK_FAIL; }

    for(uint8_t i=0; i<count; i++){
        // Send reg address
        ret = wait_flag(I2C_SR1_TXE, &I2C1_SR1, ERROR_TXE); // Wait for TXE flag
        if (ret != 0) {
            first_error= ERROR_TXE;
            mask |= (1 << i);
            break;
        }

        I2C1_DR = start_addr[i];

        if(I2C1_SR1 & I2C_SR1_BERR) { // Check for bus error
            I2C1_SR1 &= ~I2C_SR1_BERR; // Clear bus error flag
            first_error= ERROR_BUS_ERROR;
            mask |= (1 << i);
            break;
        }

        if (I2C1_SR1 & I2C_SR1_ARLO) {
            I2C1_SR1 &= ~I2C_SR1_ARLO;
            first_error= ERROR_ARBITRATION_LOST;
            mask |= (1 << i);
            break;
        }

        if (I2C1_SR1 & I2C_SR1_ACK_FAIL) {
            I2C1_SR1 &= ~I2C_SR1_ACK_FAIL;
            first_error= ERROR_ACK_FAIL;
            mask |= (1 << i);
            break;
        }
        // Data transfer readiness await
        ret = wait_flag(I2C_SR1_TXE, &I2C1_SR1, ERROR_TXE); // Wait for TXE flag
        if (ret != 0) {
            first_error= ERROR_TXE;
            mask |= (1 << i);
            break;
        }

        // Send value
        I2C1_DR = data[i];

        if (I2C1_SR1 & I2C_SR1_ACK_FAIL) {
            I2C1_SR1 &= ~I2C_SR1_ACK_FAIL;
            first_error= ERROR_ACK_FAIL;
            mask |= (1 << i);
            break;
        }

        // Wait for the transfer to complete 
        ret = wait_flag(I2C_SR1_BTF, &I2C1_SR1, ERROR_TRANSFER); // Wait for BTF flag
        if (ret != 0) {
            first_error= ERROR_TRANSFER;
            mask |= (1 << i);
            break;
        }
    (void)I2C1_SR1;
    }

    if (error_mask) {
        *error_mask = mask;
    }
    return first_error;

}

static int _i2c_write_reg(uint8_t value[], uint8_t *error_mask, uint8_t count, uint8_t start_addr[]) {
    int ret =i2c_generate_start();
    if(ret!=0){
        return ret;
    }
    ret = i2c_send_multiple_data(value, error_mask, count, start_addr);
    
    ret=i2c_generate_stop(); 
    return ret;
}

static int8_t _i2c_read_reg_ledout(void) {
    int ret =0;
    if(!is_initialized){
        ret = ERROR_NOT_INITIALIZED; 
        return ret;
    }
    
    (void)I2C1_SR1; 
    (void)I2C1_SR2; // Clear ADDR flag

    ret = wait_flag_clear(I2C_SR2_BUSY, &I2C1_SR2, ERROR_BUSY_BUS); // Wait for bus not busy
    
    if (ret != 0) {
        return ret; 
    }
    

    I2C1_CR1 |= I2C_CR1_START; 
    
    ret = wait_flag(I2C_SR1_SB, &I2C1_SR1, ERROR_SB); // Wait for start bit
    if (ret != 0) {
        return ret; 
    }

    I2C1_DR= LED_I2C_ADDR<<1 | 0; // write bit

    if(I2C1_SR1 & I2C_SR1_BERR) { // Check for bus error
        I2C1_SR1 &= ~I2C_SR1_BERR; // Clear bus error flag
        return ERROR_BUS_ERROR; 
    }

    if (I2C1_SR1 & I2C_SR1_ARLO) {
        I2C1_SR1 &= ~I2C_SR1_ARLO;
        return ERROR_ARBITRATION_LOST; 
    }

    ret = wait_flag(I2C_SR1_ADDR, &I2C1_SR1, ERROR_ADDR); 
    if (ret != 0) {
        return ret; 
    }

    (void)I2C1_SR1; 
    (void)I2C1_SR2; // Clear ADDR flag

    ret = wait_flag(I2C_SR1_TXE, &I2C1_SR1, ERROR_TXE); // Wait for TXE flag
    if (ret != 0) {
        return ret; 
    }

    I2C1_DR=REG_LEDOUT; // send register address

    ret = wait_flag(I2C_SR1_BTF, &I2C1_SR1, ERROR_TRANSFER); // Wait for BTF flag
    if (ret != 0) {
        return ret; 
    }

    I2C1_CR1 |= (1<<8); // start bit

    ret = wait_flag(I2C_SR1_SB, &I2C1_SR1, ERROR_SB); // Wait for start bit
    if (ret != 0) {
        return ret; 
    }

    I2C1_DR=LED_I2C_ADDR<<1|0x01; // read bit

    if(I2C1_SR1 & I2C_SR1_BERR) { // Check for bus error
        I2C1_SR1 &= ~I2C_SR1_BERR; // Clear bus error flag
        return ERROR_BUS_ERROR; 
    }

    if (I2C1_SR1 & I2C_SR1_ARLO) {
        I2C1_SR1 &= ~I2C_SR1_ARLO;
        return ERROR_ARBITRATION_LOST; 
    }
    
    ret = wait_flag(I2C_SR1_ADDR, &I2C1_SR1, ERROR_ADDR); // Wait for ADDR flag
    if (ret != 0) {
        return ret; 
    }

    I2C1_CR1&=~I2C_CR1_ACK ; // disable ACK bit
    (void)I2C1_SR1; // Clear ADDR flag
    (void)I2C1_SR2; // Clear ADDR flag

    I2C1_CR1 |= I2C_CR1_STOP; // stop bit

    ret = wait_flag(I2C_SR1_RXNE, &I2C1_SR1, ERROR_RXNE); // Wait for RXNE flag
    if (ret != 0) {
        return ret; 
    }

    ledout = (uint8_t)I2C1_DR; 

    (void)I2C1_SR1;

    ret = wait_flag_clear(I2C_SR1_RXNE, &I2C1_SR1, ERROR_RXNE); // Wait for RXNE flag
    if (ret != 0) {
        return ret; 
    }

    ret = wait_flag_clear(I2C_SR2_BUSY, &I2C1_SR2, ERROR_BUSY_BUS); // Wait for bus not busy
    if( ret != 0) {
        return ret; 
    }
    
    I2C1_CR1 |= I2C_CR1_ACK; // enable ACK bit
    return ret; // Success
}   

int rgbw_init(uint8_t i2c_addr, uint8_t reg_red_addr, uint8_t reg_green_addr,
                                uint8_t reg_blue_addr,uint8_t reg_white_addr) {

    if (is_initialized) {
        return ERROR_REINITIALIZATION; 
    }
    
    int8_t ret =0;                                
    
    if(i2c_addr!= RGBW_USE_DEFAULT ){
        LED_I2C_ADDR = i2c_addr;
    }
    if(reg_red_addr!= RGBW_USE_DEFAULT ){
        REG_RED = reg_red_addr;
    }
    if(reg_green_addr!= RGBW_USE_DEFAULT){
        REG_GREEN = reg_green_addr;
    }
    if(reg_blue_addr!= RGBW_USE_DEFAULT){
        REG_BLUE = reg_blue_addr;
    }
    if(reg_white_addr!= RGBW_USE_DEFAULT){
        REG_WHITE = reg_white_addr;
    }

    RCC_APB1ENR |=(1<<21); // Enable I2C1 clock
    RCC_AHB1ENR |= (1<<1);// Enable GPIOB clock

    GPIOB_MODER &= ~((3<<12) | (3<<14) ); // Clear mode bits for PB6 and PB7
    GPIOB_MODER |= ((2<<12)|(2<<14)); // Set alternative function
    GPIOB_OTYPER |= (1 << 6) | (1 << 7); //Open-drain
    GPIOB_PUPDR&= ~(3<<12 | 3<<14); 
    GPIOB_PUPDR |= (1 << 12 | 1 << 14); //Pull-up 

    GPIOB_AFRL &= ~((0xF << 24) | (0xF << 28));   
    GPIOB_AFRL |=  ((0x4 << 24) | (0x4 << 28));// AF4 for I2C
    
    I2C1_CR1 &= ~I2C_CR1_PE; // disable CR1 
    I2C1_CR2=I2C_FREQ_MHZ; // set speed
    I2C_CCR = I2C_CCR_VAL; // 
    I2C_TRISE = CALC_I2C_TRISE; // set rise time
    I2C1_CR1 |= I2C_CR1_PE; // enable I2C1

    uint8_t mode_data[] = {0x00, 0x04}; // MODE1=0x00 (SLEEP=0), MODE2=0x04 (OUTDRV=1)
    uint8_t mode_addrs[] = {0x00, 0x01};
    ret =_i2c_write_reg(mode_data, NULL, 2, mode_addrs);    
    if(ret != 0){
        return ret;
    }
    // Enable all channels
    uint8_t send_data[1]={LEDOUT_ALL_OFF};
    uint8_t addrs[1]={REG_LEDOUT};
    ret =_i2c_write_reg(send_data, NULL, 1, addrs);

    if(ret == 0){
        ledout=LEDOUT_ALL_OFF;
        is_initialized = 1;
    }

    return ret;
}

int rgbw_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t *error_mask) {
    if(!is_initialized){
        return ERROR_NOT_INITIALIZED; 
    }
    uint8_t send_data[4]={r,g,b,w};
    uint8_t addrs[4]={REG_RED,REG_GREEN,REG_BLUE,REG_WHITE};
    int ret= _i2c_write_reg(send_data, error_mask,4,addrs);
    return ret;
}

int rgbw_set_red(uint8_t r){
    return i2c_set_brightness_for_channel(REG_RED, r);
}

int rgbw_set_green(uint8_t g){
    return i2c_set_brightness_for_channel(REG_GREEN, g);
}

int rgbw_set_blue(uint8_t b){
    return i2c_set_brightness_for_channel(REG_BLUE, b);
}

int rgbw_set_white(uint8_t w){
    return i2c_set_brightness_for_channel(REG_WHITE, w);
}

int rgbw_disable_red(void) {
    return i2c_disable_channel(REG_RED);
}

int rgbw_disable_green(void) {
    return i2c_disable_channel(REG_GREEN);
}

int rgbw_disable_blue(void) {
    return i2c_disable_channel(REG_BLUE);
}

int rgbw_disable_white(void) {
    return i2c_disable_channel(REG_WHITE);
}

int rgbw_enable_red(void) {
    return i2c_enable_channel(REG_RED);
}


int rgbw_enable_green(void) {
    return i2c_enable_channel(REG_GREEN);
}

int rgbw_enable_blue(void) {
    return i2c_enable_channel(REG_BLUE);
}

int rgbw_enable_white(void) {
    return i2c_enable_channel(REG_WHITE);
}

int rgbw_enable_all(void){
    int ret = 0;
    uint8_t send_data[1]={LEDOUT_ALL_PWM};
    uint8_t addrs[1]={REG_LEDOUT};
    ret = _i2c_write_reg(send_data, NULL,1, addrs);
    if(ret == 0){
        ledout = LEDOUT_ALL_PWM;
    }
    return ret;
}

int rgbw_disable_all(void){
    int ret = 0;
    uint8_t send_data[1]={LEDOUT_ALL_OFF};
    uint8_t addrs[1]={REG_LEDOUT};
    ret = _i2c_write_reg(send_data, NULL,1, addrs);
    if(ret == 0){
        ledout = LEDOUT_ALL_OFF;
    }
    return ret;
}

int8_t rgbw_read_ledout(uint8_t *out) {
    int8_t ret = _i2c_read_reg_ledout();
    if (ret != 0) 
        return ret;
    if (out) 
        *out = ledout;
    return 0;
}


int rgbw_deinit(void){
    if (!is_initialized) {
        return ERROR_REINITIALIZATION; 
    }

    uint8_t ret=0;
    uint8_t old_ledout = ledout;
    ledout = LEDOUT_ALL_OFF;

    uint8_t send_data[1]={LEDOUT_ALL_OFF};
    uint8_t addrs[1]={REG_LEDOUT};
    ret =_i2c_write_reg(send_data, NULL, 1, addrs);

    if(ret !=0){
        return ret;
    }
    I2C1_CR1 &= ~I2C_CR1_PE; // Disable I2C1

    GPIOB_MODER &= ~((3<<12) | (3<<14));
    GPIOB_OTYPER &= ~((1 << 6) | (1 << 7));

    is_initialized = 0;
    return ret;
}

// Check if the driver is initialized
int rgbw_is_initialized(void) {
    return is_initialized;
}