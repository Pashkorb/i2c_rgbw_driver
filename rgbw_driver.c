#include "rgbw_driver.h"
#include <stdint.h>

#define RCC_BASE 0x40023800
#define I2C1_BASE 0x40005400
#define GPIOB_BASE 0x40020400
#define REG_LEDOUT 0x08

#define I2C1_CR1 (*(volatile uint32_t*)(I2C1_BASE+0x00))
#define I2C1_CR2 (*(volatile uint32_t*)(I2C1_BASE+0x04))
#define I2C_CCR (*(volatile uint32_t*)(I2C1_BASE+0x1C))
#define I2C_TRISE (*(volatile uint32_t*)(I2C1_BASE+0x20))
#define I2C1_SR1 (*(volatile uint32_t*)(I2C1_BASE+0x14))
#define I2C1_SR2 (*(volatile uint32_t*)(I2C1_BASE+0x18))
#define I2C1_DR (*(volatile uint32_t*)(I2C1_BASE+0x10))

#define I2C_FREQ_MHZ 42
#define I2C_SPEED_HZ 100000
#define I2C_CCR_VAL (I2C_FREQ_MHZ * 1e6 / (2 * I2C_SPEED_HZ))

#define RCC_APB1ENR (*(volatile uint32_t*)(RCC_BASE + 0x40))
#define RCC_AHB1ENR (*(volatile uint32_t*)(RCC_BASE + 0x30))

#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER    (*(volatile uint32_t*)(GPIOB_BASE + 0x04))
#define GPIOB_OSPEEDR   (*(volatile uint32_t*)(GPIOB_BASE + 0x08))
#define GPIOB_PUPDR     (*(volatile uint32_t*)(GPIOB_BASE + 0x0C))
#define GPIOB_AFRL      (*(volatile uint32_t*)(GPIOB_BASE + 0x20))

static uint8_t LED_I2C_ADDR =   0x44;  
static uint8_t REG_RED      =   0x02;  
static uint8_t REG_GREEN    =   0x03;
static uint8_t REG_BLUE     =   0x04;
static uint8_t REG_WHITE    =   0x05;

static uint8_t ledout =0x00;

static int _i2c_write_reg(uint8_t reg, uint8_t value) {

    I2C1_CR1 |=(1<<8);
    int timer=100000;
    
    while (!(I2C1_SR1 & (1 << 0)) && timer > 0) {
        timer--;
        if (timer == 0) {
            return -1; 
        }
    }

    I2C1_DR = LED_I2C_ADDR << 1;
    timer=100000;
    while(!(I2C1_SR1&(1<<1)) && timer > 0){
        timer--;
        if(timer == 0){
            return -2; 
        }
    }

    timer=100000;
    while(!(I2C1_SR1&(1<<7)) && timer > 0){
        timer--;
        if(timer == 0){
            return -3; 
        }
    }
    I2C1_DR=reg;

    timer=100000;
    while (!(I2C1_SR1 & (1 << 7)) && timer > 0) {
        timer--;
        if (timer == 0) {
            return -4; 
        }
    }
    I2C1_DR=value;

    timer=100000;
    while(I2C1_SR1&(1<<10) && timer > 0){
        timer--;
        if(timer == 0){
            return -5; 
        }
    }

    timer=100000;
    while (!(I2C1_SR1 & (1 << 2)) && timer > 0) {
        timer--;
        if (timer == 0) {
            return -6; 
        }
    }
    
    I2C1_CR1 |= (1 << 9);

    return 0;
}



int rgbw_init(uint8_t i2c_addr, uint8_t reg_red_addr, uint8_t reg_green_addr,
                                uint8_t reg_blue_addr,uint8_t reg_white_addr) {

    int8_t ret =0;                                
    if(i2c_addr){
        LED_I2C_ADDR = i2c_addr;
    }
    if(reg_red_addr){
        REG_RED = reg_red_addr;
    }
    if(reg_green_addr){
        REG_GREEN = reg_green_addr;
    }
    if(reg_blue_addr){
        REG_BLUE = reg_blue_addr;
    }
    if(reg_white_addr){
        REG_WHITE = reg_white_addr;
    }

    RCC_APB1ENR |=(1<<21);
    RCC_AHB1ENR |= (1<<1);

    GPIOB_MODER &= ~((3<<12) | (3<<14) );
    GPIOB_MODER |= ((2<<12)|(2<<14));
    GPIOB_OTYPER |= (1 << 6) | (1 << 7);

    GPIOB_PUPDR&= ~(3<<12 | 3<<14);
    GPIOB_PUPDR |= (1 << 12 | 1 << 14);

    GPIOB_AFRL &= ~((0xF << 24) | (0xF << 28));   
    GPIOB_AFRL |=  ((0x4 << 24) | (0x4 << 28));
    
    I2C1_CR1 &= ~(1 << 0); 
    I2C1_CR2=I2C_FREQ_MHZ;
    I2C_CCR = I2C_CCR_VAL;
    I2C_TRISE = 43;
    I2C1_CR1 |= (1<<0);

    ret =_i2c_write_reg(REG_LEDOUT, 0xAA);
    if(ret == 0){
        ledout=0xAA;
    }

    return 0;
}

int rgbw_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t *error_mask) {
    uint8_t mask = 0;
    int first_error = 0; 

    int ret = _i2c_write_reg(REG_RED, r);
    if (ret != 0) {
        mask |= (1 << 0);
        if (first_error == 0) first_error = ret;
    }

    ret = _i2c_write_reg(REG_GREEN, g);
    if (ret != 0) {
        mask |= (1 << 1);
        if (first_error == 0) first_error = ret;
    }

    ret = _i2c_write_reg(REG_BLUE, b);
    if (ret != 0) {
        mask |= (1 << 2);
        if (first_error == 0) first_error = ret;
    }

    ret = _i2c_write_reg(REG_WHITE, w);
    if (ret != 0) {
        mask |= (1 << 3);
        if (first_error == 0) first_error = ret;
    }

    if (error_mask) {
        *error_mask = mask;
    }
    return first_error;
}

int rgbw_set_channel(uint8_t channel, uint8_t brightness){
    uint8_t offset = 0;

    if (channel == REG_RED) {
        offset = 0;
    } else if (channel == REG_GREEN) {
        offset = 2;
    } else if (channel == REG_BLUE) {
        offset = 4;
    } else if (channel == REG_WHITE) {
        offset = 6;
    } else {
        return -9; 
    }

    if((ledout & (0b11 << offset)) != 0b10) {
        return -10; 
    }

    int ret=0;
    ret = _i2c_write_reg(channel,brightness);
    if(ret != 0){
        return ret;
    }
    return 0;
}

int rgbw_set_red(uint8_t r){
    return _i2c_write_reg(REG_RED,r);
}

int rgbw_set_green(uint8_t g){
    return _i2c_write_reg(REG_GREEN,g);
}

int rgbw_set_blue(uint8_t b){
    return _i2c_write_reg(REG_BLUE,b);
}

int rgbw_set_white(uint8_t w){
    return _i2c_write_reg(REG_WHITE,w);
}

int rgbw_enable_channel(uint32_t channel) {
    uint8_t offset = 0;

    if (channel == REG_RED) {
        offset = 0;
    } else if (channel == REG_GREEN) {
        offset = 2;
    } else if (channel == REG_BLUE) {
        offset = 4;
    } else if (channel == REG_WHITE) {
        offset = 6;
    } else {
        return -1; 
    }
    ledout &= ~(0b11 << offset); 
    ledout |=  (0b10 << offset);
    
    return _i2c_write_reg(REG_LEDOUT, ledout);
}

int rgbw_disable_channel(uint32_t channel) {
    uint8_t offset = 0;

    if (channel == REG_RED) {
        offset = 0;
    } else if (channel == REG_GREEN) {
        offset = 2;
    } else if (channel == REG_BLUE) {
        offset = 4;
    } else if (channel == REG_WHITE) {
        offset = 6;
    } else {
        return -1; 
    }

    ledout &= ~(0b00 << offset); 

    return _i2c_write_reg(REG_LEDOUT, ledout);
}

int rgbw_enable_all(void){
    int ret = 0;
    ret = _i2c_write_reg(REG_LEDOUT, 0xAA);
    return ret;
}

int rgbw_disable_all(void){
    int ret = 0;
    ret = _i2c_write_reg(REG_LEDOUT, 0x00);
    return ret;
}

int rgbw_read_ledout(uint8_t *state) {
    return i2c_read_reg(REG_LEDOUT, state);
}