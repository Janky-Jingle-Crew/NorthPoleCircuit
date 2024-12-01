#include "ch32v003fun.h"
#include "gpio.h"
#include "stdio.h"

gpio_t SDA = {GPIOC, 1};
gpio_t SCL = {GPIOC, 2};

#define I2C_INT_CLK 2000000
#define I2C_BUS_CLK 100000

void i2c_init(void)
{
    // Toggle reset for I2C to clear setting registers
    RCC->APB1PRSTR |= RCC_APB1Periph_I2C1;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1;

    // Enable I2C Clock
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    // Enable I2C default port (C) and Alternate Function
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO;

    // Leave remapping as 0b00 for default
    // AFIO->PCFR1

    // Set pin mode
    gpio_init_custom(&SDA, GPIO_Speed_10MHz, GPIO_CNF_OUT_OD_AF);
    gpio_init_custom(&SCL, GPIO_Speed_10MHz, GPIO_CNF_OUT_OD_AF);

    // Set I2C internal clock
    uint16_t i2c_conf = I2C1->CTLR2 & ~I2C_CTLR2_FREQ;
    i2c_conf |= (FUNCONF_SYSTEM_CORE_CLOCK / I2C_INT_CLK) & I2C_CTLR2_FREQ;
    I2C1->CTLR2 = i2c_conf;

    // Set I2C Bus Clock (CCR is a divider value)
    i2c_conf = (FUNCONF_SYSTEM_CORE_CLOCK / (2 * I2C_BUS_CLK)) & I2C_CKCFGR_CCR;
    I2C1->CKCFGR = i2c_conf;

    // Enable I2C
    I2C1->CTLR1 |= I2C_CTLR1_PE;

    if (I2C1->STAR1 & I2C_STAR1_BERR)
    {
        I2C1->STAR1 &= ~(I2C_STAR1_BERR);
        printf("I2C setup error");
    }
    else
    {
        printf("I2C good to go!");
    }
}

void i2c_write(const uint8_t addr, const uint8_t reg, const uint8_t *buf, const uint8_t len)
{   

    // Todo: Add some error handling with timeouts to prevent getting stuck here
    
    while (I2C1->STAR2 & I2C_STAR2_BUSY);

    // Send Start and wait for assert
    I2C1->CTLR1 |= I2C_CTLR1_START;
    while (!((((uint32_t)I2C1->STAR1 | (uint32_t)(I2C1->STAR2 << 16)) & I2C_EVENT_MASTER_MODE_SELECT) == I2C_EVENT_MASTER_MODE_SELECT));

    // Send addr and wait for transmission
    I2C1->DATAR = (addr << 1) & 0xFE;
    while (!((((uint32_t)I2C1->STAR1 | (uint32_t)(I2C1->STAR2 << 16)) & I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // Write reg and wait for transmission
    I2C1->DATAR = reg;
    while (!(I2C1->STAR1 & I2C_STAR1_TXE));

    // Write data from buffer and wait for transmission
    uint8_t nbyte = 0;
    while (nbyte < len)
    {
        // Write one byte at a time and wait for transmission
        while (!(I2C1->STAR1 & I2C_STAR1_TXE));
        I2C1->DATAR = buf[nbyte];

        ++nbyte;
    }

    // Wait for bus
    while (!((((uint32_t)I2C1->STAR1 | (uint32_t)(I2C1->STAR2 << 16)) & I2C_EVENT_MASTER_BYTE_TRANSMITTED) == I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // Stop transmission
    I2C1->CTLR1 |= I2C_CTLR1_STOP;
}