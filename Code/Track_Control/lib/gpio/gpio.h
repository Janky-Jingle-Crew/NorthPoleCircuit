#ifndef GPIO_GPIO_H_
#define GPIO_GPIO_H_

#include "ch32v003fun.h"

typedef struct {
  GPIO_TypeDef * port;
  uint8_t pin_number;
} gpio_t;

// Init gpio with 10MHz clock, push/pull output
void gpio_init(const gpio_t * gpio){
  // TODO: Turn on port and clock

  uint8_t pinx4 = gpio->pin_number * 4; 

  // Reset config
  gpio->port->CFGLR &= ~(0xf<<(pinx4));

  // Set config
  gpio->port->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(pinx4);

}

// Init gpio with custom speed and mode
void gpio_init_custom(const gpio_t * gpio,
  GPIOSpeed_TypeDef speed,
  uint8_t mode_conf)
{

  uint8_t pinx4 = gpio->pin_number * 4; 

  // Reset config
  gpio->port->CFGLR &= ~(0xf<<(pinx4));

  // Set config
  gpio->port->CFGLR |= (speed | (mode_conf & 0b1100) )<<(pinx4);
  
}

// Set GPIO pin high
void gpio_high(const gpio_t * gpio){
  gpio->port->BSHR = 1 << gpio->pin_number;
}

// Set GPIO pin low
void gpio_low(const gpio_t * gpio){
  gpio->port->BSHR = 1 << (16 + gpio->pin_number);
}

// Toggle GPIO pin
void gpio_toggle(const gpio_t * gpio){
  gpio->port->OUTDR ^= 1 << gpio->pin_number;
}

uint8_t gpio_to_port_num(const gpio_t * gpio){

  if(gpio->port == GPIOA){
    return 0;
  }
  if (gpio->port == GPIOC){
    return 2;
  }
  if (gpio->port == GPIOD){
    return 3;
  }

  return -1;
}

#endif /* GPIO_GPIO_H_ */
