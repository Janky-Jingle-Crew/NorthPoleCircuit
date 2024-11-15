#ifndef GPIO_GPIO_H_
#define GPIO_GPIO_H_

#include "ch32v003fun.h"

typedef struct {
  GPIO_TypeDef * port;
  uint8_t pin_number;
} gpio_t;

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
