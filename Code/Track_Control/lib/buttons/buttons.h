#ifndef BUTTONS_BUTTONS_H_
#define BUTTONS_BUTTONS_H_

#include "ch32v003fun.h"
#include "gpio.h"

uint32_t endtime;

const gpio_t spd_dec_button_gpio = {GPIOA, 1};
const gpio_t start_button_gpio = {GPIOA, 2}; 
const gpio_t spd_inc_button_gpio = {GPIOD, 0}; 
//const gpio_t music_button_gpio = {GPIOA, 1}; 
//const gpio_t next_button_gpio = {GPIOC, 6}; // Not enabled on current version

const gpio_t * button_gpios[] = {
    &spd_dec_button_gpio,
    &start_button_gpio,
    &spd_inc_button_gpio,
//    &music_button_gpio,
//    &next_button_gpio,
};

typedef enum {
    buttonSpeedDec,
    buttonStartStop,
    buttonSpeedInc,
//    buttonMusic,
//    buttonNextTrack,
    buttonNone,
} buttonPress_t;

#define num_buttons (sizeof(button_gpios)/sizeof(button_gpios[0]))

buttonPress_t last_button = buttonNone;

void EXTI7_0_IRQHandler(void) __attribute__((interrupt));
void EXTI7_0_IRQHandler(void)
{
    endtime = SysTick->CNT;
    EXTI->INTFR = 0xffffffff;
}

#define GPIOPortByBase(i) ((GPIO_TypeDef *)(GPIOA_BASE + 0x0400 * (i)))

// Low level measurement of capacitance
int MeasureTouch(const gpio_t * gpio, int pu_mode)
{
    uint32_t starttime;
    GPIO_TypeDef *port = gpio->port;
    uint32_t pin = gpio->pin_number;
    uint32_t pinx4 = pin << 2;

    // Mask out just our specific port.  This way we don't interfere with other
    // stuff that may be on this port.
    uint32_t base = port->CFGLR & (~(0xf << pinx4));

    // Mode for CFGLR when asserted.
    uint32_t setmode = base | (GPIO_CFGLR_OUT_2Mhz_PP) << (pinx4);

    // Mode for CFGLR when it drifts.
    uint32_t releasemode = base | (pu_mode) << (pinx4);

    // Assert pin
    port->CFGLR = setmode;
    port->BSHR = 1 << (pin + 16);

    // Setup pin-change-interrupt.  This will trigger when the voltage on the
    // pin rises above the  schmitt trigger threshold.
    AFIO->EXTICR = gpio_to_port_num(gpio) << (pin * 2);
    EXTI->INTENR = 1 << pin; // Enable EXT3
    EXTI->RTENR = 1 << pin;  // Rising edge trigger

    // Tricky, we want the release to happen at an un-aligned address.
    // This actually doubles our touch sensor resolution.
    asm volatile(".balign 4; c.nop");
    port->CFGLR = releasemode;
    starttime = SysTick->CNT;
    endtime = starttime + 384;
    port->BSHR = 1 << (pin);

// Allow up to 384 cycles for the pin to change.
#define DELAY8 \
    asm volatile("c.nop;c.nop;c.nop;c.nop;c.nop;c.nop;c.nop;c.nop;");
    DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8
    DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8
    DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8
    DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8
    DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8
    DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8 DELAY8

    // Optimization: If you did the nop sled in assembly, the interrupt could scoot to the end

    // Disable EXTI
    EXTI->INTENR = 0;
    EXTI->RTENR = 0;

    // Optional assert pin when done to prevent it from noodling around.
    // port->CFGLR = setmode;
    // port->BSHR = 1<<(pin+16);

    return endtime - starttime;
}

void buttons_init(){
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO 
        | RCC_APB2Periph_GPIOA;

    // enable pin-change-interrupt.
    NVIC_EnableIRQ(EXTI7_0_IRQn);
}

int button_read_robust(const gpio_t * gpio, int thr, int iterations){

    for (int i = 0; i < iterations; i++)
    {
        int result = MeasureTouch(gpio, GPIO_CFGLR_IN_FLOAT);
        if (result < thr)
        {
            return 0;
        }
        Delay_Us(100);
    }

    return 1;
}

buttonPress_t buttons_read_rising()
{

    int high_thr = 300;
    int low_thr = 200;

    // Last sample was no press, if pressed this sample => rising edge
    if (last_button == buttonNone){
        for(int i = 0; i < num_buttons; i++){
            if(button_read_robust(button_gpios[i], high_thr, 3)){
                last_button = i;
                return last_button;
            }
        }

        // No new press
        return buttonNone;
    }

    // Some button previously pressed, check if it is still pressed.
    if(!button_read_robust(button_gpios[last_button], low_thr, 3)){
        last_button = buttonNone;
    }

    return buttonNone;
}

#endif /* BUTTONS_BUTTONS_H_ */
