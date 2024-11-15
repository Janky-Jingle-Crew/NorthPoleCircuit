#include "ch32v003fun.h"
#include <stdio.h>
#include "stdbool.h"

#define WS2812BSIMPLE_IMPLEMENTATION

#include "ws2812b_simple.h"
//#define WS2812BSIMPLE_NO_IRQ_TWEAKING
//#include "buzzer.h"
#include "buttons.h"
#include "PhasePWM.h"

#define tim2_map 0b01
#define tim1_map 0b11

#define SYSTICK_INT_HZ 40000

uint8_t yellow [3] = {0x08, 0x08, 0x00};
uint8_t black [3] = {0x00, 0x00, 0x00};

int touch_vals [3] = {0};
int touch_pins [3] = {0, 1, 2};

volatile struct {
	uint8_t guard_pwm;
	uint32_t speed;
	uint32_t note;
} tick_count;

int systick_count_max;

volatile struct {
	int speed;
	bool running;
	bool music;
} ui_state;

#define PORTC_NUM 2

void systick_init(void);

int main()
{
	SystemInit();
	
	// Enable GPIOs
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC;

	// C4 T1CH1 PWM A1
	GPIOC->CFGLR &= ~(0xf<<(4*4));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*4);

	// C5 T1CH3 PWM A2
	GPIOC->CFGLR &= ~(0xf<<(4*5));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*5);

	// D4 T1CH4 PWM B1
	GPIOD->CFGLR &= ~(0xf<<(4*4));
	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*4);

	// C7 T1CH2 PWM B2
	GPIOC->CFGLR &= ~(0xf<<(4*7));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*7);

	// D3 T2CH2 PWM G2
  GPIOD->CFGLR &= ~(0xf<<(4*3));
  GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*3);

	// // Alternate Function Reset 
	// RCC->APB2PCENR |= RCC_APB2Periph_AFIO;

	//t2pwm_init();
	
	//Activate clock for Alternate Pin function
	RCC->APB2PCENR |= RCC_AFIOEN;

	// // Set pin mapping for tim2
	AFIO->PCFR1 |= ((tim2_map & 0b11) << 8); 

	// Set pin mapping for tim1
	AFIO->PCFR1 |= ((tim1_map & 0b11) << 6);

	// Init phase PWM timer
	PhasePWM_initTim1();

	// Init button gpios and NVIC
	buttons_init();

	// Init buzzer timer
	//t2pwm_init();
	//change_song(0);

	ui_state.music = false;
	ui_state.running = false;
	ui_state.speed = 5;

	systick_init();

	while(1) {
		buttonPress_t pressed = buttons_read_rising();

		switch (pressed)
		{
		
		case buttonSpeedDec:
			ui_state.speed--;
			break;

		case buttonSpeedInc:
			ui_state.speed++;
			break;

		case buttonStartStop:
			ui_state.running = !ui_state.running;

		default:
			break;
		}

		if(pressed != buttonNone){
			printf("Pressed button: %d\n", pressed);
		}

		Delay_Ms(100);
	}
}


/*
 * Start up the SysTick IRQ
 */
void systick_init(void)
{
	/* disable default SysTick behavior */
	SysTick->CTLR = 0;

	/* enable the SysTick IRQ */
	NVIC_EnableIRQ(SysTicK_IRQn);

	/* Set the tick interval to 1ms for normal op */
	SysTick->CMP = (FUNCONF_SYSTEM_CORE_CLOCK / SYSTICK_INT_HZ) - 1;

	/* Start at zero */
	SysTick->CNT = 0;
	//ms_cnt = 0;

	/* Enable SysTick counter, IRQ, HCLK/1 */
	SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE |
									SYSTICK_CTLR_STCLK;
}


/*
 * SysTick ISR just counts ticks
 * note - the __attribute__((interrupt)) syntax is crucial!
 */
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
	// move the compare further ahead in time.
	// as a warning, if more than this length of time
	// passes before triggering, you may miss your
	// interrupt.
	int count = SysTick->CNT;
	SysTick->CMP += (FUNCONF_SYSTEM_CORE_CLOCK / SYSTICK_INT_HZ);

	// clear IRQ
	SysTick->SR = 0;

	// Guard rail bit-bang PWM
	tick_count.guard_pwm ^= 16;
	GPIOD->BSHR = 1 << (3 + tick_count.guard_pwm);
	
	// Motor advance PWM phase
	if(ui_state.running){
		if(tick_count.speed++ > 10-ui_state.speed){
			PhasePWM_step(196);
			tick_count.speed = 0;
		}
	}

	count = SysTick->CNT - count;
	if(systick_count_max<count){
		systick_count_max = count;
	}
}