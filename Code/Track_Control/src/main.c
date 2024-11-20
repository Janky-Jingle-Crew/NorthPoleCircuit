#include "ch32v003fun.h"
#include <stdio.h>
#include "stdbool.h"

#define WS2812BSIMPLE_IMPLEMENTATION

#include "ws2812b_simple.h"
//#define WS2812BSIMPLE_NO_IRQ_TWEAKING
//#include "buzzer.h"
#include "buttons.h"
#include "PhasePWM.h"

#define SYSTICK_INT_HZ (2000)

uint8_t yellow [3] = {0x08, 0x08, 0x00};
uint8_t black [3] = {0x00, 0x00, 0x00};

int touch_vals [3] = {0};
int touch_pins [3] = {0, 1, 2};

track_state_t track_state;

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
	
	// Init phase PWM timer
	track_init(&track_state);

	// Init button gpios and NVIC
	buttons_init();


	ui_state.music = false;
	ui_state.running = false;
	ui_state.speed = 3;

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
			if(ui_state.running){
				track_disable(&track_state);
				ui_state.running = false;
			}else{
				track_enable(&track_state);
				ui_state.running = true;
			}
			

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
	SysTick->CMP += (FUNCONF_SYSTEM_CORE_CLOCK / SYSTICK_INT_HZ);

	// clear IRQ
	SysTick->SR = 0;

	// Motor advance PWM phase
	if(ui_state.running){
		track_step(&track_state, 1);
	}

}