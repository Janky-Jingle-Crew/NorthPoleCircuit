#include "ch32v003fun.h"
#include <stdio.h>
#include "stdbool.h"
#include "stdlib.h"

#include "buttons.h"
#include "PhasePWM.h"
#include "i2c_slave.h"

// Sign of integer, returns 1 or -1
#define sign(i) ((i >= 0)*2 - 1)

#define SYSTICK_INT_HZ (8000)
#define MAX_SPEED 5
volatile int speed_count = 0;
volatile uint8_t i2c_registers[2] = {0x00};


track_state_t track_state;

volatile struct {
	int speed;
	bool direction;
	bool running;
} ui_state;

#define PORTC_NUM 2

void systick_init(void);

void onWrite(uint8_t reg, uint8_t length) {
	printf("%x \n", i2c_registers[0]);
	printf("%x \n", i2c_registers[1]);
	track_enable(&track_state);
}

int main()
{
	SystemInit();

	// Turn on interrupt nesting and hardware stack
	uint32_t sys = __get_INTSYSCR();
	__set_INTSYSCR(sys | 0b11);

	// Init phase PWM timer
	track_init(&track_state);

	// Init button gpios and NVIC
	buttons_init();

	ui_state.running = false;
	ui_state.speed = 1;

	systick_init();

	//I2C
    SetupI2CSlave(0x07, i2c_registers, sizeof(i2c_registers), onWrite, NULL, false);
	//I2C

	//PhasePWM_initTim2_IRQ();


	while(1) {
		buttonPress_t pressed = buttons_read_rising();

		switch (pressed)
		{
		
		case buttonSpeedDec:
			ui_state.speed -= 1;
			break;

		case buttonSpeedInc:
			ui_state.speed += 1;
			break;

		case buttonStartStop:
			if(track_state.enabled){
				track_disable(&track_state);
				//ui_state.speed = 0;
			}else{
				track_enable(&track_state);
			}
			

		default:
			break;
		}

		if(pressed != buttonNone){
			printf("Pressed button: %d\n", pressed);
		}

		Delay_Ms(50);
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
	NVIC_SetPriority(SysTicK_IRQn, 0);
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
	

	// Motor advance PWM phase
	if(track_state.enabled) {
		speed_count++;
		if(speed_count > MAX_SPEED - abs(ui_state.speed)){
			track_step(&track_state, sign(ui_state.speed));
			speed_count = 0;
		}
	}

	// move the compare further ahead in time.
	// as a warning, if more than this length of time
	// passes before triggering, you may miss your
	// interrupt.

	SysTick->CMP += (FUNCONF_SYSTEM_CORE_CLOCK / SYSTICK_INT_HZ);

	// clear IRQ
	SysTick->SR = 0;

}


void TIM2_IRQHandler(void) __attribute__((interrupt));
void TIM2_IRQHandler(void)
{	
	// Clear IRQ
	if (TIM2->INTFR & TIM_UIF)
	{	
		TIM2->INTFR &= ~(TIM_UIF);
		if(ui_state.running) 
		{	
			speed_count++;
			if (speed_count > MAX_SPEED - abs(ui_state.speed)) 
			{
				track_step(&track_state, sign(ui_state.speed));
				speed_count = 0;
			}
		}
	}
}