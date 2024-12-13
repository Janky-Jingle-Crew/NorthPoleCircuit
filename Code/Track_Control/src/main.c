#include "ch32v003fun.h"
#include <stdio.h>
#include "stdbool.h"
#include "stdlib.h"

#include "buttons.h"
#include "PhasePWM.h"
#include "i2c_slave.h"

// Sign of integer, returns 1 or -1
#define sign(i) ((i >= 0)*2 - 1)

// Clamping
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define CLAMP(x, lower, upper) (MIN((upper), MAX((x), (lower))))

// Array Length
#define len(a) (sizeof(a) / sizeof(*a))


// Ramp frequency
#define SYSTICK_INT_HZ (50)

// Maximum interval
#define BASE_INTERVAL 1000 

volatile int speed_count = 0;
volatile int8_t speed_idx = 2;
volatile uint8_t i2c_registers[2] = {0x00};
volatile int step_interval = BASE_INTERVAL;
volatile bool music_running;
volatile uint8_t music_note;
volatile uint8_t last_music_note;

// Step interval = BASE_INTERVAL / speed_values 
// Ex. 60 -> 1 step per 16 ticks
// 20 -> 1 step per 50 ticks 
static const int speed_values[] = {-60, -20, 20, 60};

track_state_t track_state;

volatile struct {
	int speed;
	bool direction;
	bool running;
	int target_speed;
	uint32_t step_interval;
} ui_state;

#define PORTC_NUM 2

void systick_init(void);

void onWrite(uint8_t reg, uint8_t length) {

	if(i2c_registers[0] == 0xff){
		music_running = false;
	}else{
		music_running = true;
		
		if(i2c_registers[0] == 0){
			ui_state.speed = 0;
		}else{
			last_music_note = music_note;
			music_note = i2c_registers[0];

			if(music_note > last_music_note){
				ui_state.speed = 20;
			}else{
				ui_state.speed = -20;
			}
		}
	}

	printf("%x \n", i2c_registers[0]);
	//printf("%x \n", i2c_registers[1]);
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
	ui_state.target_speed = speed_values[speed_idx];
	systick_init();

	//I2C
    SetupI2CSlave(0x07, i2c_registers, sizeof(i2c_registers), onWrite, NULL, false);
	//I2C

	PhasePWM_initTim2_IRQ();


	while(1) {
		buttonPress_t pressed = buttons_read_rising();

		switch (pressed)
		{
		
		case buttonSpeedDec:
			speed_idx -= 1;
			speed_idx = CLAMP(speed_idx, 0, len(speed_values)-1);
			ui_state.target_speed = speed_values[speed_idx];
			break;

		case buttonSpeedInc:
			speed_idx += 1;
			speed_idx = CLAMP(speed_idx, 0, len(speed_values)-1);
			ui_state.target_speed = speed_values[speed_idx];
			break;

		case buttonStartStop:
			if(track_state.enabled){
				ui_state.target_speed = 0;
			}else{
				track_enable(&track_state);
				ui_state.target_speed = speed_values[speed_idx];
			}
			

		default:
			break;
		}

		if(pressed != buttonNone){
			//printf("Pressed button: %d\n", pressed);
			printf("speed_idx: %d\n", speed_idx);
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
 	
	// move the compare further ahead in time.
	// as a warning, if more than this length of time
	// passes before triggering, you may miss your
	// interrupt.
	SysTick->CMP += (FUNCONF_SYSTEM_CORE_CLOCK / SYSTICK_INT_HZ);

	// Ramp speed
	if(track_state.enabled && (ui_state.speed != ui_state.target_speed || music_running)) 
	{
		
		if (ui_state.speed < ui_state.target_speed && !music_running) 
		{
			ui_state.speed += 1;
		} 
		else if (ui_state.speed > ui_state.target_speed && !music_running) 
		{
			ui_state.speed -= 1;
		}

		// Convert to step interval for linear speed conversion
		ui_state.step_interval = (abs(ui_state.speed) > 0) ? BASE_INTERVAL / abs(ui_state.speed) : BASE_INTERVAL;

		// Turn off track at stand still
		if(ui_state.target_speed == 0 && ui_state.speed == 0){
			track_disable(&track_state);
		}
	}
	
	
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
	

		// Motor advance PWM phase
		if(track_state.enabled) 
		{	
			speed_count++;
			if(speed_count > ui_state.step_interval){
				track_step(&track_state, sign(ui_state.speed));
				speed_count = 0;
			}
		}
	}
}