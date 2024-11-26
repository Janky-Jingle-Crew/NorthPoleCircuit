#include "ch32v003fun.h"
#include <stdio.h>
#include "stdbool.h"

//#define WS2812BSIMPLE_IMPLEMENTATION

//#include "ws2812b_simple.h"
//#define WS2812BSIMPLE_NO_IRQ_TWEAKING
//#include "buzzer.h"
//#include "buttons.h"
#include "PhasePWM.h"

#define SYSTICK_INT_HZ (30)

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

#define WALK_BIAS (MAX_DUTY*0.2)
#define SPEED_BIAS 0
#define SPEED_MAX 50
#define SPEED_RAND_RANGE 16


typedef struct walk_state walk_state_t;

struct walk_state {
  int32_t x;
  int32_t bias;
  int32_t min;
  int32_t max;
  walk_state_t * derivative;
};

walk_state_t walk_states[4] = {
  {0, WALK_BIAS, 0, MAX_DUTY, &(walk_state_t) {0, SPEED_BIAS, -SPEED_MAX, SPEED_MAX, NULL}},
  {0, WALK_BIAS, 0, MAX_DUTY, &(walk_state_t) {0, SPEED_BIAS, -SPEED_MAX, SPEED_MAX, NULL}},
  {0, WALK_BIAS, 0, MAX_DUTY, &(walk_state_t) {0, SPEED_BIAS, -SPEED_MAX, SPEED_MAX, NULL}},
  {0, WALK_BIAS, 0, MAX_DUTY, &(walk_state_t) {0, SPEED_BIAS, -SPEED_MAX, SPEED_MAX, NULL}},
};



// xorshift
uint32_t seed = 21;
uint32_t rand(){
  seed ^= seed << 13;
  seed ^= seed >> 17;
  seed ^= seed << 5;

  return seed;
}

int32_t integrate(walk_state_t * state){

  // Calculate delta from derivative or random
  int32_t x_delta;

  if(state->derivative == NULL){
    x_delta = (rand() % (SPEED_RAND_RANGE*2)) - SPEED_RAND_RANGE;
  } else {
    x_delta = integrate(state->derivative);
  }

  // Add delta
  state->x += x_delta;

  // Slowly return to bias
  state->x = (state->x*48 + state->bias*16)/64;

  // Max and Min
  if(state->x < state->min){
    state->x = state->min;
  } 
  if(state->x > state->max){
    state->x = state->max;
  }

  return state->x;

}

int main()
{
	SystemInit();
	
	// Init phase PWM timer
	track_init(&track_state);

	// Init button gpios and NVIC
	//buttons_init();


	ui_state.music = false;
	ui_state.running = false;
	ui_state.speed = 3;

	systick_init();

	while(1) {
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
	PhasePWM_setDuty(1, integrate(&(walk_states[0])) );
	PhasePWM_setDuty(2, integrate(&(walk_states[1])) );
	PhasePWM_setDuty(3, integrate(&(walk_states[2])) );
	PhasePWM_setDuty(4, integrate(&(walk_states[3])) );


}

