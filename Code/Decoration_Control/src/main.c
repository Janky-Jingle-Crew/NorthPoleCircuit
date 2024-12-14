#include "ch32v003fun.h"
#include <stdio.h>
#include "stdbool.h"

//#define WS2812BSIMPLE_IMPLEMENTATION

//#include "ws2812b_simple.h"
//#define WS2812BSIMPLE_NO_IRQ_TWEAKING
#include "buzzer.h"
#include "buttons.h"
#include "PhasePWM.h"
#include "i2c.h"

#define SYSTICK_INT_HZ (30)

track_state_t led_pwm_state;
music_state_t music_state;

enum comms {
	MUSIC_OFF = 255,
	MUSIC_NOTE_OFF = 0,
};

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


uint8_t buf[1] = {0x77};
#define TRACK_MCU_I2C_ADDR 0x77
#define TRACK_MCU_I2C_REG 0x00

// Write 1 byte to the track mcu via the I2C bus
void send_track(uint8_t val){
	buf[0] = val;
	i2c_write(TRACK_MCU_I2C_ADDR, TRACK_MCU_I2C_REG, buf, 1);
}

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
	track_init(&led_pwm_state);

	// Init button gpios and NVIC
	buttons_init();

	// Init timer2 and state for music
	music_init(&music_state);

	// Init systick timer
	systick_init();
	
	//I2C-------------------------

	i2c_init();

	//I2C------------------------

	
	uint32_t ms_cnt = 0;
	uint32_t next_note = 0;
	uint32_t next_button = 0;


	while(1) {

		if(ms_cnt > next_button){
			next_button = ms_cnt + 50;

			if(buttons_read_rising() == buttonMusic){

				if (music_state.playing)
				{
					music_off(&music_state);
					send_track(0xff);

				}else{
					music_change_song(&music_state, rand()%NUM_SONGS);
					music_on(&music_state);
					next_note = 0;
				}
				
			}
		}

		
		if(music_state.playing && ms_cnt > next_note){
			int8_t note = music_next_note(&music_state);
			send_track(note);
			if(note != 0xff){
				next_note = ms_cnt + music_get_current_note_duration(&music_state);
			}else{
				next_note = 0; 
			}
		}

		Delay_Ms(1);
		ms_cnt++;

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

