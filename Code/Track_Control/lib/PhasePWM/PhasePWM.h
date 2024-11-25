#pragma once

#include "funconfig.h"
#include "stdint.h"
#include "ch32v003fun.h"

// PWM Frequency (Hz)
#define TRACK_PWM_FREQ 100000
// Track PWM maximum power [0, 100]
#define TRACK_PWM_POWER_PCT 83
// Guard PWM duty cycle percent [0, 100]
#define GUARD_PWM_POWER_PCT 60

// PWM Prescale
#define PRESCALE_DIV1 0

// PWM period, in system clock ticks. See ATRLR period
#define PWM_PERIOD (FUNCONF_SYSTEM_CORE_CLOCK / (TRACK_PWM_FREQ * (PRESCALE_DIV1+1)) )
// Maximum duty cycle in Sine wave
#define MAX_DUTY (TRACK_PWM_POWER_PCT * PWM_PERIOD / 100)
// Guard rail duty cycle
#define GUARD_DUTY (GUARD_PWM_POWER_PCT * PWM_PERIOD / 100)


// Global state of track 
typedef struct {
    uint8_t phase_idx;
    bool enabled;
} track_state_t;

typedef enum {
    PHASE_A,
    PHASE_B
} phase_num_t;

// Quarter sine wave, scaled to max pwm duty
const int16_t lut_sine[64] = {
    0.0000000000*MAX_DUTY, (0.0245412285*MAX_DUTY), 0.0490676743*MAX_DUTY,
    0.0735645636*MAX_DUTY, 0.0980171403*MAX_DUTY, 0.1224106752*MAX_DUTY,
    0.1467304745*MAX_DUTY, 0.1709618888*MAX_DUTY, 0.1950903220*MAX_DUTY,
    0.2191012402*MAX_DUTY, 0.2429801799*MAX_DUTY, 0.2667127575*MAX_DUTY,
    0.2902846773*MAX_DUTY, 0.3136817404*MAX_DUTY, 0.3368898534*MAX_DUTY,
    0.3598950365*MAX_DUTY, 0.3826834324*MAX_DUTY, 0.4052413140*MAX_DUTY,
    0.4275550934*MAX_DUTY, 0.4496113297*MAX_DUTY, 0.4713967368*MAX_DUTY,
    0.4928981922*MAX_DUTY, 0.5141027442*MAX_DUTY, 0.5349976199*MAX_DUTY,
    0.5555702330*MAX_DUTY, 0.5758081914*MAX_DUTY, 0.5956993045*MAX_DUTY,
    0.6152315906*MAX_DUTY, 0.6343932842*MAX_DUTY, 0.6531728430*MAX_DUTY,
    0.6715589548*MAX_DUTY, 0.6895405447*MAX_DUTY, 0.7071067812*MAX_DUTY,
    0.7242470830*MAX_DUTY, 0.7409511254*MAX_DUTY, 0.7572088465*MAX_DUTY,
    0.7730104534*MAX_DUTY, 0.7883464276*MAX_DUTY, 0.8032075315*MAX_DUTY,
    0.8175848132*MAX_DUTY, 0.8314696123*MAX_DUTY, 0.8448535652*MAX_DUTY,
    0.8577286100*MAX_DUTY, 0.8700869911*MAX_DUTY, 0.8819212643*MAX_DUTY,
    0.8932243012*MAX_DUTY, 0.9039892931*MAX_DUTY, 0.9142097557*MAX_DUTY,
    0.9238795325*MAX_DUTY, 0.9329927988*MAX_DUTY, 0.9415440652*MAX_DUTY,
    0.9495281806*MAX_DUTY, 0.9569403357*MAX_DUTY, 0.9637760658*MAX_DUTY,
    0.9700312532*MAX_DUTY, 0.9757021300*MAX_DUTY, 0.9807852804*MAX_DUTY,
    0.9852776424*MAX_DUTY, 0.9891765100*MAX_DUTY, 0.9924795346*MAX_DUTY,
    0.9951847267*MAX_DUTY, 0.9972904567*MAX_DUTY, 0.9987954562*MAX_DUTY,
    (0.9996988187*MAX_DUTY)
};

const int8_t lut2[32] = {
   0,  25,  49,  71,  90, 106, 117, 125, 127, 125,
 117, 106,  90,  71,  49,  25,   0, -25, -49, -71,
 -90,-106,-117,-125,-127,-125,-117,-106, -90, -71,
 -49, -25 };

 const int8_t lut3[16] = {
   0,  49,  90, 117, 127, 117,  90,  49,   0, -49,
 -90,-117,-127,-117, -90, -49 };



void PhasePWM_initTim1(void) 
{
    // Enable TIM1
    RCC->APB2PCENR |= RCC_APB2Periph_TIM1;

    // reset TIM1 to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    // set clock prescaler divider 
	TIM1->PSC = PRESCALE_DIV1;

    // Set frequency ATRLR = clock/(freq*(PRESCALE_DIV+1))
    // 48 MHz clock, 20 kHz PWM, PRESCALE_DIV = 0;
    TIM1->ATRLR = (PWM_PERIOD);
    
    // Output, positive polarity
    TIM1->CCER |= TIM_CC1E | TIM_CC1P;
    TIM1->CCER |= TIM_CC2E | TIM_CC2P;
    TIM1->CCER |= TIM_CC3E | TIM_CC3P;
    TIM1->CCER |= TIM_CC4E | TIM_CC4P;

    // PWM Mode 2, 0b111, high when counter > compare value
    TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1M_0;
    TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2M_0;
    TIM1->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1 | TIM_OC3M_0;
    TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1 | TIM_OC4M_0;

    // CTLR1: default is up, events generated, edge align
	// enable auto-reload of preload
	TIM1->CTLR1 |= TIM_ARPE;

	// initialize counter
	TIM1->SWEVGR |= TIM_UG;

	// disengage brake
	TIM1->BDTR |= TIM_MOE;

	// Enable TIM1
    TIM1->CTLR1 |= TIM_CEN;
}

void PhasePWM_initTim2(void) 
{
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

    // Reset TIM2 to init all regs
    RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

    TIM2->PSC = PRESCALE_DIV1;
    TIM2->ATRLR = PWM_PERIOD;
    TIM2->CH2CVR = 0;

    // PWM Mode 2
    TIM2->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2M_0;

    // Enable Channel outputs
    TIM2->CCER |= TIM_CC2E | TIM_CC2P;

    // initialize counter
    TIM2->SWEVGR |= TIM_UG;

    // Enable TIM2
    TIM2->CTLR1 |= TIM_CEN;
}




void PhasePWM_setDuty(uint8_t ch, uint16_t duty) 
{
    switch (ch)
    {
    case 1:
        TIM1->CH1CVR = duty;
        break;
    case 2:
        TIM1->CH2CVR = duty;
        break;        
    case 3:
        TIM1->CH3CVR = duty;
        break;
    case 4:
        TIM1->CH4CVR = duty;
        break;            
    default:
        break;
    }

}

void PhasePWM_setPhaseDuty(phase_num_t phase, int16_t duty) 
{   
    if (duty >= 0)
    {
        switch (phase)
        {
        case PHASE_A:
            PhasePWM_setDuty(1, (uint16_t) duty);
            PhasePWM_setDuty(3, 0);
            break;
        case PHASE_B:
            PhasePWM_setDuty(4, (uint16_t) duty);
            PhasePWM_setDuty(2, 0);
            break;
        default:
            break;
        }
    }
    else  
    {
        switch (phase)
        {
        case PHASE_A:
            PhasePWM_setDuty(3, (uint16_t) (-duty));
            PhasePWM_setDuty(1, 0);
            break;
        case PHASE_B:
            PhasePWM_setDuty(2, (uint16_t) (-duty));
            PhasePWM_setDuty(4, 0);
            break;
        default:
            break;
        }        
    }
}

// Returns value from Sine lookup table, i = [0,255] corresponds to one period.
int16_t sine_lut(uint8_t i){
    // Extract value from quarter wave LUT

    if (i < 64) {
        return lut_sine[i];
    } else if (i < 128){
        return lut_sine[127-i];
    } else if (i < 192){
        return -lut_sine[i-128];
    } else {
        return -lut_sine[255-i];
    }

}

// ----------- User facing functions ---------------

// Take steps in wave
void track_step(track_state_t * track_state, uint8_t steps) 
{   
    // overflow ok
    track_state->phase_idx += steps;

    uint8_t phase_idx = track_state->phase_idx;  

    int16_t duty1 = sine_lut(phase_idx);
    int16_t duty2 = sine_lut(phase_idx + 64);
    

    //int16_t duty1 = phase_idx*MAX_DUTY>>8 - MAX_DUTY/2;
    //int16_t duty2 = (phase_idx+64)*MAX_DUTY>>8 - MAX_DUTY/2;
    
    PhasePWM_setPhaseDuty(PHASE_A, duty1);
    PhasePWM_setPhaseDuty(PHASE_B, duty2);
    
}

void track_init(track_state_t * state){
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

	// D3 T2CH2 PWM G1
    GPIOD->CFGLR &= ~(0xf<<(4*3));
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*3);

    // C0 T2CH3 PWM G2
    GPIOC->CFGLR &= ~(0xf<<(4*0));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*0);


	//Activate clock for Alternate Pin function
	RCC->APB2PCENR |= RCC_AFIOEN;

	// // Set pin mapping for tim2
	AFIO->PCFR1 |= ((0b00) << 8); 

	// Set pin mapping for tim1
	AFIO->PCFR1 |= ((0b11) << 6);

    // Init timers
    PhasePWM_initTim1();
    PhasePWM_initTim2();    

    // C3 Guard Enable
    GPIOC->CFGLR &= ~(0xf<<(4*3));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_PP)<<(4*3);
    
    // C6 Phase Enable
    GPIOC->CFGLR &= ~(0xf<<(4*6));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_PP)<<(4*6);
    
    //state inits
    state->phase_idx = 0;
    state->enabled = false;

}


void track_enable(track_state_t * state){
    // Set PWMs
    PhasePWM_setPhaseDuty(PHASE_A, state->phase_idx);
    PhasePWM_setPhaseDuty(PHASE_B, state->phase_idx + 64);

    // Guard duty
    TIM2->CH2CVR = (GUARD_DUTY);

    GPIOC->BSHR |= 1<<3;
    GPIOC->BSHR |= 1<<6;

    // Enable timer counters
    //TIM1->CTLR1 |= TIM_CEN;
    //TIM2->CTLR2 |= TIM_CEN;
}

void track_disable(track_state_t * state){
    // Set duty to zero
    PhasePWM_setPhaseDuty(PHASE_A, 0);
    PhasePWM_setPhaseDuty(PHASE_B, 0);

    // Guard duty
    TIM2->CH2CVR = 0;

    GPIOC->BSHR |= 1<<(3+16);
    GPIOC->BSHR |= 1<<(6+16);

    // Disable timer counters
    //TIM1->CTLR1 &= ~TIM_CEN;
    //TIM2->CTLR1 &= ~TIM_CEN;
}