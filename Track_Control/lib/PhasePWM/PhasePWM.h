#pragma once

#include "funconfig.h"
#include "stdint.h"
#include "ch32v003fun.h"

#define PRESCALE_DIV1 0

volatile uint8_t phase_idx = 0;

const int8_t lut[256] = {
   0,   3,   6,   9,  12,  16,  19,  22,  25,  28,
  31,  34,  37,  40,  43,  46,  49,  51,  54,  57,
  60,  63,  65,  68,  71,  73,  76,  78,  81,  83,
  85,  88,  90,  92,  94,  96,  98, 100, 102, 104,
 106, 107, 109, 111, 112, 113, 115, 116, 117, 118,
 120, 121, 122, 122, 123, 124, 125, 125, 126, 126,
 126, 127, 127, 127, 127, 127, 127, 127, 126, 126,
 126, 125, 125, 124, 123, 122, 122, 121, 120, 118,
 117, 116, 115, 113, 112, 111, 109, 107, 106, 104,
 102, 100,  98,  96,  94,  92,  90,  88,  85,  83,
  81,  78,  76,  73,  71,  68,  65,  63,  60,  57,
  54,  51,  49,  46,  43,  40,  37,  34,  31,  28,
  25,  22,  19,  16,  12,   9,   6,   3,   0,  -3,
  -6,  -9, -12, -16, -19, -22, -25, -28, -31, -34,
 -37, -40, -43, -46, -49, -51, -54, -57, -60, -63,
 -65, -68, -71, -73, -76, -78, -81, -83, -85, -88,
 -90, -92, -94, -96, -98,-100,-102,-104,-106,-107,
-109,-111,-112,-113,-115,-116,-117,-118,-120,-121,
-122,-122,-123,-124,-125,-125,-126,-126,-126,-127,
-127,-127,-127,-127,-127,-127,-126,-126,-126,-125,
-125,-124,-123,-122,-122,-121,-120,-118,-117,-116,
-115,-113,-112,-111,-109,-107,-106,-104,-102,-100,
 -98, -96, -94, -92, -90, -88, -85, -83, -81, -78,
 -76, -73, -71, -68, -65, -63, -60, -57, -54, -51,
 -49, -46, -43, -40, -37, -34, -31, -28, -25, -22,
 -19, -16, -12,  -9,  -6,  -3 };

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
    TIM1->ATRLR = 2400;
    //TIM2->CH1CVR = 1200;
    
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

void PhasePWM_setPhaseDuty(uint8_t phase_ch, int16_t duty) 
{   
    if (duty >= 0)
    {
        switch (phase_ch)
        {
        case 1:
            PhasePWM_setDuty(1, (uint16_t) duty);
            PhasePWM_setDuty(3, 0);
            break;
        case 2:
            PhasePWM_setDuty(4, (uint16_t) duty);
            PhasePWM_setDuty(2, 0);
            break;
        default:
            break;
        }
    }
    else  
    {
        switch (phase_ch)
        {
        case 1:
            PhasePWM_setDuty(3, (uint16_t) (-duty));
            PhasePWM_setDuty(1, 0);
            break;
        case 2:
            PhasePWM_setDuty(2, (uint16_t) (-duty));
            PhasePWM_setDuty(4, 0);
            break;
        default:
            break;
        }        
    }
}


void PhasePWM_step(uint8_t power) 
{   
    // scale duty cycle [0,255] = [0, 2400]
    int32_t max_duty = (uint32_t) (power*2400) >> 8;
    
    uint8_t phase_idx_shift = phase_idx + 64;

    int32_t duty1 = (max_duty * lut[phase_idx]) >> 7;
    int32_t duty2 = (max_duty * lut[phase_idx_shift]) >> 7;
    
    PhasePWM_setPhaseDuty(1, (int16_t) duty1);
    PhasePWM_setPhaseDuty(2, (int16_t) duty2);

    // overflow ok
    phase_idx++;
    
}

void PhasePWM_step2(uint8_t power)
{
        // scale duty cycle [0,255] = [0, 2400]
    int32_t max_duty = (uint32_t) (power*2400) >> 8;
    
    uint8_t phase_idx_shift = (phase_idx + 8)%32;

    int32_t duty1 = (max_duty * lut2[phase_idx]) >> 7;
    int32_t duty2 = (max_duty * lut2[phase_idx_shift]) >> 7;
    
    PhasePWM_setPhaseDuty(1, (int16_t) duty1);
    PhasePWM_setPhaseDuty(2, (int16_t) duty2);

    // wrap at idx 32
    phase_idx++;
    phase_idx = phase_idx % 32;
}

void PhasePWM_step3(uint8_t power)
{
        // scale duty cycle [0,255] = [0, 2400]
    int32_t max_duty = (uint32_t) (power*2400) >> 8;
    
    uint8_t phase_idx_shift = (phase_idx + 4)%16;

    int32_t duty1 = (max_duty * lut3[phase_idx]) >> 7;
    int32_t duty2 = (max_duty * lut3[phase_idx_shift]) >> 7;
    
    PhasePWM_setPhaseDuty(1, (int16_t) duty1);
    PhasePWM_setPhaseDuty(2, (int16_t) duty2);

    // wrap at idx 16
    phase_idx++;
    phase_idx = phase_idx % 16;
}


void PhasePWM_disableTim1(void) 
{
    TIM1->CH1CVR = 0;
    TIM1->CH2CVR = 0;
    TIM1->CH3CVR = 0;
    TIM1->CH4CVR = 0;

    TIM1->CTLR1 &= ~TIM_CEN;   
}

void PhasePWM_enableTim1(void) 
{
    TIM1->CTLR1 |= TIM_CEN;
}

void PhasePWM_initTim2(void) 
{
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

    // Reset TIM2 to init all regs
    RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

    // Prescale Div 0 and 1600 period = 30 kHz PWM
    TIM2->PSC = PRESCALE_DIV1;
    TIM2->ATRLR = 2400;
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

void PhasePWM_setGuardDuty(uint16_t duty)  
{
    TIM2->CH2CVR = duty;
}