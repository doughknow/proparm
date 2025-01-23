/*
 * motor.c
 *
 *  Created on: May 29, 2024
 *
 */
#include "motor.h"
#include "main.h"

#define MAX_VALUE 1000

extern TIM_HandleTypeDef htim4;

void l_motor(uint32_t power)
{
   if (power > MAX_VALUE)
      power = MAX_VALUE;
   uint32_t minPulseWidth = 1000; // 1ms pulse width at a 1MHz clock
   uint32_t maxPulseWidth = 2000; // 2ms pulse width
   uint32_t pulse = ((power * (maxPulseWidth - minPulseWidth)) / MAX_VALUE) + minPulseWidth;
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse); // Changed to TIM2, Channel 1
}

void r_motor(uint32_t power)
{
   if (power > MAX_VALUE)
      power = MAX_VALUE;
   uint32_t minPulseWidth = 1000; // 1ms pulse width at a 1MHz clock
   uint32_t maxPulseWidth = 2000; // 2ms pulse width
   uint32_t pulse = ((power * (maxPulseWidth - minPulseWidth)) / MAX_VALUE) + minPulseWidth;
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse); // Changed to TIM2, Channel 1
}
