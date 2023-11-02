/*
 * buzzer.c
 *
 *  Created on: Jul 5, 2023
 *      Author: mhmmd
 */
#include "main.h"
extern TIM_HandleTypeDef htim1;
typedef struct
{
    uint16_t frequency;
    uint16_t duration;
} Tone;
const Tone super_mario_bros[] = {
	{2637,306}, // E7 x2
	{   0,153}, // x3 <-- Silence
	{2637,153}, // E7
	{   0,153}, // x3
	{2093,153}, // C7
	{2637,153}, // E7
	{   0,153}, // x3
	{3136,153}, // G7
	{   0,459}, // x3
	{1586,153}, // G6
	{   0,459}, // x3

	{2093,153}, // C7
	{   0,306}, // x2
	{1586,153}, // G6
	{   0,306}, // x2
	{1319,153}, // E6
	{   0,306}, // x2
	{1760,153}, // A6
	{   0,153}, // x1
	{1976,153}, // B6
	{   0,153}, // x1
	{1865,153}, // AS6
	{1760,153}, // A6
	{   0,153}, // x1

	{1586,204}, // G6
	{2637,204}, // E7
	{3136,204}, // G7
	{3520,153}, // A7
	{   0,153}, // x1
	{2794,153}, // F7
	{3136,153}, // G7
	{   0,153}, // x1
	{2637,153}, // E7
	{   0,153}, // x1
	{2093,153}, // C7
	{2349,153}, // D7
	{1976,153}, // B6
	{   0,306}, // x2

	{2093,153}, // C7
	{   0,306}, // x2
	{1586,153}, // G6
	{   0,306}, // x2
	{1319,153}, // E6
	{   0,306}, // x2
	{1760,153}, // A6
	{   0,153}, // x1
	{1976,153}, // B6
	{   0,153}, // x1
	{1865,153}, // AS6
	{1760,153}, // A6
	{   0,153}, // x1

	{1586,204}, // G6
	{2637,204}, // E7
	{3136,204}, // G7
	{3520,153}, // A7
	{   0,153}, // x1
	{2794,153}, // F7
	{3136,153}, // G7
	{   0,153}, // x1
	{2637,153}, // E7
	{   0,153}, // x1
	{2093,153}, // C7
	{2349,153}, // D7
	{1976,153}, // B6

	{   0,  0}	// <-- Disable PWM
};

TIM_HandleTypeDef *buzzerPwmTimer;
uint32_t buzzerPwmChannel;
void buzzerInit() {
	buzzerPwmTimer = &htim1;
	buzzerPwmChannel = TIM_CHANNEL_1;
	HAL_TIM_PWM_Start(buzzerPwmTimer, buzzerPwmChannel);
}

void buzzerChangeTone(uint16_t freq, uint16_t volume) {
	if (freq == 0 || freq > 20000) {
		__HAL_TIM_SET_COMPARE(buzzerPwmTimer, buzzerPwmChannel, 0);
	} else {
		const uint32_t internalClockFreq = HAL_RCC_GetSysClockFreq();
		const uint32_t prescaler = 1 + internalClockFreq / freq / 60000;
		const uint32_t timerClock = internalClockFreq / prescaler;
		const uint32_t periodCycles = timerClock / freq;
		const uint32_t pulseWidth = volume * periodCycles / 1000 / 2;

		buzzerPwmTimer->Instance->PSC = prescaler - 1;
		buzzerPwmTimer->Instance->ARR = periodCycles - 1;
		buzzerPwmTimer->Instance->EGR = TIM_EGR_UG;

		__HAL_TIM_SET_COMPARE(buzzerPwmTimer, buzzerPwmChannel, pulseWidth);
	}
}
#define ARRAY_LENGTH(arr) (sizeof(arr) / sizeof((arr)[0]))
volatile uint16_t melody_tone_count;
volatile uint16_t current_tone_number;
volatile uint32_t current_tone_end;
void Update_Melody()
{
	static const Tone * melody_ptr = super_mario_bros;
    if ((HAL_GetTick() > current_tone_end) && (current_tone_number < melody_tone_count))
    {
        const Tone active_tone = *(melody_ptr + current_tone_number);
        buzzerChangeTone(active_tone.frequency, 300);
        current_tone_end = HAL_GetTick() + active_tone.duration;
        current_tone_number++;
    }
}
void Change_Melody()
{
    melody_tone_count = ARRAY_LENGTH(super_mario_bros);
    current_tone_number = 0;
}
