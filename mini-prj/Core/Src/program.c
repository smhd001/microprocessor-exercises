#include "main.h"
#include "uart.h"
#include "packet.h"
#include "program.h"
#include <math.h>

#define AVG_SIZE 10
#define VOLOME 10

int abs(int a);

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


#define TIM_2_PERIOD_MS 1

uint8_t charByte;
Config conf = {cubic,10, 250, 30, 50};
int temperature = 0;
int light = 0;


void program_init() {
	//HAL_UART_Receive_IT(&huart1, &charByte, 1);
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	buzzerInit();
}

////////////////////////////////////////////////////////////////
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		int x = HAL_ADC_GetValue(&hadc1);
		light = average_l((x) * 99 / (3000 - x));
		if (light > 99)
			light = 99;
		if (light < 0)
			light = 0;
		//		 light = ((x) * 99 / (2000 - x));
	}
	if (hadc->Instance == ADC2) {
		int x = HAL_ADC_GetValue(&hadc2);
		temperature = average_t(x / 10) ;
		//		 temperature = (x / 10) ;

	}
}

////////////////////////////////////////////////////////////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		static uint32_t tim2_cnt = 0;
		static uint32_t time;
		static int w;
		static int freq;

		tim2_cnt++;



		if (tim2_cnt % (20/TIM_2_PERIOD_MS) == 0){

			if (light > conf.light_max || temperature  > conf.temperature_max)
			{
				time = tim2_cnt % 1000;
				switch (conf.buzzer_type) {
				case triangle:
					w = (conf.buzzer_max - conf.buzzer_min) * time / 1000;
					freq = max(conf.buzzer_min , (conf.buzzer_min + w) % (conf.buzzer_max + 1));
					buzzerChangeTone(freq *10, VOLOME);
					break;
				case cubic:
					if(time <= 500)
						freq = conf.buzzer_max ;
					else
						freq = conf.buzzer_min ;
					buzzerChangeTone(freq * 10, VOLOME);
					break;
				default:
					freq = (int)((conf.buzzer_max + conf.buzzer_min) / 2 + (conf.buzzer_max - conf.buzzer_min) / 2 * sin(2 * PI * time / 1000));
					buzzerChangeTone(freq * 10, VOLOME);
					break;
				}
			}else {
				buzzerChangeTone(0, 10);
			}

		}

		if (tim2_cnt % (100 /TIM_2_PERIOD_MS) == 0)//each 100ms
		{
			HAL_ADC_Start_IT(&hadc1);
			HAL_ADC_Start_IT(&hadc2);
		}



		int ones = temperature % 10;
		int tens = temperature / 10;
		int houndred = light % 10;
		int thousend = light / 10;
		if (tim2_cnt % 4 == 0) {
			num_to_seven(ones);
			HAL_GPIO_WritePin(GPIOD, D1_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, D2_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, D3_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, D4_Pin, 0);

		} else if (tim2_cnt % 4 == 1) {

			num_to_seven(tens);
			HAL_GPIO_WritePin(GPIOD, D1_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, D2_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, D3_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, D4_Pin, 1);

		} else if (tim2_cnt % 4 == 2) {

			num_to_seven(houndred);
			HAL_GPIO_WritePin(GPIOD, D1_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, D2_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, D3_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, D4_Pin, 1);

		} else if (tim2_cnt % 4 == 3) {

			num_to_seven(thousend);
			HAL_GPIO_WritePin(GPIOD, D1_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, D2_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, D3_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, D4_Pin, 1);

		}
	}
	if (htim->Instance == TIM3) {
		send_data(temperature, light);
	}

}

////////////////////////////////////////////////////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {

	}
}



int average_l(int x)
{
	static int lights[AVG_SIZE];
	static int i = 0;
	static int avg = 50;

//		if (abs(x - avg ) > 20 && i > AVG_SIZE)
//			return avg;

	lights[i % AVG_SIZE] = x;
	i++;
	int sum = 0;
	for(int j = 0; j < AVG_SIZE; j++ )
	{
		sum += lights[j];
	}
	avg = sum / AVG_SIZE;
	return avg;
}

int average_t(int x)
{
	static int temperaturs[AVG_SIZE];
	static int i = 0;
	static int avg = 30;

	//	if (abs(x - avg ) > 20 && i > AVG_SIZE)
	//		return avg;

	temperaturs[i % AVG_SIZE] = x;
	i++;
	int sum = 0;
	for(int j = 0; j < 10; j++ )
	{
		sum += temperaturs[j];
	}
	avg = sum / AVG_SIZE;
	return avg;
}
int abs(int a)
{
	if (a > 0)
		return a;
	else
		return -a;
}



TIM_HandleTypeDef *buzzerPwmTimer;
uint32_t buzzerPwmChannel;

void buzzerInit() {
	buzzerPwmTimer = &htim4;
	buzzerPwmChannel = TIM_CHANNEL_3;
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

void num_to_seven(int i) {
	int x1 = i & 1;
	int x2 = i & 2;
	int x3 = i & 4;
	int x4 = i & 8;
	if (x1 > 0)
		x1 = 1;
	if (x2 > 0)
		x2 = 1;
	if (x3 > 0)
		x3 = 1;
	if (x4 > 0)
		x4 = 1;
	HAL_GPIO_WritePin(GPIOD, A_Pin, x1);
	HAL_GPIO_WritePin(GPIOD, B_Pin, x2);
	HAL_GPIO_WritePin(GPIOD, C_Pin, x3);
	HAL_GPIO_WritePin(GPIOD, D_Pin, x4);
}
