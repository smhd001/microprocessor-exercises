/*
 * intrupts.c
 *
 *  Created on: Jul 4, 2023
 *      Author: mhmmd
 */

#include "program.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "LiquidCrystal.h"
#include "uart.h"
#define AVG_SIZE 10

extern LCD_state state;
extern int is_on;
int key;
extern SemaphoreHandle_t sec_sem;
extern SemaphoreHandle_t keypad_sem;
extern SemaphoreHandle_t on_sem;
extern SemaphoreHandle_t off_sem;
extern ADC_HandleTypeDef hadc2;
extern int temp;
extern int volume;
int average_t(int x);

// Input pull down rising edge trigger interrupt pins:
// Row1 PD3, Row2 PD5, Row3 PD7, Row4 PB4
GPIO_TypeDef *const Row_ports[] = { GPIOD, GPIOD, GPIOD, GPIOB };
const uint16_t Row_pins[] = { GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_4 };
// Output pins: Column1 PD4, Column2 PD6, Column3 PB3, Column4 PB5
GPIO_TypeDef *const Column_ports[] = { GPIOD, GPIOD, GPIOB, GPIOB };
const uint16_t Column_pins[] =
		{ GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_3, GPIO_PIN_5 };
volatile uint32_t last_gpio_exti;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (last_gpio_exti + 200 > HAL_GetTick()) // Simple button debouncing
			{
		return;
	}
	last_gpio_exti = HAL_GetTick();

	int8_t row_number = -1;
	int8_t column_number = -1;

	if (GPIO_Pin == GPIO_PIN_0) {
		if (is_on) {
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(off_sem, &xHigherPriorityTaskWoken);
		} else {
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(on_sem, &xHigherPriorityTaskWoken);
		}
	}
	if (GPIO_Pin == GPIO_PIN_1) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(sec_sem,&xHigherPriorityTaskWoken);
		return;
	}

	for (uint8_t row = 0; row < 4; row++) // Loop through Rows
			{
		if (GPIO_Pin == Row_pins[row]) {
			row_number = row;
		}
	}

	HAL_GPIO_WritePin(Column_ports[0], Column_pins[0], 0);
	HAL_GPIO_WritePin(Column_ports[1], Column_pins[1], 0);
	HAL_GPIO_WritePin(Column_ports[2], Column_pins[2], 0);
	HAL_GPIO_WritePin(Column_ports[3], Column_pins[3], 0);

	for (uint8_t col = 0; col < 4; col++) // Loop through Columns
			{
		HAL_GPIO_WritePin(Column_ports[col], Column_pins[col], 1);
		if (HAL_GPIO_ReadPin(Row_ports[row_number], Row_pins[row_number])) {
			column_number = col;
		}
		HAL_GPIO_WritePin(Column_ports[col], Column_pins[col], 0);
	}

	HAL_GPIO_WritePin(Column_ports[0], Column_pins[0], 1);
	HAL_GPIO_WritePin(Column_ports[1], Column_pins[1], 1);
	HAL_GPIO_WritePin(Column_ports[2], Column_pins[2], 1);
	HAL_GPIO_WritePin(Column_ports[3], Column_pins[3], 1);

	if (row_number == -1 || column_number == -1) {
		return; // Reject invalid scan
	}
	//   C0   C1   C2   C3
	// +----+----+----+----+
	// | 1  | 2  | 3  | 4  |  R0
	// +----+----+----+----+
	// | 5  | 6  | 7  | 8  |  R1
	// +----+----+----+----+
	// | 9  | 10 | 11 | 12 |  R2
	// +----+----+----+----+
	// | 13 | 14 | 15 | 16 |  R3
	// +----+----+----+----+

	const uint8_t button_number = row_number * 4 + column_number + 1;
	switch (button_number) {
	case 1:
		key = 1;
		break;
	case 2:
		key = 4;
		break;
	case 3:
		key = 7;
		break;
	case 4:
		key = '*';
		/* code */
		break;
	case 5:
		key = 2;
		break;
	case 6:
		key = 5;
		break;
	case 7:
		key = 6;
		break;
	case 8:
		key = 0;
		break;
	case 9:
		key = 3;
		break;
	case 10:
		key = 6;
		break;
	case 11:
		key = 9;
		break;
	case 12:
		key = '#';
		break;
	case 13:
		key = 'A';
		break;
	case 14:
		key = 'B';
		break;
	case 15:
		key = 'C';
		break;
	case 16:
		key = 'D';
		break;
	default:
		break;
	}
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(keypad_sem, &xHigherPriorityTaskWoken);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC2) {

		int x = HAL_ADC_GetValue(&hadc2);
		temp = average_t(x) / 10;
	}
}
int average_t(int x) {
	static int temperaturs[AVG_SIZE];
	static int i = 0;
	static int avg = 30;

	//	if (abs(x - avg ) > 20 && i > AVG_SIZE)
	//		return avg;

	temperaturs[i % AVG_SIZE] = x;
	i++;
	int sum = 0;
	for (int j = 0; j < 10; j++) {
		sum += temperaturs[j];
	}
	avg = sum / AVG_SIZE
	;
	return avg;
}

