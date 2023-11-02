#include "main.h"
#include "packet.h"

static uint8_t myByte;

extern UART_HandleTypeDef huart2;

void uartInit(){
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
    HAL_UART_Receive_IT(&huart2,&myByte,1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
    if (huart->Instance == USART2) {
        stateMachine(myByte);
        HAL_UART_Receive_IT(&huart2, &myByte, 1);
    }
}
