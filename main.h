#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_dma.h"
#include "rtk.h"
#include "SuperStarII.h"
#include "Myhal_uart.h"

#define USE_DMA_GPS
#define UART_GPS USART2
#define UART_RF USART6
#define UART_RESULT UART4

#define DMA_GPS_RXStream_IRQHandler DMA1_Stream5_IRQHandler

//#define DMA_RF_RXStream_IRQHandler DMA2_Stream2_IRQHandler//USART1
#define DMA_RF_RXStream_IRQHandler DMA2_Stream1_IRQHandler//USART6

#define DMA_Result_TXStream_IRQHandler DMA1_Stream4_IRQHandler

#define __HAL_UART_FLUSH_SRREGISTER(__HANDLE__) ((__HANDLE__)->Instance->SR)

#define LED3_ON	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET)
#define LED3_OFF	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET)
#define LED3_TOGGLE HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13)
#define LED4_ON	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET)
#define LED4_OFF	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET)
#define LED4_TOGGLE HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12)
#define LED5_ON	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET)
#define LED5_OFF	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET)
#define LED5_TOGGLE HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14)
#define LED6_ON	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET)
#define LED6_OFF	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET)
#define LED6_TOGGLE HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15)

typedef enum{
	
	START_BYTES,
	PAYLOAD
}STAGE;

extern uint32_t t,start;

extern void SendInt(int num);
extern void SendIntStr(int num);
extern void SendStr(const char* str);
#endif /* __MAIN_H */
