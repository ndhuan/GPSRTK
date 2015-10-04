#include "stm32f4xx_hal.h"
HAL_StatusTypeDef MyHAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void MyHAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
