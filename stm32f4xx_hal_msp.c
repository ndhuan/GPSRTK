#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)//Result
	{
		static GPIO_InitTypeDef GPIOInitStruct __attribute__((section("IRAM2")));//MUST have "static"
		static DMA_HandleTypeDef hdma_tx __attribute__((section("IRAM2")));
		__DMA1_CLK_ENABLE();		
		__GPIOC_CLK_ENABLE();
		__UART4_CLK_ENABLE();
		
		GPIOInitStruct.Pin = GPIO_PIN_10;
		GPIOInitStruct.Pull = GPIO_NOPULL;
		GPIOInitStruct.Mode = GPIO_MODE_AF_PP;
		GPIOInitStruct.Speed = GPIO_SPEED_FAST;
		GPIOInitStruct.Alternate = GPIO_AF8_UART4;//P60 datasheet
		HAL_GPIO_Init(GPIOC,&GPIOInitStruct);
		
		GPIOInitStruct.Pin = GPIO_PIN_11;
		GPIOInitStruct.Alternate = GPIO_AF8_UART4;
		HAL_GPIO_Init(GPIOC,&GPIOInitStruct);
		
		hdma_tx.Instance = DMA1_Stream4;
		hdma_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;//data width
		hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		//khi o direct mode, data width cua src va dest nhu nhau
		//chi xet data width cua peripheral
		
		hdma_tx.Init.Mode                = DMA_NORMAL;//so luong du lieu do dma controller quyet dinh, khong chay lien tuc
		hdma_tx.Init.Priority            = DMA_PRIORITY_HIGH;	
		hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;	
		hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL ;//co enable fifo dau?????
		hdma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
		hdma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

		HAL_DMA_Init(&hdma_tx);
		__HAL_LINKDMA(huart,hdmatx,hdma_tx);
		
		HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 1);
		HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);		
	}
	else 	if (huart->Instance == USART2)//rover
	{
		static GPIO_InitTypeDef GPIOInitStruct __attribute__((section("IRAM2")));//MUST have "static"
		static DMA_HandleTypeDef hdma_rx __attribute__((section("IRAM2")));
		
		__GPIOA_CLK_ENABLE();
		__USART2_CLK_ENABLE();
		__DMA1_CLK_ENABLE();
		
		GPIOInitStruct.Pin = GPIO_PIN_2;
		GPIOInitStruct.Pull = GPIO_NOPULL;
		GPIOInitStruct.Mode = GPIO_MODE_AF_PP;
		GPIOInitStruct.Speed = GPIO_SPEED_FAST;
		GPIOInitStruct.Alternate = GPIO_AF7_USART2;//P60 datasheet
		HAL_GPIO_Init(GPIOA,&GPIOInitStruct);
		
		GPIOInitStruct.Pin = GPIO_PIN_3;
		GPIOInitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA,&GPIOInitStruct);
		
		hdma_rx.Instance = DMA1_Stream5;
		hdma_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode = DMA_CIRCULAR;
		hdma_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_rx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

		HAL_DMA_Init(&hdma_rx);
		__HAL_LINKDMA(huart,hdmarx,hdma_rx);
		
		HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);		
	}
	else if (huart->Instance == USART1)
	{
		static GPIO_InitTypeDef GPIOInitStruct __attribute__((section("IRAM2")));//MUST have "static"
		static DMA_HandleTypeDef hdma_rx __attribute__((section("IRAM2")));
		__GPIOB_CLK_ENABLE();
		__USART1_CLK_ENABLE();
		__DMA2_CLK_ENABLE();
		
		GPIOInitStruct.Pin = GPIO_PIN_6;
		GPIOInitStruct.Pull = GPIO_NOPULL;
		GPIOInitStruct.Mode = GPIO_MODE_AF_PP;
		GPIOInitStruct.Speed = GPIO_SPEED_FAST;
		GPIOInitStruct.Alternate = GPIO_AF7_USART1;//P60 datasheet
		HAL_GPIO_Init(GPIOB,&GPIOInitStruct);
		
		GPIOInitStruct.Pin = GPIO_PIN_7;
		GPIOInitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOB,&GPIOInitStruct);	
		
		hdma_rx.Instance = DMA2_Stream2;
		hdma_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode = DMA_CIRCULAR;
		hdma_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_rx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

		HAL_DMA_Init(&hdma_rx);
		__HAL_LINKDMA(huart,hdmarx,hdma_rx);
		
		HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);			
	}
	else if (huart->Instance == USART6)//RF
	{
		static GPIO_InitTypeDef GPIOInitStruct __attribute__((section("IRAM2")));//MUST have "static"
		static DMA_HandleTypeDef hdma_rx2 __attribute__((section("IRAM2")));
		__GPIOC_CLK_ENABLE();
		__USART6_CLK_ENABLE();
		__DMA2_CLK_ENABLE();
		
		GPIOInitStruct.Pin = GPIO_PIN_6;
		GPIOInitStruct.Pull = GPIO_NOPULL;
		GPIOInitStruct.Mode = GPIO_MODE_AF_PP;
		GPIOInitStruct.Speed = GPIO_SPEED_FAST;
		GPIOInitStruct.Alternate = GPIO_AF8_USART6;//P60 datasheet
		HAL_GPIO_Init(GPIOC,&GPIOInitStruct);
		
		GPIOInitStruct.Pin = GPIO_PIN_7;
		GPIOInitStruct.Alternate = GPIO_AF8_USART6;
		HAL_GPIO_Init(GPIOC,&GPIOInitStruct);	
		
		hdma_rx2.Instance = DMA2_Stream1;
		hdma_rx2.Init.Channel = DMA_CHANNEL_5;
		hdma_rx2.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx2.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx2.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx2.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx2.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx2.Init.Mode = DMA_CIRCULAR;
		hdma_rx2.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_rx2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_rx2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_rx2.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_rx2.Init.PeriphBurst = DMA_PBURST_SINGLE;

		HAL_DMA_Init(&hdma_rx2);
		__HAL_LINKDMA(huart,hdmarx,hdma_rx2);
		
		HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);			
	}	
}