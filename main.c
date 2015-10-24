#include "main.h"
#include "rtk.h"

#define TIMER_FREQ_HZ 5

TIM_HandleTypeDef TimerHandle;
UART_HandleTypeDef UartGPSHandle, UartRFHandle, UartResultHandle;

TIM_IC_InitTypeDef sConfig;
TIM_SlaveConfigTypeDef sSlaveConfig;

//static rtksvr_t svr __attribute__((section(".noinit")));
static char result[SOL_MSG_LEN] __attribute__((section("IRAM1")));
static rtksvr_t svr __attribute__((section("IRAM1")));
static obsd_t obsd[2*MAX_SAT] __attribute__((section("IRAM2")));
static volatile bool flagTimeout=0;
static int fobs[2];
static volatile Error RError=INCOMPLETE;//rover data error
static volatile Error BError=INCOMPLETE;//base data error

void SendStr(const char* str);
void SendIntStr(int num);

#ifdef TIME_MEASURE
uint32_t t=0,start=0;
#endif

void SystemClockConfig()
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
	__PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
	
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}
void ConfigLED()
{
	GPIO_InitTypeDef GPIOInitStruct;
	__GPIOD_CLK_ENABLE();
	
	GPIOInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOInitStruct.Pin = GPIO_PIN_12;
	GPIOInitStruct.Speed = GPIO_SPEED_MEDIUM;
	HAL_GPIO_Init(GPIOD, &GPIOInitStruct);
	
	GPIOInitStruct.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOD,&GPIOInitStruct);

	GPIOInitStruct.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOD,&GPIOInitStruct);

	GPIOInitStruct.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOD,&GPIOInitStruct);	
	
	LED3_OFF;
	LED4_OFF;
	LED5_OFF;
	LED6_OFF;
}
void ConfigTimer()
{		
/*
	TimerHandle.Instance = TIM2;
	TimerHandle.Init.Prescaler = (uint32_t)((SystemCoreClock/2)/1000000-1);
	TimerHandle.Init.Period = 0xffffffff;
	TimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	__TIM2_CLK_ENABLE();
	HAL_TIM_Base_Init(&TimerHandle);
	HAL_TIM_Base_Start(&TimerHandle);
*/
	TimerHandle.Instance = TIM3;
	TimerHandle.Init.Prescaler = (uint32_t)((SystemCoreClock/2)/10000-1);
	TimerHandle.Init.Period = 10000/TIMER_FREQ_HZ-1;
	TimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	__TIM3_CLK_ENABLE();
	HAL_TIM_Base_Init(&TimerHandle);

//  sConfig.ICPrescaler = TIM_ICPSC_DIV1;
//  sConfig.ICFilter = 0;  
//  
//  /* Configure the Input Capture of channel 1 */
//  sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
//  sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
//	HAL_TIM_IC_ConfigChannel(&TimerHandle, &sConfig, TIM_CHANNEL_1);

//  sSlaveConfig.SlaveMode     = TIM_SLAVEMODE_RESET;
//  sSlaveConfig.InputTrigger  = TIM_TS_TI1FP1;	
//	HAL_TIM_SlaveConfigSynchronization(&TimerHandle, &sSlaveConfig);
	
	HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	
	HAL_TIM_Base_Start_IT(&TimerHandle);//start timer in interrupt mode
}
void ConfigUART(int baseFormat)
{
	UartGPSHandle.Instance        = UART_GPS;
	switch (baseFormat)
	{
		case (STRFMT_UBX):
		{
			UartGPSHandle.Init.BaudRate   = 230400;	
			break;
		}
		case (STRFMT_SS2):
		{
			UartGPSHandle.Init.BaudRate   = 19200;	
			break;
		}
	}
	
  UartGPSHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartGPSHandle.Init.StopBits   = UART_STOPBITS_1;
  UartGPSHandle.Init.Parity     = UART_PARITY_NONE;
  UartGPSHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartGPSHandle.Init.Mode       = UART_MODE_TX_RX;
	HAL_UART_Init(&UartGPSHandle);	
//	
	UartRFHandle.Instance        = UART_RF;
  UartRFHandle.Init.BaudRate   = 57600;
  UartRFHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartRFHandle.Init.StopBits   = UART_STOPBITS_1;
  UartRFHandle.Init.Parity     = UART_PARITY_NONE;
  UartRFHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartRFHandle.Init.Mode       = UART_MODE_TX_RX;
	HAL_UART_Init(&UartRFHandle);	
	
	UartResultHandle.Instance        = UART_RESULT;
  UartResultHandle.Init.BaudRate   = 230400;
  UartResultHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartResultHandle.Init.StopBits   = UART_STOPBITS_1;
  UartResultHandle.Init.Parity     = UART_PARITY_NONE;
  UartResultHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartResultHandle.Init.Mode       = UART_MODE_TX_RX;
	HAL_UART_Init(&UartResultHandle);		
}
void sendRequest(int baseFormat)
{
	switch (baseFormat)
	{
		case (STRFMT_SS2):
		{
			uint8_t SS2_RQ[19]={ 
								1,0x94,0x6B,0,0,1//msg ID 20 continuous 
//									1,0x14,0xEB,0,0,1//msg ID 20 one shot
								,1,0x96,0x69,0,0,1//msg ID 22 (ephemeris) continuous
								,1,0x97,0x68,1,0,1,1};//msg ID 23 (observation data) continuous
			HAL_UART_Transmit(&UartGPSHandle,SS2_RQ,19,1000);
			break;					
		}
	}
																							
}

int decode_raw(rtksvr_t* svr, int index)
{
	uint8_t* i;
	Error err;
	char str[20];
	int res=0,res2=0,res3=0;
	switch (svr->format[index])
	{
		case STRFMT_UBX:
		{
			if (svr->buffPtr[index] + svr->nb[index] <= MAX_RAW_LEN)
			{
				for (i = svr->buff[index] + svr->buffPtr[index] ; 
							i < svr->buff[index] + svr->buffPtr[index] + svr->nb[index]; i++)
				{
					err = input_ubx(&svr->raw[index],*i);
					if (err>=NO_ERROR1)
					{
						updatesvr(svr,err,index);
						if (err==OBS)
						{
							res+=1;
						}
						else if (err==EPHEMERIS)
						{
							res2+=1;
						}
						else if (err==SOLUTION)
						{
							res3+=1;
						}
					}
				}
			}
			else
			{
				for (i = svr->buff[index] + svr->buffPtr[index] ; 
							i < svr->buff[index] + MAX_RAW_LEN; i++)
				{
					err = input_ubx(&svr->raw[index],*i);
					if (err>=NO_ERROR1)
					{
						updatesvr(svr,err,index);
						if (err==OBS)
						{
							res+=1;
						}
						else if (err==EPHEMERIS)
						{
							res2+=1;
						}
						else if (err==SOLUTION)
						{
							res3+=1;
						}
					}
				}
				for (i = svr->buff[index] ; 
					i < svr->buff[index] + svr->nb[index] + svr->buffPtr[index] - MAX_RAW_LEN ; i++)
				{
					err = input_ubx(&svr->raw[index],*i);
					if (err>=NO_ERROR1)
					{
						updatesvr(svr,err,index);
						if (err==OBS)
						{
							res+=1;
						}
						else if (err==EPHEMERIS)
						{
							res2+=1;
						}
						else if (err==SOLUTION)
						{
							res3+=1;
						}
					}
				}		
			}
			break;
		}
		case STRFMT_SS2:
		{
			if (svr->buffPtr[index] + svr->nb[index] <= MAX_RAW_LEN)
			{
				for (i = svr->buff[index] + svr->buffPtr[index] ; 
							i < svr->buff[index] + svr->buffPtr[index] + svr->nb[index]; i++)
				{
					err = input_ss2(&svr->raw[index],*i);
					if (err>=NO_ERROR1)
					{
						updatesvr(svr,err,index);
						if (err==OBS)
						{
							res+=1;
						}
						else if (err==EPHEMERIS)
						{
							res2+=1;
						}
						else if (err==SOLUTION)
						{
							res3+=1;
						}
					}
				}
			}
			else
			{
				for (i = svr->buff[index] + svr->buffPtr[index] ; 
							i < svr->buff[index] + MAX_RAW_LEN; i++)
				{
					err = input_ss2(&svr->raw[index],*i);
					if (err>=NO_ERROR1)
					{
						updatesvr(svr,err,index);
						if (err==OBS)
						{
							res+=1;
						}
						else if (err==EPHEMERIS)
						{
							res2+=1;
						}
						else if (err==SOLUTION)
						{
							res3+=1;
						}
					}
				}
				for (i = svr->buff[index] ; 
					i < svr->buff[index] + svr->nb[index] + svr->buffPtr[index] - MAX_RAW_LEN ; i++)
				{
					err = input_ss2(&svr->raw[index],*i);
					if (err>=NO_ERROR1)
					{
						updatesvr(svr,err,index);
						if (err==OBS)
						{
							res+=1;
						}
						else if (err==EPHEMERIS)
						{
							res2+=1;
						}
						else if (err==SOLUTION)
						{
							res3+=1;
						}
					}
				}		
			}
			break;
		}
	}

/*
	if (index==0)
	{
		sprintf(str,"%d,%d,%d\r\n",res,res2,res3);
		SendStr(str);
	}
*/	
	return res;
}
void SendInt(int num)
{
	uint8_t array[5];
	array[0]=num>>24;
	array[1]=num>>16;
	array[2]=num>>8;
	array[3]=num;
	array[4]='\n';
	HAL_UART_Transmit(&UartResultHandle,array,5,5);
}
void SendIntStr(int num)
{
	uint8_t str[5]; 
	sprintf((char*)str,"%4d\n",num);
	HAL_UART_Transmit(&UartResultHandle,(unsigned char*)str,5,1);
}
void SendStr(const char* str)
{
	HAL_UART_Transmit(&UartResultHandle,(unsigned char*)str,strlen(str),1);
}
int main()//OPTIMIZATION LEVEL = 0
{
	HAL_Init();
	SystemClockConfig();
	ConfigLED();
	ConfigTimer();

	rtksvrstart(&svr);

	ConfigUART(svr.format[0]);

	fobs[0]=fobs[1]=0;
	//svr.raw[1].time.time = 1429540822;//test SS2 data
	//svr.raw[1].time.time = 1429539852;//test SS2 data
	
	while (HAL_UART_Receive_DMA(&UartGPSHandle,svr.buff[0],MAX_RAW_LEN) != HAL_OK);	
	while (HAL_UART_Receive_DMA(&UartRFHandle,svr.buff[1],MAX_RAW_LEN) != HAL_OK);	

	HAL_Delay(3000);
	sendRequest(svr.format[0]);
	
//	test();

	while(1)
	{
		
		if (flagTimeout)
		{
			int index,temp;
			flagTimeout=0;
			//SendIntStr(UartGPSHandle.Instance->SR);
			//SendIntStr(UartRFHandle.Instance->SR);
			for (index=0;index<2;index++)
			{
				if (index==0)
					temp = UartGPSHandle.hdmarx->Instance->NDTR & 0xffff;
				else
					temp = UartRFHandle.hdmarx->Instance->NDTR & 0xffff;					
				
				if (temp + svr.buffPtr[index] <= MAX_RAW_LEN)
					svr.nb[index] = MAX_RAW_LEN - svr.buffPtr[index] - temp;
				else
					svr.nb[index] = 2*MAX_RAW_LEN - temp - svr.buffPtr[index];
				
				fobs[index] =	decode_raw(&svr,index);

				svr.buffPtr[index] = MAX_RAW_LEN - temp;	

			}
			
//			temp = UartGPSHandle.hdmarx->Instance->NDTR & 0xffff;
//			if (temp + svr.buffPtr[0] <= MAX_RAW_LEN)
//					svr.nb[0] = MAX_RAW_LEN - svr.buffPtr[0] - temp;
//			else
//					svr.nb[0] = 2*MAX_RAW_LEN - temp - svr.buffPtr[0];
//			if (svr.buffPtr[0] + svr.nb[0] <= MAX_RAW_LEN)
//			{
//				for (i = svr.buff[0] + svr.buffPtr[0] ; 
//							i < svr.buff[0] + svr.buffPtr[0] + svr.nb[0]; i++)
//				{
//					HAL_UART_Transmit(&UartResultHandle,i,1,1);

//				}
//			}
//			else
//			{
//				for (i = svr.buff[0] + svr.buffPtr[0] ; 
//							i < svr.buff[0] + MAX_RAW_LEN; i++)
//				{
//					HAL_UART_Transmit(&UartResultHandle,i,1,1);

//				}
//				for (i = svr.buff[0] ; 
//					i < svr.buff[0] + svr.nb[0] + svr.buffPtr[0] - MAX_RAW_LEN ; i++)
//				{
//					HAL_UART_Transmit(&UartResultHandle,i,1,1);
//				}		
//			}
//			svr.buffPtr[0] = MAX_RAW_LEN - temp;


		//rtk positioning**********************************************************************
//		if (0)	
			if (fobs[1])
			{
				fobs[1]=0;
				LED4_TOGGLE;
			}
			if (fobs[0])
			{						
				int i;
				fobs[0]=0;
				LED3_TOGGLE;
#ifdef TIME_MEASURE
				start=HAL_GetTick();
#endif			
				temp=svr.obs[0].n;
				for (i=0;i<temp;i++)
				{
					obsd[i]=svr.obs[0].data[i];				
				}
				for (i=0;(i<svr.obs[1].n)&&(i+temp<MAX_OBS);i++)
				{
					obsd[i+temp]=svr.obs[1].data[i];				
				}			
				if (!rtkpos(&svr.rtk,obsd,i+temp,&svr.nav))
	//			if (1)
				{
					char* res=result;
					LED5_TOGGLE;

#ifdef TIME_MEASURE
					t=HAL_GetTick()-start;
					svr.rtk.sol.processTime = t;	
#endif					
					if (svr.rtk.sol.stat==SOLQ_FIX)
						LED6_TOGGLE;
					
					outsol(res,&svr.rtk.sol,svr.rtk.rb);
					SendStr(result);
				}
				else
				{
					HAL_UART_Transmit_DMA(&UartResultHandle,(unsigned char*)svr.rtk.errbuf,svr.rtk.errLen);
				}
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	flagTimeout=1;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
//	if (UartHandle->Instance==UART_RF)
//		LED6_TOGGLE;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)//PC
{	
		
}
