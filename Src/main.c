/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "keyboardBinds.h"
#include "usbd_core.h"
#include "usbd_conf.h"
#include "usbd_hid.h"
#include "usb_device.h"
#include "usbd_def.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "stdint.h"
#include "float.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern USBD_HandleTypeDef hUsbDeviceFS;

struct keyboardHID_t {
       uint8_t id;
       uint8_t modifiers;
       uint8_t key1;
       uint8_t key2;
       uint8_t key3;
   };
 struct mediaHID_t {
       uint8_t id;
       uint8_t keys;
     };

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void debugInit(){
	HAL_UART_Init(&huart1);

}
void debugTx(){

}


void debugSendStr(char * pStr){
	uint32_t i=0;
	while(pStr[i]!=0){
		ITM_SendChar(pStr[i]);
		i++;

	}

}

void debugSendMacro(struct t_macro * mac){
	char buf[64];
	macro2str(mac,buf,64);
	debugSendStr(buf);
}

struct t_controlState{
	uint16_t buttons:keysInPad;
	uint8_t pollState:2;
	uint8_t encoderState:2;
	int8_t encoderVal;
	int16_t stickX;
	int16_t stickY;
	uint8_t keysChangedFlag:4;
	uint32_t lastModified;
};
#define buttonFlag 1
#define modifiedThisCycleFlag 0b10
#define encoderFlag 0b100

void keypadReadStep(struct t_controlState * pCtrls){



	uint8_t columns=(GPIOA->IDR&0b01110000)>>4;
	uint8_t row=pCtrls->pollState * 3;

	uint16_t prevButtons=pCtrls->buttons;
	pCtrls->buttons&= ~(0b111<<row);//mask out those bits that could be set
	pCtrls->buttons|=columns<<row;//or in the new reading

	if((pCtrls->buttons)!=prevButtons){
		pCtrls->keysChangedFlag|=buttonFlag|modifiedThisCycleFlag;
	}

	if(pCtrls->pollState==4){
		pCtrls->pollState=0;
	}
	else{pCtrls->pollState++;}
	GPIOA->ODR=1<<(pCtrls->pollState);


}
#define encA 0b01
#define encB 0b10
volatile uint8_t encoderPeriod=0;//unused but for velocity
//change this as the encoder cannot stay low and the velocity can be determined by how long it is low for

void encoderPoll(struct t_controlState * pCtrls){

	uint8_t newReading=GPIOB->IDR&(encA|encB);


		switch(newReading){
		case encA:{
			if(pCtrls->encoderState){
				pCtrls->keysChangedFlag|=encoderFlag|modifiedThisCycleFlag;
				pCtrls->encoderVal+=1;
				pCtrls->encoderState=0;}
			break;
		}
		case encB:{
			if(pCtrls->encoderState){
				pCtrls->keysChangedFlag|=encoderFlag|modifiedThisCycleFlag;
				pCtrls->encoderVal-=1;
				pCtrls->encoderState=0;}
			break;
		}
		case (encA|encB):{
			pCtrls->encoderState=1;
			break;
		}
		default:{

		}

	}

}



//interval for running below function in 0.1ms increments
volatile uint16_t stateChangeDelay= 20;


void pollAllControlsStep(struct t_controlState * pCtrls){//run every 5ms
	keypadReadStep(pCtrls);
	encoderPoll(pCtrls);

	if(pCtrls->keysChangedFlag&modifiedThisCycleFlag){
		pCtrls->lastModified=TIM2->CNT;
	}
	//full keypad cycle occurs every 20ms

}

struct t_controlState * initPoll(){
	struct t_controlState * pCtrl = malloc(sizeof(struct t_controlState));
	pCtrl->buttons=0;
	pCtrl->encoderState=0;
	pCtrl->encoderVal=0;
	pCtrl->keysChangedFlag=0;
	pCtrl->pollState=0;
	pCtrl->stickX=0;
	pCtrl->stickY=0;
	pCtrl->lastModified=TIM2->CNT;
	return(pCtrl);
}

/*
void logToUsbKeys(uint16_t bitsOfButtons){
	struct keyboardHID_t keyboardHID;
	keyboardHID.id = 1;
	keyboardHID.modifiers = 0;
	keyboardHID.key1 = 0;
	keyboardHID.key2 = 0;
	keyboardHID.key3 = 0;

	if(bitsOfButtons==0){

	}
	else{
		uint8_t curButton=keysInPad;
		uint16_t buttonMask=1<<keysInPad;
		uint8_t i=0;


		while((i<usbKeySize)&&(curButton!=0)&&bitsOfButtons){
			buttonMask=buttonMask>>1;
			if(buttonMask&bitsOfButtons){
				(&keyboardHID.key1)[i]=curButton+4;
				i++;
				bitsOfButtons&=~buttonMask;

			}
			curButton--;

		}

	}
	USBD_HID_SendReport(&hUsbDeviceFS,&keyboardHID,sizeof(struct keyboardHID_t));

}
*/
#define maxOutputIndex 5
void ctrlDesignator(struct t_controlState * pCtrls,struct t_layout * pLayout){
	uint8_t keyboardOutputBytes[5]={1,0,0,0,0};

	uint8_t curOutputIndex=2;


	struct t_macro * mac;
	//check what macros are already running


	uint16_t buttons=pCtrls->buttons;


	if(buttons!=0){

			uint8_t curButton=keysInPad;
			uint16_t buttonMask=1<<keysInPad;

			while((curOutputIndex<maxOutputIndex)&&(curButton!=0)&&buttons){
				buttonMask=buttonMask>>1;
				mac=&(pLayout->keyBinds[curButton]);
				if(buttonMask&buttons){

					if((!mac->currentlyRunning)||mac->simpleKeyFlag){
						mac->currentlyRunning=1;
						mac->index=0;
						keyboardOutputBytes[curOutputIndex]=mac->keyLst[0];
						//debugSendMacro(mac);
						curOutputIndex++;
					}
					buttons&= ~buttonMask;
				}
				else if((mac->simpleKeyFlag)||(mac->index>mac->len)){
					mac->currentlyRunning=0;
				}
				curButton--;
			}

	}

	for(uint8_t i=0;i<keysInPad;i++){
			if(pLayout->keyBinds[i].currentlyRunning){
				mac=&(pLayout->keyBinds[i]);
				mac->ticksRunning++;
				mac= &(pLayout->keyBinds[i]);
				if((!(mac->simpleKeyFlag)&&(mac->len!=1))
						?(((mac->delayLst[mac->index]+1)>(mac->ticksRunning))
						&&(curOutputIndex<maxOutputIndex)):0){
					//if the delay has passed and theres space to send it in this packet
					mac->index++;
					keyboardOutputBytes[curOutputIndex]=mac->keyLst[mac->index];

					curOutputIndex++;

				}
				else{

				}

			}
		}

	USBD_HID_SendReport(&hUsbDeviceFS,keyboardOutputBytes,maxOutputIndex);


}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);

    USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID);

    USBD_Start(&hUsbDeviceFS);


    struct keyboardHID_t keyboardHID;
    keyboardHID.id = 1;
    keyboardHID.modifiers = 0;
    keyboardHID.key1 = 0;
    keyboardHID.key2 = 0;
    keyboardHID.key3 = 0;
    // HID Media

    struct mediaHID_t mediaHID;
    mediaHID.id = 2;
    mediaHID.keys = 0;
    HAL_Delay(100);
    mediaHID.keys = 0xe9;
        USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&mediaHID, sizeof(struct mediaHID_t));
        HAL_Delay(30);
        mediaHID.keys = 0;
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t *) &mediaHID, sizeof(struct mediaHID_t));
        HAL_Delay(30);

        keyboardHID.modifiers = 0;
        keyboardHID.key1 = 0;
        USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&keyboardHID, sizeof(struct keyboardHID_t));
        HAL_Delay(30);
        keyboardHID.modifiers = 0;
        keyboardHID.key1 = 0;
        USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&keyboardHID, sizeof(struct keyboardHID_t));

  HAL_TIM_Base_Start(&htim1);
  HAL_UART_Init(&huart1);
  HAL_TIM_Base_Init(&htim2);
  HAL_TIM_Base_Start(&htim2);


  htim1.Instance->CNT=0;
  	  		HAL_TIM_Base_Start(&htim1);
  	  		struct t_controlState * pControlState=initPoll();
  	  		struct t_layout * testLayout=createLayout(
  	  			"ku128;ku82;ku129;",//vol up, arrow up,vol down
  	  			"ku74;kch.5,kci;ku77;",//home, Press H, wait 5, press i, end
  	  			"ku76;ku68;ku56;",//delete, f11, /
  	  			"ku55;ku40;ku116;",//backslash, enter, execute
  	  			"Test layout",
  	  			"ku79;ku80;ku81;ku82;",
  	  			"ku128;ku129;"
  	  		);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(htim1.Instance->CNT>stateChangeDelay){
	  		htim1.Instance->CNT=0;
	  		HAL_TIM_Base_Start(&htim1);
	  		pollAllControlsStep(pControlState);
	  	}

	  if(pControlState->keysChangedFlag){
		  ctrlDesignator(pControlState,testLayout);
		  pControlState->keysChangedFlag=0;
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
