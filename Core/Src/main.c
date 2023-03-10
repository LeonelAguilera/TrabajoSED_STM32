/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "st7735.h"
#include "fonts.h"
#include "testimg.h"
#include <malloc.h>
//#include "Menus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
typedef enum {Manual = 0, Automatico_Humedad, Automatico_Tiempo} MODO;
typedef enum {Estado = 0, Modo_Actual, Cambio_Modo, Ajustes_Auto_Tiempo, Ajustes_Auto_Humedad} PANTALLA;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_HUMEDAD_MAX 850
#define ADC_HUMEDAD_MIN 735

//#define __DEBUG__
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
MODO modo;
PANTALLA pantalla;

uint8_t humedad_previa = 0;
uint8_t humedad = 0;

uint8_t humedad_minima = 40;
uint8_t humedad_maxima = 60;
uint8_t humedad_media = 50;

bool button0 = 0;
bool button1 = 0;
bool button2 = 0;
bool update_screen = 1;

bool isTimeToTurnOn = 1;

RTC_TimeTypeDef* alarmasON = 0;
RTC_TimeTypeDef* alarmasOFF = 0;

uint32_t num_alarmas = 0;
uint32_t siguiente_alarma = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Led_Init();
uint8_t sensorHumedad();
void AbrirValvula();
void CerrarValvula();
void ControlAutomatico_Humedad();
void LED_ON_OFF(uint8_t humedad);
void WriteRGB(uint8_t R, uint8_t G, uint8_t B);

void modoHandler();
void menuHandler();

void printMenu_ModoActual();
void printMenu_CambioModo();

void printSeleccion(uint8_t altura);
void printAlarma(RTC_TimeTypeDef horaON, RTC_TimeTypeDef horaOFF, uint8_t altura);
void printCrearAlarma(uint8_t altura);
void printCambiarHora(uint8_t altura);
void printTime();
void printOK(uint8_t altura);
void printMenu_Estado();
void crearAlarma(RTC_TimeTypeDef* returnVal);
void CambiarHora();
void AjustarHumedad();

void nextAlarma();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
  HAL_TIM_Base_Start_IT(&htim3);
  Led_Init();
  ST7735_Init();
  printMenu_Estado();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  humedad = sensorHumedad();

	  modoHandler();
	  menuHandler();

	  LED_ON_OFF(humedad);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

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

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_SECONDS;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1599;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/*****************
 *INICIALIZADORES*
 *****************/

//Inicializa los temporizadores necesarios para controlar
//el led RGB a trav??s de PWM
void Led_Init(){
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

/**********************
 *FUNCIONES AUXILIARES*
 **********************/

//Lee el valor del ADC y lo transforma en un valor de humedad comprendido entre 0 y 100
//a trav??s de un entero sin signo de 8 bits
uint8_t sensorHumedad(){
	uint8_t valor_porc_sens = 0;
	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY)==HAL_OK)
	{
		uint16_t ADC_val=HAL_ADC_GetValue(&hadc1);
		ADC_val = ADC_val > ADC_HUMEDAD_MAX ? ADC_HUMEDAD_MAX : ADC_val < ADC_HUMEDAD_MIN ? ADC_HUMEDAD_MIN : ADC_val;
		valor_porc_sens=100-(((ADC_val-ADC_HUMEDAD_MIN)*100)/(ADC_HUMEDAD_MAX-ADC_HUMEDAD_MIN));
	}
	HAL_ADC_Stop(&hadc1);
	return valor_porc_sens;
}

//Abre la v??lvula
void AbrirValvula(){
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1);
}

//Cierra la v??lvula
void CerrarValvula(){
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0);
}

//Cambia el color del LED RGB seg??n la humedad medida una vez cada 0.1s
void LED_ON_OFF(uint8_t humedad)
{
	static uint32_t last_time = 0;
	if(HAL_GetTick() < last_time + 100){
		return;
	}

	if(humedad < humedad_minima){
		uint8_t R = 255-(humedad*128/humedad_minima);
		uint8_t G = (humedad*127/humedad_minima);
		uint8_t B = 0;
		WriteRGB(R, G, B);
	}
	else if(humedad < humedad_media){
		uint8_t R = 127-((humedad-humedad_minima)*127/(humedad_media-humedad_minima));
		uint8_t G = 127+((humedad-humedad_minima)*128/(humedad_media-humedad_minima));
		uint8_t B = 0;
		WriteRGB(R, G, B);
	}
	else if(humedad < humedad_maxima){
		uint8_t R = 0;
		uint8_t G = 127+((humedad_maxima-humedad)*128/(humedad_maxima-humedad_media));
		uint8_t B = (humedad-humedad_media)*127/(humedad_maxima-humedad_media);
		WriteRGB(R, G, B);
	}
	else{
		uint8_t R = 0;
		uint8_t G = (100-humedad)*127/(100-humedad_maxima);
		uint8_t B = 127+((humedad-humedad_maxima)*127/(100-humedad_maxima));
		WriteRGB(R, G, B);
	}

	last_time = HAL_GetTick();
}

//Recibe un valor RGB y lo representa en el LED
void WriteRGB(uint8_t R, uint8_t G, uint8_t B){
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 255-R);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 255-G);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 255-B);
}


/********************
 *GESTI??N DE ESTADOS*
 ********************/

//Determina qu?? funciones de operaci??n llamar seg??n el modo de trabajo en el que se encuetre el sistema
void modoHandler(){
	switch(modo){
	case Manual:
		//Gestionado por interrupciones externas. Ver: void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
		break;
	case Automatico_Humedad:
		ControlAutomatico_Humedad();
		break;
	case Automatico_Tiempo:
		//Gestionado por interrupciones del RTC. Ver: void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
		break;
	default:
		modo = Manual;
		break;
	}
}

//Gestiona la interfaz que se representa en la pantalla
//
//Si se encuentra en la pantalla de estado comprueba si esta debe ser actualizada, de lo contrario, finaliza su ejecuci??n.
//Si se encuentra en otra pantalla desactiva las interrupciones de los botones, ya que en este estado esta se hace por
//polling y gestiona la interfaz en funci??n de en qu?? pantalla se encuentra.
//Tiene una variable est??tica "selecci??n" que permite recordar cu??l de las opciones de cada men?? est?? eligiendo el usuario
void menuHandler(){
	static uint32_t seleccion = 0;
	if(pantalla == Estado){
		if((humedad < humedad_minima && !(humedad_previa < humedad_minima)) || (humedad > humedad_minima && !(humedad_previa > humedad_minima)) || (humedad < humedad_maxima && !(humedad_previa < humedad_maxima)) || (humedad > humedad_maxima && !(humedad_previa > humedad_maxima))){
			update_screen = 1;
		}
		if(update_screen){
			printMenu_Estado();
			update_screen = 0;
		}
		humedad_previa = humedad;
		return;
	}
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);

	switch(pantalla){
	case Modo_Actual:
		//Muestra el modo en el que el sistema se encuentra operando en el momento
		//Si el usuario presiona el bot??n de la derecha pasa a la pantalla de
		//elecci??n de modo
		if(update_screen){
			printMenu_ModoActual();
			update_screen = 0;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			seleccion = 0;
			pantalla = Cambio_Modo;
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		break;
	case Cambio_Modo:
		//Muestra los 3 modos de operaci??n disponibles en la pantalla
		//y marca con un recuadro verde el seleccionado.
		//Con los dos botones izquierdos el usuario puede modificar su selecci??n
		//Con el bot??n de la derecha el usuario hace efectiva su elecci??n
		//Y a trav??s de un switch case se decide la pr??xima pantalla a la que
		//debe pasar
		if(update_screen){
			printMenu_CambioModo();
			printSeleccion(seleccion);
			update_screen = 0;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1 && button0 == 0){
			seleccion = seleccion==2?2:seleccion+1;
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && button1 == 0){
			seleccion = seleccion==0?0:seleccion-1;
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			switch(seleccion){
			case Automatico_Tiempo:
				pantalla = Ajustes_Auto_Tiempo;
				break;
			case Automatico_Humedad:
				pantalla = Ajustes_Auto_Humedad;
				break;
			default:
				modo = Manual;
				pantalla = Estado;
				printMenu_Estado();
				HAL_NVIC_EnableIRQ(EXTI0_IRQn);
				HAL_NVIC_EnableIRQ(EXTI2_IRQn);
				HAL_NVIC_EnableIRQ(EXTI3_IRQn);
				HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
				break;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
			seleccion = 0;
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		break;
	case Ajustes_Auto_Tiempo:
		//Muestra las alarmas creadas y permite al usuario desplazarse por ellas usando
		//los dos botones de la izquierda, al final de las alarmas se muestran los botones
		//para crear una nueva alarma, para aceptar y hacer efectiva la elecci??n de modo
		//y para cambiar la hora del dispositivo.
		if(update_screen){
			ST7735_FillScreenFast(ST7735_CYAN);
			if(alarmasON != 0)
			{
				//Si hay alarmas creadas, imprime en pantalla las alarmas desde [seleccion] hasta
				//[seleccion+2], en caso de que no existan suficientes alarmas, imprime los botones
				//extra que sean necesarios ("crear alarma", "OK" y "cambiar hora").
				if(seleccion <= num_alarmas){
					for(uint8_t i = 0; i < MIN(num_alarmas-seleccion, 3); i++){
						printAlarma(alarmasON[i], alarmasOFF[i], i);
					}
					if(num_alarmas-seleccion < 3){
						printCrearAlarma(num_alarmas-seleccion);
					}
					if(num_alarmas-seleccion < 2){
						printOK(num_alarmas-seleccion+1);
					}
					if(num_alarmas-seleccion < 1){
						printCambiarHora(num_alarmas-seleccion+2);
					}
				}
				else if(seleccion == num_alarmas+1){
					printOK(0);
					printCambiarHora(1);
				}
				else{
					printCambiarHora(0);
				}
			}
			else
			{
				//Si no hay alarmas creadas, solamente muestra los botones de crear alarma y
				//cambio de hora.
				if(seleccion == 0){
					printCrearAlarma(0);
					printCambiarHora(1);
				}
				else{
					printCambiarHora(0);
				}
			}
			printSeleccion(0);
			update_screen = 0;
		}
		//Navegaci??n por el men?? de alarmas
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1 && button0 == 0){
			if(alarmasON == 0){
				seleccion = num_alarmas+2;
			}
			else
			{
				seleccion = seleccion>=num_alarmas+2?num_alarmas+2:seleccion+1;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && button1 == 0){
			if(alarmasON == 0){
				seleccion = 0;
			}
			else
			{
				seleccion = seleccion==0?0:seleccion-1;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		//Selecci??n
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			if(seleccion >= num_alarmas + 2){
				//Si el usuario est?? seleccionando el bot??n de cambio de hora
				CambiarHora();
			}
			else if(seleccion == num_alarmas + 1)
			{
				//Si el usuario est?? seleccionando el bot??n de "OK" activa las interrupciones y genera la pr??xima
				modo = Automatico_Tiempo;
				pantalla = Estado;
				printMenu_Estado();
				__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF); //Se borra el flag de interrupciones del RTC para evitar comportamientos inesperados
				HAL_NVIC_EnableIRQ(EXTI3_IRQn);
				HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
				nextAlarma();
			}
			else if(seleccion == num_alarmas)
			{
				//Si el usuario est?? seleccionando el bot??n de crear una nueva alarma lo env??a a la funci??n
				//"crearAlarma" que proporciona la interfaz necesaria para hacerlo, reasigna la memoria din??micamente
				//y a??ade la nueva alarma al vector de alarmas de encendido y apagado
				RTC_TimeTypeDef nueva_alarma[2];
				crearAlarma(nueva_alarma);
				RTC_TimeTypeDef ON  = nueva_alarma[0];
				RTC_TimeTypeDef OFF = nueva_alarma[1];

				if(num_alarmas == 0){
					alarmasON  = (RTC_TimeTypeDef*)malloc(sizeof(RTC_TimeTypeDef));
					alarmasOFF = (RTC_TimeTypeDef*)malloc(sizeof(RTC_TimeTypeDef));

					if(alarmasON == NULL || alarmasOFF == NULL){
						Error_Handler();
					}

					alarmasON[0] = ON;
					alarmasOFF[0] = OFF;

					num_alarmas++;
				}
				else{
					alarmasON = realloc(alarmasON, (num_alarmas+1)*sizeof(RTC_TimeTypeDef));
					alarmasOFF = realloc(alarmasOFF, (num_alarmas+1)*sizeof(RTC_TimeTypeDef));

					if(alarmasON == NULL || alarmasOFF == NULL){
						Error_Handler();
					}
					alarmasON[num_alarmas] = ON;
					alarmasOFF[num_alarmas] = OFF;

					num_alarmas++;
				}
			}
			else{
				//Si el usuario est?? seleccionando una alarma cualquiera, entonces esta se elimina
				//Y se reasigna la memoria de forma adecuada
				if(num_alarmas == 1){
					free(alarmasON);
					free(alarmasOFF);
					alarmasON = 0;
					alarmasOFF = 0;
				}
				else{
					RTC_TimeTypeDef* alarmasTemp = malloc((num_alarmas-1)*sizeof(RTC_TimeTypeDef));
					if(alarmasTemp == NULL){
						Error_Handler();
					}
					for(uint32_t i = 0; i < num_alarmas-1; i++){
						alarmasTemp[i] = alarmasON[i+((uint32_t)i>=seleccion)];
					}
					free(alarmasON);
					alarmasON = alarmasTemp;

					alarmasTemp = malloc((num_alarmas-1)*sizeof(RTC_TimeTypeDef));
					if(alarmasTemp == NULL){
						Error_Handler();
					}
					for(uint32_t i = 0; i < num_alarmas-1; i++){
						alarmasTemp[i] = alarmasOFF[i+((uint32_t)i>=seleccion)];
					}
					free(alarmasOFF);
					alarmasOFF = alarmasTemp;

					num_alarmas--;
				}
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		break;
	case Ajustes_Auto_Humedad:
		//Muestra al usuario los ajustes de humedad almacenados y le permite modificarlos antes de activar
		//el modo autom??tico
		AjustarHumedad();
		modo = Automatico_Humedad;
		pantalla = Estado;
		printMenu_Estado();
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
		HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
		break;
	default:
		//Por defecto se asigna al modo manual
		modo = Manual;
		pantalla = Estado;
		printMenu_Estado();
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
		HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
	}
}


/********************
 *AUTOM??TICO HUMEDAD*
 ********************/
//	Control autom??tico de riego basado en la humedad de la tierra
//
//Si la humedad medida cae por debajo de la m??nima, se abre la v??lvula, permitiendo al agua pasar
//Si la humedad medida supera la m??xima entonces cierra la v??lvula para evitar liberar m??s agua
void ControlAutomatico_Humedad()
{
	if (humedad >= humedad_maxima){
		CerrarValvula();
	}
	else if(humedad <= humedad_minima){
		AbrirValvula();
	}
}

//Esta funci??n permite al usuario ajustar los valores de humedad m??nima y m??xima entre
//los que quiere se se mantenga la planta en el modo "Autom??tico Humedad".
//
//Primero se ajusta el valor de humedad m??nimo incrementando y decrementando su valor a
//trav??s de los dos botones de la izquierda. Si este disminuye de 0, regresa a 100; y si
//supera 100, regresa a 0.
//El mismo proceso se repite para el ajuste del valor m??ximo, con la diferencia de que este
//no puede ser menor al valor m??nimo previamnte elegido.
//La lectura de los botones se hace por polling dentro de un bucle infinito el cual se rompe
//con el bot??n de la derecha permitiendo continuar con el resto del c??digo
void AjustarHumedad(){

	update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
	ST7735_FillScreenFast(ST7735_CYAN);
	ST7735_WriteString(16, 10, "MINIMO", Font_16x26, ST7735_BLACK, ST7735_WHITE);
	char valor[4];
	while(true){ //Selecci??n de humedad minima
		if(update_screen){
			sprintf(valor,"%02d%c",humedad_minima,37);
			ST7735_WriteString(47, 40, valor, Font_11x18, ST7735_BLACK, ST7735_WHITE);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && button0 == 0){
			if(humedad_minima == 100){
				humedad_minima = 0;
				ST7735_FillScreenFast(ST7735_CYAN);
				ST7735_WriteString(16, 10, "MINIMO", Font_16x26, ST7735_BLACK, ST7735_WHITE);
			}
			else{
				humedad_minima++;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1 && button1 == 0){
			if(humedad_minima == 100){
				ST7735_FillScreenFast(ST7735_CYAN);
				ST7735_WriteString(16, 10, "MINIMO", Font_16x26, ST7735_BLACK, ST7735_WHITE);
			}
			if(humedad_minima == 0){
				humedad_minima = 100;
			}
			else{
				humedad_minima--;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			button2 = 1;
			break;
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	}
	ST7735_FillScreenFast(ST7735_CYAN);
	ST7735_WriteString(16, 10, "MAXIMO", Font_16x26, ST7735_BLACK, ST7735_WHITE);
	while(true){ //Selecci??n de humedad maxima
		if(update_screen){
			sprintf(valor,"%02d%c",humedad_maxima,37);
			ST7735_WriteString(47, 40, valor, Font_11x18, ST7735_BLACK, ST7735_WHITE);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && button0 == 0){
			if(humedad_maxima == 100){
				humedad_maxima = humedad_minima;
				ST7735_FillScreenFast(ST7735_CYAN);
				ST7735_WriteString(16, 10, "MAXIMO", Font_16x26, ST7735_BLACK, ST7735_WHITE);
			}
			else{
				humedad_maxima++;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1 && button1 == 0){
			if(humedad_maxima == 100){
				ST7735_FillScreenFast(ST7735_CYAN);
				ST7735_WriteString(16, 10, "MAXIMO", Font_16x26, ST7735_BLACK, ST7735_WHITE);
			}
			if(humedad_maxima <= humedad_minima){
				humedad_maxima = 100;
			}
			else{
				humedad_maxima--;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			button2 = 1;
			break;
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	}
	humedad_media = (humedad_minima+humedad_maxima)/2;
}

/*******************
 *AUTOM??TICO TIEMPO*
 *******************/

//Esta funci??n permite al usuario crear una nueva alarma de encendido y apagado
//
//Para esto crea una variable "ON" y otra "OFF" en las que se almacenar??n la hora de encendido
//y apagado de la alarma
//Primero permite al usuario incrementar y decrementar el valor de la hora de encendido usando
//los dos botones de la izquierda. Si supera 23h, regresa a 0; si va por debajo de 0, sube a 23.
//Una vez que presiona el bot??n de la derecha pasa al ajuste de minutos que funciona de la
//misma forma.
//Una vez ajustado correctamente el tiempo de inicio, se repite el proceso pero con la hora de
//apagado.
//La lectura de los botones se hace por polling, cad secci??n se encuentra dentro de un bucle
//infinito el cual el bot??n de la derecha rompe para pasar a la siguiente secci??n de c??digo.
//Una vez que se han elegido valores para la hora y los minutos de encendido y apagado, se
//almacenan ambos datos en el puntero que se recibi?? como par??metro.
void crearAlarma(RTC_TimeTypeDef* returnVal){
	update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
	RTC_TimeTypeDef ON = {0};
	RTC_TimeTypeDef OFF = {0};
	ST7735_FillScreenFast(ST7735_CYAN);
	ST7735_WriteString(48, 10, "ON", Font_16x26, ST7735_BLACK, ST7735_WHITE);
	char alarma[6];
	while(true){ //Selecci??n de hora de encendido
		if(update_screen){
			sprintf(alarma,"%02d:%02d",ON.Hours,ON.Minutes);
			ST7735_WriteString(36, 30, alarma, Font_11x18, ST7735_BLACK, ST7735_WHITE);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && button0 == 0){
			if(ON.Hours == 23){
				ON.Hours = 0;
			}
			else{
				ON.Hours++;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1 && button1 == 0){
			if(ON.Hours == 0){
				ON.Hours = 23;
			}
			else{
				ON.Hours--;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			button2 = 1;
			break;
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	}
	while(true){ //Selecci??n de minuto de encendido
		if(update_screen){
			sprintf(alarma,"%02d:%02d",ON.Hours,ON.Minutes);
			ST7735_WriteString(36, 30, alarma, Font_11x18, ST7735_BLACK, ST7735_WHITE);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && button0 == 0){
			if(ON.Minutes == 59){
				ON.Minutes = 0;
			}
			else{
				ON.Minutes++;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1 && button1 == 0){
			if(ON.Minutes == 0){
				ON.Minutes = 59;
			}
			else{
				ON.Minutes--;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			button2 = 1;
			break;
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	}
	ST7735_FillScreenFast(ST7735_CYAN);
	ST7735_WriteString(40, 10, "OFF", Font_16x26, ST7735_BLACK, ST7735_WHITE);
	while(true){ //Selecci??n de hora de apagado
		if(update_screen){
			sprintf(alarma,"%02d.%02d",OFF.Hours,OFF.Minutes);
			ST7735_WriteString(36, 30, alarma, Font_11x18, ST7735_BLACK, ST7735_WHITE);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && button0 == 0){
			if(OFF.Hours == 23){
				OFF.Hours = 0;
			}
			else{
				OFF.Hours++;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1 && button1 == 0){
			if(OFF.Hours == 0){
				OFF.Hours = 23;
			}
			else{
				OFF.Hours--;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			button2 = 1;
			break;
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	}
	while(true){ //Selecci??n de minuto de apagado
		if(update_screen){
			sprintf(alarma,"%02d.%02d",OFF.Hours,OFF.Minutes);
			ST7735_WriteString(36, 30, alarma, Font_11x18, ST7735_BLACK, ST7735_WHITE);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && button0 == 0){
			if(OFF.Minutes == 59){
				OFF.Minutes = 0;
			}
			else{
				OFF.Minutes++;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1 && button1 == 0){
			if(OFF.Minutes == 0){
				OFF.Minutes = 59;
			}
			else{
				OFF.Minutes--;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			button2 = 1;
			break;
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	}
	returnVal[0] = ON;
	returnVal[1] = OFF;
}


//Esta funci??n permite al usuario cambiar la hora del sistema
//
//Para esto crea una variable "Time" con los datos de la hora actual registrada en el sistema
//y la muestra en pantalla.
//Primero permite al usuario incrementar y decrementar su valor usando los dos botones de la
//izquierda. Si supera 23h, regresa a 0; si va por debajo de 0, sube a 23.
//Una vez que presiona el bot??n de la izquierda pasa al cambio de minutos que funciona de la
//misma forma.
//La lectura de los botones se hace por polling, cad secci??n se encuentra dentro de un bucle
//infinito el cual el bot??n de la derecha rompe para pasar a la siguiente secci??n de c??digo.
//Una vez que se han elegido valores para la hora y los minutos se guarda el valor de la
//variable Time en el RTC
void CambiarHora(){
	update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
	RTC_TimeTypeDef Time = sTime;
	ST7735_FillScreenFast(ST7735_CYAN);
	ST7735_WriteString(42, 10, "Hora", Font_16x26, ST7735_BLACK, ST7735_WHITE);
	char hora[6];
	while(true){ //Selecci??n de hora de encendido
		if(update_screen){
			sprintf(hora,"%02d:%02d",Time.Hours,Time.Minutes);
			ST7735_WriteString(36, 40, hora, Font_11x18, ST7735_BLACK, ST7735_WHITE);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && button0 == 0){
			if(Time.Hours == 23){
				Time.Hours = 0;
			}
			else{
				Time.Hours++;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1 && button1 == 0){
			if(Time.Hours == 0){
				Time.Hours = 23;
			}
			else{
				Time.Hours--;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			button2 = 1;
			break;
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	}
	while(true){ //Selecci??n de minuto de encendido
		if(update_screen){
			sprintf(hora,"%02d:%02d",Time.Hours,Time.Minutes);
			ST7735_WriteString(36, 40, hora, Font_11x18, ST7735_BLACK, ST7735_WHITE);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && button0 == 0){
			if(Time.Minutes == 59){
				Time.Minutes = 0;
			}
			else{
				Time.Minutes++;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1 && button1 == 0){
			if(Time.Minutes == 0){
				Time.Minutes = 59;
			}
			else{
				Time.Minutes--;
			}
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1 && button2 == 0){
			button2 = 1;
			break;
		}
		button0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		button1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		button2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	}
	HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN);
}


/************************
 *CONTROL DE LA PANTALLA*
 ************************/

/*Men??s*/

//Pantalla de inicio que muestra informaci??n sobre el nivel de humedad y la hora del dispositivo
//
//Para esto, seg??n el nivel de humedad dibuja una de 3 im??genes cuya informaci??n se encuentra
//almacenada dentro de testimg.h
//Una vez dibujada la imagen correcta se superpone la hora
void printMenu_Estado(){
	if(humedad > humedad_maxima){
		ST7735_DrawImage(0, 0, ST7735_WIDTH, ST7735_HEIGHT, (uint16_t*)DEMASIADA_AGUA);
	}
	else if(humedad > humedad_minima){
		ST7735_DrawImage(0, 0, ST7735_WIDTH, ST7735_HEIGHT, (uint16_t*)BIEN_AGUA);
	}
	else{
		ST7735_DrawImage(0, 0, ST7735_WIDTH, ST7735_HEIGHT, (uint16_t*)POCA_AGUA);
	}
	printTime();
}

//Pantalla que muestra el modo de funcionamiento actual del sistema
//
//Para esto llena la imagen del color base (ci??n) y dibuja un rect??ngulo blanco sobre
//el que escribe el nombre del modo en el que se encuentra actualmente
//
//TODO: Incluir debajo un aviso de que para cambiar el modo debe pulsar el bot??n de la derecha
void printMenu_ModoActual(){
	ST7735_FillScreenFast(ST7735_CYAN);
	ST7735_FillRectangleFast(5, 5, ST7735_WIDTH-10, 40, ST7735_WHITE);
	switch(modo){
	case Manual:
		ST7735_WriteString(31, 16, "Manual", Font_11x18, ST7735_BLACK, ST7735_WHITE);
		break;
	case Automatico_Humedad:
		ST7735_WriteString(9, 6, "Automatico", Font_11x18, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteString(25, 26, "Humedad", Font_11x18, ST7735_BLACK, ST7735_WHITE);
		break;
	case Automatico_Tiempo:
		ST7735_WriteString(9, 6, "Automatico", Font_11x18, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteString(31, 26, "Tiempo", Font_11x18, ST7735_BLACK, ST7735_WHITE);
		break;
	default:
		ST7735_WriteString(36, 16, "Error", Font_11x18, ST7735_BLACK, ST7735_WHITE);
		break;
	}
}

//Pantalla que muestra los distintos modos de funcionamiento del sistema
//
//Para esto llena la imagen del color base (ci??n) y dibuja tres rect??ngulos blanco sobre
//los cuales escribe el nombre de los distintos modos en los que opera el sistema
void printMenu_CambioModo(){
	ST7735_FillScreenFast(ST7735_CYAN);
	ST7735_FillRectangleFast(5, 5, ST7735_WIDTH-10, 48, ST7735_WHITE);
	ST7735_FillRectangleFast(5, 56, ST7735_WIDTH-10, 48, ST7735_WHITE);
	ST7735_FillRectangleFast(5, 107, ST7735_WIDTH-10, 48, ST7735_WHITE);

	ST7735_WriteString(31, 20, "Manual", Font_11x18, ST7735_BLACK, ST7735_WHITE);

	ST7735_WriteString(9, 61, "Automatico", Font_11x18, ST7735_BLACK, ST7735_WHITE);
	ST7735_WriteString(25, 81, "Humedad", Font_11x18, ST7735_BLACK, ST7735_WHITE);

	ST7735_WriteString(9, 112, "Automatico", Font_11x18, ST7735_BLACK, ST7735_WHITE);
	ST7735_WriteString(31, 132, "Tiempo", Font_11x18, ST7735_BLACK, ST7735_WHITE);
}

/*Funciones auxiliares*/

//Dibuja los lados de un rect??ngulo verde con la intenci??n de que marque al usuario la opci??n
//que est?? eligiendo
void printSeleccion(uint8_t altura){
	for(int x = 5; x < ST7735_WIDTH-5; x++) {
		ST7735_DrawPixel(x, 5+(altura*51), ST7735_GREEN);
		ST7735_DrawPixel(x, 6+(altura*51), ST7735_GREEN);
		ST7735_DrawPixel(x, 5+48+(altura*51), ST7735_GREEN);
		ST7735_DrawPixel(x, 4+48+(altura*51), ST7735_GREEN);
	}

	for(int y = 5+(altura*51); y < 5+48+(altura*51); y++) {
		ST7735_DrawPixel(5, y, ST7735_GREEN);
		ST7735_DrawPixel(6, y, ST7735_GREEN);
		ST7735_DrawPixel(ST7735_WIDTH-5, y, ST7735_GREEN);
		ST7735_DrawPixel(ST7735_WIDTH-6, y, ST7735_GREEN);
	}
}

//Dibuja la hora de inicio y de fin de una alarma determinada
void printAlarma(RTC_TimeTypeDef horaON, RTC_TimeTypeDef horaOFF, uint8_t altura){
	ST7735_FillRectangleFast(5, 5+51*altura, ST7735_WIDTH-10, 48, ST7735_WHITE);
	char alarma[12];
	sprintf(alarma,"ON:  %02d:%02d",horaON.Hours,horaON.Minutes);
	ST7735_WriteString(10, 10+51*altura, alarma, Font_11x18, ST7735_BLACK, ST7735_WHITE);
	sprintf(alarma,"OFF: %02d:%02d",horaOFF.Hours,horaOFF.Minutes);
	ST7735_WriteString(10, 30+51*altura, alarma, Font_11x18, ST7735_BLACK, ST7735_WHITE);
}

//Dibuja el bot??n de creaci??n de alarmas
void printCrearAlarma(uint8_t altura){
	ST7735_FillRectangleFast(5, 5+51*altura, ST7735_WIDTH-10, 48, ST7735_WHITE);
	for(uint16_t x = 0; x <= 17; x++){
		for(uint16_t y = 0; y <= 17; y++){
			if(x*x+y*y < 289){
				if(x < 3 && y < 12){
					continue;
				}
				if(x < 12 && y < 3){
					continue;
				}
				ST7735_DrawPixel(64+x, (29+51*altura)+y, ST7735_GREEN);
				ST7735_DrawPixel(64+x, (29+51*altura)-y, ST7735_GREEN);
				ST7735_DrawPixel(64-x, (29+51*altura)+y, ST7735_GREEN);
				ST7735_DrawPixel(64-x, (29+51*altura)-y, ST7735_GREEN);
			}
		}
	}
}

//Dibuja el bot??n de "OK"
void printOK(uint8_t altura){
	ST7735_FillRectangleFast(5, 5+51*altura, ST7735_WIDTH-10, 48, ST7735_WHITE);
	ST7735_WriteString(48, 16+51*altura, "OK", Font_16x26, ST7735_BLACK, ST7735_WHITE);
}

//Dibuja el bot??n de cambiar hora
void printCambiarHora(uint8_t altura){
	ST7735_FillRectangleFast(5, 5+51*altura, ST7735_WIDTH-10, 48, ST7735_WHITE);
	ST7735_WriteString(25, 10+51*altura, "Cambiar", Font_11x18, ST7735_BLACK, ST7735_WHITE);
	ST7735_WriteString(42, 30+51*altura, "Hora", Font_11x18, ST7735_BLACK, ST7735_WHITE);
}

//Dibuja la hora actual del sistema en la esquina superior derecha
void printTime(){
	char hora[6];
	sprintf(hora,"%02d:%02d",sTime.Hours,sTime.Minutes);
	ST7735_WriteString(92, 0, hora, Font_7x10, ST7735_BLACK, ST7735_WHITE);
}



/***************************
 *GESTI??N DE INTERRUPCIONES*
 ***************************/

//Interrupci??n de botones
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t last_press = 0;
	if(HAL_GetTick() < last_press){ //Debouncer
		return;
	}

	if(GPIO_Pin==GPIO_PIN_0){
		AbrirValvula();
	}
	else if(GPIO_Pin==GPIO_PIN_2){
		CerrarValvula();
	}
	else if(GPIO_Pin==GPIO_PIN_3){
		pantalla = Modo_Actual;
		update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
	}
	last_press = HAL_GetTick()+50;
}

//Interrupci??n de RTC
//
//Seg??n la flag "isTimeToTurnOn", enciende o apaga la electrov??lvula
//y la invierte para la pr??xima alarma.
//Luego llama a la funci??n "nextAlarma()" para generar la siguiente interrupci??n
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15, isTimeToTurnOn);
	isTimeToTurnOn ^= isTimeToTurnOn;
	nextAlarma();
}

//Interrupci??n de temporizador
//
//Si est?? en la pantalla estado, usa esta interrupci??n para actualizar la hora mostrada en la pantalla
//De lo contrario, si salta, significa que el usuario lleva mucho tiempo sin interactuar con el men?? y
//regresa a la pantalla de estado.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if (htim->Instance == TIM3){
		if(pantalla == Estado)
		{
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); //La fecha no se usa
			//Linea a??adida porque para que HAL_RTC_GetTime funcione correctamente, es necesario
			//Llamar a HAL_RTC_GetDate despues
			printTime();
		}
		//Si est?? en modo debug, deshabilita el regreso a la pantalla de estado por temporizador
		#ifndef __DEBUG__
		else
		{
			pantalla = Estado;
			update_screen = 1;__HAL_TIM_SET_COUNTER(&htim3, 0);
			HAL_NVIC_EnableIRQ(EXTI3_IRQn);
			if(modo == Manual){
				HAL_NVIC_EnableIRQ(EXTI0_IRQn);
				HAL_NVIC_EnableIRQ(EXTI2_IRQn);
			}
			__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		#endif
	}
}

//Crea la siguiente interrupci??n del RTC
//
//Para esto cuenta con el flag "isTimeToTurnOn" que indica si en la siguiente
//alarma se debe encender o apagar la bomba, en funci??n de esto toma la siguiente
//hora del vector de horas de encendido o del vector de horas de apagado
//El ??ndice "siguiente_alarma" se incrementa en 1 cada vez que se genera una
//interrupci??n de apagado de la bomba, ya que el apagado se considera el fin de
//cada ciclo de alarmas
void nextAlarma(){

	RTC_AlarmTypeDef sAlarm = {0};

	if(isTimeToTurnOn)
	{
		sAlarm.AlarmTime.Hours = alarmasON[siguiente_alarma].Hours;
		sAlarm.AlarmTime.Minutes = alarmasON[siguiente_alarma].Minutes;
	}
	else
	{
		sAlarm.AlarmTime.Hours = alarmasOFF[siguiente_alarma].Hours;
		sAlarm.AlarmTime.Minutes = alarmasOFF[siguiente_alarma].Minutes;
		siguiente_alarma = (siguiente_alarma+1)%num_alarmas;
	}


	sAlarm.AlarmTime.Seconds = 0x0;
	sAlarm.AlarmTime.SubSeconds = 0x0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x31;
	sAlarm.Alarm = RTC_ALARM_A;

	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
