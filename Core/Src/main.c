/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "delay.h"
#include "misc.h"
#include "adc.h"
#include "can.h"
#include "sensors.h"
#include "outputs.h"
#include "etc.h"
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
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
#ifdef DEBUG
volatile static uint32_t fast_irq_start = 0;
volatile static uint32_t fast_irq_end = 0;
volatile static uint16_t fast_irq_times = 0;
volatile static uint32_t fast_irq_time = 0;
volatile static float fast_irq_avg = 0;
volatile static float fast_irq_max = 0;

volatile static uint32_t slow_irq_start = 0;
volatile static uint32_t slow_irq_end = 0;
volatile static uint16_t slow_irq_times = 0;
volatile static uint32_t slow_irq_time = 0;
volatile static float slow_irq_avg = 0;
volatile static float slow_irq_max = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

STATIC_INLINE void fast_loop(void)
{
#ifdef DEBUG
  fast_irq_start = Delay_Tick;
#endif

  adc_fast_loop();
  Misc_Fast_Loop();
  etc_irq_fast_loop();

#ifdef DEBUG
  //For time measurement taken by the fast irq handler. No need to optimize anything here
  fast_irq_end = Delay_Tick;
  fast_irq_time = DelayDiff(fast_irq_end, fast_irq_start);

  if(fast_irq_avg == 0)
    fast_irq_avg = fast_irq_time;
  else if(fast_irq_times < 1000) {
    fast_irq_avg = fast_irq_avg * 0.99f + fast_irq_time * 0.01f;
    if(fast_irq_time > fast_irq_max)
      fast_irq_max = fast_irq_time;
    else fast_irq_max = fast_irq_max * 0.99f + fast_irq_time * 0.01f;
    fast_irq_times++;
  } else {
    fast_irq_avg = fast_irq_avg * 0.99999f + fast_irq_time * 0.00001f;
    if(fast_irq_time > fast_irq_max)
      fast_irq_max = fast_irq_time;
    else fast_irq_max = fast_irq_max * 0.99999f + fast_irq_time * 0.00001f;
  }
#endif
}

STATIC_INLINE void slow_loop(void)
{
#ifdef DEBUG
  slow_irq_start = Delay_Tick;
#endif

  adc_slow_loop();
  Misc_Loop();
  sensors_loop();
  outputs_loop();
  etc_irq_slow_loop();

#ifdef DEBUG
  //For time measurement taken by the slow irq handler. No need to optimize anything here
  slow_irq_end = Delay_Tick;
  slow_irq_time = DelayDiff(slow_irq_end, slow_irq_start);

  if(slow_irq_avg == 0)
    slow_irq_avg = slow_irq_time;
  else if(slow_irq_times < 1000) {
    slow_irq_avg = slow_irq_avg * 0.99f + slow_irq_time * 0.01f;
    if(slow_irq_time > slow_irq_max)
      slow_irq_max = slow_irq_time;
    else slow_irq_max = slow_irq_max * 0.99f + slow_irq_time * 0.01f;
    slow_irq_times++;
  } else {
    slow_irq_avg = slow_irq_avg * 0.99999f + slow_irq_time * 0.00001f;
    if(slow_irq_time > slow_irq_max)
      slow_irq_max = slow_irq_time;
    else slow_irq_max = slow_irq_max * 0.99999f + slow_irq_time * 0.00001f;
  }
#endif
}

INLINE void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6) {
    fast_loop();
  } else if (htim == &htim7) {
    slow_loop();
  }
}

INLINE ITCM_FUNC void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    //htim2.Instance->CCR1 = o2_pwm_period;
  }
}

INLINE void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if(hspi == &hspi2) {
    Misc_TxCpltCallback(hspi);
  } else if(hspi == &hspi1) {
    ADC_TxCpltCallback(hspi);
  }
}

INLINE void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if(hspi == &hspi2) {
    Misc_RxCpltCallback(hspi);
  } else if(hspi == &hspi1) {
    ADC_RxCpltCallback(hspi);
  }
}
INLINE void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if(hspi == &hspi2) {
    Misc_TxRxCpltCallback(hspi);
  } else if(hspi == &hspi1) {
    ADC_TxRxCpltCallback(hspi);
  }
}

INLINE void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi)
{
  if(hspi == &hspi2) {
    Misc_ErrorCallback(hspi);
  } else if(hspi == &hspi1) {
    ADC_ErrorCallback(hspi);
  }
}

INLINE void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
  can_txfifo_aborted_callback(hcan, CAN_TX_MAILBOX0);
}

INLINE void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
  can_txfifo_aborted_callback(hcan, CAN_TX_MAILBOX1);
}

INLINE void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
  can_txfifo_aborted_callback(hcan, CAN_TX_MAILBOX2);
}

INLINE void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  can_txfifo_completed_callback(hcan, CAN_TX_MAILBOX0);
}

INLINE void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  can_txfifo_completed_callback(hcan, CAN_TX_MAILBOX1);
}

INLINE void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  can_txfifo_completed_callback(hcan, CAN_TX_MAILBOX2);
}

INLINE void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  can_rxfifo_pending_callback(hcan, CAN_RX_FIFO0);
}

INLINE void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  can_rxfifo_pending_callback(hcan, CAN_RX_FIFO1);
}

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
  MX_DMA_Init();
  MX_CAN_Init();
  //MX_IWDG_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  //Freeze peripherial during debug
  DBGMCU->APB1FZ = 0x7E01BFF;
  DBGMCU->APB2FZ = 0x70003;

  __HAL_DBGMCU_FREEZE_TIM2();
  __HAL_DBGMCU_FREEZE_IWDG();

  __HAL_DBGMCU_UNFREEZE_TIM3();

  HAL_PWR_EnableBkUpAccess();

  HAL_GPIO_WritePin(MCU_OUTPUT_EN_GPIO_Port, MCU_OUTPUT_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MCU_OUTPUT_EN_GPIO_Port, MCU_OUTPUT_EN_Pin, GPIO_PIN_SET);

  DelayInit(&htim2);

  sensors_init();
  outputs_init();

  //sensors_register_digital(SensorOilPressure, SENS_OIL_GPIO_Port, SENS_OIL_Pin, 1);

  //outputs_register(OutFuelPumpRelay, FUEL_PUMP_GPIO_Port, FUEL_PUMP_Pin, 1, GPIO_PIN_SET);

  adc_register(AdcChTps1,           ADC_RANGE_0P1250, 1000, ADC_FILTER_ENABLE);
  adc_register(AdcChTps2,           ADC_RANGE_0P1250, 1000, ADC_FILTER_ENABLE);
  adc_register(AdcChPedal1,         ADC_RANGE_0P1250, 1000, ADC_FILTER_ENABLE);
  adc_register(AdcChPedal2,         ADC_RANGE_0P1250, 1000, ADC_FILTER_ENABLE);
  adc_register(AdcChRsvd5,          ADC_RANGE_0P1250, 1000, ADC_FILTER_ENABLE);
  adc_register(AdcChRsvd6,          ADC_RANGE_0P1250, 1000, ADC_FILTER_ENABLE);
  adc_register(AdcChPowerVoltage,   ADC_RANGE_0P2500, 2000, ADC_FILTER_ENABLE);
  adc_register(AdcChRefVoltage,     ADC_RANGE_0P1250, 2000, ADC_FILTER_ENABLE);

  adc_init(&hspi1);

  Misc_Init(&hspi2);

  can_init(&hcan);

  etc_init();

  //HAL_IWDG_Refresh(&hiwdg);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    etc_loop();
    //HAL_IWDG_Refresh(&hiwdg);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 128;
  hiwdg.Init.Reload = 128;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  htim2.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1000000) -1;
  htim2.Init.Period = DelayMask;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 256-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  htim3.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1000000) -1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 48-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  htim6.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1000000) -1;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 48-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
  htim7.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1000000) -1;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUTPUT_CRUIZE_G_Pin|OUTPUT_CRUIZE_R_Pin|OUTPUT_RSVD3_Pin|OUTPUT_RSVD4_Pin
                          |MOTOR_DIS_Pin|CAN_LBK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCU_OUTPUT_EN_Pin|MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_NRST_Pin|SPI1_NRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_NSS_MOTOR_Pin|SPI2_NSS_OUT_Pin|SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : INPUT_CRUIZE_START_Pin */
  GPIO_InitStruct.Pin = INPUT_CRUIZE_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INPUT_CRUIZE_START_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_CRUIZE_G_Pin OUTPUT_CRUIZE_R_Pin OUTPUT_RSVD3_Pin OUTPUT_RSVD4_Pin
                           MOTOR_DIR_Pin MOTOR_DIS_Pin */
  GPIO_InitStruct.Pin = OUTPUT_CRUIZE_G_Pin|OUTPUT_CRUIZE_R_Pin|OUTPUT_RSVD3_Pin|OUTPUT_RSVD4_Pin
                          |MOTOR_DIR_Pin|MOTOR_DIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_OUTPUT_EN_Pin */
  GPIO_InitStruct.Pin = MCU_OUTPUT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_OUTPUT_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_GND_Pin */
  GPIO_InitStruct.Pin = INPUT_GND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT_GND_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_NRST_Pin SPI2_NSS_MOTOR_Pin SPI2_NSS_OUT_Pin SPI1_NSS_Pin
                           SPI1_NRST_Pin */
  GPIO_InitStruct.Pin = SPI2_NRST_Pin|SPI2_NSS_MOTOR_Pin|SPI2_NSS_OUT_Pin|SPI1_NSS_Pin
                          |SPI1_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_RSVD6_Pin INPUT_RSVD5_Pin INPUT_RSVD4_Pin */
  GPIO_InitStruct.Pin = INPUT_RSVD6_Pin|INPUT_RSVD5_Pin|INPUT_RSVD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_LBK_Pin */
  GPIO_InitStruct.Pin = CAN_LBK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CAN_LBK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_BRAKE_Pin INPUT_CRUIZE_STOP_Pin */
  GPIO_InitStruct.Pin = INPUT_BRAKE_Pin|INPUT_CRUIZE_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
