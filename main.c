/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
	uint8_t buff[4];
	int32_t value;
} packet_buffer_t;

typedef struct {                                // object data type
  uint32_t x_cord;
  uint32_t y_cord;
  uint32_t speed; // For now just one speed
  uint32_t confirm_byte;
  uint32_t is_done;
  //... Add more thing in future
} Msqcom_t;

struct robo_se {
  float current_speed_l;
  float current_speed_r;
  float current_x;
  float current_y;
  float current_angle;
};

struct robo_se my_robot;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 50
#define PWMPERIOD 1000
#define MSGQUEUE_OBJECTS 16

#define PI 3.141592653589
#define SHAFT_PULSE_PER_ROT 28.0
#define GEAR_RATIO 210.0
#define PULSE_PER_REVOLUTION (GEAR_RATIO*SHAFT_PULSE_PER_ROT)
#define WHEEL_DIAMETER 4 //cm
#define WHEEL_DISTANCE 16.05 //cm
#define WHEEL_CIRCUMFERENCE PI*WHEEL_DIAMETER
#define DIST_CONVERSION WHEEL_CIRCUMFERENCE/PULSE_PER_REVOLUTION

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pwmTask */
osThreadId_t pwmTaskHandle;
const osThreadAttr_t pwmTask_attributes = {
  .name = "pwmTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pwmControlQueue */
osMessageQueueId_t pwmControlQueueHandle;
const osMessageQueueAttr_t pwmControlQueue_attributes = {
  .name = "pwmControlQueue"
};
/* Definitions for encoderQueue */
osMessageQueueId_t encoderQueueHandle;
const osMessageQueueAttr_t encoderQueue_attributes = {
  .name = "encoderQueue"
};
/* USER CODE BEGIN PV */

uint8_t RxBuffer[BUFFER_SIZE];
uint8_t TxBuffer[BUFFER_SIZE];

packet_buffer_t encoder_value;
packet_buffer_t command_value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void pwmTaskFunction(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_SPI_TransmitReceive_DMA(&hspi1, TxBuffer, RxBuffer, BUFFER_SIZE);



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of pwmControlQueue */
  pwmControlQueueHandle = osMessageQueueNew (16, sizeof(Msqcom_t), &pwmControlQueue_attributes);

  /* creation of encoderQueue */
  encoderQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &encoderQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of pwmTask */
  pwmTaskHandle = osThreadNew(pwmTaskFunction, NULL, &pwmTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  /* USER CODE END TIM2_Init 2 */

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
  htim4.Init.Prescaler = 60 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PB1_SDBY_Pin|PB5_R_Pin|PB6_R_Pin|PB7_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PA15_L_GPIO_Port, PA15_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1_SDBY_Pin PB5_R_Pin PB6_R_Pin PB7_L_Pin */
  GPIO_InitStruct.Pin = PB1_SDBY_Pin|PB5_R_Pin|PB6_R_Pin|PB7_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15_L_Pin */
  GPIO_InitStruct.Pin = PA15_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PA15_L_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	osDelay(1000);
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // Slow blink when receiving a command

//	TxBuffer[5] = encoder_value.buff[0];
//	TxBuffer[6] = encoder_value.buff[1];
//	TxBuffer[7] = encoder_value.buff[2];
//	TxBuffer[8] = encoder_value.buff[3];
//	typedef struct {                                // object data type
//	  uint32_t x_cord;
//	  uint32_t y_cord;
//	  uint32_t speed; // For now just one speed
//	  uint32_t confirm_byte;
//	  uint32_t is_done;
//	  //... Add more thing in future
//	} Msqcom_t;

	Msqcom_t msg;
	msg.confirm_byte = 0;

	command_value.buff[0] = RxBuffer[5];
	command_value.buff[1] = RxBuffer[6];
	command_value.buff[2] = RxBuffer[7];
	command_value.buff[3] = RxBuffer[8];
	msg.x_cord = command_value.value;

	command_value.buff[0] = RxBuffer[9];
	command_value.buff[1] = RxBuffer[10];
	command_value.buff[2] = RxBuffer[11];
	command_value.buff[3] = RxBuffer[12];
	msg.y_cord = command_value.value;

	command_value.buff[0] = RxBuffer[13];
	command_value.buff[1] = RxBuffer[14];
	command_value.buff[2] = RxBuffer[15];
	command_value.buff[3] = RxBuffer[16];
	msg.speed = command_value.value;

	msg.is_done = 0;

	osMessageQueuePut(pwmControlQueueHandle, &msg, 0U, 0U);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  char msg[50] = "";
  uint32_t lastMsgSend = 0;
  int lastEncoderValue = 0;
  int encoder_delta = 0;
  float distance;
  /* Infinite loop */
  for(;;)
  {
	 encoder_value.value = (int32_t)TIM2->CNT;

	 if(HAL_GetTick()-lastMsgSend >= 500)
	 {
		 lastMsgSend = HAL_GetTick();

		 encoder_delta = encoder_value.value - lastEncoderValue;
		 sprintf(msg, "encoder value : %ld\r\n", encoder_value.value);
		 CDC_Transmit_FS((uint8_t *)&msg, strlen(msg));

		 sprintf(msg, "delta = %d \r\n", encoder_delta);
		 CDC_Transmit_FS((uint8_t *)&msg, strlen(msg));
		 distance = (encoder_delta / PULSE_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE; // CM
		 my_robot.current_speed_l = distance/0.25;
		 sprintf(msg, "DISTANCE = %f \r\n", distance);
		 CDC_Transmit_FS((uint8_t *)&msg, strlen(msg));
		 lastEncoderValue = encoder_value.value;
	 }


    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_pwmTaskFunction */
/**
* @brief Function implementing the pwmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pwmTaskFunction */
void pwmTaskFunction(void *argument)
{
  /* USER CODE BEGIN pwmTaskFunction */
	MX_USB_DEVICE_Init();
	Msqcom_t msg;
	char msgBuf[50];
	osStatus_t status;

  /* Infinite loop */
  for(;;)
  {
	  // Variable resets after every iteration for new command
	  int pwm_l = 0;
	  int pwm_r = 0;
	  int error_l = 0;
	  int error_r = 0;
	  int error_dif = 0;
	  int error_int = 0;
	  int error_dif_prev = 0;
	  int bias = 0;
	  float K1 = 10.0;
	  float K2= 0.0001;

	  my_robot.current_x = 0;
	  my_robot.current_y = 0;
	  my_robot.current_angle = 0;

	  sprintf(msgBuf, "Wait command\r\n");
	  CDC_Transmit_FS((uint8_t *)&msgBuf, strlen(msgBuf));

	  status = osMessageQueueGet(pwmControlQueueHandle, &msg, NULL, osWaitForever);

	  if(status != osOK ){
		  continue;
	  }

	  sprintf(msgBuf, "COMMAND RECEIVE\r\n");
	  CDC_Transmit_FS((uint8_t *)&msgBuf, strlen(msgBuf));

	  // While loop to execute command

	  while(msg.is_done == 0){
		  sprintf(msgBuf, "speed %f\r\n", my_robot.current_speed_l);
		  CDC_Transmit_FS((uint8_t *)&msgBuf, strlen(msgBuf));
		  error_l = msg.speed - my_robot.current_speed_l;
		  error_r = msg.speed - my_robot.current_speed_r;

		  error_int = K2 * (bias + error_dif + (error_l - error_r));
		  error_dif = error_l - error_r;

		  pwm_l = K1 * (msg.speed - error_l - error_int);
		  pwm_r = K1 * (msg.speed - error_r - error_int);

		  sprintf(msgBuf, "pwm_l %d\r\n", pwm_l);
		  CDC_Transmit_FS((uint8_t *)&msgBuf, strlen(msgBuf));
		  sprintf(msgBuf, "pwm_r %d\r\n", pwm_r);
		  CDC_Transmit_FS((uint8_t *)&msgBuf, strlen(msgBuf));

//		  if(pwm_l < 0 || pwm_l > 100 || pwm_r < 0 || pwm_r > 0){
//			  sprintf(msgBuf, "PWM VIOLATION");
//			  CDC_Transmit_FS((uint8_t *)&msgBuf, strlen(msgBuf));
//			  break;
//		  }
//
//		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWMPERIOD * pwm_l / 100 );
//		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWMPERIOD * pwm_r / 100 );
//
//		  //ALWAYSFORWARD!
//		  HAL_GPIO_WritePin(PB7_L_GPIO_Port, PB7_L_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(PA15_L_GPIO_Port, PA15_L_Pin, GPIO_PIN_RESET);

//		if( msg.Dir_LR[0] == 1 ){ // Forward
//			HAL_GPIO_WritePin(PB7_L_GPIO_Port, PB7_L_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(PA15_L_GPIO_Port, PA15_L_Pin, GPIO_PIN_RESET);
//		}else{ // Backwardz
//			HAL_GPIO_WritePin(PB7_L_GPIO_Port, PB7_L_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(PA15_L_GPIO_Port, PA15_L_Pin, GPIO_PIN_SET);
//		}

	  }//end while

  }// end infinite loop
    osDelay(1);
  /* USER CODE END pwmTaskFunction */
}//end function

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
