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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "usbd_custom_hid_if.h"
//#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
/*
typedef struct{
	uint16_t X_16bit,Y_16bit,Z_16bit;
}QUEUE_DATA;
*/

typedef struct{
	//int16_t X_16bit,Y_16bit,Z_16bit;
}QUEUE_DATA;

typedef struct{
	int16_t X_16bit;
	int16_t Y_16bit;
	int16_t Z_16bit;

	double angle_ax;
	double angle_ay;
	double angle_az;
	double gx;
	double gy;
	double gz;
}LIS3DSH;


typedef struct{
	//double angle_ax, angle_ay, angle_az;
//	double ax;
//	double ay;
//	double az;
}QUEUE_UART;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define XH_REG 0x29
#define XL_REG 0x28

#define YH_REG 0x2B
#define YL_REG 0x2A

#define ZH_REG 0x2D
#define ZL_REG 0x2C

#define OUT_T_REG 0x0C

#define STATUS_REG 0x27

///testttttt
#define SENS 16834
//#define TO_DEG 57.2958

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Read_XYZ */
osThreadId_t Read_XYZHandle;
uint32_t Read_XYZBuffer[ 256 ];
osStaticThreadDef_t Read_XYZControlBlock;
const osThreadAttr_t Read_XYZ_attributes = {
  .name = "Read_XYZ",
  .cb_mem = &Read_XYZControlBlock,
  .cb_size = sizeof(Read_XYZControlBlock),
  .stack_mem = &Read_XYZBuffer[0],
  .stack_size = sizeof(Read_XYZBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Angle_Task */
osThreadId_t Angle_TaskHandle;
uint32_t Angle_TaskBuffer[ 256 ];
osStaticThreadDef_t Angle_TaskControlBlock;
const osThreadAttr_t Angle_Task_attributes = {
  .name = "Angle_Task",
  .cb_mem = &Angle_TaskControlBlock,
  .cb_size = sizeof(Angle_TaskControlBlock),
  .stack_mem = &Angle_TaskBuffer[0],
  .stack_size = sizeof(Angle_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_test */
osThreadId_t Task_testHandle;
uint32_t Task_testBuffer[ 512 ];
osStaticThreadDef_t Task_testControlBlock;
const osThreadAttr_t Task_test_attributes = {
  .name = "Task_test",
  .cb_mem = &Task_testControlBlock,
  .cb_size = sizeof(Task_testControlBlock),
  .stack_mem = &Task_testBuffer[0],
  .stack_size = sizeof(Task_testBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for QUEUE_DATA */
osMessageQueueId_t QUEUE_DATAHandle;
const osMessageQueueAttr_t QUEUE_DATA_attributes = {
  .name = "QUEUE_DATA"
};
/* Definitions for QUEUE_UART */
osMessageQueueId_t QUEUE_UARTHandle;
const osMessageQueueAttr_t QUEUE_UART_attributes = {
  .name = "QUEUE_UART"
};
/* Definitions for Mutex1 */
osMutexId_t Mutex1Handle;
const osMutexAttr_t Mutex1_attributes = {
  .name = "Mutex1"
};
/* Definitions for AccelSem */
osSemaphoreId_t AccelSemHandle;
const osSemaphoreAttr_t AccelSem_attributes = {
  .name = "AccelSem"
};
/* USER CODE BEGIN PV */
char test[30];

_Bool LED_flag=RESET;


LIS3DSH accel;

/*
int16_t X_16bit,Y_16bit,Z_16bit;

double angle_ax, angle_ay, angle_az;
double ax;
double ay;
double az;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void *argument);
void StartRead_XYZ(void *argument);
void StartAngle_Task(void *argument);
void StartTask_test(void *argument);

/* USER CODE BEGIN PFP */
void Acc_Inin(void);
double Filter_Kalman(double val);
void LED_Proc(double angle_ax, double angle_ay);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Acc_Inin(void){

	uint8_t Reg_ctrl_4=0x20;
	uint8_t acc_set=0x57;
	uint8_t Reg_ctrl_5=0x24;
	uint8_t acc_reg=0x00;

	uint8_t Reg_ctrl_3=0x23;
	uint8_t acc_int=0x88;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, &Reg_ctrl_4, 1);
	HAL_SPI_Transmit_DMA(&hspi1, &acc_set, 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);



    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(&hspi1, &Reg_ctrl_5, 1);
    HAL_SPI_Transmit_DMA(&hspi1, &acc_reg, 1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(&hspi1, &Reg_ctrl_3, 1);
    HAL_SPI_Transmit_DMA(&hspi1, &acc_int, 1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);


	/*	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		Reg_ctrl_5=0x80|Reg_ctrl_5;
		HAL_SPI_Transmit(&hspi1, &Reg_ctrl_5, 1, 100);
		HAL_SPI_Receive(&hspi1, &test_acc, 1, 100);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

		*/
/*
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
HAL_SPI_Transmit_DMA(&hspi1, &Reg_ctrl_3, 1);
HAL_SPI_Transmit_DMA(&hspi1, &acc_int2, 1);
//HAL_SPI_Transmit(&hspi1, &Reg_ctrl_5, 1, 100);
//HAL_SPI_Transmit(&hspi1, &acc_reg, 1, 100);
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
*/

}


double Filter_Kalman(double val) { //функция фильтрации

	float varVolt = 0.3; // среднее отклонение (расчет в программе)
	float varProcess = 0.2; // скорость реакции на изменение (подбирается вручную)
	float Pc = 0.0;
	float G = 0.0;
	float P = 1.0;
	float Xp = 0.0;
	float Zp = 0.0;
	float Xe = 0.0;
/*

Pc = P + varProcess;
G = Pc/(Pc + varVolt);
P = (1-G)*Pc;
Xp = Xe;
Zp = Xp;
Xe = G*(val-Zp)+Xp; // "фильтрованное" значение

return(Xe);
*/
	return(val);
}

void LED_Proc(double angle_ax,double angle_ay){

	if(angle_ax>=10){

		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_SET);
	}
	if (angle_ax<=-10){
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET);
	}


	if ((angle_ax>-10) && (angle_ax<10))
	{
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET);

	}

	if(angle_ay>=10){
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin,GPIO_PIN_SET);
	}
	if (angle_ay<=-10){

		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin,GPIO_PIN_RESET);
	}


	if ((angle_ay>-10) && (angle_ay<10))
			{

		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin,GPIO_PIN_RESET);

	}

}


//uint8_t USB_TX_Buffer[65];
uint8_t USB_RX_Buffer[65];


//uint8_t dataUART[65];
int USB_packet_enable=0;
extern USBD_HandleTypeDef hUsbDeviceFS;   //указатель на дескриптор

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
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  Acc_Inin();
  sprintf(test, "test of Accselerometr\r\n");
 // CDC_Transmit_FS((uint8_t*)&test, strlen(test));
 // HAL_UART_Transmit(&huart2, (uint8_t*)&test, strlen(test), 100);
  //=============================================================================
  //Fill TX buffer
/*

  USB_TX_Buffer[0]=0x01;  //ID report (=0x01 for input)
  USB_TX_Buffer[1]=0x48;  //'H'
  USB_TX_Buffer[2]=0x65;  //'e'
  USB_TX_Buffer[3]=0x6c;  //'l'
  USB_TX_Buffer[4]=0x6c;  //'l'
  USB_TX_Buffer[5]=0x6f;  //'o'
  USB_TX_Buffer[6]=0x21;  //'!'
  USB_TX_Buffer[7]=0x00;  // null-terminate string

*/
  //=============================================================================
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Mutex1 */
  Mutex1Handle = osMutexNew(&Mutex1_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of AccelSem */
  AccelSemHandle = osSemaphoreNew(1, 1, &AccelSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QUEUE_DATA */
  QUEUE_DATAHandle = osMessageQueueNew (2, sizeof(QUEUE_DATA), &QUEUE_DATA_attributes);

  /* creation of QUEUE_UART */
  QUEUE_UARTHandle = osMessageQueueNew (2, sizeof(QUEUE_UART), &QUEUE_UART_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Read_XYZ */
  Read_XYZHandle = osThreadNew(StartRead_XYZ, NULL, &Read_XYZ_attributes);

  /* creation of Angle_Task */
  Angle_TaskHandle = osThreadNew(StartAngle_Task, NULL, &Angle_Task_attributes);

  /* creation of Task_test */
  Task_testHandle = osThreadNew(StartTask_test, NULL, &Task_test_attributes);

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  htim7.Init.Prescaler = 15;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
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

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port, CS_LIS3DSH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_LIS3DSH_Pin */
  GPIO_InitStruct.Pin = CS_LIS3DSH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_LIS3DSH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_ORANGE_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : INT1_Pin */
  GPIO_InitStruct.Pin = INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRead_XYZ */
/**
* @brief Function implementing the Read_XYZ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRead_XYZ */
void StartRead_XYZ(void *argument)
{
  /* USER CODE BEGIN StartRead_XYZ */
	//QUEUE_DATA XYZ;



	uint8_t X_read_L=XL_REG;
	uint8_t OUT_X_H;
	uint8_t OUT_X_L;

	uint8_t Y_read_L=YL_REG;
	uint8_t OUT_Y_H;
	uint8_t OUT_Y_L;

	uint8_t Z_read_L=ZL_REG;
	uint8_t OUT_Z_H;
	uint8_t OUT_Z_L;

	uint8_t Tmp_read=OUT_T_REG;
	int8_t Temp;


	uint8_t StatusReg = STATUS_REG;
	uint8_t StatusRegData;



  /* Infinite loop */
  for(;;)
  {

	 // xSemaphoreTake(AccelSemHandle, portMAX_DELAY);
	//  osSemaphoreAcquire(AccelSemHandle, 50);
//	  osSemaphoreRelease(AccelSemHandle);
	  osMutexAcquire(Mutex1Handle, osWaitForever);



	 // taskENTER_CRITICAL();
	//		  {

	  StatusRegData=0x00;
	  while(StatusRegData!=0x08){

	    HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin , GPIO_PIN_RESET);
	    StatusReg|=0x80;
		HAL_SPI_Transmit_DMA(&hspi1, &StatusReg, 1);//(&hspi1, &X_read_L, 1, 100);
		HAL_SPI_Receive_DMA(&hspi1, &StatusRegData, 1);//(&hspi1, &OUT_X_L, 1, 100);
		HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_SET);

		StatusRegData&=0x08;
	  }

	    HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin , GPIO_PIN_RESET);
		X_read_L=0x80|X_read_L;
		HAL_SPI_Transmit_DMA(&hspi1, &X_read_L, 1);//(&hspi1, &X_read_L, 1, 100);
		HAL_SPI_Receive_DMA(&hspi1, &OUT_X_L, 1);//(&hspi1, &OUT_X_L, 1, 100);
	//	HAL_SPI_TransmitReceive_DMA(&hspi1, &X_read_L, &OUT_X_L, 1);
		//HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_RESET);
		//X_read_H=0x80|X_read_H;
		//HAL_SPI_Transmit_DMA(&hspi1, &X_read_H, 1);//(&hspi1, &X_read_H, 1, 100);
		HAL_SPI_Receive_DMA(&hspi1, &OUT_X_H, 1);//(&hspi1, &OUT_X_H, 1, 100);
	//	HAL_SPI_TransmitReceive_DMA(&hspi1, &X_read_H, &OUT_X_H, 1);
		HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_SET);
	//	XYZ.X_16bit=((OUT_X_H<<8)|(OUT_X_L));

		accel.X_16bit=((OUT_X_H<<8)|(OUT_X_L));



		HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_RESET);
		Y_read_L=0x80|Y_read_L;
		HAL_SPI_Transmit_DMA(&hspi1, &Y_read_L, 1); //(&hspi1, &Y_read_L, 1, 100);
		HAL_SPI_Receive_DMA(&hspi1, &OUT_Y_L, 1); //(&hspi1, &OUT_Y_L, 1, 100);
	//	HAL_SPI_TransmitReceive_DMA(&hspi1, &Y_read_L, &OUT_Y_L, 1);
		//HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_RESET);
		//Y_read_H=0x80|Y_read_H;
		//HAL_SPI_Transmit_DMA(&hspi1, &Y_read_H, 1); //(&hspi1, &Y_read_H, 1, 100);
		HAL_SPI_Receive_DMA(&hspi1, &OUT_Y_H, 1); //(&hspi1, &OUT_Y_H, 1, 100);
	//	HAL_SPI_TransmitReceive_DMA(&hspi1, &Y_read_H, &OUT_Y_H, 1);
		HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_SET);
	//	XYZ.Y_16bit=((OUT_Y_H<<8)|(OUT_Y_L));

		accel.Y_16bit=((OUT_Y_H<<8)|(OUT_Y_L));

		HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_RESET);
		Z_read_L=0x80|Z_read_L;
		HAL_SPI_Transmit_DMA(&hspi1, &Z_read_L, 1); //(&hspi1, &Z_read_L, 1, 100);
		HAL_SPI_Receive_DMA(&hspi1, &OUT_Z_L, 1); //(&hspi1, &OUT_Z_L, 1, 100);
	//	HAL_SPI_TransmitReceive_DMA(&hspi1, &Z_read_L, &OUT_Z_L, 1);
		//HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_RESET);
		//Z_read_H=0x80|Z_read_H;
		//HAL_SPI_Transmit_DMA(&hspi1, &Z_read_H, 1); //(&hspi1, &Z_read_H, 1, 100);
		HAL_SPI_Receive_DMA(&hspi1, &OUT_Z_H, 1); //(&hspi1, &OUT_Z_H, 1, 100);
	//	HAL_SPI_TransmitReceive_DMA(&hspi1, &Z_read_H, &OUT_Z_H, 1);
		HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_SET);
	//	XYZ.Z_16bit=((OUT_Z_H<<8)|(OUT_Z_L));

		accel.Z_16bit=((OUT_Z_H<<8)|(OUT_Z_L));

		//HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_RESET);
		//Tmp_read|=0x80;
		//HAL_SPI_Transmit_DMA(&hspi1, &Tmp_read, 1); //(&hspi1, &Z_read_L, 1, 100);
		//HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&Temp, 1); //(&hspi1, &OUT_Z_L, 1, 100);
		//HAL_GPIO_WritePin(CS_LIS3DSH_GPIO_Port,CS_LIS3DSH_Pin, GPIO_PIN_SET);

		//XYZ.Z_16bit=Temp;


	  //}

	//  taskEXIT_CRITICAL();



	//	osMessageQueuePut(QUEUE_DATAHandle, &XYZ, 0, 1);


		  osMutexRelease(Mutex1Handle);

    osDelay(10);
  }
  /* USER CODE END StartRead_XYZ */
}

/* USER CODE BEGIN Header_StartAngle_Task */
/**
* @brief Function implementing the Angle_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAngle_Task */
void StartAngle_Task(void *argument)
{
  /* USER CODE BEGIN StartAngle_Task */
//	QUEUE_DATA XYZ;
//	QUEUE_UART Angle;





	double X_1;
	double Y_1;
	double Z_1;


	double sens=SENS;
	double TO_DEG=57.2958;

  /* Infinite loop */
  for(;;)
  {
	  	  osMutexAcquire(Mutex1Handle, osWaitForever);

///прием данных из очереди

		//  osMessageQueueGet(QUEUE_DATAHandle, &XYZ, 0, 1);



		  ///расчет коэффициентов для углов

		  accel.gx=(double)accel.X_16bit/sens;
		  accel.gy=(double)accel.Y_16bit/sens;
		  accel.gz=(double)accel.Z_16bit/sens;



		  ///фильтр Калмана
		  /*


// переменные для калмана
float varVolt = 0; // среднее отклонение (расчет в программе)
float varProcess = 2; // скорость реакции на изменение (подбирается вручную)
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

float filter(float val) { //функция фильтрации
Pc = P + varProcess;
G = Pc/(Pc + varVolt);
P = (1-G)*Pc;
Xp = Xe;
Zp = Xp;
Xe = G*(val-Zp)+Xp; // "фильтрованное" значение
return(Xe);
}

*/

	/*

		  Angle.angle_ax=filter(XYZ.X_16bit)


	*/



		  ///расчет угла через арктангенс с умножением на константу для перевода из радианы в градусы



		  X_1=TO_DEG*atan(accel.gx/(sqrt(pow(accel.gy,2)+pow(accel.gz,2))));
		  Y_1=TO_DEG*atan(accel.gy/(sqrt(pow(accel.gx,2)+pow(accel.gz,2))));
		 Z_1=TO_DEG*atan(accel.gz/(sqrt(pow(accel.gx,2)+pow(accel.gy,2))));

		  accel.angle_ax=Filter_Kalman(X_1);
		  accel.angle_ay=Filter_Kalman(Y_1);
		  accel.angle_az=Filter_Kalman(Z_1);





		  ///отправка в очередь


//	  osMessageQueuePut(QUEUE_UARTHandle, &Angle, 0, 1);

		  osMutexRelease(Mutex1Handle);


    osDelay(10);
  }
  /* USER CODE END StartAngle_Task */
}

/* USER CODE BEGIN Header_StartTask_test */
/**
* @brief Function implementing the Task_test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_test */
void StartTask_test(void *argument)
{
  /* USER CODE BEGIN StartTask_test */
//	QUEUE_DATA XYZ;
//	QUEUE_UART Angle;

	char dataUART[64];

	uint8_t USB_TX_Buffer[65];

	char rep=0x01;


  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(Mutex1Handle, osWaitForever);


	//  osMessageQueueGet(QUEUE_DATAHandle, &XYZ, 0, 1);




//	  osMessageQueueGet(QUEUE_UARTHandle, &Angle, 0, 1);

	  if(LED_flag==SET){
	  LED_Proc(accel.angle_ax,accel.angle_ay);
	  }
	  else{
		  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET);
	  }
	  ///вывод значений

//	 sprintf(dataUART,"X=%d Y=%d Z=%d\r\n", XYZ.X_16bit, XYZ.Y_16bit, XYZ.Z_16bit);
//	  HAL_UART_Transmit(&huart2, (uint8_t*)&dataUART, strlen(dataUART), osWaitForever);
//
	  ///вывод тяжести g

//	  snprintf(dataUART,sizeof(dataUART),"G_X=%.2f G_Y=%.2f G_Z=%.2f\r\n", Angle.ax, Angle.ay, Angle.az);
	//  HAL_UART_Transmit(&huart2, (uint8_t*)&dataUART, strlen(dataUART), osWaitForever);



	  ///вывод углов

	  snprintf((char*)&USB_TX_Buffer,sizeof(USB_TX_Buffer),"%c %.2f \t %.2f \t %.2f\r\n",rep, accel.angle_ax, accel.angle_ay, accel.angle_az);

	  USB_TX_Buffer[0]=0x01;  //ID report (=0x01 for input)



	//  memcpy(USB_TX_Buffer[1],dataUART,sizeof(dataUART));
//	  snprintf((char*)&USB_TX_Buffer,sizeof(USB_TX_Buffer),"%.2f %.2f %.2f\r\n", accel.angle_ax, accel.angle_ay, accel.angle_az);
	//  snprintf(dataUART,sizeof(dataUART),"%.2f %.2f %.2f\r\n", Angle.angle_ax, Angle.angle_ay, Angle.angle_az);
	//  snprintf(dataUART,sizeof(dataUART),"1X_ax=%.2f 2Y_ay=%.2f 3Z_az=%.2f\r\n", Angle.angle_ax, Angle.angle_ay, Angle.angle_az);
	//  CDC_Transmit_FS((uint8_t*)&dataUART, strlen(dataUART));

		 USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,USB_TX_Buffer,sizeof(USB_TX_Buffer));
	//  HAL_UART_Transmit(&huart2, (uint8_t*)&dataUART, strlen(dataUART), osWaitForever);

	  osMutexRelease(Mutex1Handle);

    osDelay(10);
  }
  /* USER CODE END StartTask_test */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
