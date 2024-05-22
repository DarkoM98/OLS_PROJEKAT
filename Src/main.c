/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "SSD1963.h"
#include "STMPE610.h"

#include "GUI.h"
#include "LCDConf.h"

#include "stdint.h"
#include "stdio.h"

#include "CO23Click.h"
#include <stdbool.h>
#include "Windows/Co2GraphDLG.h"
#include "Windows/WindowDLG.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef uint8_t bool_t;

typedef struct
{
  uint16_t Result;
  uint16_t Count;
} MeasurementMessage_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MEAS_RES_INVALID  (0xFFFFu)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
extern volatile GUI_TIMER_TIME OS_TimeMS;
uint16_t X_koordinata = 0;
uint16_t Y_koordinata = 0;
uint16_t keypressed = 0;
static bool_t NewMeasurementEvent = FALSE;
static MeasurementMessage_t MeasurementMsg =
{
  .Result = MEAS_RES_INVALID,
  .Count = 0u
};

GRAPH_Handle GraphHandle;
GRAPH_DATA_Handle GraphDataHandle;
GRAPH_SCALE_Handle GraphScaleHandle;
GRAPH_SCALE_Handle GraphScaleHandle2;
#define MAX_GRAPH_DATA 1000
#define GRAPH_STEP_SIZE 25
int16_t GraphData[MAX_GRAPH_DATA] = { 0 };
uint16_t lastMeasuredPpm = 0u;
uint16_t MeasurementRateInS = 5;

void HAL_SYSTICK_Callback(void)
{
	OS_TimeMS++;
  C02Click_CyclicJob1ms();
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void Co2MeasurementFinished(uint16_t Co2Ppm);
static void SetupGraph(void);
static void AddMesToGraph(uint16_t MeasuredPpm);
static void SetGraphScale(uint16_t MesRate);
WM_HWIN hWin;

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Co2MeasurementFinished(uint16_t Co2Ppm)
{
  NewMeasurementEvent = TRUE;
  MeasurementMsg.Result = Co2Ppm;
  AddMesToGraph(Co2Ppm);
}
static void PrintToDisplay(char* msg)
{
  GUI_Clear();
  GUI_DispString(msg);
}

static void SetGraphScale(uint16_t MesRate)
{
	float rate = 1.0/MesRate;
	GRAPH_SCALE_SetFactor(GraphScaleHandle, rate);
}
static int mescount = 0;
static void AddMesToGraph(uint16_t MeasuredPpm)
{
	int stepSize = (MeasuredPpm - lastMeasuredPpm)/ (GRAPH_STEP_SIZE - 1);
	for(int x = 0; x < GRAPH_STEP_SIZE; x++) //25pixels = 5s
	{
		  GRAPH_DATA_YT_AddValue(GraphDataHandle, ((int16_t)lastMeasuredPpm + x*(int16_t)stepSize)/10);
	}
	lastMeasuredPpm = MeasuredPpm;
	mescount++;
	if(mescount > 12)
	{
		static int scroll = 0;
		scroll -= 25;
		GRAPH_SCALE_SetOff(GraphScaleHandle, scroll);
	}
}

static void SetupGraph()
{
  GraphHandle = WM_GetDialogItem(hWin, ID_GRAPH_0);
  GRAPH_SetColor(GraphHandle, GUI_LIGHTGRAY, GRAPH_CI_BK);
  GRAPH_SetGridVis(GraphHandle, TRUE);
  GRAPH_SetAutoScrollbar(GraphHandle, GUI_COORD_X, TRUE);

  GraphScaleHandle = GRAPH_SCALE_Create(390, GUI_TA_HCENTER, GRAPH_SCALE_CF_HORIZONTAL, 50);
  GRAPH_SCALE_SetFactor(GraphScaleHandle, 0.2);
  GRAPH_AttachScale(GraphHandle, GraphScaleHandle);

  GraphScaleHandle2 = GRAPH_SCALE_Create(0, GUI_TA_VCENTER, GRAPH_SCALE_CF_VERTICAL, 50);
  GRAPH_SCALE_SetFactor(GraphScaleHandle2, 10);
  GRAPH_AttachScale(GraphHandle, GraphScaleHandle2);

  GraphDataHandle = GRAPH_DATA_YT_Create(GUI_RED, 300, GraphData, 0);
  GRAPH_DATA_YT_SetAlign(GraphDataHandle, GRAPH_ALIGN_LEFT);
  GRAPH_AttachData(GraphHandle, GraphDataHandle);

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /* Initialize all configured peripherals */
  Init_LCD_GPIO();
  Init_TOUCH_GPIO(hi2c1);
  C02Click_Init(hi2c2, Co2MeasurementFinished);

  STMPE610_Init();

  //WM_SetCreateFlags(WM_CF_MEMDEV); // eliminise flickering
  GUI_Init();

  GUI_SetBkColor(GUI_BLUE);
  GUI_Clear();
//  GUI_SetBkColor(GUI_BLACK);
//  GUI_SetFont(&GUI_Font32_1);
//  GUI_DispString("HELLO DM");

  //hWin = CreateWindow();
  char msg[100u] = { 0u };
  GUI_Delay(100);

  C02Click_TriggerContinuousMeasurement(5u); //trigger continuous measurement for 5s rate.

//  hWin=CreateCo2Graph();
  hWin = CreateWindow();
  SetupGraph();
  SetGraphScale(MeasurementRateInS);
//  int16_t graphData[100] = { 0 };
//  GRAPH_DATA_Handle GraphDataHandle = GRAPH_DATA_YT_Create(GUI_LIGHTGRAY, 100, graphData, 100);
//  GRAPH_Handle GraphHandle = WM_GetDialogItem(hWin, ID_GRAPH_0);
//  GRAPH_AttachData(GraphHandle, GraphDataHandle);
//  for(int i = 0; i < 400; i++) GRAPH_DATA_YT_AddValue(GraphDataHandle, i);
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  STMPE610_read_xyz();
	  GUI_TOUCH_Exec();
	  keypressed = GUI_GetKey();

		  GUI_Delay(1);

		  X_koordinata = STMPE610_GetX_TOUCH();
		  Y_koordinata = STMPE610_GetY_TOUCH();
		  if(X_koordinata > 0 && X_koordinata < 80 && Y_koordinata > 0 && Y_koordinata < 80){

			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
		  }

      if(FALSE != NewMeasurementEvent)
      {
          sprintf(msg, "Mes count: %d\nMes result: %d", MeasurementMsg.Count, MeasurementMsg.Result);
          PrintToDisplay(msg);
          NewMeasurementEvent = FALSE;
          if(MeasurementMsg.Count == 10)
          {
            C02Click_StopMeasurement();
            sprintf(msg, "Measurement stopped after %d measurements", MeasurementMsg.Count);
            PrintToDisplay(msg);
          }
      }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
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
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
