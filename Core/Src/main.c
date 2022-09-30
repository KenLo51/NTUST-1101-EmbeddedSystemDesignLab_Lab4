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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_ts.h"
#include "stm32f7xx_hal.h"

#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"

#include "lvgl.h"
#include "tft.h"
#include "touchpad.h"

//#include "../lvgl_examples/lv_examples.h"
//#include "RGB_s.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	lv_event_t* e;
	uint16_t btnIndex;
}mineBtn_cb_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOARD_WIDTH 15
#define BOARD_HEIGHT 8

#define BOARD_MINE_SYMBOL 128
#define BOARD_MINE_UNKNOW 64

#define GAME_STATE_WAITFORSTART 0
#define GAME_STATE_PLAYING 1
#define GAME_STATE_DONE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
//fs variables
DIR imgDir;

//gui variables
static lv_obj_t * btnm = NULL;//keyboard
static lv_obj_t * ta = NULL;//text area
static lv_obj_t * label_ntust = NULL;
static lv_obj_t * label_minesNum = NULL;
static lv_obj_t * label_timeCnt = NULL;
static lv_obj_t * label_result = NULL;

static lv_obj_t * label_gameBoard;
static lv_obj_t * btn_mines[BOARD_HEIGHT][BOARD_WIDTH];

static uint16_t minesNum;
static uint8_t boardWidth, boardHeight;
static uint8_t board[BOARD_HEIGHT][BOARD_WIDTH];//
static uint8_t gameState=0;
static uint16_t totalTime;
static uint8_t userBtnFlag=0;


//debug output variables
char testStr[80];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void arrayShuffle(void* array, size_t itemSize, uint16_t len);

static void kb_event_cb(lv_event_t* e);
static void ta_event_cb(lv_event_t *e);
static void pict_event_cb(lv_event_t *e);
static void kb_create(void);//create keyboard 
static void ta_create(void);//create textarea 
static void lb_create(void);//create "NTUST" label 
static void createConfigFrame(void);
static void showConfigFrame(void);
static void delConfigFrame(void);

static void showPlayFrame(void);
static void delPlayFrame(void);

static void mineBtn_cd(lv_event_t *e);
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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_SDMMC1_SD_Init();
  MX_DMA_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);
	BSP_LCD_SelectLayer(1);
	BSP_LCD_DisplayOn();
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	
	
	/*if(f_open(&SDFile, "0:/logo_1.bmp", FA_READ) == FR_OK){
		BSP_LCD_DisplayStringAtLine(1, "read 0:/logo_1.bmp success");
	}else{
		BSP_LCD_DisplayStringAtLine(1, "read 0:/logo_1.bmp fail");
	}
	HAL_Delay(500);
	f_close(&SDFile);
	uint16_t* testStr_u16 = (uint16_t*)(0XC0100000);
	*testStr_u16 = 123; 
	sprintf(testStr, "write %d to %x", 123, testStr_u16);
	BSP_LCD_DisplayStringAtLine(0, (uint8_t*)testStr);
	sprintf(testStr, "read %d from %x", *testStr_u16, testStr_u16);
	BSP_LCD_DisplayStringAtLine(1, (uint8_t*)testStr);
	HAL_Delay(1000);*/
	
	//BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"test");
	//HAL_Delay(1000);
	
	//lvgl init
	//SCB_EnableICache();
	//SCB_EnableDCache();
	
	lv_init();
  tft_init();
	touchpad_init();
	//fs_init();
	
	//lv_example_colorwheel_1();
	//lv_example_img_1();
	createConfigFrame();
	
	
	HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//lv_task_handler();
		//HAL_Delay(5);
		lv_task_handler();
		if(userBtnFlag){
			userBtnFlag=0;
			delPlayFrame();
			createConfigFrame();
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 2;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  htim6.Init.Prescaler = 10799;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 24;
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

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin|GPIO_PIN_1|LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin|ARDUINO_SDA_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D6_Pin DCMI_D7_Pin */
  GPIO_InitStruct.Pin = DCMI_D6_Pin|DCMI_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin OTG_FS_ID_Pin */
  GPIO_InitStruct.Pin = OTG_FS_P_Pin|OTG_FS_N_Pin|OTG_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
  GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SDB_Pin */
  GPIO_InitStruct.Pin = SAI2_SDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_D5_Pin */
  GPIO_InitStruct.Pin = DCMI_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin PI1 LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|GPIO_PIN_1|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_VSYNC_Pin */
  GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_PWR_EN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D4_Pin DCMI_D3_Pin DCMI_D0_Pin DCMI_D2_Pin
                           DCMI_D1_Pin */
  GPIO_InitStruct.Pin = DCMI_D4_Pin|DCMI_D3_Pin|DCMI_D0_Pin|DCMI_D2_Pin
                          |DCMI_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_CS_D5_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(ARDUINO_PWM_CS_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PI11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D10_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(ARDUINO_PWM_D10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_RX_D0_Pin ARDUINO_TX_D1_Pin */
  GPIO_InitStruct.Pin = ARDUINO_RX_D0_Pin|ARDUINO_TX_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_A4_Pin ARDUINO_A5_Pin ARDUINO_A1_Pin ARDUINO_A2_Pin
                           ARDUINO_A3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A4_Pin|ARDUINO_A5_Pin|ARDUINO_A1_Pin|ARDUINO_A2_Pin
                          |ARDUINO_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_A0_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARDUINO_A0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_HSYNC_Pin PA6 */
  GPIO_InitStruct.Pin = DCMI_HSYNC_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_SCL_Pin LCD_SDA_Pin */
  GPIO_InitStruct.Pin = LCD_SCL_Pin|LCD_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D6_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(ARDUINO_PWM_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
  GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//user functions define
void swap(void* a, void* b, size_t itemSize){
	uint8_t *aa = (uint8_t*)a;
	uint8_t *bb = (uint8_t*)b;
	uint8_t temp;
	for(uint16_t i=0; i<itemSize; i++){
		temp = *aa;
		*aa = *bb;
		*bb = temp;
		
		aa++;
		bb++;
	}
}
void arrayShuffle(void* array, size_t itemSize, uint16_t len){
	uint8_t *arr = (uint8_t *)array;
	for(uint16_t i=0; i<len; i++){
		uint16_t exchangeIdx = rand()%len;
		swap(arr+i*itemSize, arr+(itemSize*exchangeIdx), itemSize);
	}
}



static void kb_event_cb(lv_event_t* e){
		lv_obj_t * obj = lv_event_get_target(e);
    lv_obj_t * ta = lv_event_get_user_data(e);
	
		const char * txt = lv_btnmatrix_get_btn_text(obj, lv_btnmatrix_get_selected_btn(obj));
		
		if(strcmp(txt, "C") == 0) {
			lv_textarea_set_text(ta, "");
			return;
		}
		if(strcmp(txt, "E") == 0) {
				const char* ch = lv_textarea_get_text(ta);
				minesNum = (uint16_t)atoi(ch);
			
				//check if num is currect
				if(minesNum<5 || minesNum>99) return;
				
				delConfigFrame();
				showPlayFrame();
				return;
    }
		
		lv_textarea_add_text(ta, txt);
		return;
}

static void kb_create(void){
		if(btnm != NULL) return;
	
		static const char * btnm_map[] = {"1", "2", "3", "0", "\n",
																			"4", "5", "6", "C", "\n",
																			"7", "8", "9", "E", ""};

		btnm = lv_btnmatrix_create(lv_scr_act());
		lv_obj_set_size(btnm, 300, 170);
		lv_obj_align(btnm, LV_ALIGN_CENTER, 0, 10);
		lv_btnmatrix_set_map(btnm, btnm_map);
		lv_obj_clear_flag(btnm, LV_OBJ_FLAG_CLICK_FOCUSABLE);

		lv_obj_add_event_cb(btnm, kb_event_cb, LV_EVENT_PRESSED , ta);
}

static void ta_event_cb(lv_event_t *e){
		lv_event_code_t code = lv_event_get_code(e);
		lv_obj_t * kb = lv_event_get_user_data(e);
    if(code == LV_EVENT_CLICKED && kb == NULL) {
        kb_create();
    }
}

static void ta_create(void){
		if(ta != NULL) return;
	
		ta  = lv_textarea_create(lv_scr_act());
		lv_textarea_set_placeholder_text(ta, "Number of Mines");
		lv_obj_align(ta, LV_ALIGN_TOP_MID, 0, 10);
		lv_textarea_set_text(ta, "");
		lv_textarea_set_one_line(ta, true);
		lv_textarea_set_max_length(ta, 2);
		lv_obj_set_style_text_font(ta, &lv_font_montserrat_16 , 0);
}
static void lb_create(void){
		if(label_ntust != NULL) return;
	
		label_ntust = lv_label_create(lv_scr_act());
		lv_label_set_text(label_ntust, "Bomb Setting");
		lv_obj_align(label_ntust, LV_ALIGN_BOTTOM_MID, 0, -10);
		lv_obj_set_style_text_font(label_ntust, &lv_font_montserrat_16 , 0);
}
static void createConfigFrame(){
	
	lv_obj_set_scrollbar_mode(lv_scr_act(), LV_SCROLLBAR_MODE_OFF);//Never show the scrollbars
	lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);//non-scrollable
	
	ta_create();
	kb_create();
	lb_create();
}
static void showConfigFrame(void){
	lv_obj_clear_flag(ta, LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(btnm, LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(label_ntust, LV_OBJ_FLAG_HIDDEN);
}
static void delConfigFrame(void){
	lv_obj_del(ta);
	ta = NULL;
	lv_obj_del(btnm);
	btnm = NULL;
	lv_obj_del(label_ntust);
	label_ntust = NULL;
}
static uint8_t countAroundMines(uint16_t rowIdx, uint16_t colIdx){
	
	uint8_t num=0;
	int16_t scanRowIdx, scanColIdx;
	
	//left-top
	scanRowIdx = rowIdx-1;
	scanColIdx = colIdx-1;
	if((scanRowIdx>=0 && scanRowIdx<BOARD_HEIGHT)&&(scanColIdx>=0 && scanColIdx<BOARD_WIDTH))
		num += (board[scanRowIdx][scanColIdx]==BOARD_MINE_SYMBOL)? 1:0;
	//middle-top
	scanRowIdx = rowIdx-1;
	scanColIdx = colIdx;
	if((scanRowIdx>=0 && scanRowIdx<BOARD_HEIGHT)&&(scanColIdx>=0 && scanColIdx<BOARD_WIDTH))
		num += (board[scanRowIdx][scanColIdx]==BOARD_MINE_SYMBOL)? 1:0;
	//right-top
	scanRowIdx = rowIdx-1;
	scanColIdx = colIdx+1;
	if((scanRowIdx>=0 && scanRowIdx<BOARD_HEIGHT)&&(scanColIdx>=0 && scanColIdx<BOARD_WIDTH))
		num += (board[scanRowIdx][scanColIdx]==BOARD_MINE_SYMBOL)? 1:0;
	//left-middle
	scanRowIdx = rowIdx;
	scanColIdx = colIdx-1;
	if((scanRowIdx>=0 && scanRowIdx<BOARD_HEIGHT)&&(scanColIdx>=0 && scanColIdx<BOARD_WIDTH))
		num += (board[scanRowIdx][scanColIdx]==BOARD_MINE_SYMBOL)? 1:0;
	//right-middle
	scanRowIdx = rowIdx;
	scanColIdx = colIdx+1;
	if((scanRowIdx>=0 && scanRowIdx<BOARD_HEIGHT)&&(scanColIdx>=0 && scanColIdx<BOARD_WIDTH))
		num += (board[scanRowIdx][scanColIdx]==BOARD_MINE_SYMBOL)? 1:0;
	//left-bottom
	scanRowIdx = rowIdx+1;
	scanColIdx = colIdx-1;
	if((scanRowIdx>=0 && scanRowIdx<BOARD_HEIGHT)&&(scanColIdx>=0 && scanColIdx<BOARD_WIDTH))
		num += (board[scanRowIdx][scanColIdx]==BOARD_MINE_SYMBOL)? 1:0;
	//middle-bottom
	scanRowIdx = rowIdx+1;
	scanColIdx = colIdx;
	if((scanRowIdx>=0 && scanRowIdx<BOARD_HEIGHT)&&(scanColIdx>=0 && scanColIdx<BOARD_WIDTH))
		num += (board[scanRowIdx][scanColIdx]==BOARD_MINE_SYMBOL)? 1:0;
	//right-bottom
	scanRowIdx = rowIdx+1;
	scanColIdx = colIdx+1;
	if((scanRowIdx>=0 && scanRowIdx<BOARD_HEIGHT)&&(scanColIdx>=0 && scanColIdx<BOARD_WIDTH))
		num += (board[scanRowIdx][scanColIdx]==BOARD_MINE_SYMBOL)? 1:0;
	
	return num;
}
static void showMinesNum(lv_obj_t* obj, uint8_t num){
	//lv_palette_lighten(LV_PALETTE_GREY, 2)
	lv_obj_set_style_bg_color(obj, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
	if(num == 0) return;
	
	switch(num){
		case 1: lv_obj_set_style_text_color(obj, lv_palette_darken(LV_PALETTE_BLUE, 1), 0);
			break;
		case 2: lv_obj_set_style_text_color(obj, lv_palette_darken(LV_PALETTE_BLUE, 4), 0);
			break;
		case 3: lv_obj_set_style_text_color(obj, lv_palette_darken(LV_PALETTE_GREEN, 1), 0);
			break;
		case 4: lv_obj_set_style_text_color(obj, lv_palette_darken(LV_PALETTE_GREEN, 4), 0);
			break;
		case 5: lv_obj_set_style_text_color(obj, lv_palette_darken(LV_PALETTE_ORANGE, 1), 0);
			break;
		case 6: lv_obj_set_style_text_color(obj, lv_palette_darken(LV_PALETTE_ORANGE, 4), 0);
			break;
		case 7: lv_obj_set_style_text_color(obj, lv_palette_darken(LV_PALETTE_CYAN, 1), 0);
			break;
		case 8: lv_obj_set_style_text_color(obj, lv_palette_darken(LV_PALETTE_CYAN, 4), 0);
			break;
		default:
			break;
	}
	
	char ch[2] = {" "};
	lv_obj_t * label = lv_label_create(obj);
	ch[0] = num + '0';
	lv_label_set_text(label, ch);
	lv_obj_center(label);
}
static void showMines(){
	for(int16_t rowIdx=0; rowIdx<BOARD_HEIGHT; rowIdx++){
		for(int16_t colIdx=0; colIdx<BOARD_WIDTH; colIdx++){
			if(board[rowIdx][colIdx] == BOARD_MINE_SYMBOL){
				lv_obj_t * label = lv_label_create(btn_mines[rowIdx][colIdx]);
				lv_obj_set_style_text_font(label, &lv_font_montserrat_16 , 0);
				lv_label_set_text(label, LV_SYMBOL_SETTINGS);
				lv_obj_set_style_text_color(label, lv_palette_darken(LV_PALETTE_GREY, 4), 0);
				lv_obj_center(label);
			}
		}
	}
}
static void mineBtn_cd(lv_event_t *e){
	lv_obj_t * pressedBtn = lv_event_get_target(e);
	uint32_t btnIdx = (uint32_t)e->user_data;
	
	uint16_t rowIdx = btnIdx/BOARD_WIDTH;
	uint16_t colIdx = btnIdx%BOARD_WIDTH;
	
	if(gameState == GAME_STATE_DONE) return;
	
	if(gameState == GAME_STATE_WAITFORSTART){//first pressed
		if(board[rowIdx][colIdx] == BOARD_MINE_SYMBOL){
			uint16_t exchangeRowIdx, exchangeColIdx;
			for(exchangeRowIdx=0; exchangeRowIdx<BOARD_HEIGHT; exchangeRowIdx++)
				for(exchangeColIdx=0; exchangeColIdx<BOARD_WIDTH; exchangeColIdx++)
					if(board[exchangeRowIdx][exchangeColIdx] != BOARD_MINE_SYMBOL) break;
			
			board[exchangeRowIdx][exchangeColIdx] = BOARD_MINE_SYMBOL;
			board[rowIdx][colIdx] = 0;
		}
		
		gameState = GAME_STATE_PLAYING;
	}
	
	if(board[rowIdx][colIdx] == BOARD_MINE_SYMBOL){
		lv_obj_set_style_bg_color(pressedBtn, lv_palette_main(LV_PALETTE_RED), 0);
		showMines();
		lv_label_set_text(label_result, "Fail");
		gameState = GAME_STATE_DONE;
	}
	else{
		uint8_t minesNum = countAroundMines(rowIdx, colIdx);
		board[rowIdx][colIdx] = minesNum;
		showMinesNum(btn_mines[rowIdx][colIdx], minesNum);
	}
	
	int16_t scanRowIdx, scanColIdx;
	uint8_t hasUpdate = 1;
	while(hasUpdate){
		hasUpdate = 0;
		for(int16_t rowIdx=0; rowIdx<BOARD_HEIGHT; rowIdx++){
			for(int16_t colIdx=0; colIdx<BOARD_WIDTH; colIdx++){
				
				for(scanRowIdx=rowIdx-1; scanRowIdx<=rowIdx+1; scanRowIdx++){
					for(scanColIdx=colIdx-1; scanColIdx<=colIdx+1; scanColIdx++){
						if((scanRowIdx>=0 && scanRowIdx<BOARD_HEIGHT)&&(scanColIdx>=0 && scanColIdx<BOARD_WIDTH)){
							if(board[rowIdx][colIdx] != BOARD_MINE_SYMBOL && board[rowIdx][colIdx] != BOARD_MINE_UNKNOW) continue;
							if(board[scanRowIdx][scanColIdx] == 0){
								uint8_t minesNum = countAroundMines(rowIdx, colIdx);
								board[rowIdx][colIdx] = minesNum;
								showMinesNum(btn_mines[rowIdx][colIdx], minesNum);
								hasUpdate |= 1;
							}
						}
					}
				}
				
				
			}
		}
	}
	
	uint16_t unknowNum = BOARD_WIDTH*BOARD_HEIGHT;
	for(int16_t rowIdx=0; rowIdx<BOARD_HEIGHT; rowIdx++){
		for(int16_t colIdx=0; colIdx<BOARD_WIDTH; colIdx++){
			if(board[rowIdx][colIdx] == BOARD_MINE_SYMBOL) continue;
			if(board[rowIdx][colIdx] != BOARD_MINE_UNKNOW) unknowNum--;
		}
	}
	
	if(unknowNum == minesNum){
		lv_label_set_text(label_result, "Win");
		gameState = GAME_STATE_DONE;
	}
	/*
	if(board[rowIdx][colIdx] >0 && board[rowIdx][colIdx]!=BOARD_MINE_SYMBOL){
		char ch[2] = {" "};
		lv_obj_t * label = lv_label_create(btn_mines[rowIdx][colIdx]);
		ch[0] = board[rowIdx][colIdx] + '0';
		lv_label_set_text(label, ch);
		lv_obj_center(label);
	}
	
	lv_obj_set_style_bg_color(pressedBtn, lv_palette_main(LV_PALETTE_BLUE), 0);
	*/
	
}
static void showPlayFrame(void){
	//create score label
	if(label_minesNum == NULL)
		label_minesNum = lv_label_create(lv_scr_act());
	char minesNumStr[4];
	sprintf(minesNumStr, "%2d",minesNum);
	lv_label_set_text(label_minesNum, minesNumStr);
	lv_obj_align(label_minesNum, LV_ALIGN_TOP_LEFT, 0, 0);
	lv_obj_set_style_text_font(label_minesNum, &lv_font_montserrat_16 , 0);
	
	//create result label
	if(label_result == NULL)
		label_result = lv_label_create(lv_scr_act());
	lv_label_set_text(label_result, "");
	lv_obj_align(label_result, LV_ALIGN_TOP_MID, 0, 0);
	lv_obj_set_style_text_font(label_result, &lv_font_montserrat_16 , 0);

	//create time's counter label
	if(label_timeCnt == NULL)
		label_timeCnt = lv_label_create(lv_scr_act());
	lv_label_set_text(label_timeCnt, "00:00");
	lv_obj_align(label_timeCnt, LV_ALIGN_TOP_RIGHT, 0, 0);
	lv_obj_set_style_text_font(label_timeCnt, &lv_font_montserrat_16 , 0);
	
	//create a frame contain buttons
	label_gameBoard = lv_label_create(lv_scr_act());
	
	lv_label_set_text(label_gameBoard, "");
	lv_obj_align(label_gameBoard, LV_ALIGN_CENTER, 0, 10);
	
	//create buttons
	const uint16_t btnSize = 30;
	for(int rowIdx=0; rowIdx<BOARD_HEIGHT; rowIdx++){
		for(int colIdx=0; colIdx<BOARD_WIDTH; colIdx++){
			lv_obj_t * newBtn;
			newBtn = lv_btn_create(label_gameBoard);
			lv_obj_set_size(newBtn, btnSize-1, btnSize-1);
			lv_obj_align(newBtn, LV_ALIGN_TOP_LEFT, btnSize*colIdx, btnSize*rowIdx);
			lv_obj_set_style_bg_color(newBtn, lv_palette_main(LV_PALETTE_GREY), 0);
			lv_obj_set_style_radius(newBtn, 1, 0);
			
			lv_obj_add_event_cb(newBtn, mineBtn_cd, LV_EVENT_RELEASED, (void*)(rowIdx*BOARD_WIDTH + colIdx));
			
			btn_mines[rowIdx][colIdx] = newBtn;
			
			//sprintf(testStr, "%7x", newBtn);
			//BSP_LCD_DisplayStringAtLine(0,testStr);
		}
	}
	
	//initial minesweeper data
	srand(HAL_GetTick());
	uint8_t index[BOARD_HEIGHT*BOARD_WIDTH];
	for(uint16_t idx=0; idx<BOARD_HEIGHT*BOARD_WIDTH; idx++)
		index[idx] = idx;
	arrayShuffle(index, sizeof(index[0]), sizeof(index)/sizeof(index[0]));
	
	//count mines number
	for(uint16_t rowIdx=0; rowIdx<BOARD_HEIGHT; rowIdx++){
		for(uint16_t colIdx=0; colIdx<BOARD_WIDTH; colIdx++){
			board[rowIdx][colIdx] = BOARD_MINE_UNKNOW;
		}
	}
	
	//
	for(uint16_t i=0; i<minesNum; i++){
		uint16_t rowIdx = index[i]/BOARD_WIDTH;
		uint16_t colIdx = index[i]%BOARD_WIDTH;
		board[rowIdx][colIdx] = BOARD_MINE_SYMBOL;
	}
	
	
	/*char ch[2] = {" "};
	for(uint16_t rowIdx=0; rowIdx<BOARD_HEIGHT; rowIdx++){
		for(uint16_t colIdx=0; colIdx<BOARD_WIDTH; colIdx++){
			if(board[rowIdx][colIdx] == BOARD_MINE_SYMBOL){
				ch[0] = 'X';
			}
			else{
				uint8_t minesNum = countAroundMines(rowIdx, colIdx);
				ch[0] = minesNum + '0';
			}
			lv_obj_t * label = lv_label_create(btn_mines[rowIdx][colIdx]);
			lv_label_set_text(label, ch);
			lv_obj_center(label);
		}
	}*/
	
	gameState = GAME_STATE_WAITFORSTART;
	totalTime = 0;
}
static void delPlayFrame(void){
	gameState = 0;
	totalTime = 0;
	
	if(label_minesNum != NULL)
		lv_obj_del(label_minesNum);
	label_minesNum = NULL;
	
	if(label_timeCnt != NULL)
		lv_obj_del(label_timeCnt);
	label_timeCnt = NULL;

	if(label_result != NULL)
		lv_obj_del(label_result);
	label_result = NULL;
	
	//lv_obj_clean(label_gameBoard);
	if(label_gameBoard != NULL)
		lv_obj_del(label_gameBoard);
	label_gameBoard = NULL;
}

//interrupt callback functions
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t cnt_400hz;
	static uint8_t cnt_20hz;
	static uint8_t cnt_1hz;
	static uint8_t userBtnPrvState;
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim6 )
  {
    cnt_400hz++;
		if(cnt_400hz>=20){//20HZ
			cnt_400hz = 0;

			
			if(gameState == GAME_STATE_PLAYING){
				if(cnt_20hz >=19){//1Hz
					cnt_20hz = 0;
					cnt_1hz++;
					
					//update time
					if(totalTime < (99*60 + 59)) totalTime = totalTime + 1;
					char timeStr[10];
					sprintf(timeStr,"%02d:%02d", totalTime/60, totalTime%60);
					lv_label_set_text(label_timeCnt, timeStr);
					
				}
				else{
					cnt_20hz = cnt_20hz + 1;
				}
			}
			
			
			//GPIO
			if(HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_11) == GPIO_PIN_SET)
				userBtnPrvState = (userBtnPrvState<<1)|1;
			else
				userBtnPrvState = (userBtnPrvState<<1)|0;
			if((userBtnPrvState & 0x03)==0x01){
				userBtnFlag = 1;
			}
			
		}
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
		BSP_LCD_DisplayStringAtLine(1, (uint8_t*)"Error");
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
