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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"  // CMSIS-DSP
#include <math.h>
#include "stdlib.h"
#include "stdio.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SAMPLES 128  // S? di?m FFT
#define SAMPLING_RATE 16000  // T?n s? l?y m?u ADC
#define MCP4725_ADDRESS 0xC0
#define LCD_ADRESS 0x27<<1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile uint16_t adcBuf[SAMPLES];


//uint32_t *inBufptr = &adcBuf[0];
float32_t *input_fft = NULL;      
//float32_t *fft_output = NULL;
float32_t fft_output[SAMPLES];
float32_t input_ifft[SAMPLES];  
extern const float32_t coeff[3][3][3][129];

int taps =0;
float32_t dac_out[SAMPLES];
uint8_t *dac_dma_buff =NULL;
//uint8_t dac_dma_buff[SAMPLES*2];
uint16_t bass = 1,mid=1,tremble=1;
//float32_t real_output[SAMPLES];   

arm_rfft_fast_instance_f32 fft_instance;

char bassgain[2];char midgain[2];char tremgain[2];
int process_ready = 0;
const float32_t hamming[128] =
{0.08000,0.08056,0.08225,0.08506,0.08898,0.09400,0.10012,0.10731,0.11556,0.12485,0.13516,0.14645,0.15871,0.17191,0.18600,0.20096,0.21675,0.23333,0.25066,0.26870,0.28740,0.30672,0.32661,0.34703,0.36792,0.38922,0.41090,0.43289,0.45515,0.47761,0.50022,0.52294,0.54569,0.56843,0.59110,0.61365,0.63601,0.65814,0.67998,0.70148,0.72258,0.74324,0.76340,0.78301,0.80203,0.82041,0.83810,0.85506,0.87125,0.88663,0.90116,0.91480,0.92753,0.93932,0.95012,0.95992,0.96869,0.97642,0.98308,0.98865,0.99312,0.99649,0.99873,0.99986,0.99986,0.99873,0.99649,0.99312,0.98865,0.98308,0.97642,0.96869,0.95992,0.95012,0.93932,0.92753,0.91480,0.90116,0.88663,0.87125,0.85506,0.83810,0.82041,0.80203,0.78301,0.76340,0.74324,0.72258,0.70148,0.67998,0.65814,0.63601,0.61365,0.59110,0.56843,0.54569,0.52294,0.50022,0.47761,0.45515,0.43289,0.41090,0.38922,0.36792,0.34703,0.32661,0.30672,0.28740,0.26870,0.25066,0.23333,0.21675,0.20096,0.18600,0.17191,0.15871,0.14645,0.13516,0.12485,0.11556,0.10731,0.10012,0.09400,0.08898,0.08506,0.08225,0.08056,0.08000};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
ADC_ChannelConfTypeDef sConfig1 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Select_CH1(void){
	
	sConfig1.Channel = ADC_CHANNEL_1;
  sConfig1.Rank = 1;
  sConfig1.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig1) != HAL_OK)
  {
    Error_Handler();
  }
}
void Select_CH2(void){
	//  sConfig.Channel = ADC_CHANNEL_2;
	sConfig1.Channel = ADC_CHANNEL_2;
  sConfig1.Rank = 1;
	sConfig1.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig1) != HAL_OK)
  {
    Error_Handler();
  }
}
void Select_CH3(void){
	//  sConfig.Channel = ADC_CHANNEL_2;

	sConfig1.Channel = ADC_CHANNEL_3;
  sConfig1.Rank = 1;
	sConfig1.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig1) != HAL_OK)
  {
    Error_Handler();
  }
}

void get_gain(){
		Select_CH1();
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 100);
		if(HAL_ADC_GetValue(&hadc2)<1365){bass = 0;}
		else{
			if(HAL_ADC_GetValue(&hadc2)>2730){bass =2;}
			else bass=1;
		}
		sprintf(bassgain, "%d", bass);
		HAL_ADC_Stop(&hadc2);	
		lcd_goto_XY(1,6);
		lcd_send_string(bassgain);
   
		Select_CH2();
		HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 100);
				if(HAL_ADC_GetValue(&hadc2)<1365){mid = 0;}
		else{
			if(HAL_ADC_GetValue(&hadc2)>2730){mid =2;}
			else mid=1;
		}
		HAL_ADC_Stop(&hadc2);
		sprintf(midgain, "%d", mid);
		lcd_goto_XY(1,14);
		lcd_send_string(midgain);
   
		Select_CH3();
		HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 100);
		if(HAL_ADC_GetValue(&hadc2)<1365){tremble = 0;}
		else{
			if(HAL_ADC_GetValue(&hadc2)>2730){tremble =2;}
			else tremble=1;
		}
		HAL_ADC_Stop(&hadc2);
		sprintf(tremgain, "%d", tremble);
		lcd_goto_XY(2,7);
		lcd_send_string(tremgain);
		taps = (int)coeff[bass][mid][tremble][128];
}
void apply_equalizer() {
		//Chuyen gia tri tu buffer DMA vao mang + zeropadding
		input_fft = (float32_t *)malloc(SAMPLES * sizeof(float32_t));
	//fft_output = (float32_t *)malloc(SAMPLES * sizeof(float32_t));
		for (int i = 0; i<SAMPLES;i++){
			if((i>taps/2-1)&(i<SAMPLES-taps/2)){
				*(input_fft+i) = adcBuf[i-taps/2]*hamming[i];}
			else{*(input_fft+i)=0;}
		}
		arm_rfft_fast_f32(&fft_instance, input_fft, fft_output, 0);
		//free(input_fft);
		arm_cmplx_mult_cmplx_f32(fft_output,(float32_t *)coeff[bass][mid][tremble],input_ifft,SAMPLES/2);
		arm_rfft_fast_f32(&fft_instance, input_ifft, dac_out, 1);
		free(input_fft);
		dac_dma_buff = (uint8_t *)malloc(2*(SAMPLES-taps+1)*sizeof(uint8_t));
		for (int i=taps-1;i<SAMPLES;i++){
				dac_dma_buff[2*(i-taps+1)] = ((uint16_t)(*(dac_out+i))>>8) & 0x0F;
				dac_dma_buff[2*(i-taps+1)+1] = ((uint16_t)(*(dac_out+i))) & 0xFF;
		}
		HAL_I2C_Master_Transmit(&hi2c1,MCP4725_ADDRESS, dac_dma_buff, 2*(SAMPLES-taps+1),10);
		free(dac_dma_buff);
		process_ready = 0;

//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
//			HAL_Delay(1000);

}

//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef*adc1){
//	inBufptr = &adcBuf[0];
//	process_ready = 1;
//}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*adc1){
	//inBufptr = &adcBuf[SAMPLES];
	
	//apply_equalizer();
	process_ready = 1;

}


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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();

	lcd_goto_XY(1,1);
	lcd_send_string("BASS");
	lcd_goto_XY(1,10);
	lcd_send_string("MID");
	lcd_goto_XY(2,0);
	lcd_send_string("TREMB");
	arm_rfft_fast_init_f32(&fft_instance, SAMPLES);
		get_gain();
	HAL_ADC_Start_DMA (&hadc1,(uint32_t*)adcBuf,SAMPLES-taps+1);
	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
						get_gain();
		if (process_ready == 1){
			HAL_TIM_Base_Stop_IT(&htim3);
			apply_equalizer();
			HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcBuf,SAMPLES-taps+1);
			HAL_TIM_Base_Start_IT(&htim3);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4500-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
