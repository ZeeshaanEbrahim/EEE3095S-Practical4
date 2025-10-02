/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS    128U    // Number of samples in LUT
#define TIM2CLK 16000000UL  // STM Clock frequency: Hint You might want to check the ioc file
#define F_SIGNAL  1000UL	// Frequency of output analog signal --- DAC UPDATE RATE 1MS/s - from Datasheet - MAX FREQUENCY = 7.8125HZ, 1KHZ safe conservative

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs
uint32_t Sin_LUT[NS]= {2047, 2147, 2248, 2347, 2446, 2545, 2641, 2737, 2831, 2922, 3012, 3100, 3185, 3267, 3346, 3422, 3495, 3564, 3630, 3692, 3749, 3803, 3853, 3898, 3939, 3975, 4006, 4033, 4055, 4072, 4085, 4092, 4095, 4092, 4085, 4072, 4055, 4033, 4006, 3975, 3939, 3898, 3853, 3803, 3749, 3692, 3630, 3564, 3495, 3422, 3346, 3267, 3185, 3100, 3012, 2922, 2831, 2737, 2641, 2545, 2446, 2347, 2248, 2147, 2047, 1947, 1846, 1747, 1648, 1549, 1453, 1357, 1263, 1172, 1082, 994, 909, 827, 748, 672, 599, 530, 464, 402, 345, 291, 241, 196, 155, 119, 88, 61, 39, 22, 9, 2, 0, 2, 9, 22, 39, 61, 88, 119, 155, 196, 241, 291, 345, 402, 464, 530, 599, 672, 748, 827, 909, 994, 1082, 1172, 1263, 1357, 1453, 1549, 1648, 1747, 1846, 1947};
uint32_t Saw_LUT[NS] = {0, 32, 64, 96, 128, 161, 193, 225, 257, 290, 322, 354, 386, 419, 451, 483, 515, 548, 580, 612, 644, 677, 709, 741, 773, 806, 838, 870, 902, 935, 967, 999, 1031, 1064, 1096, 1128, 1160, 1193, 1225, 1257, 1289, 1322, 1354, 1386, 1418, 1450, 1483, 1515, 1547, 1579, 1612, 1644, 1676, 1708, 1741, 1773, 1805, 1837, 1870, 1902, 1934, 1966, 1999, 2031, 2063, 2095, 2128, 2160, 2192, 2224, 2257, 2289, 2321, 2353, 2386, 2418, 2450, 2482, 2515, 2547, 2579, 2611, 2644, 2676, 2708, 2740, 2772, 2805, 2837, 2869, 2901, 2934, 2966, 2998, 3030, 3063, 3095, 3127, 3159, 3192, 3224, 3256, 3288, 3321, 3353, 3385, 3417, 3450, 3482, 3514, 3546, 3579, 3611, 3643, 3675, 3708, 3740, 3772, 3804, 3837, 3869, 3901, 3933, 3966, 3998, 4030, 4062, 4095};
uint32_t Trianlge_LUT = {0, 127, 255, 383, 511, 639, 767, 895, 1023, 1151, 1279, 1407, 1535, 1663, 1791, 1919, 2047, 2175, 2303, 2431, 2559, 2687, 2815, 2943, 3071, 3199, 3327, 3455, 3583, 3711, 3839, 3967, 4095, 4222, 4350, 4478, 4606, 4734, 4862, 4990, 5118, 5246, 5374, 5502, 5630, 5758, 5886, 6014, 6142, 6270, 6398, 6526, 6654, 6782, 6910, 7038, 7166, 7294, 7422, 7550, 7678, 7806, 7934, 8062, 8190, 8062, 7934, 7806, 7678, 7550, 7422, 7294, 7166, 7038, 6910, 6782, 6654, 6526, 6398, 6270, 6142, 6014, 5886, 5758, 5630, 5502, 5374, 5246, 5118, 4990, 4862, 4734, 4606, 4478, 4350, 4222, 4095, 3967, 3839, 3711, 3583, 3455, 3327, 3199, 3071, 2943, 2815, 2687, 2559, 2431, 2303, 2175, 2047, 1919, 1791, 1663, 1535, 1407, 1279, 1151, 1023, 895, 767, 639, 511, 383, 255, 127};

uint32_t Piano_LUT[NS] = {
   2047, 2183, 1984, 2098, 2061, 2088, 2038, 2158,
   2097, 2153, 2084, 2054, 2016, 2333, 2059, 1765,
   2027, 2045, 1970, 2055, 2063, 2051, 1583, 2022,
   2334, 2044, 2884, 1692, 1759, 1843, 2242, 2195,
   1531, 2163, 1969, 2061, 1987, 2088, 1733, 1952,
   1934, 2101, 2252, 1924, 2253, 1929, 2616, 2030,
   2097, 2063, 1985, 2063, 2024, 2037, 2818, 1944,
   2393, 1867, 2000, 1949, 1472, 1911, 1691, 2035,
   1778, 2070, 2143, 2047, 2015, 2021, 1718, 2153,
   1902, 2086, 1794, 2456, 2201, 2032, 2360, 2175,
   2027, 2043, 2004, 2072, 2034, 2041, 2187, 2053,
   2089, 1928, 1843, 2034, 2011, 2226, 2009, 2107,
   2347, 2052, 2077, 2038, 2071, 2023, 1754, 1922,
   1992, 1996, 2162, 2084, 2036, 1999, 1837, 2150,
   2005, 2079, 1997, 2072, 2031, 2060, 1942, 1911,
   1604, 1780, 2077, 2056, 2001, 1938, 2094, 2047,
};
uint16_t Guitar_LUT[128] = {
   2047, 2362, 2360, 2123, 2097, 1861, 1833, 2015,
   1534, 2092, 2080, 2052, 2087, 2075, 1893, 2036,
   2043, 2027, 1705, 2501, 1916, 1896, 2466, 2231,
   1775, 2052, 2015, 2062, 2036, 1994, 1964, 2054,
   2052, 2098, 1856, 2149, 1858, 2127, 2003, 1750,
   2249, 2024, 2056, 2052, 2064, 1933, 2144, 2041,
   1876, 2006, 2183, 2009, 2191, 2143, 1891, 2005,
   2146, 2139, 2095, 1827, 2163, 2097, 2761, 2230,
   2106, 2230, 2269, 2247, 2138, 2119, 1591, 1686,
   2065, 2040, 2079, 2063, 2268, 2044, 2032, 2056,
   2165, 2180, 1801, 2129, 2079, 1737, 2126, 1881,
   2087, 2101, 2046, 2033, 1959, 1889, 2115, 2045,
   2289, 1971, 2244, 1984, 2030, 2005, 2520, 2424,
   2011, 2052, 2034, 2041, 2117, 1936, 2045, 2051,
   1932, 2027, 2157, 1895, 2298, 1944, 2213, 1654,
   2105, 1854, 2386, 1905, 1640, 2618, 1933, 2048,
};
uint32_t Drum_LUT[NS] = {
		   2047, 3396, 1864, 2110, 2063, 3400, 1893, 1933,
		   2086, 2144, 2036, 2030, 2039, 1949, 1414, 1848,
		   2091, 2032, 2044, 2059, 2048, 2093, 2044, 2020,
		   2094, 1803, 2072, 2045, 2173, 2017, 3235, 1276,
		   1671, 2193, 2436, 2104, 1931, 2062, 2028, 2032,
		   4095, 1922, 2035, 2042, 2502, 2372, 1820, 1996,
		   2284, 2072, 2018, 2051, 671, 2425, 2075, 2068,
		   17, 2239, 2042, 2023, 2192, 2052, 2902, 2386,
		   4073, 2009, 2285, 2038, 2518, 2711, 2048, 2096,
		   1629, 2001, 2086, 2043, 2549, 1894, 2061, 2076,
		   2001, 2055, 2043, 2049, 2098, 2047, 2065, 2047,
		   2923, 2071, 2017, 2080, 2018, 2049, 1831, 2014,
		   837, 2295, 1978, 2040, 1980, 2036, 2106, 2042,
		   1643, 2055, 2066, 2032, 2050, 2442, 2145, 2218,
		   2126, 2032, 2038, 1781, 2154, 2441, 2132, 4095,
		   2150, 2012, 2036, 1615, 1981, 323, 2525, 2047,
		};





// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = 125; // How often to write new LUT value -- TIM2CLK/NS*FSIGNAL
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_IRQHandler(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1

  // TODO: Start DMA in IT mode on TIM2->CH1. Source is LUT and Dest is TIM3->CCR3; start with Sine LUT

  // TODO: Write current waveform to LCD(Sine is the first waveform)

  // TODO: Enable DMA (start transfer from LUT to CCR)

  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* TIM2_CH1 DMA Init */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;         // TIM2_CH1 is on channel 3
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory -> TIM3->CCR3
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;    // Peripheral address fixed
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;        // Memory address increments
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;            // Repeat LUT automatically
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Link DMA handle to TIM2 handle */
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // -------------------------------
  // LCD pins configuration
  // -------------------------------
  // Configure PC14 (RS) and PC15 (E) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure PB8 (D4) and PB9 (D5) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure PA12 (D6) and PA15 (D7) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Set all LCD pins LOW initially
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);


  // -------------------------------
  // Button0 configuration (PA0)
  // -------------------------------
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;         // Use pull-up resistor
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable and set EXTI line 0 interrupt priority
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void){

	// TODO: Debounce using HAL_GetTick()


	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes




	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
#ifdef USE_FULL_ASSERT
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
