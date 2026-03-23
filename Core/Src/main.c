/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_spi.h"

#include "reverb_dsp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_i2s3_ext_rx;
DMA_HandleTypeDef hdma_spi3_tx;

/* USER CODE BEGIN PV */
uint8_t current_algorithm = 0;
uint8_t shimmerOn = 0;
int32_t tone_history = 0;

volatile uint16_t adc_values[3];
uint32_t mix_val = 0, decay_val = 0, tone_val = 0;

#define AUDIO_BUF_SIZE 512
uint16_t rx_buf[AUDIO_BUF_SIZE];
uint16_t tx_buf[AUDIO_BUF_SIZE];

int16_t comb1_buf[1433];
int16_t comb2_buf[1601];
int16_t comb3_buf[1867];
int16_t comb4_buf[2053];

Delay_Filter comb1, comb2, comb3, comb4; //Comb filters

int16_t ap1_buf[227];
int16_t ap2_buf[347];

Delay_Filter ap1, ap2; //All Pass filters
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN 0 */


// Пришвидшимо згладжування для тесту (>> 5 замість >> 8), щоб ручки реагували миттєво
void Read_Pots_And_Smooth(void) {
    // 1. Створюємо "мертві зони" для ручок
    uint32_t raw_mix = adc_values[0];
    if (raw_mix < 50) raw_mix = 0;           // Глушимо мікроструми в нулі
    else if (raw_mix > 4040) raw_mix = 4095; // Гарантуємо 100% на максимумі

    uint32_t raw_decay = adc_values[1];
    if (raw_decay < 50) raw_decay = 0;
    else if (raw_decay > 4040) raw_decay = 4095;

    uint32_t raw_tone = adc_values[2];
    if (raw_tone < 50) raw_tone = 0;
    else if (raw_tone > 4040) raw_tone = 4095;

    // 2. Згладжуємо вже "очищені" значення
    mix_val   = ((mix_val   * 15) + raw_mix) >> 4;
    decay_val = ((decay_val * 15) + raw_decay) >> 4;
    tone_val  = ((tone_val  * 15) + raw_tone) >> 4;

    // 3. Масштабуємо Decay у Feedback
    // 30000 - максимальний фідбек. Якщо decay_val = 0, фідбек буде гарантовано 0.
    int32_t current_feedback = (decay_val * 30000) >> 12;

    comb1.feedback = current_feedback;
    comb2.feedback = current_feedback;
    comb3.feedback = current_feedback;
    comb4.feedback = current_feedback;
}

// Only left channel is processed and copied to both channels
void Process_Audio_Block(uint32_t start_idx, uint32_t end_idx) {
    int32_t l_mix = (int32_t)mix_val;
    int32_t l_tone = (int32_t)tone_val;

    for (uint32_t i = start_idx; i < end_idx; i += 4) {
        
        // ПОВЕРТАЄМОСЯ ДО ТВОЇХ ПРАВИЛЬНИХ ІНДЕКСІВ:
        // i   = Лівий MSB (Основний звук)
        // i+1 = Лівий LSB (Втрачені мікродеталі та верха)
        
        int16_t  dry_L_MSB = (int16_t)rx_buf[i];     // Твій старий-добрий робочий індекс!
        uint16_t dry_L_LSB = rx_buf[i+1];            

        // 1. Вхід для ревербератора (гучний)
        int16_t input_att = dry_L_MSB >> 1; 

        // 2. Паралельні фільтри
        int32_t reverb_sum = 0;
        reverb_sum += Comb_Process(&comb1, input_att);
        reverb_sum += Comb_Process(&comb2, input_att);
        reverb_sum += Comb_Process(&comb3, input_att);
        reverb_sum += Comb_Process(&comb4, input_att);

        // 3. Дифузія (гучна)
        int16_t wet = (int16_t)(reverb_sum >> 1); 
        wet = AllPass_Process(&ap1, wet);
        wet = AllPass_Process(&ap2, wet);

        // 4. Тон-фільтр
        tone_history = ((wet * l_tone) + (tone_history * (4096 - l_tone)) + 2048) >> 12;
        wet = (int16_t)tone_history;

        // 5. ДОДАЄМО ЕФЕКТ
        int32_t out_L = dry_L_MSB + ((wet * l_mix) >> 12);
        
        // Лімітер
        if (out_L > 32767) out_L = 32767; else if (out_L < -32768) out_L = -32768;

        // 6. ІДЕАЛЬНИЙ ВИХІД НА ЦАП
        tx_buf[i]   = (uint16_t)out_L;       // Оброблений основний звук
        tx_buf[i+1] = dry_L_LSB;             // Повертаємо мікродеталі замість нулів!
        tx_buf[i+2] = (uint16_t)out_L;       // Праве вухо
        tx_buf[i+3] = dry_L_LSB;             
    }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S3_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  // Запуск АЦП один раз у фоні
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 3);
  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn); // Вимикаємо переривання АЦП

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);

  LL_I2S_Disable(SPI3);
  LL_I2S_Disable(I2S3ext);

  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, AUDIO_BUF_SIZE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_0, (uint32_t)&I2S3ext->DR, (uint32_t)rx_buf, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, AUDIO_BUF_SIZE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_5, (uint32_t)tx_buf, (uint32_t)&SPI3->DR, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_I2S_EnableDMAReq_RX(I2S3ext);
  LL_I2S_EnableDMAReq_TX(SPI3);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
  LL_I2S_Enable(I2S3ext);
  LL_I2S_Enable(SPI3);

  Delay_Filter_Init(&comb1, comb1_buf, 1433, 28000);
  Delay_Filter_Init(&comb2, comb2_buf, 1601, 28000);
  Delay_Filter_Init(&comb3, comb3_buf, 1867, 28000);
  Delay_Filter_Init(&comb4, comb4_buf, 2053, 28000);
  Delay_Filter_Init(&ap1, ap1_buf, 227, 22937);
  Delay_Filter_Init(&ap2, ap2_buf, 347, 22937);
  /* USER CODE END 2 */


  while (1)
{
    if (LL_DMA_IsActiveFlag_HT0(DMA1))
    {
        LL_DMA_ClearFlag_HT0(DMA1); 
        
        // Просто читаємо. Ніяких "if READY", ніяких "Start_DMA".
        Read_Pots_And_Smooth(); 
        Process_Audio_Block(0, AUDIO_BUF_SIZE / 2);
    }

    if (LL_DMA_IsActiveFlag_TC0(DMA1))
    {
        LL_DMA_ClearFlag_TC0(DMA1);    
        Read_Pots_And_Smooth();
        Process_Audio_Block(AUDIO_BUF_SIZE / 2, AUDIO_BUF_SIZE);
    }
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_3)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSE_EnableCSS();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 200, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(100000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSI, LL_RCC_MCO1_DIV_1);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLI2SM_DIV_25, 258, LL_RCC_PLLI2SR_DIV_3);
  LL_RCC_PLLI2S_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLLI2S_IsReady() != 1)
  {

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);

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