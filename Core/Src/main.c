/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_spi.h"
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
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_i2s3_ext_rx;
DMA_HandleTypeDef hdma_spi3_tx;

/* USER CODE BEGIN PV */
// Буфери для DMA (Подвійна буферизація / Ping-Pong)
#define AUDIO_BUF_SIZE 512
uint16_t rx_buf[AUDIO_BUF_SIZE];
uint16_t tx_buf[AUDIO_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);

// --- ЗМІННІ ДЛЯ ДІЛЕЮ ---
#define DELAY_SIZE 26500 // 24000 семплів = 0.5 секунди затримки при 48 кГц
int16_t delay_line[DELAY_SIZE];
uint32_t delay_idx = 0;
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  /* Обов'язково вмикаємо тактування SPI3 для роботи LL функцій */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

  /* Вимикаємо переривання DMA в NVIC, щоб HAL не скидав прапорці HT/TC замість нас */
  HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);

  /* --- Знімаємо живлення з I2S перед налаштуванням --- */
  LL_I2S_Disable(SPI3);
  LL_I2S_Disable(I2S3ext);

  /* --- Налаштування DMA для прийому (RX - Stream 0) --- */
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, AUDIO_BUF_SIZE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_0, 
                         (uint32_t)&I2S3ext->DR, 
                         (uint32_t)rx_buf, 
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* --- Налаштування DMA для передачі (TX - Stream 5) --- */
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, AUDIO_BUF_SIZE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_5, 
                         (uint32_t)tx_buf, 
                         (uint32_t)&SPI3->DR, 
                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  /* Дозволяємо I2S генерувати запити до DMA */
  LL_I2S_EnableDMAReq_RX(I2S3ext);
  LL_I2S_EnableDMAReq_TX(SPI3);

  /* Вмикаємо потоки DMA */
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);

  /* Вмикаємо I2S (ПОРЯДОК ВАЖЛИВИЙ: Спочатку приймач, потім майстер-передавач) */
  LL_I2S_Enable(I2S3ext);
  LL_I2S_Enable(SPI3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Блималка (Heartbeat)
    static uint32_t heartbeat = 0;
    if (++heartbeat >= 500000) {  
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
        heartbeat = 0;
    }

    // --- ОБРОБКА ПЕРШОЇ ПОЛОВИНИ БУФЕРА ---
    if (LL_DMA_IsActiveFlag_HT0(DMA1))
    {
        LL_DMA_ClearFlag_HT0(DMA1); 
        
        // Крокуємо по 4 слова (Лівий MSB, Лівий LSB, Правий MSB, Правий LSB)
        for (uint32_t i = 0; i < (AUDIO_BUF_SIZE / 2); i += 4) {
            
            // 1. Читаємо вхідний сигнал (беремо лише старші 16 біт) і переводимо у знакове число
            int16_t dry_sample = (int16_t)rx_buf[i];

            // 2. Читаємо семпл з лінії затримки (те, що ми зіграли пів секунди тому)
            int16_t wet_sample = delay_line[delay_idx];

            // 3. Мікшуємо вихідний сигнал: 50% чистого звуку + 50% відлуння
            // (Ділимо на 2 за допомогою побітового зсуву або звичайного ділення, щоб не було перевантаження/хрипу)
            int16_t out_sample = (dry_sample / 2) + (wet_sample / 2);

            // 4. Фідбек (Feedback): записуємо в буфер ділею свіжий звук + трохи старого, щоб він затухав
            delay_line[delay_idx] = dry_sample + (wet_sample / 2); 

            // 5. Рухаємо "головку запису/читання" по колу
            delay_idx++;
            if (delay_idx >= DELAY_SIZE) {
                delay_idx = 0;
            }

            // 6. Записуємо результат в буфер передачі на ЦАП (робимо стерео з моно)
            tx_buf[i]   = (uint16_t)out_sample; // Лівий канал MSB
            tx_buf[i+1] = 0;                    // Лівий канал LSB (просто зануляємо молодші біти)
            
            tx_buf[i+2] = (uint16_t)out_sample; // Правий канал MSB
            tx_buf[i+3] = 0;                    // Правий канал LSB
        }
    }

    // --- ОБРОБКА ДРУГОЇ ПОЛОВИНИ БУФЕРА ---
    if (LL_DMA_IsActiveFlag_TC0(DMA1))
    {
        LL_DMA_ClearFlag_TC0(DMA1); 
        
        for (uint32_t i = (AUDIO_BUF_SIZE / 2); i < AUDIO_BUF_SIZE; i += 4) {
            
            // Робимо абсолютно те саме для другої половини масиву
            int16_t dry_sample = (int16_t)rx_buf[i];
            int16_t wet_sample = delay_line[delay_idx];

            int16_t out_sample = (dry_sample / 2) + (wet_sample / 2);
            delay_line[delay_idx] = dry_sample + (wet_sample / 2); 

            delay_idx++;
            if (delay_idx >= DELAY_SIZE) {
                delay_idx = 0;
            }

            tx_buf[i]   = (uint16_t)out_sample;
            tx_buf[i+1] = 0; 
            tx_buf[i+2] = (uint16_t)out_sample;
            tx_buf[i+3] = 0; 
        }
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

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
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
  
  /* Ініціалізація світлодіода на PC13, яку стер CubeMX */
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
  * where the assert_param error has occurred.
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