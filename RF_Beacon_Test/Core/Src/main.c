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
#include "aprs_bits.h"
#include <gps.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SAMPLE_RATE       19200
#define BAUD              1200
#define SAMPLES_PER_BIT   (SAMPLE_RATE / BAUD)   /* = 16, exact integer */

#define MAX_BITS          1000
#define AUDIO_BUFFER_SIZE 512

#define DAC_MID           2048
#define DAC_AMPLITUDE     1800

/*
 * 32-bit phase accumulator increments.
 * Formula: inc = freq / Fs * 2^32
 *
 * MARK  (1200 Hz): 1200 / 19200 * 2^32 = 268435456
 * SPACE (2200 Hz): 2200 / 19200 * 2^32 = 491520000
 *
 * Verify: 268435456 / 2^32 * 19200 = 1200.000 Hz  ✓
 *         491520000 / 2^32 * 19200 = 2200.000 Hz  ✓
 */
#define PHASE_INC_MARK    268435456UL
#define PHASE_INC_SPACE   491520000UL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

static const uint16_t SINE_LUT[256] = {
    2048, 2092, 2136, 2180, 2224, 2268, 2312, 2356,
    2399, 2442, 2485, 2528, 2571, 2613, 2654, 2696,
    2737, 2777, 2818, 2857, 2897, 2935, 2973, 3011,
    3048, 3084, 3120, 3155, 3190, 3224, 3257, 3289,
    3321, 3352, 3382, 3411, 3439, 3467, 3494, 3520,
    3545, 3569, 3592, 3614, 3635, 3656, 3675, 3694,
    3711, 3727, 3743, 3757, 3770, 3783, 3794, 3804,
    3813, 3821, 3829, 3834, 3839, 3843, 3846, 3847,
    3848, 3847, 3846, 3843, 3839, 3834, 3829, 3821,
    3813, 3804, 3794, 3783, 3770, 3757, 3743, 3727,
    3711, 3694, 3675, 3656, 3635, 3614, 3592, 3569,
    3545, 3520, 3494, 3467, 3439, 3411, 3382, 3352,
    3321, 3289, 3257, 3224, 3190, 3155, 3120, 3084,
    3048, 3011, 2973, 2935, 2897, 2857, 2818, 2777,
    2737, 2696, 2654, 2613, 2571, 2528, 2485, 2442,
    2399, 2356, 2312, 2268, 2224, 2180, 2136, 2092,
    2048, 2004, 1960, 1916, 1872, 1828, 1784, 1740,
    1697, 1654, 1611, 1568, 1525, 1483, 1442, 1400,
    1359, 1319, 1278, 1239, 1199, 1161, 1123, 1085,
    1048, 1012,  976,  941,  906,  872,  839,  807,
     775,  744,  714,  685,  657,  629,  602,  576,
     551,  527,  504,  482,  461,  440,  421,  402,
     385,  369,  353,  339,  326,  313,  302,  292,
     283,  275,  267,  262,  257,  253,  250,  249,
     248,  249,  250,  253,  257,  262,  267,  275,
     283,  292,  302,  313,  326,  339,  353,  369,
     385,  402,  421,  440,  461,  482,  504,  527,
     551,  576,  602,  629,  657,  685,  714,  744,
     775,  807,  839,  872,  906,  941,  976, 1012,
    1048, 1085, 1123, 1161, 1199, 1239, 1278, 1319,
    1359, 1400, 1442, 1483, 1525, 1568, 1611, 1654,
    1697, 1740, 1784, 1828, 1872, 1916, 1960, 2004,
};


uint8_t aprsFreq[] = "AT+DMOSETGROUP=0,144.3900,144.3900,0000,1,0000\r\n";
uint8_t mainFreq[] = "AT+DMOSETGROUP=0,147.4500,147.4500,0000,1,0000\r\n";
uint8_t filterConfig[] = "AT+SETFILTER=0,0,0\r\n";

uint8_t  bitstream[MAX_BITS];
size_t   bit_len = 0;

uint16_t audio_buffer[AUDIO_BUFFER_SIZE];

/* TX state — written in ISR, read in main: must be volatile */
volatile int tx_done   = 0;
volatile int tx_active = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void setPowerLow(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // LOW = 0.5W
}

void setPowerHigh(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;   // FLOAT
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void setRadio(UART_HandleTypeDef *huart, uint8_t *cmd)
{
    HAL_UART_Transmit(huart, cmd, strlen((char *)cmd), 1000);
    HAL_Delay(200); // give radio time to switch
}

void gps_to_aprs_position(int32_t lat, int32_t lon, char *out)
{
    char lat_hemi = (lat >= 0) ? 'N' : 'S';
    char lon_hemi = (lon >= 0) ? 'E' : 'W';

    if (lat < 0) lat = -lat;
    if (lon < 0) lon = -lon;

    int32_t lat_deg = lat / 10000000;
    int32_t lon_deg = lon / 10000000;

    int32_t lat_rem = lat % 10000000;
    int32_t lon_rem = lon % 10000000;

    // FIX: use int64_t to prevent overflow
    int32_t lat_min = (int32_t)(((int64_t)lat_rem * 60 * 100) / 10000000);
    int32_t lon_min = (int32_t)(((int64_t)lon_rem * 60 * 100) / 10000000);

    int lat_min_int = lat_min / 100;
    int lat_min_dec = lat_min % 100;

    int lon_min_int = lon_min / 100;
    int lon_min_dec = lon_min % 100;

    sprintf(out,
        "!%02ld%02d.%02d%c/%03ld%02d.%02d%c",
        lat_deg, lat_min_int, lat_min_dec, lat_hemi,
        lon_deg, lon_min_int, lon_min_dec, lon_hemi
    );
}
/* =========================================================
 * Bit collector — called by aprs_sendPacket()
 * ========================================================= */
void collectBit(uint8_t bit)
{
    if (bit_len < MAX_BITS) {
        bitstream[bit_len++] = bit;
    }
}

/* =========================================================
 * AFSK modem state
 * Uses a 32-bit phase accumulator + 256-entry LUT.
 * No floating point — safe to call from ISR on Cortex-M0.
 * ========================================================= */
typedef struct {
    uint8_t  *bits;
    size_t    bit_len;
    uint32_t  phase;         /* 32-bit accumulator, full 0..2^32 range */
    uint32_t  sample_index;  /* counts samples since TX start */
} afsk_state_t;

static afsk_state_t afsk;

static void afsk_init(uint8_t *bits, size_t len)
{
    afsk.bits         = bits;
    afsk.bit_len      = len;
    afsk.phase        = 0;
    afsk.sample_index = 0;
}

static uint16_t afsk_next_sample(void)
{
    if (tx_done) {
        return DAC_MID;
    }

    /* SAMPLES_PER_BIT = 16 (exact), so integer division is perfect */
    uint32_t bit_index = afsk.sample_index / SAMPLES_PER_BIT;

    if (bit_index >= afsk.bit_len) {
        tx_done = 1;
        return DAC_MID;
    }

    uint32_t phase_inc = afsk.bits[bit_index] ? PHASE_INC_MARK : PHASE_INC_SPACE;

    /* Advance phase — naturally wraps at 2^32, no branch needed */
    afsk.phase += phase_inc;

    /* Top 8 bits index into the 256-entry LUT */
    uint16_t sample = SINE_LUT[afsk.phase >> 24];

    afsk.sample_index++;

    return sample;
}

/* =========================================================
 * Buffer fill — called from DMA callbacks (ISR context)
 * ========================================================= */
static void fill_buffer(uint16_t *buf, size_t len)
{
    if (tx_done) {
        for (size_t i = 0; i < len; i++) {
            buf[i] = DAC_MID;
        }
    } else {
        for (size_t i = 0; i < len; i++) {
            buf[i] = afsk_next_sample();
        }
    }
}

/* =========================================================
 * DMA half-complete: refill first half of buffer
 * ========================================================= */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac_cb)
{
    fill_buffer(audio_buffer, AUDIO_BUFFER_SIZE / 2);
}

/* =========================================================
 * DMA complete: refill second half, handle shutdown
 * ========================================================= */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac_cb)
{
    /* Always refill second half first so DMA never starves */
    fill_buffer(&audio_buffer[AUDIO_BUFFER_SIZE / 2], AUDIO_BUFFER_SIZE / 2);

    /* Once tx_done and we've drained enough silent buffers, stop */
    if (tx_done && tx_active) {
        static int drain_cycles = 0;
        if (++drain_cycles > 2) {
            HAL_DAC_Stop_DMA(hdac_cb, DAC_CHANNEL_1);
            HAL_TIM_Base_Stop(&htim6);
            tx_active    = 0;
            drain_cycles = 0;
            /* De-assert PTT */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        }
    }
}

void transmit_packet(void)
{
    afsk_init(bitstream, bit_len);

    tx_done   = 0;
    tx_active = 1;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

    fill_buffer(audio_buffer, AUDIO_BUFFER_SIZE);

    HAL_Delay(250);

    HAL_TIM_Base_Start(&htim6);
    HAL_DAC_Start_DMA(
        &hdac,
        DAC_CHANNEL_1,
        (uint32_t *)audio_buffer,
        AUDIO_BUFFER_SIZE,
        DAC_ALIGN_12B_R
    );

    while (tx_active) { }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
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
  MX_DAC_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_UART_Transmit(&huart2, mainFreq, strlen((char *)mainFreq), 1000);
    HAL_Delay(1000);
    HAL_UART_Transmit(&huart2, filterConfig, strlen((char *)filterConfig), 1000);
    HAL_Delay(1000);

    uint32_t lastAprs = 0;
    uint32_t lastSecondary = 0;

    typedef enum {
        FREQ_MAIN,
        FREQ_APRS
    } radio_freq_t;

    radio_freq_t currentFreq = FREQ_MAIN;

    GPS_Begin(&huart3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // Always keep GPS updated
        GPS_Update(&huart3);

        // =========================
        // === APRS (PRIORITY) ===
        // =========================
        if (now - lastAprs >= 60000)
        {
        	while (tx_active) {
        	    // wait or yield
        	}
            lastAprs = now;

            // --- Build packet (STEP 1 goes HERE) ---
            char msg[30];
            gps_to_aprs_position(gps_lat, gps_lon, msg);



            bit_len = 0;
            aprs_sendPacket(
                "KR4GAM",
                "APRS",
                "WIDE1-1,WIDE2-1",
                msg,
                collectBit
            );

            // --- Switch to APRS freq if needed ---
            if (currentFreq != FREQ_APRS) {
            	setPowerHigh();
                setRadio(&huart2, aprsFreq);
                currentFreq = FREQ_APRS;
                HAL_Delay(300);
            }

            // --- Transmit ---
            transmit_packet();
            while (tx_active) {}

            // NEW — let radio fully settle
            HAL_Delay(150);   // try 150–300 ms
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

            HAL_Delay(300);   // let RF chain fully shut down before retuning

            // --- Return to main freq ---
            if (currentFreq != FREQ_MAIN) {
            	setPowerLow();
                setRadio(&huart2, mainFreq);
                currentFreq = FREQ_MAIN;
                HAL_Delay(200);
            }

            continue; // skip secondary this loop
        }

        // =========================
        // === SECONDARY (5 sec) ===
        // =========================
        if (now - lastSecondary >= 5000 && now - lastAprs >= 5000)
        {
            lastSecondary = now;
        	while (tx_active) {
        	    // wait or yield
        	}

            // --- Build SAME packet (STEP 1 here too) ---
            char msg[30];

            gps_to_aprs_position(gps_lat, gps_lon, msg);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

            bit_len = 0;
            aprs_sendPacket(
                "KR4GAM",
                "APRS",
                "WIDE1-1,WIDE2-1",
                msg,
                collectBit
            );

            // --- Ensure we're on main freq ---
            if (currentFreq != FREQ_MAIN) {
            	setPowerLow();
                setRadio(&huart2, mainFreq);
                currentFreq = FREQ_MAIN;
                HAL_Delay(200);
            }

            // --- Transmit ---
            transmit_packet();
            while (tx_active) {}

            // NEW — let radio fully settle
            HAL_Delay(150);   // try 150–300 ms
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

            HAL_Delay(300);   // let RF chain fully shut down before retuning
        }

        HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim6.Init.Prescaler = 24;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
