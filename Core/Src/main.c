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

#include <stdio.h> // For sprintf
#include <string.h> // For strlen
#include "arm_math.h"
#include "wifi.h"
#include "stm32l475e_iot01.h"
//#include "es_wifi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USER_SSID     "Shaun"
#define PASSWORD "titeufff"


#define RemotePORT	8002

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000

#define CONNECTION_TRIAL_MAX          10

#define TERMINAL_USE



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c2;



UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint32_t pChannel = 0; // Needed to store which channel the data came from
int32_t mic_val = 0;   // To store the actual audio data


/** DMA **/
#define BUFF_SIZE 2048
int32_t RecBuff[BUFF_SIZE]; // The "Big Box" for audio samples
uint8_t half_transfer_flag = 0; // Flag: First half is full
uint8_t full_transfer_flag = 0; // Flag: Second half is full


/** FT SETTINGS  **/
#define FFT_LEN 1024  // Length of the analysis window
arm_rfft_fast_instance_f32 fft_handler; // The FFT "Object"
float fft_in_buf[FFT_LEN];    // Input: Audio in floats
float fft_out_buf[FFT_LEN];   // Output: Complex numbers (Real + Imaginary)
float fft_mag_buf[FFT_LEN/2]; // Final: Magnitude (Just the loudness)
volatile uint8_t buffer_state = 0;

/**Wifi**/
uint8_t  MAC_Addr[6] = {0};
uint8_t RemoteIP[] = { 10, 13, 170, 182 };
//uint8_t RemoteIP[] = { 100,107,45,161};

int32_t Socket = -1;
int16_t Trials = CONNECTION_TRIAL_MAX;
uint8_t  IP_Addr[4] = {0};
uint16_t Datalen;
int32_t ret;
extern ES_WIFIObject_t EsWifiObj;

#if defined (TERMINAL_USE)
#define TERMOUT(...)  printf(__VA_ARGS__)
#else
#define TERMOUT(...)
#endif

extern SPI_HandleTypeDef hspi;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int _write(int file, char *ptr, int len)
{
//  (void)file;
//  int DataIdx;
//
//  for (DataIdx = 0; DataIdx < len; DataIdx++)
//  {
//    ITM_SendChar(*ptr++);
//  }
	HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);
  return len;
}

void Process_FFT(int32_t *source_buffer)
{

    // --- STEP 1: MATH (Calculate FFT) ---
    // 1. Remove DC Offset
    int64_t sum = 0;
    for (int i = 0; i < FFT_LEN; i++) sum += source_buffer[i];
    float avg = (float)sum / (float)FFT_LEN;

    for (int i = 0; i < FFT_LEN; i++) fft_in_buf[i] = (float)source_buffer[i] - avg;

    // 2. Run FFT
    arm_rfft_fast_f32(&fft_handler, fft_in_buf, fft_out_buf, 0);
    arm_cmplx_mag_f32(fft_out_buf, fft_mag_buf, FFT_LEN/2);



}

void Connect_to_Mobile() {

    if(WIFI_Init() == WIFI_STATUS_OK) {
        TERMOUT("> WIFI Module Initialized.\n");
//        Scan_For_Networks();
        HAL_Delay(2000);
        if(WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK)
        {
            TERMOUT("> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\n",
                     MAC_Addr[0], MAC_Addr[1], MAC_Addr[2],
                     MAC_Addr[3], MAC_Addr[4], MAC_Addr[5]);
        }
        else
        {
            TERMOUT("> ERROR : CANNOT get MAC address\n");
            BSP_LED_On(LED2);
        }

        // Try to connectå
        // WPA_WPA2_PSK often handles "Security: 3" better on some hotspots
        if( WIFI_Connect(USER_SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
        {
            TERMOUT("> es-wifi module connected \n");
            // --- Standard Success Path ---
            if(WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK)
            {
                TERMOUT("> es-wifi module got IP Address : %d.%d.%d.%d\n",
                       IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]);

                TERMOUT("> Trying to connect to Server: %d.%d.%d.%d:%d ...\n",
                       RemoteIP[0], RemoteIP[1], RemoteIP[2], RemoteIP[3], RemotePORT);

                while (Trials--)
                {
                    if( WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
                    {
                        TERMOUT("> TCP Connection opened successfully.\n");
                        Socket = 0;
                        break;
                    }
                }
                if(Socket == -1)
                {
                    TERMOUT("> ERROR : Cannot open Connection\n");
                    BSP_LED_On(LED2);
                }
            }
            else
            {
                TERMOUT("> ERROR : es-wifi module CANNOT get IP address\n");
                BSP_LED_On(LED2);
            }
        }
        else
        {
                // If we still don't have an IP, it's a real failure.
                TERMOUT("> ERROR : es-wifi module truly NOT connected\n");
                BSP_LED_On(LED2);

        }
    }
    else
    {
        TERMOUT("> ERROR : WIFI Module cannot be initialized.\n");
        BSP_LED_On(LED2);
    }
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
  MX_DFSDM1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
//  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */



  BSP_LED_Init(LED2);


  Connect_to_Mobile();

//  MX_DMA_Init();
//  MX_DFSDM1_Init();
//
//  HAL_StatusTypeDef status;
//
//  arm_rfft_fast_init_f32(&fft_handler, FFT_LEN);
//
//  status = HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff, BUFF_SIZE);
//
//  printf("yolo2\n");
//
//  if (status != HAL_OK)
//  {
//      // If you are in Debug mode, put a breakpoint here!
//      // Check if status is HAL_BUSY or HAL_ERROR
//      Error_Handler();
//  }
//
//  if (HAL_DFSDM_FilterRegularStart(&hdfsdm1_filter0) != HAL_OK)
//    {
//  	  Error_Handler();
//    }

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = DISABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = DISABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 64;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 26;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x08;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  hi2c2.Init.Timing = 0x10D19CE4;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
//  */
//static void MX_SPI3_Init(void)
//{
//
//  /* USER CODE BEGIN SPI3_Init 0 */
//////
//  /* USER CODE END SPI3_Init 0 */
//
//  /* USER CODE BEGIN SPI3_Init 1 */
//////
//  /* USER CODE END SPI3_Init 1 */
//  /* SPI3 parameter configuration*/
//  hspi3.Instance = SPI3;
//  hspi3.Init.Mode = SPI_MODE_MASTER;
//  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
//  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi3.Init.NSS = SPI_NSS_SOFT;
//  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi3.Init.CRCPolynomial = 7;
//  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
//  if (HAL_SPI_Init(&hspi3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI3_Init 2 */
//////
//  /* USER CODE END SPI3_Init 2 */
//
//}

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* USER CODE BEGIN 4 */

// 1. Called when the first half (0 to 1023) is full
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    buffer_state = 1; // Set flag: "Go check the first half!"
}

// 2. Second Half Full
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    buffer_state = 2; // Set flag: "Go check the second half!"
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (GPIO_PIN_1):
    {
      SPI_WIFI_ISR();
      break;
    }
    default:
    {
      break;
    }
  }
}

void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
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
