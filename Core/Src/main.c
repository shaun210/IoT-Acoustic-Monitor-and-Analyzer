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

#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "secrets.h"
#include "wifi.h"
#include "stm32l475e_iot01.h"
#include "ota_update.h"
#include "connect_to_wifi.h"

#include "wolfssl/ssl.h"
#include "wolfssl/error-ssl.h"
#include "wolfssl/wolfcrypt/error-crypt.h"

/*
 * Always comment out void SPI3_IRQHandler(void) in stm32l4xx_it.c
 *
 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */





/** Process_fft**/
 #define VOLUME_THRESHOLD 100000
#define CRY_FREQ_MIN 700
#define CRY_FREQ_MAX 1200
#define SAMPLE_RATE       12500  // (Clock speed/divider) / oversampling rate
#define HALF_BUFFER_LEN           1024   // Must match your buffer processing
#define WIFI_WRITE_TIMEOUT 10000

/** Smart monitor timing PD **/
// 1. How long is one "tick"? (1024 samples / 12.5kHz) = 81ms, with divider = 100
#define FRAME_S ((float)HALF_BUFFER_LEN / (float)SAMPLE_RATE)

// 2. Window Size: 5 / 0.08192 = 61.03 -> (int)61
#define WINDOW_SEC 5
#define WINDOW_SIZE (int)(WINDOW_SEC / FRAME_S)

#define PROOF_SEC 0.5
#define TRIGGER_THRESHOLD (int)(PROOF_SEC / FRAME_S)



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c2;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint32_t pChannel = 0; // Needed to store which channel the data came from
int32_t mic_val = 0;   // To store the actual audio data

volatile uint8_t button_pressed_flag = 0;


/** DMA **/
#define BUFF_SIZE 2048
int32_t RecBuff[BUFF_SIZE]; // The "Big Box" for audio samples
uint8_t half_transfer_flag = 0; // Flag: First half is full
uint8_t full_transfer_flag = 0; // Flag: Second half is full


/** FT SETTINGS  **/
#define HALF_BUFFER_LEN 1024  // Length of the analysis window
arm_rfft_fast_instance_f32 fft_handler; // The FFT "Object"
float fft_in_buf[HALF_BUFFER_LEN];    // Input: Audio in floats
float fft_out_buf[HALF_BUFFER_LEN];   // Output: Complex numbers (Real + Imaginary)
float fft_mag_buf[HALF_BUFFER_LEN/2]; // Final: Magnitude (Just the loudness)
volatile uint8_t buffer_state = 0;


uint16_t history_idx = 0;   // Where are we writing in the circle?
uint16_t current_score = 0; // Running sum of votes (How many 1s are in the array?)
uint8_t alarm_active = 0;   // Prevent spamming WiFi

/**Wifi**/


int32_t Socket = -1;

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
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define REQ_BUFFER_SIZE  1024
#define RESP_BUFFER_SIZE 1024

static uint8_t req_buffer[REQ_BUFFER_SIZE];
static uint8_t resp_buffer[RESP_BUFFER_SIZE];

const char ca_cert_bundle[] =
"-----BEGIN CERTIFICATE-----\n"
"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\n"
"-----END CERTIFICATE-----\n";


static int my_verify_callback(int preverify, WOLFSSL_X509_STORE_CTX* store) {
    // Return 1 to bypass strict verification for testing purposes.
    // In production, return 'preverify' to enforce CA validation.
    return 1;

}

static int my_IOSend(WOLFSSL* ssl, char* buf, int sz, void* ctx) {
    int socket_id = wolfSSL_get_fd(ssl);
    uint16_t total_sent = 0;
    int retries = 3;

    while (total_sent < sz) {
        uint16_t chunk_size = (sz - total_sent > 1200) ? 1200 : (sz - total_sent);
        uint16_t sent_len = 0;

        if (WIFI_SendData(socket_id, (uint8_t*)(buf + total_sent), chunk_size, &sent_len, 2000) != WIFI_STATUS_OK) {
            return WOLFSSL_CBIO_ERR_GENERAL;
        }

        if (sent_len == 0) {
        	retries--;
            if (retries <= 0) {
                return WOLFSSL_CBIO_ERR_TIMEOUT;
            }
            continue;
        }

        retries = 3;
        total_sent += sent_len;
    }

    return total_sent;
}

/**
  * @brief  Custom wolfSSL I/O receive callback for the Inventek ISM43362 Wi-Fi module.
  * @param  ssl:  Pointer to the wolfSSL session context.
  * @param  buf:  Destination buffer for received bytes.
  * @param  sz:   Requested read length (in bytes).
  * @param  ctx:  User-defined I/O context pointer.
  * @retval Number of bytes successfully received, or a negative wolfSSL I/O error code.
  *
  * @note   If there is a timeout, Inventek will still return WIFI_STATUS_OK (no error), thus leading to an infinite loop.
  *			This is why I added a limited amount of retry.
  */
static int my_IORecv(WOLFSSL* ssl, char* buf, int sz, void* ctx) {
    int socket_id = wolfSSL_get_fd(ssl);
    uint16_t rec_len = 0;
    uint16_t chunk_size = (sz > 1200) ? 1200 : sz;
    int retries = 3;

    while (retries--) {
        if (WIFI_ReceiveData(socket_id, (uint8_t*)buf, chunk_size, &rec_len, 5000) != WIFI_STATUS_OK) {
            return WOLFSSL_CBIO_ERR_GENERAL;
        }
        if (rec_len > 0) {
            return rec_len;
        }
    }

    return WOLFSSL_CBIO_ERR_TIMEOUT;
}


typedef enum {
	SYS_SLEEP,
	SYS_WIFI_CONNECTING,
	SYS_RUNNING_WITH_NO_WIFI,
	SYS_TCP_CONNECTING,
	SYS_RUNNING_WITH_NO_TCP,
	OTA_CHECKING_UPDATE,
	OTA_UPDATING,
	SYS_NORMAL_RUNNING


} Device_State_t;


Device_State_t current_device_state = SYS_SLEEP;



int _write(int file, char *ptr, int len)
{
//  (void)file;
//  int DataIdx;
//
//  for (DataIdx = 0; DataIdx < len; DataIdx++)
//  {
//    ITM_SendChar(*ptr++);
//  }
	HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}








float Process_FFT(int32_t *r)
{
    int32_t *source_buffer = r;

    // --- STEP 1: MATH (Calculate FFT) ---
    // 1. Remove DC Offset
    int64_t sum = 0;
    for (int i = 0; i < HALF_BUFFER_LEN; i++) sum += source_buffer[i];
    float avg = (float)sum / (float)HALF_BUFFER_LEN;
    for (int i = 0; i < HALF_BUFFER_LEN; i++) fft_in_buf[i] = (float)source_buffer[i] - avg;

    // 2. Run FFT
    arm_rfft_fast_f32(&fft_handler, fft_in_buf, fft_out_buf, 0);
    arm_cmplx_mag_f32(fft_out_buf, fft_mag_buf, HALF_BUFFER_LEN/2);


    // --- STEP 2: ANALYZE (Find the Peak strictly for Baby Crying) ---
    float maxVal = 0.0f;
    uint32_t maxIndex = 0;

    uint32_t bin_min = (CRY_FREQ_MIN * HALF_BUFFER_LEN) / SAMPLE_RATE;
    uint32_t bin_max = (CRY_FREQ_MAX * HALF_BUFFER_LEN) / SAMPLE_RATE;

    // Only scan for peaks WITHIN the baby frequency band
    for (uint32_t i = bin_min; i <= bin_max; i++)
    {
        if (fft_mag_buf[i] > maxVal)
        {
            maxVal = fft_mag_buf[i];
            maxIndex = i;
        }
    }

    // Convert Bin Index back to Frequency in Hz
    float dominant_freq = (float)maxIndex * ((float)SAMPLE_RATE / (float)HALF_BUFFER_LEN);

    // --- STEP 3: FILTER (Is it loud enough?) ---
    if (maxVal > VOLUME_THRESHOLD)
    {
        printf(">>> MATCH! Baby Frequency Detected: %.0f Hz  at max frequency: %.2f<<<\r\n", dominant_freq, maxVal);
        return dominant_freq;
    }

    return 0.0f; // Return 0 if silence or non-baby noise
}


static int Send_Sensor_Payload(WOLFSSL* ssl, const char *sensor_name, float value, int keep_alive) {
    char json_body[128];
    int json_len = snprintf(json_body, sizeof(json_body),  "{\"sensor_name\": \"%s\", \"value\": %.2f}", sensor_name, value);

    int req_len = snprintf((char *)req_buffer, sizeof(req_buffer),
        "POST /data HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Content-Type: application/json\r\n"
        "X-API-Key: %s\r\n"
        "Content-Length: %d\r\n"
        "Connection: %s\r\n"
        "\r\n"
        "%s",
        SERVER_HOST, API_KEY, json_len, keep_alive ? "keep-alive" : "close", json_body);

    if (wolfSSL_write(ssl, req_buffer, req_len) < 0) {
        return 0;
    }

    memset(resp_buffer, 0, sizeof(resp_buffer));
    int total_bytes = 0;
    int success = 0;

    // The railway server send multiple respond packet. We must clear Inventek buffer before moving on.
    while (total_bytes < (sizeof(resp_buffer) - 1)) {
        int bytes_read = wolfSSL_read(ssl, resp_buffer + total_bytes, sizeof(resp_buffer) - total_bytes - 1);

        if (bytes_read <= 0) {
            break;
        }

        total_bytes += bytes_read;
        if (strstr((char *)resp_buffer, "HTTP/1.1 201") != NULL ||
            strstr((char *)resp_buffer, "HTTP/1.1 200") != NULL) {
            success = 1;
        }
        if (strstr((char *)resp_buffer, "}") != NULL) {
            break;
        }
    }

    return success;
}

int Process_Backlog_And_Send_Live(float press_val) {
    uint8_t server_ip[4] = {0};
    uint8_t TCP_success = 1;

    if (WIFI_GetHostAddress((char *)SERVER_HOST, server_ip, sizeof(server_ip)) != WIFI_STATUS_OK) {
        printf("> DNS lookup failed\r\n");
        return 0;
    }

    wolfSSL_Init();
    WOLFSSL_CTX* ctx = wolfSSL_CTX_new(wolfTLSv1_2_client_method());
    if (ctx == NULL) {
        printf("> Error: Failed to create wolfSSL context\r\n");
        return 0;
    }

    wolfSSL_SetIOSend(ctx, my_IOSend);
    wolfSSL_SetIORecv(ctx, my_IORecv);

    int ret = wolfSSL_CTX_load_verify_buffer(ctx, (const unsigned char*)ca_cert_bundle, strlen(ca_cert_bundle), WOLFSSL_FILETYPE_PEM);
    if (ret != WOLFSSL_SUCCESS) {
        char err_str[80];
        wolfSSL_ERR_error_string(ret, err_str);
        printf("Buffer load failed: %s\n", err_str);
        wolfSSL_CTX_free(ctx);
        wolfSSL_Cleanup();
        return 0;
    }

    wolfSSL_CTX_set_verify(ctx, WOLFSSL_VERIFY_PEER, my_verify_callback);
    wolfSSL_CTX_UseSNI(ctx, WOLFSSL_SNI_HOST_NAME, SERVER_HOST, strlen(SERVER_HOST));

    if (WIFI_OpenClientConnection(SOCKET_ID, WIFI_TCP_PROTOCOL, "FlaskServer", server_ip, SERVER_PORT, 0) != WIFI_STATUS_OK) {
        printf("> TCP connection failed\r\n");
        wolfSSL_CTX_free(ctx);
        return 0;
    }

    WOLFSSL* ssl = wolfSSL_new(ctx);
    wolfSSL_set_fd(ssl, SOCKET_ID);

    printf("> Starting wolfSSL TLS Handshake...\r\n");
    if (wolfSSL_connect(ssl) != WOLFSSL_SUCCESS) {
        int err = wolfSSL_get_error(ssl, 0);
        char err_str[80];
        wolfSSL_ERR_error_string(err, err_str);
        printf("wolfSSL_connect error %d: %s\n", err, err_str);
        wolfSSL_free(ssl);
        wolfSSL_CTX_free(ctx);
        WIFI_CloseClientConnection(SOCKET_ID);
        return 0;
    }
    printf("> Secure TLS connection established via wolfSSL!\r\n");

    /*
     * --- LITTLEFS BACKLOG STUBBED FOR TLS TESTING ---
     *
     * lfs_file_t file;
     * int file_opened = (lfs_file_open(&lfs, &file, "backlog.bin", LFS_O_RDONLY) == 0);
     * if (file_opened) { ... }
     */

    if (TCP_success) {
        printf("> Sending live reading: %.2f\r\n", press_val);
        if (Send_Sensor_Payload(ssl, "humidity_percent", press_val, 0) == 0) {
            // Save_Offline_Data("humidity_percent", press_val);
            printf("> Failed to send live payload.\r\n");
        }
    }

    wolfSSL_free(ssl);
    wolfSSL_CTX_free(ctx);
    wolfSSL_Cleanup();
    WIFI_CloseClientConnection(SOCKET_ID);
    printf("> Connection closed.\r\n");

    return TCP_success;
}


void Process_Acoustic_Monitor(void)
{
    if (buffer_state != 0)
    {

        int32_t *current_chunk = (buffer_state == 1) ? &RecBuff[0] : &RecBuff[BUFF_SIZE/2];


        float freq = Process_FFT(current_chunk);


        uint8_t new_vote = (freq > 0) ? 1 : 0;

        current_score -= vote_history[history_idx];
        current_score += new_vote;
        vote_history[history_idx] = new_vote;

        history_idx++;
        if (history_idx >= WINDOW_SIZE) history_idx = 0;


        if (current_score >= TRIGGER_THRESHOLD)
        {
            if (alarm_active == 0)
            {
                 printf("!!! ALARM TRIGGERED !!! (Score: %d/%d)\n", current_score, WINDOW_SIZE);

                 if ((current_device_state == SYS_NORMAL_RUNNING)) {
					  char msg[256];

					  sprintf(msg,
							  "GET /alert?freq=%.0f HTTP/1.1\r\n"
							  "Host: %d.%d.%d.%d\r\n"
							  "Connection: keep-alive\r\n"
							  "\r\n",
							  freq, RemoteIP[0], RemoteIP[1], RemoteIP[2], RemoteIP[3]);



					  if ( WIFI_SendData(0, (uint8_t*)msg, strlen(msg), &Datalen, WIFI_WRITE_TIMEOUT) != WIFI_STATUS_OK) {

					      printf("> ERROR: Backend stream disconnected unexpectedly.\r\n");

					      WIFI_CloseClientConnection(0);

					      current_device_state = SYS_RUNNING_WITH_NO_TCP;

					      return;
					  }
				  }
                 alarm_active = 1;
            }
        }
        else
        {

            if (current_score < (TRIGGER_THRESHOLD / 2) && alarm_active == 1)
            {
                 printf("Alarm Cleared. (Score: %d)\n", current_score);
                 BSP_LED_Off(LED2);
                 alarm_active = 0;
            }
        }

        buffer_state = 0;
    }
}




void Go_To_Stop_Sleep(void) {
	printf("> Entering STOP 2 Low-Power Mode...\r\n");

	//  wait for the printf text to fully leave the UART hardware
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
		// Wait...
	}

    WIFI_CloseClientConnection(0);


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // LED1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // LED2

	HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);

	HAL_NVIC_DisableIRQ(EXTI1_IRQn);  // wifi interrupt
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All); // Clear any pending interrupt flag
	HAL_SuspendTick();

	// The CPU stops here until any external interrupt fires (in our case, user_btn)
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	HAL_ResumeTick();

	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	SystemClock_Config();

	if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff, BUFF_SIZE) != HAL_OK)
	{
	  Error_Handler();
	}

	printf("> Woken up from STOP 2 Mode!\r\n");
}



void Device_ProcessEvent() {

	static server_update_info_t update_data = {0};

	switch(current_device_state) {


		case SYS_SLEEP:
			Go_To_Stop_Sleep();

			if (button_pressed_flag) {
				button_pressed_flag = 0;
				current_device_state = SYS_WIFI_CONNECTING;
			}
			break;



		case SYS_WIFI_CONNECTING:

			if (button_pressed_flag) {
				button_pressed_flag = 0;
				current_device_state = SYS_SLEEP;
				break;
			}

			if (Init_And_Connect_WiFi()) {

				current_device_state = SYS_TCP_CONNECTING;
			}

			else {
				current_device_state = SYS_RUNNING_WITH_NO_WIFI;
			}


			break;


		case SYS_TCP_CONNECTING:

			if (!Open_TCP_Connection()) {
				current_device_state = SYS_RUNNING_WITH_NO_TCP;
				break;
			}

			current_device_state = OTA_CHECKING_UPDATE;
			break;


		case OTA_CHECKING_UPDATE:

			if (Check_For_Update_OTA(&update_data)) {

				if (update_data.update_flag == UPDATE_AVAILABLE_FLAG) {
					// turn on LED
					current_device_state = OTA_UPDATING;
				}
				else {
					HAL_Delay(500);
					if (Open_TCP_Connection()) {
						current_device_state = SYS_NORMAL_RUNNING;
					} else {
						current_device_state = SYS_RUNNING_WITH_NO_TCP;
					}
				}
			}
			else {
				HAL_Delay(500);
				if (Open_TCP_Connection()) {
					current_device_state = SYS_NORMAL_RUNNING;
				} else {
					current_device_state = SYS_RUNNING_WITH_NO_TCP;
				}
			}

			break;



		case OTA_UPDATING:
			Download_And_Flash_Update(&update_data);
			// this will reboot system
			break;



		case SYS_RUNNING_WITH_NO_WIFI:
		{
			if (hdfsdm1_filter0.State == HAL_DFSDM_FILTER_STATE_READY) { // this checks if dma already on
				HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff, BUFF_SIZE);
			}
			Process_Acoustic_Monitor();

			static uint32_t last_wifi_retry_tick = 0;
			uint32_t current_tick = HAL_GetTick();

			if (current_tick - last_wifi_retry_tick >= 60000)
			{
				last_wifi_retry_tick = current_tick; // Reset the timer

				printf("> 60 seconds passed. Attempting Wi-Fi reconnection...\r\n");

				// Stop the audio DMA so it doesn't overflow while the CPU is busy connecting
				HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);

				current_device_state = SYS_WIFI_CONNECTING;
				break;
			}

			if (button_pressed_flag) {
				button_pressed_flag = 0;
				printf("> User requested Sleep Mode. Transitioning...\r\n");
				current_device_state = SYS_SLEEP;
			}
			break;
		}


		case SYS_RUNNING_WITH_NO_TCP:
		{
			if (hdfsdm1_filter0.State == HAL_DFSDM_FILTER_STATE_READY) { // this checks if dma already on
				HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff, BUFF_SIZE); // Start the microphone data samplinh
			}

			Process_Acoustic_Monitor();

			static uint32_t last_tcp_retry_tick = 0;
			uint32_t current_tick = HAL_GetTick();

			if (current_tick - last_tcp_retry_tick >= 10000)
			{
				last_tcp_retry_tick = current_tick;

				uint8_t local_ip[4] = {0};


				if (WIFI_GetIP_Address(local_ip, sizeof(local_ip))  != WIFI_STATUS_OK ||
				   (local_ip[0] == 0 && local_ip[1] == 0))
				{
					printf("> ERROR: Lost IP Address. Router disconnected.\r\n");

					HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
					current_device_state = SYS_RUNNING_WITH_NO_WIFI;
					break;
				}
				else
				{
					printf("> Wi-Fi is active (IP: %d.%d.%d.%d). Retrying TCP connection...\r\n",
							local_ip[0], local_ip[1], local_ip[2], local_ip[3]);

					HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
					current_device_state = SYS_TCP_CONNECTING;
					break;
				}
			}

			if (button_pressed_flag) {
				button_pressed_flag = 0;
				printf("> User requested Sleep Mode. Transitioning...\r\n");
				current_device_state = SYS_SLEEP;
			}
			break;
		}

		case SYS_NORMAL_RUNNING:
		{
			if (hdfsdm1_filter0.State == HAL_DFSDM_FILTER_STATE_READY) {
				HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff, BUFF_SIZE);
			}

			Process_Acoustic_Monitor();

			// Periodically check if connection alive ( every 300 seconds)
			static uint32_t last_tcp_check_tick = 0;
			uint32_t current_tick = HAL_GetTick();

			if (current_tick - last_tcp_check_tick >= 10000)
			{
				last_tcp_check_tick = current_tick;
				uint16_t sent_size = 0;
				char heartbeat_req[128];
				snprintf(heartbeat_req, sizeof(heartbeat_req),
						 "GET /ping HTTP/1.1\r\n"
						 "Host: %d.%d.%d.%d\r\n"
						 "Connection: keep-alive\r\n\r\n",
						 RemoteIP[0], RemoteIP[1], RemoteIP[2], RemoteIP[3]);

				if (WIFI_SendData(0, (uint8_t*)heartbeat_req, strlen(heartbeat_req), &sent_size, 500) != WIFI_STATUS_OK)
				{
					printf("> ERROR: Periodic TCP heartbeat failed. Dropping to offline mode...\r\n");

					WIFI_CloseClientConnection(0);
					current_device_state = SYS_RUNNING_WITH_NO_TCP;
					break;
				}
			}

			if (button_pressed_flag) {
				button_pressed_flag = 0;
				printf("> User requested Sleep Mode. Transitioning...\r\n");
				current_device_state = SYS_SLEEP;
			}
			break;
		}

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

  SCB->VTOR = 0x08008000;

  __enable_irq();



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
  MX_SPI3_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */



  arm_rfft_fast_init_f32(&fft_handler, HALF_BUFFER_LEN);


//  if (Init_And_Connect_WiFi()) {
//        Process_Backlog_And_Send_Live(10.0);
//    } else {
//        printf("> Wi-Fi Connection Failed. Cannot send payload.\r\n");
//    }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  Process_Acoustic_Monitor();
	  Device_ProcessEvent();
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
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
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
  hdfsdm1_channel2.Init.OutputClock.Divider = 100;
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
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */
//
  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */
//
  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
//
  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UPDATE_LED_GPIO_Port, UPDATE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UPDATE_LED_Pin */
  GPIO_InitStruct.Pin = UPDATE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UPDATE_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* USER CODE BEGIN 4 */

// 1. Called when the first half (0 to 1023) is full
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    buffer_state = 1;
}

// 2. Second Half Full
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    buffer_state = 2;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == USER_BTN_Pin)
    {
        button_pressed_flag = 1;
        printf("button pressed mdk \r\n");
    }

    else if (GPIO_Pin == GPIO_PIN_1)
    {
        SPI_WIFI_ISR();
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
