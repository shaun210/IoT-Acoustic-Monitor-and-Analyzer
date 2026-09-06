#include "connect_to_wifi.h"
#include "wifi.h"
#include "main.h"
#include <stdio.h>
#include <string.h>


#define USER_SSID "Waneve"
#define PASSWORD "WAApassword2210"
#define RemotePORT	8080
#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define CONNECTION_TRIAL_MAX          10
#define TERMINAL_USE


uint8_t  IP_Addr[4] = {0};

uint8_t  MAC_Addr[6] = {0};

uint8_t RemoteIP[4] = { 10, 0, 0, 236 };



void WIFI_Hardware_Reset(void) {
    printf("> Power-cycling Wi-Fi Module...\r\n");
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8; // pin connect to power state of WIFI module
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(500);
}




int Init_And_Connect_WiFi(void) {

    // restart wifi module//
    WIFI_Hardware_Reset();


    if(WIFI_Init() == WIFI_STATUS_OK) {
        printf("> WIFI Module Initialized.\r\n");

        printf("> Connecting to Network: %s...\r\n", USER_SSID);
        if( WIFI_Connect(USER_SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK) {
            printf("> Successfully connected to Wi-Fi!\r\n");

            // 3. Get IP Address
            int dhcp_trials = 10;
            while(dhcp_trials--) {
                if(WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK && IP_Addr[0] != 0) {
                    printf("> STM32 IP Address : %d.%d.%d.%d\r\n", IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]);
                    return 1;
                }
                HAL_Delay(1000);
            }
        }
    }

    printf("> ERROR : Wi-Fi Boot Failed. Halting.\n");

    return 0;
}






int Open_TCP_Connection(void) {
    printf("\r\n> === Connecting to Python Backend ===\r\n");
    printf("> Target: %d.%d.%d.%d:%d\r\n", RemoteIP[0], RemoteIP[1], RemoteIP[2], RemoteIP[3], RemotePORT);

    int trials = CONNECTION_TRIAL_MAX;
    while (trials > 0) {
        if( WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK) {
            printf("> Backend API Connection opened successfully.\r\n");
            return 1;
        }
        trials--;
        HAL_Delay(500);
    }
    printf("> ERROR : Cannot open Backend API Connection\r\n");
    return 0;
}


