#include "ota_update.h"
#include "main.h"
#include "wifi.h"
#include <stdio.h>
#include <string.h>
#include "connect_to_wifi.h"

/* --- WI-FI CREDENTIALS (CHANGE THESE) --- */
#define WIFI_SSID "Waneve"
#define WIFI_PASSWORD "WAApassword2210"
#define SLOT_B_ADDRESS 0x08084000

/* Global Variables */
char http_request[128];
uint8_t http_response[256];
uint32_t expected_ota_crc = 0;

/* Your updated Python Server IP Address */
uint8_t server_ip[] = {10, 0, 0, 236};
uint16_t server_port = 8080;
uint8_t MAC_Addr_test[6] = {0};
uint8_t IP_Addr_test[4] = {0};


OTA_Metadata_t *current_meta = (OTA_Metadata_t *) META_ADDR;




/*
 * Save new version and checksum obtained from online server
 */

void Commit_Update_To_Flash(server_update_info_t *new_info) {
    HAL_FLASH_Unlock();

    // 1. Clear any previous flash errors
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    // 2. Erase the metadata page
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t PageError;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Page = 15;
    EraseInitStruct.NbPages = 1;
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    // 3. Write the new Version and Checksum
    // We cast the struct to a 64-bit pointer to write chunks of 8 bytes
    uint64_t *data_to_write = (uint64_t *)new_info;

    // Writes the version string
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, META_ADDR, data_to_write[0]);

    // Writes the Checksum and the Magic Number (0xDEADBEEF)
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, META_ADDR + 8, data_to_write[1]);

    HAL_FLASH_Lock();

    printf("> Metadata updated. Rebooting to Bootloader...\r\n");
    HAL_Delay(500);
    NVIC_SystemReset();
}






/**
 * @brief  Queries the OTA server for the latest firmware version.
 *         Sends an HTTP GET request for 'version.txt', parses the HTTP response,
 *         and compares the server's version against the board's current version.
 *
 * @note   This function assumes a TCP connection has already been opened by
 *         the state machine. It will ALWAYS close the socket before returning.
 *
 * @param  info: Pointer to a server_update_info_t struct where the new version,
 *               checksum, and update flags will be saved if an update is found.
 *
 * @retval int : 1 if the check was completed and parsed successfully.
 *               0 if a network timeout, transmission error, or parsing error occurred.
 */
int Check_For_Update_OTA(server_update_info_t *info)
{
    uint16_t sent_size = 0;
    bool connected = false;
    int status = 0;

    snprintf(http_request, sizeof(http_request),
             "GET /version.txt HTTP/1.1\r\n"
             "Host: %d.%d.%d.%d\r\n"
             "Connection: close\r\n\r\n",
             server_ip[0], server_ip[1], server_ip[2], server_ip[3]);


    if (WIFI_SendData(0, (uint8_t*)http_request, strlen(http_request),
                      &sent_size, 2000) != WIFI_STATUS_OK)
    {
        printf("> ERROR: Failed to send HTTP GET request.\r\n");
        WIFI_CloseClientConnection(0);
        return 0;
    }

    printf("> Sending data \r\n");
    uint16_t total_received = 0;
    uint16_t chunk_size = 0;
    char *body = NULL;
    int timeout_counter = 0;

    memset(http_response, 0, sizeof(http_response));

    while (total_received < sizeof(http_response) - 1)
    {
        if (WIFI_ReceiveData(0, (uint8_t*)&http_response[total_received], sizeof(http_response) - 1 - total_received, &chunk_size, 5000) == WIFI_STATUS_OK)
        {

            if (chunk_size > 0)
            {
                timeout_counter = 0; // Reset timeout on successful read
                total_received += chunk_size;
                http_response[total_received] = '\0';


                body = strstr(http_response, "\r\n\r\n"); // look for the end of the HTTP headers

                if (body != NULL)
                {
                    // Check if we have received enough of the actual body yet
                    if (strlen(body + 4) >= 15)
                    {
                        body += 4; // Shift pointer past the headers
                        break;
                    }
                }
            }
            else
            {
                // No data in this chunk, wait and increment timeout
                HAL_Delay(100);
                timeout_counter++;

                if (timeout_counter > 50) // 50 * 100ms = 5 seconds
                {
                    printf("> ERROR: Connection timed out or server closed early.\r\n");
                    break;
                }
            }
        }

        else
        {
            printf("> ERROR: Failed while receiving data from server.\r\n");
            break;
        }
    }


    if (body == NULL)
    {
        printf("> ERROR: HTTP body not found in server response.\r\n");
    }
    else
    {
        char server_version[8] = {0}; // We are allowing version of up to 7 character.
        uint32_t server_crc = 0;

        if (sscanf(body, "%7s\r\n%lx", server_version, &server_crc) != 2)
        {
            printf("> ERROR: Failed to parse version.txt.\r\n");
            printf("> Body contents:\r\n%s\r\n", body);
        }
        else
        {
            printf("> Server version : %s\r\n", server_version);
            printf("> Server CRC     : 0x%08lX\r\n", server_crc);

            if (strncmp(server_version, current_meta->version, 8) != 0)
            {
                strncpy(info->version, server_version, 8);
                info->checksum = server_crc;
                info->update_flag = UPDATE_AVAILABLE_FLAG;

                printf("> Update Available! (v%s)\r\n", server_version);
            }
            else
            {
                printf("> Acoustic Monitor firmware is up to date.\r\n");
            }
            status = 1;
        }
    }

    WIFI_CloseClientConnection(0);
    printf("> OTA connection closed.\r\n");

    return status;
}




/**
 * @brief  Erases the secondary flash slot, downloads the new firmware binary
 *         over TCP, and flashes it directly to memory in 8-byte chunks.
 *
 * @note   This function explicitly opens its own TCP connection AFTER the
 *         flash erase process is complete to prevent the server from dropping
 *         the connection due to a timeout during the hardware stall.
 *         If successful, this function will trigger a system reset and NEVER return.
 *
 * @param  update_info: Pointer to the struct containing the expected checksum
 *                      to verify the download integrity.
 *
 * @retval None
 */
void Download_And_Flash_Update(server_update_info_t *update_info)
{
    uint16_t sent_size = 0;
    uint16_t received_size = 0;
    uint32_t current_flash_address = SLOT_B_ADDRESS;
    bool headers_skipped = false;
    uint8_t flash_buffer[2048];
    uint16_t leftover_bytes = 0;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS); // erase slot B

    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t PageError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = FLASH_BANK_2;
    EraseInitStruct.Page      = (SLOT_B_ADDRESS - BANK_2_BASE) / FLASH_PAGE_SIZE;
    EraseInitStruct.NbPages   = SLOT_SIZE / FLASH_PAGE_SIZE;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        printf("> ERROR: Flash Erase Failed at page %lu\r\n", PageError);
        HAL_FLASH_Lock();
        return;
    }
    printf("> Slot B Erased Successfully!\r\n");


    if (!Open_TCP_Connection())
    {
        printf("> ERROR: Could not connect to download binary.\r\n");
        HAL_FLASH_Lock();
        return;
    }

    snprintf(http_request, sizeof(http_request),
             "GET /Audio_Spectrum_Analyser.bin HTTP/1.1\r\n"
             "Host: %d.%d.%d.%d\r\n"
             "Connection: close\r\n\r\n",
             RemoteIP[0], RemoteIP[1], RemoteIP[2], RemoteIP[3]);

    if (WIFI_SendData(0, (uint8_t*)http_request, strlen(http_request), &sent_size, 2000) != WIFI_STATUS_OK)
    {
        printf("> ERROR: Failed to send HTTP GET request.\r\n");
        WIFI_CloseClientConnection(0);
        HAL_FLASH_Lock();
        return;
    }



    printf("> Downloading and Flashing...\r\n");
    int empty_reads = 0;

    while (1)
    {
        uint16_t request_size = sizeof(flash_buffer) - leftover_bytes;
        if (request_size > 1000) {
            request_size = 1000;
        }

        WIFI_Status_t rx_status = WIFI_ReceiveData(0, flash_buffer + leftover_bytes, request_size, &received_size, 5000);

        if (rx_status == WIFI_STATUS_OK)
        {
            if (received_size == 0)
            {
                empty_reads++;
                if (empty_reads > 20) {
                    break;
                }
                HAL_Delay(50);
                continue;
            }

            empty_reads = 0;

            uint8_t *payload = flash_buffer;
            uint16_t total_data_len = received_size + leftover_bytes;

            // Strip HTTP headers robustly
            if (!headers_skipped)
            {
                char *body_start = NULL;

                // go through Header till we find \r\n\r\n
                for(uint16_t k = 0; k < total_data_len - 3; k++) {
                    if(flash_buffer[k] == '\r' && flash_buffer[k+1] == '\n' &&
                       flash_buffer[k+2] == '\r' && flash_buffer[k+3] == '\n') {
                        body_start = (char*)&flash_buffer[k];
                        break;
                    }
                }

                if (body_start != NULL)
                {
                    body_start += 4;
                    uint16_t header_size = (uint16_t)((uint8_t*)body_start - flash_buffer);
                    payload = (uint8_t*)body_start;
                    total_data_len -= header_size;
                    headers_skipped = true;
                }
                else
                {
                    leftover_bytes = total_data_len;
                    continue;
                }
            }

            // Write to Flash in 8-byte chunks
            uint16_t bytes_written = 0;
            for (uint16_t i = 0; i + 8 <= total_data_len; i += 8)
            {
                uint64_t double_word;
                memcpy(&double_word, &payload[i], 8);

                if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, current_flash_address, double_word) == HAL_OK)
                {
                    current_flash_address += 8;
                    bytes_written += 8;
                }
                else
                {
                    printf("\r\n> ERROR: Flash Write Failed at 0x%08lX\r\n", current_flash_address);
                    break;
                }
            }

            // Handle leftovers
            leftover_bytes = total_data_len - bytes_written;
            if (leftover_bytes > 0)
            {
                memmove(flash_buffer, &payload[bytes_written], leftover_bytes);
            }
        }
        else
        {
            // Network connection closed by server
            break;
        }
    }

    // Write any final remaining bytes
    if (leftover_bytes > 0)
    {
        uint64_t final_word = 0xFFFFFFFFFFFFFFFF;
        memcpy(&final_word, flash_buffer, leftover_bytes);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, current_flash_address, final_word);
        current_flash_address += 8;
    }

    printf("\r\n> Download and Flash Complete!\r\n");
    WIFI_CloseClientConnection(0);

    uint32_t total_padded_bytes = current_flash_address - SLOT_B_ADDRESS;
    printf("> [DEBUG] Total Padded Bytes Written to Flash: %lu\r\n", total_padded_bytes);

    if (Verify_Downloaded_Firmware(update_info->checksum, total_padded_bytes))
    {
        printf("> Proceeding to set installation flags...\r\n");
        Commit_Update_To_Flash(update_info); // NOTE: This function resets the board!
    }
    else
    {
        printf("> Aborting update. The corrupted file will be ignored.\r\n");
    }

    HAL_FLASH_Lock();
}



// Standard IEEE 802.3 (zlib) Software CRC
uint32_t Calculate_Software_CRC32(uint8_t *data, uint32_t length)
{
    // Rule 1: Initial value is 0xFFFFFFFF
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < length; i++)
    {
        // Rule 3: XOR the current byte with the lowest 8 bits of the CRC register
        crc ^= data[i];

        // Rule 4: Process all 8 bits of the current byte (LSB first)
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1) {
                // If the lowest bit is 1, shift right and XOR with reversed polynomial
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                // If the lowest bit is 0, just shift right
                crc >>= 1;
            }
        }
    }

    // Rule 5: Final XOR
    return crc ^ 0xFFFFFFFF;
}


bool Verify_Downloaded_Firmware(uint32_t expected_crc, uint32_t total_bytes_written)
{
    printf("\r\n> === Verifying Firmware Checksum ===\r\n");


    uint32_t calculated_crc = Calculate_Software_CRC32((uint8_t *)SLOT_B_ADDRESS, total_bytes_written);

    printf("> Expected CRC   : 0x%08lX\r\n", expected_crc);
    printf("> Calculated CRC : 0x%08lX\r\n", calculated_crc);

    if (calculated_crc == expected_crc)
    {
        printf("> SUCCESS: Checksum matches! Firmware is fully intact.\r\n");
        return true;
    }
    else
    {
        printf("> ERROR: Checksum mismatch! Firmware was corrupted in transit.\r\n");
        return false;
    }
}



//void Initialize_Metadata_If_Fresh(void) {
//    OTA_Metadata_t *meta = (OTA_Metadata_t *)META_ADDR;
//
//    // Check if the Flash is empty (0xFF)
//    if ((uint8_t)meta->version[0] == 0xFF) {
//        printf("> Fresh board detected. Initializing Metadata to v1.0...\r\n");
//
//        HAL_FLASH_Unlock();
//
//        // 1. Erase Page 15 (Where 0x08007800 lives)
//        FLASH_EraseInitTypeDef EraseInitStruct;
//        uint32_t PageError;
//        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//        EraseInitStruct.Banks     = FLASH_BANK_1;
//        EraseInitStruct.Page      = 15;
//        EraseInitStruct.NbPages   = 1;
//        HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
//
//        // 2. Prepare the v1.0 data
//        OTA_Metadata_t initial_meta = {0};
//        strncpy(initial_meta.version, "1.0", 8);
//        initial_meta.checksum = 0;
//        initial_meta.update_flag = 0;
//
//        // 3. Program the data (DoubleWord = 8 bytes)
//        uint64_t *ptr = (uint64_t *)&initial_meta;
//        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, META_ADDR, ptr[0]);
//        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, META_ADDR + 8, ptr[1]);
//
//        HAL_FLASH_Lock();
//    }
//}
//





