/*
 * ota_update.h
 *
 *  Created on: Mar 1, 2026
 *      Author: shaunsoobagrah
 */

#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#include <stdint.h>
#include <stdbool.h>


extern bool update_available;
extern float new_version;


#define META_ADDR  0x08007800
#define UPDATE_AVAILABLE_FLAG 0xDEADBEEF


#define SLOT_A_START           0x08008000
#define SLOT_B_START           0x08084000
#define SLOT_SIZE              (496 * 1024) // 496 KB
#define BANK_2_BASE      0x08080000

typedef struct  {
	char     version[8];
	uint32_t checksum;
	uint32_t update_flag;

} server_update_info_t;


typedef struct {
    char     version[8];
    uint32_t checksum;
    uint32_t update_flag;   // 0xDEADBEEF means "Update Ready"

} OTA_Metadata_t;

/* Function Prototypes */
int Check_For_Update_OTA(server_update_info_t *info);
void Download_And_Flash_Update(server_update_info_t *update_info);
bool Verify_Downloaded_Firmware(uint32_t expected_crc, uint32_t file_size_bytes);
void Initialize_Metadata_If_Fresh(void);

#endif
