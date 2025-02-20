/****************************************************************************
 *
 *   Copyright (c) 2021 IMProject Development Team. All rights reserved.
 *   Authors: Igor Misic <igy1000mb@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name IMProject nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "crc32.h"
#include "utils.h"
#include "software_info.h"
#include "bootloader.h"
#ifdef STM32L4xx
#include "stm32l4xx_hal.h"
#elif STM32H7xx
#include "stm32h7xx_hal.h"
#elif STM32F7xx
#include "stm32f7xx_hal.h"
#endif
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdint.h>

typedef struct signature {
    uint64_t magic_key;
    uint64_t unused[7];
} signature_s;

#ifdef STM32L4xx
#define MAGIC_KEY_ADDRESS (0x0800F800)
#elif STM32H7xx
#define MAGIC_KEY_ADDRESS (0x080202A0)
#elif STM32F7xx
#define MAGIC_KEY_ADDRESS (0x08020200U)
#endif

#ifdef STM32H735xx
#define CDC_Transmit CDC_Transmit_HS
#else
#define CDC_Transmit CDC_Transmit_FS
#endif

#define SIGNATURE_MAGIC_KEY 0xDEC0DE5528101987
#define BOOTLOADER_MAGIC_KEY 0x28101987A5B5C5D5

__attribute__ ((section(".fw_signature"))) signature_s firmware_signature = {.magic_key = SIGNATURE_MAGIC_KEY};
__attribute__ ((section(".bootloader_flag_flash"))) uint64_t bootloader_flag_flash[4] =
{ BOOTLOADER_MAGIC_KEY, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF };
__attribute__ ((section(".bootloader_flag_ram"))) uint64_t bootloader_flag_ram[4] =
{ 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF };

#define SW_TYPE_STR             "software_type"         //!< String for bootloader to send if IMFlasher is connected to bootloader
#define GET_VERSION_CMD         "version"               //!< String command for bootloader to send version
#define GET_SW_INFO_JSON_CMD    "software_info_json"    //!< String command for bootloader to send software info in JSON format
#define STR_ENTER_BL            "enter_bl"              //!< String command to write flag to RAM and enter bootloader
#define STR_FLASH_FW            "flash_fw"              //!< String command for erase magic key and enter bootloader
#define STR_IM_APPLICATION      "IMApplication"         //!< String for inform IMFlasher this is application
#define STR_ACK_OK              "OK"
#define STR_ACK_NOK             "NOK"

#define CRC_INIT_VALUE  (0xFFFFFFFFU)   //!< CRC init value
#define XOR_CRC_VALUE   (0xFFFFFFFFU)   //!< XOR CRC value

static bool FirmwareUpdate_sendStringWithCrc(uint8_t* string, size_t size);

bool static s_reset = false;

void
Bootloader_checkCommand(uint8_t* buf, uint32_t length) {

    uint8_t tx_buffer[300];

    if (0 == strcmp((char*)buf, SW_TYPE_STR)) {
        CDC_Transmit((uint8_t*)STR_IM_APPLICATION, strlen(STR_IM_APPLICATION));

    } else if (0 == strcmp((char*)buf, STR_ENTER_BL)) {

        CDC_Transmit((uint8_t*)STR_ACK_OK, strlen(STR_ACK_OK));
        Bootloader_enterBLOverRam();

    } else if (0 == strcmp((char*)buf, STR_FLASH_FW)) {

        Bootloader_enterBLOverFlash();

    } else if (0 == strcmp((char*)buf, GET_VERSION_CMD)) {

        SwInfo_getVersion(tx_buffer, sizeof(tx_buffer));
        CDC_Transmit(tx_buffer, strlen((char*)tx_buffer));

    } else if (0 == strcmp((char*)buf, GET_SW_INFO_JSON_CMD)) {

        SwInfo_getDataJson(tx_buffer, sizeof(tx_buffer));
        FirmwareUpdate_sendStringWithCrc(tx_buffer, sizeof(tx_buffer));
    } else {
        CDC_Transmit((uint8_t*)STR_ACK_NOK, strlen(STR_ACK_NOK));
    }
}

void
Bootloader_enterBLOverRam(void) {
    bootloader_flag_ram[0] =  BOOTLOADER_MAGIC_KEY;
    s_reset = true;
}

void
Bootloader_enterBLOverFlash(void) {

    /* Erase the page with the magic key so the bootloader knows it needs to flash the firmware.
     * Once erased new firmware needs to be flashed. For entering in BL without flashing use enterBLOverRam.
     */
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef      status = HAL_OK;
#ifdef STM32L4xx
    FLASH_EraseInitTypeDef pEraseInit;
    uint32_t               PageError  = 0;

    uint32_t erase_page = (MAGIC_KEY_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;

    pEraseInit.Banks     = FLASH_BANK_1;
    pEraseInit.NbPages   = 1;
    pEraseInit.Page      = erase_page;
    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    status               = HAL_FLASHEx_Erase(&pEraseInit, &PageError);

#elif STM32H7xx
    // H7 support only sector erase. To save space we are using the method to overwrite flash of the firmware.
    uint8_t data[32] = {0};
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, MAGIC_KEY_ADDRESS, (uint32_t)data);

#elif STM32F7xx
    uint32_t data = 0U;

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MAGIC_KEY_ADDRESS, (uint64_t) data);

    if (status == HAL_OK) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MAGIC_KEY_ADDRESS + 4U, (uint64_t) data);
    }
#else
    status = HAL_ERROR;
#endif


    if (status == HAL_OK) {
        s_reset = true;
    }
}

void
Bootloader_resetHandler(void) {
    if (s_reset) {
        HAL_Delay(200); //wait for last ACK to be sent
        HAL_NVIC_SystemReset();
    }
}

static bool
FirmwareUpdate_sendStringWithCrc(uint8_t* string, size_t size) {

    bool success = false;
    bool null_terminator_exist = false;

    for (size_t i = 0U; i < size; ++i) {
        if (string[i] == '\0') {
            null_terminator_exist = true;
            break;
        }
    }

    if (null_terminator_exist) {

        size_t last_char = strlen((char*)string);

        if (size >= last_char) {
            uint32_t crc = CalculateCRC32(&string[0], last_char, CRC_INIT_VALUE, XOR_CRC_VALUE, false, false, true);
            Utils_Serialize32BE(&string[last_char], crc);
            CDC_Transmit(string, last_char + sizeof(crc));
            success = true;
        }
    }

    return success;
}
