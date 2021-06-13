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

#include "version.h"
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

#ifdef STM32L4xx
#define MAGIC_KEY_ADDRESS (0x08007800)
#elif STM32H7xx
#define MAGIC_KEY_ADDRESS (0x080202A0)
#elif STM32F7xx
#define MAGIC_KEY_ADDRESS (0x08020200U)
#endif

#define SW_TYPE_STR                 "software_type"     //!< String for bootloader to send if IMFlasher is connected to bootloader
#define GET_VERSION_CMD             "version"           //!< String command for bootloader to send version
#define STR_FLASH_FW                "flash_fw"          //!< String command for erase magic key and enter bootloader
#define STR_IM_APPLICATION          "IMApplication"     //!< String for inform IMFlasher this is application

void
Bootloader_checkCommand(uint8_t* buf, uint32_t length) {

    char buffer[300];

    if (0 == strcmp((char*)buf, SW_TYPE_STR)) {
        sprintf(buffer, STR_IM_APPLICATION);
        CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));

    } else if (0 == strcmp((char*)buf, STR_FLASH_FW)) {

        Bootloader_enterBL();
    } else if (0 == strcmp((char*)buf, GET_VERSION_CMD)) {

        Version_copyToBuffer((uint8_t*)buffer, sizeof(buffer));
        CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
    }
}

void
Bootloader_enterBL(void) {

    //Erase the page with the magic key so the bootloader knows it needs to flash the firmware.
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

        HAL_NVIC_SystemReset();
    }
}

