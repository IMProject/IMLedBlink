/*
 * Copyright (C) 2021  Igor Misic, igy1000mb@gmail.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 *
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#include "version.h"
#include "bootloader.h"
#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define MAGIC_KEY_ADDRESS 0x08007800
#define ENTER_MAGIC_KEY 0x5555281019875555

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
    } else if (0 == strcmp((char*)buf, STR_FLASH_FW)) {

        Version_copyToBuffer((uint8_t*)buffer, sizeof(buffer));
        CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
    }

}

void
Bootloader_enterBL(void) {

    //Erase the page with the magic key so the bootloader knows it needs to flash the firmware.
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef      status = HAL_OK;
    FLASH_EraseInitTypeDef pEraseInit;
    uint32_t               PageError  = 0;

    uint32_t erase_page = (MAGIC_KEY_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;

    pEraseInit.Banks     = FLASH_BANK_1;
    pEraseInit.NbPages   = 1;
    pEraseInit.Page      = erase_page;
    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    status               = HAL_FLASHEx_Erase(&pEraseInit, &PageError);

    if (status == HAL_OK) {

        HAL_NVIC_SystemReset();
    }
}

