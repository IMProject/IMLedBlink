######################################
# target
######################################
export TARGET = IMLedBlink

######################################
# building variables
######################################
# debug build?
export DEBUG = 1
# optimization
export OPT = -Og

######################################
# Git branch and hash
######################################
export BRANCH := $(shell git rev-parse --abbrev-ref HEAD)
export HASH := $(shell git rev-parse HEAD)
export TAG := $(shell git tag --sort=committerdate | tail -1)

#######################################
# paths
#######################################

export COMMON_SRCS =  \
LedBlink/Src/main.c \
LedBlink/Src/bootloader.c \
LedBlink/STM32/Src/usb_device.c \
LedBlink/STM32/Src/usbd_conf.c \
LedBlink/STM32/Src/usbd_desc.c \
LedBlink/STM32/Src/usbd_cdc_if.c \
LedBlink/Utility/Src/version.c \
LedBlink/Utility/Src/utils.c \
LedBlink/Utility/Src/json.c \
LedBlink/Utility/Src/crc32.c \
LedBlink/STM32/Src/system_clock.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

export COMMON_INCS = \
-ILedBlink/Inc \
-ILedBlink/Utility/Inc \
-ILedBlink/STM32/Inc \
-IDrivers/CMSIS/Include \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IExternalDevices/Winbond/Inc/

#######################################
# binaries
#######################################
export PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
export CC = $(GCC_PATH)/$(PREFIX)gcc
export AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
export CP = $(GCC_PATH)/$(PREFIX)objcopy
export SZ = $(GCC_PATH)/$(PREFIX)size
else
export CC = $(PREFIX)gcc
export AS = $(PREFIX)gcc -x assembler-with-cpp
export CP = $(PREFIX)objcopy
export SZ = $(PREFIX)size
endif
export HEX = $(CP) -O ihex
export BIN = $(CP) -O binary -S

#######################################
# Astyle
#######################################
.PHONY: check_format format

check_format:
	$(call colorecho,'Checking formatting with astyle')
	@Tools/astyle/check_code_style_all.sh
	@git diff --check

format:
	$(call colorecho,'Formatting with astyle')
	@Tools/astyle/check_code_style_all.sh --fix

#
# LedBlinks to build
#
TARGETS	= \
	stm32l4xx \
	stm32h7xx \
	stm32f7xx \
	matek_H7_slim \
	pixhawk4

all:	$(TARGETS)

clean:
	rm -f *.elf *.bin *.map # Remove any elf or bin files contained directly in the LedBlink directory
	rm -rf build # Remove build directories

#
# Board specific targets.
#
matek_H7_slim:
	${MAKE} stm32h7xx BOARD=MATEK_H743_SLIM BOARD_FILE_NAME=$@

pixhawk4:
	${MAKE} stm32f7xx BOARD=PIXHAWK4 BOARD_FILE_NAME=$@

#
# Microcontroller specific targets.
#

stm32l4xx: $(MAKEFILE_LIST)
	${MAKE} -f Makefile.stm32l4xx LDSCRIPT=STM32L4xx FLASH=INTERNAL_FLASH TARGET_FILE_NAME=$@

stm32h7xx: $(MAKEFILE_LIST)
	${MAKE} -f Makefile.stm32h7xx LDSCRIPT=STM32H7xx FLASH=INTERNAL_FLASH TARGET_FILE_NAME=$@

stm32h7xx_ext: $(MAKEFILE_LIST)
	${MAKE} -f Makefile.stm32h7xx LDSCRIPT=STM32H7xx FLASH=EXTERNAL_FLASH TARGET_FILE_NAME=$@

stm32f7xx: $(MAKEFILE_LIST)
	${MAKE} -f Makefile.stm32f7xx LDSCRIPT=STM32F7xx FLASH=INTERNAL_FLASH TARGET_FILE_NAME=$@
