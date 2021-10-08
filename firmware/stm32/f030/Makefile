# Makefile for 'i2c_bme280' example Nodate project for STM32.
#

# Architecture must be set.
# E.g.: STM32, AVR, SAM, ESP8266.
ARCH ?= stm32

# Target programming language (Ada, C++)
NDLANGUAGE ?= cpp

# One can use the board preset.
#BOARD ?= nucleo-f042k6
#BOARD ?= blue_pill
#BOARD ?= blue_pill_wch
#BOARD ?= stm32f4-discovery
#BOARD ?= nucleo-f746zg

#BOARD ?= purple
# the following is used for the Purple board
#APP_FLAGS = -DPURPLE
BOARD ?= nucleo-f030r8
#BOARD ?= nucleo-f103rb
#BOARD ?= nucleo-f446re
#BOARD ?= nucleo-f042k6
#BOARD ?= nucleo-l432kc
#BOARD ?= nucleo-f746zg
# the following is used for the nucleo-f746zg
#APP_FLAGS = -DHSE_VALUE=8000000

# Set the MCU and programmer types.
#
# MCU
#MCU ?= stm32f042k6t

# Set the name of the output (ELF & Hex) file.
OUTPUT := controller


# Add files to include for compilation to these variables.
APP_CPP_FILES = $(wildcard src/*.cpp) 
APP_CPP_FILES += $(wildcard src/mcu/*.cpp)
APP_C_FILES = $(wildcard src/*.c)
APP_C_FILES += $(wildcard src/mcu/*.c)
LIB_INCLUDES += -I $(APPFOLDER)/inc
LIB_INCLUDES += -I $(APPFOLDER)/inc/mcu


# Set Nodate modules to enable.
# Available modules:
# ethernet, i2c, gpio, interrupts, timer, usart
NODATE_MODULES = gpio timer i2c usart

# Set library modules to enable.
# library name matches the folder name in libs/. E.g. freertos, LwIP, libscpi, bme280
NODATE_LIBRARIES = bme280



#
# --- End of user-editable variables --- #
#

# Nodate includes. Requires that the NODATE_HOME environment variable has been set.
APPFOLDER=$(CURDIR)
export

all:
	$(MAKE) -C $(NODATE_HOME)
	
flash:
	$(MAKE) -C $(NODATE_HOME) flash
	
clean:
	$(MAKE) -C $(NODATE_HOME) clean
