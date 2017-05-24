# ============================================================================
# Makefile for nRF52 projects. Most of the settings is picked from the examples
# and the makefile is a bit simpler than the ones included in the SDK.
# =============================================================================

# =============================================================================
# Tool chain settings
#
GNU_INSTALL_ROOT := /usr/local
GNU_VERSION      := 5.4.1 
GNU_PREFIX       := arm-none-eabi

# =============================================================================
# Directories
#
FLASH_DRIVE    := /Volumes/JLINK
NRF52_SDK_ROOT := ~/Source/11.nrf52sdk
TEMPLATE_PATH  := $(NRF52_SDK_ROOT)/components/toolchain/gcc

# =============================================================================
# Project settings
#
PROJECT_NAME  := nrf52lora
LINKER_SCRIPT := gcc_nrf52.ld

# =============================================================================
# Commands
#
RM := rm -fR
CP := cp

# =============================================================================
# Tool chain names
#
CC      := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS      := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR      := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD      := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM      := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE    := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

# =============================================================================
# Assembler sources
#
ASM_SOURCE_FILES  =3 $(NRF52_SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52.s

# =============================================================================
# Includes
#
INC_PATHS += -I ./config/
INC_PATHS += -I ./lmic/lmic/
INC_PATHS += -I ./lmic/sx1276/
INC_PATHS += -I ./lsm9ds1/
INC_PATHS += -I ./gps/
INC_PATHS += -I ./battery/
INC_PATHS += -I ./cache/
INC_PATHS += -I ./jobs/
INC_PATHS += -I ./tps22994/
INC_PATHS += -I ./bno055/


INC_PATHS += -I $(NRF52_SDK_ROOT)/components/toolchain/CMSIS/Include/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/toolchain/gcc/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/toolchain/gcc/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/toolchain/
INC_PATHS += -I $(NRF52_SDK_ROOT)/examples/bsp/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/device/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/delay/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/hal/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/libraries/trace/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/libraries/uart/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/libraries/util/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/softdevice/s132/headers/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/uart/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/config/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/common/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/libraries/fifo/
INC_PATHS += -I $(NRF52_SDK_ROOT)/external/segger_rtt/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/timer/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/rtc/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/clock/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/gpiote/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/spi_master/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/saadc/
INC_PATHS += -I $(NRF52_SDK_ROOT)/components/drivers_nrf/twi_master/

# =============================================================================
# Compiler flags
#
CFLAGS += -DDEBUG
#CFLAGS += -DCFG_noassert
CFLAGS += -DBOARD_PCA10040
CFLAGS += -DNRF52
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O0 -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums
CFLAGS += -DNRF_LOG_USES_RTT=1
# Set LMiC to use 868MHz
CFLAGS += -DCFG_eu868
CFLAGS += -DCFG_sx1276_radio
# Because sloppy programming
LMIC_CFLAGS := -Werror=unused-variable

# =============================================================================
# Linker flags
#
LDFLAGS += -Xlinker -Map=$(PROJECT_NAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs -lc -lnosys
# For debugging; print floats
LDFLAGS += -u _printf_float

# =============================================================================
# Assembler flags
#
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DNRF52
ASMFLAGS += -DBSP_DEFINES_ONLY


ifeq ("$(VERBOSE)","1")
	NO_ECHO :=
else
	NO_ECHO := @
endif

all: hexfile

clean:
	$(NO_ECHO)$(RM) *.o *.out *.hex *.bin *.map

# Note the clever placement of -lm because linking static libs (and sx1276
# lib uses math)
linker: compiles
	$(NO_ECHO)$(CC) $(LDFLAGS) *.o -lm -o $(PROJECT_NAME).out

compiles: ee0x lmiclib nrf52

ee0x:
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c main.c
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c cache/*.c
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c jobs/*.c
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c battery/*.c
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c gps/*.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) tps22994/*.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) bno055/*.c

lmiclib:
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) lmic/lmic/*.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) lmic/nrf52/*.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) lmic/sx1276/*.c

# =============================================================================
# nRF52 Modules

nrf52:
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/toolchain/system_nrf52.c
	$(NO_ECHO)$(CC) -c $(ASMFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52.s
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/libraries/util/app_util_platform.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/libraries/util/app_error.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/libraries/trace/app_trace.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/libraries/util/nrf_log.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/drivers_nrf/delay/nrf_delay.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/drivers_nrf/rtc/nrf_drv_rtc.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/drivers_nrf/spi_master/nrf_drv_spi.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/libraries/uart/app_uart_fifo.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/libraries/fifo/app_fifo.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/drivers_nrf/saadc/nrf_drv_saadc.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/drivers_nrf/hal/nrf_saadc.c
	$(NO_ECHO)$(CC) -c $(CFLAGS) $(INC_PATHS) \
		$(NRF52_SDK_ROOT)/components/drivers_nrf/twi_master/nrf_drv_twi.c


# =============================================================================
# Modules
genbin: linker
	$(NO_ECHO)$(OBJCOPY) -O binary $(PROJECT_NAME).out  $(PROJECT_NAME).bin

hexfile: genbin
	$(NO_ECHO)$(OBJCOPY) -O ihex $(PROJECT_NAME).out $(PROJECT_NAME).hex

deploy: hexfile
	@echo Deploying $(PROJECT_NAME).hex to $(FLASH_DRIVE)
	$(NO_ECHO)$(CP) $(PROJECT_NAME).hex $(FLASH_DRIVE)
