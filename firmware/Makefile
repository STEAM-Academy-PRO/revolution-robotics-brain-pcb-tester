# This Makefile was generated using "python -m tools.generate_makefile"
C_SRCS += \
    ./Atmel/Device_Startup\startup_samd51.c \
    ./Atmel/Device_Startup\system_samd51.c \
    ./Atmel/hal/src/hal_adc_async.c \
    ./Atmel/hal/src/hal_adc_sync.c \
    ./Atmel/hal/src/hal_atomic.c \
    ./Atmel/hal/src/hal_cache.c \
    ./Atmel/hal/src/hal_delay.c \
    ./Atmel/hal/src/hal_evsys.c \
    ./Atmel/hal/src/hal_ext_irq.c \
    ./Atmel/hal/src/hal_flash.c \
    ./Atmel/hal/src/hal_gpio.c \
    ./Atmel/hal/src/hal_init.c \
    ./Atmel/hal/src/hal_io.c \
    ./Atmel/hal/src/hal_sleep.c \
    ./Atmel/hal/src/hal_spi_m_dma.c \
    ./Atmel/hal/src/hal_spi_m_sync.c \
    ./Atmel/hal/src/hal_timer.c \
    ./Atmel/hal/src/hal_wdt.c \
    ./Atmel/hal/utils/src/utils_assert.c \
    ./Atmel/hal/utils/src/utils_event.c \
    ./Atmel/hal/utils/src/utils_list.c \
    ./Atmel/hal/utils/src/utils_ringbuffer.c \
    ./Atmel/hal/utils/src/utils_syscalls.c \
    ./Atmel/hpl/adc/hpl_adc.c \
    ./Atmel/hpl/cmcc/hpl_cmcc.c \
    ./Atmel/hpl/core/hpl_core_m4.c \
    ./Atmel/hpl/core/hpl_init.c \
    ./Atmel/hpl/dmac/hpl_dmac.c \
    ./Atmel/hpl/eic/hpl_eic.c \
    ./Atmel/hpl/evsys/hpl_evsys.c \
    ./Atmel/hpl/gclk/hpl_gclk.c \
    ./Atmel/hpl/mclk/hpl_mclk.c \
    ./Atmel/hpl/nvmctrl/hpl_nvmctrl.c \
    ./Atmel/hpl/osc32kctrl/hpl_osc32kctrl.c \
    ./Atmel/hpl/oscctrl/hpl_oscctrl.c \
    ./Atmel/hpl/pm/hpl_pm.c \
    ./Atmel/hpl/ramecc/hpl_ramecc.c \
    ./Atmel/hpl/sercom/hpl_sercom.c \
    ./Atmel/hpl/systick/hpl_systick.c \
    ./Atmel/hpl/tcc/tcc_lite.c \
    ./Atmel/hpl/tc/hpl_tc.c \
    ./Atmel/hpl/tc/tc_lite.c \
    ./driver_init.c \
    ./imu.c \
    ./led.c \
    ./main.c \
    ./test.c

INCLUDE_PATHS += \
    ./Config \
    ./ \
    ./Atmel/hal/include \
    ./Atmel/hal/utils/include \
    ./Atmel/hpl/adc \
    ./Atmel/hpl/cmcc \
    ./Atmel/hpl/core \
    ./Atmel/hpl/dmac \
    ./Atmel/hpl/eic \
    ./Atmel/hpl/gclk \
    ./Atmel/hpl/mclk \
    ./Atmel/hpl/osc32kctrl \
    ./Atmel/hpl/oscctrl \
    ./Atmel/hpl/pm \
    ./Atmel/hpl/port \
    ./Atmel/hpl/ramecc \
    ./Atmel/hpl/rtc \
    ./Atmel/hpl/sercom \
    ./Atmel/hpl/systick \
    ./Atmel/hpl/tc \
    ./Atmel/hpl/tcc \
    ./Atmel/hpl/usb \
    ./Atmel/hpl/wdt \
    ./Atmel/hri \
	./Atmel/samd51a/include \
    ./CMSIS/Include

COMPILE_FLAGS += \
	-x c \
	-mthumb \
	-D__SAMD51P19A__ \
	-ffunction-sections \
	-fdata-sections \
	-mlong-calls \
	-Wall \
	-Wextra \
	-Wundef \
	-Wdouble-promotion \
	-mcpu=cortex-m4 \
	-c \
	-std=gnu99 \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-MD \
	-MP

LINKER_FLAGS := \
	-mthumb \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-mcpu=cortex-m4 \
	--specs=nano.specs \
	-TConfig/samd51p19a_flash.ld

ifeq ($(OS),Windows_NT)
	SHELL := cmd.exe
	MKDIR := md
	GCC_BINARY_PREFIX := "C:/gcc/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-
	GCC_BINARY_SUFFIX := .exe"
	NULL := nul
	DEL := rmdir /s /q
	TRUE := VER>nul
else
	SHELL := /bin/bash
	MKDIR := mkdir -p
	GCC_BINARY_PREFIX := /usr/share/gcc-arm/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-
	GCC_BINARY_SUFFIX :=
	NULL := /dev/null
	DEL := rm -rf
	TRUE := true
endif

DEBUG_COMPILE_FLAGS := 	\
	-DDEBUG \
	-O0 \
	-fanalyzer \
	-g3

RELEASE_COMPILE_FLAGS := \
	-O3 \
	-g3 \
	-fanalyzer \
	-flto

ifeq ($(config), debug)
OUTPUT_DIR :=Build/Debug/mcu-tester
COMPILE_FLAGS += $(DEBUG_COMPILE_FLAGS)
else
OUTPUT_DIR :=Build/Release/mcu-tester
COMPILE_FLAGS += $(RELEASE_COMPILE_FLAGS)
LINKER_FLAGS += \
	-flto
endif

OUTPUT_FILE :=$(OUTPUT_DIR)/rrrc_eoltest

all: $(OUTPUT_FILE).elf

OBJS := $(C_SRCS:%.c=$(OUTPUT_DIR)/%.o)
C_DEPS := $(OBJS:%.o=%.d)

ifneq ($(MAKECMDGOALS),clean)
-include $(C_DEPS)
endif

$(OUTPUT_DIR)/%.d: %.c
	@echo Collecting dependencies: $<
	@$(MKDIR) "$(@D)" 2>$(NULL) || $(TRUE)
	@$(GCC_BINARY_PREFIX)gcc$(GCC_BINARY_SUFFIX) $(addprefix -I,$(INCLUDE_PATHS)) $(COMPILE_FLAGS) -MF $@ -MT$@ -M $<

$(OUTPUT_DIR)/%.o: %.c
	@echo Building file: $<
	@$(GCC_BINARY_PREFIX)gcc$(GCC_BINARY_SUFFIX) $(addprefix -I,$(INCLUDE_PATHS)) $(COMPILE_FLAGS) -o $@ $<
	@echo Finished building: $<

$(OUTPUT_FILE).elf: $(OBJS)
	@echo Building target: $@
	@$(GCC_BINARY_PREFIX)gcc$(GCC_BINARY_SUFFIX) -o$(OUTPUT_FILE).elf $(OBJS) $(LINKER_FLAGS) -Wl,-Map=$(OUTPUT_FILE).map -Wl,--start-group -lm  -Wl,--end-group -Wl,--gc-sections
	@echo Finished building target: $@
	@$(GCC_BINARY_PREFIX)objcopy$(GCC_BINARY_SUFFIX) -O binary $(OUTPUT_FILE).elf $(OUTPUT_FILE).bin
	$(GCC_BINARY_PREFIX)size$(GCC_BINARY_SUFFIX) $(OUTPUT_FILE).elf

clean:
	-@$(DEL) Build
	@echo Removed Build directory
