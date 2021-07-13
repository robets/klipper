# Additional atsamc build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/atsamc src/generic
dirs-$(CONFIG_MACH_SAMC21) += lib/samc21/gcc
dirs-$(CONFIG_CANSERIAL) += lib/fast-hash

MCU := $(shell echo $(CONFIG_MCU) | tr a-z A-Z)

CFLAGS-$(CONFIG_MACH_SAMC21) += -mcpu=cortex-m0plus -Ilib/samc21/include
CFLAGS += $(CFLAGS-y) -D__$(MCU)__ -mthumb -Ilib/cmsis-core -Ilib/fast-hash

CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

# Add source files
src-y += atsamc/main.c atsamc/gpio.c generic/crc16_ccitt.c
src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_reset.c
src-$(CONFIG_USBSERIAL) += atsamc/usbserial.c atsamc/chipid.c generic/usb_cdc.c
src-$(CONFIG_SERIAL) += atsamc/serial.c generic/serial_irq.c
src-$(CONFIG_CANSERIAL) += atsamc/can.c ../lib/fast-hash/fasthash.c atsamc/mcan.c
src-$(CONFIG_CANSERIAL) += generic/canbus.c
src-$(CONFIG_HAVE_GPIO_ADC) += atsamc/adc.c
src-$(CONFIG_HAVE_GPIO_I2C) += atsamc/i2c.c
src-$(CONFIG_HAVE_GPIO_SPI) += atsamc/spi.c
src-$(CONFIG_HAVE_SERCOM) += atsamc/sercom.c
src-$(CONFIG_HAVE_GPIO_HARD_PWM) += atsamc/hard_pwm.c
src-$(CONFIG_MACH_SAMC21) += atsamc/watchdog.c
src-$(CONFIG_MACH_SAMC21) += atsamc/clock.c atsamc/timer.c generic/timer_irq.c

# Build the additional hex and bin output files
target-y += $(OUT)klipper.bin $(OUT)klipper.elf.hex

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating bin file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

$(OUT)klipper.elf.hex: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -j .text -j .relocate -O ihex $< $@

# Flash rules
lib/bossac/bin/bossac:
	@echo "  Building bossac"
	$(Q)make -C lib/bossac bin/bossac

BOSSAC_OFFSET-$(CONFIG_MACH_SAMC21) := 0x2000

flash: $(OUT)klipper.bin lib/bossac/bin/bossac
	@echo "  Flashing $< to $(FLASH_DEVICE)"
	$(Q)$(PYTHON) ./scripts/flash_usb.py -t $(CONFIG_MCU) -d "$(FLASH_DEVICE)" -s "$(BOSSAC_OFFSET-y)" $(OUT)klipper.bin
