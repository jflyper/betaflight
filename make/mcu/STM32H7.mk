#
# H7 Make file include
#

ifeq ($(DEBUG_HARDFAULTS),H7)
CFLAGS               += -DDEBUG_HARDFAULTS
endif

#CMSIS
CMSIS_DIR      := $(ROOT)/lib/main/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32H7/Drivers/STM32H7xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))

EXCLUDES        = \
                #stm32h7xx_hal.c \
                #stm32h7xx_hal_adc.c \
                #stm32h7xx_hal_adc_ex.c \
                stm32h7xx_hal_cec.c \
                stm32h7xx_hal_comp.c \
                #stm32h7xx_hal_cortex.c \
                stm32h7xx_hal_crc.c \
                stm32h7xx_hal_crc_ex.c \
                stm32h7xx_hal_cryp.c \
                stm32h7xx_hal_cryp_ex.c \
                stm32h7xx_hal_dac.c \
                stm32h7xx_hal_dac_ex.c \
                stm32h7xx_hal_dcmi.c \
                stm32h7xx_hal_dfsdm.c \
                #stm32h7xx_hal_dma.c \
                stm32h7xx_hal_dma2d.c \
                #stm32h7xx_hal_dma_ex.c \
                stm32h7xx_hal_eth.c \
                stm32h7xx_hal_eth_ex.c \
                stm32h7xx_hal_fdcan.c \
                #stm32h7xx_hal_flash.c \
                #stm32h7xx_hal_flash_ex.c \
                #stm32h7xx_hal_gpio.c \
                stm32h7xx_hal_hash.c \
                stm32h7xx_hal_hash_ex.c \
                stm32h7xx_hal_hcd.c \
                stm32h7xx_hal_hrtim.c \
                stm32h7xx_hal_hsem.c \
                #stm32h7xx_hal_i2c.c \
                #stm32h7xx_hal_i2c_ex.c \
                stm32h7xx_hal_i2s.c \
                stm32h7xx_hal_i2s_ex.c \
                stm32h7xx_hal_irda.c \
                stm32h7xx_hal_iwdg.c \
                stm32h7xx_hal_jpeg.c \
                stm32h7xx_hal_lptim.c \
                stm32h7xx_hal_ltdc.c \
                stm32h7xx_hal_mdios.c \
                stm32h7xx_hal_mdma.c \
                stm32h7xx_hal_mmc.c \
                stm32h7xx_hal_mmc_ex.c \
                stm32h7xx_hal_nand.c \
                stm32h7xx_hal_nor.c \
                stm32h7xx_hal_opamp.c \
                stm32h7xx_hal_opamp_ex.c \
                #stm32h7xx_hal_pcd.c \
                #stm32h7xx_hal_pcd_ex.c \
                #stm32h7xx_hal_pwr.c \
                #stm32h7xx_hal_pwr_ex.c \
                stm32h7xx_hal_qspi.c \
                #stm32h7xx_hal_rcc.c \
                #stm32h7xx_hal_rcc_ex.c \
                stm32h7xx_hal_rng.c \
                stm32h7xx_hal_rtc.c \
                stm32h7xx_hal_rtc_ex.c \
                stm32h7xx_hal_sai.c \
                stm32h7xx_hal_sai_ex.c \
                stm32h7xx_hal_sd.c \
                stm32h7xx_hal_sd_ex.c \
                stm32h7xx_hal_sdram.c \
                stm32h7xx_hal_smartcard.c \
                stm32h7xx_hal_smartcard_ex.c \
                stm32h7xx_hal_smbus.c \
                stm32h7xx_hal_spdifrx.c \
                #stm32h7xx_hal_spi.c \
                #stm32h7xx_hal_spi_ex.c \
                stm32h7xx_hal_sram.c \
                stm32h7xx_hal_swpmi.c \
                #stm32h7xx_hal_tim.c \
                #stm32h7xx_hal_tim_ex.c \
                #stm32h7xx_hal_uart.c \
                #stm32h7xx_hal_uart_ex.c \
                stm32h7xx_hal_usart.c \
                stm32h7xx_hal_wwdg.c \
                stm32h7xx_ll_delayblock.c \
                stm32h7xx_ll_fmc.c \
                stm32h7xx_ll_sdmmc.c \
                #stm32h7xx_ll_usb.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

#USB
USBCORE_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/Src/*.c))
EXCLUDES    = usbd_conf_template.c
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/Src/*.c))
EXCLUDES   = usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

USBHID_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Class/HID
USBHID_SRC = $(notdir $(wildcard $(USBHID_DIR)/Src/*.c))

USBHIDCDC_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Class/CDC_HID
USBHIDCDC_SRC = $(notdir $(wildcard $(USBHIDCDC_DIR)/Src/*.c))

USBMSC_DIR = $(ROOT)/lib/main/STM32H7/Middlewares/ST/STM32_USB_Device_Library/Class/MSC
USBMSC_SRC = $(notdir $(wildcard $(USBMSC_DIR)/Src/*.c))
EXCLUDES   = usbd_msc_storage_template.c usbd_msc_scsi.c
USBMSC_SRC := $(filter-out ${EXCLUDES}, $(USBMSC_SRC))

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src:$(USBHID_DIR)/Src:$(USBHIDCDC_DIR)/Src:$(USBMSC_DIR)/Src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC) \
                        $(USBHID_SRC) \
                        $(USBHIDCDC_SRC) \
                        $(USBMSC_SRC)

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device/ST/STM32H7xx
VPATH           := $(VPATH):$(STDPERIPH_DIR)/Src
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCORE_DIR)/Inc \
                   $(USBCDC_DIR)/Inc \
                   $(USBHID_DIR)/Inc \
                   $(USBHIDCDC_DIR)/Inc \
                   $(USBMSC_DIR)/Inc \
                   $(CMSIS_DIR)/Core/Include \
                   $(ROOT)/lib/main/STM32H7/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
                   $(ROOT)/src/main/vcph7

ifneq ($(filter SDCARD_SPI,$(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR)
VPATH           := $(VPATH):$(FATFS_DIR)
endif

ifneq ($(filter SDCARD_SDIO,$(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR)
VPATH           := $(VPATH):$(FATFS_DIR)
endif

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -fsingle-precision-constant -Wdouble-promotion

DEVICE_FLAGS    = -DUSE_HAL_DRIVER

#
# H743xI : 2M FLASH, 1M RAM (H753xI also)
# H743xG : 1M FLASH, 1M RAM (H753xG also)
# H750xB : 128K FLASH, 1M RAM
#
ifeq ($(TARGET),$(filter $(TARGET),$(H743xI_TARGETS)))
DEVICE_FLAGS   += -DSTM32H743xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_h743_2m.ld
STARTUP_SRC     = startup_stm32h743xx.s
TARGET_FLASH   := 2048
else
$(error Unknown MCU for H7 target)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

TARGET_FLAGS    = -D$(TARGET)

VCP_SRC = \
            vcph7/usbd_desc.c \
            vcph7/usbd_conf.c \
            vcph7/usbd_cdc_interface.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            target/system_stm32h7xx.c \
            drivers/system_stm32h7xx.c \
            drivers/serial_uart_stm32h7xx.c \
            drivers/serial_uart_hal.c
            #drivers/accgyro/accgyro_mpu.c \
            #drivers/adc_stm32h7xx.c \
            #drivers/audio_stm32h7xx.c \
            #drivers/bus_i2c_hal.c \
            #drivers/dma_stm32h7xx.c \
            #drivers/light_ws2811strip_hal.c \
            #drivers/transponder_ir_io_hal.c \
            #drivers/bus_spi_ll.c \
            #drivers/pwm_output_dshot_hal.c \
            #drivers/timer_hal.c \
            #drivers/timer_stm32h7xx.c \

MCU_EXCLUDES = \
            drivers/bus_i2c.c \
            drivers/timer.c \
            drivers/serial_uart.c

#MSC_SRC = \
#            drivers/usb_msc_h7xx.c \
#            msc/usbd_storage.c

#ifneq ($(filter SDCARD_SDIO,$(FEATURES)),)
#MCU_COMMON_SRC += \
#            drivers/sdio_h7xx.c
#MSC_SRC += \
#            msc/usbd_storage_sdio.c
#endif

#ifneq ($(filter SDCARD_SPI,$(FEATURES)),)
#MSC_SRC += \
#            msc/usbd_storage_sd_spi.c
#endif

#ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
#MSC_SRC += \
#            msc/usbd_storage_emfat.c \
#            msc/emfat.c \
#            msc/emfat_file.c
#endif

DSP_LIB := $(ROOT)/lib/main/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM7
