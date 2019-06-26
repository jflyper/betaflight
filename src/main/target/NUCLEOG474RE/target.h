/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "N474" // STM32 Nucleo F446RE
#define USBD_PRODUCT_STRING     "Nucleo-G474RE"

#define LED0_PIN                PB8  // BOOT0, can be used as plain output shortly after reset
#define LED0_INVERTED

#define USE_BUTTONS
#define BUTTON_A_PIN            PC13 // Onboard user Button (B1)
#define BUTTON_A_PIN_INVERTED        // Active high
#define BUTTON_B_PIN            PC13 // Same as BUTTON_A to trigger EEPROM clear
#define BUTTON_B_PIN_INVERTED        // Active high

#define USE_BEEPER
#define BEEPER_PIN              NONE
#define BEEPER_INVERTED             // Positive logic, supress OD

#define USE_EXTI

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5 // D13
#define SPI1_MISO_PIN           PA6 // D12
#define SPI1_MOSI_PIN           PA7 // D11

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C1_SCL                NONE // PA15, PB6, PB8 (PB8 is BOOT0)
#define I2C1_SDA                NONE // PB7, PB9 (PA14 is SWCLK)

#define USE_I2C_DEVICE_2
#define I2C2_SCL                NONE // PA9
#define I2C2_SDA                NONE // PA10

#define USE_I2C_DEVICE_3
#define I2C3_SCL                NONE // PA10, PC8
#define I2C3_SDA                NONE // PB5, PC9, PC11

#define USE_I2C_DEVICE_4
#define I2C4_SCL                NONE // PB6, PC6
#define I2C4_SDA                NONE // PB7, PC7

#define I2C_DEVICE              (I2CDEV_1)

#define USE_GYRO
#define USE_FAKE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU9250

#define USE_ACC
#define USE_FAKE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU9250

#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PB6  // D10

//#define USE_GYRO_EXTI
//#define GYRO_1_EXTI_PIN PC13
//#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_BARO
//#define USE_BARO_BMP085
//#define USE_BARO_BMP280
//#define USE_BARO_BMP388
//#define USE_BARO_LPS
//#define USE_BARO_MS5611
//#define USE_BARO_QMP6988
#define USE_BARO_SPI_BMP280
#define USE_BARO_SPI_BMP388
#define USE_BARO_SPI_LPS
#define USE_BARO_SPI_MS5611

#define USE_MAG
//#define USE_MAG_DATA_READY_SIGNAL
////#define USE_MAG_AK8963
//#define USE_MAG_AK8975
//#define USE_MAG_HMC5883
//#define USE_MAG_LIS3MDL
//#define USE_MAG_QMC5883
#define USE_MAG_SPI_AK8963
#define USE_MAG_SPI_HMC5883

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB1

#define USE_FLASH_CHIP
#define FLASH_SPI_INSTANCE      SPI3
#define FLASH_CS_PIN            PC4

#define USE_FLASH_M25P16
#define USE_FLASH_W25M512
#define USE_FLASH_W25N01G

#define USE_FLASHFS
#define USE_BLACKBOX
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9  // PA9, PB6, PC4
#define UART1_RX_PIN            PA10 // PA10, PB7, PC5
#define UART1_TX_DMA_STREAM     NULL // DMA1_Channel1
#define UART1_RX_DMA_STREAM     NULL
#define UART1_TX_DMA_OPT        (-1)
#define UART1_RX_DMA_OPT        (-1)

#define USE_UART2
// Default configuration of Nucleo-G747RE connects PA2 and PA3 to ST-LINK virtual com port
// on Nucleo-G747RE default configuration
#define UART2_TX_PIN            PA2 // PA2, PA14, PB4
#define UART2_RX_PIN            PA3 // PA3, PA15, PB5
#define UART2_TX_DMA_STREAM     NULL
#define UART2_RX_DMA_STREAM     NULL
#define UART2_TX_DMA_OPT        (-1)
#define UART2_RX_DMA_OPT        (-1)

#define USE_UART3
#define UART3_TX_PIN            PB11 // PB9, PB11, PC11
#define UART3_RX_PIN            PB10 // PB8, PB10, PC10
#define UART3_TX_DMA_STREAM     NULL
#define UART3_RX_DMA_STREAM     NULL
#define UART3_TX_DMA_OPT        (-1)
#define UART3_RX_DMA_OPT        (-1)

#define USE_UART4
#define UART4_TX_PIN            NONE // PC10
#define UART4_RX_PIN            PB11 // PC11
#define UART4_TX_DMA_STREAM     NULL
#define UART4_RX_DMA_STREAM     NULL
#define UART4_TX_DMA_OPT        (-1)
#define UART4_RX_DMA_OPT        (-1)

#define USE_UART5
#define UART5_TX_PIN            NONE // PC12
#define UART5_RX_PIN            NONE // PD2
#define UART5_TX_DMA_STREAM     NULL
#define UART5_RX_DMA_STREAM     NULL
#define UART5_TX_DMA_OPT        (-1)
#define UART5_RX_DMA_OPT        (-1)

// LPUART1 as UART6
// Has RX/TX pins swapped against UART3 on PB10 and PB11
#define USE_UART6
#define UART6_TX_PIN            NONE // PA2, PB11, PC1
#define UART6_RX_PIN            NONE // PA3, PB10, PC0
#define UART6_TX_DMA_STREAM     NULL // DMA2_Channel1
#define UART6_RX_DMA_STREAM     NULL
#define UART6_TX_DMA_OPT        (-1)
#define UART6_RX_DMA_OPT        (-1)

#define USE_SOFTSERIAL1
#define SOFTSERIAL1_TX_PIN      NONE
#define SOFTSERIAL1_RX_PIN      NONE

#define USE_SOFTSERIAL2
#define SOFTSERIAL2_TX_PIN      NONE
#define SOFTSERIAL2_RX_PIN      NONE

#define SERIAL_PORT_COUNT       9  // 1 (VCP) + 6 (UART) + 2 (SOFTSERIAL)

//#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8  // (HARDARE=0,PPM)

#define USE_ADC
#define USE_ADC_INTERNAL
#define ADC1_INSTANCE           ADC1
#define ADC1_DMA_OPT            12  // DMA 2 Channel 4 (compat default)
#define ADC1_DMA_CHANNEL        DMA2_Channel4
#define ADC2_INSTANCE           ADC2
//#define ADC2_DMA_OPT          11  // DMA 2 Channel 3 (compat default)
#define ADC3_INSTANCE           ADC3
#define VBAT_ADC_PIN            NONE // PC2
#define CURRENT_METER_ADC_PIN   NONE // PC3
#define RSSI_ADC_PIN            NONE
#define EXTERNAL1_ADC_PIN       NONE

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN       NONE
#define RANGEFINDER_HCSR04_ECHO_PIN          NONE

#define USE_ESCSERIAL

#undef USE_RCDEVICE
#undef USE_CAMERA_CONTROL

#define DEFAULT_FEATURES        (FEATURE_TELEMETRY|FEATURE_OSD)

#define MAX_SUPPORTED_MOTORS    12

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13))) // Less SWCLK and SWDIO
#define TARGET_IO_PORTB (0xffff)
#define TARGET_IO_PORTC (0xffff)
#define TARGET_IO_PORTD BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 15
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(15) | TIM_N(16) | TIM_N(17) | TIM_N(20))

//#define USE_RX_SPI
//#define RX_SPI_INSTANCE         SPI1
// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
//#define RX_CE_PIN               NONE // D9
//#define RX_NSS_PIN              NONE // D10

//#define USE_RX_NRF24
//#define USE_RX_CX10
//#define USE_RX_H8_3D
//#define USE_RX_INAV
//#define USE_RX_SYMA
//#define USE_RX_V202
//#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_NRF24_H8_3D
