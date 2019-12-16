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

#define USE_DISPLAYPORT_MSP_VENDOR_SPECIFIC

#define TARGET_BOARD_IDENTIFIER "AB7M"
#define USBD_PRODUCT_STRING     "Airbot-F7-MIS"

#define USE_TARGET_CONFIG

#define LED0_PIN                PA3

#define USE_BEEPER
#define BEEPER_PIN              PB2
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON

// *************** SPI and I2C Buses **********************

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

//#define USE_SPI_DEVICE_3
//#define SPI3_SCK_PIN            PB3
//#define SPI3_MISO_PIN           PB4
//#define SPI3_MOSI_PIN           PB5

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9 

// *************** Gyro & ACC **********************

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500

#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PC4
#define GYRO_1_ALIGN            CW90_DEG
#define GYRO_1_EXTI_PIN         PA4

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500

// *************** FLASH *****************************

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_SPI_INSTANCE      SPI2
#define FLASH_CS_PIN            PB12
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// *************** UART *****************************
#define USE_VCP
#define USE_USB_DETECT

#define USE_UART1
#define UART1_TX_PIN            PA9   // P1-6

#define USE_UART2
#define UART2_TX_PIN            PA2   // 4-in-1 ESC TLM

#define USE_UART3
#define UART3_RX_PIN            PC11  // P1-4 Serial OSD
#define UART3_TX_PIN            PC10  // P1-3 Serial OSD

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2   // BT-RX
#define UART5_TX_PIN            PC12  // BT-TX

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1

#define SERIAL_PORT_COUNT       8

// *************** Additional Sensors **************

#define USE_BARO
#define USE_BARO_BMP280
#define I2C_DEVICE              I2CDEV_1

// *************** ADC *****************************
#define USE_ADC
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1 // PC1 for 4-in-1 socket, PC2 for pad
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 179

// *************** Feature control *****************

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL | FEATURE_AIRMODE)

#define USE_ESCSERIAL

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS             (TIM_N(1)|TIM_N(2)|TIM_N(4)|TIM_N(8))
