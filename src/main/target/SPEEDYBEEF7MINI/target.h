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

#define TARGET_BOARD_IDENTIFIER "SB7M"
#define TARGET_MANUFACTURER_IDENTIFIER "SPBE"

#define LED0_PIN                PB12
#define USE_BEEPER
#define BEEPER_PIN              PB10
#define BEEPER_INVERTED

#define USE_EXTI

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

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
#define I2C1_SCK_PIN            PB8
#define I2C1_SDA_PIN            PB9
#define I2C_DEVICE              (I2CDEV_1)

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500         // ICM20602
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define GYRO_1_SPI_INSTANCE     SPI3
#define GYRO_1_CS_PIN           PD2
#define GYRO_1_EXTI_PIN         PA4

#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_RX_PIN            PA3  // TLM

#define USE_UART3
#define UART3_TX_PIN            PC10
#define UART3_RX_PIN            PC11

#define USE_UART4
#define UART4_TX_PIN            PA0  // BT
#define UART4_RX_PIN            PA1  // BT

#define USE_UART5
#define UART5_TX_PIN            PC12 // SMARTAUDIO

#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       9    // VCP, USART1, USART3, USART4, USART6, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA15 // LED_STRIP, needs testing

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PC15

#define USE_BARO
#define USE_BARO_BMP280

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC1_DMA_OPT            0  // DMA 2 Stream 4 Channel 0

#define VBAT_ADC_PIN            PC0  // 11:1 (10K + 1K) divider
#define EXTERNAL1_ADC_PIN       PC1  // CURRENT_SOCKET
#define CURRENT_METER_ADC_PIN   PC2  // CURRENT_ADC

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define DEFAULT_FEATURES        (FEATURE_RX_SERIAL | FEATURE_LED_STRIP | FEATURE_OSD )

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC (0xffff)
#define TARGET_IO_PORTD BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) )
