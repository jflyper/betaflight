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

#define TARGET_BOARD_IDENTIFIER "H743"
#define USBD_PRODUCT_STRING "Nucleo-H743"

#define USE_TARGET_CONFIG

#define LED0_PIN                PB0
#define LED1_PIN                PB7
#define LED2_PIN                PB14

//#define USE_BUTTONS
#define	BUTTON_A_PIN            PC13

#undef USE_BEEPER

#define USE_UART

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PD6
#define UART2_TX_PIN            PD5

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_UART7
#define UART7_RX_PIN            PE7
#define UART7_TX_PIN            PE8

#define USE_UART8
#define UART8_RX_PIN            PE0
#define UART8_TX_PIN            PE1

#define USE_VCP

#define SERIAL_PORT_COUNT       9

#define USE_SPI

#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DEVICE_6

#define USE_GYRO
#define USE_MULTI_GYRO
#define USE_ACC

#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU9250
#define USE_ACC_SPI_MPU9250

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#undef USE_BRUSHED_ESC_AUTODETECT  // Detect if brushed motors are connected and set defaults appropriately to avoid motors spinning on boot
#undef USE_GYRO_REGISTER_DUMP  // Adds gyroregisters command to cli to dump configured register values
#undef USE_TIMER
#undef USE_MOTOR

#undef USE_PWM_OUTPUT
#undef USE_EXTI
#undef USE_PPM
#undef USE_PWM
#undef USE_UART
#define USE_SERIAL_RX
#undef USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define USE_SERIALRX_FPORT
#undef USE_SERIALRX_JETIEXBUS
#define USE_TELEMETRY

#undef USE_LED_STRIP

#undef USE_ACRO_TRAINER
#undef USE_BLACKBOX
#undef USE_RUNAWAY_TAKEOFF     // Runaway Takeoff Prevention (anti-taz)
#undef USE_SERVOS
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_LTM
#undef USE_TELEMETRY_SMARTPORT

#undef USE_CMS
#undef USE_OSD
#undef USE_VTX_COMMON
#undef USE_VTX_CONTROL
#undef USE_VTX_TRAMP
#undef USE_VTX_SMARTAUDIO
#undef USE_CAMERA_CONTROL
#undef USE_GPS
#undef USE_GPS_RESCUE

#undef USE_I2C_OLED_DISPLAY
#undef USE_MSP_DISPLAYPORT
#undef USE_OSD_OVER_MSP_DISPLAYPORT

#undef USE_DMA
#undef USE_ADC
#undef USE_DSHOT
#undef USE_GYRO_DATA_ANALYSE
#undef USE_ADC_INTERNAL
#undef USE_USB_CDC_HID
#undef USE_USB_MSC
#undef USE_OVERCLOCK
#undef USE_RTC_TIME
#undef USE_RCDEVICE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 13
#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(9) )
