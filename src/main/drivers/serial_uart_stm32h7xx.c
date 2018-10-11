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

/*
 * jflyper - Refactoring, cleanup and made pin-configurable
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#ifdef USE_DMA
static void handleUsartTxDma(uartPort_t *s);
#endif

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .reg = USART1,
#ifdef USE_DMA
        .DMAChannel = DMA_CHANNEL_4,
        .rxDMAStream = DMA2_Stream5,
        .txDMAStream = DMA2_Stream7,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA10), GPIO_AF7_USART1 },
            { DEFIO_TAG_E(PB7),  GPIO_AF4_USART1 },
            { DEFIO_TAG_E(PB15), GPIO_AF4_USART1 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA9),  GPIO_AF7_USART1 },
            { DEFIO_TAG_E(PB6),  GPIO_AF4_USART1 },
            { DEFIO_TAG_E(PB14), GPIO_AF4_USART1 },
        },
        .rcc_apb2 = RCC_APB2(USART1),
        //.txIrq = DMA2_ST7_HANDLER,
        .rxIrq = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1
    },
#endif

#ifdef USE_UART2
    {
        .device = UARTDEV_2,
        .reg = USART2,
#ifdef USE_DMA
        .DMAChannel = DMA_CHANNEL_4,
        .rxDMAStream = DMA1_Stream5,
        .txDMAStream = DMA1_Stream6,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA3), GPIO_AF7_USART2 },
            { DEFIO_TAG_E(PD6), GPIO_AF7_USART2 }
        },
        .txPins = {
            { DEFIO_TAG_E(PA2), GPIO_AF7_USART2 },
            { DEFIO_TAG_E(PD5), GPIO_AF7_USART2 }
        },
        .rcc_apb1 = RCC_APB1L(USART2),
        //.txIrq = DMA1_ST6_HANDLER,
        .rxIrq = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2
    },
#endif

#ifdef USE_UART3
    {
        .device = UARTDEV_3,
        .reg = USART3,
#ifdef USE_DMA
        .DMAChannel = DMA_CHANNEL_4,
        .rxDMAStream = DMA1_Stream1,
        .txDMAStream = DMA1_Stream3,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PB11), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PC11), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PD9), GPIO_AF7_USART3 }
        },
        .txPins = {
            { DEFIO_TAG_E(PB10), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PC10), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PD8), GPIO_AF7_USART3 }
        },
        .rcc_apb1 = RCC_APB1L(USART3),
        //.txIrq = DMA1_ST3_HANDLER,
        .rxIrq = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3
    },
#endif

#ifdef USE_UART4
    {
        .device = UARTDEV_4,
        .reg = UART4,
#ifdef USE_DMA
        .DMAChannel = DMA_CHANNEL_4,
        .rxDMAStream = DMA1_Stream2,
        .txDMAStream = DMA1_Stream4,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA1),  GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PA11), GPIO_AF6_UART4 },
            { DEFIO_TAG_E(PB8),  GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PC11), GPIO_AF8_UART4 },         
            { DEFIO_TAG_E(PD0),  GPIO_AF8_UART4 }
        },
        .txPins = {
            { DEFIO_TAG_E(PA0),  GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PA12), GPIO_AF6_UART4 },
            { DEFIO_TAG_E(PB9),  GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PC10), GPIO_AF8_UART4 },         
            { DEFIO_TAG_E(PD1),  GPIO_AF8_UART4 }
        },
        .rcc_apb1 = RCC_APB1L(UART4),
        //.txIrq = DMA1_ST4_HANDLER,
        .rxIrq = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4
    },
#endif

#ifdef USE_UART5
    {
        .device = UARTDEV_5,
        .reg = UART5,
#ifdef USE_DMA
        .DMAChannel = DMA_CHANNEL_4,
        .rxDMAStream = DMA1_Stream0,
        .txDMAStream = DMA1_Stream7,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PB5),  GPIO_AF14_UART5 },
            { DEFIO_TAG_E(PB12), GPIO_AF14_UART5 },
            { DEFIO_TAG_E(PD2),  GPIO_AF8_UART5 },
        },
        .txPins = {
            { DEFIO_TAG_E(PB6),  GPIO_AF14_UART5 },
            { DEFIO_TAG_E(PB13), GPIO_AF14_UART5 },
            { DEFIO_TAG_E(PC12), GPIO_AF8_UART5 },
        },
        .rcc_apb1 = RCC_APB1L(UART5),
        //.txIrq = DMA1_ST7_HANDLER,
        .rxIrq = UART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART5
    },
#endif

#ifdef USE_UART6
    {
        .device = UARTDEV_6,
        .reg = USART6,
#ifdef USE_DMA
        .DMAChannel = DMA_CHANNEL_5,
        .rxDMAStream = DMA2_Stream1,
        .txDMAStream = DMA2_Stream6,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PC7), GPIO_AF7_USART6  },
            { DEFIO_TAG_E(PG9), GPIO_AF7_USART6 }
        },
        .txPins = {
            { DEFIO_TAG_E(PC6), GPIO_AF7_USART6 },
            { DEFIO_TAG_E(PG14), GPIO_AF7_USART6 }
        },
        .rcc_apb2 = RCC_APB2(USART6),
        //.txIrq = DMA2_ST6_HANDLER,
        .rxIrq = USART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6
    },
#endif

#ifdef USE_UART7
    {
        .device = UARTDEV_7,
        .reg = UART7,
#ifdef USE_DMA
        .DMAChannel = DMA_CHANNEL_5,
        .rxDMAStream = DMA1_Stream3,
        .txDMAStream = DMA1_Stream1,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA8), GPIO_AF11_UART7 },
            { DEFIO_TAG_E(PB3), GPIO_AF11_UART7 },
            { DEFIO_TAG_E(PE7), GPIO_AF7_UART7 },
            { DEFIO_TAG_E(PF6), GPIO_AF7_UART7 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA15), GPIO_AF11_UART7 },
            { DEFIO_TAG_E(PB4),  GPIO_AF11_UART7 },
            { DEFIO_TAG_E(PE8),  GPIO_AF7_UART7 },
            { DEFIO_TAG_E(PF7),  GPIO_AF7_UART7 },
        },
        .rcc_apb1 = RCC_APB1L(UART7),
        //.txIrq = DMA1_ST1_HANDLER,
        .rxIrq = UART7_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART7_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART7
    },
#endif

#ifdef USE_UART8
    {
        .device = UARTDEV_8,
        .reg = UART8,
#ifdef USE_DMA
        .DMAChannel = DMA_CHANNEL_5,
        .rxDMAStream = DMA1_Stream6,
        .txDMAStream = DMA1_Stream0,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PE0), GPIO_AF8_UART8 }
        },
        .txPins = {
            { DEFIO_TAG_E(PE1), GPIO_AF8_UART8 }
        },
        .rcc_apb1 = RCC_APB1L(UART8),
        //.txIrq = DMA1_ST0_HANDLER,
        .rxIrq = UART8_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART8_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART8
    },
#endif
};

void uartIrqHandler(uartPort_t *s)
{
    UART_HandleTypeDef *huart = &s->Handle;
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET)) {
        uint8_t rbyte = (uint8_t)(huart->Instance->RDR & (uint8_t) 0xff);

        if (s->port.rxCallback) {
            s->port.rxCallback(rbyte, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = rbyte;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE));

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    }

    /* UART parity error interrupt occurred -------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    }

    /* UART in mode Transmitter ------------------------------------------------*/
    if (
#ifdef USE_DMA
        !s->txDMAStream &&
#endif
        (__HAL_UART_GET_IT(huart, UART_IT_TXE) != RESET)) {
        /* Check that a Tx process is ongoing */
        if (huart->gState != HAL_UART_STATE_BUSY_TX) {
            if (s->port.txBufferTail == s->port.txBufferHead) {
                huart->TxXferCount = 0;
                /* Disable the UART Transmit Data Register Empty Interrupt */
                CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
            } else {
                if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE)) {
                    huart->Instance->TDR = (((uint16_t) s->port.txBuffer[s->port.txBufferTail]) & (uint16_t) 0x01FFU);
                } else {
                    huart->Instance->TDR = (uint8_t)(s->port.txBuffer[s->port.txBufferTail]);
                }
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            }
        }
    }

    /* UART in mode Transmitter (transmission end) -----------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET)) {
        HAL_UART_IRQHandler(huart);
#ifdef USE_DMA
        if (s->txDMAStream) {
            handleUsartTxDma(s);
        }
#endif
    }
}

#ifdef USE_DMA
static void handleUsartTxDma(uartPort_t *s)
{
    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
    {
        s->txDMAEmpty = true;
    }
}

void dmaIRQHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
    HAL_DMA_IRQHandler(&s->txDMAHandle);
}
#endif

// XXX Should serialUART be consolidated?

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uartdev = uartDevmap[device];
    if (!uartdev) {
        return NULL;
    }

    uartPort_t *s = &(uartdev->port);

    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uartdev->rxBuffer;
    s->port.txBuffer = uartdev->txBuffer;
    s->port.rxBufferSize = ARRAYLEN(uartdev->rxBuffer);
    s->port.txBufferSize = ARRAYLEN(uartdev->txBuffer);

    const uartHardware_t *hardware = uartdev->hardware;

    s->USARTx = hardware->reg;

#ifdef USE_DMA
    if (hardware->rxDMAStream) {
        s->rxDMAChannel = hardware->DMAChannel;
        s->rxDMAStream = hardware->rxDMAStream;
    }

    if (hardware->txDMAStream) {
        s->txDMAChannel = hardware->DMAChannel;
        s->txDMAStream = hardware->txDMAStream;

        // DMA TX Interrupt
        dmaInit(hardware->txIrq, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        dmaSetHandler(hardware->txIrq, dmaIRQHandler, hardware->txPriority, (uint32_t)uartdev);
    }

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->TDR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->RDR;
#endif

    s->Handle.Instance = hardware->reg;

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);

    if ((options & SERIAL_BIDIR) && txIO) {
        ioConfig_t ioCfg = IO_CONFIG(
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_MODE_AF_PP : GPIO_MODE_AF_OD,
            GPIO_SPEED_FREQ_HIGH,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_PULLDOWN : GPIO_PULLUP
        );

        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(txIO, ioCfg, uartdev->tx.af);
    } else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uartdev->tx.af);
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP, uartdev->rx.af);
        }
    }

#ifdef USE_DMA
    if (s->rxDMAChannel) {
#endif
        HAL_NVIC_SetPriority(hardware->rxIrq, NVIC_PRIORITY_BASE(hardware->rxPriority), NVIC_PRIORITY_SUB(hardware->rxPriority));
        HAL_NVIC_EnableIRQ(hardware->rxIrq);
#ifdef USE_DMA
    }
#endif
    return s;
}
#endif // USE_UART
