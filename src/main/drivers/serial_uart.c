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
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/inverter.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

static void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.baudRate = baudRate;
    uartReconfigure(uartPort);
}

static void uartSetMode(serialPort_t *instance, portMode_e mode)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.mode = mode;
    uartReconfigure(uartPort);
}

void uartTryStartTxDMA(uartPort_t *s)
{
    // uartTryStartTxDMA must be protected, since it is called from
    // uartWrite and handleUsartTxDma (an ISR).

    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
#ifdef STM32F4

        // This is STDPERIPHS case

        if (IS_DMA_ENABLED(s->txDMAInstance)) {
            // DMA is already in progress
            return;
        }

        // For F4 (and F1), there are cases that NDTR (CNDTR for F1) is non-zero upon TC interrupt.
        // We couldn't find out the root cause, so mask the case here.

        if (DMA_GetCurrDataCounter((DMA_INSTANCE_TYPE *)s->txDMAInstance)) {
        // if (s->txDMAInstance->NDTR) {
            // Possible premature TC case.
            goto reenable;
        }

        // DMA_Cmd(s->txDMAInstance, DISABLE); // XXX It's already disabled.

        if (s->port.txBufferHead == s->port.txBufferTail) {
            // No more data to transmit.
            s->txDMAEmpty = true;
            return;
        }

        // Start a new transaction.

        DMA_MemoryTargetConfig((DMA_INSTANCE_TYPE *)s->txDMAInstance, (uint32_t)&s->port.txBuffer[s->port.txBufferTail], DMA_Memory_0);
        //s->txDMAInstance->M0AR = (uint32_t)&s->port.txBuffer[s->port.txBufferTail];
        if (s->port.txBufferHead > s->port.txBufferTail) {
            DMA_SetCurrDataCounter((DMA_INSTANCE_TYPE *)s->txDMAInstance, s->port.txBufferHead - s->port.txBufferTail);
            s->port.txBufferTail = s->port.txBufferHead;
        }
        else {
            DMA_SetCurrDataCounter((DMA_INSTANCE_TYPE *)s->txDMAInstance, s->port.txBufferSize - s->port.txBufferTail);
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;

    reenable:
        DMA_Cmd((DMA_INSTANCE_TYPE *)s->txDMAInstance, ENABLE);
#else
        if (IS_DMA_ENABLED(s->txDMAInstance)) {
            // DMA is already in progress
            return;
        }

        // For F1 (and F4), there are cases that CNDTR (NDTR for F4) is non-zero upon TC interrupt.
        // We couldn't find out the root cause, so mask the case here.
        // F3 is not confirmed to be vulnerable, but not excluded as a safety.

        if (DMA_GetCurrDataCounter((DMA_INSTANCE_TYPE *)s->txDMAInstance)) {
            // Possible premature TC case.
            goto reenable;
        }

        if (s->port.txBufferHead == s->port.txBufferTail) {
            // No more data to transmit.
            s->txDMAEmpty = true;
            return;
        }

        // Start a new transaction.

        // s->txDMAInstance->CMAR = (uint32_t)&s->port.txBuffer[s->port.txBufferTail];
        DMAx_SetMemoryAddress(s->txDMAInstance, (uint32_t)&s->port.txBuffer[s->port.txBufferTail]);

        if (s->port.txBufferHead > s->port.txBufferTail) {
            DMA_SetCurrDataCounter((DMA_INSTANCE_TYPE *)s->txDMAInstance, s->port.txBufferHead - s->port.txBufferTail);
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            DMA_SetCurrDataCounter((DMA_INSTANCE_TYPE *)s->txDMAInstance, s->port.txBufferSize - s->port.txBufferTail);
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;

    reenable:
        DMA_Cmd((DMA_INSTANCE_TYPE *)s->txDMAInstance, ENABLE);
#endif
    }
}

static uint32_t uartTotalRxBytesWaiting(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t*)instance;
#ifdef STM32F4
    if (s->rxDMAInstance) {
        uint32_t rxDMAHead = DMA_GetCurrDataCounter((DMA_INSTANCE_TYPE *)s->rxDMAInstance);
#else
    if (s->rxDMAInstance) {
        uint32_t rxDMAHead = DMA_GetCurrDataCounter((DMA_INSTANCE_TYPE *)s->rxDMAInstance);
#endif
        if (rxDMAHead >= s->rxDMAPos) {
            return rxDMAHead - s->rxDMAPos;
        } else {
            return s->port.rxBufferSize + rxDMAHead - s->rxDMAPos;
        }
    }

    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        return s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        return s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
}

static uint32_t uartTotalTxBytesFree(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t*)instance;

    uint32_t bytesUsed;

    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }

    if (s->txDMAInstance) {
        /*
         * When we queue up a DMA request, we advance the Tx buffer tail before the transfer finishes, so we must add
         * the remaining size of that in-progress transfer here instead:
         */
#ifdef STM32F4
        bytesUsed += DMA_GetCurrDataCounter((DMA_INSTANCE_TYPE *)s->txDMAInstance);
#else
        bytesUsed += DMA_GetCurrDataCounter((DMA_INSTANCE_TYPE *)s->txDMAInstance);
#endif
        /*
         * If the Tx buffer is being written to very quickly, we might have advanced the head into the buffer
         * space occupied by the current DMA transfer. In that case the "bytesUsed" total will actually end up larger
         * than the total Tx buffer size, because we'll end up transmitting the same buffer region twice. (So we'll be
         * transmitting a garbage mixture of old and new bytes).
         *
         * Be kind to callers and pretend like our buffer can only ever be 100% full.
         */
        if (bytesUsed >= s->port.txBufferSize - 1) {
            return 0;
        }
    }

    return (s->port.txBufferSize - 1) - bytesUsed;
}

static bool isUartTransmitBufferEmpty(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t *)instance;
#ifdef STM32F4
    if (s->txDMAInstance)
#else
    if (s->txDMAInstance)
#endif
        return s->txDMAEmpty;
    else
        return s->port.txBufferTail == s->port.txBufferHead;
}

static uint8_t uartRead(serialPort_t *instance)
{
    uint8_t ch;
    uartPort_t *s = (uartPort_t *)instance;

    if (s->rxDMAInstance) {
        ch = s->port.rxBuffer[s->port.rxBufferSize - s->rxDMAPos];
        if (--s->rxDMAPos == 0)
            s->rxDMAPos = s->port.rxBufferSize;
    } else {
        ch = s->port.rxBuffer[s->port.rxBufferTail];
        if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferTail = 0;
        } else {
            s->port.rxBufferTail++;
        }
    }

    return ch;
}

static void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *s = (uartPort_t *)instance;
    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }

    if (s->txDMAInstance) {
        uartTryStartTxDMA(s);
    } else {
        USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
    }
}

const struct serialPortVTable uartVTable[] = {
    {
        .serialWrite = uartWrite,
        .serialTotalRxWaiting = uartTotalRxBytesWaiting,
        .serialTotalTxFree = uartTotalTxBytesFree,
        .serialRead = uartRead,
        .serialSetBaudRate = uartSetBaudRate,
        .isSerialTransmitBufferEmpty = isUartTransmitBufferEmpty,
        .setMode = uartSetMode,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
    }
};

#ifdef USE_UART1
// USART1 Rx/Tx IRQ Handler
void USART1_IRQHandler(void)
{
    uartPort_t *s = &(uartDevmap[UARTDEV_1]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART2
// USART2 Rx/Tx IRQ Handler
void USART2_IRQHandler(void)
{
    uartPort_t *s = &(uartDevmap[UARTDEV_2]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART3
// USART3 Rx/Tx IRQ Handler
void USART3_IRQHandler(void)
{
    uartPort_t *s = &(uartDevmap[UARTDEV_3]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART4
// UART4 Rx/Tx IRQ Handler
void UART4_IRQHandler(void)
{
    uartPort_t *s = &(uartDevmap[UARTDEV_4]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART5
// UART5 Rx/Tx IRQ Handler
void UART5_IRQHandler(void)
{
    uartPort_t *s = &(uartDevmap[UARTDEV_5]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART6
// USART6 Rx/Tx IRQ Handler
void USART6_IRQHandler(void)
{
    uartPort_t *s = &(uartDevmap[UARTDEV_6]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART7
// UART7 Rx/Tx IRQ Handler
void UART7_IRQHandler(void)
{
    uartPort_t *s = &(uartDevmap[UARTDEV_7]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART8
// UART8 Rx/Tx IRQ Handler
void UART8_IRQHandler(void)
{
    uartPort_t *s = &(uartDevmap[UARTDEV_8]->port);
    uartIrqHandler(s);
}
#endif
#endif
