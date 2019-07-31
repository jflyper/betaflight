/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/memprot.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

void enablePeripherialClocks(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_CORDIC_CLK_ENABLE();
    __HAL_RCC_FMAC_CLK_ENABLE();
    __HAL_RCC_FLASH_CLK_ENABLE();
    __HAL_RCC_CRC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC345_CLK_ENABLE();
    //__HAL_RCC_DAC1_CLK_ENABLE();
    //__HAL_RCC_DAC2_CLK_ENABLE();
    //__HAL_RCC_DAC3_CLK_ENABLE();
    //__HAL_RCC_DAC4_CLK_ENABLE();
    //__HAL_RCC_AES_CLK_ENABLE();
    //__HAL_RCC_RNG_CLK_ENABLE();
    //__HAL_RCC_FMC_CLK_ENABLE();
    //__HAL_RCC_QSPI_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();
    __HAL_RCC_CRS_CLK_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    __HAL_RCC_WWDG_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    //__HAL_RCC_USB_CLK_ENABLE();
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();
    //__HAL_RCC_LPTIM1_CLK_ENABLE();
    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_I2C4_CLK_ENABLE();
    //__HAL_RCC_UCPD1_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_SPI4_CLK_ENABLE();
    __HAL_RCC_TIM15_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();
    __HAL_RCC_TIM20_CLK_ENABLE();
    //__HAL_RCC_SAI1_CLK_ENABLE();
    //__HAL_RCC_HRTIM1_CLK_ENABLE();
}

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_CSR_SFTRSTF)
        return true;
    else
        return false;
}

void systemInit(void)
{
    memProtReset();
    memProtConfigure(mpuRegions, mpuRegionCount);

    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->RSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCC->CSR;

    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
    //extern void *isr_vector_table_base;
    //NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
    //__HAL_RCC_USB_OTG_FS_CLK_DISABLE;

    //RCC_ClearFlag();

    enablePeripherialClocks();

    // Init cycle counter
    cycleCounterInit();

    // SysTick is updated whenever HAL_RCC_ClockConfig is called.
}

void systemReset(void)
{
    // SCB_DisableDCache();
    // SCB_DisableICache();

    __disable_irq();
    NVIC_SystemReset();
}

void forcedSystemResetWithoutDisablingCaches(void)
{
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_FORCED);

    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    switch (requestType) {
#if defined(USE_FLASH_BOOT_LOADER)
    case BOATLOADER_REQUEST_FLASH:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_FLASH_BOOTLOADER_REQUEST);

        break;
#endif
    case BOOTLOADER_REQUEST_ROM:
    default:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);

        break;
    }

    __disable_irq();
    NVIC_SystemReset();
}

static uint32_t bootloaderRequest;

void systemCheckResetReason(void)
{
    bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (bootloaderRequest) {
#if defined(USE_FLASH_BOOT_LOADER)
    case BOATLOADER_REQUEST_FLASH:
#endif
    case RESET_BOOTLOADER_REQUEST_ROM:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_POST);
        break;

    case RESET_MSC_REQUEST:
        // RESET_REASON will be reset by MSC
        return;

    case RESET_FORCED:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        return;

    case RESET_NONE:
        if (!(RCC->CSR & RCC_CSR_SFTRSTF)) {
            // Direct hard reset case
            return;
        }
        // Soft reset; boot loader may have been active with BOOT pin pulled high.
        FALLTHROUGH;

    case RESET_BOOTLOADER_POST:
        // Boot loader activity magically prevents SysTick from interrupting.
        // Issue a soft reset to prevent the condition.
        forcedSystemResetWithoutDisablingCaches(); // observed that disabling dcache after cold boot with BOOT pin high causes segfault.
    }

    void (*SysMemBootJump)(void);
    __SYSCFG_CLK_ENABLE();

#define SYSTEM_BOOTLOADER_VEC 0x1fff0000

    uint32_t p =  (*((uint32_t *)SYSTEM_BOOTLOADER_VEC));
    __set_MSP(p); //Set the main stack pointer to its defualt values
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(SYSTEM_BOOTLOADER_VEC + 4))); // Point the PC to the System Memory reset vector (+4)
    SysMemBootJump();
    while (1);
}

// Nucleo-G474RE board seems to come with software BOOT0 enabled.
// Call this function once from init() to honor PB8-BOOT0 pin status for boot loader invocation.
void systemBOOT0PinBootLoaderEnable(void)
{
    FLASH_OBProgramInitTypeDef OBInit;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    HAL_FLASH_OB_Unlock();

    HAL_FLASHEx_OBGetConfig(&OBInit);

    if ((OBInit.USERConfig & (OB_BOOT0_FROM_PIN|OB_BOOT1_SYSTEM)) != (OB_BOOT0_FROM_PIN|OB_BOOT1_SYSTEM)) {
        OBInit.OptionType = OPTIONBYTE_USER;
        OBInit.USERType = OB_USER_nSWBOOT0|OB_USER_nBOOT1;
        OBInit.USERConfig = OB_BOOT0_FROM_PIN|OB_BOOT1_SYSTEM;
        HAL_FLASHEx_OBProgram(&OBInit);

        HAL_FLASH_OB_Launch();
    }

    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}
