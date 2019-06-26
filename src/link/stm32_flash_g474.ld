/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32G474 (Category 3) device with
**                512KByte FLASH and 96KByte SRAM and 32KByte CCM SRAM
**
*****************************************************************************
*/

/* Specify the memory areas. */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 4K
    FLASH1 (rx)       : ORIGIN = 0x08001000, LENGTH = 492K
    FLASH_CONFIG (r)  : ORIGIN = 0x0807C000, LENGTH = 16K

    SYSTEM_MEMORY (r) : ORIGIN = 0x1FFF0000, LENGTH = 64K

    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 96K
    CCM (xrw)         : ORIGIN = 0x20018000, LENGTH = 32K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", CCM)
REGION_ALIAS("FASTRAM", CCM)
REGION_ALIAS("VECTAB", RAM) /* CCM? */

INCLUDE "stm32_flash_split_g4.ld"
