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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "io/serial.h"
#include "io/displayport_msp.h"

#include "pg/pg.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "pg/rx.h"

#include "sensors/esc_sensor.h"

#include "config_helper.h"

static targetSerialPortFunction_t targetSerialPortFunction[] = {
    { SERIAL_PORT_USART1, FUNCTION_RX_SERIAL },
    { SERIAL_PORT_USART2, FUNCTION_ESC_SENSOR },
    { SERIAL_PORT_USART3, FUNCTION_MSP },         // MSP DisplayPort for external OSD
    { SERIAL_PORT_UART5,  FUNCTION_MSP },         // BT MSP
};

void targetConfiguration(void)
{
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));

    // ESC sensor is on TX2 and serial RX is on TX1
    escSensorConfigMutable()->halfDuplex = true;
    rxConfigMutable()->halfDuplex = true;

    // Configure MSP displayport output device
    displayPortProfileMspMutable()->displayPortSerial = SERIAL_PORT_USART3;

    // Configure BT MSP module control
    pinioConfigMutable()->ioTag[0] = IO_TAG(PC14);
    pinioConfigMutable()->config[0] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioBoxConfigMutable()->permanentId[0] = BOXARM;

    // Configure VTX power control
    pinioConfigMutable()->ioTag[1] = IO_TAG(PA10);
    pinioConfigMutable()->config[1] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioBoxConfigMutable()->permanentId[1] = BOXUSER1;
}
#endif
