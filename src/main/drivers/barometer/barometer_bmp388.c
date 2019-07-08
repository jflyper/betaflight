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
 *
 * BMP388 Driver author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"

#if defined(USE_BARO) && (defined(USE_BARO_BMP388) || defined(USE_BARO_SPI_BMP388))

#include "build/debug.h"

#include "common/maths.h"

#include "barometer.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/nvic.h"

#include "barometer_bmp388.h"


#define BMP388_I2C_ADDR                                 (0x76) // same as BMP280/BMP180
#define BMP388_DEFAULT_CHIP_ID                          (0x50) // from https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3_defs.h#L130

#define BMP388_CMD_REG                                  (0x7E)
#define BMP388_RESERVED_UPPER_REG                       (0x7D)
// everything between BMP388_RESERVED_UPPER_REG and BMP388_RESERVED_LOWER_REG is reserved.
#define BMP388_RESERVED_LOWER_REG                       (0x20)
#define BMP388_CONFIG_REG                               (0x1F)
#define BMP388_RESERVED_0x1E_REG                        (0x1E)
#define BMP388_ODR_REG                                  (0x1D)
#define BMP388_OSR_REG                                  (0x1C)
#define BMP388_PWR_CTRL_REG                             (0x1B)
#define BMP388_IF_CONFIG_REG                            (0x1A)
#define BMP388_INT_CTRL_REG                             (0x19)
#define BMP388_FIFO_CONFIG_2_REG                        (0x18)
#define BMP388_FIFO_CONFIG_1_REG                        (0x17)
#define BMP388_FIFO_WTM_1_REG                           (0x16)
#define BMP388_FIFO_WTM_0_REG                           (0x15)
#define BMP388_FIFO_DATA_REG                            (0x14)
#define BMP388_FIFO_LENGTH_1_REG                        (0x13)
#define BMP388_FIFO_LENGTH_0_REG                        (0x12)
#define BMP388_INT_STATUS_REG                           (0x11)
#define BMP388_EVENT_REG                                (0x10)
#define BMP388_SENSORTIME_3_REG                         (0x0F) // BME780 only
#define BMP388_SENSORTIME_2_REG                         (0x0E)
#define BMP388_SENSORTIME_1_REG                         (0x0D)
#define BMP388_SENSORTIME_0_REG                         (0x0C)
#define BMP388_RESERVED_0x0B_REG                        (0x0B)
#define BMP388_RESERVED_0x0A_REG                        (0x0A)

// see friendly register names below
#define BMP388_DATA_5_REG                               (0x09)
#define BMP388_DATA_4_REG                               (0x08)
#define BMP388_DATA_3_REG                               (0x07)
#define BMP388_DATA_2_REG                               (0x06)
#define BMP388_DATA_1_REG                               (0x05)
#define BMP388_DATA_0_REG                               (0x04)

#define BMP388_STATUS_REG                               (0x03)
#define BMP388_ERR_REG                                  (0x02)
#define BMP388_RESERVED_0x01_REG                        (0x01)
#define BMP388_CHIP_ID_REG                              (0x00)

// friendly register names, from datasheet 4.3.4
#define BMP388_PRESS_MSB_23_16_REG                      BMP388_DATA_2_REG
#define BMP388_PRESS_LSB_15_8_REG                       BMP388_DATA_1_REG
#define BMP388_PRESS_XLSB_7_0_REG                       BMP388_DATA_0_REG

// friendly register names, from datasheet 4.3.5
#define BMP388_TEMP_MSB_23_16_REG                       BMP388_DATA_5_REG
#define BMP388_TEMP_LSB_15_8_REG                        BMP388_DATA_4_REG
#define BMP388_TEMP_XLSB_7_0_REG                        BMP388_DATA_3_REG

#define BMP388_DATA_FRAME_LENGTH                        ((BMP388_DATA_5_REG - BMP388_DATA_0_REG) + 1) // +1 for inclusive

// from Datasheet 3.3
#define BMP388_MODE_SLEEP                               (0x00)
#define BMP388_MODE_FORCED                              (0x01)
#define BMP388_MODE_NORMAL                              (0x02)

#define BMP388_CALIRATION_LOWER_REG                     (0x30) // See datasheet 4.3.19, "calibration data"
#define BMP388_TRIMMING_NVM_PAR_T1_LSB_REG              (0x31) // See datasheet 3.11.1 "Memory map trimming coefficients"
#define BMP388_TRIMMING_NVM_PAR_P11_REG                 (0x45) // See datasheet 3.11.1 "Memory map trimming coefficients"
#define BMP388_CALIRATION_UPPER_REG                     (0x57)

#define BMP388_TRIMMING_DATA_LENGTH                     ((BMP388_TRIMMING_NVM_PAR_P11_REG - BMP388_TRIMMING_NVM_PAR_T1_LSB_REG) + 1) // +1 for inclusive

#define BMP388_DATA_FRAME_SIZE               (6)

#define BMP388_OVERSAMP_1X               (0x00)
#define BMP388_OVERSAMP_2X               (0x01)
#define BMP388_OVERSAMP_4X               (0x02)
#define BMP388_OVERSAMP_8X               (0x03)
#define BMP388_OVERSAMP_16X              (0x04)
#define BMP388_OVERSAMP_32X              (0x05)

// INT_CTRL register
#define BMP388_INT_OD_BIT                   0
#define BMP388_INT_LEVEL_BIT                1
#define BMP388_INT_LATCH_BIT                2
#define BMP388_INT_FWTM_EN_BIT              3
#define BMP388_INT_FFULL_EN_BIT             4
#define BMP388_INT_RESERVED_5_BIT           5
#define BMP388_INT_DRDY_EN_BIT              6
#define BMP388_INT_RESERVED_7_BIT           7

// OSR register
#define BMP388_OSR_P_BIT                    0 // to 2
#define BMP388_OSR4_T_BIT                   3 // to 5
#define BMP388_OSR_P_MASK                   (0x03)  // -----111
#define BMP388_OSR4_T_MASK                  (0x38)  // --111---

// configure pressure and temperature oversampling, forced sampling mode
#define BMP388_PRESSURE_OSR              (BMP388_OVERSAMP_8X)
#define BMP388_TEMPERATURE_OSR           (BMP388_OVERSAMP_1X)

// see Datasheet 3.11.1 Memory Map Trimming Coefficients
typedef struct bmp388_calib_param_s {
    uint16_t T1;
    uint16_t T2;
    int8_t T3;
    int16_t P1;
    int16_t P2;
    int8_t P3;
    int8_t P4;
    uint16_t P5;
    uint16_t P6;
    int8_t P7;
    int8_t P8;
    int16_t P9;
    int8_t P10;
    int8_t P11;
} __attribute__((packed)) bmp388_calib_param_t;

STATIC_ASSERT(sizeof(bmp388_calib_param_t) == BMP388_TRIMMING_DATA_LENGTH, bmp388_calibration_structure_incorrectly_packed);

static uint8_t bmp388_chip_id = 0;
STATIC_UNIT_TESTED bmp388_calib_param_t bmp388_cal;
// uncompensated pressure and temperature
uint32_t bmp388_up = 0;
uint32_t bmp388_ut = 0;
STATIC_UNIT_TESTED int64_t t_lin = 0;

static void bmp388StartUT(baroDev_t *baro);
static void bmp388GetUT(baroDev_t *baro);
static void bmp388StartUP(baroDev_t *baro);
static void bmp388GetUP(baroDev_t *baro);

STATIC_UNIT_TESTED void bmp388Calculate(int32_t *pressure, int32_t *temperature);

#ifdef USE_EXTI
void bmp388_extiHandler(extiCallbackRec_t* cb)
{
#ifdef DEBUG
    static uint32_t bmp388ExtiCallbackCounter = 0;

    bmp388ExtiCallbackCounter++;
#endif

    baroDev_t *baro = container_of(cb, baroDev_t, exti);

    uint8_t intStatus = 0;
    busReadRegisterBuffer(&baro->busdev, BMP388_INT_STATUS_REG, &intStatus, 1);
}
#endif

void bmp388BusInit(busDevice_t *busdev)
{
#ifdef USE_BARO_SPI_BMP388
    if (busdev->bustype == BUSTYPE_SPI) {
        IOHi(busdev->busdev_u.spi.csnPin); // Disable
        IOInit(busdev->busdev_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(busdev->busdev_u.spi.csnPin, IOCFG_OUT_PP);
        spiBusSetDivisor(busdev, SPI_CLOCK_STANDARD);
    }
#else
    UNUSED(busdev);
#endif
}

void bmp388BusDeinit(busDevice_t *busdev)
{
#ifdef USE_BARO_SPI_BMP388
    if (busdev->bustype == BUSTYPE_SPI) {
        spiPreinitByIO(busdev->busdev_u.spi.csnPin);
    }
#else
    UNUSED(busdev);
#endif
}

void bmp388BeginForcedMeasurement(busDevice_t *busdev)
{
    // enable pressure measurement, temperature measurement, set power mode and start sampling
    uint8_t mode = (BMP388_MODE_FORCED << 4) | (1 << 1) | (1 << 0);
    busWriteRegister(busdev, BMP388_PWR_CTRL_REG, mode);
}

bool bmp388Detect(const bmp388Config_t *config, baroDev_t *baro)
{
    delay(20);

#ifdef USE_EXTI
    IO_t baroIntIO = IOGetByTag(config->eocTag);
    if (baroIntIO) {
        IOInit(baroIntIO, OWNER_BARO_EOC, 0);
        EXTIHandlerInit(&baro->exti, bmp388_extiHandler);
        EXTIConfig(baroIntIO, &baro->exti, NVIC_PRIO_BARO_EXTI, IOCFG_IN_FLOATING, UEXTI_TRIGGER_RISING);
        EXTIEnable(baroIntIO, true);
    }
#else
    UNUSED(config);
#endif

    busDevice_t *busdev = &baro->busdev;
    bool defaultAddressApplied = false;

    bmp388BusInit(busdev);

    if ((busdev->bustype == BUSTYPE_I2C) && (busdev->busdev_u.i2c.address == 0)) {
        // Default address for BMP388
        busdev->busdev_u.i2c.address = BMP388_I2C_ADDR;
        defaultAddressApplied = true;
    }

    busReadRegisterBuffer(busdev, BMP388_CHIP_ID_REG, &bmp388_chip_id, 1);

    if (bmp388_chip_id != BMP388_DEFAULT_CHIP_ID) {
        bmp388BusDeinit(busdev);
        if (defaultAddressApplied) {
            busdev->busdev_u.i2c.address = 0;
        }
        return false;
    }

#ifdef USE_EXTI
    if (baroIntIO) {
        uint8_t intCtrlValue = (1 << BMP388_INT_DRDY_EN_BIT) | (0 << BMP388_INT_FFULL_EN_BIT) | (0 << BMP388_INT_FWTM_EN_BIT) | (0 << BMP388_INT_LATCH_BIT) | (1 << BMP388_INT_LEVEL_BIT) | (0 << BMP388_INT_OD_BIT);
        busWriteRegister(busdev, BMP388_INT_CTRL_REG, intCtrlValue);
    }
#endif

    // read calibration
    busReadRegisterBuffer(busdev, BMP388_TRIMMING_NVM_PAR_T1_LSB_REG, (uint8_t *)&bmp388_cal, sizeof(bmp388_calib_param_t));


    // set oversampling
    busWriteRegister(busdev, BMP388_OSR_REG,
        ((BMP388_PRESSURE_OSR << BMP388_OSR_P_BIT) & BMP388_OSR_P_MASK) |
        ((BMP388_TEMPERATURE_OSR << BMP388_OSR4_T_BIT) & BMP388_OSR4_T_MASK)
    );

    bmp388BeginForcedMeasurement(busdev);

    // these are dummy as temperature is measured as part of pressure
    baro->ut_delay = 0;
    baro->get_ut = bmp388GetUT;
    baro->start_ut = bmp388StartUT;
    // only _up part is executed, and gets both temperature and pressure
    baro->start_up = bmp388StartUP;
    baro->get_up = bmp388GetUP;

    // See datasheet 3.9.2 "Measurement rate in forced mode and normal mode"
    baro->up_delay = 234 +
        (392 + (powerf(2, BMP388_PRESSURE_OSR + 1) * 2000)) +
        (313 + (powerf(2, BMP388_TEMPERATURE_OSR + 1) * 2000));

    baro->calculate = bmp388Calculate;

    return true;
}

static void bmp388StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
}

static void bmp388GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
}

static void bmp388StartUP(baroDev_t *baro)
{
    // start measurement
    bmp388BeginForcedMeasurement(&baro->busdev);
}

static void bmp388GetUP(baroDev_t *baro)
{
    uint8_t dataFrame[BMP388_DATA_FRAME_LENGTH];

    // read data from sensor
    busReadRegisterBuffer(&baro->busdev, BMP388_DATA_0_REG, dataFrame, BMP388_DATA_FRAME_LENGTH);

    bmp388_up = ((uint32_t)(dataFrame[0]) << 0) | ((uint32_t)(dataFrame[1]) << 8) | ((uint32_t)(dataFrame[2]) << 16);
    bmp388_ut = ((uint32_t)(dataFrame[3]) << 0) | ((uint32_t)(dataFrame[4]) << 8) | ((uint32_t)(dataFrame[5]) << 16);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
static int64_t bmp388CompensateTemperature(uint32_t uncomp_temperature)
{
    uint64_t partial_data1;
    uint64_t partial_data2;
    uint64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t comp_temp;

    partial_data1 = uncomp_temperature - (256 * bmp388_cal.T1);
    partial_data2 = bmp388_cal.T2 * partial_data1;
    partial_data3 = partial_data1 * partial_data1;
    partial_data4 = (int64_t)partial_data3 * bmp388_cal.T3;
    partial_data5 = ((int64_t)(partial_data2 * 262144) + partial_data4);
    partial_data6 = partial_data5 / 4294967296;
    /* Update t_lin, needed for pressure calculation */
    t_lin = partial_data6;
    comp_temp = (int64_t)((partial_data6 * 25)  / 16384);

    return comp_temp;
}

static uint64_t bmp388CompensatePressure(uint32_t uncomp_pressure)
{
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t offset;
    int64_t sensitivity;
    uint64_t comp_press;

    partial_data1 = t_lin * t_lin;
    partial_data2 = partial_data1 / 64;
    partial_data3 = (partial_data2 * t_lin) / 256;
    partial_data4 = (bmp388_cal.P8 * partial_data3) / 32;
    partial_data5 = (bmp388_cal.P7 * partial_data1) * 16;
    partial_data6 = (bmp388_cal.P6 * t_lin) * 4194304;
    offset = (bmp388_cal.P5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6;

    partial_data2 = (bmp388_cal.P4 * partial_data3) / 32;
    partial_data4 = (bmp388_cal.P3 * partial_data1) * 4;
    partial_data5 = (bmp388_cal.P2 - 16384) * t_lin * 2097152;
    sensitivity = ((bmp388_cal.P1 - 16384) * 70368744177664) + partial_data2 + partial_data4
            + partial_data5;

    partial_data1 = (sensitivity / 16777216) * uncomp_pressure;
    partial_data2 = bmp388_cal.P10 * t_lin;
    partial_data3 = partial_data2 + (65536 * bmp388_cal.P9);
    partial_data4 = (partial_data3 * uncomp_pressure) / 8192;
    partial_data5 = (partial_data4 * uncomp_pressure) / 512;
    partial_data6 = (int64_t)((uint64_t)uncomp_pressure * (uint64_t)uncomp_pressure);
    partial_data2 = (bmp388_cal.P11 * partial_data6) / 65536;
    partial_data3 = (partial_data2 * uncomp_pressure) / 128;
    partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
    comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);

#ifdef DEBUG_BARO_BMP388
    debug[1] = ((comp_press & 0xFFFF0000) >> 32);
    debug[2] = ((comp_press & 0x0000FFFF) >>  0);
#endif
    return comp_press;
}

STATIC_UNIT_TESTED void bmp388Calculate(int32_t *pressure, int32_t *temperature)
{
    // calculate
    int64_t t;
    int64_t p;

    t = bmp388CompensateTemperature(bmp388_ut);
    p = bmp388CompensatePressure(bmp388_up);

    if (pressure)
        *pressure = (int32_t)(p / 256);
    if (temperature)
        *temperature = t;
}

#endif
