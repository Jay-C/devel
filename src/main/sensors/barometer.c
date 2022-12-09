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
#include <math.h>

#include "platform.h"

#ifdef USE_BARO

#include "build/debug.h"

#include "common/maths.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp085.h"
#include "drivers/barometer/barometer_bmp280.h"
#include "drivers/barometer/barometer_bmp388.h"
#include "drivers/barometer/barometer_dps310.h"
#include "drivers/barometer/barometer_qmp6988.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/barometer/barometer_ms5611.h"
#include "drivers/barometer/barometer_lps.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"

#include "scheduler/scheduler.h"

#include "barometer.h"

PG_REGISTER_WITH_RESET_FN(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 1);

void pgResetFn_barometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->baro_hardware = BARO_DEFAULT;

#if defined(DEFAULT_BARO_SPI_BMP388) || defined(DEFAULT_BARO_SPI_BMP280) || defined(DEFAULT_BARO_SPI_MS5611) || defined(DEFAULT_BARO_SPI_QMP6988) || defined(DEFAULT_BARO_SPI_LPS) || defined(DEFAULT_BARO_SPI_DPS310)
    barometerConfig->baro_busType = BUS_TYPE_SPI;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(spiDeviceByInstance(BARO_SPI_INSTANCE));
    barometerConfig->baro_spi_csn = IO_TAG(BARO_CS_PIN);
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    barometerConfig->baro_i2c_address = 0;
#elif defined(DEFAULT_BARO_MS5611) || defined(DEFAULT_BARO_BMP388) || defined(DEFAULT_BARO_BMP280) || defined(DEFAULT_BARO_BMP085) ||defined(DEFAULT_BARO_QMP6988) || defined(DEFAULT_BARO_DPS310)
    // All I2C devices shares a default config with address = 0 (per device default)
    barometerConfig->baro_busType = BUS_TYPE_I2C;
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(BARO_I2C_INSTANCE);
    barometerConfig->baro_i2c_address = 0;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    barometerConfig->baro_spi_csn = IO_TAG_NONE;
#else
    barometerConfig->baro_hardware = BARO_NONE;
    barometerConfig->baro_busType = BUS_TYPE_NONE;
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    barometerConfig->baro_i2c_address = 0;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    barometerConfig->baro_spi_csn = IO_TAG_NONE;
#endif

    barometerConfig->baro_eoc_tag = IO_TAG(BARO_EOC_PIN);
    barometerConfig->baro_xclr_tag = IO_TAG(BARO_XCLR_PIN);
}

baro_t baro;

static bool baroReady = false;

static uint16_t calibCycles = 0;

static int32_t baroPressure = 0;
static int32_t baroTemperature = 0;

static float baroGroundAltitude = 0;
static float baroGroundPressure = 101325;

#define CALIBRATING_BARO_CYCLES 200 // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
#define SET_GROUND_LEVEL_BARO_CYCLES 20 // calibrate baro to new ground level (20 * 25 ms = ~500 ms non blocking)


void baroPreInit(void)
{
#ifdef USE_SPI
    if (barometerConfig()->baro_busType == BUS_TYPE_SPI) {
        spiPreinitRegister(barometerConfig()->baro_spi_csn, IOCFG_IPU, 1);
    }
#endif
}

bool baroDetect(baroDev_t *baroDev, baroSensor_e baroHardwareToUse)
{
    extDevice_t *dev = &baroDev->dev;

    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function

    baroSensor_e baroHardware = baroHardwareToUse;

#if !defined(USE_BARO_BMP085) && !defined(USE_BARO_MS5611) && !defined(USE_BARO_SPI_MS5611) && !defined(USE_BARO_BMP388) && !defined(USE_BARO_BMP280) && !defined(USE_BARO_SPI_BMP280)&& !defined(USE_BARO_QMP6988) && !defined(USE_BARO_SPI_QMP6988) && !defined(USE_BARO_DPS310) && !defined(USE_BARO_SPI_DPS310)
    UNUSED(dev);
#endif

    switch (barometerConfig()->baro_busType) {
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        i2cBusSetInstance(dev, barometerConfig()->baro_i2c_device);
        dev->busType_u.i2c.address = barometerConfig()->baro_i2c_address;
        break;
#endif

#ifdef USE_SPI
    case BUS_TYPE_SPI:
        {
            if (!spiSetBusInstance(dev, barometerConfig()->baro_spi_device)) {
                return false;
            }

            dev->busType_u.spi.csnPin = IOGetByTag(barometerConfig()->baro_spi_csn);
        }
        break;
#endif

    default:
        return false;
    }

    switch (baroHardware) {
    case BARO_DEFAULT:
        FALLTHROUGH;

    case BARO_BMP085:
#ifdef USE_BARO_BMP085
        {
            static bmp085Config_t defaultBMP085Config;
            defaultBMP085Config.xclrTag = barometerConfig()->baro_xclr_tag;
            defaultBMP085Config.eocTag = barometerConfig()->baro_eoc_tag;

            static const bmp085Config_t *bmp085Config = &defaultBMP085Config;

            if (bmp085Detect(bmp085Config, baroDev)) {
                baroHardware = BARO_BMP085;
                break;
            }
        }
#endif
        FALLTHROUGH;

    case BARO_MS5611:
#if defined(USE_BARO_MS5611) || defined(USE_BARO_SPI_MS5611)
        if (ms5611Detect(baroDev)) {
            baroHardware = BARO_MS5611;
            break;
        }
#endif
        FALLTHROUGH;

    case BARO_LPS:
#if defined(USE_BARO_SPI_LPS)
        if (lpsDetect(baroDev)) {
            baroHardware = BARO_LPS;
            break;
        }
#endif
        FALLTHROUGH;

    case BARO_DPS310:
#if defined(USE_BARO_DPS310) || defined(USE_BARO_SPI_DPS310)
        {
            if (baroDPS310Detect(baroDev)) {
                baroHardware = BARO_DPS310;
                break;
            }
        }
#endif
        FALLTHROUGH;

    case BARO_BMP388:
#if defined(USE_BARO_BMP388) || defined(USE_BARO_SPI_BMP388)
        {
            static bmp388Config_t defaultBMP388Config;

            defaultBMP388Config.eocTag = barometerConfig()->baro_eoc_tag;

            static const bmp388Config_t *bmp388Config = &defaultBMP388Config;

            if (bmp388Detect(bmp388Config, baroDev)) {
                baroHardware = BARO_BMP388;
                break;
            }
        }
#endif
        FALLTHROUGH;

    case BARO_BMP280:
#if defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
        if (bmp280Detect(baroDev)) {
            baroHardware = BARO_BMP280;
            break;
        }
#endif
        FALLTHROUGH;

     case BARO_QMP6988:
#if defined(USE_BARO_QMP6988) || defined(USE_BARO_SPI_QMP6988)
        if (qmp6988Detect(baroDev)) {
            baroHardware = BARO_QMP6988;
            break;
        }
#endif
        FALLTHROUGH;
    case BARO_NONE:
        baroHardware = BARO_NONE;
        break;
    }

    if (baroHardware == BARO_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_BARO] = baroHardware;
    sensorsSet(SENSOR_BARO);
    return true;
}

static float pressureToAltitude(const float pressure)
{
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

bool baroIsReady(void)
{
    return baroReady && calibCycles == 0;
}

bool baroIsCalibrationComplete(void)
{
    return calibCycles == 0;
}

static void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibCycles = calibrationCyclesRequired;
}

void baroStartCalibration(void)
{
    baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
}

void baroSetGroundLevel(void)
{
    baroSetCalibrationCycles(SET_GROUND_LEVEL_BARO_CYCLES);
}

void performBaroCalibrationCycle(void)
{
    static uint16_t currentCycle = 0;

    float delta = (baroPressure - baroGroundPressure) / MIN(++currentCycle, 20);

    if (currentCycle > calibCycles) {
        baroGroundAltitude = pressureToAltitude(baroGroundPressure);
        currentCycle = 0;
        calibCycles = 0;
        baroReady = true;
    }
    else {
        baroGroundPressure += delta;
    }
}


typedef enum {
    BARO_STATE_TEMPERATURE_READ = 0,
    BARO_STATE_TEMPERATURE_SAMPLE,
    BARO_STATE_PRESSURE_START,
    BARO_STATE_PRESSURE_READ,
    BARO_STATE_PRESSURE_SAMPLE,
    BARO_STATE_TEMPERATURE_START,
    BARO_STATE_COUNT
} barometerState_e;


uint32_t baroUpdate(timeUs_t currentTimeUs)
{
    static timeUs_t baroStateDurationUs[BARO_STATE_COUNT];
    static barometerState_e state = BARO_STATE_PRESSURE_START;
    barometerState_e oldState = state;
    timeUs_t executeTimeUs;
    timeUs_t sleepTime = 1000; // Wait 1ms between states

    DEBUG_SET(DEBUG_BARO, 0, state);

    if (busBusy(&baro.dev.dev, NULL)) {
        // If the bus is busy, simply return to have another go later
        schedulerIgnoreTaskStateTime();
        return sleepTime;
    }

    switch (state) {
        default:
        case BARO_STATE_TEMPERATURE_START:
            baro.dev.start_ut(&baro.dev);
            state = BARO_STATE_TEMPERATURE_READ;
            sleepTime = baro.dev.ut_delay;
            break;

        case BARO_STATE_TEMPERATURE_READ:
            if (baro.dev.read_ut(&baro.dev)) {
                state = BARO_STATE_TEMPERATURE_SAMPLE;
            } else {
                // No action was taken as the read has not completed
                schedulerIgnoreTaskExecTime();
            }
            break;

        case BARO_STATE_TEMPERATURE_SAMPLE:
            if (baro.dev.get_ut(&baro.dev)) {
                state = BARO_STATE_PRESSURE_START;
            } else {
                // No action was taken as the read has not completed
                schedulerIgnoreTaskExecTime();
            }
            break;

        case BARO_STATE_PRESSURE_START:
            baro.dev.start_up(&baro.dev);
            state = BARO_STATE_PRESSURE_READ;
            sleepTime = baro.dev.up_delay;
            break;

        case BARO_STATE_PRESSURE_READ:
            if (baro.dev.read_up(&baro.dev)) {
                state = BARO_STATE_PRESSURE_SAMPLE;
            } else {
                // No action was taken as the read has not completed
                schedulerIgnoreTaskExecTime();
            }
            break;

        case BARO_STATE_PRESSURE_SAMPLE:
            if (!baro.dev.get_up(&baro.dev)) {
                // No action was taken as the read has not completed
                schedulerIgnoreTaskExecTime();
                break;
            }

            baro.dev.calculate(&baroPressure, &baroTemperature);
            baro.baroPressure = baroPressure;
            baro.baroTemperature = baroTemperature;

            if (baroIsCalibrationComplete()) {
                baro.baroAltitude = lrintf(pressureToAltitude(baroPressure) - baroGroundAltitude);
            }
            else {
                performBaroCalibrationCycle();
                baro.baroAltitude = 0;
            }

            if (baro.dev.combined_read) {
                state = BARO_STATE_PRESSURE_START;
            } else {
                state = BARO_STATE_TEMPERATURE_START;
            }

            DEBUG_SET(DEBUG_BARO, 1, baro.baroTemperature);
            DEBUG_SET(DEBUG_BARO, 2, baro.baroPressure);
            DEBUG_SET(DEBUG_BARO, 3, baro.baroAltitude);

            sleepTime = baro.dev.ut_delay;
            break;
    }

    // Where we are using a state machine call schedulerIgnoreTaskExecRate() for all states bar one
    if (sleepTime != baro.dev.ut_delay) {
        schedulerIgnoreTaskExecRate();
    }

    executeTimeUs = micros() - currentTimeUs;

    if (executeTimeUs > baroStateDurationUs[oldState]) {
        baroStateDurationUs[oldState] = executeTimeUs;
    }

    schedulerSetNextStateTime(baroStateDurationUs[state]);

    return sleepTime;
}

#endif /* BARO */
