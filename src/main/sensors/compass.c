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
#include <string.h>

#include "platform.h"

#if defined(USE_MAG)

#include "config/config.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_ak8975.h"
#include "drivers/compass/compass_ak8963.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/compass/compass_hmc5883l.h"
#include "drivers/compass/compass_lis3mdl.h"
#include "drivers/compass/compass_mpu925x_ak8963.h"
#include "drivers/compass/compass_qmc5883l.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/sensors.h"

#include "compass.h"

static timeUs_t tCal = 0;
static flightDynamicsTrims_t magZeroTempMin;
static flightDynamicsTrims_t magZeroTempMax;

magDev_t magDev;
mag_t mag;

static int16_t magADCRaw[XYZ_AXIS_COUNT];
static uint8_t magInit = 0;

void compassPreInit(void)
{
#ifdef USE_SPI
    if (compassConfig()->mag_busType == BUS_TYPE_SPI) {
        spiPreinitRegister(compassConfig()->mag_spi_csn, IOCFG_IPU, 1);
    }
#endif
}

#if !defined(SIMULATOR_BUILD)
bool compassDetect(magDev_t *magDev, uint8_t *alignment)
{
    *alignment = ALIGN_DEFAULT;  // may be overridden if target specifies MAG_*_ALIGN

    magSensor_e magHardware = MAG_NONE;

    extDevice_t *dev = &magDev->dev;
    // Associate magnetometer bus with its device
    dev->bus = &magDev->bus;

#ifdef USE_MAG_DATA_READY_SIGNAL
    magDev->magIntExtiTag = compassConfig()->interruptTag;
#endif

    switch (compassConfig()->mag_busType) {
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        i2cBusSetInstance(dev, compassConfig()->mag_i2c_device);
        dev->busType_u.i2c.address = compassConfig()->mag_i2c_address;
        break;
#endif

#ifdef USE_SPI
    case BUS_TYPE_SPI:
        {
            if (!spiSetBusInstance(dev, compassConfig()->mag_spi_device)) {
                return false;
            }

            dev->busType_u.spi.csnPin = IOGetByTag(compassConfig()->mag_spi_csn);
        }
        break;
#endif

#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUS_TYPE_MPU_SLAVE:
        {
            if (gyroMpuDetectionResult()->sensor == MPU_9250_SPI) {
                extDevice_t *masterDev = &gyroActiveDev()->dev;

                dev->busType_u.i2c.address = compassConfig()->mag_i2c_address;
                dev->bus->busType = BUS_TYPE_MPU_SLAVE;
                dev->bus->busType_u.mpuSlave.master = masterDev;
            } else {
                return false;
            }
        }
        break;
#endif

    default:
        return false;
    }

    switch (compassConfig()->mag_hardware) {
    case MAG_DEFAULT:
        FALLTHROUGH;

    case MAG_HMC5883:
#if defined(USE_MAG_HMC5883) || defined(USE_MAG_SPI_HMC5883)
        if (dev->bus->busType == BUS_TYPE_I2C) {
            dev->busType_u.i2c.address = compassConfig()->mag_i2c_address;
        }

        if (hmc5883lDetect(magDev)) {
#ifdef MAG_HMC5883_ALIGN
            *alignment = MAG_HMC5883_ALIGN;
#endif
            magHardware = MAG_HMC5883;
            break;
        }
#endif
        FALLTHROUGH;

    case MAG_LIS3MDL:
#if defined(USE_MAG_LIS3MDL)
        if (dev->bus->busType == BUS_TYPE_I2C) {
            dev->busType_u.i2c.address = compassConfig()->mag_i2c_address;
        }

        if (lis3mdlDetect(magDev)) {
#ifdef MAG_LIS3MDL_ALIGN
            *alignment = MAG_LIS3MDL_ALIGN;
#endif
            magHardware = MAG_LIS3MDL;
            break;
        }
#endif
        FALLTHROUGH;

    case MAG_AK8975:
#ifdef USE_MAG_AK8975
        if (dev->bus->busType == BUS_TYPE_I2C) {
            dev->busType_u.i2c.address = compassConfig()->mag_i2c_address;
        }

        if (ak8975Detect(magDev)) {
#ifdef MAG_AK8975_ALIGN
            *alignment = MAG_AK8975_ALIGN;
#endif
            magHardware = MAG_AK8975;
            break;
        }
#endif
        FALLTHROUGH;

    case MAG_AK8963:
#if defined(USE_MAG_AK8963) || defined(USE_MAG_SPI_AK8963)
        if (dev->bus->busType == BUS_TYPE_I2C) {
            dev->busType_u.i2c.address = compassConfig()->mag_i2c_address;
        }
        if (gyroMpuDetectionResult()->sensor == MPU_9250_SPI) {
            dev->bus->busType = BUS_TYPE_MPU_SLAVE;
            dev->busType_u.mpuSlave.address = compassConfig()->mag_i2c_address;
            dev->bus->busType_u.mpuSlave.master = &gyroActiveDev()->dev;
        }

        if (ak8963Detect(magDev)) {
#ifdef MAG_AK8963_ALIGN
            *alignment = MAG_AK8963_ALIGN;
#endif
            magHardware = MAG_AK8963;
            break;
        }
#endif
        FALLTHROUGH;

    case MAG_QMC5883:
#ifdef USE_MAG_QMC5883
        if (dev->bus->busType == BUS_TYPE_I2C) {
            dev->busType_u.i2c.address = compassConfig()->mag_i2c_address;
        }

        if (qmc5883lDetect(magDev)) {
#ifdef MAG_QMC5883L_ALIGN
            *alignment = MAG_QMC5883L_ALIGN;
#endif
            magHardware = MAG_QMC5883;
            break;
        }
#endif
        FALLTHROUGH;

    case MAG_NONE:
        magHardware = MAG_NONE;
        break;
    }

    // MAG_MPU925X_AK8963 is an MPU925x configured as I2C passthrough to the built-in AK8963 magnetometer
    // Passthrough mode disables the gyro/acc part of the MPU, so we only want to detect this sensor if mag_hardware was explicitly set to MAG_MPU925X_AK8963
#ifdef USE_MAG_MPU925X_AK8963
    if(compassConfig()->mag_hardware == MAG_MPU925X_AK8963){
        if (mpu925Xak8963CompassDetect(magDev)) {
            magHardware = MAG_MPU925X_AK8963;
        } else {
            return false;
        }
    }
#endif

    if (magHardware == MAG_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_MAG] = magHardware;
    sensorsSet(SENSOR_MAG);
    return true;
}
#else
bool compassDetect(magDev_t *dev, sensor_align_e *alignment)
{
    UNUSED(dev);
    UNUSED(alignment);

    return false;
}
#endif // !SIMULATOR_BUILD

bool compassInit(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)

    sensor_align_e alignment;

    if (!compassDetect(&magDev, &alignment)) {
        return false;
    }

    LED1_ON;
    magDev.init(&magDev);
    LED1_OFF;
    magInit = 1;

    magDev.magAlignment = alignment;

    if (compassConfig()->mag_alignment != ALIGN_DEFAULT) {
        magDev.magAlignment = compassConfig()->mag_alignment;
    }

    buildRotationMatrixFromAlignment(&compassConfig()->mag_customAlignment, &magDev.rotationMatrix);

    return true;
}

bool compassIsHealthy(void)
{
    return (mag.magADC[X] != 0) && (mag.magADC[Y] != 0) && (mag.magADC[Z] != 0);
}

void compassStartCalibration(void)
{
    tCal = micros();
    flightDynamicsTrims_t *magZero = &compassConfigMutable()->magZero;
    for (int axis = 0; axis < 3; axis++) {
        magZero->raw[axis] = 0;
        magZeroTempMin.raw[axis] = mag.magADC[axis];
        magZeroTempMax.raw[axis] = mag.magADC[axis];
    }
}

bool compassIsCalibrationComplete(void)
{
    return tCal == 0;
}

uint32_t compassUpdate(timeUs_t currentTimeUs)
{
    if (busBusy(&magDev.dev, NULL) || !magDev.read(&magDev, magADCRaw)) {
        // No action was taken as the read has not completed
        schedulerIgnoreTaskExecRate();
        return 1000; // Wait 1ms between states
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mag.magADC[axis] = magADCRaw[axis];
    }
    if (magDev.magAlignment == ALIGN_CUSTOM) {
        alignSensorViaMatrix(mag.magADC, &magDev.rotationMatrix);
    } else {
        alignSensorViaRotation(mag.magADC, magDev.magAlignment);
    }

    flightDynamicsTrims_t *magZero = &compassConfigMutable()->magZero;
    if (magInit) {              // we apply offset only once mag calibration is done
        mag.magADC[X] -= magZero->raw[X];
        mag.magADC[Y] -= magZero->raw[Y];
        mag.magADC[Z] -= magZero->raw[Z];
    }

    if (tCal != 0) {
        if ((currentTimeUs - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            LED0_TOGGLE;
            for (int axis = 0; axis < 3; axis++) {
                if (mag.magADC[axis] < magZeroTempMin.raw[axis])
                    magZeroTempMin.raw[axis] = mag.magADC[axis];
                if (mag.magADC[axis] > magZeroTempMax.raw[axis])
                    magZeroTempMax.raw[axis] = mag.magADC[axis];
            }
        } else {
            tCal = 0;
            for (int axis = 0; axis < 3; axis++) {
                magZero->raw[axis] = (magZeroTempMin.raw[axis] + magZeroTempMax.raw[axis]) / 2; // Calculate offsets
            }

            saveConfigAndNotify();
        }
    }

    return TASK_PERIOD_HZ(10);
}
#endif // USE_MAG
