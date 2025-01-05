/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"
#include "types.h"

#define DEBUG_VALUE_COUNT       8
#define DEBUG16_VALUE_COUNT     8  // BF compat

extern uint8_t debugMode;
extern uint8_t debugAxis;

extern int32_t debug[DEBUG_VALUE_COUNT];

extern uint32_t __timing[DEBUG_VALUE_COUNT];

#define DEBUG_SET(mode, index, value)             do { if (debugMode == (mode)) { debug[(index)] = (value); } } while (0)
#define DEBUG_AXIS_SET(mode, axis, index, value)  do { if (debugAxis == (axis) && debugMode == (mode)) { debug[(index)] = (value); } } while (0)
#define DEBUG_COND_SET(mode, cond, index, value)  do { if ((cond) && debugMode == (mode)) { debug[(index)] = (value); } } while (0)

#define DEBUG_VAL(mode, index, value)             DEBUG_SET(DEBUG_ ## mode, index, value)
#define DEBUG_AXIS(mode, axis, index, value)      DEBUG_AXIS_SET(DEBUG_ ## mode, axis, index, value)
#define DEBUG_COND(mode, cond, index, value)      DEBUG_COND_SET(DEBUG_ ## mode, cond, index, value)

#define DEBUG_TIME_START(mode, index)             do { if (debugMode == (DEBUG_ ## mode)) { __timing[(index)] = micros(); } } while (0)
#define DEBUG_TIME_END(mode, index)               do { if (debugMode == (DEBUG_ ## mode)) { debug[(index)] = micros() - __timing[(index)]; } } while (0)


typedef enum {
    DEBUG_NONE = 0,
    DEBUG_CYCLETIME,
    DEBUG_BATTERY,
    DEBUG_GYRO_FILTERED,
    DEBUG_ACCELEROMETER,
    DEBUG_PIDLOOP,
    DEBUG_RC_INTERPOLATION,
    DEBUG_ANGLERATE,
    DEBUG_ESC_SENSOR,
    DEBUG_SCHEDULER,
    DEBUG_STACK,
    DEBUG_ESC_SENSOR_RPM,
    DEBUG_ESC_SENSOR_TMP,
    DEBUG_ALTITUDE,
    DEBUG_FFT,
    DEBUG_FFT_TIME,
    DEBUG_FFT_FREQ,
    DEBUG_RX_FRSKY_SPI,
    DEBUG_RX_SFHSS_SPI,
    DEBUG_GYRO_RAW,
    DEBUG_DUAL_GYRO_RAW,
    DEBUG_DUAL_GYRO_DIFF,
    DEBUG_MAX7456_SIGNAL,
    DEBUG_MAX7456_SPICLOCK,
    DEBUG_SBUS,
    DEBUG_FPORT,
    DEBUG_RANGEFINDER,
    DEBUG_RANGEFINDER_QUALITY,
    DEBUG_OPTICALFLOW,
    DEBUG_LIDAR_TF,
    DEBUG_ADC_INTERNAL,
    DEBUG_SDIO,
    DEBUG_CURRENT_SENSOR,
    DEBUG_USB,
    DEBUG_SMARTAUDIO,
    DEBUG_RTH,
    DEBUG_RX_SIGNAL_LOSS,
    DEBUG_RX_SPEKTRUM_SPI,
    DEBUG_DSHOT_RPM_TELEMETRY,
    DEBUG_DUAL_GYRO_SCALED,
    DEBUG_DSHOT_RPM_ERRORS,
    DEBUG_CRSF_LINK_STATISTICS_UPLINK,
    DEBUG_CRSF_LINK_STATISTICS_PWR,
    DEBUG_CRSF_LINK_STATISTICS_DOWN,
    DEBUG_BARO,
    DEBUG_AUTOPILOT_ALTITUDE,
    DEBUG_BLACKBOX_OUTPUT,
    DEBUG_GYRO_SAMPLE,
    DEBUG_RX_TIMING,
    DEBUG_D_LPF,
    DEBUG_VTX_TRAMP,
    DEBUG_GHST,
    DEBUG_GHST_MSP,
    DEBUG_SCHEDULER_DETERMINISM,
    DEBUG_TIMING_ACCURACY,
    DEBUG_RX_EXPRESSLRS_SPI,
    DEBUG_RX_EXPRESSLRS_PHASELOCK,
    DEBUG_RX_STATE_TIME,
    DEBUG_GPS_RESCUE_VELOCITY,
    DEBUG_GPS_RESCUE_HEADING,
    DEBUG_GPS_RESCUE_TRACKING,
    DEBUG_GPS_CONNECTION,
    DEBUG_ATTITUDE,
    DEBUG_VTX_MSP,
    DEBUG_GPS_DOP,
    DEBUG_FAILSAFE,
    DEBUG_GYRO_CALIBRATION,
    DEBUG_ANGLE_MODE,
    DEBUG_ANGLE_TARGET,
    DEBUG_CURRENT_ANGLE,
    DEBUG_DSHOT_TELEMETRY_COUNTS,
    DEBUG_RC_STATS,
    DEBUG_MAG_CALIB,
    DEBUG_MAG_TASK_RATE,
    DEBUG_TASK,
    DEBUG_WING_SETPOINT,
    DEBUG_AUTOPILOT_POSITION,
    DEBUG_COUNT
} debugType_e;

extern const char * const debugModeNames[DEBUG_COUNT];

void debugInit(void);
