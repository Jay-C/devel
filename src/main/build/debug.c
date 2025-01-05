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

#include "platform.h"

#include "debug.h"

FAST_DATA_ZERO_INIT uint8_t debugMode;
FAST_DATA_ZERO_INIT uint8_t debugAxis;

FAST_DATA_ZERO_INIT int32_t debug[DEBUG_VALUE_COUNT];

FAST_DATA_ZERO_INIT uint32_t __timing[DEBUG_VALUE_COUNT];

#define ENTRY(_NAME)  [DEBUG_##_NAME] = #_NAME

const char * const debugModeNames[DEBUG_COUNT] = {
    ENTRY(NONE),
    ENTRY(CYCLETIME),
    ENTRY(BATTERY),
    ENTRY(GYRO_FILTERED),
    ENTRY(ACCELEROMETER),
    ENTRY(PIDLOOP),
    ENTRY(RC_INTERPOLATION),
    ENTRY(ANGLERATE),
    ENTRY(ESC_SENSOR),
    ENTRY(SCHEDULER),
    ENTRY(STACK),
    ENTRY(ESC_SENSOR_RPM),
    ENTRY(ESC_SENSOR_TMP),
    ENTRY(ALTITUDE),
    ENTRY(FFT),
    ENTRY(FFT_TIME),
    ENTRY(FFT_FREQ),
    ENTRY(RX_FRSKY_SPI),
    ENTRY(RX_SFHSS_SPI),
    ENTRY(GYRO_RAW),
    ENTRY(DUAL_GYRO_RAW),
    ENTRY(DUAL_GYRO_DIFF),
    ENTRY(MAX7456_SIGNAL),
    ENTRY(MAX7456_SPICLOCK),
    ENTRY(SBUS),
    ENTRY(FPORT),
    ENTRY(RANGEFINDER),
    ENTRY(RANGEFINDER_QUALITY),
    ENTRY(OPTICALFLOW),
    ENTRY(LIDAR_TF),
    ENTRY(ADC_INTERNAL),
    ENTRY(SDIO),
    ENTRY(CURRENT_SENSOR),
    ENTRY(USB),
    ENTRY(SMARTAUDIO),
    ENTRY(RTH),
    ENTRY(RX_SIGNAL_LOSS),
    ENTRY(RX_SPEKTRUM_SPI),
    ENTRY(DSHOT_RPM_TELEMETRY),
    ENTRY(DUAL_GYRO_SCALED),
    ENTRY(DSHOT_RPM_ERRORS),
    ENTRY(CRSF_LINK_STATISTICS_UPLINK),
    ENTRY(CRSF_LINK_STATISTICS_PWR),
    ENTRY(CRSF_LINK_STATISTICS_DOWN),
    ENTRY(BARO),
    ENTRY(AUTOPILOT_ALTITUDE),
    ENTRY(BLACKBOX_OUTPUT),
    ENTRY(GYRO_SAMPLE),
    ENTRY(RX_TIMING),
    ENTRY(D_LPF),
    ENTRY(VTX_TRAMP),
    ENTRY(GHST),
    ENTRY(GHST_MSP),
    ENTRY(SCHEDULER_DETERMINISM),
    ENTRY(TIMING_ACCURACY),
    ENTRY(RX_EXPRESSLRS_SPI),
    ENTRY(RX_EXPRESSLRS_PHASELOCK),
    ENTRY(RX_STATE_TIME),
    ENTRY(GPS_RESCUE_VELOCITY),
    ENTRY(GPS_RESCUE_HEADING),
    ENTRY(GPS_RESCUE_TRACKING),
    ENTRY(GPS_CONNECTION),
    ENTRY(ATTITUDE),
    ENTRY(VTX_MSP),
    ENTRY(GPS_DOP),
    ENTRY(FAILSAFE),
    ENTRY(GYRO_CALIBRATION),
    ENTRY(ANGLE_MODE),
    ENTRY(ANGLE_TARGET),
    ENTRY(CURRENT_ANGLE),
    ENTRY(DSHOT_TELEMETRY_COUNTS),
    ENTRY(RC_STATS),
    ENTRY(MAG_CALIB),
    ENTRY(MAG_TASK_RATE),
    ENTRY(TASK),
    ENTRY(WING_SETPOINT),
    ENTRY(AUTOPILOT_POSITION),
};

#undef ENTRY
