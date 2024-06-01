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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pg/pg.h"
#include "pg/telemetry.h"

#include "common/streambuf.h"

#include "flight/motors.h"
#include "flight/servos.h"


typedef enum
{
    TELEM_NONE = 0,

    TELEM_HEARTBEAT,

    TELEM_MODEL_ID,

    TELEM_BATTERY,
    TELEM_BATTERY_VOLTAGE,
    TELEM_BATTERY_CURRENT,
    TELEM_BATTERY_CONSUMPTION,
    TELEM_BATTERY_CHARGE_LEVEL,
    TELEM_BATTERY_TEMPERATURE,
    TELEM_BATTERY_CELLS,

    TELEM_ESC1_DATA,
    TELEM_ESC1_VOLTAGE,
    TELEM_ESC1_CURRENT,
    TELEM_ESC1_ERPM,
    TELEM_ESC1_POWER,
    TELEM_ESC1_THROTTLE,
    TELEM_ESC1_TEMP1,
    TELEM_ESC1_TEMP2,
    TELEM_ESC1_BEC_VOLTAGE,
    TELEM_ESC1_BEC_CURRENT,
    TELEM_ESC1_ERRORS,
    TELEM_ESC1_STATUS,

    TELEM_ESC2_DATA,
    TELEM_ESC2_VOLTAGE,
    TELEM_ESC2_CURRENT,
    TELEM_ESC2_ERPM,
    TELEM_ESC2_POWER,
    TELEM_ESC2_THROTTLE,
    TELEM_ESC2_TEMP1,
    TELEM_ESC2_TEMP2,
    TELEM_ESC2_BEC_VOLTAGE,
    TELEM_ESC2_BEC_CURRENT,
    TELEM_ESC2_ERRORS,
    TELEM_ESC2_STATUS,

    TELEM_ESC_VOLTAGE,
    TELEM_BEC_VOLTAGE,
    TELEM_BUS_VOLTAGE,
    TELEM_MCU_VOLTAGE,

    TELEM_ESC_CURRENT,
    TELEM_BEC_CURRENT,
    TELEM_BUS_CURRENT,
    TELEM_MCU_CURRENT,

    TELEM_ESC_TEMP,
    TELEM_BEC_TEMP,
    TELEM_MCU_TEMP,
    TELEM_AIR_TEMP,
    TELEM_MOTOR_TEMP,

    TELEM_EXT1_TEMP,
    TELEM_EXT2_TEMP,
    TELEM_EXT3_TEMP,
    TELEM_EXT4_TEMP,

    TELEM_ALTITUDE,
    TELEM_VARIOMETER,

    TELEM_HEADSPEED,
    TELEM_TAILSPEED,
    TELEM_MOTOR_RPM,

    TELEM_ATTITUDE,
    TELEM_ATTITUDE_PITCH,
    TELEM_ATTITUDE_ROLL,
    TELEM_ATTITUDE_YAW,

    TELEM_ACCEL,
    TELEM_ACCEL_X,
    TELEM_ACCEL_Y,
    TELEM_ACCEL_Z,

    TELEM_GPS,
    TELEM_GPS_HEADING,
    TELEM_GPS_LATITUDE,
    TELEM_GPS_LONGITUDE,
    TELEM_GPS_ALTITUDE,
    TELEM_GPS_DISTANCE,
    TELEM_GPS_GROUNDSPEED,
    TELEM_GPS_SAT_COUNT,
    TELEM_GPS_DATE_TIME,

    TELEM_FC,
    TELEM_FC_UPTIME,
    TELEM_FC_CPU_LOAD,
    TELEM_FC_SYS_LOAD,
    TELEM_FC_RT_LOAD,

    TELEM_FLIGHT_MODE,
    TELEM_ARMING_FLAGS,
    TELEM_GOVERNOR_STATE,

    TELEM_PROFILES,
    TELEM_PID_PROFILE,
    TELEM_RATES_PROFILE,
    TELEM_BATTERY_PROFILE,
    TELEM_LED_PROFILE,

    TELEM_ADJFUNC,

    TELEM_SENSOR_COUNT,

} sensor_id_e;


typedef int telemetryValue_t;

typedef int (*telemetryValue_f)(void);
typedef void (*telemetryEncode_f)(sbuf_t *buf, telemetryValue_t value);

typedef uint16_t sensor_code_t;

typedef struct {

    sensor_id_e             index;
    sensor_code_t           code;

    int                     min_period;
    int                     max_period;

    telemetryEncode_f       encode;
    telemetryValue_f        value;

} telemetrySensor_t;


/* Compatibility */

typedef enum {
    SENSOR_VOLTAGE         = BIT(0),
    SENSOR_CURRENT         = BIT(1),
    SENSOR_FUEL            = BIT(2),
    SENSOR_MODE            = BIT(3),
    SENSOR_ACC_X           = BIT(4),
    SENSOR_ACC_Y           = BIT(5),
    SENSOR_ACC_Z           = BIT(6),
    SENSOR_PITCH           = BIT(7),
    SENSOR_ROLL            = BIT(8),
    SENSOR_HEADING         = BIT(9),
    SENSOR_ALTITUDE        = BIT(10),
    SENSOR_VARIO           = BIT(11),
    SENSOR_LAT_LONG        = BIT(12),
    SENSOR_GROUND_SPEED    = BIT(13),
    SENSOR_DISTANCE        = BIT(14),
    ESC_SENSOR_CURRENT     = BIT(15),
    ESC_SENSOR_VOLTAGE     = BIT(16),
    ESC_SENSOR_RPM         = BIT(17),
    ESC_SENSOR_TEMPERATURE = BIT(18),
    SENSOR_TEMPERATURE     = BIT(19),
    SENSOR_CAP_USED        = BIT(20),
    SENSOR_ADJUSTMENT      = BIT(21),
    SENSOR_GOV_MODE        = BIT(22),
} sensor_e;

sensor_e telemetrySensorGetLegacy(sensor_id_e sensor_id);

void legacySensorInit(void);
