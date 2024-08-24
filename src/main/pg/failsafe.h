/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "pg/pg.h"

typedef enum {
    FAILSAFE_SWITCH_MODE_STAGE1 = 0,
    FAILSAFE_SWITCH_MODE_KILL,
    FAILSAFE_SWITCH_MODE_STAGE2
} failsafeSwitchMode_e;

typedef enum {
    FAILSAFE_PROCEDURE_AUTO_LANDING = 0,
    FAILSAFE_PROCEDURE_DROP_IT,
#ifdef USE_GPS_RESCUE
    FAILSAFE_PROCEDURE_GPS_RESCUE,
#endif
    FAILSAFE_PROCEDURE_COUNT   // must be last
} failsafeProcedure_e;

typedef struct {
    uint16_t failsafe_throttle;             // Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
    uint16_t failsafe_throttle_low_delay;   // Time throttle stick must have been below 'min_check' to "JustDisarm" instead of "full failsafe procedure".
    uint8_t failsafe_delay;                 // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t failsafe_landing_time;          // Time for Landing before disarm in seconds.
    uint8_t failsafe_switch_mode;           // failsafe switch action is 0: Stage 1, 1: Disarms instantly, 2: Stage 2
    uint8_t failsafe_procedure;             // selected full failsafe procedure is 0: auto-landing, 1: Drop it
    uint16_t failsafe_recovery_delay;       // Time (in 0.1sec) of valid rx data (min 100ms PERIOD_RXDATA_RECOVERY) to allow recovering from failsafe procedure
    uint8_t failsafe_stick_threshold;       // Stick deflection percentage to exit GPS Rescue procedure
} failsafeConfig_t;

PG_DECLARE(failsafeConfig_t, failsafeConfig);

