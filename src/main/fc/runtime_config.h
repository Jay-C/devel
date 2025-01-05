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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "common/utils.h"

// FIXME some of these are flight modes, some of these are general status indicators
typedef enum {
    ARMED                       = BIT(0),
    WAS_EVER_ARMED              = BIT(1),
    WAS_ARMED_WITH_PREARM       = BIT(2),
} armingFlag_e;

extern uint8_t armingFlags;

#define DISABLE_ARMING_FLAG(mask)   (armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask)    (armingFlags |= (mask))
#define ARMING_FLAG(mask)           (armingFlags & (mask))

/*
 * Arming disable flags are listed in the order of criticalness.
 * (Beeper code can notify the most critical reason.)
 */
typedef enum {
    ARMING_DISABLED_NO_GYRO_BIT         = 0,
    ARMING_DISABLED_FAILSAFE_BIT        = 1,
    ARMING_DISABLED_RX_FAILSAFE_BIT     = 2,
    ARMING_DISABLED_NOT_DISARMED_BIT    = 3,
    ARMING_DISABLED_BOXFAILSAFE_BIT     = 4,
    ARMING_DISABLED_UNUSED_5_BIT        = 5,
    ARMING_DISABLED_UNUSED_6_BIT        = 6,
    ARMING_DISABLED_THROTTLE_BIT        = 7,
    ARMING_DISABLED_ANGLE_BIT           = 8,
    ARMING_DISABLED_BOOT_GRACE_TIME_BIT = 9,
    ARMING_DISABLED_NOPREARM_BIT        = 10,
    ARMING_DISABLED_LOAD_BIT            = 11,
    ARMING_DISABLED_CALIBRATING_BIT     = 12,
    ARMING_DISABLED_CLI_BIT             = 13,
    ARMING_DISABLED_CMS_MENU_BIT        = 14,
    ARMING_DISABLED_BST_BIT             = 15,
    ARMING_DISABLED_MSP_BIT             = 16,
    ARMING_DISABLED_PARALYZE_BIT        = 17,
    ARMING_DISABLED_GPS_BIT             = 18,
    ARMING_DISABLED_RESC_BIT            = 19,
    ARMING_DISABLED_DSHOT_TELEM_BIT     = 20,
    ARMING_DISABLED_REBOOT_REQUIRED_BIT = 21,
    ARMING_DISABLED_DSHOT_BITBANG_BIT   = 22,
    ARMING_DISABLED_ACC_CALIBRATION_BIT = 23,
    ARMING_DISABLED_MOTOR_PROTOCOL_BIT  = 24,
    ARMING_DISABLED_UNUSED_25_BIT       = 25,
    ARMING_DISABLED_ALTHOLD_BIT         = 26,
    ARMING_DISABLED_POSHOLD_BIT         = 27,

    // Needs to be the last element, since it's always activated if one of the others is active when arming
    ARMING_DISABLED_ARM_SWITCH_BIT      = 31,

    ARMING_DISABLE_FLAGS_COUNT
} armingDisableBits_e;

#define ENTRY(NAME)   ARMING_DISABLED_##NAME = BIT(ARMING_DISABLED_##NAME##_BIT)
typedef enum {
    ENTRY(NO_GYRO),
    ENTRY(FAILSAFE),
    ENTRY(RX_FAILSAFE),
    ENTRY(NOT_DISARMED),
    ENTRY(BOXFAILSAFE),
    ENTRY(UNUSED_5),
    ENTRY(UNUSED_6),
    ENTRY(THROTTLE),
    ENTRY(ANGLE),
    ENTRY(BOOT_GRACE_TIME),
    ENTRY(NOPREARM),
    ENTRY(LOAD),
    ENTRY(CALIBRATING),
    ENTRY(CLI),
    ENTRY(CMS_MENU),
    ENTRY(BST),
    ENTRY(MSP),
    ENTRY(PARALYZE),
    ENTRY(GPS),
    ENTRY(RESC),
    ENTRY(DSHOT_TELEM),
    ENTRY(REBOOT_REQUIRED),
    ENTRY(DSHOT_BITBANG),
    ENTRY(ACC_CALIBRATION),
    ENTRY(MOTOR_PROTOCOL),
    ENTRY(UNUSED_25),
    ENTRY(ALTHOLD),
    ENTRY(POSHOLD),
    ENTRY(ARM_SWITCH)
} armingDisableFlags_e;
#undef ENTRY

void setArmingDisabled(armingDisableFlags_e flag);
void unsetArmingDisabled(armingDisableFlags_e flag);
bool isArmingDisabled(void);

armingDisableFlags_e getArmingDisableFlags(void);
const char *getArmingDisableFlagName(armingDisableFlags_e flag);

typedef enum {
    ARMED_MODE_BIT       = 0,
    ANGLE_MODE_BIT       = 1,
    HORIZON_MODE_BIT     = 2,
    ALT_HOLD_MODE_BIT    = 3,
    POS_HOLD_MODE_BIT    = 4,
    GPS_RESCUE_MODE_BIT  = 5,
    FAILSAFE_MODE_BIT    = 6,
} flightModeBits_e;

typedef enum {
    ARMED_MODE           = BIT(ARMED_MODE_BIT),
    ANGLE_MODE           = BIT(ANGLE_MODE_BIT),
    HORIZON_MODE         = BIT(HORIZON_MODE_BIT),
    ALT_HOLD_MODE        = BIT(ALT_HOLD_MODE_BIT),
    POS_HOLD_MODE        = BIT(POS_HOLD_MODE_BIT),
    FAILSAFE_MODE        = BIT(FAILSAFE_MODE_BIT),
    GPS_RESCUE_MODE      = BIT(GPS_RESCUE_MODE_BIT),
} flightModeFlags_e;

extern uint16_t flightModeFlags;

#define DISABLE_FLIGHT_MODE(mask) disableFlightMode(mask)
#define ENABLE_FLIGHT_MODE(mask) enableFlightMode(mask)
#define FLIGHT_MODE(mask) (flightModeFlags & (mask))

// macro to initialize map from boxId_e flightModeBits. Keep it in sync with flightModeFlags_e enum.
// [BOXARM] is left unpopulated
#define BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER {           \
   [BOXANGLE]       = ANGLE_MODE_BIT,                    \
   [BOXHORIZON]     = HORIZON_MODE_BIT,                  \
   [BOXALTHOLD]     = ALT_HOLD_MODE_BIT,                 \
   [BOXPOSHOLD]     = POS_HOLD_MODE_BIT,                 \
   [BOXFAILSAFE]    = FAILSAFE_MODE_BIT,                 \
   [BOXGPSRESCUE]   = GPS_RESCUE_MODE_BIT,               \
}                                                        \
/**/

typedef enum {
    GPS_FIX_HOME   = BIT(0),
    GPS_FIX        = BIT(1),
    GPS_FIX_EVER   = BIT(2),
} stateFlags_t;

#define DISABLE_STATE(mask) (stateFlags &= ~(mask))
#define ENABLE_STATE(mask) (stateFlags |= (mask))
#define STATE(mask) (stateFlags & (mask))

extern uint8_t stateFlags;

uint16_t enableFlightMode(flightModeFlags_e mask);
uint16_t disableFlightMode(flightModeFlags_e mask);

bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);

