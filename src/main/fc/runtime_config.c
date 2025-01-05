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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "fc/runtime_config.h"
#include "io/beeper.h"

uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;

static uint32_t enabledSensors = 0;

#define ENTRY(_NAME)   [ARMING_DISABLED_ ## _NAME ## _BIT] = #_NAME
const char  * const armingDisableFlagNames[ARMING_DISABLE_FLAGS_COUNT] = {
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
    ENTRY(ARM_SWITCH),
};
#undef ENTRY

static armingDisableFlags_e armingDisableFlags = 0;

void setArmingDisabled(armingDisableFlags_e flag)
{
    armingDisableFlags = armingDisableFlags | flag;
}

void unsetArmingDisabled(armingDisableFlags_e flag)
{
    armingDisableFlags = armingDisableFlags & ~flag;
}

bool isArmingDisabled(void)
{
    return armingDisableFlags;
}

armingDisableFlags_e getArmingDisableFlags(void)
{
    return armingDisableFlags;
}

// return name for given flag
// will return first name (LSB) if multiple bits are passed
const char *getArmingDisableFlagName(armingDisableFlags_e flag)
{
    if (!flag) {
        return "NONE";
    }
    unsigned idx = ffs(flag & -flag) - 1;   // use LSB if there are multiple bits set
    return idx < ARRAYLEN(armingDisableFlagNames) ? armingDisableFlagNames[idx] : "UNKNOWN";
}

/**
 * Enables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint16_t enableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

/**
 * Disables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint16_t disableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;

    flightModeFlags &= ~(mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}
