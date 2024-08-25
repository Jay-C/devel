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

#include "common/color.h"

#include "drivers/io_types.h"

#define LED_CONFIGURABLE_COLOR_COUNT   16
#define LED_MODE_COUNT                  6
#define LED_DIRECTION_COUNT             6
#define LED_BASEFUNCTION_COUNT          10
#define LED_OVERLAY_COUNT               7
#define LED_SPECIAL_COLOR_COUNT        11

typedef struct {
    ioTag_t     ioTag;
    uint8_t     ledstrip_visual_beeper;
    uint8_t     ledstrip_grb_rgb;
    uint8_t     ledstrip_profile;
    uint8_t     ledstrip_race_color;
    uint8_t     ledstrip_beacon_color;
    uint16_t    ledstrip_beacon_period_ms;
    uint8_t     ledstrip_beacon_percent;
    uint8_t     ledstrip_beacon_armed_only;
    uint8_t     ledstrip_visual_beeper_color;
    uint8_t     ledstrip_brightness;
    uint16_t    ledstrip_rainbow_delta;
    uint16_t    ledstrip_rainbow_freq;
} ledStripConfig_t;

PG_DECLARE(ledStripConfig_t, ledStripConfig);

typedef uint32_t ledConfig_t;

typedef struct {
    uint8_t color[LED_DIRECTION_COUNT];
} modeColorIndexes_t;

typedef struct {
    uint8_t color[LED_SPECIAL_COLOR_COUNT];
} specialColorIndexes_t;

typedef struct {
    ledConfig_t ledConfigs[LED_STRIP_MAX_LENGTH];
    hsvColor_t colors[LED_CONFIGURABLE_COLOR_COUNT];
    modeColorIndexes_t modeColors[LED_MODE_COUNT];
    specialColorIndexes_t specialColors;
    uint8_t ledstrip_aux_channel;
} ledStripStatusModeConfig_t;

PG_DECLARE(ledStripStatusModeConfig_t, ledStripStatusModeConfig);
