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

typedef enum  {
    BLACKBOX_DEVICE_NONE = 0,
    BLACKBOX_DEVICE_FLASH = 1,
    BLACKBOX_DEVICE_SDCARD = 2,
    BLACKBOX_DEVICE_SERIAL = 3
} BlackboxDevice_e;

typedef enum {
    BLACKBOX_MODE_NORMAL = 0,
} BlackboxMode;

typedef enum {
    BLACKBOX_RATE_ONE = 0,
    BLACKBOX_RATE_HALF,
    BLACKBOX_RATE_QUARTER,
    BLACKBOX_RATE_8TH,
    BLACKBOX_RATE_16TH
} BlackboxSampleRate_e;

typedef struct {
    uint32_t fields_disabled_mask;
    uint8_t sample_rate;
    uint8_t device;
    uint8_t mode;
    uint8_t high_resolution;
} blackboxConfig_t;

PG_DECLARE(blackboxConfig_t, blackboxConfig);
