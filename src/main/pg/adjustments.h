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

#include "fc/rc_modes.h"

#define MAX_ADJUSTMENT_RANGE_COUNT 30

typedef struct {
    // when aux channel is in range...
    uint8_t auxChannelIndex;
    channelRange_t range;

    // ..then apply the adjustment function to the auxSwitchChannel ...
    uint8_t adjustmentConfig;
    uint8_t auxSwitchChannelIndex;

    uint16_t adjustmentCenter;
    uint16_t adjustmentScale;

} adjustmentRange_t;

PG_DECLARE_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges);
