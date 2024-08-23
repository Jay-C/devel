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

// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100

typedef struct {
    uint8_t         startStep;
    uint8_t         endStep;
} channelRange_t;

typedef struct {
    uint8_t         modeId;
    uint8_t         auxChannelIndex;
    channelRange_t  range;
    uint8_t         modeLogic;
    uint8_t         linkedTo;
} modeActivationCondition_t;


#define MAX_MODE_ACTIVATION_CONDITION_COUNT 20

PG_DECLARE_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions);


#ifdef USE_CUSTOM_BOX_NAMES

#define MAX_BOX_USER_NAME_LENGTH 16
#define BOX_USER_NAME_COUNT      4

typedef struct {
    char box_user_names[BOX_USER_NAME_COUNT][MAX_BOX_USER_NAME_LENGTH];
} modeActivationConfig_t;

PG_DECLARE(modeActivationConfig_t, modeActivationConfig);

#endif
