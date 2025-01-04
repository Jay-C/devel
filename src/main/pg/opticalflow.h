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
    OPTICALFLOW_NONE = 0,
    OPTICALFLOW_MT = 1,
} opticalflowType_e;

typedef struct {
    uint8_t  opticalflow_hardware;
    uint16_t rotation;
    uint8_t  flip_x;
    uint16_t flow_lpf;
} opticalflowConfig_t;

PG_DECLARE(opticalflowConfig_t, opticalflowConfig);

