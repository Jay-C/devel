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

typedef struct {
    uint16_t    imu_dcm_kp;         // DCM filter proportional gain ( x 10000)
    uint16_t    imu_dcm_ki;         // DCM filter integral gain ( x 10000)
    uint8_t     imu_process_denom;
    uint16_t    mag_declination;    // Magnetic declination in degrees * 10
} imuConfig_t;

PG_DECLARE(imuConfig_t, imuConfig);
