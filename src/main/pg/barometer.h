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

#include "drivers/barometer/barometer.h"


typedef struct {
    uint8_t baro_busType;
    uint8_t baro_spi_device;
    ioTag_t baro_spi_csn;                   // Also used as XCLR (positive logic) for BMP085
    uint8_t baro_i2c_device;
    uint8_t baro_i2c_address;
    uint8_t baro_hardware;                  // Barometer hardware to use
    ioTag_t baro_eoc_tag;
    ioTag_t baro_xclr_tag;
} barometerConfig_t;

PG_DECLARE(barometerConfig_t, barometerConfig);

