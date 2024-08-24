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

#include "types.h"

#include "pg/pg.h"

#include "drivers/io_types.h"

#define SERIAL_PORT_MAX_INDEX 11

typedef struct serialPinConfig_s {
    ioTag_t ioTagTx[SERIAL_PORT_MAX_INDEX];
    ioTag_t ioTagRx[SERIAL_PORT_MAX_INDEX];
    ioTag_t ioTagInverter[SERIAL_PORT_MAX_INDEX];
} serialPinConfig_t;

PG_DECLARE(serialPinConfig_t, serialPinConfig);


#if defined(USE_SOFTSERIAL)

#define SOFTSERIAL_COUNT 2

typedef struct softSerialPinConfig_s {
    ioTag_t ioTagTx[SOFTSERIAL_COUNT];
    ioTag_t ioTagRx[SOFTSERIAL_COUNT];
} softSerialPinConfig_t;

PG_DECLARE(softSerialPinConfig_t, softSerialPinConfig);

#endif
