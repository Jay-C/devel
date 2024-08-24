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

#include "drivers/io_types.h"
#include "drivers/serial_resource.h"

typedef struct serialPinConfig_s {
    ioTag_t ioTagTx[RESOURCE_SERIAL_COUNT];
    ioTag_t ioTagRx[RESOURCE_SERIAL_COUNT];
#ifdef USE_INVERTER
    ioTag_t ioTagInverter[RESOURCE_UART_COUNT];
#endif
} serialPinConfig_t;

PG_DECLARE(serialPinConfig_t, serialPinConfig);
