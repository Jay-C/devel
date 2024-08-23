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

#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/blackbox.h"

#ifndef DEFAULT_BLACKBOX_DEVICE
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_NONE
#endif


PG_REGISTER_WITH_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 3);

PG_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig,
    .fields_disabled_mask = 0,
    .sample_rate = BLACKBOX_RATE_QUARTER,
    .device = DEFAULT_BLACKBOX_DEVICE,
    .mode = BLACKBOX_MODE_NORMAL,
    .high_resolution = false
);
