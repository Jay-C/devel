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

#include "config/config_reset.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/alignment.h"

#ifndef DEFAULT_ALIGN_BOARD_ROLL
#define DEFAULT_ALIGN_BOARD_ROLL 0
#endif
#ifndef DEFAULT_ALIGN_BOARD_PITCH
#define DEFAULT_ALIGN_BOARD_PITCH 0
#endif
#ifndef DEFAULT_ALIGN_BOARD_YAW
#define DEFAULT_ALIGN_BOARD_YAW 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(boardAlignment_t, boardAlignment, PG_BOARD_ALIGNMENT, 1);

PG_RESET_TEMPLATE(boardAlignment_t, boardAlignment,
    .rollDegrees = DEFAULT_ALIGN_BOARD_ROLL,
    .pitchDegrees = DEFAULT_ALIGN_BOARD_PITCH,
    .yawDegrees = DEFAULT_ALIGN_BOARD_YAW,
);

