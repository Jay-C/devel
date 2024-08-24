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

#include "pg/pid.h"

#ifndef DEFAULT_PID_PROCESS_DENOM
#define DEFAULT_PID_PROCESS_DENOM   1
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 4);

PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = DEFAULT_PID_PROCESS_DENOM,
);

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 8);

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles);
