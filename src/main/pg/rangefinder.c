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

#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/rangefinder.h"


PG_REGISTER_WITH_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig, PG_RANGEFINDER_CONFIG, 0);

PG_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig,
    .rangefinder_hardware = RANGEFINDER_NONE,
);

#ifdef USE_RANGEFINDER_HCSR04
PG_REGISTER_WITH_RESET_TEMPLATE(sonarConfig_t, sonarConfig, PG_SONAR_CONFIG, 1);

PG_RESET_TEMPLATE(sonarConfig_t, sonarConfig,
    .triggerTag = IO_TAG(RANGEFINDER_HCSR04_TRIGGER_PIN),
    .echoTag = IO_TAG(RANGEFINDER_HCSR04_ECHO_PIN),
);
#endif
