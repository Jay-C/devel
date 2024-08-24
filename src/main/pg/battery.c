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

#include "sensors/battery.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/battery.h"

#ifndef DEFAULT_CURRENT_METER_SOURCE
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_NONE
#endif

#ifndef DEFAULT_VOLTAGE_METER_SOURCE
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_NONE
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 3);

PG_RESET_TEMPLATE(batteryConfig_t, batteryConfig,
    // voltage
    .vbatmaxcellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MAX,
    .vbatmincellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MIN,
    .vbatwarningcellvoltage = 350,
    .vbatnotpresentcellvoltage = 300, //A cell below 3 will be ignored
    .voltageMeterSource = DEFAULT_VOLTAGE_METER_SOURCE,
    .lvcPercentage = 100, //Off by default at 100%

    // current
    .batteryCapacity = 0,
    .currentMeterSource = DEFAULT_CURRENT_METER_SOURCE,

    // cells
    .forceBatteryCellCount = 0, //0 will be ignored

    // warnings / alerts
    .useVBatAlerts = true,
    .useConsumptionAlerts = false,
    .consumptionWarningPercentage = 10,
    .vbathysteresis = 1, // 0.01V

    .vbatfullcellvoltage = 410,

    .vbatDisplayLpfPeriod = 30,
    .vbatSagLpfPeriod = 2,
    .ibatLpfPeriod = 10,
    .vbatDurationForWarning = 0,
    .vbatDurationForCritical = 0,
);
