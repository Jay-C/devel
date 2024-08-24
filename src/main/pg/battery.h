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

#include "platform.h"

#include "pg/pg.h"

typedef struct {
    // voltage
    uint16_t vbatmaxcellvoltage;            // maximum voltage per cell, used for auto-detecting battery voltage in 0.01V units, default is 430 (4.30V)
    uint16_t vbatmincellvoltage;            // minimum voltage per cell, this triggers battery critical alarm, in 0.01V units, default is 330 (3.30V)
    uint16_t vbatwarningcellvoltage;        // warning voltage per cell, this triggers battery warning alarm, in 0.01V units, default is 350 (3.50V)
    uint16_t vbatnotpresentcellvoltage;     // Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
    uint8_t lvcPercentage;                  // Percentage of throttle when lvc is triggered
    uint8_t voltageMeterSource;             // source of battery voltage meter used, either ADC or ESC

    // current
    uint8_t currentMeterSource;             // source of battery current meter used, either ADC, Virtual or ESC
    uint16_t batteryCapacity;               // mAh

    // warnings / alerts
    bool useVBatAlerts;                     // Issue alerts based on VBat readings
    bool useConsumptionAlerts;              // Issue alerts based on total power consumption
    uint8_t consumptionWarningPercentage;   // Percentage of remaining capacity that should trigger a battery warning
    uint8_t vbathysteresis;                 // hysteresis for alarm in 0.01V units, default 1 = 0.01V

    uint16_t vbatfullcellvoltage;           // Cell voltage at which the battery is deemed to be "full" 0.01V units, default is 410 (4.1V)

    uint8_t forceBatteryCellCount;          // Number of cells in battery, used for overwriting auto-detected cell count if someone has issues with it.
    uint8_t vbatDisplayLpfPeriod;           // Period of the cutoff frequency for the Vbat filter for display and startup (in 0.1 s)
    uint8_t ibatLpfPeriod;                  // Period of the cutoff frequency for the Ibat filter (in 0.1 s)
    uint8_t vbatDurationForWarning;         // Period voltage has to sustain before the battery state is set to BATTERY_WARNING (in 0.1 s)
    uint8_t vbatDurationForCritical;        // Period voltage has to sustain before the battery state is set to BATTERY_CRIT (in 0.1 s)
    uint8_t vbatSagLpfPeriod;               // Period of the cutoff frequency for the Vbat sag and PID compensation filter (in 0.1 s)

#ifdef USE_BATTERY_CONTINUE
    bool isBatteryContinueEnabled;
#endif
} batteryConfig_t;

PG_DECLARE(batteryConfig_t, batteryConfig);

