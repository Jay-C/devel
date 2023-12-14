/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/adc.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"

#include "io/beeper.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "scheduler/scheduler.h"

#include "sensors/battery.h"


/**
 * terminology: meter vs sensors
 *
 * voltage and current sensors are used to collect data.
 * - e.g. voltage at an MCU ADC input pin, value from an ESC sensor.
 *   sensors require very specific configuration, such as resistor values.
 *
 * voltage and current meters are used to process and expose data collected from sensors to the rest of the system.
 * - e.g. a meter exposes normalized, and often filtered, values from a sensor.
 *   meters require different or little configuration.
 *   meters also have different precision concerns, and may use different units to the sensors.
 *
 */

#ifndef DEFAULT_CURRENT_METER_SOURCE
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_NONE
#endif

#ifndef DEFAULT_VOLTAGE_METER_SOURCE
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_NONE
#endif

#define VBAT_STABLE_MAX_DELTA           200         // mV
#define LVC_AFFECT_TIME                 10000000    // 10 secs for the LVC to slowly kick in


PG_REGISTER_WITH_RESET_TEMPLATE(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 3);

PG_RESET_TEMPLATE(batteryConfig_t, batteryConfig,
    .forceBatteryCellCount = 0,
    .voltageMeterSource = DEFAULT_VOLTAGE_METER_SOURCE,
    .currentMeterSource = DEFAULT_CURRENT_METER_SOURCE,
    .vbatmaxcellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MAX,
    .vbatmincellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MIN,
    .vbatfullcellvoltage = 410,
    .vbatwarningcellvoltage = 350,
    .vbatnotpresentcellvoltage = 300,
    .vbathysteresis = 1,
    .lvcPercentage = 100, // Off by default at 100%
    .batteryCapacity = 0,
    .consumptionWarningPercentage = 10,
    .useVoltageAlerts = true,
    .useConsumptionAlerts = false,
    .vbatDurationForWarning = 0,
    .vbatDurationForCritical = 0,
    .vbatLpfPeriod = 30,
    .ibatLpfPeriod = 10,
    .vbatUpdateHz = VOLTAGE_TASK_FREQ_HZ,
    .ibatUpdateHz = CURRENT_TASK_FREQ_HZ,
);


const char * const batteryVoltageSourceNames[VOLTAGE_METER_COUNT] = {
    [VOLTAGE_METER_NONE]    = "NONE",
    [VOLTAGE_METER_ADC]     = "ADC",
    [VOLTAGE_METER_ESC]     = "ESC",
};

const char * const batteryCurrentSourceNames[CURRENT_METER_COUNT] = {
    [CURRENT_METER_NONE]    = "NONE",
    [CURRENT_METER_ADC]     = "ADC",
    [CURRENT_METER_ESC]     = "ESC",
};


// Note: Cell count can be 0 when no battery is detected or
//       when the battery voltage sensor is missing or disabled
static uint8_t batteryCellCount;

static uint16_t batteryWarningVoltage;
static uint16_t batteryCriticalVoltage;
static uint16_t batteryWarningHysteresisVoltage;
static uint16_t batteryCriticalHysteresisVoltage;

static lowVoltageCutoff_t lowVoltageCutoff;

static currentMeter_t currentMeter;
static voltageMeter_t voltageMeter;

static batteryState_e batteryState;
static batteryState_e voltageState;
static batteryState_e consumptionState;


/** Access function **/

const lowVoltageCutoff_t *getLowVoltageCutoff(void)
{
    return &lowVoltageCutoff;
}

bool isBatteryVoltageConfigured(void)
{
    return batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE;
}

const voltageMeter_t * getBatteryVoltageMeter()
{
    return &voltageMeter;
}

uint16_t getBatteryVoltage(void)
{
    return voltageMeter.voltage / 10;
}

uint16_t getLegacyBatteryVoltage(void)
{
    return voltageMeter.voltage / 100;
}

uint16_t getBatteryVoltageSample(void)
{
    return voltageMeter.sample / 10;
}

uint8_t getBatteryCellCount(void)
{
    return batteryCellCount;
}

uint16_t getBatteryAverageCellVoltage(void)
{
    return (batteryCellCount > 0) ? getBatteryVoltage() / batteryCellCount : 0;
}

bool isBatteryCurrentConfigured(void)
{
    return batteryConfig()->currentMeterSource != CURRENT_METER_NONE;
}

const currentMeter_t * getBatteryCurrentMeter()
{
    return &currentMeter;
}

uint16_t getBatteryCurrent(void) {
    return currentMeter.current / 10;
}

uint16_t getLegacyBatteryCurrent(void)
{
    return currentMeter.current / 100;
}

uint16_t getBatteryCurrentSample(void)
{
    return currentMeter.sample / 10;
}

uint32_t getBatteryCapacityUsed(void)
{
    return currentMeter.capacity;
}

batteryState_e getBatteryState(void)
{
    return batteryState;
}

batteryState_e getVoltageState(void)
{
    return voltageState;
}

batteryState_e getConsumptionState(void)
{
    return consumptionState;
}

static const char * const batteryStateStrings[] = { "OK", "WARNING", "CRITICAL", "NOT PRESENT", "INIT" };

const char * getBatteryStateString(void)
{
    return batteryStateStrings[getBatteryState()];
}


uint8_t calculateBatteryPercentageRemaining(void)
{
    uint8_t batteryPercentage = 0;

    if (batteryCellCount > 0) {
        uint16_t batteryCapacity = batteryConfig()->batteryCapacity;

        if (batteryCapacity > 0) {
            batteryPercentage = constrain((batteryCapacity - currentMeter.capacity) * 100 / batteryCapacity, 0, 100);
        } else {
            batteryPercentage = constrain((((uint32_t)getBatteryVoltage() - (batteryConfig()->vbatmincellvoltage * batteryCellCount)) * 100) / ((batteryConfig()->vbatmaxcellvoltage - batteryConfig()->vbatmincellvoltage) * batteryCellCount), 0, 100);
        }
    }

    return batteryPercentage;
}


/** Internal functions **/

static void updateBatteryBeeperAlert(void)
{
    switch (getBatteryState()) {
        case BATTERY_WARNING:
            beeper(BEEPER_BAT_LOW);
            break;
        case BATTERY_CRITICAL:
            beeper(BEEPER_BAT_CRIT_LOW);
            break;
        case BATTERY_OK:
        case BATTERY_NOT_PRESENT:
        case BATTERY_INIT:
            break;
    }
}

static void batteryUpdateAlarms(void)
{
    // use the state to trigger beeper alerts
    if (batteryConfig()->useVoltageAlerts) {
        updateBatteryBeeperAlert();
    }
}

static bool isVoltageStable(void)
{
    return ABS(voltageMeter.voltage - voltageMeter.sample) <= VBAT_STABLE_MAX_DELTA;
}

static bool isVoltageFromBat(void)
{
    const uint32_t voltage = getBatteryVoltage();

    // We want to disable battery getting detected around USB voltage or 0V
    return (voltage >= batteryConfig()->vbatnotpresentcellvoltage         // Above ~0V
            && voltage <= batteryConfig()->vbatmaxcellvoltage)            // 1s max cell voltage check
            || voltage > batteryConfig()->vbatnotpresentcellvoltage * 2;  // USB voltage - 2s or more check
}

void batteryUpdatePresence(void)
{
    if ((voltageState == BATTERY_NOT_PRESENT || voltageState == BATTERY_INIT) && isVoltageFromBat() && isVoltageStable()) {
        // Battery has just been connected - calculate cells, warning voltages and reset state
        consumptionState = voltageState = BATTERY_OK;

        if (batteryConfig()->forceBatteryCellCount != 0) {
            batteryCellCount = batteryConfig()->forceBatteryCellCount;
        } else {
            unsigned cells = (getBatteryVoltage() / batteryConfig()->vbatmaxcellvoltage) + 1;
            if (cells > MAX_AUTO_DETECT_CELL_COUNT) {
                // something is wrong, we expect MAX_CELL_COUNT cells maximum (and autodetection will be problematic at 6+ cells)
                cells = MAX_AUTO_DETECT_CELL_COUNT;
            }
            batteryCellCount = cells;
        }

        batteryWarningVoltage = batteryCellCount * batteryConfig()->vbatwarningcellvoltage;
        batteryCriticalVoltage = batteryCellCount * batteryConfig()->vbatmincellvoltage;
        batteryWarningHysteresisVoltage = (batteryWarningVoltage > batteryConfig()->vbathysteresis) ? batteryWarningVoltage - batteryConfig()->vbathysteresis : 0;
        batteryCriticalHysteresisVoltage = (batteryCriticalVoltage > batteryConfig()->vbathysteresis) ? batteryCriticalVoltage - batteryConfig()->vbathysteresis : 0;
        lowVoltageCutoff.percentage = 100;
        lowVoltageCutoff.startTime = 0;
    }
    else if (voltageState != BATTERY_NOT_PRESENT && isVoltageStable() && !isVoltageFromBat()) {
        // battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of batteryConfig()->vbatnotpresentcellvoltage
        consumptionState = voltageState = BATTERY_NOT_PRESENT;

        batteryCellCount = 0;
        batteryWarningVoltage = 0;
        batteryCriticalVoltage = 0;
        batteryWarningHysteresisVoltage = 0;
        batteryCriticalHysteresisVoltage = 0;
    }
}

static void batteryUpdateVoltageState(void)
{
    // alerts are currently used by beeper, osd and other subsystems
    static uint32_t lastVoltageChangeMs;

    const uint32_t voltage = getBatteryVoltage();

    switch (voltageState) {
        case BATTERY_OK:
            if (voltage <= batteryWarningHysteresisVoltage) {
                if (cmp32(millis(), lastVoltageChangeMs) >= batteryConfig()->vbatDurationForWarning * 100) {
                    voltageState = BATTERY_WARNING;
                }
            } else {
                lastVoltageChangeMs = millis();
            }
            break;

        case BATTERY_WARNING:
            if (voltage <= batteryCriticalHysteresisVoltage) {
                if (cmp32(millis(), lastVoltageChangeMs) >= batteryConfig()->vbatDurationForCritical * 100) {
                    voltageState = BATTERY_CRITICAL;
                }
            } else {
                if (voltage > batteryWarningVoltage) {
                    voltageState = BATTERY_OK;
                }
                lastVoltageChangeMs = millis();
            }
            break;

        case BATTERY_CRITICAL:
            if (voltage > batteryCriticalVoltage) {
                voltageState = BATTERY_WARNING;
                lastVoltageChangeMs = millis();
            }
            break;

        default:
            break;
    }

}

static void batteryUpdateLVC(timeUs_t currentTimeUs)
{
    if (batteryConfig()->lvcPercentage < 100) {
        if (voltageState == BATTERY_CRITICAL && !lowVoltageCutoff.enabled) {
            lowVoltageCutoff.enabled = true;
            lowVoltageCutoff.startTime = currentTimeUs;
            lowVoltageCutoff.percentage = 100;
        }
        if (lowVoltageCutoff.enabled) {
            if (cmp32(currentTimeUs,lowVoltageCutoff.startTime) < LVC_AFFECT_TIME) {
                lowVoltageCutoff.percentage = 100 - (cmp32(currentTimeUs,lowVoltageCutoff.startTime) * (100 - batteryConfig()->lvcPercentage) / LVC_AFFECT_TIME);
            }
            else {
                lowVoltageCutoff.percentage = batteryConfig()->lvcPercentage;
            }
        }
    }
}

static void batteryUpdateConsumptionState(void)
{
    if (batteryConfig()->useConsumptionAlerts && batteryConfig()->batteryCapacity > 0 && batteryCellCount > 0) {
        uint8_t batteryPercentageRemaining = calculateBatteryPercentageRemaining();

        if (batteryPercentageRemaining == 0) {
            consumptionState = BATTERY_CRITICAL;
        } else if (batteryPercentageRemaining <= batteryConfig()->consumptionWarningPercentage) {
            consumptionState = BATTERY_WARNING;
        } else {
            consumptionState = BATTERY_OK;
        }
    }
}

static void batteryUpdateStates(timeUs_t currentTimeUs)
{
    batteryUpdateVoltageState();
    batteryUpdateConsumptionState();
    batteryUpdateLVC(currentTimeUs);

    batteryState = MAX(voltageState, consumptionState);
}


/** Battery Alert Task **/

void taskBatteryAlerts(timeUs_t currentTimeUs)
{
    if (!ARMING_FLAG(ARMED)) {
        // the battery *might* fall out in flight, but if that happens the FC will likely be off too unless the user has battery backup.
        batteryUpdatePresence();
    }

    batteryUpdateStates(currentTimeUs);
    batteryUpdateAlarms();
}


/** Battery Voltage Task **/

void taskBatteryVoltageUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    voltageSensorADCRefresh();

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        voltageSensorESCRefresh();
    }
#endif

    switch (batteryConfig()->voltageMeterSource) {
        case VOLTAGE_METER_ADC:
            voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BAT, &voltageMeter);
            break;
        case VOLTAGE_METER_ESC:
#ifdef USE_ESC_SENSOR
            if (featureIsEnabled(FEATURE_ESC_SENSOR))
                voltageSensorESCReadTotal(&voltageMeter);
#endif
            break;
    }

    DEBUG(BATTERY, 0, voltageMeter.sample);
    DEBUG(BATTERY, 1, voltageMeter.voltage);
}


/** Battery Current Task **/

void taskBatteryCurrentUpdate(timeUs_t currentTimeUs)
{
    currentSensorADCRefresh(currentTimeUs);

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        currentSensorESCRefresh();
    }
#endif

    switch (batteryConfig()->currentMeterSource) {
        case CURRENT_METER_ADC:
            currentSensorADCRead(CURRENT_SENSOR_ADC_BAT, &currentMeter);
            break;

        case CURRENT_METER_ESC:
#ifdef USE_ESC_SENSOR
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                currentSensorESCReadTotal(&currentMeter);
            }
#endif
            break;
    }

    DEBUG(BATTERY, 2, currentMeter.sample);
    DEBUG(BATTERY, 3, currentMeter.current);
}


void batteryInit(void)
{
    voltageMeterReset(&voltageMeter);
    currentMeterReset(&currentMeter);

    voltageSensorADCInit();
    currentSensorADCInit();

#ifdef USE_ESC_SENSOR
    voltageSensorESCInit();
    currentSensorESCInit();
#endif

    // presence
    batteryState = BATTERY_INIT;
    batteryCellCount = 0;

    // voltage
    voltageState = BATTERY_INIT;
    batteryWarningVoltage = 0;
    batteryCriticalVoltage = 0;
    batteryWarningHysteresisVoltage = 0;
    batteryCriticalHysteresisVoltage = 0;
    lowVoltageCutoff.enabled = false;
    lowVoltageCutoff.percentage = 100;
    lowVoltageCutoff.startTime = 0;

    // current
    consumptionState = BATTERY_OK;
}

