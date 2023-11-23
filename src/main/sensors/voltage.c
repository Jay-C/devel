/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "stdbool.h"
#include "stdint.h"
#include "string.h"

#include "platform.h"

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/adc.h"

#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/adcinternal.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "voltage.h"

const char * const voltageMeterSourceNames[VOLTAGE_METER_COUNT] = {
    "NONE", "ADC", "ESC"
};

const uint8_t voltageMeterIds[] = {
    VOLTAGE_METER_ID_BATTERY,
    VOLTAGE_METER_ID_BEC,
    VOLTAGE_METER_ID_5V,
    VOLTAGE_METER_ID_3V3,
#ifdef USE_ESC_SENSOR
    VOLTAGE_METER_ID_ESC_COMBINED,
    VOLTAGE_METER_ID_ESC_1,
    VOLTAGE_METER_ID_ESC_2,
    VOLTAGE_METER_ID_ESC_3,
    VOLTAGE_METER_ID_ESC_4,
#endif
};

const uint8_t supportedVoltageMeterCount = ARRAYLEN(voltageMeterIds);


//
// ADC/ESC shared
//

void voltageMeterReset(voltageMeter_t *meter)
{
    meter->displayFiltered = 0;
    meter->unfiltered = 0;
}
//
// ADC
//

#ifndef VBAT_SCALE_DEFAULT
#define VBAT_SCALE_DEFAULT 110
#endif

#ifndef VBAT_RESDIVVAL_DEFAULT
#define VBAT_RESDIVVAL_DEFAULT 10
#endif

#ifndef VBAT_RESDIVMULTIPLIER_DEFAULT
#define VBAT_RESDIVMULTIPLIER_DEFAULT 1
#endif

typedef struct voltageMeterADCState_s {
    uint16_t voltageDisplayFiltered;         // battery voltage in 0.01V steps (filtered)
    uint16_t voltageUnfiltered;       // battery voltage in 0.01V steps (unfiltered)
    filter_t displayFilter;
} voltageMeterADCState_t;

voltageMeterADCState_t voltageMeterADCStates[MAX_VOLTAGE_SENSOR_ADC];

voltageMeterADCState_t *getVoltageMeterADC(uint8_t index)
{
    return &voltageMeterADCStates[index];
}

PG_REGISTER_ARRAY_WITH_RESET_FN(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig, PG_VOLTAGE_SENSOR_ADC_CONFIG, 0);

void pgResetFn_voltageSensorADCConfig(voltageSensorADCConfig_t *instance)
{
    for (int i = 0; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
        RESET_CONFIG(voltageSensorADCConfig_t, &instance[i],
            .vbatscale = VBAT_SCALE_DEFAULT,
            .vbatresdivval = VBAT_RESDIVVAL_DEFAULT,
            .vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT,
        );
    }
}


static const uint8_t voltageMeterAdcChannelMap[] = {
    ADC_BATTERY,
    ADC_EXT1,
    ADC_EXT2,
    ADC_EXT3,
};

STATIC_UNIT_TESTED uint16_t voltageAdcToVoltage(const uint16_t src, const voltageSensorADCConfig_t *config)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.01V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 10:1 voltage divider (10k:1k) * 100 for 0.01V
    return ((((uint32_t)src * config->vbatscale * getVrefMv() / 10 + (0xFFF * 5)) / (0xFFF * config->vbatresdivval)) / config->vbatresdivmultiplier);
}

void voltageMeterADCRefresh(void)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_SENSOR_ADC && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        voltageMeterADCState_t *state = &voltageMeterADCStates[i];
#ifdef USE_ADC
        // store the battery voltage with some other recent battery voltage readings

        const voltageSensorADCConfig_t *config = voltageSensorADCConfig(i);

        uint8_t channel = voltageMeterAdcChannelMap[i];
        uint16_t rawSample = adcGetChannel(channel);
        uint16_t filteredDisplaySample = filterApply(&state->displayFilter, rawSample);

        // always calculate the latest voltage, see getLatestVoltage() which does the calculation on demand.
        state->voltageDisplayFiltered = voltageAdcToVoltage(filteredDisplaySample, config);
        state->voltageUnfiltered = voltageAdcToVoltage(rawSample, config);

#else
        UNUSED(voltageAdcToVoltage);

        state->voltageDisplayFiltered = 0;
        state->voltageUnfiltered = 0;
#endif
    }
}

void voltageMeterADCRead(voltageSensorADC_e adcChannel, voltageMeter_t *voltageMeter)
{
    voltageMeterADCState_t *state = &voltageMeterADCStates[adcChannel];

    voltageMeter->displayFiltered = state->voltageDisplayFiltered;
    voltageMeter->unfiltered = state->voltageUnfiltered;
}

void voltageMeterADCInit(void)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_SENSOR_ADC && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        // store the battery voltage with some other recent battery voltage readings

        voltageMeterADCState_t *state = &voltageMeterADCStates[i];
        memset(state, 0, sizeof(voltageMeterADCState_t));

        lowpassFilterInit(&state->displayFilter, LPF_BESSEL, GET_BATTERY_LPF_FREQUENCY(batteryConfig()->vbatDisplayLpfPeriod), batteryConfig()->vbatUpdateHz, 0);
    }
}

void voltageMeterGenericInit(void)
{

}

//
// ESC
//

#ifdef USE_ESC_SENSOR
typedef struct voltageMeterESCState_s {
    uint16_t voltageDisplayFiltered;         // battery voltage in 0.01V steps (filtered)
    uint16_t voltageUnfiltered;       // battery voltage in 0.01V steps (unfiltered)
    filter_t displayFilter;
} voltageMeterESCState_t;

static voltageMeterESCState_t voltageMeterESCState;
#endif



void voltageMeterESCInit(void)
{
#ifdef USE_ESC_SENSOR
    memset(&voltageMeterESCState, 0, sizeof(voltageMeterESCState_t));
    lowpassFilterInit(&voltageMeterESCState.displayFilter, LPF_BESSEL, GET_BATTERY_LPF_FREQUENCY(batteryConfig()->vbatDisplayLpfPeriod), batteryConfig()->vbatUpdateHz, 0);
#endif
}

void voltageMeterESCRefresh(void)
{
#ifdef USE_ESC_SENSOR
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData) {
        voltageMeterESCState.voltageUnfiltered = escData->dataAge <= ESC_BATTERY_AGE_MAX ? escData->voltage : 0;
        voltageMeterESCState.voltageDisplayFiltered = filterApply(&voltageMeterESCState.displayFilter, voltageMeterESCState.voltageUnfiltered);
    }
#endif
}

void voltageMeterESCReadMotor(uint8_t motorNumber, voltageMeter_t *voltageMeter)
{
#ifndef USE_ESC_SENSOR
    UNUSED(motorNumber);
    voltageMeterReset(voltageMeter);
#else
    escSensorData_t *escData = getEscSensorData(motorNumber);
    if (escData) {
        voltageMeter->unfiltered = escData->dataAge <= ESC_BATTERY_AGE_MAX ? escData->voltage : 0;
        voltageMeter->displayFiltered = voltageMeter->unfiltered; // no filtering for ESC motors currently.
    } else {
        voltageMeterReset(voltageMeter);
    }

#endif
}

void voltageMeterESCReadCombined(voltageMeter_t *voltageMeter)
{
#ifndef USE_ESC_SENSOR
    voltageMeterReset(voltageMeter);
#else
    voltageMeter->displayFiltered = voltageMeterESCState.voltageDisplayFiltered;
    voltageMeter->unfiltered = voltageMeterESCState.voltageUnfiltered;
#endif
}

//
// API for using voltage meters using IDs
//
// This API is used by MSP, for configuration/status.
//


// the order of these much match the indexes in voltageSensorADC_e
const uint8_t voltageMeterADCtoIDMap[MAX_VOLTAGE_SENSOR_ADC] = {
    VOLTAGE_METER_ID_BATTERY,
    VOLTAGE_METER_ID_BEC,
    VOLTAGE_METER_ID_5V,
    VOLTAGE_METER_ID_3V3,
};

void voltageMeterRead(voltageMeterId_e id, voltageMeter_t *meter)
{
    if (id == VOLTAGE_METER_ID_BATTERY) {
        voltageMeterADCRead(VOLTAGE_SENSOR_ADC_VBAT, meter);
    } else
    if (id == VOLTAGE_METER_ID_BEC) {
        voltageMeterADCRead(VOLTAGE_SENSOR_ADC_VX, meter);
    } else
    if (id == VOLTAGE_METER_ID_5V) {
        voltageMeterADCRead(VOLTAGE_SENSOR_ADC_5V, meter);
    } else
    if (id == VOLTAGE_METER_ID_3V3) {
        voltageMeterADCRead(VOLTAGE_SENSOR_ADC_3V3, meter);
    } else
#ifdef USE_ESC_SENSOR
    if (id == VOLTAGE_METER_ID_ESC_COMBINED) {
        voltageMeterESCReadCombined(meter);
    } else
    if (id >= VOLTAGE_METER_ID_ESC_1 && id <= VOLTAGE_METER_ID_ESC_1 + 3) {
        int motor = id - VOLTAGE_METER_ID_ESC_1;
        voltageMeterESCReadMotor(motor, meter);
    } else
#endif
    {
        voltageMeterReset(meter);
    }
}
