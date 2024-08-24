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

#pragma once

#include "common/filter.h"
#include "common/rtc.h"
#include "sensors/current.h"
#include "sensors/voltage.h"

#include "pg/battery.h"

//TODO: Make the 'cell full' voltage user adjustble
#define CELL_VOLTAGE_FULL_CV 420

#define VBAT_CELL_VOTAGE_RANGE_MIN 100
#define VBAT_CELL_VOTAGE_RANGE_MAX 500
#define VBAT_CELL_VOLTAGE_DEFAULT_MIN 330
#define VBAT_CELL_VOLTAGE_DEFAULT_MAX 430

#define MAX_AUTO_DETECT_CELL_COUNT 8

#define GET_BATTERY_LPF_FREQUENCY(period) (1 / (period / 10.0f))

typedef struct lowVoltageCutoff_s {
    bool enabled;
    uint8_t percentage;
    timeUs_t startTime;
} lowVoltageCutoff_t;

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT,
    BATTERY_INIT
} batteryState_e;

void batteryInit(void);
void batteryUpdateVoltage(timeUs_t currentTimeUs);
void batteryUpdatePresence(void);

bool isVoltageFromBattery(void);

batteryState_e getBatteryState(void);
batteryState_e getVoltageState(void);
batteryState_e getConsumptionState(void);
const  char * getBatteryStateString(void);

void batteryUpdateStates(timeUs_t currentTimeUs);
void batteryUpdateAlarms(void);

struct rxConfig_s;

uint8_t calculateBatteryPercentageRemaining(void);
bool isBatteryVoltageConfigured(void);
uint16_t getBatteryVoltage(void);
uint16_t getLegacyBatteryVoltage(void);
uint16_t getBatteryVoltageLatest(void);
uint8_t getBatteryCellCount(void);
uint16_t getBatteryAverageCellVoltage(void);
uint16_t getBatterySagCellVoltage(void);

bool isAmperageConfigured(void);
int32_t getAmperage(void);
int32_t getAmperageLatest(void);
int32_t getMAhDrawn(void);
float getWhDrawn(void);
#ifdef USE_BATTERY_CONTINUE
bool hasUsedMAh(void);
void setMAhDrawn(uint32_t mAhDrawn);
#endif

void batteryUpdateCurrentMeter(timeUs_t currentTimeUs);

const lowVoltageCutoff_t *getLowVoltageCutoff(void);
