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

#include <stdint.h>

#include "drivers/rangefinder/rangefinder.h"

#include "pg/rangefinder.h"

typedef struct rangefinder_s {
    rangefinderDev_t dev;
    float maxTiltCos;
    int32_t rawAltitude;
    int32_t calculatedAltitude;
    timeMs_t lastValidResponseTimeMs;

    bool snrThresholdReached;
    int32_t dynamicDistanceThreshold;
    int16_t snr;
} rangefinder_t;

void rangefinderResetDynamicThreshold(void);
bool rangefinderInit(void);

int32_t rangefinderGetLatestAltitude(void);
int32_t rangefinderGetLatestRawAltitude(void);

void rangefinderUpdate(void);
bool rangefinderProcess(float cosTiltAngle);
bool rangefinderIsHealthy(void);
