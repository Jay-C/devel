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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/time.h"

#include "pg/pg.h"

#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

extern float motor[MAX_SUPPORTED_MOTORS];
extern float motor_disarmed[MAX_SUPPORTED_MOTORS];

uint8_t getMotorCount(void);
float getMotorMixRange(void);
bool areMotorsRunning(void);
bool areMotorsSaturated(void);

void initEscEndpoints(void);
void mixerResetDisarmedMotors(void);
void mixTable(timeUs_t currentTimeUs);
void stopMotors(void);
void writeMotors(void);

float mixerGetThrottle(void);
float getMotorOutputLow(void);
float getMotorOutputHigh(void);
