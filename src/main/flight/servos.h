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

#include "pg/pg.h"
#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

#define PWM_SERVO_MIN   500       // minimum servo PWM pulse width which we can set from cli
#define PWM_SERVO_MAX   2500      // maximum servo PWM pulse width which we can set from cli

#define DEFAULT_SERVO_MIN 1000
#define DEFAULT_SERVO_MIDDLE 1500
#define DEFAULT_SERVO_MAX 2000

#define MAX_SERVO_SPEED UINT8_MAX
#define MAX_SERVO_BOXES 3

typedef struct servoParam_s {
    uint32_t reversedSources;               // the direction of servo movement for each input source of the servo mixer, bit set=inverted
    int16_t min;                            // servo min
    int16_t max;                            // servo max
    int16_t middle;                         // servo middle
    int8_t rate;                            // range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
} servoParam_t;

PG_DECLARE_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams);

typedef struct servoConfig_s {
    servoDevConfig_t dev;
} servoConfig_t;

PG_DECLARE(servoConfig_t, servoConfig);

typedef struct servoProfile_s {
    servoParam_t servoConf[MAX_SUPPORTED_SERVOS];
} servoProfile_t;

extern int16_t servo[MAX_SUPPORTED_SERVOS];

void writeServos(void);
void servosInit(void);
void servosFilterInit(void);
