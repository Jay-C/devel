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

#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "platform.h"

#include "build/debug.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/motor.h"
#include "drivers/time.h"

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "flight/alt_hold.h"
#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "mixer.h"


float FAST_DATA_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];

float motor_disarmed[MAX_SUPPORTED_MOTORS];


void writeMotors(void)
{
}

void stopMotors(void)
{
}

float getMotorOutputLow(void)
{
    return 0;
}

float getMotorOutputHigh(void)
{
    return 1;
}

bool areMotorsRunning(void)
{
    return true;
}

bool hasServos(void)
{
    return true;
}

uint8_t getMotorCount(void)
{
    return MAX_SUPPORTED_MOTORS;
}

void mixerResetDisarmedMotors(void)
{
}

void initEscEndpoints(void)
{
    //motorInitEndpoints(motorConfig(), 1, &mixerRuntime.motorOutputLow, &mixerRuntime.motorOutputHigh, &mixerRuntime.disarmMotorOutput);
}

float mixerGetThrottle(void)
{
    return 0;
}

float mixerGetRcThrottle(void)
{
    return 0;
}

void mixerInit(void)
{
    initEscEndpoints();
}

