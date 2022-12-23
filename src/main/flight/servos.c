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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/pwm_output.h"

#include "sensors/gyro.h"

#include "fc/runtime_config.h"

#include "flight/servos.h"
#include "flight/mixer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"


static FAST_DATA_ZERO_INIT uint8_t      servoCount;

static FAST_DATA_ZERO_INIT uint16_t     servoOutput[MAX_SUPPORTED_SERVOS];
static FAST_DATA_ZERO_INIT int16_t      servoOverride[MAX_SUPPORTED_SERVOS];

static FAST_DATA_ZERO_INIT timerChannel_t servoChannel[MAX_SUPPORTED_SERVOS];


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    for (unsigned i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoConfig->ioTags[i] = timerioTagGetByUsage(TIM_USE_SERVO, i);
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
                     .mid   = DEFAULT_SERVO_CENTER,
                     .min   = DEFAULT_SERVO_MIN,
                     .max   = DEFAULT_SERVO_MAX,
                     .rneg  = DEFAULT_SERVO_RANGE,
                     .rpos  = DEFAULT_SERVO_RANGE,
                     .rate  = DEFAULT_SERVO_RATE,
                     .flags = DEFAULT_SERVO_FLAGS,
        );
    }
}


uint8_t getServoCount(void)
{
    return servoCount;
}

uint16_t getServoOutput(uint8_t servo)
{
    return servoOutput[servo];
}

bool hasServoOverride(uint8_t servo)
{
    return (servoOverride[servo] >= SERVO_OVERRIDE_MIN && servoOverride[servo] <= SERVO_OVERRIDE_MAX);
}

int16_t getServoOverride(uint8_t servo)
{
    return servoOverride[servo];
}

int16_t setServoOverride(uint8_t servo, int16_t val)
{
    return servoOverride[servo] = val;
}

void validateAndFixServoConfig(void)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
#ifndef USE_SERVO_GEOMETRY_CORRECTION
        servoParamsMutable(i)->flags &= ~SERVO_FLAG_GEO_CORR;
#endif
    }
}

void servoInit(void)
{
    const ioTag_t *ioTags = servoConfig()->ioTags;
    const timerHardware_t *timer[MAX_SUPPORTED_SERVOS];
    uint8_t index;

    for (index = 0; index < MAX_SUPPORTED_SERVOS; index++)
    {
        servoOutput[index] = servoParams(index)->mid;
        servoOverride[index] = SERVO_OVERRIDE_OFF;
    }

    for (index = 0; index < MAX_SUPPORTED_SERVOS && ioTags[index]; index++)
    {
        const ioTag_t tag = ioTags[index];
        const IO_t io = IOGetByTag(tag);

        timer[index] = timerAllocate(tag, OWNER_SERVO, RESOURCE_INDEX(index));

        if (!timer[index])
            break;

        IOInit(io, OWNER_SERVO, RESOURCE_INDEX(index));
        IOConfigGPIOAF(io, IOCFG_AF_PP, timer[index]->alternateFunction);
    }

    servoCount = index;

    for (index = 0; index < servoCount; index++)
    {
        int rate = servoParams(index)->rate;
        int max = servoParams(index)->max;

        for (int jndex = 0; jndex < servoCount; jndex++) {
            if (timer[index]->tim == timer[jndex]->tim) {
                if (servoParams(jndex)->rate < rate)
                    rate = servoParams(jndex)->rate;
                if (servoParams(jndex)->max > max)
                    max = servoParams(jndex)->max;
            }
        }

        rate = MAX(rate, SERVO_RATE_MIN);
        rate = MIN(rate, 1000000 / (max + 10)); // At least 10us low

        pwmOutConfig(&servoChannel[index], timer[index], PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / rate, 0, 0);
    }
}

static inline void servoWrite(uint8_t index, uint16_t value)
{
    if (servoChannel[index].ccr)
        *servoChannel[index].ccr = value;
}

static inline float limitTravel(uint8_t servo, float pos, float min, float max)
{
    if (pos > max) {
        mixerSaturateServoOutput(servo);
        return max;
    } else if (pos < min) {
        mixerSaturateServoOutput(servo);
        return min;
    }
    return pos;
}

#ifdef USE_SERVO_GEOMETRY_CORRECTION
static float geometryCorrection(float pos)
{
    // 1.0 == 50° without correction
    float height = constrainf(pos * 0.7660444431f, -1, 1);

    // Scale 50° in rad => 1.0
    float rotation = asin_approx(height) * 1.14591559026f;

    return rotation;
}
#endif

void servoUpdate(void)
{
    for (int i = 0; i < servoCount; i++)
    {
        const servoParam_t *servo = servoParams(i);
        float pos = mixerGetServoOutput(i);

#ifdef USE_SERVO_GEOMETRY_CORRECTION
        if (servo->flags & SERVO_FLAG_GEO_CORR)
            pos = geometryCorrection(pos);
#endif

        if (!ARMING_FLAG(ARMED) && hasServoOverride(i))
            pos = servoOverride[i] / 1000.0f;

        if (servo->flags & SERVO_FLAG_REVERSED)
            pos = -pos;

        float rate = (pos > 0) ? servo->rpos : servo->rneg;

        pos = servo->mid + rate * pos;

        pos = limitTravel(i, pos, servo->min, servo->max);

        servoOutput[i] = lrintf(pos);

        servoWrite(i, servoOutput[i]);
    }
}

#endif
