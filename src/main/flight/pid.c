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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config_reset.h"

#include "pg/pg.h"
#include "pg/pid.h"
#include "pg/pg_ids.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot_command.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/rc_rates.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/trainer.h"
#include "flight/leveling.h"
#include "flight/rpm_filter.h"

#include "pid.h"


static FAST_DATA_ZERO_INIT pid_t pid;


float pidGetDT()
{
    return pid.dT;
}

float pidGetPidFrequency()
{
    return pid.freq;
}

float pidGetSetpoint(int axis)
{
    return pid.data[axis].setPoint;
}

float pidGetOutput(int axis)
{
    return pid.data[axis].pidSum;
}

float pidGetCollective(void)
{
    return pid.collective;
}

const pidAxisData_t * pidGetAxisData(void)
{
    return pid.data;
}

void INIT_CODE pidReset(void)
{
    memset(pid.data, 0, sizeof(pid.data));
}

void INIT_CODE pidResetAxisError(int axis)
{
    pid.data[axis].I = 0;
    pid.data[axis].axisError = 0;
}

void INIT_CODE pidResetAxisErrors(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pid.data[axis].I = 0;
        pid.data[axis].axisError = 0;
    }
}


static void INIT_CODE pidSetLooptime(uint32_t pidLooptime)
{
    pid.dT = pidLooptime * 1e-6f;
    pid.freq = 1.0f / pid.dT;

#ifdef USE_DSHOT
    dshotSetPidLoopTime(pidLooptime);
#endif
}

void INIT_CODE pidInit(const pidProfile_t *pidProfile)
{
    pidSetLooptime(gyro.targetLooptime);
    pidInitProfile(pidProfile);
}

void INIT_CODE pidInitProfile(const pidProfile_t *pidProfile)
{
    pid.mode = constrain(pidProfile->mode, 1, 6);

    // Roll axis
    pid.coef[PID_ROLL].Kp = ROLL_P_TERM_SCALE * pidProfile->pid[PID_ROLL].P;
    pid.coef[PID_ROLL].Ki = ROLL_I_TERM_SCALE * pidProfile->pid[PID_ROLL].I;
    pid.coef[PID_ROLL].Kd = ROLL_D_TERM_SCALE * pidProfile->pid[PID_ROLL].D;
    pid.coef[PID_ROLL].Kf = ROLL_F_TERM_SCALE * pidProfile->pid[PID_ROLL].F;

    // Pitch axis
    pid.coef[PID_PITCH].Kp = PITCH_P_TERM_SCALE * pidProfile->pid[PID_PITCH].P;
    pid.coef[PID_PITCH].Ki = PITCH_I_TERM_SCALE * pidProfile->pid[PID_PITCH].I;
    pid.coef[PID_PITCH].Kd = PITCH_D_TERM_SCALE * pidProfile->pid[PID_PITCH].D;
    pid.coef[PID_PITCH].Kf = PITCH_F_TERM_SCALE * pidProfile->pid[PID_PITCH].F;

    // Yaw axis
    pid.coef[PID_YAW].Kp = YAW_P_TERM_SCALE * pidProfile->pid[PID_YAW].P;
    pid.coef[PID_YAW].Ki = YAW_I_TERM_SCALE * pidProfile->pid[PID_YAW].I;
    pid.coef[PID_YAW].Kd = YAW_D_TERM_SCALE * pidProfile->pid[PID_YAW].D;
    pid.coef[PID_YAW].Kf = YAW_F_TERM_SCALE * pidProfile->pid[PID_YAW].F;

    // Yaw alt. axis
    pid.coef[PID_WAY].Kp = YAW_P_TERM_SCALE * pidProfile->pid[PID_WAY].P;
    pid.coef[PID_WAY].Ki = YAW_I_TERM_SCALE * pidProfile->pid[PID_WAY].I;
    pid.coef[PID_WAY].Kd = YAW_D_TERM_SCALE * pidProfile->pid[PID_WAY].D;
    pid.coef[PID_WAY].Kf = YAW_F_TERM_SCALE * pidProfile->pid[PID_WAY].F;

    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        pid.errorLimit[i] = constrain(pidProfile->error_limit[i], 0, 360);

    pid.errorDecay = 1.0f - (pidProfile->error_decay) ? pid.dT * 10 / pidProfile->error_decay : 0;

#if 0
    // Collective impulse high-pass filter
    pid.precomp.collectiveImpulseFilterGain = pt1FilterGain(pidProfile->yaw_collective_ff_impulse_freq / 100.0f, dT);

    // Tail/yaw parameters
    pid.precomp.yawCyclicFFGain = pidProfile->yaw_cyclic_ff_gain;
    pid.precomp.yawCollectiveFFGain = pidProfile->yaw_collective_ff_gain;
    pid.precomp.yawCollectiveImpulseFFGain = pidProfile->yaw_collective_ff_impulse_gain;

    // Pitch parameters
    pid.precomp.pitchCollectiveFFGain = pidProfile->pitch_collective_ff_gain;
    pid.precomp.pitchCollectiveImpulseFFGain = pidProfile->pitch_collective_ff_impulse_gain;
#endif

#ifdef USE_ACC
    pidLevelInit(pidProfile);
#endif
#ifdef USE_ACRO_TRAINER
    acroTrainerInit(pidProfile);
#endif
}

void INIT_CODE pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT &&
        dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}


/*
 * 2D Rotation matrix
 *
 *        | cos(r)   -sin r |
 *    R = |                 |
 *        | sin(r)    cos r |
 *
 *
 *                3     5     7     9
 *               x     x     x     x
 * sin(x) = x - --- + --- - --- + --- - ...
 *               3!    5!    7!    9!
 *
 *                2     4     6     8
 *               x     x     x     x
 * cos(x) = 1 - --- + --- - --- + --- - ...
 *               2!    4!    6!    8!
 *
 *
 * For very small values of x, sin(x) ~= x and cos(x) ~= 1.
 *
 * In the use case below, using two first terms gives nearly 24bits of
 * resolution, which is close to what can be stored in a float.
 */

static inline void rotateAxisError(void)
{
    if (pid.errorRotation) {
        const float x = pid.data[PID_ROLL].axisError;
        const float y = pid.data[PID_PITCH].axisError;
        const float r = gyro.gyroADCf[Z] * RAD * pid.dT;

        const float c = 1 - r * r / 2;
        const float s = r * (1 - r * r / 6);

        pid.data[PID_ROLL].axisError  = x * c + y * s;
        pid.data[PID_PITCH].axisError = y * c - x * s;
    }
}

static inline float pidApplySetpoint(const pidProfile_t *pidProfile, uint8_t axis)
{
    UNUSED(pidProfile);

    // Rate setpoint
    float setpoint = getRcSetpoint(axis);

#ifdef USE_ACC
    if (axis == PID_ROLL || axis == PID_PITCH) {
        // Apply leveling
        if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)) {
            setpoint = pidLevelApply(axis, setpoint);
        }
#ifdef USE_ACRO_TRAINER
        else {
            // Apply trainer
            setpoint = acroTrainerApply(axis, setpoint);
        }
#endif
    }
#endif

    // Save setpoint
    pid.data[axis].setPoint = setpoint;

    return setpoint;
}

static inline void pidApplyCollective(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);

    pid.collective = getRcSetpoint(FD_COLL);
}

static FAST_CODE void pidApplyPrecomp(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);

    // Yaw precompensation direction
    const float rotSign = mixerRotationSign();

    // Get stick throws (from previous cycle)
    const float cyclicDeflection = getCyclicDeflection();
    const float collectiveDeflection = getCollectiveDeflection();

    // Collective pitch impulse filter - TODO replace with proper filter
    pid.precomp.collectiveDeflectionLPF += (collectiveDeflection - pid.precomp.collectiveDeflectionLPF) * pid.precomp.collectiveImpulseFilterGain;
    const float collectiveDeflectionHPF = collectiveDeflection - pid.precomp.collectiveDeflectionLPF;

    // Pitch precomp
    const float pitchCollectiveFF = collectiveDeflection * pid.precomp.pitchCollectiveFFGain;
    const float pitchCollectiveImpulseFF = collectiveDeflectionHPF * pid.precomp.pitchCollectiveImpulseFFGain;

    // Total pitch precomp
    const float pitchPrecomp = pitchCollectiveFF + pitchCollectiveImpulseFF;

    // Add to PITCH feedforward
    pid.data[FD_PITCH].F += pitchPrecomp;
    pid.data[FD_PITCH].pidSum += pitchPrecomp;

    DEBUG(PITCH_PRECOMP, 0, collectiveDeflection * 1000);
    DEBUG(PITCH_PRECOMP, 1, collectiveDeflectionHPF * 1000);
    DEBUG(PITCH_PRECOMP, 2, pitchCollectiveFF * 10);
    DEBUG(PITCH_PRECOMP, 3, pitchCollectiveImpulseFF * 10);
    DEBUG(PITCH_PRECOMP, 4, pitchPrecomp * 10);

    // Collective components
    const float yawCollectiveFF = fabsf(collectiveDeflection) * pid.precomp.yawCollectiveFFGain;
    const float yawCollectiveImpulseFF = fabsf(collectiveDeflectionHPF) * pid.precomp.yawCollectiveImpulseFFGain;

    // Cyclic component
    float yawCyclicFF = fabsf(cyclicDeflection) * pid.precomp.yawCyclicFFGain;

    // Calculate total precompensation
    float yawPrecomp = (yawCollectiveFF + yawCollectiveImpulseFF + yawCyclicFF) * rotSign;

    // Add to YAW feedforward
    pid.data[FD_YAW].F += yawPrecomp;
    pid.data[FD_YAW].pidSum += yawPrecomp;

    DEBUG(YAW_PRECOMP, 0, collectiveDeflection * 1000);
    DEBUG(YAW_PRECOMP, 1, collectiveDeflectionHPF * 1000);
    DEBUG(YAW_PRECOMP, 2, yawCollectiveFF * 10);
    DEBUG(YAW_PRECOMP, 3, yawCollectiveImpulseFF * 10);
    DEBUG(YAW_PRECOMP, 4, cyclicDeflection * 1000);
    DEBUG(YAW_PRECOMP, 5, yawCyclicFF * 10);
    DEBUG(YAW_PRECOMP, 6, yawPrecomp * 10);
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 1 - SAME AS RF-1.0
 **
 **   gyro ADC => errorFilter => Kp => P-term
 **   gyro dterm ADC => Kd => D-term
 **   gyro ADC => Relax => Ki => I-term
 **
 **   Using gyro-only D-term
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static FAST_CODE void pidApplyCyclicMode1(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;


  //// P-term

    // P-term
    float pTerm = errorRate;

    // Limit P bandwidth
    if (pidProfile->error_cutoff[axis]) {
        pTerm = pt1FilterApply(&pid.errorFilter[axis], pTerm);
    }

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * pTerm;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dError = -gyro.gyroDtermADCf[axis];
    float dTerm = (dError - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = dError;

    // No D if axis saturated
    if (pidAxisSaturated(axis)) {
        dTerm = 0;
    }

    // D-term filter
    //dTerm = pt1FilterApply(&pid.dtermFilter[axis], dTerm);

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        dTerm = 0;
    }

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // I-term change
    float itermDelta = errorRate * pid.dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        itermDelta = 0;
    }

    // Calculate I-component
    pid.data[axis].axisError = constrainf(pid.data[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


static FAST_CODE void pidApplyYawMode1(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;


  //// P-term

    // P-term
    float pTerm = errorRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        pTerm = pt1FilterApply(&pid.errorFilter[axis], pTerm);
    }

    // Select stop gain
    float stopGain = (pTerm > 0) ? pid.yawCWStopGain : pid.yawCCWStopGain;

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * pTerm * stopGain;


  //// D-term

    // Calculate D-term from filtered gyro signal
    float dError = -gyro.gyroDtermADCf[axis];
    float dTerm = (dError - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = dError;

    // No D if axis saturated
    if (pidAxisSaturated(axis)) {
        dTerm = 0;
    }

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm * stopGain;


  //// I-term

    // Apply I-term relax
#ifdef USE_ITERM_RELAX_XXX
    if (pid.itermRelax) {
        errorRate = applyItermRelax(axis, pid.coef[axis].Ki * pid.data[axis].axisError, errorRate, gyroRate, setpoint);
    }
#endif

    // I-term change
    float itermDelta = errorRate * pid.dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        itermDelta = 0;
    }

    // Calculate I-component
    pid.data[axis].axisError = constrainf(pid.data[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 2
 **
 **   gyro ADC => errorFilter => Kp => P-term
 **   gyro ADC => errorFilter => dtermFilter => Kd => D-term
 **   gyro ADC => errorFilter => Ki => I-term
 **
 **   Yaw Stop gain on D only
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static FAST_CODE void pidApplyCyclicMode2(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&pid.errorFilter[axis], errorRate);
    }


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dTerm = (errorRate - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = errorRate;

    // Filter D-term
    dTerm = pt1FilterApply(&pid.dtermFilter[axis],  dTerm);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // I-term change
    float itermDelta = errorRate * pid.dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        if (pid.data[axis].axisError * itermDelta > 0) // Same sign and not zero
            itermDelta = 0;
    }

    // Calculate I-component
    pid.data[axis].axisError = constrainf(pid.data[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    float fTerm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fTerm -= pt1FilterApply(&pid.ftermFilter[axis], setpoint);
    }
    pid.data[axis].F = pid.coef[axis].Kf * fTerm;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


static FAST_CODE void pidApplyYawMode2(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&pid.errorFilter[axis], errorRate);
    }

    // Select stop gain
    float stopGain = (errorRate > 0) ? pid.yawCWStopGain : pid.yawCCWStopGain;


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dTerm = (errorRate - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = errorRate;

    // Filter D-term * stopGain
    dTerm = pt1FilterApply(&pid.dtermFilter[axis], dTerm * stopGain);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // I-term change
    float itermDelta = errorRate * pid.dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        if (pid.data[axis].axisError * itermDelta > 0) // Same sign and not zero
            itermDelta = 0;
    }

    // Calculate I-component
    pid.data[axis].axisError = constrainf(pid.data[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    float fTerm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fTerm -= pt1FilterApply(&pid.ftermFilter[axis], setpoint);
    }
    pid.data[axis].F = pid.coef[axis].Kf * fTerm;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 6
 **
 **   gyro ADC => errorFilter => Kp => P-term
 **   gyro ADC => errorFilter => dtermFilter => Kd => D-term
 **   gyro ADC => errorFilter => Ki => I-term
 **
 **   Separate gains for yaw CW & CCW
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static FAST_CODE void pidApplyCyclicMode6(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&pid.errorFilter[axis], errorRate);
    }

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].I * errorRate > 0);


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dTerm = (errorRate - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = errorRate;

    // Filter D-term
    dTerm = pt1FilterApply(&pid.dtermFilter[axis],  dTerm);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // I-term change
    float itermDelta = errorRate * pid.dT;

    // No accumulation if axis saturated
    if (saturation) {
        itermDelta = 0;
    }

    // Calculate I-component
    pid.data[axis].axisError = constrainf(pid.data[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * pid.data[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    float fTerm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fTerm -= pt1FilterApply(&pid.ftermFilter[axis], setpoint);
    }
    pid.data[axis].F = pid.coef[axis].Kf * fTerm;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


static FAST_CODE void pidApplyYawMode6(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = pidApplySetpoint(pidProfile, axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&pid.errorFilter[axis], errorRate);
    }

    // Select direction
    const uint8_t index = (errorRate > 0) ? PID_YAW : PID_WAY;

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].I * errorRate > 0);


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[index].Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dTerm = (errorRate - pid.data[axis].prevError) * pid.freq;
    pid.data[axis].prevError = errorRate;

    // Filter D-term * Kd
    dTerm = pt1FilterApply(&pid.dtermFilter[axis], pid.coef[index].Kd * dTerm);

    // Calculate D-component
    pid.data[axis].D = dTerm;


  //// I-term

    // I-term change
    float itermDelta = errorRate * pid.dT;

    // No accumulation if axis saturated
    if (saturation) {
        itermDelta = 0;
    }

    // Calculate I-component
    const float minI = pid.coef[PID_WAY].Ki * -pid.errorLimit[axis];
    const float maxI = pid.coef[PID_YAW].Ki *  pid.errorLimit[axis];
    pid.data[axis].I = constrainf(pid.data[axis].I + pid.coef[index].Ki * itermDelta, minI, maxI);

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.data[axis].I *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    float fTerm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fTerm -= pt1FilterApply(&pid.ftermFilter[axis], setpoint);
    }
    pid.data[axis].F = pid.coef[axis].Kf * fTerm;


  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D + pid.data[axis].F;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

FAST_CODE void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Rotate pitch/roll axis error with yaw rotation
    rotateAxisError();

    // Calculate stabilized collective
    pidApplyCollective(pidProfile);

    // Apply PID for each axis
    switch (pid.mode) {
        case 6:
            pidApplyCyclicMode6(pidProfile, PID_ROLL);
            pidApplyCyclicMode6(pidProfile, PID_PITCH);
            pidApplyYawMode6(pidProfile);
            break;
        case 2:
            pidApplyCyclicMode2(pidProfile, PID_ROLL);
            pidApplyCyclicMode2(pidProfile, PID_PITCH);
            pidApplyYawMode2(pidProfile);
            break;
        default:
            pidApplyCyclicMode1(pidProfile, PID_ROLL);
            pidApplyCyclicMode1(pidProfile, PID_PITCH);
            pidApplyYawMode1(pidProfile);
            break;
    }

    // Calculate cyclic/collective precompensation
    pidApplyPrecomp(pidProfile);

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}
