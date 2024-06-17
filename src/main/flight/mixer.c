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

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer_init.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "io/gps.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "mixer.h"

#define DYN_LPF_THROTTLE_STEPS           100
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US 5000 // minimum of 5ms between updates

static FAST_DATA_ZERO_INIT float motorMixRange;

float FAST_DATA_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

float getMotorMixRange(void)
{
    return motorMixRange;
}

void writeMotors(void)
{
    motorWriteAll(motor);
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(mixerRuntime.disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

static FAST_DATA_ZERO_INIT float throttle = 0;
static FAST_DATA_ZERO_INIT float mixerThrottle = 0;
static FAST_DATA_ZERO_INIT float motorOutputMin;
static FAST_DATA_ZERO_INIT float motorRangeMin;
static FAST_DATA_ZERO_INIT float motorRangeMax;
static FAST_DATA_ZERO_INIT float motorOutputRange;
static FAST_DATA_ZERO_INIT int8_t motorOutputMixSign;

static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    static float motorRangeMinIncrease = 0;

    float currentThrottleInputRange = 0;
    {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN;
        currentThrottleInputRange = PWM_RANGE;
#ifdef USE_DYN_IDLE
        if (mixerRuntime.dynIdleMinRps > 0.0f) {
            const float maxIncrease = isAirmodeActivated()
                ? mixerRuntime.dynIdleMaxIncrease : mixerRuntime.dynIdleStartIncrease;
            float minRps = getMinMotorFrequencyHz();
            DEBUG_SET(DEBUG_DYN_IDLE, 3, lrintf(minRps * 10.0f));
            float rpsError = mixerRuntime.dynIdleMinRps - minRps;
            // PT1 type lowpass delay and smoothing for D
            minRps = mixerRuntime.prevMinRps + mixerRuntime.minRpsDelayK * (minRps - mixerRuntime.prevMinRps);
            float dynIdleD = (mixerRuntime.prevMinRps - minRps) * mixerRuntime.dynIdleDGain;
            mixerRuntime.prevMinRps = minRps;
            float dynIdleP = rpsError * mixerRuntime.dynIdlePGain;
            rpsError = MAX(-0.1f, rpsError); //I rises fast, falls slowly
            mixerRuntime.dynIdleI += rpsError * mixerRuntime.dynIdleIGain;
            mixerRuntime.dynIdleI = constrainf(mixerRuntime.dynIdleI, 0.0f, maxIncrease);
            motorRangeMinIncrease = constrainf((dynIdleP + mixerRuntime.dynIdleI + dynIdleD), 0.0f, maxIncrease);
            DEBUG_SET(DEBUG_DYN_IDLE, 0, MAX(-1000, lrintf(dynIdleP * 10000)));
            DEBUG_SET(DEBUG_DYN_IDLE, 1, lrintf(mixerRuntime.dynIdleI * 10000));
            DEBUG_SET(DEBUG_DYN_IDLE, 2, lrintf(dynIdleD * 10000));
        } else {
            motorRangeMinIncrease = 0;
        }
#endif

        motorRangeMax = mixerRuntime.motorOutputHigh;
        motorRangeMin = mixerRuntime.motorOutputLow + motorRangeMinIncrease * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
        motorOutputMin = motorRangeMin;
        motorOutputRange = motorRangeMax - motorRangeMin;
        motorOutputMixSign = 1;
    }

    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

#ifdef USE_RPM_LIMIT
#define STICK_HIGH_DEADBAND 5    // deadband to make sure throttle cap can raise, even with maxcheck set around 2000
static void applyRpmLimiter(mixerRuntime_t *mixer)
{
    static float prevError = 0.0f;
    const float unsmoothedAverageRpm = getDshotRpmAverage();
    const float averageRpm = pt1FilterApply(&mixer->rpmLimiterAverageRpmFilter, unsmoothedAverageRpm);
    const float error = averageRpm - mixer->rpmLimiterRpmLimit;

    // PID
    const float p = error * mixer->rpmLimiterPGain;
    const float d = (error - prevError) * mixer->rpmLimiterDGain; // rpmLimiterDGain already adjusted for looprate (see mixer_init.c)
    mixer->rpmLimiterI += error * mixer->rpmLimiterIGain;         // rpmLimiterIGain already adjusted for looprate (see mixer_init.c)
    mixer->rpmLimiterI = MAX(0.0f, mixer->rpmLimiterI);
    float pidOutput = p + mixer->rpmLimiterI + d;

    // Throttle limit learning
    if (error > 0.0f && rcCommand[THROTTLE] < rxConfig()->maxcheck) {
        mixer->rpmLimiterThrottleScale *= 1.0f - 4.8f * pidGetDT();
    } else if (pidOutput < -400.0f * pidGetDT() && lrintf(rcCommand[THROTTLE]) >= rxConfig()->maxcheck - STICK_HIGH_DEADBAND && !areMotorsSaturated()) { // Throttle accel corresponds with motor accel
        mixer->rpmLimiterThrottleScale *= 1.0f + 3.2f * pidGetDT();
    }
    mixer->rpmLimiterThrottleScale = constrainf(mixer->rpmLimiterThrottleScale, 0.01f, 1.0f);

    float rpmLimiterThrottleScaleOffset = pt1FilterApply(&mixer->rpmLimiterThrottleScaleOffsetFilter, constrainf(mixer->rpmLimiterRpmLimit / motorEstimateMaxRpm(), 0.0f, 1.0f) - mixer->rpmLimiterInitialThrottleScale);
    throttle *= constrainf(mixer->rpmLimiterThrottleScale + rpmLimiterThrottleScaleOffset, 0.0f, 1.0f);

    // Output
    pidOutput = MAX(0.0f, pidOutput);
    throttle = constrainf(throttle - pidOutput, 0.0f, 1.0f);
    prevError = error;

    DEBUG_SET(DEBUG_RPM_LIMIT, 0, lrintf(averageRpm));
    DEBUG_SET(DEBUG_RPM_LIMIT, 1, lrintf(rpmLimiterThrottleScaleOffset * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 2, lrintf(mixer->rpmLimiterThrottleScale * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 3, lrintf(throttle * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 4, lrintf(error));
    DEBUG_SET(DEBUG_RPM_LIMIT, 5, lrintf(p * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 6, lrintf(mixer->rpmLimiterI * 100.0f));
    DEBUG_SET(DEBUG_RPM_LIMIT, 7, lrintf(d * 100.0f));
}
#endif // USE_RPM_LIMIT

static void applyMixToMotors(const float motorMix[MAX_SUPPORTED_MOTORS], motorMixer_t *activeMixer)
{
    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float motorOutput = motorOutputMixSign * motorMix[i] + throttle * activeMixer[i].throttle;
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;

#ifdef USE_SERVOS
        if (mixerIsTricopter()) {
            motorOutput += mixerTricopterMotorCorrection(i);
        }
#endif
        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? mixerRuntime.disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrainf(motorOutput, mixerRuntime.disarmMotorOutput, motorRangeMax);
        } else {
            motorOutput = constrainf(motorOutput, motorRangeMin, motorRangeMax);
        }
        motor[i] = motorOutput;
    }

    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
    DEBUG_SET(DEBUG_EZLANDING, 1, throttle * 10000U);
    // DEBUG_EZLANDING 0 is the ezLanding factor 2 is the throttle limit
}

static void applyMotorStop(void)
{
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mixerRuntime.disarmMotorOutput;
    }
}

#ifdef USE_DYN_LPF
static void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static int dynLpfPreviousQuantizedThrottle = -1;  // to allow an initial zero throttle to set the filter cutoff

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
        const int quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); // quantize the throttle reduce the number of filter updates
        if (quantizedThrottle != dynLpfPreviousQuantizedThrottle) {
            // scale the quantized value back to the throttle range so the filter cutoff steps are repeatable
            const float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
            dynLpfGyroUpdate(dynLpfThrottle);
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
#endif

static void applyMixerAdjustmentLinear(float *motorMix, const bool airmodeEnabled)
{
    float airmodeTransitionPercent = 1.0f;
    float motorDeltaScale = 0.5f;

    if (!airmodeEnabled && throttle < 0.5f) {
        // this scales the motor mix authority to be 0.5 at 0 throttle, and 1.0 at 0.5 throttle as airmode off intended for things to work.
        // also lays the groundwork for how an airmode percent would work.
        airmodeTransitionPercent = scaleRangef(throttle, 0.0f, 0.5f, 0.5f, 1.0f); // 0.5 throttle is full transition, and 0.0 throttle is 50% airmodeTransitionPercent
        motorDeltaScale *= airmodeTransitionPercent; // this should be half of the motor authority allowed
    }

    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? airmodeTransitionPercent / motorMixRange : airmodeTransitionPercent;

    const float motorMixDelta = motorDeltaScale * motorMixRange;

    float minMotor = FLT_MAX;
    float maxMotor = FLT_MIN;

    for (int i = 0; i < mixerRuntime.motorCount; ++i) {
        if (mixerConfig()->mixer_type == MIXER_LINEAR) {
            motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + motorMixDelta, motorMix[i] - motorMixDelta);
        } else {
            motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + fabsf(motorMix[i]), motorMix[i] - fabsf(motorMix[i]));
        }
        motorMix[i] *= motorMixNormalizationFactor;

        maxMotor = MAX(motorMix[i], maxMotor);
        minMotor = MIN(motorMix[i], minMotor);
    }

    // constrain throttle so it won't clip any outputs
    throttle = constrainf(throttle, -minMotor, 1.0f - maxMotor);
}

static float calcEzLandLimit(float maxDeflection, float speed)
{
    // calculate limit to where the mixer can raise the throttle based on RPY stick deflection
    // 0.0 = no increas allowed, 1.0 = 100% increase allowed
    const float deflectionLimit = mixerRuntime.ezLandingThreshold > 0.0f ? fminf(1.0f, maxDeflection / mixerRuntime.ezLandingThreshold) : 0.0f;
    DEBUG_SET(DEBUG_EZLANDING, 4, lrintf(deflectionLimit * 10000.0f));

    // calculate limit to where the mixer can raise the throttle based on speed
    // TODO sanity checks like number of sats, dop, accuracy?
    const float speedLimit = mixerRuntime.ezLandingSpeed > 0.0f ? fminf(1.0f, speed / mixerRuntime.ezLandingSpeed) : 0.0f;
    DEBUG_SET(DEBUG_EZLANDING, 5, lrintf(speedLimit * 10000.0f));

    // get the highest of the limits from deflection, speed, and the base ez_landing_limit
    const float deflectionAndSpeedLimit = fmaxf(deflectionLimit, speedLimit);
    return fmaxf(mixerRuntime.ezLandingLimit, deflectionAndSpeedLimit);
}

static void applyMixerAdjustmentEzLand(float *motorMix, const float motorMixMin, const float motorMixMax)
{
    // Calculate factor for normalizing motor mix range to <= 1.0
    const float baseNormalizationFactor = motorMixRange > 1.0f ? 1.0f / motorMixRange : 1.0f;
    const float normalizedMotorMixMin = motorMixMin * baseNormalizationFactor;
    const float normalizedMotorMixMax = motorMixMax * baseNormalizationFactor;

#ifdef USE_GPS
    const float speed = STATE(GPS_FIX) ? gpsSol.speed3d / 100.0f : 0.0f;  // m/s
#else
    const float speed = 0.0f;
#endif

    const float ezLandLimit = calcEzLandLimit(getMaxRcDeflectionAbs(), speed);
    // use the largest of throttle and limit calculated from RPY stick positions
    float upperLimit = fmaxf(ezLandLimit, throttle);
    // limit throttle to avoid clipping the highest motor output
    upperLimit = fminf(upperLimit, 1.0f - normalizedMotorMixMax);

    // Lower throttle Limit
    const float epsilon = 1.0e-6f;  // add small value to avoid divisions by zero
    const float absMotorMixMin = fabsf(normalizedMotorMixMin) + epsilon;
    const float lowerLimit = fminf(upperLimit, absMotorMixMin);

    // represents how much motor values have to be scaled to avoid clipping
    const float ezLandFactor = upperLimit / absMotorMixMin;

    // scale motor values
    const float normalizationFactor = baseNormalizationFactor * fminf(1.0f, ezLandFactor);
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motorMix[i] *= normalizationFactor;
    }
    motorMixRange *= baseNormalizationFactor;
    // Make anti windup recognize reduced authority range
    motorMixRange = fmaxf(motorMixRange, 1.0f / ezLandFactor);

    // Constrain throttle
    throttle = constrainf(throttle, lowerLimit, upperLimit);

    // Log ezLandFactor, upper throttle limit, and ezLandFactor if throttle was zero
    DEBUG_SET(DEBUG_EZLANDING, 0, fminf(1.0f, ezLandFactor) * 10000U);
    // DEBUG_EZLANDING 1 is the adjusted throttle
    DEBUG_SET(DEBUG_EZLANDING, 2, upperLimit * 10000U);
    DEBUG_SET(DEBUG_EZLANDING, 3, fminf(1.0f, ezLandLimit / absMotorMixMin) * 10000U);
    // DEBUG_EZLANDING 4 and 5 is the upper limits based on stick input and speed respectively
}

static void applyMixerAdjustment(float *motorMix, const float motorMixMin, const float motorMixMax, const bool airmodeEnabled)
{
#ifdef USE_AIRMODE_LPF
    const float unadjustedThrottle = throttle;
    throttle += pidGetAirmodeThrottleOffset();
    float airmodeThrottleChange = 0.0f;
#endif
    float airmodeTransitionPercent = 1.0f;

    if (!airmodeEnabled && throttle < 0.5f) {
        // this scales the motor mix authority to be 0.5 at 0 throttle, and 1.0 at 0.5 throttle as airmode off intended for things to work.
        // also lays the groundwork for how an airmode percent would work.
        airmodeTransitionPercent = scaleRangef(throttle, 0.0f, 0.5f, 0.5f, 1.0f); // 0.5 throttle is full transition, and 0.0 throttle is 50% airmodeTransitionPercent
    }

    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? airmodeTransitionPercent / motorMixRange : airmodeTransitionPercent;

    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motorMix[i] *= motorMixNormalizationFactor;
    }

    const float normalizedMotorMixMin = motorMixMin * motorMixNormalizationFactor;
    const float normalizedMotorMixMax = motorMixMax * motorMixNormalizationFactor;
    throttle = constrainf(throttle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax);

#ifdef USE_AIRMODE_LPF
    airmodeThrottleChange = constrainf(unadjustedThrottle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax) - unadjustedThrottle;
    pidUpdateAirmodeLpf(airmodeThrottleChange);
#endif
}

FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs)
{
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    motorMixer_t * activeMixer = &mixerRuntime.currentMixer[0];

    // Calculate and Limit the PID sum
    const float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;

    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;

    float scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    // use scaled throttle, without dynamic idle throttle offset, as the input to antigravity
    pidUpdateAntiGravityThrottleFilter(throttle);

    // and for TPA
    pidUpdateTpaFactor(throttle);

#ifdef USE_DYN_LPF
    // keep the changes to dynamic lowpass clean, without unnecessary dynamic changes
    updateDynLpfCutoffs(currentTimeUs, throttle);
#endif

    // send throttle value to blackbox, including scaling and throttle boost, but not TL compensation, dyn idle or airmode
    mixerThrottle = throttle;

#ifdef USE_DYN_IDLE
    // Set min throttle offset of 1% when stick is at zero and dynamic idle is active
    if (mixerRuntime.dynIdleMinRps > 0.0f) {
        throttle = MAX(throttle, 0.01f);
    }
#endif

#ifdef USE_RPM_LIMIT
    if (RPM_LIMIT_ACTIVE && useDshotTelemetry && ARMING_FLAG(ARMED)) {
        applyRpmLimiter(&mixerRuntime);
    }
#endif

    // Find roll/pitch/yaw desired output
    // ??? Where is the optimal location for this code?
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float mix =
            scaledAxisPidRoll  * activeMixer[i].roll +
            scaledAxisPidPitch * activeMixer[i].pitch +
            scaledAxisPidYaw   * activeMixer[i].yaw;

        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }

    //  The following fixed throttle values will not be shown in the blackbox log
    // ?? Should they be influenced by airmode?  If not, should go after the apply airmode code.
    const bool airmodeEnabled = airmodeIsEnabled();
#ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif

    motorMixRange = motorMixMax - motorMixMin;

    switch (mixerConfig()->mixer_type) {
    case MIXER_LEGACY:
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
        break;
    case MIXER_LINEAR:
    case MIXER_DYNAMIC:
        applyMixerAdjustmentLinear(motorMix, airmodeEnabled);
        break;
    case MIXER_EZLANDING:
        applyMixerAdjustmentEzLand(motorMix, motorMixMin, motorMixMax);
        break;
    default:
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
        break;
    }

    if (ARMING_FLAG(ARMED)
        && !airmodeEnabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable motor_stop while GPS Rescue is active
        && (rcData[THROTTLE] < rxConfig()->mincheck)) {
        // motor_stop handling
        applyMotorStop();
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(motorMix, activeMixer);
    }
}

float mixerGetThrottle(void)
{
    return mixerThrottle;
}
