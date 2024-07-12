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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_DATA_ZERO_INIT uint32_t targetPidLooptime;
FAST_DATA_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];
FAST_DATA_ZERO_INIT pidRuntime_t pidRuntime;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3);

#ifndef DEFAULT_PID_PROCESS_DENOM
#define DEFAULT_PID_PROCESS_DENOM       1
#endif

PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = DEFAULT_PID_PROCESS_DENOM,
);

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 8);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] =   PID_YAW_DEFAULT,
            [PID_LEVEL] = { 50, 75, 75, 50 },
            [PID_MAG] =   { 40, 0, 0, 0 },
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindupPointPercent = 85,
        .angle_limit = 60,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .horizon_limit_degrees = 135,
        .horizon_ignore_sticks = false,
        .itermLimit = 400,
        .iterm_rotation = false,
        .dterm_lpf1_static_hz = DTERM_LPF1_HZ_DEFAULT,
        .dterm_lpf2_static_hz = DTERM_LPF2_HZ_DEFAULT,
        .dterm_lpf1_type = FILTER_PT1,
        .dterm_lpf2_type = FILTER_PT1,
        .transient_throttle_limit = 0,
        .profileName = { 0 },
        .angle_earth_ref = 100,
        .horizon_delay_ms = 500, // 500ms time constant on any increase in horizon strength
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

// Scale factors to make best use of range with D_LPF debugging, aiming for max +/-16K as debug values are 16 bit
#define D_LPF_RAW_SCALE 25

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
    }
}

#if defined(USE_ACC)
// Calculate strength of horizon leveling; 0 = none, 1.0 = most leveling
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float calcHorizonLevelStrength(void)
{
    const float currentInclination = MAX(abs(attitude.values.roll), abs(attitude.values.pitch)) * 0.1f;
    // 0 when level, 90 when vertical, 180 when inverted (degrees):
    float absMaxStickDeflection = MAX(fabsf(getRcDeflection(FD_ROLL)), fabsf(getRcDeflection(FD_PITCH)));
    // 0-1, smoothed if RC smoothing is enabled

    float horizonLevelStrength = MAX((pidRuntime.horizonLimitDegrees - currentInclination) * pidRuntime.horizonLimitDegreesInv, 0.0f);
    // 1.0 when attitude is 'flat', 0 when angle is equal to, or greater than, horizonLimitDegrees
    horizonLevelStrength *= MAX((pidRuntime.horizonLimitSticks - absMaxStickDeflection) * pidRuntime.horizonLimitSticksInv, pidRuntime.horizonIgnoreSticks);
    // use the value of horizonIgnoreSticks to enable/disable this effect.
    // value should be 1.0 at center stick, 0.0 at max stick deflection:
    horizonLevelStrength *= pidRuntime.horizonGain;

    if (pidRuntime.horizonDelayMs) {
        const float horizonLevelStrengthSmoothed = pt1FilterApply(&pidRuntime.horizonSmoothingPt1, horizonLevelStrength);
        horizonLevelStrength = MIN(horizonLevelStrength, horizonLevelStrengthSmoothed);
    }
    return horizonLevelStrength;
    // 1 means full levelling, 0 means none
}

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim,
                                                        float currentPidSetpoint, float horizonLevelStrength)
{
    // Applies only to axes that are in Angle mode
    // We now use Acro Rates, transformed into the range +/- 1, to provide setpoints
    const float angleLimit = pidProfile->angle_limit;
    float angleTarget = angleLimit * currentPidSetpoint * pidRuntime.maxRcRateInv[axis];
    // use acro rates for the angle target in both horizon and angle modes, converted to -1 to +1 range using maxRate

#ifdef USE_GPS_RESCUE
    angleTarget += gpsRescueAngle[axis] / 100.0f; // Angle is in centidegrees, stepped on roll at 10Hz but not on pitch
#endif
    const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f; // stepped at 500hz with some 4ms flat spots
    const float errorAngle = angleTarget - currentAngle;
    float angleRate = errorAngle * pidRuntime.angleGain;

    // minimise cross-axis wobble due to faster yaw responses than roll or pitch, and make co-ordinated yaw turns
    // by compensating for the effect of yaw on roll while pitched, and on pitch while rolled
    // earthRef code here takes about 76 cycles, if conditional on angleEarthRef it takes about 100.  sin_approx costs most of those cycles.
    float sinAngle = sin_approx(DEGREES_TO_RADIANS(pidRuntime.angleTarget[axis == FD_ROLL ? FD_PITCH : FD_ROLL]));
    sinAngle *= (axis == FD_ROLL) ? -1.0f : 1.0f; // must be negative for Roll
    const float earthRefGain = FLIGHT_MODE(GPS_RESCUE_MODE) ? 1.0f : pidRuntime.angleEarthRef;
    angleRate += pidRuntime.angleYawSetpoint * sinAngle * earthRefGain;
    pidRuntime.angleTarget[axis] = angleTarget;  // set target for alternate axis to current axis, for use in preceding calculation

    // smooth final angle rate output to clean up attitude signal steps (500hz), GPS steps (10 or 100hz), RC steps etc
    // this filter runs at ATTITUDE_CUTOFF_HZ, currently 50hz, so GPS roll may be a bit steppy
    angleRate = pt3FilterApply(&pidRuntime.attitudeFilter[axis], angleRate);

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
        currentPidSetpoint = angleRate;
    } else {
        // can only be HORIZON mode - crossfade Angle rate and Acro rate
        currentPidSetpoint = currentPidSetpoint * (1.0f - horizonLevelStrength) + angleRate * horizonLevelStrength;
    }

    //logging
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_ANGLE_MODE, 0, lrintf(angleTarget * 10.0f)); // target angle
        DEBUG_SET(DEBUG_ANGLE_MODE, 1, lrintf(errorAngle * pidRuntime.angleGain * 10.0f)); // un-smoothed error correction in degrees
        DEBUG_SET(DEBUG_ANGLE_MODE, 3, lrintf(currentAngle * 10.0f)); // angle returned

        DEBUG_SET(DEBUG_ANGLE_TARGET, 0, lrintf(angleTarget * 10.0f));
        DEBUG_SET(DEBUG_ANGLE_TARGET, 1, lrintf(sinAngle * 10.0f)); // modification factor from earthRef
        // debug ANGLE_TARGET 2 is yaw attenuation
        DEBUG_SET(DEBUG_ANGLE_TARGET, 3, lrintf(currentAngle * 10.0f)); // angle returned
    }

    DEBUG_SET(DEBUG_CURRENT_ANGLE, axis, lrintf(currentAngle * 10.0f)); // current angle
    return currentPidSetpoint;
}

#endif // USE_ACC

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (fabsf(currentVelocity) > pidRuntime.maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + pidRuntime.maxVelocity[axis] : previousSetpoint[axis] - pidRuntime.maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

STATIC_UNIT_TESTED void rotateItermAndAxisError(void)
{
    if (pidRuntime.itermRotation
        ) {
        const float gyroToAngle = pidRuntime.dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
        if (pidRuntime.itermRotation) {
            float v[XYZ_AXIS_COUNT];
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            rotateVector(v, rotationRads );
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    static float previousGyroRateDterm[XYZ_AXIS_COUNT];
    static float previousRawGyroRateDterm[XYZ_AXIS_COUNT];

#if defined(USE_ACC)
    static timeUs_t levelModeStartTimeUs = 0;
    static bool gpsRescuePreviousState = false;
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    float horizonLevelStrength = 0.0f;

    const bool gpsRescueIsActive = FLIGHT_MODE(GPS_RESCUE_MODE);
    levelMode_e levelMode;
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || gpsRescueIsActive) {
        levelMode = LEVEL_MODE_RP;

        // Keep track of when we entered a self-level mode so that we can
        // add a guard time before crash recovery can activate.
        // Also reset the guard time whenever GPS Rescue is activated.
        if ((levelModeStartTimeUs == 0) || (gpsRescueIsActive && !gpsRescuePreviousState)) {
            levelModeStartTimeUs = currentTimeUs;
        }

        // Calc horizonLevelStrength if needed
        if (FLIGHT_MODE(HORIZON_MODE)) {
            horizonLevelStrength = calcHorizonLevelStrength();
        }
    } else {
        levelMode = LEVEL_MODE_OFF;
        levelModeStartTimeUs = 0;
    }

    gpsRescuePreviousState = gpsRescueIsActive;
#else
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);
#endif

    // iTerm windup (attenuation of iTerm if motorMix range is large)
    float dynCi = 1.0;
    if (pidRuntime.itermWindupPointInv > 1.0f) {
        dynCi = constrainf((1.0f - getMotorMixRange()) * pidRuntime.itermWindupPointInv, 0.0f, 1.0f);
    }

    // Precalculate gyro delta for D-term here, this allows loop unrolling
    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyro.gyroADCf[axis];

        // Log the unfiltered D for ROLL and PITCH
        if (debugMode == DEBUG_D_LPF && axis != FD_YAW) {
            const float delta = (previousRawGyroRateDterm[axis] - gyroRateDterm[axis]) * pidRuntime.pidFrequency / D_LPF_RAW_SCALE;
            previousRawGyroRateDterm[axis] = gyroRateDterm[axis];
            DEBUG_SET(DEBUG_D_LPF, axis, lrintf(delta)); // debug d_lpf 2 and 3 used for pre-TPA D
        }

        gyroRateDterm[axis] = pidRuntime.dtermNotchApplyFn((filter_t *) &pidRuntime.dtermNotch[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpassApplyFn((filter_t *) &pidRuntime.dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpass2ApplyFn((filter_t *) &pidRuntime.dtermLowpass2[axis], gyroRateDterm[axis]);
    }

    rotateItermAndAxisError();

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

        float currentPidSetpoint = getSetpointRate(axis);
        if (pidRuntime.maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }
        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        // When Race Mode is active PITCH control is also GYRO based in level or horizon mode
#if defined(USE_ACC)
        pidRuntime.axisInAngleMode[axis] = false;
        if (axis < FD_YAW) {
            if (levelMode == LEVEL_MODE_RP || (levelMode == LEVEL_MODE_R && axis == FD_ROLL)) {
                pidRuntime.axisInAngleMode[axis] = true;
                currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint, horizonLevelStrength);
            }
        } else { // yaw axis only
            if (levelMode == LEVEL_MODE_RP) {
                // if earth referencing is requested, attenuate yaw axis setpoint when pitched or rolled
                // and send yawSetpoint to Angle code to modulate pitch and roll
                // code cost is 107 cycles when earthRef enabled, 20 otherwise, nearly all in cos_approx
                const float earthRefGain = FLIGHT_MODE(GPS_RESCUE_MODE) ? 1.0f : pidRuntime.angleEarthRef;
                if (earthRefGain) {
                    pidRuntime.angleYawSetpoint = currentPidSetpoint;
                    float maxAngleTargetAbs = earthRefGain * fmaxf( fabsf(pidRuntime.angleTarget[FD_ROLL]), fabsf(pidRuntime.angleTarget[FD_PITCH]) );
                    maxAngleTargetAbs *= (FLIGHT_MODE(HORIZON_MODE)) ? horizonLevelStrength : 1.0f;
                    // reduce compensation whenever Horizon uses less levelling
                    currentPidSetpoint *= cos_approx(DEGREES_TO_RADIANS(maxAngleTargetAbs));
                    DEBUG_SET(DEBUG_ANGLE_TARGET, 2, currentPidSetpoint); // yaw setpoint after attenuation
                }
            }
        }
#endif

        // -----calculate error rate
        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
        float errorRate = currentPidSetpoint - gyroRate; // r - y
        const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;
        // --------low-level gyro-based PID based on 2DOF PID controller. ----------

        // -----calculate P component
        pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * errorRate;

        // -----calculate I component
        float Ki = pidRuntime.pidCoefficient[axis].Ki;
        const float iTermChange = Ki * dynCi * pidRuntime.dT * itermErrorRate;
        pidData[axis].I = constrainf(previousIterm + iTermChange, -pidRuntime.itermLimit, pidRuntime.itermLimit);

        // -----calculate D component

        float pidSetpointDelta = 0;
        pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint; // this is the value sent to blackbox, and used for Dmin setpoint

        if (pidRuntime.pidCoefficient[axis].Kd > 0) {
            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            const float delta = - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidRuntime.pidFrequency;
            pidData[axis].D = pidRuntime.pidCoefficient[axis].Kd * delta;
        } else {
            pidData[axis].D = 0;
            if (axis != FD_YAW) {
                DEBUG_SET(DEBUG_D_LPF, axis - FD_ROLL + 2, 0);
            }
        }

        previousGyroRateDterm[axis] = gyroRateDterm[axis];

        // -----calculate feedforward component
        const float feedforwardGain = pidRuntime.pidCoefficient[axis].Kf;
        pidData[axis].F = feedforwardGain * pidSetpointDelta;

        // calculating the PID sum
        pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
    }

    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if (gyroOverflowDetected()) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;
            pidData[axis].Sum = 0;
        }
    }
}

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo)
{
    const float expof = expo / 10.0f;
    const float curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

float pidGetPreviousSetpoint(int axis)
{
    return pidRuntime.previousPidSetpoint[axis];
}

float pidGetDT(void)
{
    return pidRuntime.dT;
}

float pidGetPidFrequency(void)
{
    return pidRuntime.pidFrequency;
}
