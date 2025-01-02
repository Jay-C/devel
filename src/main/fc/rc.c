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
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/utils.h"
#include "common/vector.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/pid.h"
#include "flight/pid_init.h"

#include "pg/rx.h"
#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "rc.h"

#define RX_INTERVAL_MIN_US     950 // 0.950ms to fit 1kHz without an issue
#define RX_INTERVAL_MAX_US   65500 // 65.5ms or 15.26hz

typedef float (applyRatesFn)(const int axis, float rcCommandf, const float rcCommandfAbs);
// note that rcCommand[] is an external float

static float rawSetpoint[XYZ_AXIS_COUNT];

static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3]; // deflection range -1 to 1
static float maxRcDeflectionAbs;

static applyRatesFn *applyRates;

static uint16_t currentRxIntervalUs;  // packet interval in microseconds, constrained to above range
static uint16_t previousRxIntervalUs; // previous packet interval in microseconds
static float currentRxRateHz;         // packet interval in Hz, constrained as above

static bool isRxDataNew = false;
static bool isRxRateValid = false;
static float rcCommandDivider = 500.0f;
static float rcCommandYawDivider = 500.0f;

enum {
    ROLL_FLAG = 1 << ROLL,
    PITCH_FLAG = 1 << PITCH,
    YAW_FLAG = 1 << YAW,
    THROTTLE_FLAG = 1 << THROTTLE,
};

#ifdef USE_RC_SMOOTHING_FILTER
static FAST_DATA_ZERO_INIT rcSmoothingFilter_t rcSmoothingData;
static float rcDeflectionSmoothed[3];
#endif // USE_RC_SMOOTHING_FILTER

float getSetpointRate(int axis)
{
#ifdef USE_RC_SMOOTHING_FILTER
    return setpointRate[axis];
#else
    return rawSetpoint[axis];
#endif
}

static float maxRcRate[3];
float getMaxRcRate(int axis)
{
    return maxRcRate[axis];
}

float getRcDeflection(int axis)
{
#ifdef USE_RC_SMOOTHING_FILTER
    return rcDeflectionSmoothed[axis];
#else
    return rcDeflection[axis];
#endif
}

float getRcDeflectionRaw(int axis)
{
    return rcDeflection[axis];
}

float getRcDeflectionAbs(int axis)
{
    return rcDeflectionAbs[axis];
}

float getMaxRcDeflectionAbs(void)
{
    return maxRcDeflectionAbs;
}

#define THROTTLE_LOOKUP_LENGTH 12
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE

static int16_t rcLookupThrottle(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
    return lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;
}

#define SETPOINT_RATE_LIMIT_MIN -1998.0f
#define SETPOINT_RATE_LIMIT_MAX 1998.0f
STATIC_ASSERT(CONTROL_RATE_CONFIG_RATE_LIMIT_MAX <= (uint16_t)SETPOINT_RATE_LIMIT_MAX, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX_too_large);

#define RC_RATE_INCREMENTAL 14.54f

float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    if (currentControlRateProfile->rcExpo[axis]) {
        const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
    }

    float rcRate = currentControlRateProfile->rcRates[axis] / 100.0f;
    if (rcRate > 2.0f) {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
    }
    float angleRate = 200.0f * rcRate * rcCommandf;
    if (currentControlRateProfile->rates[axis]) {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }

    return angleRate;
}

float applyRaceFlightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    // -1.0 to 1.0 ranged and curved
    rcCommandf = ((1.0f + 0.01f * currentControlRateProfile->rcExpo[axis] * (rcCommandf * rcCommandf - 1.0f)) * rcCommandf);
    // convert to -2000 to 2000 range using acro+ modifier
    float angleRate = 10.0f * currentControlRateProfile->rcRates[axis] * rcCommandf;
    angleRate = angleRate * (1 + rcCommandfAbs * (float)currentControlRateProfile->rates[axis] * 0.01f);

    return angleRate;
}

float applyKissRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    const float rcCurvef = currentControlRateProfile->rcExpo[axis] / 100.0f;

    float kissRpyUseRates = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
    float kissRcCommandf = (power3(rcCommandf) * rcCurvef + rcCommandf * (1 - rcCurvef)) * (currentControlRateProfile->rcRates[axis] / 1000.0f);
    float kissAngle = constrainf(((2000.0f * kissRpyUseRates) * kissRcCommandf), SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);

    return kissAngle;
}

float applyActualRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
    expof = rcCommandfAbs * (power5(rcCommandf) * expof + rcCommandf * (1 - expof));

    const float centerSensitivity = currentControlRateProfile->rcRates[axis] * 10.0f;
    const float stickMovement = MAX(0, currentControlRateProfile->rates[axis] * 10.0f - centerSensitivity);
    const float angleRate = rcCommandf * centerSensitivity + stickMovement * expof;

    return angleRate;
}

float applyQuickRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    const uint16_t rcRate = currentControlRateProfile->rcRates[axis] * 2;
    const uint16_t maxDPS = MAX(currentControlRateProfile->rates[axis] * 10, rcRate);
    const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
    const float superFactorConfig = ((float)maxDPS / rcRate - 1) / ((float)maxDPS / rcRate);

    float curve;
    float superFactor;
    float angleRate;

    if (currentControlRateProfile->quickRatesRcExpo) {
        curve = power3(rcCommandf) * expof + rcCommandf * (1 - expof);
        superFactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * superFactorConfig), 0.01f, 1.00f));
        angleRate = constrainf(curve * rcRate * superFactor, SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
    } else {
        curve = power3(rcCommandfAbs) * expof + rcCommandfAbs * (1 - expof);
        superFactor = 1.0f / (constrainf(1.0f - (curve * superFactorConfig), 0.01f, 1.00f));
        angleRate = constrainf(rcCommandf * rcRate * superFactor, SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
    }

    return angleRate;
}

void updateRcRefreshRate(timeUs_t currentTimeUs, bool rxReceivingSignal)
{
    // this function runs from processRx in core.c
    // rxReceivingSignal is true:
    // - every time a new frame is detected,
    // - if we stop getting data, at the expiry of RXLOSS_TRIGGER_INTERVAL since the last good frame
    // - if that interval is exceeded and still no data, every RX_FRAME_RECHECK_INTERVAL, until a new frame is detected
    static timeUs_t lastRxTimeUs = 0;
    timeDelta_t delta = 0;

    if (rxReceivingSignal) { // true while receiving data and until RXLOSS_TRIGGER_INTERVAL expires, otherwise false
        previousRxIntervalUs = currentRxIntervalUs;
        // use driver rx time if available, current time otherwise
        const timeUs_t rxTime = rxRuntimeState.lastRcFrameTimeUs ? rxRuntimeState.lastRcFrameTimeUs : currentTimeUs;

        if (lastRxTimeUs) {  // report delta only if previous time is available
            delta = cmpTimeUs(rxTime, lastRxTimeUs);
        }
        lastRxTimeUs = rxTime;
        DEBUG_SET(DEBUG_RX_TIMING, 1, rxTime / 100);   // output value in tenths of ms
    } else {
        if (lastRxTimeUs) {
            // no packet received, use current time for delta
            delta = cmpTimeUs(currentTimeUs, lastRxTimeUs);
        }
    }

    // temporary debugs
    DEBUG_SET(DEBUG_RX_TIMING, 4, MIN(delta / 10, INT16_MAX));   // time between frames based on rxFrameCheck
#ifdef USE_RX_LINK_QUALITY_INFO
    DEBUG_SET(DEBUG_RX_TIMING, 6, rxGetLinkQualityPercent());    // raw link quality value
#endif
    DEBUG_SET(DEBUG_RX_TIMING, 7, isRxReceivingSignal());        // flag to initiate RXLOSS signal and Stage 1 values

    // constrain to a frequency range no lower than about 15Hz and up to about 1000Hz
    // these intervals and rates will be used for RCSmoothing, Feedforward, etc.
    currentRxIntervalUs = constrain(delta, RX_INTERVAL_MIN_US, RX_INTERVAL_MAX_US);
    currentRxRateHz = 1e6f / currentRxIntervalUs;
    isRxRateValid = delta == currentRxIntervalUs; // delta is not constrained, therefore not outside limits

    DEBUG_SET(DEBUG_RX_TIMING, 0, MIN(delta / 10, INT16_MAX));   // output value in hundredths of ms
    DEBUG_SET(DEBUG_RX_TIMING, 2, isRxRateValid);
    DEBUG_SET(DEBUG_RX_TIMING, 3, MIN(currentRxIntervalUs / 10, INT16_MAX));
}

uint16_t getCurrentRxRateHz(void)
{
    return currentRxRateHz;
}

bool getRxRateValid(void)
{
    return isRxRateValid;
}

#ifdef USE_RC_SMOOTHING_FILTER

// Initialize or update the filters base on either the manually selected cutoff, or
// the auto-calculated cutoff frequency based on detected rx frame rate.
FAST_CODE_NOINLINE void rcSmoothingSetFilterCutoffs(rcSmoothingFilter_t *smoothingData)
{
    // in auto mode, calculate the RC smoothing cutoff from the smoothed Rx link frequency
    const uint16_t oldSetpointCutoff = smoothingData->setpointCutoffFrequency;
    const uint16_t oldFeedforwardCutoff = smoothingData->feedforwardCutoffFrequency;
    const uint16_t minCutoffHz = 15; // don't let any RC smoothing filter cutoff go below 15Hz
    if (smoothingData->setpointCutoffSetting == 0) {
        smoothingData->setpointCutoffFrequency = MAX(minCutoffHz, (uint16_t)(smoothingData->smoothedRxRateHz * smoothingData->autoSmoothnessFactorSetpoint));
    }
    if (smoothingData->throttleCutoffSetting == 0) {
        smoothingData->throttleCutoffFrequency = MAX(minCutoffHz, (uint16_t)(smoothingData->smoothedRxRateHz * smoothingData->autoSmoothnessFactorThrottle));
    }

    if (smoothingData->feedforwardCutoffSetting == 0) {
        smoothingData->feedforwardCutoffFrequency = MAX(minCutoffHz, (uint16_t)(smoothingData->smoothedRxRateHz * smoothingData->autoSmoothnessFactorFeedforward));
    }

    const float dT = targetPidLooptime * 1e-6f;
    if ((smoothingData->setpointCutoffFrequency != oldSetpointCutoff) || !smoothingData->filterInitialized) {
        // note that cutoff frequencies are integers, filter cutoffs won't re-calculate until there is > 1hz variation from previous cutoff
        // initialize or update the setpoint cutoff based filters
        const float setpointCutoffFrequency = smoothingData->setpointCutoffFrequency;
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            if (i < THROTTLE) {
                if (!smoothingData->filterInitialized) {
                    pt3FilterInit(&smoothingData->filterSetpoint[i], pt3FilterGain(setpointCutoffFrequency, dT));
                } else {
                    pt3FilterUpdateCutoff(&smoothingData->filterSetpoint[i], pt3FilterGain(setpointCutoffFrequency, dT));
                }
            } else {
                const float throttleCutoffFrequency = smoothingData->throttleCutoffFrequency;
                if (!smoothingData->filterInitialized) {
                    pt3FilterInit(&smoothingData->filterSetpoint[i], pt3FilterGain(throttleCutoffFrequency, dT));
                } else {
                    pt3FilterUpdateCutoff(&smoothingData->filterSetpoint[i], pt3FilterGain(throttleCutoffFrequency, dT));
                }
            }
        }
        // initialize or update the RC Deflection filter
        for (int i = FD_ROLL; i < FD_YAW; i++) {
            if (!smoothingData->filterInitialized) {
                pt3FilterInit(&smoothingData->filterRcDeflection[i], pt3FilterGain(setpointCutoffFrequency, dT));
            } else {
                pt3FilterUpdateCutoff(&smoothingData->filterRcDeflection[i], pt3FilterGain(setpointCutoffFrequency, dT));
            }
        }
    }
    // initialize or update the Feedforward filter
    if ((smoothingData->feedforwardCutoffFrequency != oldFeedforwardCutoff) || !smoothingData->filterInitialized) {
       for (int i = FD_ROLL; i <= FD_YAW; i++) {
            const float feedforwardCutoffFrequency = smoothingData->feedforwardCutoffFrequency;
            if (!smoothingData->filterInitialized) {
                pt3FilterInit(&smoothingData->filterFeedforward[i], pt3FilterGain(feedforwardCutoffFrequency, dT));
            } else {
                pt3FilterUpdateCutoff(&smoothingData->filterFeedforward[i], pt3FilterGain(feedforwardCutoffFrequency, dT));
            }
        }
    }

    DEBUG_SET(DEBUG_RC_SMOOTHING, 1, smoothingData->setpointCutoffFrequency);
    DEBUG_SET(DEBUG_RC_SMOOTHING, 2, smoothingData->feedforwardCutoffFrequency);
}

// Determine if we need to caclulate filter cutoffs. If not then we can avoid
// examining the rx frame times completely
FAST_CODE_NOINLINE bool rcSmoothingAutoCalculate(void)
{
    // if any rc smoothing cutoff is 0 (auto) then we need to calculate cutoffs
    if ((rcSmoothingData.setpointCutoffSetting == 0) || (rcSmoothingData.feedforwardCutoffSetting == 0) || (rcSmoothingData.throttleCutoffSetting == 0)) {
        return true;
    }
    return false;
}

static FAST_CODE void processRcSmoothingFilter(void)
{
    static FAST_DATA_ZERO_INIT float rxDataToSmooth[4];
    static FAST_DATA_ZERO_INIT bool initialized;
    static FAST_DATA_ZERO_INIT bool calculateCutoffs;

    // first call initialization
    if (!initialized) {
        initialized = true;
        rcSmoothingData.filterInitialized = false;
        rcSmoothingData.smoothedRxRateHz = 0.0f;
        rcSmoothingData.sampleCount = 0;
        rcSmoothingData.debugAxis = rxConfig()->rc_smoothing_debug_axis;

        rcSmoothingData.autoSmoothnessFactorSetpoint = 1.5f / (1.0f + (rxConfig()->rc_smoothing_auto_factor_rpy / 10.0f));
        rcSmoothingData.autoSmoothnessFactorFeedforward = 1.5f / (1.0f + (rxConfig()->rc_smoothing_auto_factor_rpy / 10.0f));
        rcSmoothingData.autoSmoothnessFactorThrottle = 1.5f / (1.0f + (rxConfig()->rc_smoothing_auto_factor_throttle / 10.0f));

        rcSmoothingData.setpointCutoffSetting = rxConfig()->rc_smoothing_setpoint_cutoff;
        rcSmoothingData.throttleCutoffSetting = rxConfig()->rc_smoothing_throttle_cutoff;
        rcSmoothingData.feedforwardCutoffSetting = rxConfig()->rc_smoothing_feedforward_cutoff;

        rcSmoothingData.setpointCutoffFrequency = rcSmoothingData.setpointCutoffSetting;
        rcSmoothingData.feedforwardCutoffFrequency = rcSmoothingData.feedforwardCutoffSetting;
        rcSmoothingData.throttleCutoffFrequency = rcSmoothingData.throttleCutoffSetting;

        if (rxConfig()->rc_smoothing_mode) {
            calculateCutoffs = rcSmoothingAutoCalculate();
            // if we don't need to calculate cutoffs dynamically then the filters can be initialized now
            if (!calculateCutoffs) {
                rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                rcSmoothingData.filterInitialized = true;
            }
        }
    }

    if (isRxDataNew) {
        if (calculateCutoffs) {
            // for auto calculated filters, calculate the link interval and update the RC smoothing filters at regular intervals
            // this is more efficient than monitoring for significant changes and making comparisons to decide whether to update the filter
            const timeMs_t currentTimeMs = millis();
            int sampleState = 0;
            const bool ready = (currentTimeMs > 1000) && (targetPidLooptime > 0);
            if (ready) { // skip during FC initialization
                // Wait 1000ms after power to let the PID loop stabilize before starting average frame rate calculation
                if (isRxReceivingSignal() && isRxRateValid) {

                    if (abs(currentRxIntervalUs - previousRxIntervalUs) < (previousRxIntervalUs - (previousRxIntervalUs / 8))) {
                        // exclude large steps, eg after dropouts or telemetry
                        // by using interval here, we catch a dropout/telemetry where the inteval increases by 100%, but accept
                        // the return to normal value, which is only 50% different from the 100% interval of a single drop, and 66% of a return after a double drop.
                        static float prevSmoothedRxRateHz;
                        // smooth the current Rx link frequency estimates
                        const float kF = 0.1f; // first order kind of lowpass smoothing filter coefficient
                        // add one tenth of the new estimate to the smoothed estimate.
                        const float smoothedRxRateHz = prevSmoothedRxRateHz + kF * (currentRxRateHz - prevSmoothedRxRateHz);
                        prevSmoothedRxRateHz = smoothedRxRateHz;

                        // recalculate cutoffs every 3 acceptable samples
                        if (rcSmoothingData.sampleCount) {
                            rcSmoothingData.sampleCount --;
                            sampleState = 1;
                        } else {
                            rcSmoothingData.smoothedRxRateHz = smoothedRxRateHz;
                            rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                            rcSmoothingData.filterInitialized = true;
                            rcSmoothingData.sampleCount = 3;
                            sampleState = 2;
                        }
                    }
                } else {
                    // either we stopped receiving rx samples (failsafe?) or the sample interval is unreasonable
                    // require a full re-evaluation period after signal is restored
                    rcSmoothingData.sampleCount = 0;
                    sampleState = 4;
                }
            }
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 0, currentRxIntervalUs / 10);
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 1, rcSmoothingData.sampleCount);
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 2, rcSmoothingData.smoothedRxRateHz); // value used by filters
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 3, sampleState); // guard time = 1, guard time expired = 2
        }
        // Get new values to be smoothed
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            rxDataToSmooth[i] = i == THROTTLE ? rcCommand[i] : rawSetpoint[i];
            if (i < THROTTLE) {
                DEBUG_SET(DEBUG_RC_INTERPOLATION, i, lrintf(rxDataToSmooth[i]));
            } else {
                DEBUG_SET(DEBUG_RC_INTERPOLATION, i, ((lrintf(rxDataToSmooth[i])) - 1000));
            }
        }
    }

    DEBUG_SET(DEBUG_RC_SMOOTHING, 0, rcSmoothingData.smoothedRxRateHz);
    DEBUG_SET(DEBUG_RC_SMOOTHING, 3, rcSmoothingData.sampleCount);

    // each pid loop, apply the last received channel value to the filter, if initialised - thanks @klutvott
    for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
        float *dst = i == THROTTLE ? &rcCommand[i] : &setpointRate[i];
        if (rcSmoothingData.filterInitialized) {
            *dst = pt3FilterApply(&rcSmoothingData.filterSetpoint[i], rxDataToSmooth[i]);
        } else {
            // If filter isn't initialized yet, as in smoothing off, use the actual unsmoothed rx channel data
            *dst = rxDataToSmooth[i];
        }
    }

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        // Horizon mode smoothing of rcDeflection on pitch and roll to provide a smooth angle element
        const bool smoothRcDeflection = FLIGHT_MODE(HORIZON_MODE) && rcSmoothingData.filterInitialized;
        if (smoothRcDeflection && axis < FD_YAW) {
            rcDeflectionSmoothed[axis] = pt3FilterApply(&rcSmoothingData.filterRcDeflection[axis], rcDeflection[axis]);
        } else {
            rcDeflectionSmoothed[axis] = rcDeflection[axis];
        }
    }
}
#endif // USE_RC_SMOOTHING_FILTER

FAST_CODE void processRcCommand(void)
{
    if (isRxDataNew) {
        maxRcDeflectionAbs = 0.0f;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {

            float angleRate;

#ifdef USE_GPS_RESCUE
            if ((axis == FD_YAW) && FLIGHT_MODE(GPS_RESCUE_MODE)) {
                // If GPS Rescue is active then override the setpointRate used in the
                // pid controller with the value calculated from the desired heading logic.
                angleRate = gpsRescueGetYawRate();
                // Treat the stick input as centered to avoid any stick deflection base modifications (like acceleration limit)
                rcDeflection[axis] = 0;
                rcDeflectionAbs[axis] = 0;
            } else
#endif
            {
                // scale rcCommandf to range [-1.0, 1.0]
                float rcCommandf;
                if (axis == FD_YAW) {
                    rcCommandf = rcCommand[axis] / rcCommandYawDivider;
                } else {
                    rcCommandf = rcCommand[axis] / rcCommandDivider;
                }
                rcDeflection[axis] = rcCommandf;
                const float rcCommandfAbs = fabsf(rcCommandf);
                rcDeflectionAbs[axis] = rcCommandfAbs;
                maxRcDeflectionAbs = fmaxf(maxRcDeflectionAbs, rcCommandfAbs);

                angleRate = applyRates(axis, rcCommandf, rcCommandfAbs);
            }

            rawSetpoint[axis] = constrainf(angleRate, -1.0f * currentControlRateProfile->rate_limit[axis], 1.0f * currentControlRateProfile->rate_limit[axis]);
            DEBUG_SET(DEBUG_ANGLERATE, axis, angleRate);
        }
    }

#ifdef USE_RC_SMOOTHING_FILTER
    processRcSmoothingFilter();
#endif

    isRxDataNew = false;
}

FAST_CODE_NOINLINE void updateRcCommands(void)
{
    isRxDataNew = true;

    for (int axis = 0; axis < 3; axis++) {
        float rc = constrainf(rcData[axis] - rxConfig()->midrc, -500.0f, 500.0f); // -500 to 500
        float rcDeadband = 0;
        if (axis == ROLL || axis == PITCH) {
            rcDeadband = rcControlsConfig()->deadband;
        } else {
            rcDeadband  = rcControlsConfig()->yaw_deadband;
            rc = -rc;  // Yaw direction reversed
        }
        rcCommand[axis] = fapplyDeadband(rc, rcDeadband);
    }

    int32_t tmp;
    tmp = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);

    if (getLowVoltageCutoff()->enabled) {
        tmp = tmp * getLowVoltageCutoff()->percentage / 100;
    }

    rcCommand[THROTTLE] = rcLookupThrottle(tmp);

    if (FLIGHT_MODE(HEADFREE_MODE)) {
        static vector3_t rcCommandBuff;

        rcCommandBuff.x = rcCommand[ROLL];
        rcCommandBuff.y = rcCommand[PITCH];
        if (!FLIGHT_MODE(ANGLE_MODE | ALT_HOLD_MODE | POS_HOLD_MODE | HORIZON_MODE | GPS_RESCUE_MODE)) {
            rcCommandBuff.z = rcCommand[YAW];
        } else {
            rcCommandBuff.z = 0;
        }
        imuQuaternionHeadfreeTransformVectorEarthToBody(&rcCommandBuff);
        rcCommand[ROLL] = rcCommandBuff.x;
        rcCommand[PITCH] = rcCommandBuff.y;
        if (!FLIGHT_MODE(ANGLE_MODE | ALT_HOLD_MODE | POS_HOLD_MODE | HORIZON_MODE | GPS_RESCUE_MODE)) {
            rcCommand[YAW] = rcCommandBuff.z;
        }
    }
}

void resetYawAxis(void)
{
    rcCommand[YAW] = 0;
    setpointRate[YAW] = 0;
}

void initRcProcessing(void)
{
    rcCommandDivider = 500.0f - rcControlsConfig()->deadband;
    rcCommandYawDivider = 500.0f - rcControlsConfig()->yaw_deadband;

    switch (currentControlRateProfile->rates_type) {
    case RATES_TYPE_BETAFLIGHT:
    default:
        applyRates = applyBetaflightRates;
        break;
    case RATES_TYPE_RACEFLIGHT:
        applyRates = applyRaceFlightRates;
        break;
    case RATES_TYPE_KISS:
        applyRates = applyKissRates;
        break;
    case RATES_TYPE_ACTUAL:
        applyRates = applyActualRates;
        break;
    case RATES_TYPE_QUICK:
        applyRates = applyQuickRates;
        break;
    }

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        maxRcRate[i] = applyRates(i, 1.0f, 1.0f);
    }
}

// send rc smoothing details to blackbox
#ifdef USE_RC_SMOOTHING_FILTER
rcSmoothingFilter_t *getRcSmoothingData(void)
{
    return &rcSmoothingData;
}

bool rcSmoothingInitializationComplete(void)
{
    return rcSmoothingData.filterInitialized;
}
#endif // USE_RC_SMOOTHING_FILTER
