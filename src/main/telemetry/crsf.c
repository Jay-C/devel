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

#include "platform.h"

#ifdef USE_TELEMETRY_CRSF

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/version.h"

#include "cms/cms.h"

#include "config/config.h"
#include "config/feature.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/persistent.h"

#include "fc/rc_modes.h"
#include "fc/rc_adjustments.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/position.h"
#include "flight/governor.h"

#include "io/displayport_crsf.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/ledstrip.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/crsf.h"
#include "rx/crsf_protocol.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "scheduler/scheduler.h"

#include "telemetry/telemetry.h"
#include "telemetry/msp_shared.h"

#include "crsf.h"


#define CRSF_MSP_GRACE_US                   100000
#define CRSF_CMS_GRACE_US                   100000
#define CRSF_DEVICE_INFO_GRACE_US           50000

#define CRSF_DEVICEINFO_VERSION             0x01
#define CRSF_DEVICEINFO_PARAMETER_COUNT     0

#define CRSF_MSP_BUFFER_SIZE                128
#define CRSF_MSP_LENGTH_OFFSET              1

static bool crsfTelemetryEnabled;
static bool crsfCustomTelemetryEnabled;
static timeDelta_t crsfTelemetryInterval;

static bool deviceInfoReplyPending;

static sbuf_t crsfSbuf;
static uint8_t crsfFrame[CRSF_FRAME_SIZE_MAX + 32];

static timeUs_t crsfNextCycleTime = 0;


#if defined(USE_CRSF_V3)

static bool isCrsfV3Running = false;

typedef struct {
    bool hasPendingReply;
    bool isNewSpeedValid;
    uint8_t portID:3;
    uint8_t index;
    timeUs_t confirmationTime;
} crsfSpeedControl_t;

static crsfSpeedControl_t crsfSpeed = {0};


uint32_t getCrsfCachedBaudrate(void)
{
    uint32_t crsfCachedBaudrate = persistentObjectRead(PERSISTENT_OBJECT_SERIALRX_BAUD);
    // check if valid first. return default baudrate if not
    for (unsigned i = 0; i < BAUD_COUNT; i++) {
        if (crsfCachedBaudrate == baudRates[i] && baudRates[i] >= CRSF_BAUDRATE) {
            return crsfCachedBaudrate;
        }
    }
    return CRSF_BAUDRATE;
}

bool checkCrsfCustomizedSpeed(void)
{
    return crsfSpeed.index < BAUD_COUNT;
}

uint32_t getCrsfDesiredSpeed(void)
{
    return checkCrsfCustomizedSpeed() ? baudRates[crsfSpeed.index] : CRSF_BAUDRATE;
}

void setCrsfDefaultSpeed(void)
{
    crsfSpeed.hasPendingReply = false;
    crsfSpeed.isNewSpeedValid = false;
    crsfSpeed.confirmationTime = 0;
    crsfSpeed.index = BAUD_COUNT;
    isCrsfV3Running = false;
    crsfRxUpdateBaudrate(getCrsfDesiredSpeed());
}

bool crsfBaudNegotiationInProgress(void)
{
    return crsfSpeed.hasPendingReply || crsfSpeed.isNewSpeedValid;
}

#endif /* USE_CRSF_V3 */


#if defined(USE_MSP_OVER_TELEMETRY)

typedef struct mspBuffer_s {
    uint8_t bytes[CRSF_MSP_BUFFER_SIZE];
    int len;
} mspBuffer_t;

static mspBuffer_t mspRxBuffer;

void initCrsfMspBuffer(void)
{
    mspRxBuffer.len = 0;
}

bool bufferCrsfMspFrame(uint8_t *frameStart, int frameLength)
{
    if (mspRxBuffer.len + CRSF_MSP_LENGTH_OFFSET + frameLength > CRSF_MSP_BUFFER_SIZE) {
        return false;
    } else {
        uint8_t *p = mspRxBuffer.bytes + mspRxBuffer.len;
        *p++ = frameLength;
        memcpy(p, frameStart, frameLength);
        mspRxBuffer.len += CRSF_MSP_LENGTH_OFFSET + frameLength;
        return true;
    }
}

bool handleCrsfMspFrameBuffer(mspResponseFnPtr responseFn)
{
    static bool replyPending = false;

    if (replyPending) {
        if (crsfRxIsTelemetryBufEmpty()) {
            replyPending = sendMspReply(CRSF_FRAME_TX_MSP_FRAME_SIZE, responseFn);
        }
        return replyPending;
    }

    if (mspRxBuffer.len == 0)
        return false;

    int pos = 0;
    while (true) {
        const uint8_t mspFrameLength = mspRxBuffer.bytes[pos];
        if (handleMspFrame(&mspRxBuffer.bytes[CRSF_MSP_LENGTH_OFFSET + pos], mspFrameLength, NULL)) {
            if (crsfRxIsTelemetryBufEmpty()) {
                replyPending = sendMspReply(CRSF_FRAME_TX_MSP_FRAME_SIZE, responseFn);
            } else {
                replyPending = true;
            }
        }
        pos += CRSF_MSP_LENGTH_OFFSET + mspFrameLength;
        ATOMIC_BLOCK(NVIC_PRIO_SERIALUART1) {
            if (pos >= mspRxBuffer.len) {
                mspRxBuffer.len = 0;
                return replyPending;
            }
        }
    }

    return replyPending;
}
#endif /* USE_MSP_OVER_TELEMETRY */


/*
 * CRSF Stream buffer handling
 */

static size_t crsfLinkFrameSize(size_t size)
{
    // Telemetry data is send as multiples of 5 bytes with 4 bytes (virtual) overhead.
    return ((size + 8) / 5) * 5;
}

static size_t crsfSbufLen(sbuf_t *buf)
{
    return buf->ptr - crsfFrame;
}

static sbuf_t * crsfInitializeSbuf(void)
{
    sbuf_t * dst = &crsfSbuf;

    dst->ptr = crsfFrame;
    dst->end = crsfFrame + CRSF_FRAME_SIZE_MAX;

    sbufWriteU8(dst, CRSF_SYNC_BYTE);
    sbufWriteU8(dst, CRSF_FRAME_LENGTH_TYPE_CRC);  // placeholder

    return dst;
}

static size_t crsfFinalizeSbuf(sbuf_t *dst)
{
    // Frame length including CRC
    const size_t frameLength = crsfSbufLen(dst) + 2;

    if (frameLength <= CRSF_FRAME_SIZE_MAX)
    {
        // Set frame length into the placeholder
        crsfFrame[1] = frameLength - 3;

        // CRC does not include device address and frame length
        crc8_dvb_s2_sbuf_append(dst, &crsfFrame[2]);

        // Write the telemetry frame to the receiver
        crsfRxWriteTelemetryData(crsfFrame, frameLength);

        return frameLength;
    }

    return 0;
}


/*
 * CRSF frame has the structure:
 * <Device address> <Frame length> <Type> <Payload> <CRC>
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t), crc of <Type> and <Payload>
 */

/*
 * 0x02 GPS
 * Payload:
 * int32_t     Latitude ( degree / 10`000`000 )
 * int32_t     Longitude (degree / 10`000`000 )
 * uint16_t    Groundspeed ( km/h / 10 )
 * uint16_t    GPS heading ( degree / 100 )
 * uint16      Altitude ( meter ­1000m offset )
 * uint8_t     Satellites in use ( counter )
 */
static void crsfFrameGps(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_GPS);
    sbufWriteS32BE(dst, gpsSol.llh.lat);
    sbufWriteS32BE(dst, gpsSol.llh.lon);
    sbufWriteU16BE(dst, (gpsSol.groundSpeed * 36 + 50) / 100); // cm/s
    sbufWriteU16BE(dst, gpsSol.groundCourse * 10); // degrees * 10
    sbufWriteU16BE(dst, getEstimatedAltitudeCm() / 100 + 1000);
    sbufWriteU8(dst, gpsSol.numSat);
}

/*
 * 0x07 Variometer sensor
 * Payload:
 * int16_t     Variometer
*/
static void crsfFrameVarioSensor(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_VARIO_SENSOR);
    sbufWriteS16BE(dst, getEstimatedVarioCms());
}

/*
 * 0x08 Battery sensor
 * Payload:
 * uint16_t    Voltage (100mV steps)
 * uint16_t    Current (100mA steps)
 * uint24_t    Fuel (drawn mAh)
 * uint8_t     Battery remaining (percent)
*/
static void crsfFrameBatterySensor(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_BATTERY_SENSOR);
    sbufWriteU16BE(dst, getLegacyBatteryVoltage());
    sbufWriteU16BE(dst, getLegacyBatteryCurrent());
    sbufWriteU24BE(dst, getBatteryCapacityUsed());
    sbufWriteU8(dst, calculateBatteryPercentageRemaining());
}

/*
 * 0x09 Baro altitude sensor
 * Payload:
 * uint16_t    Altitude (dm)
 * int16_t     Variometer
*/
static void crsfFrameAltitudeSensor(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_ALTITUDE_SENSOR);
    sbufWriteU16BE(dst, getEstimatedAltitudeCm() / 10 + 10000);
    sbufWriteS16BE(dst, getEstimatedVarioCms());
}

/*
 * 0x0B Heartbeat
 * Payload:
 * int16_t    Origin Device address
*/
static void crsfFrameHeartbeat(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_HEARTBEAT);
    sbufWriteU16BE(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
}

/*
 * 0x1E Attitude
 * Payload:
 * int16_t     Pitch angle (rad / 10000)
 * int16_t     Roll angle (rad / 10000)
 * int16_t     Yaw angle (rad / 10000)
 */

// convert angle in decidegree to radians/10000 with +/-180 degree range
static int16_t decidegrees2Radians10000(int16_t angle_decidegree)
{
    while (angle_decidegree > 1800)
        angle_decidegree -= 3600;
    while (angle_decidegree < -1800)
        angle_decidegree += 3600;
    return RAD * 1000 * angle_decidegree;
}

// fill dst buffer with crsf-attitude telemetry frame
void crsfFrameAttitude(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_ATTITUDE);
    sbufWriteS16BE(dst, decidegrees2Radians10000(attitude.values.pitch));
    sbufWriteS16BE(dst, decidegrees2Radians10000(attitude.values.roll));
    sbufWriteS16BE(dst, decidegrees2Radians10000(attitude.values.yaw));
}

/*
 * 0x21 Flight mode text based
 * Payload:
 * char[]      Flight mode (Null terminated string)
 */

static const char * govStateNames[] = {
    "OFF",
    "IDLE",
    "SPOOLUP",
    "RECOVERY",
    "ACTIVE",
    "THR-OFF",
    "LOST-HS",
    "AUTOROT",
    "BAILOUT",
};

static void crsfGovernorInfo(char *buf)
{
    // Modes that are only relevant when disarmed
    if (!ARMING_FLAG(ARMED)) {
        if (isArmingDisabled())
            strcpy(buf, "DISABLED");
        else
            strcpy(buf, "DISARMED");
    }
    else {
        strcpy(buf, govStateNames[getGovernorState()]);
    }
}

void crsfFrameFlightMode(sbuf_t *dst)
{
    char buff[32] = INIT_ZERO;

    crsfGovernorInfo(buff);

    sbufWriteU8(dst, CRSF_FRAMETYPE_FLIGHT_MODE);
    sbufWriteStringWithZeroTerminator(dst, buff);
}

/*
 * 0x29 Device Info
 * Payload:
 * uint8_t     Destination
 * uint8_t     Origin
 * char[]      Device Name (Null terminated string)
 * uint32_t    Null Bytes
 * uint32_t    Null Bytes
 * uint32_t    Null Bytes
 * uint8_t     255 (Max MSP Parameter)
 * uint8_t     0x01 (Parameter version 1)
 */
static void crsfFrameDeviceInfo(sbuf_t *dst)
{
    char buff[30];

    tfp_sprintf(buff, "%s %s: %s", FC_FIRMWARE_NAME, FC_VERSION_STRING, systemConfig()->boardIdentifier);

    sbufWriteU8(dst, CRSF_FRAMETYPE_DEVICE_INFO);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteStringWithZeroTerminator(dst, buff);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU8(dst, CRSF_DEVICEINFO_PARAMETER_COUNT);
    sbufWriteU8(dst, CRSF_DEVICEINFO_VERSION);
}


/*
 * 0x88 Custom telemetry
 * Payload:
 * uint16_t    Sensor id
 * <type>      Sensor data
 * ...
 */

void crsfSensorEncodeNil(sbuf_t *buf, telemetrySensor_t *sensor)
{
    UNUSED(buf);
    UNUSED(sensor);
}

void crsfSensorEncodeU8(sbuf_t *buf, telemetrySensor_t *sensor)
{
    sbufWriteU8(buf, constrain(sensor->value, 0, 0xFF));
}

void crsfSensorEncodeS8(sbuf_t *buf, telemetrySensor_t *sensor)
{
    sbufWriteS8(buf, constrain(sensor->value, -0x80, 0x7F));
}

void crsfSensorEncodeU16(sbuf_t *buf, telemetrySensor_t *sensor)
{
    sbufWriteU16BE(buf, constrain(sensor->value, 0, 0xFFFF));
}

void crsfSensorEncodeS16(sbuf_t *buf, telemetrySensor_t *sensor)
{
    sbufWriteS16BE(buf, constrain(sensor->value, -0x8000, 0x7FFF));
}

void crsfSensorEncodeU24(sbuf_t *buf, telemetrySensor_t *sensor)
{
    sbufWriteU24BE(buf, constrain(sensor->value, 0, 0xFFFFFF));
}

void crsfSensorEncodeS24(sbuf_t *buf, telemetrySensor_t *sensor)
{
    sbufWriteS24BE(buf, constrain(sensor->value, -0x800000, 0x7FFFFF));
}

void crsfSensorEncodeU32(sbuf_t *buf, telemetrySensor_t *sensor)
{
    sbufWriteU32BE(buf, sensor->value);
}

void crsfSensorEncodeS32(sbuf_t *buf, telemetrySensor_t *sensor)
{
    sbufWriteS32BE(buf, sensor->value);
}

void crsfSensorEncodeCells(sbuf_t *buf, telemetrySensor_t *sensor)
{
    UNUSED(sensor);
    const int cells = getBatteryCellCount();
    sbufWriteU8(buf, cells);
    for (int i = 0; i < cells; i++) {
        int volt = constrain(getBatteryCellVoltage(i) - 200, 0, 0xFF);
        sbufWriteU8(buf, volt);
    }
}

void crsfSensorEncodeControl(sbuf_t *buf, telemetrySensor_t *sensor)
{
    UNUSED(sensor);
    const int p = lrintf(mixerGetInput(MIXER_IN_STABILIZED_PITCH) * 1200);
    const int r = lrintf(mixerGetInput(MIXER_IN_STABILIZED_ROLL) * 1200);
    const int y = lrintf(mixerGetInput(MIXER_IN_STABILIZED_YAW) * 2400);
    const int c = lrintf(mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE) * 1200);
    sbufWriteU8(buf, ((p >> 8) & 0x0F) | ((r >> 4) & 0xF0));
    sbufWriteU8(buf, (p & 0xFF));
    sbufWriteU8(buf, (r & 0xFF));
    sbufWriteU8(buf, ((y >> 8) & 0x0F) | ((c >> 4) & 0xF0));
    sbufWriteU8(buf, (y & 0xFF));
    sbufWriteU8(buf, (c & 0xFF));
}

void crsfSensorEncodeAttitude(sbuf_t *buf, telemetrySensor_t *sensor)
{
    UNUSED(sensor);
    sbufWriteS16BE(buf, attitude.values.pitch);
    sbufWriteS16BE(buf, attitude.values.roll);
    sbufWriteS16BE(buf, attitude.values.yaw);
}

void crsfSensorEncodeAccel(sbuf_t *buf, telemetrySensor_t *sensor)
{
    UNUSED(sensor);
    sbufWriteS16BE(buf, acc.accADC[0] * acc.dev.acc_1G_rec * 100);
    sbufWriteS16BE(buf, acc.accADC[1] * acc.dev.acc_1G_rec * 100);
    sbufWriteS16BE(buf, acc.accADC[2] * acc.dev.acc_1G_rec * 100);
}

void crsfSensorEncodeLatLong(sbuf_t *buf, telemetrySensor_t *sensor)
{
    UNUSED(sensor);
    sbufWriteS32BE(buf, gpsSol.llh.lat);
    sbufWriteS32BE(buf, gpsSol.llh.lon);
}

void crsfSensorEncodeAdjFunc(sbuf_t *buf, telemetrySensor_t *sensor)
{
    UNUSED(sensor);
    if (getAdjustmentsRangeName()) {
        sbufWriteU16BE(buf, getAdjustmentsRangeFunc());
        sbufWriteS32BE(buf, getAdjustmentsRangeValue());
    }
    else {
        sbufWriteU16BE(buf, 0);
        sbufWriteS32BE(buf, 0);
    }
}

static void crsfFrameCustomTelemetryHeader(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_CUSTOM_TELEM);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
}

static void crsfFrameCustomTelemetrySensor(sbuf_t *dst, telemetrySensor_t * sensor)
{
    sbufWriteU16BE(dst, sensor->tcode);
    sensor->encode(dst, sensor);
}


#define TLM_SENSOR(NAME, CODE, MINI, MAXI, ENCODER) \
    { \
        .telid = TELEM_##NAME, \
        .tcode = (CODE), \
        .min_interval = (MINI), \
        .max_interval = (MAXI), \
        .bucket = 0, \
        .value = 0, \
        .update = 0, \
        .active = false, \
        .encode = crsfSensorEncode##ENCODER, \
    }

static telemetrySensor_t * crsfHeartBeatSensor = NULL;

static telemetrySensor_t crsfNativeTelemetrySensors[] =
{
    TLM_SENSOR(HEARTBEAT,                    0,   100,   100,    Nil),
    TLM_SENSOR(FLIGHT_MODE,                  0,   200,   200,    Nil),
    TLM_SENSOR(BATTERY,                      0,   200,   200,    Nil),
    TLM_SENSOR(ATTITUDE,                     0,   200,   200,    Nil),
    TLM_SENSOR(ALTITUDE,                     0,   200,   200,    Nil),
    TLM_SENSOR(VARIOMETER,                   0,   200,   200,    Nil),
    TLM_SENSOR(GPS,                          0,   200,   200,    Nil),
};

static telemetrySensor_t crsfCustomTelemetrySensors[] =
{
    TLM_SENSOR(HEARTBEAT,               0x0001,  1000,  1000,    U16),

    TLM_SENSOR(BATTERY_VOLTAGE,         0x0011,   200,  3000,    U16),
    TLM_SENSOR(BATTERY_CURRENT,         0x0012,   200,  3000,    U16),
    TLM_SENSOR(BATTERY_CONSUMPTION,     0x0013,   200,  3000,    U16),
    TLM_SENSOR(BATTERY_CHARGE_LEVEL,    0x0014,   200,  3000,    U8),

    TLM_SENSOR(BATTERY_CELL_COUNT,      0x0020,   200,  3000,    U8),
    TLM_SENSOR(BATTERY_CELL_VOLTAGES,   0x0021,   200,  3000,    Cells),

    TLM_SENSOR(CONTROL,                 0x0030,   100,  3000,    Control),
    TLM_SENSOR(PITCH_CONTROL,           0x0031,   200,  3000,    S16),
    TLM_SENSOR(ROLL_CONTROL,            0x0032,   200,  3000,    S16),
    TLM_SENSOR(YAW_CONTROL,             0x0033,   200,  3000,    S16),
    TLM_SENSOR(COLLECTIVE_CONTROL,      0x0034,   200,  3000,    S16),
    TLM_SENSOR(THROTTLE_CONTROL,        0x0035,   200,  3000,    S8),

    TLM_SENSOR(ESC1_VOLTAGE,            0x0041,   200,  3000,    U16),
    TLM_SENSOR(ESC1_CURRENT,            0x0042,   200,  3000,    U16),
    TLM_SENSOR(ESC1_CAPACITY,           0x0043,   200,  3000,    U16),
    TLM_SENSOR(ESC1_ERPM,               0x0044,   200,  3000,    U16),
    TLM_SENSOR(ESC1_POWER,              0x0045,   200,  3000,    U16),
    TLM_SENSOR(ESC1_THROTTLE,           0x0046,   200,  3000,    U16),
    TLM_SENSOR(ESC1_TEMP1,              0x0047,   200,  3000,    U8),
    TLM_SENSOR(ESC1_TEMP2,              0x0048,   200,  3000,    U8),
    TLM_SENSOR(ESC1_BEC_VOLTAGE,        0x0049,   200,  3000,    U16),
    TLM_SENSOR(ESC1_BEC_CURRENT,        0x004A,   200,  3000,    U16),
    TLM_SENSOR(ESC1_STATUS,             0x004F,   200,  3000,    U32),

    TLM_SENSOR(ESC2_VOLTAGE,            0x0051,   200,  3000,    U16),
    TLM_SENSOR(ESC2_CURRENT,            0x0052,   200,  3000,    U16),
    TLM_SENSOR(ESC2_CAPACITY,           0x0053,   200,  3000,    U16),
    TLM_SENSOR(ESC2_ERPM,               0x0054,   200,  3000,    U16),
    TLM_SENSOR(ESC2_POWER,              0x0055,   200,  3000,    U16),
    TLM_SENSOR(ESC2_THROTTLE,           0x0056,   200,  3000,    U16),
    TLM_SENSOR(ESC2_TEMP1,              0x0057,   200,  3000,    U8),
    TLM_SENSOR(ESC2_TEMP2,              0x0058,   200,  3000,    U8),
    TLM_SENSOR(ESC2_BEC_VOLTAGE,        0x0059,   200,  3000,    U16),
    TLM_SENSOR(ESC2_BEC_CURRENT,        0x005A,   200,  3000,    U16),
    TLM_SENSOR(ESC2_STATUS,             0x005F,   200,  3000,    U32),

    TLM_SENSOR(ESC_VOLTAGE,             0x0080,   200,  3000,    U16),
    TLM_SENSOR(BEC_VOLTAGE,             0x0081,   200,  3000,    U16),
    TLM_SENSOR(BUS_VOLTAGE,             0x0082,   200,  3000,    U16),
    TLM_SENSOR(MCU_VOLTAGE,             0x0083,   200,  3000,    U16),

    TLM_SENSOR(ESC_CURRENT,             0x0090,   200,  3000,    U16),
    TLM_SENSOR(BEC_CURRENT,             0x0091,   200,  3000,    U16),
    TLM_SENSOR(BUS_CURRENT,             0x0092,   200,  3000,    U16),
    TLM_SENSOR(MCU_CURRENT,             0x0093,   200,  3000,    U16),

    TLM_SENSOR(ESC_TEMP,                0x00A0,   500,  3000,    U8),
    TLM_SENSOR(BEC_TEMP,                0x00A1,   500,  3000,    U8),
    TLM_SENSOR(MCU_TEMP,                0x00A3,   500,  3000,    U8),

    TLM_SENSOR(HEADING,                 0x00B1,   200,  3000,    S16),
    TLM_SENSOR(ALTITUDE,                0x00B2,   200,  3000,    S24),
    TLM_SENSOR(VARIOMETER,              0x00B3,   200,  3000,    S16),

    TLM_SENSOR(HEADSPEED,               0x00C0,   200,  3000,    U16),
    TLM_SENSOR(TAILSPEED,               0x00C1,   200,  3000,    U16),

    TLM_SENSOR(ATTITUDE,                0x0100,   100,  3000,    Attitude),
    TLM_SENSOR(ATTITUDE_PITCH,          0x0101,   200,  3000,    S16),
    TLM_SENSOR(ATTITUDE_ROLL,           0x0102,   200,  3000,    S16),
    TLM_SENSOR(ATTITUDE_YAW,            0x0103,   200,  3000,    S16),

    TLM_SENSOR(ACCEL,                   0x0110,   100,  3000,    Accel),
    TLM_SENSOR(ACCEL_X,                 0x0111,   200,  3000,    S16),
    TLM_SENSOR(ACCEL_Y,                 0x0112,   200,  3000,    S16),
    TLM_SENSOR(ACCEL_Z,                 0x0113,   200,  3000,    S16),

    TLM_SENSOR(GPS_SATS,                0x0121,   500,  3000,    U8),
    TLM_SENSOR(GPS_PDOP,                0x0122,   500,  3000,    U8),
    TLM_SENSOR(GPS_HDOP,                0x0123,   500,  3000,    U8),
    TLM_SENSOR(GPS_VDOP,                0x0124,   500,  3000,    U8),
    TLM_SENSOR(GPS_COORD,               0x0125,   200,  3000,    LatLong),
    TLM_SENSOR(GPS_ALTITUDE,            0x0126,   200,  3000,    S16),
    TLM_SENSOR(GPS_HEADING,             0x0127,   200,  3000,    S16),
    TLM_SENSOR(GPS_GROUNDSPEED,         0x0128,   200,  3000,    U16),
    TLM_SENSOR(GPS_HOME_DISTANCE,       0x0129,   200,  3000,    U16),
    TLM_SENSOR(GPS_HOME_DIRECTION,      0x012A,   200,  3000,    S16),

    TLM_SENSOR(CPU_LOAD,                0x0141,   250,  3000,    U8),
    TLM_SENSOR(SYS_LOAD,                0x0142,   250,  3000,    U8),
    TLM_SENSOR(RT_LOAD,                 0x0143,   250,  3000,    U8),

    TLM_SENSOR(MODEL_ID,                0x0200,   200,  3000,    U8),
    TLM_SENSOR(FLIGHT_MODE,             0x0201,   200,  3000,    U32),
    TLM_SENSOR(ARMING_FLAGS,            0x0202,   200,  3000,    U32),
    TLM_SENSOR(RESCUE_STATE,            0x0203,   200,  3000,    U8),
    TLM_SENSOR(GOVERNOR_STATE,          0x0204,   200,  3000,    U8),

    TLM_SENSOR(PID_PROFILE,             0x0211,   200,  3000,    U8),
    TLM_SENSOR(RATES_PROFILE,           0x0212,   200,  3000,    U8),
    TLM_SENSOR(LED_PROFILE,             0x0213,   200,  3000,    U8),

    TLM_SENSOR(ADJFUNC,                 0x0220,   200,  3000,    AdjFunc),

    TLM_SENSOR(DEBUG_0,                 0xFE00,   100,  3000,    S32),
    TLM_SENSOR(DEBUG_1,                 0xFE01,   100,  3000,    S32),
    TLM_SENSOR(DEBUG_2,                 0xFE02,   100,  3000,    S32),
    TLM_SENSOR(DEBUG_3,                 0xFE03,   100,  3000,    S32),
    TLM_SENSOR(DEBUG_4,                 0xFE04,   100,  3000,    S32),
    TLM_SENSOR(DEBUG_5,                 0xFE05,   100,  3000,    S32),
    TLM_SENSOR(DEBUG_6,                 0xFE06,   100,  3000,    S32),
    TLM_SENSOR(DEBUG_7,                 0xFE07,   100,  3000,    S32),
};

telemetrySensor_t * crsfGetNativeSensor(sensor_id_e id)
{
    for (size_t i = 0; i < ARRAYLEN(crsfNativeTelemetrySensors); i++) {
        telemetrySensor_t * sensor = &crsfNativeTelemetrySensors[i];
        if (sensor->telid == id)
            return sensor;
    }

    return NULL;
}

telemetrySensor_t * crsfGetCustomSensor(sensor_id_e id)
{
    for (size_t i = 0; i < ARRAYLEN(crsfCustomTelemetrySensors); i++) {
        telemetrySensor_t * sensor = &crsfCustomTelemetrySensors[i];
        if (sensor->telid == id)
            return sensor;
    }

    return NULL;
}


#if defined(USE_CRSF_V3)

static void crsfFrameSpeedNegotiationResponse(sbuf_t *dst, bool reply)
{
    uint8_t *start = sbufPtr(dst);

    sbufWriteU8(dst, CRSF_FRAMETYPE_COMMAND);
    sbufWriteU8(dst, CRSF_ADDRESS_CRSF_RECEIVER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteU8(dst, CRSF_COMMAND_SUBCMD_GENERAL);
    sbufWriteU8(dst, CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE);
    sbufWriteU8(dst, crsfSpeed.portID);
    sbufWriteU8(dst, reply);

    crc8_poly_0xba_sbuf_append(dst, start);
}

static void crsfProcessSpeedNegotiationCmd(uint8_t *frameStart)
{
    uint32_t newBaudrate = frameStart[2] << 24 | frameStart[3] << 16 | frameStart[4] << 8 | frameStart[5];
    unsigned index = 0;
    for (index = 0; index < BAUD_COUNT; index++) {
        if (newBaudrate == baudRates[index])
            break;
    }
    crsfSpeed.portID = frameStart[1];
    crsfSpeed.index = index;
}

static void crsfScheduleSpeedNegotiationResponse(void)
{
    crsfSpeed.hasPendingReply = true;
    crsfSpeed.isNewSpeedValid = false;
}

/* TASK_SPEED_NEGOTIATION @ 100Hz */
void speedNegotiationProcess(timeUs_t currentTimeUs)
{
    if (crsfSpeed.hasPendingReply) {
        bool found = (crsfSpeed.index < BAUD_COUNT) && crsfRxUseNegotiatedBaud();
        sbuf_t *dst = crsfInitializeSbuf();
        crsfFrameSpeedNegotiationResponse(dst, found);
        crsfRxSendTelemetryData(); // prevent overwriting previous data
        crsfFinalizeSbuf(dst);
        crsfRxSendTelemetryData();
        crsfSpeed.hasPendingReply = false;
        crsfSpeed.isNewSpeedValid = found;
        crsfSpeed.confirmationTime = currentTimeUs;
    }
    else if (crsfSpeed.isNewSpeedValid) {
        if (cmpTimeUs(currentTimeUs, crsfSpeed.confirmationTime) >= 4000) {
            // delay 4ms before applying the new baudrate
            crsfRxUpdateBaudrate(getCrsfDesiredSpeed());
            crsfSpeed.isNewSpeedValid = false;
            isCrsfV3Running = true;
        }
    }
    else if (!featureIsEnabled(FEATURE_TELEMETRY) && crsfRxUseNegotiatedBaud()) {
        // Send heartbeat if telemetry is disabled to allow RX to detect baud rate mismatches
        sbuf_t *dst = crsfInitializeSbuf();
        crsfFrameHeartbeat(dst);
        crsfRxSendTelemetryData(); // prevent overwriting previous data
        crsfFinalizeSbuf(dst);
        crsfRxSendTelemetryData();
    }
}

void crsfProcessCommand(uint8_t *frameStart)
{
    uint8_t cmd = frameStart[0];
    uint8_t sub = frameStart[1];

    if (cmd == CRSF_COMMAND_SUBCMD_GENERAL && sub == CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL) {
        crsfProcessSpeedNegotiationCmd(&frameStart[1]);
        crsfScheduleSpeedNegotiationResponse();
    }
}

#endif


#if defined(USE_CRSF_CMS_TELEMETRY)

#define CRSF_DISPLAYPORT_MAX_CHUNK_LENGTH   50
#define CRSF_DISPLAYPORT_BATCH_MAX          0x3F
#define CRSF_DISPLAYPORT_FIRST_CHUNK_MASK   0x80
#define CRSF_DISPLAYPORT_LAST_CHUNK_MASK    0x40
#define CRSF_DISPLAYPORT_SANITIZE_MASK      0x60
#define CRSF_RLE_CHAR_REPEATED_MASK         0x80
#define CRSF_RLE_MAX_RUN_LENGTH             256
#define CRSF_RLE_BATCH_SIZE                 2

static uint16_t getRunLength(const void *start, const void *end)
{
    uint8_t *cursor = (uint8_t*)start;
    uint8_t c = *cursor;
    size_t runLength = 0;
    for (; cursor != end; cursor++) {
        if (*cursor == c) {
            runLength++;
        } else {
            break;
        }
    }
    return runLength;
}

static void cRleEncodeStream(sbuf_t *source, sbuf_t *dest, uint8_t maxDestLen)
{
    const uint8_t *destEnd = sbufPtr(dest) + maxDestLen;
    while (sbufBytesRemaining(source) && (sbufPtr(dest) < destEnd)) {
        const uint8_t destRemaining = destEnd - sbufPtr(dest);
        const uint8_t *srcPtr = sbufPtr(source);
        const uint16_t runLength = getRunLength(srcPtr, source->end);
        uint8_t c = *srcPtr;
        if (runLength > 1) {
            c |=  CRSF_RLE_CHAR_REPEATED_MASK;
            const uint8_t fullBatches = (runLength / CRSF_RLE_MAX_RUN_LENGTH);
            const uint8_t remainder = (runLength % CRSF_RLE_MAX_RUN_LENGTH);
            const uint8_t totalBatches = fullBatches + (remainder ? 1 : 0);
            if (destRemaining >= totalBatches * CRSF_RLE_BATCH_SIZE) {
                for (unsigned int i = 1; i <= totalBatches; i++) {
                    const uint8_t batchLength = (i < totalBatches) ? CRSF_RLE_MAX_RUN_LENGTH : remainder;
                    sbufWriteU8(dest, c);
                    sbufWriteU8(dest, batchLength);
                }
                sbufAdvance(source, runLength);
            } else {
                break;
            }
        }
        else if (destRemaining >= runLength) {
            sbufWriteU8(dest, c);
            sbufAdvance(source, runLength);
        }
    }
}

static void crsfFrameDisplayPortChunk(sbuf_t *dst, sbuf_t *src, uint8_t batchId, uint8_t idx)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_DISPLAYPORT_CMD);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteU8(dst, CRSF_DISPLAYPORT_SUBCMD_UPDATE);
    uint8_t *metaPtr = sbufPtr(dst);
    sbufWriteU8(dst, batchId);
    sbufWriteU8(dst, idx);
    cRleEncodeStream(src, dst, CRSF_DISPLAYPORT_MAX_CHUNK_LENGTH);
    if (idx == 0)
        *metaPtr |= CRSF_DISPLAYPORT_FIRST_CHUNK_MASK;
    if (sbufBytesRemaining(src) == 0)
        *metaPtr |= CRSF_DISPLAYPORT_LAST_CHUNK_MASK;
}

static void crsfFrameDisplayPortClear(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_DISPLAYPORT_CMD);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteU8(dst, CRSF_DISPLAYPORT_SUBCMD_CLEAR);
}

void crsfProcessDisplayPortCmd(uint8_t *frameStart)
{
    const uint8_t cmd = frameStart[0];

    switch (cmd) {
        case CRSF_DISPLAYPORT_SUBCMD_OPEN:;
            const uint8_t rows = frameStart[CRSF_DISPLAYPORT_OPEN_ROWS_OFFSET];
            const uint8_t cols = frameStart[CRSF_DISPLAYPORT_OPEN_COLS_OFFSET];
            crsfDisplayPortSetDimensions(rows, cols);
            crsfDisplayPortMenuOpen();
            break;
        case CRSF_DISPLAYPORT_SUBCMD_CLOSE:
            crsfDisplayPortMenuExit();
            break;
        case CRSF_DISPLAYPORT_SUBCMD_POLL:
            crsfDisplayPortRefresh();
            break;
        default:
            break;
    }
}

#endif /* USE_CRSF_CMS_TELEMETRY */


#if defined(USE_MSP_OVER_TELEMETRY)

static bool mspReplyPending = false;
static uint8_t mspRequestOriginID = 0;

void crsfScheduleMspResponse(uint8_t requestOriginID)
{
    mspReplyPending = true;
    mspRequestOriginID = requestOriginID;
}

// sends MSP response chunk over CRSF. Must be of type mspResponseFnPtr
static void crsfSendMspResponse(uint8_t *payload, const uint8_t payloadSize)
{
    sbuf_t *dst = crsfInitializeSbuf();
    sbufWriteU8(dst, CRSF_FRAMETYPE_MSP_RESP);
    sbufWriteU8(dst, mspRequestOriginID);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteData(dst, payload, payloadSize);
    crsfFinalizeSbuf(dst);
}

#endif /* USE_MSP_OVER_TELEMETRY */


#if defined(USE_RX_EXPRESSLRS)

static int crsfFinalizeSbufBuf(sbuf_t *dst, uint8_t *frame)
{
    // frame size including CRC
    const size_t frameSize = sbufPtr(dst) - crsfFrame + 2;

    // Set frame length into the placeholder
    crsfFrame[1] = frameSize - 3;

    // frame CRC
    crc8_dvb_s2_sbuf_append(dst, &crsfFrame[2]); // start at byte 2, since CRC does not include device address and frame length

    // Copy data to the frame
    memcpy(frame, crsfFrame, frameSize);

    return frameSize;
}

int getCrsfFrame(uint8_t *frame, crsfFrameType_e frameType)
{
    sbuf_t *dst = crsfInitializeSbuf();

    switch (frameType) {
        default:
        case CRSF_FRAMETYPE_ATTITUDE:
            crsfFrameAttitude(dst);
            break;
        case CRSF_FRAMETYPE_VARIO_SENSOR:
            crsfFrameVarioSensor(dst);
            break;
        case CRSF_FRAMETYPE_ALTITUDE_SENSOR:
            crsfFrameAltitudeSensor(dst);
            break;
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            crsfFrameBatterySensor(dst);
            break;
        case CRSF_FRAMETYPE_FLIGHT_MODE:
            crsfFrameFlightMode(dst);
            break;
#if defined(USE_GPS)
        case CRSF_FRAMETYPE_GPS:
            crsfFrameGps(dst);
            break;
#endif
#if defined(USE_MSP_OVER_TELEMETRY)
        case CRSF_FRAMETYPE_DEVICE_INFO:
            crsfFrameDeviceInfo(dst);
            break;
#endif
    }

    return crsfFinalizeSbufBuf(dst, frame);
}

#if defined(USE_MSP_OVER_TELEMETRY)
int getCrsfMspFrame(uint8_t *frame, uint8_t *payload, const uint8_t payloadSize)
{
    sbuf_t *dst = crsfInitializeSbuf();

    sbufWriteU8(dst, CRSF_FRAMETYPE_MSP_RESP);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteData(dst, payload, payloadSize);

    return crsfFinalizeSbufBuf(dst, frame);
}
#endif /* USE_MSP_OVER_TELEMETRY */
#endif /* USE_RX_EXPRESSLRS */


bool checkCrsfTelemetryState(void)
{
    return crsfTelemetryEnabled;
}

void crsfScheduleDeviceInfoResponse(void)
{
    deviceInfoReplyPending = true;
}


static void processCrsfTelemetry(void)
{
    if (crsfRxIsTelemetryBufEmpty()) {
        telemetrySensor_t *sensor = telemetryScheduleNext();
        if (sensor) {
            sbuf_t *dst = crsfInitializeSbuf();
            switch (sensor->telid) {
                case TELEM_ATTITUDE:
                    crsfFrameAttitude(dst);
                    break;
                case TELEM_VARIOMETER:
                    crsfFrameVarioSensor(dst);
                    break;
                case TELEM_ALTITUDE:
                    crsfFrameAltitudeSensor(dst);
                    break;
                case TELEM_BATTERY:
                    crsfFrameBatterySensor(dst);
                    break;
                case TELEM_FLIGHT_MODE:
                    crsfFrameFlightMode(dst);
                    break;
#ifdef USE_GPS
                case TELEM_GPS:
                    crsfFrameGps(dst);
                    break;
#endif
                case TELEM_HEARTBEAT:
                default:
                    crsfFrameHeartbeat(dst);
                    break;
            }
            size_t bytes = crsfFinalizeSbuf(dst);
            bytes = crsfLinkFrameSize(bytes);
            telemetryScheduleCommit(sensor, bytes);
            telemetryScheduleCommit(crsfHeartBeatSensor, 0);
        }
    }
}

static void processCustomTelemetry(void)
{

    if (crsfRxIsTelemetryBufEmpty() && telemetryScheduleCanTransmit()) {
        sbuf_t *dst = crsfInitializeSbuf();
        crsfFrameCustomTelemetryHeader(dst);
if (0) {
        while (sbufBytesRemaining(dst) > 32) {
            telemetrySensor_t *sensor = telemetryScheduleNext();
            if (sensor) {
                uint8_t *ptr = sbufPtr(dst);
                crsfFrameCustomTelemetrySensor(dst, sensor);
                if (sbufBytesRemaining(dst) < 2) {
                    sbufReset(dst, ptr);
                    break;
                }
                telemetryScheduleCommit(sensor, 0);
            }
            else {
                break;
            }
        }
} else {
        {
            static uint32_t counter = 0;
            uint8_t size = (counter >> 7) & 0x3F;
            sbufWriteU16BE(dst, 0xBEEF);
            sbufWriteU32BE(dst, counter++);
            sbufWriteU8(dst, size);
            while (crsfSbufLen(dst) < size) {
                sbufWriteU8(dst, 0);
            }
        }
}
        if (crsfSbufLen(dst) > 8) {
            size_t bytes = crsfFinalizeSbuf(dst);
            bytes = crsfLinkFrameSize(bytes);
            telemetryScheduleCommit(NULL, bytes);
        }
    }
}

void handleCrsfTelemetry(timeUs_t currentTimeUs)
{
    if (!crsfTelemetryEnabled)
        return;

#if defined(USE_CRSF_V3)
    if (crsfBaudNegotiationInProgress())
        return;
#endif

    // Give the receiver a chance to send any outstanding telemetry data.
    // This needs to be done at high frequency, to enable the RX to send the telemetry frame
    // in between the RX frames.
    crsfRxSendTelemetryData();

    // Send ad-hoc response frames as soon as possible
#if defined(USE_MSP_OVER_TELEMETRY)
    if (mspReplyPending) {
        mspReplyPending = handleCrsfMspFrameBuffer(&crsfSendMspResponse);
        crsfNextCycleTime = currentTimeUs + CRSF_MSP_GRACE_US;
        return;
    }
#endif

    if (deviceInfoReplyPending) {
        sbuf_t *dst = crsfInitializeSbuf();
        crsfFrameDeviceInfo(dst);
        crsfFinalizeSbuf(dst);
        deviceInfoReplyPending = false;
        crsfNextCycleTime = currentTimeUs + CRSF_DEVICE_INFO_GRACE_US;
        return;
    }

#if defined(USE_CRSF_CMS_TELEMETRY)
    if (crsfDisplayPortScreen()->reset) {
        crsfDisplayPortScreen()->reset = false;
        sbuf_t *dst = crsfInitializeSbuf();
        crsfFrameDisplayPortClear(dst);
        crsfFinalizeSbuf(dst);
        crsfNextCycleTime = currentTimeUs + CRSF_CMS_GRACE_US;
        return;
    }
    static uint8_t displayPortBatchId = 0;
    if (crsfDisplayPortIsReady() && crsfDisplayPortScreen()->updated) {
        crsfDisplayPortScreen()->updated = false;
        uint16_t screenSize = crsfDisplayPortScreen()->rows * crsfDisplayPortScreen()->cols;
        uint8_t *srcStart = (uint8_t*)crsfDisplayPortScreen()->buffer;
        uint8_t *srcEnd = (uint8_t*)(crsfDisplayPortScreen()->buffer + screenSize);
        sbuf_t displayPortSbuf;
        sbuf_t *src = sbufInit(&displayPortSbuf, srcStart, srcEnd);
        displayPortBatchId = (displayPortBatchId  + 1) % CRSF_DISPLAYPORT_BATCH_MAX;
        uint8_t i = 0;
        while (sbufBytesRemaining(src)) {
            sbuf_t *dst = crsfInitializeSbuf();
            crsfFrameDisplayPortChunk(dst, src, displayPortBatchId, i);
            crsfFinalizeSbuf(dst);
            crsfRxSendTelemetryData();
            i++;
        }
        crsfNextCycleTime = currentTimeUs + CRSF_CMS_GRACE_US;
        return;
    }
#endif

    if (currentTimeUs >= crsfNextCycleTime) {
        telemetryScheduleUpdate(currentTimeUs);
        if (crsfCustomTelemetryEnabled)
            processCustomTelemetry();
        else
            processCrsfTelemetry();
        crsfNextCycleTime += crsfTelemetryInterval;
    }

    crsfRxSendTelemetryData();
}

static void INIT_CODE crsfInitNativeTelemetry(void)
{
    const float rate = telemetryConfig()->telemetry_link_rate;
    const float ratio = telemetryConfig()->telemetry_link_ratio;
    const float maxrate = 5 * rate / ratio;

    crsfTelemetryInterval = 1000000 * ratio / rate;

    telemetryScheduleInit(crsfNativeTelemetrySensors, ARRAYLEN(crsfNativeTelemetrySensors), maxrate);

    crsfHeartBeatSensor = crsfGetNativeSensor(TELEM_HEARTBEAT);
    telemetryScheduleAdd(crsfHeartBeatSensor);

    for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        sensor_id_e id = telemetryConfig()->telemetry_sensors[i];
        if (telemetrySensorActive(id)) {
            telemetrySensor_t * sensor = crsfGetNativeSensor(id);
            if (telemetryConfig()->telemetry_interval[i])
                sensor->min_interval = telemetryConfig()->telemetry_interval[i];
            telemetryScheduleAdd(sensor);
        }
    }
}

static void INIT_CODE crsfInitCustomTelemetry(void)
{
    const float rate = telemetryConfig()->telemetry_link_rate;
    const float ratio = telemetryConfig()->telemetry_link_ratio;
    const float maxrate = 5 * rate / ratio;

    crsfTelemetryInterval = 1000000 * ratio / rate;

    telemetryScheduleInit(crsfCustomTelemetrySensors, ARRAYLEN(crsfCustomTelemetrySensors), maxrate);

    for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        sensor_id_e id = telemetryConfig()->telemetry_sensors[i];
        if (telemetrySensorActive(id)) {
            telemetrySensor_t * sensor = crsfGetCustomSensor(id);
            if (telemetryConfig()->telemetry_interval[i])
                sensor->min_interval = telemetryConfig()->telemetry_interval[i];
            telemetryScheduleAdd(sensor);
        }
    }
}

void INIT_CODE initCrsfTelemetry(void)
{
    // check if there is a serial port open for CRSF telemetry (ie opened by the CRSF RX)
    // and feature is enabled, if so, set CRSF telemetry enabled
    crsfTelemetryEnabled = crsfRxIsActive();
    crsfCustomTelemetryEnabled = telemetryConfig()->custom_telemetry;

    if (crsfTelemetryEnabled)
    {
        deviceInfoReplyPending = false;
#if defined(USE_MSP_OVER_TELEMETRY)
        mspReplyPending = false;
#endif
#if defined(USE_CRSF_CMS_TELEMETRY)
        crsfDisplayportRegister();
#endif
        if (crsfCustomTelemetryEnabled)
            crsfInitCustomTelemetry();
        else
            crsfInitNativeTelemetry();
    }
}

#endif /* USE_TELEMETRY_CRSF */
