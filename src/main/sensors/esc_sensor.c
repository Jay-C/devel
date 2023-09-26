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

#include "platform.h"

#if defined(USE_ESC_SENSOR)

#include "config/feature.h"
#include "config/config.h"

#include "build/debug.h"

#include "blackbox/blackbox.h"

#include "common/time.h"
#include "common/crc.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/timer.h"
#include "drivers/motor.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "flight/mixer.h"

#include "io/serial.h"

#include "esc_sensor.h"



PG_REGISTER_WITH_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig, PG_ESC_SENSOR_CONFIG, 0);

PG_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig,
        .protocol = ESC_SENSOR_PROTO_NONE,
        .halfDuplex = 0,
        .update_hz = ESC_SENSOR_TASK_FREQ_HZ,
        .current_offset = 0,
        .hw4_current_offset = 0,
        .hw4_current_gain = 0,
        .hw4_voltage_gain = 0,
);


enum {
    DEBUG_ESC_1_RPM = 0,
    DEBUG_ESC_1_TEMP,
    DEBUG_ESC_1_VOLTAGE,
    DEBUG_ESC_1_CURRENT,
    DEBUG_ESC_2_RPM,
    DEBUG_ESC_2_TEMP,
    DEBUG_ESC_2_VOLTAGE,
    DEBUG_ESC_2_CURRENT,
};

enum {
    DEBUG_DATA_RPM = 0,
    DEBUG_DATA_PWM,
    DEBUG_DATA_TEMP,
    DEBUG_DATA_VOLTAGE,
    DEBUG_DATA_CURRENT,
    DEBUG_DATA_CAPACITY,
    DEBUG_DATA_EXTRA,
    DEBUG_DATA_AGE,
};

enum {
    DEBUG_FRAME_BYTE_COUNT = 0,
    DEBUG_FRAME_FRAME_COUNT,
    DEBUG_FRAME_SYNC_COUNT,
    DEBUG_FRAME_SYNC_ERRORS,
    DEBUG_FRAME_CRC_ERRORS,
    DEBUG_FRAME_TIMEOUTS,
    DEBUG_FRAME_BUFFER,
};

#define TELEMETRY_BUFFER_SIZE    40

static serialPort_t *escSensorPort = NULL;

static escSensorData_t escSensorData[MAX_SUPPORTED_MOTORS];
static escSensorData_t escSensorDataCombined;
static bool combinedNeedsUpdate = true;

static timeUs_t dataUpdateUs = 0;

static uint32_t totalByteCount = 0;
static uint32_t totalFrameCount = 0;
static uint32_t totalTimeoutCount = 0;
static uint32_t totalCrcErrorCount = 0;
static uint32_t totalSyncErrorCount = 0;

static uint8_t buffer[TELEMETRY_BUFFER_SIZE] = { 0, };

static volatile uint8_t bufferSize = 0;
static volatile uint8_t bufferPos = 0;

static uint8_t  readBytes = 0;
static uint32_t syncCount = 0;


bool isEscSensorActive(void)
{
    return escSensorPort != NULL;
}

uint16_t getEscSensorRPM(uint8_t motorNumber)
{
    return escSensorData[motorNumber].rpm;
}

static void combinedDataUpdate(void)
{
    const int motorCount = getMotorCount();

    if (combinedNeedsUpdate && motorCount > 0) {
        escSensorDataCombined.dataAge = 0;
        escSensorDataCombined.temperature = 0;
        escSensorDataCombined.voltage = 0;
        escSensorDataCombined.current = 0;
        escSensorDataCombined.consumption = 0;
        escSensorDataCombined.rpm = 0;

        for (int i = 0; i < motorCount; i++) {
            escSensorDataCombined.dataAge = MAX(escSensorDataCombined.dataAge, escSensorData[i].dataAge);
            escSensorDataCombined.temperature = MAX(escSensorDataCombined.temperature, escSensorData[i].temperature);
            escSensorDataCombined.voltage += escSensorData[i].voltage;
            escSensorDataCombined.current += escSensorData[i].current;
            escSensorDataCombined.consumption += escSensorData[i].consumption;
            escSensorDataCombined.rpm += escSensorData[i].rpm;
        }

        escSensorDataCombined.voltage = escSensorDataCombined.voltage / motorCount;
        escSensorDataCombined.rpm = escSensorDataCombined.rpm / motorCount;

        combinedNeedsUpdate = false;
    }
}

escSensorData_t * getEscSensorData(uint8_t motorNumber)
{
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_BLHELI32) {
            if (motorNumber < getMotorCount()) {
                return &escSensorData[motorNumber];
            }
            else if (motorNumber == ESC_SENSOR_COMBINED) {
                combinedDataUpdate();
                return &escSensorDataCombined;
            }
        }
        else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW4 ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_KONTRONIK ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_OMPHOBBY ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_ZTW) {
            if (motorNumber == 0 || motorNumber == ESC_SENSOR_COMBINED)
                return &escSensorData[0];
        }
    }

    return NULL;
}


/*
 * Common functions
 */

static void frameSyncError(void)
{
    readBytes = 0;
    syncCount = 0;

    totalSyncErrorCount++;
}

static void frameTimeoutError(void)
{
    readBytes = 0;
    syncCount = 0;

    totalTimeoutCount++;
}

static void increaseDataAge(uint8_t motor)
{
    if (escSensorData[motor].dataAge < ESC_DATA_INVALID) {
        escSensorData[motor].dataAge++;
        combinedNeedsUpdate = true;
    }

    DEBUG_AXIS(ESC_SENSOR_DATA, motor, DEBUG_DATA_AGE, escSensorData[motor].dataAge);
}

// Only for non-BLHeli32 protocols with single ESC support
static void checkFrameTimeout(timeUs_t currentTimeUs, timeDelta_t timeout)
{
    // Increment data age counter if no updates
    if (cmp32(currentTimeUs, dataUpdateUs) > timeout) {
        increaseDataAge(0);
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * BLHeli32 / KISS Telemetry Protocol
 *
 *     - Serial protocol is 115200,8N1
 *     - Big-Endian byte order
 *
 * Byte 0:      Temperature
 * Byte 1,2:    Voltage in 10mV
 * Byte 3,4:    Current in 10mA
 * Byte 5,6:    Consumption mAh
 * Byte 7,8:    RPM in 100rpm steps
 * Byte 9:      CRC8
 *
 */

#define BLHELI32_BOOT_DELAY       5000            // 5 seconds
#define BLHELI32_REQ_TIMEOUT      100             // 100 ms (data transfer takes only 900us)
#define BLHELI32_FRAME_SIZE       10

enum {
    BLHELI32_FRAME_FAILED    = 0,
    BLHELI32_FRAME_PENDING   = 1,
    BLHELI32_FRAME_COMPLETE  = 2,
};

enum {
    DSHOT_TRIGGER_WAIT = 0,
    DSHOT_TRIGGER_ACTIVE = 1,
};

static uint32_t dshotTriggerTimestamp = 0;
static uint8_t dshotTriggerState = DSHOT_TRIGGER_WAIT;

static uint8_t currentEsc = 0;


static FAST_CODE void blDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    totalByteCount++;

    if (bufferPos < bufferSize) {
        buffer[bufferPos++] = c;
    }
}

static void sendDShotTelemetryReqeust(timeMs_t currentTimeMs)
{
    bufferPos = 0;
    bufferSize = BLHELI32_FRAME_SIZE;

    dshotTriggerTimestamp = currentTimeMs;

    getMotorDmaOutput(currentEsc)->protocolControl.requestTelemetry = true;
}

static void blSelectNextEsc(void)
{
    currentEsc = (currentEsc + 1) % getMotorCount();
}

static uint8_t blDecodeTelemetryFrame(void)
{
    // First, check the variables that can change in the interrupt
    if (bufferPos < bufferSize)
        return BLHELI32_FRAME_PENDING;

    // Verify CRC8 checksum
    uint16_t chksum = crc8_kiss_update(0, buffer, BLHELI32_FRAME_SIZE - 1);
    uint16_t tlmsum = buffer[BLHELI32_FRAME_SIZE - 1];

    if (chksum == tlmsum) {
        uint16_t temp = buffer[0];
        uint16_t volt = buffer[1] << 8 | buffer[2];
        uint16_t curr = buffer[3] << 8 | buffer[4];
        uint16_t capa = buffer[5] << 8 | buffer[6];
        uint16_t erpm = buffer[7] << 8 | buffer[8];

        escSensorData[currentEsc].dataAge = 0;
        escSensorData[currentEsc].temperature = temp;
        escSensorData[currentEsc].voltage = volt;
        escSensorData[currentEsc].current = curr;
        escSensorData[currentEsc].consumption = capa;
        escSensorData[currentEsc].rpm = erpm;

        combinedNeedsUpdate = true;

        totalFrameCount++;

        if (currentEsc == 0) {
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, erpm * 100);
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, volt);
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, curr);
        }
        else if (currentEsc == 1) {
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_RPM, erpm * 100);
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_TEMP, temp * 10);
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_VOLTAGE, volt);
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_CURRENT, curr);
        }

        if (currentEsc == debugAxis) {
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, erpm);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, volt);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, curr);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capa);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);
        }

        return BLHELI32_FRAME_COMPLETE;
    }

    totalCrcErrorCount++;

    return BLHELI32_FRAME_FAILED;
}

static void blSensorProcess(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = currentTimeUs / 1000;

    switch (dshotTriggerState) {
        case DSHOT_TRIGGER_WAIT:
            if (currentTimeMs >= BLHELI32_BOOT_DELAY) {
                dshotTriggerState = DSHOT_TRIGGER_ACTIVE;
                sendDShotTelemetryReqeust(currentTimeMs);
            }
            break;

        case DSHOT_TRIGGER_ACTIVE:
            if (currentTimeMs < dshotTriggerTimestamp + BLHELI32_REQ_TIMEOUT) {
                uint8_t state = blDecodeTelemetryFrame();
                switch (state) {
                    case BLHELI32_FRAME_PENDING:
                        break;
                    case BLHELI32_FRAME_FAILED:
                        increaseDataAge(currentEsc);
                        FALLTHROUGH;
                    case BLHELI32_FRAME_COMPLETE:
                        blSelectNextEsc();
                        sendDShotTelemetryReqeust(currentTimeMs);
                        break;
                }
            }
            else {
                increaseDataAge(currentEsc);
                blSelectNextEsc();
                sendDShotTelemetryReqeust(currentTimeMs);
                totalTimeoutCount++;
            }
            break;
    }
}


/*
 * Calculate temperature from an NTC sensor ADC reading
 *
 * Let
 *     Rᵣ = Reference R
 *     Rₙ = NTC nominal R
 *     Tₙ = NTC nominal temp (25°C)
 *     Tₖ = 0°C in Kelvin = 273.15K
 *     β  = NTC beta
 *
 * and
 *
 *     T₁ = Tₖ + Tₙ = 298.15K
 *
 * Then
 *
 *          x         Rᵣ
 *  R = ―――――――――― ⋅ ――――
 *       4096 - x     Rₙ
 *
 *
 *             1
 *  T = ――――――――――――――― - Tₖ
 *       ln(R)/β + 1/T₁
 *
 *
 * Simplify:
 *
 *            1
 *  T = ―――――――――――――― - Tₖ
 *        γ⋅ln(S) + δ
 *
 * Where
 *          x
 *  S = ――――――――――
 *       4096 - x
 *
 *  γ = 1 / β
 *
 *  δ = γ⋅ln(Rᵣ/Rₙ) + 1/T₁
 *
 */

static float calcTempNTC(uint16_t adc, float gamma, float delta)
{
    const float X = constrainf(adc, 1, 4095);
    const float R = X / (4096 - X);
    const float A = logf(R) * gamma + delta;
    const float T = 1 / A - 273.15f;

    return T;
}


/*
 * Hobbywing V4 telemetry
 *
 *    - Serial protocol 19200,8N1
 *    - Big-Endian fields
 *
 * Data frame:
 *
 * Byte 0:          Sync 0x9B
 * Byte 1-3:        Packet counter
 * Byte 4-5:        Throttle
 * Byte 6-7:        PWM
 * Byte 8-10:       RPM
 * Byte 11-12:      Voltage
 * Byte 13-14:      Current
 * Byte 15-16:      Temperature (FETs)
 * Byte 17-18:      Temperature (CAP)
 * Byte 19:         Sync 0xB9 (present only in slow rate)
 *
 * Info frame:
 *
 * Byte 0:          Sync 0x9B
 * Byte 1:          Sync 0x9B
 * Byte 2,3:        Throttle steps (1000)
 * Byte 4:          RPM steps (1)
 * Byte 5-6:        Voltage constants
 * Byte 7-9:        Current constants
 * Byte 10-11:      Temperature constants
 * Byte 12:         Sync 0xB9
 *
 * Empirical gain values:
 *
 * Voltage Gain:
 *   3-6S  (LV):    gain = 110
 *   3-8S  (LVv2):  gain = 154
 *   5-12s (HV):    gain = 210
 *
 * Current Gain:
 *   60A:           gain = 60
 *   80A:           gain = 78
 *   100A:          gain = 90
 *   120A:          gain = 100
 *   130A:          gain = 113
 *   150A:          gain = 129
 *   160A:          gain = 137
 *   200A:          gain = 169
 *
 * Temp sensor design:
 *
 *  β  = 3950
 *  Tₙ = 25°C
 *  Rᵣ = 10k
 *  Rₙ = 47k
 *
 *  γ = 1 / β = 0.0002531…
 *
 *  δ = γ⋅ln(Rᵣ/Rₙ) + 1/T₁ = γ⋅ln(10/47) + 1/298.15 = 0.002962…
 */

#define HW4_GAMMA   0.00025316455696f
#define HW4_DELTA   0.00296226896087f

#define calcTempHW(adc)  calcTempNTC(adc, HW4_GAMMA, HW4_DELTA)

#define HW4_VOLTAGE_SCALE    0.0008056640625f
#define HW4_CURRENT_SCALE    32.2265625f

static float hw4VoltageScale = 0;
static float hw4CurrentScale = 0;
static float hw4CurrentOffset = 0;

static inline float calcVoltHW(uint16_t voltADC)
{
    return voltADC * hw4VoltageScale;
}

static inline float calcCurrHW(uint16_t currentADC)
{
    return (currentADC > hw4CurrentOffset) ?
        (currentADC - hw4CurrentOffset) * hw4CurrentScale : 0;
}

#define HW4_FRAME_NONE   0
#define HW4_FRAME_INFO   1
#define HW4_FRAME_DATA   2

static uint8_t processHW4TelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte == 0x9B) {
            syncCount++;
        }
        else if (dataByte == 0xB9) {
            readBytes = 0;
        }
        else {
            frameSyncError();
        }
    }
    else if (readBytes == 13) {
        if (buffer[1] == 0x9B && buffer[4] == 0x01 && buffer[12] == 0xB9) {
            readBytes = 0;
            if (syncCount > 2)
                return HW4_FRAME_INFO;
        }
    }
    else if (readBytes == 19) {
        readBytes = 0;
        if (syncCount > 2)
            return HW4_FRAME_DATA;
    }

    return HW4_FRAME_NONE;
}

static void hw4SensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        const uint8_t frameType = processHW4TelemetryStream(serialRead(escSensorPort));

        if (frameType == HW4_FRAME_DATA) {
            if (buffer[4] < 4 && buffer[6] < 4 && buffer[11] < 0x10 &&
                buffer[13] < 0x10 && buffer[15] < 0x10 && buffer[17] < 0x10) {

                //uint32_t cnt = buffer[1] << 16 | buffer[2] << 8 | buffer[3];
                //uint16_t thr = buffer[4] << 8 | buffer[5];
                uint16_t pwm = buffer[6] << 8 | buffer[7];
                uint32_t rpm = buffer[8] << 16 | buffer[9] << 8 | buffer[10];

                float voltage = calcVoltHW(buffer[11] << 8 | buffer[12]);
                float current = calcCurrHW(buffer[13] << 8 | buffer[14]);
                float tempFET = calcTempHW(buffer[15] << 8 | buffer[16]);
                float tempCAP = calcTempHW(buffer[17] << 8 | buffer[18]);

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = lrintf(tempFET);
                escSensorData[0].voltage = lrintf(voltage * 100);
                escSensorData[0].current = lrintf(current * 100);
                escSensorData[0].rpm = rpm / 100;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, lrintf(tempFET * 10));
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, lrintf(voltage * 100));
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, lrintf(current * 100));

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, lrintf(tempFET * 10));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, lrintf(voltage * 100));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, lrintf(current * 100));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, lrintf(tempCAP * 10));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
        else if (frameType == HW4_FRAME_INFO) {
            if (escSensorConfig()->hw4_voltage_gain)
                hw4VoltageScale = HW4_VOLTAGE_SCALE * escSensorConfig()->hw4_voltage_gain;
            else
                hw4VoltageScale = 0.1f * buffer[5] / buffer[6];

            if (escSensorConfig()->hw4_current_gain)
                hw4CurrentScale = HW4_CURRENT_SCALE / escSensorConfig()->hw4_current_gain;
            else
                hw4CurrentScale = 0; // Replace with data from ESC if possible

            hw4CurrentOffset = escSensorConfig()->hw4_current_offset;
        }
    }

    checkFrameTimeout(currentTimeUs, 1000000);
}


/*
 * Kontronik Telemetry V4
 *
 *    - Serial protocol is 115200,8E1
 *    - Little-Endian fields
 *    - CRC32
 *    - Error flags:
 *         0:  Undervoltage on the battery
 *         1:  Overvoltage on the battery
 *         2:  Overcurrent error
 *         3:  Overcurrent warning
 *         4:  Temperature warning
 *         5:  Temperature error
 *         6:  BEC under-voltage error
 *         7:  BEC over-voltage error
 *         8:  BEC over-current error
 *         9:  BEC temperature error
 *         10: Switch-off by rudder movement
 *         11: Capacity limit reached
 *         12: Operational error
 *         13: Operational warning
 *         14: Self-test error
 *         15: EEPROM error
 *         16: Watchdog error
 *         17: Programming is still permitted
 *         18: Battery limit reached
 *         19: Current limit reached
 *         20: ESC temperature limit reached
 *         21: BEC temperature limit reached
 *         22: ESC current limit reached
 *         23: Capacity limit reached
 *
 * Byte 0-3:        Sync 0x4B 0x4F 0x44 0x4C "KODL"
 * Byte 4-7:        RPM
 * Byte 8-9:        Battery voltage in 10mV
 * Byte 10-11:      Battery current in 0.1A
 * Byte 12-13:      Motor current average in 0.1A
 * Byte 14-15:      Motor current peak in 0.1A
 * Byte 16-17:      Capacity in mAh
 * Byte 18-19:      BEC current in mA
 * Byte 20-21:      BEC Voltage n mV
 * Byte 22-23:      PWM in us
 * Byte 24:         Throttle % (-100..100)
 * Byte 25:         Output throttle 0..100%
 * Byte 26:         FET temperature -128..127°C
 * Byte 27:         BEC temperature -128..127°C
 * Byte 28-31:      Error Flags
 * Byte 32:         Operational condition
 * Byte 33:         Timing 0..30
 * Byte 34-37:      CRC32
 *
 */

static uint32_t calculateCRC32(const uint8_t *ptr, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;

    while (len--) {
        crc ^= *ptr++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
    }

    return ~crc;
}

static bool processKontronikTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0x4B)
            frameSyncError();
    }
    else if (readBytes == 2) {
        if (dataByte != 0x4F)
            frameSyncError();
    }
    else if (readBytes == 3) {
        if (dataByte != 0x44)
            frameSyncError();
    }
    else if (readBytes == 4) {
        if (dataByte != 0x4C)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 38) {
        readBytes = 0;
        if (syncCount > 2)
            return true;
    }

    return false;
}

static void kontronikSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processKontronikTelemetryStream(serialRead(escSensorPort))) {
            uint32_t crc = buffer[37] << 24 | buffer[36] << 16 | buffer[35] << 8 | buffer[34];

            if (calculateCRC32(buffer, 34) == crc) {
                uint32_t rpm = buffer[7] << 24 | buffer[6] << 16 | buffer[5] << 8 | buffer[4];
                uint16_t pwm = buffer[23] << 8 | buffer[22];
                uint16_t voltage = buffer[9] << 8 | buffer[8];
                uint16_t current = buffer[11] << 8 | buffer[10];
                uint16_t capacity = buffer[17] << 8 | buffer[16];
                uint16_t tempFET = buffer[26];
                uint16_t tempBEC = buffer[27];

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = tempFET;
                escSensorData[0].voltage = voltage;
                escSensorData[0].current = current;
                escSensorData[0].rpm = rpm / 100;
                escSensorData[0].consumption = capacity;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, tempFET * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, tempFET);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capacity);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, tempBEC);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    checkFrameTimeout(currentTimeUs, 1000000);
}


/*
 * OMP Hobby M4 Telemetry
 *
 *    - Serial protocol is 115200,8N1
 *    - Frame length includes header and CRC
 *    - Big-Endian fields
 *    - Status Code bits:
 *         0:  Short-circuit protection
 *         1:  Motor connection error
 *         2:  Throttle signal lost
 *         3:  Throttle signal >0 on startup error
 *         4:  Low voltage protection
 *         5:  Temperature protection
 *         6:  Startup protection
 *         7:  Current protection
 *         8:  Throttle signal error
 *        12:  Battery voltage error
 *
 * Byte 0:          Start Flag 0xdd
 * Byte 1:          Protocol version 0x01
 * Byte 2:          Frame lenght (32 for v1)
 * Byte 3-4:        Battery voltage in 0.1V
 * Byte 5-6:        Battery current in 0.1V
 * Byte 7:          Input Throttle in %
 * Byte 8-9:        RPM in 10rpm steps
 * Byte 10:         ESC Temperature
 * Byte 11:         Motor Temperature
 * Byte 12:         PWM Throttle in %
 * Byte 13-14:      Status Code
 * Byte 15-16:      Capacity mAh
 * Byte 17-31:      Unused / Zeros
 *
 */

static bool processOMPTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0xDD)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 32) {
        readBytes = 0;
        if (syncCount > 2)
            return true;
    }

    return false;
}

static void ompSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processOMPTelemetryStream(serialRead(escSensorPort))) {
            // Make sure this is OMP M4 ESC
            if (buffer[1] == 0x01 && buffer[2] == 0x20 && buffer[11] == 0 && buffer[18] == 0 && buffer[20] == 0) {
                uint16_t rpm = buffer[8] << 8 | buffer[9];
                uint16_t pwm = buffer[12];
                uint16_t temp = buffer[10];
                uint16_t voltage = buffer[3] << 8 | buffer[4];
                uint16_t current = buffer[5] << 8 | buffer[6];
                uint16_t capacity = buffer[15] << 8 | buffer[16];
                uint16_t status = buffer[13] << 8 | buffer[14];

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = temp;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 10;
                escSensorData[0].rpm = rpm / 10;
                escSensorData[0].consumption = 0; // capacity; // FIXME bogus value

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 10);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capacity);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, status);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    checkFrameTimeout(currentTimeUs, 1000000);
}


/*
 * ZTW Telemetry
 *
 *    - Serial protocol is 115200,8N1
 *    - Frame length includes header and CRC
 *    - Big-Endian fields
 *    - Checksum (unknown)
 *    - Status Code bits:
 *         0:  Short-circuit protection
 *         1:  Motor connection error
 *         2:  Throttle signal lost
 *         3:  Throttle signal >0 on startup error
 *         4:  Low voltage protection
 *         5:  Temperature protection
 *         6:  Startup protection
 *         7:  Current protection
 *         8:  Throttle signal error
 *         9:  UART throttle error
 *        10:  UART throttle lost
 *        11:  CAN throttle lost
 *        12:  Battery voltage error
 *
 * Byte 0:          Start Flag 0xdd
 * Byte 1:          Protocol version 0x01
 * Byte 2:          Frame lenght (32 for v1)
 * Byte 3-4:        Battery voltage in 0.1V
 * Byte 5-6:        Battery current in 0.1V
 * Byte 7:          Input Throttle in %
 * Byte 8-9:        RPM in 10rpm steps
 * Byte 10:         ESC Temperature
 * Byte 11:         Motor Temperature
 * Byte 12:         PWM Throttle in %
 * Byte 13-14:      Status Code
 * Byte 15-16:      Capacity mAh
 * Byte 17:         Serial Throttle input (unused)
 * Byte 18:         CAN Throttle input (unused)
 * Byte 19:         BEC Voltage
 * Byte 20-29:      Unused
 * Byte 30-31:      Checksum
 *
 */

static bool processZTWTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0xDD)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 32) {
        readBytes = 0;
        if (syncCount > 2)
            return true;
    }

    return false;
}

static void ztwSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processZTWTelemetryStream(serialRead(escSensorPort))) {
            if (buffer[1] == 0x01 && buffer[2] == 0x20) {
                uint16_t rpm = buffer[8] << 8 | buffer[9];
                uint16_t temp = buffer[10];
                uint16_t power = buffer[12];
                uint16_t voltage = buffer[3] << 8 | buffer[4];
                uint16_t current = buffer[5] << 8 | buffer[6];
                uint16_t capacity = buffer[15] << 8 | buffer[16];
                uint16_t status = buffer[13] << 8 | buffer[14];

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = temp;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 10;
                escSensorData[0].rpm = rpm / 10;
                escSensorData[0].consumption = capacity;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 10);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, power);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capacity);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, status);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    checkFrameTimeout(currentTimeUs, 1000000);
}


/*
 * Raw Telemetry Data Recorder
 */

static void recordSensorProcess(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort) && readBytes < 32) {
        totalByteCount++;
        buffer[readBytes++] = serialRead(escSensorPort);
    }

    if (readBytes > 0) {
        blackboxLogCustomData(buffer, readBytes);
        totalFrameCount++;
        readBytes = 0;
    }
}


void escSensorProcess(timeUs_t currentTimeUs)
{
    if (escSensorPort && motorIsEnabled()) {
        switch (escSensorConfig()->protocol) {
            case ESC_SENSOR_PROTO_BLHELI32:
                blSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_HW4:
                hw4SensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_KONTRONIK:
                kontronikSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_OMPHOBBY:
                ompSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_ZTW:
                ztwSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_RECORD:
                recordSensorProcess(currentTimeUs);
                break;
        }

        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_BYTE_COUNT, totalByteCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_FRAME_COUNT, totalFrameCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_SYNC_COUNT, syncCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_SYNC_ERRORS, totalSyncErrorCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_CRC_ERRORS, totalCrcErrorCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_TIMEOUTS, totalTimeoutCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_BUFFER, readBytes);
    }
}

bool INIT_CODE escSensorInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESC_SENSOR);
    serialReceiveCallbackPtr callback = NULL;
    portOptions_e options = 0;
    uint32_t baudrate = 0;

    if (!portConfig) {
        return false;
    }

    options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);

    switch (escSensorConfig()->protocol) {
        case ESC_SENSOR_PROTO_BLHELI32:
            callback = blDataReceive;
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_HW4:
            baudrate = 19200;
            break;
        case ESC_SENSOR_PROTO_KONTRONIK:
            baudrate = 115200;
            options |= SERIAL_PARITY_EVEN;
            break;
        case ESC_SENSOR_PROTO_OMPHOBBY:
        case ESC_SENSOR_PROTO_ZTW:
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_RECORD:
            baudrate = baudRates[portConfig->telemetry_baudrateIndex];
            break;
    }

    if (baudrate) {
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, callback, NULL, baudrate, MODE_RX, options);
    }

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        escSensorData[i].dataAge = ESC_DATA_INVALID;
    }

    return (escSensorPort != NULL);
}

#endif
