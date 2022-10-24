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

#include "platform.h"

#if defined(USE_ESC_SENSOR)

#include "build/debug.h"

#include "common/time.h"

#include "config/feature.h"
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

#include "esc_sensor.h"

#include "config/config.h"

#include "flight/mixer.h"

#include "io/serial.h"


PG_REGISTER_WITH_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig, PG_ESC_SENSOR_CONFIG, 0);

PG_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig,
        .protocol = ESC_SENSOR_PROTO_KISS,
        .halfDuplex = 0,
        .offset = 0,
        .update_hz = ESC_SENSOR_TASK_FREQ_HZ,
);


enum {
    DEBUG_ESC_MOTOR_INDEX = 0,
    DEBUG_ESC_NUM_TIMEOUTS = 1,
    DEBUG_ESC_NUM_CRC_ERRORS = 2,
    DEBUG_ESC_DATA_AGE = 3,
};

typedef enum {
    ESC_SENSOR_FRAME_PENDING = 0,
    ESC_SENSOR_FRAME_COMPLETE = 1,
    ESC_SENSOR_FRAME_FAILED = 2
} escTlmFrameState_t;

typedef enum {
    ESC_SENSOR_TRIGGER_STARTUP = 0,
    ESC_SENSOR_TRIGGER_PENDING = 1,
} escSensorTriggerState_t;

#define ESC_SENSOR_BAUDRATE 115200
#define ESC_BOOTTIME 5000               // 5 seconds
#define ESC_REQUEST_TIMEOUT 100         // 100 ms (data transfer takes only 900us)

#define TELEMETRY_FRAME_SIZE 10

static uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE] = { 0, };

static volatile uint8_t *buffer;
static volatile uint8_t bufferSize = 0;
static volatile uint8_t bufferPosition = 0;

static serialPort_t *escSensorPort = NULL;

static escSensorData_t escSensorData[MAX_SUPPORTED_MOTORS];

static escSensorTriggerState_t escSensorTriggerState = ESC_SENSOR_TRIGGER_STARTUP;
static uint32_t escTriggerTimestamp;
static uint8_t escSensorMotor = 0;      // motor index

static escSensorData_t combinedEscSensorData;
static bool combinedDataNeedsUpdate = true;

static uint16_t totalTimeoutCount = 0;
static uint16_t totalCrcErrorCount = 0;


void startEscDataRead(uint8_t *frameBuffer, uint8_t frameLength)
{
    buffer = frameBuffer;
    bufferPosition = 0;
    bufferSize = frameLength;
}

uint8_t getNumberEscBytesRead(void)
{
    return bufferPosition;
}

bool isEscSensorActive(void)
{
    return escSensorPort != NULL;
}

uint16_t getEscSensorRPM(uint8_t motorNumber)
{
    return escSensorData[motorNumber].rpm;
}

escSensorData_t * getEscSensorData(uint8_t motorNumber)
{
    if (!featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return NULL;
    }

    if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KISS) {
        if (motorNumber < getMotorCount()) {
            return &escSensorData[motorNumber];
        }
        else if (motorNumber == ESC_SENSOR_COMBINED) {
            if (combinedDataNeedsUpdate && getMotorCount() > 0) {
                combinedEscSensorData.dataAge = 0;
                combinedEscSensorData.temperature = 0;
                combinedEscSensorData.voltage = 0;
                combinedEscSensorData.current = 0;
                combinedEscSensorData.consumption = 0;
                combinedEscSensorData.rpm = 0;

                for (int i = 0; i < getMotorCount(); i = i + 1) {
                    combinedEscSensorData.dataAge = MAX(combinedEscSensorData.dataAge, escSensorData[i].dataAge);
                    combinedEscSensorData.temperature = MAX(combinedEscSensorData.temperature, escSensorData[i].temperature);
                    combinedEscSensorData.voltage += escSensorData[i].voltage;
                    combinedEscSensorData.current += escSensorData[i].current;
                    combinedEscSensorData.consumption += escSensorData[i].consumption;
                    combinedEscSensorData.rpm += escSensorData[i].rpm;
                }

                combinedEscSensorData.voltage = combinedEscSensorData.voltage / getMotorCount();
                combinedEscSensorData.rpm = combinedEscSensorData.rpm / getMotorCount();

                combinedDataNeedsUpdate = false;

                DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_DATA_AGE, combinedEscSensorData.dataAge);
            }

            return &combinedEscSensorData;
        }
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW4) {
        return &escSensorData[0];
    }

    return NULL;
}

static inline bool isFrameComplete(void)
{
    return bufferPosition == bufferSize;
}

// Receive ISR callback
static FAST_CODE void escSensorDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    // KISS ESC sends some data during startup, ignore this for now (maybe future use)
    // startup data could be firmware version and serialnumber

    if (isFrameComplete()) {
        return;
    }

    buffer[bufferPosition++] = (uint8_t)c;
}

bool escSensorInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESC_SENSOR);
    if (!portConfig) {
        return false;
    }

    if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KISS) {
        portOptions_e options = SERIAL_NOT_INVERTED  | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);

        // Initialize serial port
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, escSensorDataReceive, NULL, ESC_SENSOR_BAUDRATE, MODE_RX, options);
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW4) {
        portOptions_e options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);

        // Initialize serial port with no callback.  We will just process the buffer.
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, NULL, NULL, 19200, MODE_RX, options);
    }

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i = i + 1) {
        escSensorData[i].dataAge = ESC_DATA_INVALID;
    }

    return (escSensorPort != NULL);
}


/*
 * KISS ESC TELEMETRY PROTOCOL
 * ---------------------------
 *
 * One transmission will have 10 times 8-bit bytes sent with 115200 baud and 3.6V.
 *
 * Byte 0: Temperature
 * Byte 1: Voltage high byte
 * Byte 2: Voltage low byte
 * Byte 3: Current high byte
 * Byte 4: Current low byte
 * Byte 5: Consumption high byte
 * Byte 6: Consumption low byte
 * Byte 7: Rpm high byte
 * Byte 8: Rpm low byte
 * Byte 9: 8-bit CRC
 *
 */

static uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = crc;
    crc_u ^= crc_seed;

    for (int i=0; i<8; i++) {
        crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }

    return (crc_u);
}

uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen)
{
    uint8_t crc = 0;
    for (int i = 0; i < BufLen; i++) {
        crc = updateCrc8(Buf[i], crc);
    }

    return crc;
}

static uint8_t decodeEscFrame(void)
{
    if (!isFrameComplete()) {
        return ESC_SENSOR_FRAME_PENDING;
    }

    // Get CRC8 checksum
    uint16_t chksum = calculateCrc8(telemetryBuffer, TELEMETRY_FRAME_SIZE - 1);
    uint16_t tlmsum = telemetryBuffer[TELEMETRY_FRAME_SIZE - 1];     // last byte contains CRC value
    uint8_t frameStatus;
    if (chksum == tlmsum) {
        escSensorData[escSensorMotor].dataAge = 0;
        escSensorData[escSensorMotor].temperature = telemetryBuffer[0];
        escSensorData[escSensorMotor].voltage = telemetryBuffer[1] << 8 | telemetryBuffer[2];
        escSensorData[escSensorMotor].current = telemetryBuffer[3] << 8 | telemetryBuffer[4];
        escSensorData[escSensorMotor].consumption = telemetryBuffer[5] << 8 | telemetryBuffer[6];
        escSensorData[escSensorMotor].rpm = telemetryBuffer[7] << 8 | telemetryBuffer[8];

        combinedDataNeedsUpdate = true;

        frameStatus = ESC_SENSOR_FRAME_COMPLETE;

        if (escSensorMotor < 4) {
            DEBUG_SET(DEBUG_ESC_SENSOR_RPM, escSensorMotor, calcMotorRPM(escSensorMotor, escSensorData[escSensorMotor].rpm) / 10); // output actual rpm/10 to fit in 16bit signed.
            DEBUG_SET(DEBUG_ESC_SENSOR_TMP, escSensorMotor, escSensorData[escSensorMotor].temperature);
        }
    } else {
        frameStatus = ESC_SENSOR_FRAME_FAILED;
    }

    return frameStatus;
}

static void increaseDataAge(void)
{
    if (escSensorData[escSensorMotor].dataAge < ESC_DATA_INVALID) {
        escSensorData[escSensorMotor].dataAge++;

        combinedDataNeedsUpdate = true;
    }
}

static void selectNextMotor(void)
{
    escSensorMotor++;
    if (escSensorMotor == getMotorCount()) {
        escSensorMotor = 0;
    }
}

static void setRequest(timeMs_t currentTimeMs)
{
    startEscDataRead(telemetryBuffer, TELEMETRY_FRAME_SIZE);
    getMotorDmaOutput(escSensorMotor)->protocolControl.requestTelemetry = true;

    escSensorTriggerState = ESC_SENSOR_TRIGGER_PENDING;
    escTriggerTimestamp = currentTimeMs;

    DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_MOTOR_INDEX, escSensorMotor + 1);
}

static void kissSensorProcess(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = currentTimeUs / 1000;

    switch (escSensorTriggerState) {
        case ESC_SENSOR_TRIGGER_STARTUP:
            // Wait period of time before requesting telemetry (let the system boot first)
            if (currentTimeMs >= ESC_BOOTTIME) {
                setRequest(currentTimeMs);
            }
            break;

        case ESC_SENSOR_TRIGGER_PENDING:
            if (currentTimeMs < escTriggerTimestamp + ESC_REQUEST_TIMEOUT) {
                uint8_t state = decodeEscFrame();
                switch (state) {
                    case ESC_SENSOR_FRAME_COMPLETE:
                        selectNextMotor();
                        setRequest(currentTimeMs);
                        break;
                    case ESC_SENSOR_FRAME_FAILED:
                        increaseDataAge();
                        selectNextMotor();
                        setRequest(currentTimeMs);
                        DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_NUM_CRC_ERRORS, ++totalCrcErrorCount);
                        break;
                    case ESC_SENSOR_FRAME_PENDING:
                        break;
                }
            } else {
                // Move on to next ESC, we'll come back to this one
                increaseDataAge();
                selectNextMotor();
                setRequest(currentTimeMs);
                DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_NUM_TIMEOUTS, ++totalTimeoutCount);
            }
            break;
    }
}

/*
 * Hobbywing V4 telemetry
 *
 * Credit to:  https://github.com/dgatf/msrc/
 *
 */

static uint8_t telemetryData[18] = { 0, };
static uint8_t skipPackets = 0;
static uint8_t bytesRead = 0;

static timeUs_t lastProcessTimeUs = 0;
static float consumption = 0.0f;

static bool processHW4TelemetryStream(uint8_t dataByte)
{
    if (skipPackets > 0) {
        // Ignore the data in these ?non-telemetry? packets while throttle = 0
        skipPackets--;
    }
    else if (bytesRead == 0 && dataByte == 0x9B) {
        // Start of a potentially valid read
        bytesRead = 1;
    }
    else if (bytesRead == 1 && dataByte == 0x9B) {
        // We received two 0x9B in a row at the start of a read.. invalid packet.
        // For the first byte of the packet counter to be 0x9B you would need 10,158,080 data packets.
        //   That's 84,650 seconds at 120Hz tranmission rate.  So not very likely to occur accidentally.
        bytesRead = 0;
        skipPackets = 11;
    }
    else if (bytesRead > 0) {
        // Store each portion of what looks to be a valid data packet
        telemetryData[bytesRead-1] = dataByte;
        bytesRead++;
        if (bytesRead == 19) {
            bytesRead = 0;
            return true;
        }
    }
    return false;
}

#define HW4_V_REF            3.3f
#define HW4_DIFFAMP_GAIN     13.6f
#define HW4_DIFFAMP_SHUNT    (0.25f / 1000.0f)
#define HW4_ADC_RESOLUTION   4096.0f
#define HW4_NTC_BETA         3950.0f
#define HW4_NTC_R1           10000.0f
#define HW4_NTC_R_REF        47000.0f

static float calcTempHW(uint16_t tempRaw)
{
    float voltage = tempRaw * HW4_V_REF / HW4_ADC_RESOLUTION;
    float ntcR_Rref = (voltage * HW4_NTC_R1 / (HW4_V_REF - voltage)) / HW4_NTC_R_REF;

    float temperature = 1.0f / (logf(ntcR_Rref) / HW4_NTC_BETA + 1.0f / 298.15f) - 273.15f;

    if (temperature < 0)
        return 0;

    return temperature;
}

static float calcVoltHW(uint16_t voltRaw)
{
    return HW4_V_REF * voltRaw / HW4_ADC_RESOLUTION * escSensorConfig()->hw4_voltage_div;
}

static float calcCurrHW(uint16_t currentRaw)
{
    if (currentRaw > escSensorConfig()->hw4_current_offset) {
        return (currentRaw - escSensorConfig()->hw4_current_offset) *
            (escSensorConfig()->hw4_current_scale / 100.0f) *
            (HW4_V_REF / (HW4_DIFFAMP_GAIN * HW4_DIFFAMP_SHUNT * HW4_ADC_RESOLUTION));
    }

    return 0;
}

static void hw4SensorProcess(timeUs_t currentTimeUs)
{
    //  Only supports one motor
    escSensorMotor = 0;

    // Increment data aging so we'll know if we don't get a valid data packet on a ESC sensor read
    escSensorData[escSensorMotor].dataAge++;

    // check for any available ESC telemetry bytes in the buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processHW4TelemetryStream(serialRead(escSensorPort))) {
            //uint16_t thr = (uint16_t)telemetryData[3] << 8 | telemetryData[4]; // 0-1024
            //uint16_t pwm = (uint16_t)telemetryData[5] << 8 | telemetryData[6]; // 0-1024
            float rpm = (uint32_t)telemetryData[7] << 16 | (uint32_t)telemetryData[8] << 8 | (uint32_t)telemetryData[9];
            float voltage = calcVoltHW((uint16_t)telemetryData[10] << 8 | (uint16_t)telemetryData[11]);
            float current = calcCurrHW((uint16_t)telemetryData[12] << 8 | (uint16_t)telemetryData[13]);
            float tempFET = calcTempHW((uint16_t)telemetryData[14] << 8 | (uint16_t)telemetryData[15]);

            escSensorData[escSensorMotor].dataAge = 0;
            escSensorData[escSensorMotor].temperature = tempFET;
            escSensorData[escSensorMotor].voltage = voltage * 100;
            escSensorData[escSensorMotor].current = current * 100;
            escSensorData[escSensorMotor].rpm = rpm / 100;

            //DEBUG(ESC_SENSOR_RPM, 0, calcEscRpm(escSensorMotor, escSensorData[escSensorMotor].rpm));
            DEBUG(ESC_SENSOR_TMP, 0, escSensorData[escSensorMotor].temperature);

            // Increment counter every time we decode a Hobbywing telemetry packet
            DEBUG(ESC_SENSOR, DEBUG_ESC_NUM_CRC_ERRORS, ++totalCrcErrorCount);

        }

        // Increment counter every time a new byte is read over the uart
        DEBUG(ESC_SENSOR, DEBUG_ESC_NUM_TIMEOUTS, ++totalTimeoutCount);
    }

    // Log the data age to see how old the data gets between HW telemetry packets
    DEBUG(ESC_SENSOR, DEBUG_ESC_DATA_AGE, escSensorData[escSensorMotor].dataAge);

    // Hobbywing just reports the last current reading it had when throttle goes to zero.  That's completely useless, so set it to zero.
    // RF TODO:  Consider reading the actual throttle telemetry from the HW ESC protocol instead of RPM to make this more resistant to error?
    if (escSensorData[escSensorMotor].rpm < 1) {
        escSensorData[escSensorMotor].current = 0.0f;
    }

    // Accumulate consumption (mAh) as a float since we're updating at 100Hz... even 100A for 10ms is only 0.28 mAh.
    //  Calculate it using the last valid current reading we received
    consumption += (currentTimeUs - lastProcessTimeUs) * escSensorData[escSensorMotor].current * 10.0f / 1000000.0f / 3600.0f;
    lastProcessTimeUs = currentTimeUs;

    escSensorData[escSensorMotor].consumption = lrintf(consumption);
}


void escSensorProcess(timeUs_t currentTimeUs)
{
    if (!escSensorPort || !motorIsEnabled()) {
        return;
    }

    if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KISS) {
        kissSensorProcess(currentTimeUs);
    }
    else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW4) {
        hw4SensorProcess(currentTimeUs);
    }
}


#endif
