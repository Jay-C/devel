/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "pg/pg.h"

typedef struct {
    uint8_t gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t gyro_hardware_lpf;                // gyro DLPF setting
    uint8_t gyro_high_fsr;
    uint8_t gyro_to_use;

    uint16_t gyro_lpf1_static_hz;
    uint16_t gyro_lpf2_static_hz;

    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
    int16_t gyro_offset_yaw;
    uint8_t checkOverflow;

    // Lowpass primary/secondary
    uint8_t gyro_lpf1_type;
    uint8_t gyro_lpf2_type;

    uint16_t gyroCalibrationDuration;   // Gyro calibration duration in 1/100 second

    uint8_t gyro_filter_debug_axis;

    uint8_t gyrosDetected; // What gyros should detection be attempted for on startup. Automatically set on first startup.
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

