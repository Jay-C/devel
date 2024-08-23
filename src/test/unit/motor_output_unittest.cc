/*
 * This file is part of Betaflight.
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

#include <stdint.h>
#include <iostream>

extern "C" {
#include "drivers/dshot.h"
#include "build/atomic.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {

bool featureIsEnabled(uint8_t f);
float scaleRangef(float a, float b, float c, float d, float e);

// Mocking functions

bool featureIsEnabled(uint8_t f)
{
    UNUSED(f);
    return true;
}

float scaleRangef(float a, float b, float c, float d, float e)
{
    UNUSED(a);
    UNUSED(b);
    UNUSED(c);
    UNUSED(d);
    UNUSED(e);
    return 0;
}

}

