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

#include <math.h>

#include "platform.h"
#include "types.h"

#include "build/build_config.h"

#include "common/axis.h"

#include "maths.h"

#ifndef USE_STANDARD_MATH

#define sinPolyCoef3 -1.666665709650470145824129400050267289858e-1f
#define sinPolyCoef5  8.333017291562218127986291618761571373087e-3f
#define sinPolyCoef7 -1.980661520135080504411629636078917643846e-4f
#define sinPolyCoef9  2.600054767890361277123254766503271638682e-6f

FAST_CODE float sin_approx(float x)
{
    x = fmodf(x, 2.0f * M_PIf);

    // Wrap angle to 2π with range [-π π]
    if (x > M_PIf)
        x -= M_2PIf;
    else if (x < -M_PIf)
        x += M_2PIf;

    // Use axis symmetry around x = ±π/2 for polynomial outside of [-π/2 π/2]
    if (x >  M_PI2f)
        x =  M_PIf - x;
    else if (x < -M_PI2f)
        x = -M_PIf - x;

    const float x2 = x * x;

    return x + x *
        x2 * (sinPolyCoef3 +
        x2 * (sinPolyCoef5 +
        x2 * (sinPolyCoef7 +
        x2 * (sinPolyCoef9))));
}

FAST_CODE float cos_approx(float x)
{
    return sin_approx(x + M_PI2f);
}

FAST_CODE float asin_approx(float x)
{
    return M_PI2f - acos_approx(x);
}

FAST_CODE float acos_approx(float x)
{
    const float xa = fabsf(x);

    float result = sqrtf(1.0f - xa) *
        (1.57072880f + xa *
        (-0.2121144f + xa *
        (0.07426100f + xa *
        (-0.0187293f))));

    if (x < 0)
        result = M_PIf - result;

    return result;
}

#define atanPolyCoef1  3.14551665884836e-07f
#define atanPolyCoef2  0.99997356613987f
#define atanPolyCoef3  0.14744007058297684f
#define atanPolyCoef4  0.3099814292351353f
#define atanPolyCoef5  0.05030176425872175f
#define atanPolyCoef6  0.1471039133652469f
#define atanPolyCoef7  0.6444640676891548f

FAST_CODE float atan2_approx(float y, float x)
{
    float absX = fabsf(x);
    float absY = fabsf(y);
    float res  = MAX(absX, absY);

    if (res)
        res = MIN(absX, absY) / res;
    else
        res = 0.0f;

    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);

    if (absY > absX)
        res = M_PI2f - res;

    if (x < 0)
        res = M_PIf - res;
    if (y < 0)
        res = -res;

    return res;
}

#endif /* USE_STANDARD_MATH */

// Cubic polynomial blending function
static float cubicBlend(const float t)
{
    return t * t * (3.0f - 2.0f * t);
}

// Smooth step-up transition function from 0 to 1
float smoothStepUpTransition(const float x, const float center, const float width)
{
    const float half_width = width * 0.5f;
    const float left_limit = center - half_width;
    const float right_limit = center + half_width;

    if (x < left_limit) {
        return 0.0f;
    } else if (x > right_limit) {
        return 1.0f;
    } else {
        const float t = (x - left_limit) / width; // Normalize x within the range
        return cubicBlend(t);
    }
}

void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}


// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }
#define QMF_SORTF(a,b) { if ((a)>(b)) QMF_SWAPF((a),(b)); }
#define QMF_SWAPF(a,b) { float temp=(a);(a)=(b);(b)=temp; }

int32_t quickMedianFilter3(const int32_t * v)
{
    int32_t p[3];
    QMF_COPY(p, v, 3);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[1], p[2]); QMF_SORT(p[0], p[1]) ;
    return p[1];
}

int32_t quickMedianFilter5(const int32_t * v)
{
    int32_t p[5];
    QMF_COPY(p, v, 5);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[2]); QMF_SORT(p[2], p[3]);
    QMF_SORT(p[1], p[2]);
    return p[2];
}

int32_t quickMedianFilter7(const int32_t * v)
{
    int32_t p[7];
    QMF_COPY(p, v, 7);

    QMF_SORT(p[0], p[5]); QMF_SORT(p[0], p[3]); QMF_SORT(p[1], p[6]);
    QMF_SORT(p[2], p[4]); QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[5]);
    QMF_SORT(p[2], p[6]); QMF_SORT(p[2], p[3]); QMF_SORT(p[3], p[6]);
    QMF_SORT(p[4], p[5]); QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[3]);
    QMF_SORT(p[3], p[4]);
    return p[3];
}

int32_t quickMedianFilter9(const int32_t * v)
{
    int32_t p[9];
    QMF_COPY(p, v, 9);

    QMF_SORT(p[1], p[2]); QMF_SORT(p[4], p[5]); QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[6], p[7]);
    QMF_SORT(p[1], p[2]); QMF_SORT(p[4], p[5]); QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[3]); QMF_SORT(p[5], p[8]); QMF_SORT(p[4], p[7]);
    QMF_SORT(p[3], p[6]); QMF_SORT(p[1], p[4]); QMF_SORT(p[2], p[5]);
    QMF_SORT(p[4], p[7]); QMF_SORT(p[4], p[2]); QMF_SORT(p[6], p[4]);
    QMF_SORT(p[4], p[2]);
    return p[4];
}

float quickMedianFilter3f(const float * v)
{
    float p[3];
    QMF_COPY(p, v, 3);

    QMF_SORTF(p[0], p[1]); QMF_SORTF(p[1], p[2]); QMF_SORTF(p[0], p[1]) ;
    return p[1];
}

float quickMedianFilter5f(const float * v)
{
    float p[5];
    QMF_COPY(p, v, 5);

    QMF_SORTF(p[0], p[1]); QMF_SORTF(p[3], p[4]); QMF_SORTF(p[0], p[3]);
    QMF_SORTF(p[1], p[4]); QMF_SORTF(p[1], p[2]); QMF_SORTF(p[2], p[3]);
    QMF_SORTF(p[1], p[2]);
    return p[2];
}

float quickMedianFilter7f(const float * v)
{
    float p[7];
    QMF_COPY(p, v, 7);

    QMF_SORTF(p[0], p[5]); QMF_SORTF(p[0], p[3]); QMF_SORTF(p[1], p[6]);
    QMF_SORTF(p[2], p[4]); QMF_SORTF(p[0], p[1]); QMF_SORTF(p[3], p[5]);
    QMF_SORTF(p[2], p[6]); QMF_SORTF(p[2], p[3]); QMF_SORTF(p[3], p[6]);
    QMF_SORTF(p[4], p[5]); QMF_SORTF(p[1], p[4]); QMF_SORTF(p[1], p[3]);
    QMF_SORTF(p[3], p[4]);
    return p[3];
}

float quickMedianFilter9f(const float * v)
{
    float p[9];
    QMF_COPY(p, v, 9);

    QMF_SORTF(p[1], p[2]); QMF_SORTF(p[4], p[5]); QMF_SORTF(p[7], p[8]);
    QMF_SORTF(p[0], p[1]); QMF_SORTF(p[3], p[4]); QMF_SORTF(p[6], p[7]);
    QMF_SORTF(p[1], p[2]); QMF_SORTF(p[4], p[5]); QMF_SORTF(p[7], p[8]);
    QMF_SORTF(p[0], p[3]); QMF_SORTF(p[5], p[8]); QMF_SORTF(p[4], p[7]);
    QMF_SORTF(p[3], p[6]); QMF_SORTF(p[1], p[4]); QMF_SORTF(p[2], p[5]);
    QMF_SORTF(p[4], p[7]); QMF_SORTF(p[4], p[2]); QMF_SORTF(p[6], p[4]);
    QMF_SORTF(p[4], p[2]);
    return p[4];
}
