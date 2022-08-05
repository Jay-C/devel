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

// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/version.h"
#include "build/build_config.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/pwm_output.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/pg.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "cli/settings.h"

//
// PID
//
static uint8_t tmpPidProfileIndex;
static uint8_t pidProfileIndex;
static char pidProfileIndexString[MAX_PROFILE_NAME_LENGTH + 5];
static uint8_t tempPid[3][3];
static uint16_t tempPidF[3];

static uint8_t tmpRateProfileIndex;
static uint8_t rateProfileIndex;
static char rateProfileIndexString[MAX_RATE_PROFILE_NAME_LENGTH + 5];
static controlRateConfig_t rateProfile;

#ifdef USE_MULTI_GYRO
static const char * const osdTableGyroToUse[] = {
    "FIRST", "SECOND", "BOTH"
};
#endif

static void setProfileIndexString(char *profileString, int profileIndex, char *profileName)
{
    int charIndex = 0;
    profileString[charIndex++] = '1' + profileIndex;

#ifdef USE_PROFILE_NAMES
    const int profileNameLen = strlen(profileName);

    if (profileNameLen > 0) {
        profileString[charIndex++] = ' ';
        profileString[charIndex++] = '(';
        for (int i = 0; i < profileNameLen; i++) {
            profileString[charIndex++] = toupper(profileName[i]);
        }
        profileString[charIndex++] = ')';
    }
#else
    UNUSED(profileName);
#endif

    profileString[charIndex] = '\0';
}

static const void *cmsx_menuImu_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    pidProfileIndex = getCurrentPidProfileIndex();
    tmpPidProfileIndex = pidProfileIndex + 1;

    rateProfileIndex = getCurrentControlRateProfileIndex();
    tmpRateProfileIndex = rateProfileIndex + 1;

    return NULL;
}

static const void *cmsx_menuImu_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    changePidProfile(pidProfileIndex);
    changeControlRateProfile(rateProfileIndex);

    return NULL;
}

static const void *cmsx_profileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    pidProfileIndex = tmpPidProfileIndex - 1;
    changePidProfile(pidProfileIndex);

    return NULL;
}

static const void *cmsx_rateProfileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    rateProfileIndex = tmpRateProfileIndex - 1;
    changeControlRateProfile(rateProfileIndex);

    return NULL;
}

static const void *cmsx_PidRead(void)
{

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    for (uint8_t i = 0; i < 3; i++) {
        tempPid[i][0] = pidProfile->pid[i].P;
        tempPid[i][1] = pidProfile->pid[i].I;
        tempPid[i][2] = pidProfile->pid[i].D;
        tempPidF[i] = pidProfile->pid[i].F;
    }

    return NULL;
}

static const void *cmsx_PidOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(pidProfileIndexString, pidProfileIndex, currentPidProfile->profileName);
    cmsx_PidRead();

    return NULL;
}

static const void *cmsx_PidWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;
    for (uint8_t i = 0; i < 3; i++) {
        pidProfile->pid[i].P = tempPid[i][0];
        pidProfile->pid[i].I = tempPid[i][1];
        pidProfile->pid[i].D = tempPid[i][2];
        pidProfile->pid[i].F = tempPidF[i];
    }
    pidInitProfile(currentPidProfile);

    return NULL;
}

static OSD_Entry cmsx_menuPidEntries[] =
{
    { "-- PID --", OME_Label, NULL, pidProfileIndexString},

    { "ROLL  P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][0],  0, 200, 1 }},
    { "ROLL  I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][1],  0, 200, 1 }},
    { "ROLL  D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][2],  0, 200, 1 }},
    { "ROLL  F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_ROLL],  0, 2000, 1 }},

    { "PITCH P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][0], 0, 200, 1 }},
    { "PITCH I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][1], 0, 200, 1 }},
    { "PITCH D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][2], 0, 200, 1 }},
    { "PITCH F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_PITCH], 0, 2000, 1 }},

    { "YAW   P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][0],   0, 200, 1 }},
    { "YAW   I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][1],   0, 200, 1 }},
    { "YAW   D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][2],   0, 200, 1 }},
    { "YAW   F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_YAW],   0, 2000, 1 }},

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuPid = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPID",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_PidOnEnter,
    .onExit = cmsx_PidWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuPidEntries
};

//
// Rate & Expo
//

static const void *cmsx_RateProfileRead(void)
{
    memcpy(&rateProfile, controlRateProfiles(rateProfileIndex), sizeof(controlRateConfig_t));

    return NULL;
}

static const void *cmsx_RateProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    memcpy(controlRateProfilesMutable(rateProfileIndex), &rateProfile, sizeof(controlRateConfig_t));

    return NULL;
}

static const void *cmsx_RateProfileOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(rateProfileIndexString, rateProfileIndex, controlRateProfilesMutable(rateProfileIndex)->profileName);
    cmsx_RateProfileRead();

    return NULL;
}

static const OSD_Entry cmsx_menuRateProfileEntries[] =
{
    { "-- RATE --", OME_Label, NULL, rateProfileIndexString },

    { "RC R RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_ROLL],    1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 } },
    { "RC P RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_PITCH],    1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 } },
    { "RC Y RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_YAW], 1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 } },

    { "ROLL SUPER",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_ROLL],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 } },
    { "PITCH SUPER", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_PITCH],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 } },
    { "YAW SUPER",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_YAW],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 } },

    { "RC R EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_ROLL],    0, 100, 1, 10 } },
    { "RC P EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_PITCH],    0, 100, 1, 10 } },
    { "RC Y EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_YAW], 0, 100, 1, 10 } },

    { "ROLL LVL EXPO",  OME_FLOAT, NULL, &(OSD_FLOAT_t) { &rateProfile.levelExpo[FD_ROLL],  0, 100, 1, 10 } },
    { "PITCH LVL EXPO", OME_FLOAT, NULL, &(OSD_FLOAT_t) { &rateProfile.levelExpo[FD_PITCH], 0, 100, 1, 10 } },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuRateProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENURATE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_RateProfileOnEnter,
    .onExit = cmsx_RateProfileWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuRateProfileEntries
};

static uint8_t  cmsx_angleStrength;
static uint8_t  cmsx_horizonStrength;
static uint8_t  cmsx_horizonTransition;

static const void *cmsx_profileOtherOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(pidProfileIndexString, pidProfileIndex, currentPidProfile->profileName);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_angleStrength =     pidProfile->angle.level_strength;
    cmsx_horizonStrength =   pidProfile->horizon.level_strength;
    cmsx_horizonTransition = pidProfile->horizon.transition;

    return NULL;
}

static const void *cmsx_profileOtherOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);
    pidInitProfile(currentPidProfile);

    pidProfile->angle.level_strength = cmsx_angleStrength;
    pidProfile->horizon.level_strength = cmsx_horizonStrength;
    pidProfile->horizon.transition = cmsx_horizonTransition;

    return NULL;
}

static const OSD_Entry cmsx_menuProfileOtherEntries[] = {
    { "-- OTHER PP --", OME_Label, NULL, pidProfileIndexString },
    { "ANGLE STR",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_angleStrength,          0,    200,   1  }    },
    { "HORZN STR",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonStrength,        0,    200,   1  }    },
    { "HORZN TRS",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonTransition,      0,    200,   1  }    },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuProfileOther = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPROFOTHER",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileOtherOnEnter,
    .onExit = cmsx_profileOtherOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuProfileOtherEntries,
};


static uint16_t gyroConfig_gyro_lpf1_static_hz;
static uint16_t gyroConfig_gyro_lpf2_static_hz;
static uint16_t gyroConfig_gyro_soft_notch_hz_1;
static uint16_t gyroConfig_gyro_soft_notch_cutoff_1;
static uint16_t gyroConfig_gyro_soft_notch_hz_2;
static uint16_t gyroConfig_gyro_soft_notch_cutoff_2;
static uint8_t  gyroConfig_gyro_to_use;

static const void *cmsx_menuGyro_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    gyroConfig_gyro_lpf1_static_hz =  gyroConfig()->gyro_lpf1_static_hz;
    gyroConfig_gyro_lpf2_static_hz =  gyroConfig()->gyro_lpf2_static_hz;
    gyroConfig_gyro_soft_notch_hz_1 = gyroConfig()->gyro_soft_notch_hz_1;
    gyroConfig_gyro_soft_notch_cutoff_1 = gyroConfig()->gyro_soft_notch_cutoff_1;
    gyroConfig_gyro_soft_notch_hz_2 = gyroConfig()->gyro_soft_notch_hz_2;
    gyroConfig_gyro_soft_notch_cutoff_2 = gyroConfig()->gyro_soft_notch_cutoff_2;
    gyroConfig_gyro_to_use = gyroConfig()->gyro_to_use;

    return NULL;
}

static const void *cmsx_menuGyro_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gyroConfigMutable()->gyro_lpf1_static_hz =  gyroConfig_gyro_lpf1_static_hz;
    gyroConfigMutable()->gyro_lpf2_static_hz =  gyroConfig_gyro_lpf2_static_hz;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = gyroConfig_gyro_soft_notch_hz_1;
    gyroConfigMutable()->gyro_soft_notch_cutoff_1 = gyroConfig_gyro_soft_notch_cutoff_1;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = gyroConfig_gyro_soft_notch_hz_2;
    gyroConfigMutable()->gyro_soft_notch_cutoff_2 = gyroConfig_gyro_soft_notch_cutoff_2;
    gyroConfigMutable()->gyro_to_use = gyroConfig_gyro_to_use;

    return NULL;
}

static const OSD_Entry cmsx_menuFilterGlobalEntries[] =
{
    { "-- FILTER GLB  --", OME_Label, NULL, NULL },

    { "GYRO LPF1",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lpf1_static_hz, 0, LPF_MAX_HZ, 1 } },
#ifdef USE_GYRO_LPF2
    { "GYRO LPF2",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lpf2_static_hz,  0, LPF_MAX_HZ, 1 } },
#endif
    { "GYRO NF1",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_hz_1,     0, 500, 1 } },
    { "GYRO NF1C",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_cutoff_1, 0, 500, 1 } },
    { "GYRO NF2",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_hz_2,     0, 500, 1 } },
    { "GYRO NF2C",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_cutoff_2, 0, 500, 1 } },
#ifdef USE_MULTI_GYRO
    { "GYRO TO USE",  OME_TAB | REBOOT_REQUIRED,  NULL, &(OSD_TAB_t)    { &gyroConfig_gyro_to_use,  2, osdTableGyroToUse} },
#endif

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuFilterGlobal = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTGLB",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuGyro_onEnter,
    .onExit = cmsx_menuGyro_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFilterGlobalEntries,
};

#if (defined(USE_DYN_NOTCH_FILTER) || defined(USE_DYN_LPF)) && defined(USE_EXTENDED_CMS_MENUS)

#ifdef USE_DYN_NOTCH_FILTER
static uint16_t dynFiltNotchMaxHz;
static uint8_t  dynFiltNotchCount;
static uint16_t dynFiltNotchQ;
static uint16_t dynFiltNotchMinHz;
#endif
#ifdef USE_DYN_LPF
static uint16_t gyroLpfDynMin;
static uint16_t gyroLpfDynMax;
static uint16_t dtermLpfDynMin;
static uint16_t dtermLpfDynMax;
#endif

static const void *cmsx_menuDynFilt_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

#ifdef USE_DYN_NOTCH_FILTER
    dynFiltNotchMaxHz   = dynNotchConfig()->dyn_notch_max_hz;
    dynFiltNotchCount   = dynNotchConfig()->dyn_notch_count;
    dynFiltNotchQ       = dynNotchConfig()->dyn_notch_q;
    dynFiltNotchMinHz   = dynNotchConfig()->dyn_notch_min_hz;
#endif
#ifdef USE_DYN_LPF
    gyroLpfDynMin       = gyroConfig()->gyro_lpf1_dyn_min_hz;
    gyroLpfDynMax       = gyroConfig()->gyro_lpf1_dyn_max_hz;
    dtermLpfDynMin      = gyroConfig()->dterm_lpf1_dyn_min_hz;
    dtermLpfDynMax      = gyroConfig()->dterm_lpf1_dyn_max_hz;
#endif

    return NULL;
}

static const void *cmsx_menuDynFilt_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

#ifdef USE_DYN_NOTCH_FILTER
    dynNotchConfigMutable()->dyn_notch_max_hz        = dynFiltNotchMaxHz;
    dynNotchConfigMutable()->dyn_notch_count         = dynFiltNotchCount;
    dynNotchConfigMutable()->dyn_notch_q             = dynFiltNotchQ;
    dynNotchConfigMutable()->dyn_notch_min_hz        = dynFiltNotchMinHz;
#endif
#ifdef USE_DYN_LPF
    gyroConfigMutable()->gyro_lpf1_dyn_min_hz        = gyroLpfDynMin;
    gyroConfigMutable()->gyro_lpf1_dyn_max_hz        = gyroLpfDynMax;
    gyroConfigMutable()->dterm_lpf1_dyn_min_hz       = dtermLpfDynMin;
    gyroConfigMutable()->dterm_lpf1_dyn_max_hz       = dtermLpfDynMax;
#endif

    return NULL;
}

static const OSD_Entry cmsx_menuDynFiltEntries[] =
{
    { "-- DYN FILT --", OME_Label, NULL, NULL },

#ifdef USE_DYN_NOTCH_FILTER
    { "NOTCH COUNT",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &dynFiltNotchCount,   0, DYN_NOTCH_COUNT_MAX, 1 } },
    { "NOTCH Q",        OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchQ,       1, 1000, 1 } },
    { "NOTCH MIN HZ",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchMinHz,   60, 250, 1 } },
    { "NOTCH MAX HZ",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchMaxHz,   200, 1000, 1 } },
#endif

#ifdef USE_DYN_LPF
    { "GYRO DLPF MIN",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroLpfDynMin,  0, 1000, 1 } },
    { "GYRO DLPF MAX",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroLpfDynMax,  0, 1000, 1 } },
    { "DTERM DLPF MIN",  OME_UINT16, NULL, &(OSD_UINT16_t) { &dtermLpfDynMin, 0, 1000, 1 } },
    { "DTERM DLPF MAX",  OME_UINT16, NULL, &(OSD_UINT16_t) { &dtermLpfDynMax, 0, 1000, 1 } },
#endif

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuDynFilt = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XDYNFLT",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuDynFilt_onEnter,
    .onExit = cmsx_menuDynFilt_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuDynFiltEntries,
};

#endif

static uint16_t cmsx_dterm_lpf1_static_hz;
static uint16_t cmsx_dterm_lpf2_static_hz;
static uint16_t cmsx_dterm_notch_hz;
static uint16_t cmsx_dterm_notch_cutoff;

static const void *cmsx_FilterPerProfileRead(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_dterm_lpf1_static_hz   = gyroConfig()->dterm_lpf1_static_hz;
    cmsx_dterm_lpf2_static_hz   = gyroConfig()->dterm_lpf2_static_hz;
    cmsx_dterm_notch_hz         = gyroConfig()->dterm_notch_hz;
    cmsx_dterm_notch_cutoff     = gyroConfig()->dterm_notch_cutoff;

    return NULL;
}

static const void *cmsx_FilterPerProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gyroConfigMutable()->dterm_lpf1_static_hz  = cmsx_dterm_lpf1_static_hz;
    gyroConfigMutable()->dterm_lpf2_static_hz  = cmsx_dterm_lpf2_static_hz;
    gyroConfigMutable()->dterm_notch_hz        = cmsx_dterm_notch_hz;
    gyroConfigMutable()->dterm_notch_cutoff    = cmsx_dterm_notch_cutoff;

    return NULL;
}

static const OSD_Entry cmsx_menuFilterPerProfileEntries[] =
{
    { "-- FILTER PP  --", OME_Label, NULL, NULL },

    { "DTERM LPF1", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lpf1_static_hz, 0, LPF_MAX_HZ, 1 } },
    { "DTERM LPF2", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lpf2_static_hz, 0, LPF_MAX_HZ, 1 } },
    { "DTERM NF",   OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_notch_hz,       0, LPF_MAX_HZ, 1 } },
    { "DTERM NFCO", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_notch_cutoff,   0, LPF_MAX_HZ, 1 } },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuFilterPerProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTPP",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_FilterPerProfileRead,
    .onExit = cmsx_FilterPerProfileWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFilterPerProfileEntries,
};

#ifdef USE_EXTENDED_CMS_MENUS

static uint8_t cmsx_dstPidProfile;
static uint8_t cmsx_dstControlRateProfile;

static const char * const cmsx_ProfileNames[] = {
    "-",
    "1",
    "2",
    "3"
};

static OSD_TAB_t cmsx_PidProfileTable = { &cmsx_dstPidProfile, 3, cmsx_ProfileNames };
static OSD_TAB_t cmsx_ControlRateProfileTable = { &cmsx_dstControlRateProfile, 3, cmsx_ProfileNames };

static const void *cmsx_menuCopyProfile_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_dstPidProfile = 0;
    cmsx_dstControlRateProfile = 0;

    return NULL;
}

static const void *cmsx_CopyPidProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstPidProfile > 0) {
        pidCopyProfile(cmsx_dstPidProfile - 1, getCurrentPidProfileIndex());
    }

    return NULL;
}

static const void *cmsx_CopyControlRateProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstControlRateProfile > 0) {
        copyControlRateProfile(cmsx_dstControlRateProfile - 1, getCurrentControlRateProfileIndex());
    }

    return NULL;
}

static const OSD_Entry cmsx_menuCopyProfileEntries[] =
{
    { "-- COPY PROFILE --", OME_Label, NULL, NULL},

    { "CPY PID PROF TO",   OME_TAB,      NULL,                        &cmsx_PidProfileTable },
    { "COPY PP",           OME_Funcall,  cmsx_CopyPidProfile,         NULL },
    { "CPY RATE PROF TO",  OME_TAB,      NULL,                        &cmsx_ControlRateProfileTable },
    { "COPY RP",           OME_Funcall,  cmsx_CopyControlRateProfile, NULL },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuCopyProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XCPY",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuCopyProfile_onEnter,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuCopyProfileEntries,
};

#endif

static const OSD_Entry cmsx_menuImuEntries[] =
{
    { "-- PROFILE --", OME_Label, NULL, NULL},

    {"PID PROF",  OME_UINT8,   cmsx_profileIndexOnChange,     &(OSD_UINT8_t){ &tmpPidProfileIndex, 1, PID_PROFILE_COUNT, 1}},
    {"PID",       OME_Submenu, cmsMenuChange,                 &cmsx_menuPid},
    {"MISC PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileOther},
    {"FILT PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterPerProfile},

    {"RATE PROF", OME_UINT8,   cmsx_rateProfileIndexOnChange, &(OSD_UINT8_t){ &tmpRateProfileIndex, 1, CONTROL_RATE_PROFILE_COUNT, 1}},
    {"RATE",      OME_Submenu, cmsMenuChange,                 &cmsx_menuRateProfile},

    {"FILT GLB",  OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal},
#if  (defined(USE_DYN_NOTCH_FILTER) || defined(USE_DYN_LPF)) && defined(USE_EXTENDED_CMS_MENUS)
    {"DYN FILT",  OME_Submenu, cmsMenuChange,                 &cmsx_menuDynFilt},
#endif

#ifdef USE_EXTENDED_CMS_MENUS
    {"COPY PROF", OME_Submenu, cmsMenuChange,                 &cmsx_menuCopyProfile},
#endif /* USE_EXTENDED_CMS_MENUS */

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuImu = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XIMU",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuImu_onEnter,
    .onExit = cmsx_menuImu_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuImuEntries,
};

#endif // CMS
