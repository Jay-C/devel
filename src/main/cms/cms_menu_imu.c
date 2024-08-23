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
#include "flight/pid_init.h"

#include "pg/pg.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "cli/settings.h"

//
// PID
//

#define PROFILE_INDEX_STRING_ADDITIONAL_SIZE 5 // Additional symbols for setProfileIndexString(): "2 (NAMENAME)\0"

static uint8_t tmpPidProfileIndex;
static uint8_t pidProfileIndex;
static char pidProfileIndexString[MAX_PROFILE_NAME_LENGTH + PROFILE_INDEX_STRING_ADDITIONAL_SIZE];
static uint8_t tempPid[3][3];
static uint16_t tempPidF[3];

static uint8_t tmpRateProfileIndex;
static uint8_t rateProfileIndex;
static char rateProfileIndexString[MAX_RATE_PROFILE_NAME_LENGTH + PROFILE_INDEX_STRING_ADDITIONAL_SIZE];
static controlRateConfig_t rateProfile;

#ifdef USE_MULTI_GYRO
static const char * const osdTableGyroToUse[] = {
    "FIRST", "SECOND", "BOTH"
};
#endif

static void setProfileIndexString(char *profileString, int profileIndex, const char *profileName)
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

static char pidProfileNames[PID_PROFILE_COUNT][MAX_PROFILE_NAME_LENGTH + PROFILE_INDEX_STRING_ADDITIONAL_SIZE];
static const char *pidProfileNamePtrs[PID_PROFILE_COUNT];

static char rateProfileNames[CONTROL_RATE_PROFILE_COUNT][MAX_PROFILE_NAME_LENGTH + PROFILE_INDEX_STRING_ADDITIONAL_SIZE];
static const char *rateProfileNamePtrs[CONTROL_RATE_PROFILE_COUNT];

static const void *cmsx_menuImu_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        setProfileIndexString(pidProfileNames[i], i, pidProfiles(i)->profileName);
        pidProfileNamePtrs[i] = pidProfileNames[i];
    }

    pidProfileIndex = getCurrentPidProfileIndex();
    tmpPidProfileIndex = pidProfileIndex;

    for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        setProfileIndexString(rateProfileNames[i], i, controlRateProfilesMutable(i)->profileName);
        rateProfileNamePtrs[i] = rateProfileNames[i];
    }

    rateProfileIndex = getCurrentControlRateProfileIndex();
    tmpRateProfileIndex = rateProfileIndex;

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

    pidProfileIndex = tmpPidProfileIndex;
    changePidProfile(pidProfileIndex);

    return NULL;
}

static const void *cmsx_rateProfileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    rateProfileIndex = tmpRateProfileIndex;
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
    pidInitConfig(currentPidProfile);

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

static uint8_t  cmsx_angleP;
static uint8_t  cmsx_angleFF;
static uint8_t  cmsx_angleLimit;
static uint8_t  cmsx_angleEarthRef;

static uint8_t  cmsx_horizonStrength;
static uint8_t  cmsx_horizonLimitSticks;
static uint8_t  cmsx_horizonLimitDegrees;

static const void *cmsx_profileOtherOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(pidProfileIndexString, pidProfileIndex, currentPidProfile->profileName);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_angleP =             pidProfile->pid[PID_LEVEL].P;
    cmsx_angleFF =            pidProfile->pid[PID_LEVEL].F;
    cmsx_angleLimit =         pidProfile->angle_limit;
    cmsx_angleEarthRef =      pidProfile->angle_earth_ref;

    cmsx_horizonStrength =    pidProfile->pid[PID_LEVEL].I;
    cmsx_horizonLimitSticks = pidProfile->pid[PID_LEVEL].D;
    cmsx_horizonLimitDegrees = pidProfile->horizon_limit_degrees;

    return NULL;
}

static const void *cmsx_profileOtherOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);
    pidInitConfig(currentPidProfile);

    pidProfile->pid[PID_LEVEL].P = cmsx_angleP;
    pidProfile->pid[PID_LEVEL].F = cmsx_angleFF;
    pidProfile->angle_limit = cmsx_angleLimit;
    pidProfile->angle_earth_ref = cmsx_angleEarthRef;

    pidProfile->pid[PID_LEVEL].I = cmsx_horizonStrength;
    pidProfile->pid[PID_LEVEL].D = cmsx_horizonLimitSticks;
    pidProfile->horizon_limit_degrees = cmsx_horizonLimitDegrees;

    initEscEndpoints();
    return NULL;
}

static const OSD_Entry cmsx_menuProfileOtherEntries[] = {
    { "-- OTHER PP --", OME_Label, NULL, pidProfileIndexString },

    { "ANGLE P",         OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_angleP,                     0,    200,   1  }    },
    { "ANGLE FF",        OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_angleFF,                    0,    200,   1  }    },
    { "ANGLE LIMIT",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_angleLimit,                10,     90,   1  }    },
    { "ANGLE E_REF",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_angleEarthRef,              0,    100,   1  }    },

    { "HORZN STR",       OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonStrength,            0,    100,   1  }    },
    { "HORZN LIM_STK",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonLimitSticks,        10,    200,   1  }    },
    { "HORZN LIM_DEG",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonLimitDegrees,       10,    250,   1  }    },

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

#if defined(USE_EXTENDED_CMS_MENUS)

static const void *cmsx_menuDynFilt_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    return NULL;
}

static const void *cmsx_menuDynFilt_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    return NULL;
}

static const OSD_Entry cmsx_menuDynFiltEntries[] =
{
    { "-- DYN FILT --", OME_Label, NULL, NULL },

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

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_dterm_lpf1_static_hz   = pidProfile->dterm_lpf1_static_hz;
    cmsx_dterm_lpf2_static_hz   = pidProfile->dterm_lpf2_static_hz;
    cmsx_dterm_notch_hz         = pidProfile->dterm_notch_hz;
    cmsx_dterm_notch_cutoff     = pidProfile->dterm_notch_cutoff;

    return NULL;
}

static const void *cmsx_FilterPerProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;

    pidProfile->dterm_lpf1_static_hz  = cmsx_dterm_lpf1_static_hz;
    pidProfile->dterm_lpf2_static_hz  = cmsx_dterm_lpf2_static_hz;
    pidProfile->dterm_notch_hz        = cmsx_dterm_notch_hz;
    pidProfile->dterm_notch_cutoff    = cmsx_dterm_notch_cutoff;

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

    {"PID PROF",  OME_TAB,     cmsx_profileIndexOnChange,     &(OSD_TAB_t){&tmpPidProfileIndex, PID_PROFILE_COUNT-1, pidProfileNamePtrs}},
    {"PID",       OME_Submenu, cmsMenuChange,                 &cmsx_menuPid},
    {"MISC PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileOther},
    {"FILT PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterPerProfile},

    {"RATE PROF", OME_TAB,   cmsx_rateProfileIndexOnChange, &(OSD_TAB_t){&tmpRateProfileIndex, CONTROL_RATE_PROFILE_COUNT-1, rateProfileNamePtrs}},
    {"RATE",      OME_Submenu, cmsMenuChange,                 &cmsx_menuRateProfile},

    {"FILT GLB",  OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal},
#if defined(USE_EXTENDED_CMS_MENUS)
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
