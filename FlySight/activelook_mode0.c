/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2025 Bionic Avionics Inc.                                   **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/

/*
 * Configurable data display (al_mode = 1) — Phase state machine
 *
 * ┌──────────────────────────────────────┐
 * │ IDLE ──(gpsFix == 3)──► ACTIVE       │
 * │  ▲                         │         │
 * │  └───(gpsFix != 3)─────────┘         │
 * └──────────────────────────────────────┘
 *
 * IDLE:   Status bar + "Acquiring GPS..." message.
 * ACTIVE: Status bar + up to 4 configurable data lines.
 *
 * All rendering uses activelook_draw.c (AL_Draw_TextField).
 * All value formatting uses %d / %ld only — newlib-nano has no %f support.
 * Computation style mirrors activelook_mode1.c throughout.
 */

#include "activelook_mode0.h"
#include "activelook_draw.h"
#include "activelook_client.h"
#include "config.h"
#include "gnss.h"
#include "nav.h"
#include "vbat.h"
#include "app_common.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* --------------------------------------------------------------------------
   Phase
   -------------------------------------------------------------------------- */

typedef enum {
    MODE0_PHASE_IDLE   = 0,
    MODE0_PHASE_ACTIVE = 1,
} Mode0Phase_t;

/* --------------------------------------------------------------------------
   Module state
   -------------------------------------------------------------------------- */

static Mode0Phase_t   s_phase        = MODE0_PHASE_IDLE;
static Mode0Phase_t   s_render_phase = MODE0_PHASE_IDLE;
static bool           s_first_update = false;
static uint8_t        s_data_font    = 2;

/*
 * Logical layout (Y, physical viewer perspective, 0 = bottom):
 *   y=240  status bar
 *   y=190  line 0
 *   y=145  line 1
 *   y=100  line 2
 *   y= 55  line 3
 */
static AL_TextField_t s_tf_line[4] = {
    AL_TEXTFIELD_INIT(AL_TX(30), 190),
    AL_TEXTFIELD_INIT(AL_TX(30), 145),
    AL_TEXTFIELD_INIT(AL_TX(30), 100),
    AL_TEXTFIELD_INIT(AL_TX(30),  55),
};

/* --------------------------------------------------------------------------
   Helpers
   -------------------------------------------------------------------------- */

static void OnPhaseEnter(void)
{
    AL_Draw_ClearScreen();
}

/* --------------------------------------------------------------------------
   Per-phase rendering
   -------------------------------------------------------------------------- */

static void DrawStatusBar(const FS_GNSS_Data_t *gnss, const FS_VBAT_Data_t *vbat,
                           uint8_t al_batt)
{
    enum { X = 25, Y = 240, FONT = 1 };
    static AL_TextField_t tf = AL_TEXTFIELD_INIT(AL_TX(X), Y);

    char al_str[5];
    if (al_batt == 255) snprintf(al_str, sizeof(al_str), "??");
    else                snprintf(al_str, sizeof(al_str), "%d", al_batt);

    int fs_pct = (100 * ((int)vbat->voltage - 3300)) / (4200 - 3300);
    if (fs_pct < 0)   fs_pct = 0;
    if (fs_pct > 100) fs_pct = 100;

    char raw[AL_TEXTFIELD_MAXLEN];
    snprintf(raw, sizeof(raw), "A:%s%%  F:%d%%  N:%d", al_str, fs_pct, gnss->numSV);
    AL_Draw_TextField(&tf, FONT, raw);
}

static void RenderIdle(void)
{
    static AL_TextField_t tf_wait = AL_TEXTFIELD_INIT(AL_TX(40), 120);
    AL_Draw_TextField(&tf_wait, 2, "Acquiring GPS...");
    for (int i = 0; i < 4; i++)
        AL_Draw_TextField(&s_tf_line[i], 1, "");
}

/*
 * Render one configured data line.  Computation style matches mode1:
 *   - gSpeed (cm/s) * 0.036  → km/h
 *   - gSpeed (cm/s) * 0.022369 → mph
 *   - velD   (mm/s) / 100    → tenths of m/s   (1-decimal display)
 *   - hMSL   (mm)   / 1000   → m AGL
 *   - GR = gSpeed * 10 / velD  (same formula as mode1 RenderCompetitionRun)
 * No %f format specifier — newlib-nano strips float printf support.
 */
static void RenderLine(int i, const FS_GNSS_Data_t *gnss, const FS_Config_Data_t *cfg)
{
    bool    has_fix = (gnss->gpsFix == 3);
    bool    metric  = (cfg->al_lines[i].units == FS_UNIT_SYSTEM_METRIC);
    int32_t agl_m   = (gnss->hMSL - cfg->dz_elev) / 1000;
    char    raw[AL_TEXTFIELD_MAXLEN];

    switch (cfg->al_lines[i].mode)
    {
    /* ---- Horizontal speed (cm/s → km/h or mph) ------------------------- */
    case FS_CONFIG_MODE_HORIZONTAL_SPEED:
        if (has_fix) {
            int v = metric ? (int)((float)gnss->gSpeed * 0.036f)
                           : (int)((float)gnss->gSpeed * 0.022369f);
            snprintf(raw, sizeof(raw), "HSpd: %d %s", v, metric ? "km/h" : "mph");
        } else {
            snprintf(raw, sizeof(raw), "HSpd: --");
        }
        break;

    /* ---- Vertical speed (mm/s → m/s, 1 decimal, signed) --------------- */
    case FS_CONFIG_MODE_VERTICAL_SPEED: {
        if (has_fix) {
            int32_t v10 = gnss->velD / 100;   /* tenths of m/s, signed */
            int neg = (v10 < 0);
            int32_t av = neg ? -v10 : v10;
            if (metric) {
                snprintf(raw, sizeof(raw), "VSpd: %s%d.%d m/s",
                         neg ? "-" : "", (int)(av / 10), (int)(av % 10));
            } else {
                int mph10 = (int)((float)av / 10.0f * 2.23694f * 10.0f + 0.5f);
                snprintf(raw, sizeof(raw), "VSpd: %s%d.%d mph",
                         neg ? "-" : "", mph10 / 10, mph10 % 10);
            }
        } else {
            snprintf(raw, sizeof(raw), "VSpd: --");
        }
        break;
    }

    /* ---- Glide ratio (1 decimal) — identical to mode1 formula ---------- */
    case FS_CONFIG_MODE_GLIDE_RATIO:
        if (has_fix && gnss->velD > 0) {
            float gr  = (float)gnss->gSpeed * 10.0f / (float)gnss->velD;
            int   gr10 = (int)(gr * 10.0f + 0.5f);
            snprintf(raw, sizeof(raw), "GR: %d.%d", gr10 / 10, gr10 % 10);
        } else {
            snprintf(raw, sizeof(raw), "GR: --");
        }
        break;

    /* ---- Inverse glide ratio (1 decimal) ------------------------------- */
    case FS_CONFIG_MODE_INVERSE_GLIDE_RATIO:
        if (has_fix && gnss->gSpeed > 0) {
            float igr  = (float)gnss->velD / ((float)gnss->gSpeed * 10.0f);
            int   igr10 = (int)(igr * 10.0f + 0.5f);
            snprintf(raw, sizeof(raw), "1/GR: %d.%d", igr10 / 10, igr10 % 10);
        } else {
            snprintf(raw, sizeof(raw), "1/GR: --");
        }
        break;

    /* ---- 3-D total speed (cm/s → km/h or mph) -------------------------- */
    case FS_CONFIG_MODE_TOTAL_SPEED:
        if (has_fix) {
            int v = metric ? (int)((float)gnss->speed * 0.036f)
                           : (int)((float)gnss->speed * 0.022369f);
            snprintf(raw, sizeof(raw), "Spd: %d %s", v, metric ? "km/h" : "mph");
        } else {
            snprintf(raw, sizeof(raw), "Spd: --");
        }
        break;

    /* ---- Altitude AGL (mm → m or ft) ----------------------------------- */
    case FS_CONFIG_MODE_ALTITUDE:
        if (has_fix) {
            if (metric) {
                snprintf(raw, sizeof(raw), "Alt: %ldm", (long)agl_m);
            } else {
                snprintf(raw, sizeof(raw), "Alt: %ldft",
                         (long)((float)agl_m * 3.28084f));
            }
        } else {
            snprintf(raw, sizeof(raw), "Alt: --");
        }
        break;

    /* ---- Dive angle (degrees, 1 decimal, signed) ----------------------- */
    case FS_CONFIG_MODE_DIVE_ANGLE:
        if (has_fix && gnss->gSpeed > 0) {
            float angle = atan2f((float)gnss->velD  / 1000.0f,
                                 (float)gnss->gSpeed / 100.0f) * (180.0f / (float)M_PI);
            int neg = (angle < 0.0f);
            int a10 = (int)(fabsf(angle) * 10.0f + 0.5f);
            snprintf(raw, sizeof(raw), "Dive: %s%d.%ddeg",
                     neg ? "-" : "", a10 / 10, a10 % 10);
        } else {
            snprintf(raw, sizeof(raw), "Dive: --");
        }
        break;

    /* ---- Direction to destination (degrees) ---------------------------- */
    case FS_CONFIG_MODE_DIRECTION_TO_DESTINATION:
        if (has_fix && cfg->enable_nav) {
            int dir = calcDirection(gnss->lat, gnss->lon,
                                    cfg->lat, cfg->lon, gnss->heading);
            snprintf(raw, sizeof(raw), "Dir: %ddeg", dir);
        } else {
            snprintf(raw, sizeof(raw), "Dir: --");
        }
        break;

    /* ---- Distance to destination (m or ft) ----------------------------- */
    case FS_CONFIG_MODE_DISTANCE_TO_DESTINATION:
        if (has_fix && cfg->enable_nav) {
            int32_t dist_m = (int32_t)calcDistance(gnss->lat, gnss->lon,
                                                    cfg->lat, cfg->lon);
            if (metric) snprintf(raw, sizeof(raw), "Dist: %ldm",  (long)dist_m);
            else        snprintf(raw, sizeof(raw), "Dist: %ldft",
                                 (long)((float)dist_m * 3.28084f));
        } else {
            snprintf(raw, sizeof(raw), "Dist: --");
        }
        break;

    /* ---- Relative bearing (degrees) ------------------------------------ */
    case FS_CONFIG_MODE_DIRECTION_TO_BEARING:
        if (has_fix && cfg->enable_nav) {
            int brg = calcRelBearing(cfg->bearing, gnss->heading / 100000);
            snprintf(raw, sizeof(raw), "Brg: %ddeg", brg);
        } else {
            snprintf(raw, sizeof(raw), "Brg: --");
        }
        break;

    /* ---- Heading (degrees) --------------------------------------------- */
    case 13:
        if (has_fix) {
            snprintf(raw, sizeof(raw), "Hdg: %lddeg",
                     (long)(gnss->heading / 100000));
        } else {
            snprintf(raw, sizeof(raw), "Hdg: --");
        }
        break;

    default:
        snprintf(raw, sizeof(raw), "?: --");
        break;
    }

    AL_Draw_TextField(&s_tf_line[i], s_data_font, raw);
}

static void RenderActive(const FS_GNSS_Data_t *gnss, const FS_Config_Data_t *cfg)
{
    for (int i = 0; i < 4; i++) {
        if (i < cfg->num_al_lines)
            RenderLine(i, gnss, cfg);
        else
            AL_Draw_TextField(&s_tf_line[i], s_data_font, "");
    }
}

/* --------------------------------------------------------------------------
   Public API
   -------------------------------------------------------------------------- */

void FS_ActiveLook_Mode0_Init(void)
{
    const FS_Config_Data_t *cfg = FS_Config_Get();
    s_phase        = MODE0_PHASE_IDLE;
    s_render_phase = MODE0_PHASE_IDLE;
    s_first_update = true;
    s_data_font    = (cfg->num_al_lines <= 2) ? 2 : 2;
}

FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode0_Setup(void)
{
    return FS_AL_SETUP_DONE;
}

void FS_ActiveLook_Mode0_Update(void)
{
    if (s_first_update) {
        OnPhaseEnter();
        s_first_update = false;
    }

    const FS_GNSS_Data_t   *gnss   = FS_GNSS_GetData();
    const FS_Config_Data_t *cfg    = FS_Config_Get();
    const FS_VBAT_Data_t   *vbat   = FS_VBAT_GetData();
    uint8_t                 al_bat = FS_ActiveLook_Client_GetBatteryLevel();

    Mode0Phase_t new_phase = (gnss->gpsFix == 3) ? MODE0_PHASE_ACTIVE : MODE0_PHASE_IDLE;
    if (new_phase != s_phase)
        s_phase = new_phase;

    if (s_phase != s_render_phase) {
        OnPhaseEnter();
        s_render_phase = s_phase;
    }

    DrawStatusBar(gnss, vbat, al_bat);

    switch (s_phase) {
    case MODE0_PHASE_IDLE:
        RenderIdle();
        break;
    case MODE0_PHASE_ACTIVE:
        RenderActive(gnss, cfg);
        break;
    }
}
