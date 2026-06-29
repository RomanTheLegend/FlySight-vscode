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
 * Competition Mode (al_mode = 2) — ActiveLook graphics renderer
 *
 * Flight logic lives in mode1_logic.c (shared with Vuzix).  This file owns
 * only the AL-specific arrow and text-field state plus the font upload.
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ IDLE  ──(AGL > 100m)──►  AIRPLANE  ──(freefall)──►  FREEFALL       │
 * │  ▲                           │                          │           │
 * │  │                   (AGL < 50m)                   (T + 10 s)      │
 * │  │                           ▼                          ▼           │
 * │  │                         IDLE          COMPETITION_RUN            │
 * │  │                                          │                       │
 * │  │                              (AGL<1500m AND velD<10m/s)          │
 * │  │                                          ▼                       │
 * │  │                                      RUN_SCORE                   │
 * │  │                                          │                       │
 * │  │                                      (T + 15 s)                  │
 * │  │                                          ▼                       │
 * │  └─────────(avg velD 10s < 3 m/s)──── HOME_NAV                     │
 * └─────────────────────────────────────────────────────────────────────┘
 */

#include "activelook_mode1.h"
#include "activelook_draw.h"
#include "activelook_client.h"
#include "mode1_logic.h"
#include "vbat.h"
#include "app_common.h"
#include "inconsolata_bold_90.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
   Custom font constants
   -------------------------------------------------------------------------- */
#define AL_FONT_INCONSOLATA_BOLD_90_ID   10u
#define AL_FONT_CHUNK_SIZE              242u

/* --------------------------------------------------------------------------
   Lane deviation display thresholds (metres cross-track)
   -------------------------------------------------------------------------- */
#define LANE_MINOR_M    150.0f
#define LANE_MODERATE_M 200.0f
#define LANE_MAJOR_M    250.0f

/* --------------------------------------------------------------------------
   AL-specific module state
   -------------------------------------------------------------------------- */

static AL_ArrowState_t  s_target_arrow = AL_ARROW_STATE_INIT;
static AL_ArrowState_t  s_dz_arrow     = AL_ARROW_STATE_INIT;
static uint8_t          s_font_setup_step = 0;
static bool             s_first_update    = false;

/* --------------------------------------------------------------------------
   Local helpers
   -------------------------------------------------------------------------- */

static const char *LaneArrow(float dev_m)
{
    if      (dev_m >  LANE_MAJOR_M)    return "<<<";
    else if (dev_m >  LANE_MODERATE_M) return "  <<";
    else if (dev_m >  LANE_MINOR_M)    return "    <";
    else if (dev_m < -LANE_MAJOR_M)    return "     >>>";
    else if (dev_m < -LANE_MODERATE_M) return "     >>";
    else if (dev_m < -LANE_MINOR_M)    return "     >";
    else                               return "    +";
}

static void DrawStatusBar(const Mode1_Data_t *d, const FS_VBAT_Data_t *vbat, uint8_t al_batt)
{
    enum { X = 25, Y = 240, FONT = 1 };
    static AL_TextField_t tf = AL_TEXTFIELD_INIT(AL_TX(X), Y);

    char al_str[5];
    if (al_batt == 255) snprintf(al_str, sizeof(al_str), "??");
    else                snprintf(al_str, sizeof(al_str), "%d", al_batt);

    int fs_pct = (100 * ((int)vbat->voltage - 3300)) / (4200 - 3200);
    if (fs_pct < 0)   fs_pct = 0;
    if (fs_pct > 100) fs_pct = 100;

    char raw[AL_TEXTFIELD_MAXLEN];
    snprintf(raw, sizeof(raw), "A:%s%%  F:%d%%  N:%d", al_str, fs_pct, d->num_sv);
    AL_Draw_TextField(&tf, FONT, raw);
}

static void OnPhaseEnter(void)
{
    AL_Draw_ClearScreen();
    s_target_arrow.valid = false;
    s_dz_arrow.valid     = false;
}

/* --------------------------------------------------------------------------
   Per-phase rendering
   -------------------------------------------------------------------------- */

static void RenderIdle(const Mode1_Data_t *d)
{
    enum {
        ALT_X = 25,  ALT_Y  = 130, ALT_FONT  = 3,
        HMSL_X = 25, HMSL_Y =  83, HMSL_FONT = 1,
        DZ_X  = 25,  DZ_Y   =  60, DZ_FONT   = 1,
        LBL_X = 55,  LBL_Y  = 155, LBL_FONT  = 1,
        ARROW_LX = 100, ARROW_LY = 70,
    };
    static AL_TextField_t tf_alt   = AL_TEXTFIELD_INIT(AL_TX(ALT_X),  ALT_Y);
    static AL_TextField_t tf_hmsl  = AL_TEXTFIELD_INIT(AL_TX(HMSL_X), HMSL_Y);
    static AL_TextField_t tf_dz    = AL_TEXTFIELD_INIT(AL_TX(DZ_X),   DZ_Y);
    static AL_TextField_t tf_label = AL_TEXTFIELD_INIT(AL_TX(LBL_X),  LBL_Y);

    char raw[16];
    if (d->has_gps) snprintf(raw, sizeof(raw), "Alt: %ldm", (long)d->alt_agl_m);
    else            snprintf(raw, sizeof(raw), "Alt: --");
    AL_Draw_TextField(&tf_alt, ALT_FONT, raw);

    if (d->has_gps) snprintf(raw, sizeof(raw), "hMSL: %ldm", (long)d->hMSL_m);
    else            snprintf(raw, sizeof(raw), "hMSL: --");
    AL_Draw_TextField(&tf_hmsl, HMSL_FONT, raw);

    snprintf(raw, sizeof(raw), "DZ e: %ldm", (long)d->dz_elev_m);
    AL_Draw_TextField(&tf_dz, DZ_FONT, raw);

    AL_Arrow_Erase(&s_target_arrow);
    if (d->has_target && d->has_gps) {
        AL_Arrow_Draw(&s_target_arrow, ARROW_LX, ARROW_LY, d->rel_to_target_deg);
        AL_Draw_TextField(&tf_label, LBL_FONT, "Run target");
    } else {
        AL_Draw_TextField(&tf_label, LBL_FONT, "");
    }
}

static void RenderAirplane(const Mode1_Data_t *d)
{
    enum {
        ALT_X = 25,  ALT_Y = 95, ALT_FONT = 2,
        HS_X  = 25,  HS_Y  = 60, HS_FONT  = 3,
        LBL_X = 55, LBL_Y = 130, LBL_FONT = 1,
        ARROW_LX = 100, ARROW_LY = 90,
    };
    static AL_TextField_t tf_alt   = AL_TEXTFIELD_INIT(AL_TX(ALT_X), ALT_Y);
    static AL_TextField_t tf_hs    = AL_TEXTFIELD_INIT(AL_TX(HS_X),  HS_Y);
    static AL_TextField_t tf_label = AL_TEXTFIELD_INIT(AL_TX(LBL_X), LBL_Y);

    char raw[16];
    if (d->has_gps) snprintf(raw, sizeof(raw), "Alt: %ldm", (long)d->alt_agl_m);
    else            snprintf(raw, sizeof(raw), "--");
    AL_Draw_TextField(&tf_alt, ALT_FONT, raw);

    if (d->has_gps) snprintf(raw, sizeof(raw), "Vh: %ld", (long)d->gspeed_kmh);
    else            snprintf(raw, sizeof(raw), "--");
    AL_Draw_TextField(&tf_hs, HS_FONT, raw);

    AL_Arrow_Erase(&s_target_arrow);
    if (d->has_target && d->has_gps) {
        AL_CompassArrow_Draw(&s_target_arrow, ARROW_LX, ARROW_LY, d->rel_to_target_deg);
        AL_Draw_TextField(&tf_label, LBL_FONT, "Run target");
    } else {
        AL_Draw_TextField(&tf_label, LBL_FONT, "");
    }
}

static void RenderFreefall(const Mode1_Data_t *d)
{
    enum {
        LBL_X = 25,  LBL_Y = 130, LBL_FONT = 2,
        CNT_X = 90,  CNT_Y =  90, CNT_FONT = 3,
        ARROW_LX = 80, ARROW_LY = 95,
    };
    static AL_TextField_t tf_label = AL_TEXTFIELD_INIT(AL_TX(LBL_X), LBL_Y);
    static AL_TextField_t tf_count = AL_TEXTFIELD_INIT(AL_TX(CNT_X), CNT_Y);

    AL_Draw_TextField(&tf_label, LBL_FONT, "Lane lock in");

    char raw[4];
    snprintf(raw, sizeof(raw), "%d", d->freefall_cntdn_s);
    AL_Draw_TextField(&tf_count, CNT_FONT, raw);

    AL_Arrow_Erase(&s_target_arrow);
    if (d->has_target && d->has_gps)
        AL_Arrow_Draw(&s_target_arrow, ARROW_LX, ARROW_LY, d->rel_to_target_deg);
}

static void RenderCompetitionRun(const Mode1_Data_t *d)
{
    enum {
        GR_X   = 25,  GR_Y   = 200, GR_FONT   = 3,
        HS_X   = 55,  HS_Y   = 140, HS_FONT   = AL_FONT_INCONSOLATA_BOLD_90_ID,
        LANE_X = 30,  LANE_Y =  80, LANE_FONT  = 3,
    };
    static AL_TextField_t tf_gr   = AL_TEXTFIELD_INIT(AL_TX(GR_X),   GR_Y);
    static AL_TextField_t tf_hs   = AL_TEXTFIELD_INIT(AL_TX(HS_X),   HS_Y);
    static AL_TextField_t tf_lane = AL_TEXTFIELD_INIT(AL_TX(LANE_X), LANE_Y);

    char raw[16];
    if (d->has_gps && d->glide_ratio > 0.0f) {
        int gr10 = (int)(d->glide_ratio * 10.0f + 0.5f);
        snprintf(raw, sizeof(raw), "Gr: %d.%d", gr10 / 10, gr10 % 10);
    } else {
        snprintf(raw, sizeof(raw), "Gr: --");
    }
    AL_Draw_TextField(&tf_gr, GR_FONT, raw);

    if (d->has_gps) snprintf(raw, sizeof(raw), "%ld", (long)d->gspeed_kmh);
    else            snprintf(raw, sizeof(raw), "--");
    AL_Draw_Spaced_TextField(&tf_hs, HS_FONT, raw);

    if (d->lane_valid && d->has_gps)
        AL_Draw_TextField(&tf_lane, LANE_FONT, LaneArrow(d->lane_dev_m));
    else
        AL_Draw_TextField(&tf_lane, 2, "---NO LANE---");
}

static void RenderRunScore(const Mode1_Data_t *d)
{
    enum {
        TIME_X = 15,  TIME_Y =  60, TIME_FONT = 2,
        SPD_X  = 15,  SPD_Y  =  95, SPD_FONT  = 2,
        DIST_X = 15,  DIST_Y = 130, DIST_FONT = 2,
        LBL_X  = 130, LBL_Y  = 200, LBL_FONT  = 1,
        ARROW_LX = 90, ARROW_LY = 70,
    };
    static AL_TextField_t tf_time = AL_TEXTFIELD_INIT(AL_TX(TIME_X), TIME_Y);
    static AL_TextField_t tf_spd  = AL_TEXTFIELD_INIT(AL_TX(SPD_X),  SPD_Y);
    static AL_TextField_t tf_dist = AL_TEXTFIELD_INIT(AL_TX(DIST_X), DIST_Y);
    static AL_TextField_t tf_lbl  = AL_TEXTFIELD_INIT(AL_TX(LBL_X),  LBL_Y);

    char raw[AL_TEXTFIELD_MAXLEN];
    if (d->score_valid) {
        { int v = (int)(d->score_time_s * 10.0f + 0.5f);
          snprintf(raw, sizeof(raw), "T: %d.%ds", v / 10, v % 10); }
        AL_Draw_TextField(&tf_time, TIME_FONT, raw);
        { int v = (int)(d->score_speed_kmh * 10.0f + 0.5f);
          snprintf(raw, sizeof(raw), "S: %d.%dkm/h", v / 10, v % 10); }
        AL_Draw_TextField(&tf_spd,  SPD_FONT,  raw);
        { int v = (int)(d->score_dist_m * 10.0f + 0.5f);
          snprintf(raw, sizeof(raw), "D: %d.%dm", v / 10, v % 10); }
        AL_Draw_TextField(&tf_dist, DIST_FONT, raw);
    } else {
        AL_Draw_TextField(&tf_time, TIME_FONT, "T: --");
        AL_Draw_TextField(&tf_spd,  SPD_FONT,  "S: --");
        AL_Draw_TextField(&tf_dist, DIST_FONT, "D: --");
    }

    AL_Arrow_Erase(&s_dz_arrow);
    if (d->has_dz && d->has_gps) {
        AL_Arrow_Draw(&s_dz_arrow, ARROW_LX, ARROW_LY, d->rel_to_dz_deg);
        AL_Draw_TextField(&tf_lbl, LBL_FONT, "DZ");
    } else {
        AL_Draw_TextField(&tf_lbl, LBL_FONT, "");
    }
}

static void RenderHomeNav(const Mode1_Data_t *d)
{
    enum {
        ALT_X  = 25,  ALT_Y  = 120, ALT_FONT  = 3,
        DIST_X = 25,  DIST_Y =  60, DIST_FONT = 2,
        LBL_X  = 130, LBL_Y  = 200, LBL_FONT  = 1,
        ARROW_LX = 90, ARROW_LY = 70,
    };
    static AL_TextField_t tf_alt  = AL_TEXTFIELD_INIT(AL_TX(ALT_X),  ALT_Y);
    static AL_TextField_t tf_dist = AL_TEXTFIELD_INIT(AL_TX(DIST_X), DIST_Y);
    static AL_TextField_t tf_lbl  = AL_TEXTFIELD_INIT(AL_TX(LBL_X),  LBL_Y);

    char raw[16];
    if (d->has_gps) snprintf(raw, sizeof(raw), "Alt: %ldm", (long)d->alt_agl_m);
    else            snprintf(raw, sizeof(raw), "Alt: --");
    AL_Draw_TextField(&tf_alt, ALT_FONT, raw);

    if (d->has_dz && d->has_gps) snprintf(raw, sizeof(raw), "DZ: %ldm", (long)d->dz_dist_m);
    else                         snprintf(raw, sizeof(raw), "DZ: --");
    AL_Draw_TextField(&tf_dist, DIST_FONT, raw);

    AL_Arrow_Erase(&s_dz_arrow);
    if (d->has_dz && d->has_gps) {
        AL_Arrow_Draw(&s_dz_arrow, ARROW_LX, ARROW_LY, d->rel_to_dz_deg);
        AL_Draw_TextField(&tf_lbl, LBL_FONT, "DZ");
    } else {
        AL_Draw_TextField(&tf_lbl, LBL_FONT, "");
    }
}

/* --------------------------------------------------------------------------
   Mode 1 public API
   -------------------------------------------------------------------------- */

void FS_ActiveLook_Mode1_Init(void)
{
    Mode1_Logic_Init();

    s_target_arrow.valid = false;
    s_dz_arrow.valid     = false;
    s_font_setup_step    = 0;
    s_first_update       = true;
}

FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode1_Setup(void)
{
    /* Font upload state machine for slot AL_FONT_INCONSOLATA_BOLD_90_ID */

    if (s_font_setup_step == 0)
    {
        uint8_t pkt[8] = {
            0xFF, 0x51, 0x00, 0x08,
            AL_FONT_INCONSOLATA_BOLD_90_ID,
            (uint8_t)(INCONSOLATA_BOLD_90_SIZE >> 8),
            (uint8_t)(INCONSOLATA_BOLD_90_SIZE & 0xFF),
            0xAA
        };
        if (FS_ActiveLook_Client_WriteWithoutResp(pkt, sizeof(pkt)) != 0)
            return FS_AL_SETUP_IN_PROGRESS;
        s_font_setup_step = 1;
        return FS_AL_SETUP_IN_PROGRESS;
    }

    uint16_t offset = (uint16_t)(s_font_setup_step - 1) * AL_FONT_CHUNK_SIZE;
    if (offset >= INCONSOLATA_BOLD_90_SIZE)
        return FS_AL_SETUP_DONE;

    uint16_t chunk = INCONSOLATA_BOLD_90_SIZE - offset;
    if (chunk > AL_FONT_CHUNK_SIZE)
        chunk = AL_FONT_CHUNK_SIZE;

    uint8_t total_len = (uint8_t)(5u + chunk);
    uint8_t pkt[5u + AL_FONT_CHUNK_SIZE];
    pkt[0] = 0xFF;
    pkt[1] = 0x51;
    pkt[2] = 0x00;
    pkt[3] = total_len;
    memcpy(&pkt[4], &inconsolata_bold_90[offset], chunk);
    pkt[4 + chunk] = 0xAA;
    if (FS_ActiveLook_Client_WriteWithoutResp(pkt, total_len) != 0)
        return FS_AL_SETUP_IN_PROGRESS;

    s_font_setup_step++;

    return ((uint16_t)(offset + chunk) >= INCONSOLATA_BOLD_90_SIZE)
           ? FS_AL_SETUP_DONE
           : FS_AL_SETUP_IN_PROGRESS;
}

void FS_ActiveLook_Mode1_Update(void)
{
    if (s_first_update)
    {
        OnPhaseEnter();
        s_first_update = false;
    }

    bool phase_changed = Mode1_Logic_Update();
    const Mode1_Data_t *d = Mode1_Logic_GetData();

    if (phase_changed)
        OnPhaseEnter();

    const FS_VBAT_Data_t *vbat   = FS_VBAT_GetData();
    uint8_t               al_bat = FS_ActiveLook_Client_GetBatteryLevel();

    switch ((CompetitionPhase_t)d->phase)
    {
    case COMP_PHASE_IDLE:
        DrawStatusBar(d, vbat, al_bat);
        RenderIdle(d);
        break;
    case COMP_PHASE_AIRPLANE:
        DrawStatusBar(d, vbat, al_bat);
        RenderAirplane(d);
        break;
    case COMP_PHASE_FREEFALL:        RenderFreefall(d);        break;
    case COMP_PHASE_COMPETITION_RUN: RenderCompetitionRun(d);  break;
    case COMP_PHASE_RUN_SCORE:       RenderRunScore(d);        break;
    case COMP_PHASE_HOME_NAV:        RenderHomeNav(d);         break;
    }
}
