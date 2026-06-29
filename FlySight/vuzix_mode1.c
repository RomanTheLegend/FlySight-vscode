/***************************************************************************
**  FlySight 2 firmware — Vuzix Z100 competition mode renderer            **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
**                                                                        **
**  Thin text renderer — delegates all flight logic to mode1_logic.c and  **
**  sends one multiline string per frame via FS_Vuzix_Client_SendText().  **
****************************************************************************/

#include "vuzix_mode1.h"
#include "mode1_logic.h"
#include "vuzix_client.h"
#include <stdio.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
   Lane deviation display thresholds (metres cross-track) — matches AL
   -------------------------------------------------------------------------- */
#define LANE_MINOR_M    150.0f
#define LANE_MODERATE_M 200.0f
#define LANE_MAJOR_M    250.0f

/* --------------------------------------------------------------------------
   Helpers
   -------------------------------------------------------------------------- */

/* Relative bearing (−180..180) → 8-direction ASCII arrow. */
static const char *BearingArrow(float rel_deg)
{
    if      (rel_deg >  -22.5f && rel_deg <=   22.5f) return "^";
    else if (rel_deg >   22.5f && rel_deg <=   67.5f) return "^>";
    else if (rel_deg >   67.5f && rel_deg <=  112.5f) return ">";
    else if (rel_deg >  112.5f && rel_deg <=  157.5f) return "v>";
    else if (rel_deg <= -22.5f && rel_deg >  -67.5f)  return "<^";
    else if (rel_deg <= -67.5f && rel_deg > -112.5f)  return "<";
    else if (rel_deg <= -112.5f && rel_deg > -157.5f) return "<v";
    else                                               return "v";
}

static const char *LaneDevText(float dev_m)
{
    if      (dev_m >  LANE_MAJOR_M)    return "<<<";
    else if (dev_m >  LANE_MODERATE_M) return " <<";
    else if (dev_m >  LANE_MINOR_M)    return "  <";
    else if (dev_m < -LANE_MAJOR_M)    return ">>>";
    else if (dev_m < -LANE_MODERATE_M) return ">> ";
    else if (dev_m < -LANE_MINOR_M)    return ">  ";
    else                               return " + ";
}

/* --------------------------------------------------------------------------
   Public API
   -------------------------------------------------------------------------- */

void FS_Vuzix_Mode1_Init(void)
{
    Mode1_Logic_Init();
}

VZ_SetupStatus_t FS_Vuzix_Mode1_Setup(void)
{
    return VZ_SETUP_DONE;
}

void FS_Vuzix_Mode1_Update(void)
{
    Mode1_Logic_Update();
    const Mode1_Data_t *d = Mode1_Logic_GetData();

    char buf[256];
    int  len = 0;

#define APPEND(...)  len += snprintf(buf + len, (int)sizeof(buf) - len, __VA_ARGS__)

    switch ((CompetitionPhase_t)d->phase)
    {
    /* ---- IDLE ---------------------------------------------------------- */
    case COMP_PHASE_IDLE:
        if (d->has_gps) APPEND("Alt: %ldm\nhMSL: %ldm\nDZ e: %ldm",
                                (long)d->alt_agl_m, (long)d->hMSL_m, (long)d->dz_elev_m);
        else            APPEND("Alt: --\nhMSL: --\nDZ e: %ldm", (long)d->dz_elev_m);
        if (d->has_target && d->has_gps)
            APPEND("\nTGT: %s", BearingArrow(d->rel_to_target_deg));
        break;

    /* ---- AIRPLANE ------------------------------------------------------ */
    case COMP_PHASE_AIRPLANE:
        if (d->has_gps) APPEND("Alt: %ldm\nVh: %ldkm/h",
                                (long)d->alt_agl_m, (long)d->gspeed_kmh);
        else            APPEND("Alt: --\nVh: --");
        if (d->has_target && d->has_gps)
            APPEND("\nTGT: %s", BearingArrow(d->rel_to_target_deg));
        break;

    /* ---- FREEFALL ------------------------------------------------------ */
    case COMP_PHASE_FREEFALL:
        APPEND("Lane lock in\n%d", d->freefall_cntdn_s);
        if (d->has_target && d->has_gps)
            APPEND("\nTGT: %s", BearingArrow(d->rel_to_target_deg));
        break;

    /* ---- COMPETITION_RUN ----------------------------------------------- */
    case COMP_PHASE_COMPETITION_RUN:
        if (d->has_gps && d->glide_ratio > 0.0f) {
            int gr10 = (int)(d->glide_ratio * 10.0f + 0.5f);
            APPEND("Gr: %d.%d\n", gr10 / 10, gr10 % 10);
        } else {
            APPEND("Gr: --\n");
        }
        if (d->has_gps) APPEND("%ldkm/h\n", (long)d->gspeed_kmh);
        else            APPEND("--\n");
        if (d->lane_valid && d->has_gps)
            APPEND("%s", LaneDevText(d->lane_dev_m));
        else
            APPEND("NO LANE");
        break;

    /* ---- RUN_SCORE ----------------------------------------------------- */
    case COMP_PHASE_RUN_SCORE:
        if (d->score_valid) {
            int tv = (int)(d->score_time_s    * 10.0f + 0.5f);
            int sv = (int)(d->score_speed_kmh * 10.0f + 0.5f);
            int dv = (int)(d->score_dist_m    * 10.0f + 0.5f);
            APPEND("T: %d.%ds\nS: %d.%dkm/h\nD: %d.%dm",
                   tv/10, tv%10, sv/10, sv%10, dv/10, dv%10);
        } else {
            APPEND("T: --\nS: --\nD: --");
        }
        if (d->has_dz && d->has_gps)
            APPEND("\nDZ: %s", BearingArrow(d->rel_to_dz_deg));
        break;

    /* ---- HOME_NAV ------------------------------------------------------ */
    case COMP_PHASE_HOME_NAV:
        if (d->has_gps) APPEND("Alt: %ldm\n", (long)d->alt_agl_m);
        else            APPEND("Alt: --\n");
        if (d->has_dz && d->has_gps)
            APPEND("DZ: %ldm\nDZ: %s", (long)d->dz_dist_m, BearingArrow(d->rel_to_dz_deg));
        else
            APPEND("DZ: --");
        break;
    }

#undef APPEND

    FS_Vuzix_Client_SendText(buf);
}
