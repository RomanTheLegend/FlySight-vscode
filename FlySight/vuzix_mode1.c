/***************************************************************************
**  FlySight 2 firmware — Vuzix Z100 competition mode renderer            **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
**                                                                        **
**  Mirrors activelook_mode1.c: same 6 phases, same data fields.         **
**  Arrows are rasterised at runtime into a 28×28 2-bit canvas image and  **
**  sent via SendCanvasImg, giving continuous rotation rather than 8      **
**  fixed directions.                                                     **
****************************************************************************/

#include "vuzix_mode1.h"
#include "mode1_logic.h"
#include "vuzix_client.h"
#include "vbat.h"
#include "inconsolata_bold_90_lvgl.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
   Layout constants (480 × 480 display, portrait)
   -------------------------------------------------------------------------- */
#define LABEL_X       20

#define LABEL_Y0      60
#define LABEL_Y1     160
#define LABEL_Y2     260
#define LABEL_Y3     360

/* Arrow: right column, both dimensions 4-pixel aligned → fast canvas path */
#define ARROW_X      448u
#define ARROW_Y      160u
#define ARROW_W       28u
#define ARROW_H       28u
#define ARROW_LBL_X  400
#define ARROW_LBL_Y  130

#define ARROW_PIX    ((ARROW_W * ARROW_H) / 4u)   /* 196 bytes */

/* --------------------------------------------------------------------------
   Lane deviation thresholds (metres cross-track) — matches activelook_mode1
   -------------------------------------------------------------------------- */
#define LANE_MINOR_M    150.0f
#define LANE_MODERATE_M 200.0f
#define LANE_MAJOR_M    250.0f

/* --------------------------------------------------------------------------
   Module state
   -------------------------------------------------------------------------- */
static bool s_first_update;

/* --------------------------------------------------------------------------
   Arrow polygon — 7-vertex arrowhead pointing UP (north) at θ=0.
   Centre of mass at (0, 0); fits inside ±12 px from centre.
   -------------------------------------------------------------------------- */
#define ARROW_N  7

static const float kVX[ARROW_N] = {  0.0f, -10.0f, -4.0f, -4.0f,  4.0f,  4.0f, 10.0f };
static const float kVY[ARROW_N] = { -12.0f,  -2.0f, -2.0f, 12.0f, 12.0f, -2.0f, -2.0f };

/* --------------------------------------------------------------------------
   ArrowRender — rasterise rotated polygon into a 28×28 2-bpp buffer.

   Pixel format: LV_IMG_CF_INDEXED_2BIT, MSB-first, 4 px/byte, no row pad.
   Palette index 0=black (background), 3=white (arrow fill).

   Rotation formula (clockwise compass bearing θ, y-down screen):
     x' = cx + vx·cos(θ) − vy·sin(θ)
     y' = cy + vx·sin(θ) + vy·cos(θ)
   At θ=0 the tip points up (north); θ=90° → tip points right (east).
   -------------------------------------------------------------------------- */
static void ArrowRender(float bearing_deg, uint8_t *buf)
{
    float theta = bearing_deg * (3.14159265f / 180.0f);
    float cs = cosf(theta);
    float sn = sinf(theta);
    const float cx = 13.5f, cy = 13.5f;

    float rx[ARROW_N], ry[ARROW_N];
    for (int i = 0; i < ARROW_N; i++) {
        rx[i] = cx + kVX[i] * cs - kVY[i] * sn;
        ry[i] = cy + kVX[i] * sn + kVY[i] * cs;
    }

    memset(buf, 0, ARROW_PIX);

    for (int y = 0; y < (int)ARROW_H; y++) {
        float fy = (float)y + 0.5f;
        float xs[ARROW_N];
        int   nx = 0;

        for (int i = 0; i < ARROW_N; i++) {
            int   j  = (i + 1) % ARROW_N;
            float y0 = ry[i], y1 = ry[j];
            /* Count edge crossing when fy is strictly inside [min,max) */
            if ((y0 <= fy && y1 > fy) || (y1 <= fy && y0 > fy))
                xs[nx++] = rx[i] + (fy - y0) * (rx[j] - rx[i]) / (y1 - y0);
        }

        /* Insertion sort — nx ≤ ARROW_N = 7 */
        for (int a = 1; a < nx; a++) {
            float t = xs[a];
            int   b = a;
            while (b > 0 && xs[b - 1] > t) { xs[b] = xs[b - 1]; b--; }
            xs[b] = t;
        }

        /* Fill even/odd spans */
        for (int p = 0; p + 1 < nx; p += 2) {
            int x0 = (int)(xs[p]       + 0.5f);
            int x1 = (int)(xs[p + 1]   + 0.5f);
            if (x0 < 0)             x0 = 0;
            if (x1 > (int)ARROW_W)  x1 = (int)ARROW_W;
            for (int x = x0; x < x1; x++) {
                int bit = (y * (int)ARROW_W + x) * 2;
                buf[bit >> 3] |= (uint8_t)(3u << (6u - (unsigned)(bit & 7)));
            }
        }
    }
}

/* --------------------------------------------------------------------------
   Canvas helpers
   -------------------------------------------------------------------------- */

static void DrawArrow(float bearing_deg)
{
    static uint8_t s_pix[ARROW_PIX];
    ArrowRender(bearing_deg, s_pix);
    FS_Vuzix_Client_SendCanvasImg(ARROW_X, ARROW_Y, ARROW_W, ARROW_H,
                                   s_pix, (uint8_t)ARROW_PIX);
}

static void ClearArrow(void)
{
    FS_Vuzix_Client_SendCanvasRect(ARROW_X, ARROW_Y,
                                   ARROW_X + ARROW_W - 1u,
                                   ARROW_Y + ARROW_H - 1u, 0u);
}

/* --------------------------------------------------------------------------
   Text helpers
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

static void DrawStatusBar(const Mode1_Data_t *d, const FS_VBAT_Data_t *vbat,
                          uint8_t vz_batt)
{
    char vz_str[5];
    if (vz_batt == 255u) snprintf(vz_str, sizeof(vz_str), "??");
    else                 snprintf(vz_str, sizeof(vz_str), "%d", vz_batt);

    int fs_pct = (100 * ((int)vbat->voltage - 3300)) / (4200 - 3200);
    if (fs_pct < 0)   fs_pct = 0;
    if (fs_pct > 100) fs_pct = 100;

    char raw[32];
    snprintf(raw, sizeof(raw), "V:%s%%  F:%d%%  N:%d", vz_str, fs_pct, d->num_sv);
    FS_Vuzix_Client_SendLabel(4u, LABEL_X, 10, 1u, raw);
}

/* --------------------------------------------------------------------------
   Phase enter: wipe all labels and the arrow canvas region
   -------------------------------------------------------------------------- */

static void OnPhaseEnter(void)
{
    /* Clear the entire canvas so any OS-drawn icon (e.g. BT connection indicator)
     * doesn't bleed through into our layout. */
    FS_Vuzix_Client_SendCanvasRect(0, 0, 479, 479, 0);
    for (uint8_t i = 0u; i <= 4u; i++)
        FS_Vuzix_Client_SendLabel(i, 0, 0, 0u, "");
    FS_Vuzix_Client_SendFlush();
}

/* --------------------------------------------------------------------------
   Per-phase renderers — direct Vuzix equivalents of activelook_mode1.c
   -------------------------------------------------------------------------- */

static void RenderIdle(const Mode1_Data_t *d)
{
    char raw[32];

    if (d->has_gps) snprintf(raw, sizeof(raw), "Alt: %ldm",  (long)d->alt_agl_m);
    else            snprintf(raw, sizeof(raw), "Alt: --");
    FS_Vuzix_Client_SendLabel(0u, LABEL_X, LABEL_Y0, 1u, raw);

    if (d->has_gps) snprintf(raw, sizeof(raw), "hMSL: %ldm", (long)d->hMSL_m);
    else            snprintf(raw, sizeof(raw), "hMSL: --");
    FS_Vuzix_Client_SendLabel(1u, LABEL_X, LABEL_Y1, 1u, raw);

    snprintf(raw, sizeof(raw), "DZ e: %ldm", (long)d->dz_elev_m);
    FS_Vuzix_Client_SendLabel(2u, LABEL_X, LABEL_Y2, 1u, raw);

    ClearArrow();
    if (d->has_target && d->has_gps) {
        DrawArrow(d->rel_to_target_deg);
        FS_Vuzix_Client_SendLabel(3u, ARROW_LBL_X, ARROW_LBL_Y, 1u, "Target");
    } else {
        FS_Vuzix_Client_SendLabel(3u, 0, 0, 0u, "");
    }
}

static void RenderAirplane(const Mode1_Data_t *d)
{
    char raw[32];

    if (d->has_gps) snprintf(raw, sizeof(raw), "Alt: %ldm", (long)d->alt_agl_m);
    else            snprintf(raw, sizeof(raw), "Alt: --");
    FS_Vuzix_Client_SendLabelFont(0u, 0u, LABEL_X, LABEL_Y0, 1u, raw);

    if (d->has_gps) snprintf(raw, sizeof(raw), "Vh: %ldkm/h", (long)d->gspeed_kmh);
    else            snprintf(raw, sizeof(raw), "Vh: --");
    FS_Vuzix_Client_SendLabel(1u, LABEL_X, LABEL_Y1, 1u, raw);

    FS_Vuzix_Client_SendLabel(2u, 0, 0, 0u, "");

    ClearArrow();
    if (d->has_target && d->has_gps) {
        DrawArrow(d->rel_to_target_deg);
        FS_Vuzix_Client_SendLabel(3u, ARROW_LBL_X, ARROW_LBL_Y, 1u, "Target");
    } else {
        FS_Vuzix_Client_SendLabel(3u, 0, 0, 0u, "");
    }
}

static void RenderFreefall(const Mode1_Data_t *d)
{
    FS_Vuzix_Client_SendLabel(0u, LABEL_X, LABEL_Y0, 1u, "Lane lock in");

    char raw[8];
    snprintf(raw, sizeof(raw), "%d", d->freefall_cntdn_s);
    FS_Vuzix_Client_SendLabel(1u, LABEL_X, LABEL_Y1, 1u, raw);

    FS_Vuzix_Client_SendLabel(2u, 0, 0, 0u, "");

    ClearArrow();
    if (d->has_target && d->has_gps) {
        DrawArrow(d->rel_to_target_deg);
        FS_Vuzix_Client_SendLabel(3u, ARROW_LBL_X, ARROW_LBL_Y, 1u, "Target");
    } else {
        FS_Vuzix_Client_SendLabel(3u, 0, 0, 0u, "");
    }
}

static void RenderCompetitionRun(const Mode1_Data_t *d)
{
    char raw[32];

    if (d->has_gps && d->glide_ratio > 0.0f) {
        int gr10 = (int)(d->glide_ratio * 10.0f + 0.5f);
        snprintf(raw, sizeof(raw), "Gr: %d.%d", gr10 / 10, gr10 % 10);
    } else {
        snprintf(raw, sizeof(raw), "Gr: --");
    }
    FS_Vuzix_Client_SendLabel(0u, LABEL_X, LABEL_Y0, 1u, raw);

    if (d->has_gps) snprintf(raw, sizeof(raw), "%ldkm/h", (long)d->gspeed_kmh);
    else            snprintf(raw, sizeof(raw), "--");
    FS_Vuzix_Client_SendLabel(1u, LABEL_X, LABEL_Y1, 1u, raw);

    if (d->lane_valid && d->has_gps)
        FS_Vuzix_Client_SendLabel(2u, LABEL_X, LABEL_Y2, 1u, LaneArrow(d->lane_dev_m));
    else
        FS_Vuzix_Client_SendLabel(2u, LABEL_X, LABEL_Y2, 1u, "---NO LANE---");

    /* No navigation arrow during the run */
    ClearArrow();
    FS_Vuzix_Client_SendLabel(3u, 0, 0, 0u, "");
}

static void RenderRunScore(const Mode1_Data_t *d)
{
    char raw[32];

    if (d->score_valid) {
        { int v = (int)(d->score_time_s    * 10.0f + 0.5f);
          snprintf(raw, sizeof(raw), "T: %d.%ds", v / 10, v % 10); }
        FS_Vuzix_Client_SendLabel(0u, LABEL_X, LABEL_Y0, 1u, raw);
        { int v = (int)(d->score_speed_kmh * 10.0f + 0.5f);
          snprintf(raw, sizeof(raw), "S: %d.%dkm/h", v / 10, v % 10); }
        FS_Vuzix_Client_SendLabel(1u, LABEL_X, LABEL_Y1, 1u, raw);
        { int v = (int)(d->score_dist_m    * 10.0f + 0.5f);
          snprintf(raw, sizeof(raw), "D: %d.%dm", v / 10, v % 10); }
        FS_Vuzix_Client_SendLabel(2u, LABEL_X, LABEL_Y2, 1u, raw);
    } else {
        FS_Vuzix_Client_SendLabel(0u, LABEL_X, LABEL_Y0, 1u, "T: --");
        FS_Vuzix_Client_SendLabel(1u, LABEL_X, LABEL_Y1, 1u, "S: --");
        FS_Vuzix_Client_SendLabel(2u, LABEL_X, LABEL_Y2, 1u, "D: --");
    }

    ClearArrow();
    if (d->has_dz && d->has_gps) {
        DrawArrow(d->rel_to_dz_deg);
        FS_Vuzix_Client_SendLabel(3u, ARROW_LBL_X, ARROW_LBL_Y, 1u, "DZ");
    } else {
        FS_Vuzix_Client_SendLabel(3u, 0, 0, 0u, "");
    }
}

static void RenderHomeNav(const Mode1_Data_t *d)
{
    char raw[32];

    if (d->has_gps) snprintf(raw, sizeof(raw), "Alt: %ldm", (long)d->alt_agl_m);
    else            snprintf(raw, sizeof(raw), "Alt: --");
    FS_Vuzix_Client_SendLabel(0u, LABEL_X, LABEL_Y0, 1u, raw);

    if (d->has_dz && d->has_gps) snprintf(raw, sizeof(raw), "DZ: %ldm", (long)d->dz_dist_m);
    else                         snprintf(raw, sizeof(raw), "DZ: --");
    FS_Vuzix_Client_SendLabel(1u, LABEL_X, LABEL_Y1, 1u, raw);

    FS_Vuzix_Client_SendLabel(2u, 0, 0, 0u, "");

    ClearArrow();
    if (d->has_dz && d->has_gps) {
        DrawArrow(d->rel_to_dz_deg);
        FS_Vuzix_Client_SendLabel(3u, ARROW_LBL_X, ARROW_LBL_Y, 1u, "DZ");
    } else {
        FS_Vuzix_Client_SendLabel(3u, 0, 0, 0u, "");
    }
}

/* --------------------------------------------------------------------------
   Public API
   -------------------------------------------------------------------------- */

void FS_Vuzix_Mode1_Init(void)
{
    Mode1_Logic_Init();
    s_first_update = true;
}

VZ_SetupStatus_t FS_Vuzix_Mode1_Setup(void)
{
    tBleStatus st = FS_Vuzix_Client_SendFontSlot(0u, inconsolata_bold_90_lvgl,
                                                   inconsolata_bold_90_lvgl_len);
    return (st == BLE_STATUS_SUCCESS) ? VZ_SETUP_DONE : VZ_SETUP_IN_PROGRESS;
}

void FS_Vuzix_Mode1_Update(void)
{
    if (s_first_update) {
        OnPhaseEnter();
        s_first_update = false;
    }

    bool phase_changed = Mode1_Logic_Update();
    const Mode1_Data_t *d = Mode1_Logic_GetData();

    if (phase_changed)
        OnPhaseEnter();

    const FS_VBAT_Data_t *vbat   = FS_VBAT_GetData();
    uint8_t               vz_bat = FS_Vuzix_Client_GetBatteryLevel();

    switch ((CompetitionPhase_t)d->phase)
    {
    case COMP_PHASE_IDLE:
        DrawStatusBar(d, vbat, vz_bat);
        RenderIdle(d);
        break;
    case COMP_PHASE_AIRPLANE:
        DrawStatusBar(d, vbat, vz_bat);
        RenderAirplane(d);
        break;
    case COMP_PHASE_FREEFALL:        RenderFreefall(d);        break;
    case COMP_PHASE_COMPETITION_RUN: RenderCompetitionRun(d);  break;
    case COMP_PHASE_RUN_SCORE:       RenderRunScore(d);        break;
    case COMP_PHASE_HOME_NAV:        RenderHomeNav(d);         break;
    }

    FS_Vuzix_Client_SendFlush();
}
