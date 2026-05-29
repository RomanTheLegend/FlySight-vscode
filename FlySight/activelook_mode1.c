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
 * Competition Mode (al_mode = 2) — Phase state machine
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
 *
 * Display refresh: AL_Draw_ClearScreen() is called only on phase changes
 * (one-time blink).  Within a phase, text fields are overwritten using '$'
 * padding and the arrow is erased/redrawn per frame (no full-screen flash).
 *
 * Config keys used by this mode:
 *   Lat / Lon           — DZ landing area (for RUN_SCORE and HOME_NAV arrows)
 *   Target_Lat / _Lon   — competition run target (for IDLE … FREEFALL arrows)
 *   DZ_Elev             — drop-zone elevation in mm
 *
 * Score capture (COMPETITION_RUN):
 *   Point A = GNSS sample closest to 2500 m AGL (descending)
 *   Point B = GNSS sample closest to 1500 m AGL (descending)
 *   Time     = iTOW(B) − iTOW(A)  [seconds]
 *   Distance = great-circle distance A→B  [metres]
 *   Speed    = Distance / Time  [km/h]
 */

/*
 * Debug: force the state machine to a specific phase on the first GPS fix,
 * bypassing all transition logic.  Set DEBUG_FORCE_PHASE to one of:
 *   -1                      — disabled (normal flight logic)
 *   COMP_PHASE_IDLE         = 0
 *   COMP_PHASE_AIRPLANE     = 1
 *   COMP_PHASE_FREEFALL     = 2
 *   COMP_PHASE_COMPETITION_RUN = 3
 *   COMP_PHASE_RUN_SCORE    = 4
 *   COMP_PHASE_HOME_NAV     = 5
 * Set to -1 before any real flight.
 */
#define DEBUG_FORCE_PHASE  -1

#include "activelook_mode1.h"
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
   Freefall detection constants  (ported from freefall.js / FlyScope)
   -------------------------------------------------------------------------- */

#define FREEFALL_WINDOW_SIZE         4
#define A_GRAVITY_MS2                9.81f
#define EXIT_VELOCITY_THRESHOLD_MS   A_GRAVITY_MS2
#define ACCEL_MIN_THRESHOLD_MS2      (A_GRAVITY_MS2 / 5.0f)   /* g/5 */
#define GPS_VACC_MAX_METERS          10.0f

/* --------------------------------------------------------------------------
   Phase-transition thresholds
   -------------------------------------------------------------------------- */

#define AIRPLANE_ENTER_AGL_M         100    /* IDLE → AIRPLANE (m AGL) */
#define AIRPLANE_EXIT_AGL_M           50    /* AIRPLANE → IDLE if altitude lost */
#define FREEFALL_LOCK_TIME_MS      10000    /* ms after freefall → COMPETITION_RUN */
#define CANOPY_VELDOWN_MS           10.0f   /* m/s  — FREEFALL→CANOPY threshold */
#define COMP_SCORE_ALT_A_M          2500    /* AGL for point A capture */
#define COMP_SCORE_ALT_B_M          1500    /* AGL for point B capture; also phase exit */
#define RUN_SCORE_DISPLAY_DURATION_MS          15000    /* ms in RUN_SCORE before HOME_NAV */
#define HOME_NAV_EXIT_VELDOWN_MS     3.0f   /* avg velD below this → IDLE (landed) */
#define HOME_NAV_RING_SIZE            10    /* GNSS samples in velD averaging window */

/* --------------------------------------------------------------------------
   Lane deviation thresholds (metres cross-track)
   -------------------------------------------------------------------------- */

#define LANE_MINOR_M    10.0f
#define LANE_MODERATE_M 50.0f
#define LANE_MAJOR_M   100.0f

/* --------------------------------------------------------------------------
   Coordinate system reference
   *
   * raw_y  = AL_DISPLAY_HEIGHT − logical_y  (256 − logical_y)
   * raw_x  = AL_TX(logical_x)               (304 − logical_x)
   *
   * Logical layout (physical viewer perspective, 0,0 = bottom-left):
   *
   *  y=248  status bar
   *  y=200  typical first content line
   *  y=180  typical second content line
   *  y=160  typical third content line
   *  y=108  arrow centre  (ARROW_LX=152, ARROW_LY=108 are typical defaults)
   *  y= 58  arrow label
   *
   * Every render function owns its element coordinates via a local enum — edit
   * the enum inside the function to reposition or resize individual elements.
   * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
   Types
   -------------------------------------------------------------------------- */

typedef enum {
    COMP_PHASE_IDLE            = 0,
    COMP_PHASE_AIRPLANE        = 1,
    COMP_PHASE_FREEFALL        = 2,
    COMP_PHASE_COMPETITION_RUN = 3,
    COMP_PHASE_RUN_SCORE       = 4,
    COMP_PHASE_HOME_NAV        = 5,
} CompetitionPhase_t;

typedef struct {
    uint32_t itow_ms;
    float    vel_down_ms;   /* velD in m/s, positive = descending */
    float    vert_acc_m;    /* vAcc in m */
    float    accel_ms2;     /* least-squares slope (vertical acceleration) */
    int32_t  lat;           /* 1e-7 degrees */
    int32_t  lon;
    int32_t  hMSL_mm;
} FreefallSample_t;

/* --------------------------------------------------------------------------
   Module-level state
   -------------------------------------------------------------------------- */

static CompetitionPhase_t  s_phase           = COMP_PHASE_IDLE;
static CompetitionPhase_t  s_render_phase    = COMP_PHASE_IDLE; /* phase of last draw */

/* Freefall detection */
static FreefallSample_t    s_ff_window[FREEFALL_WINDOW_SIZE];
static uint8_t             s_ff_count        = 0;

/* Phase timing (GPS iTOW, ms) */
static uint32_t            s_freefall_itow   = 0;
static uint32_t            s_cr_start_itow   = 0;   /* COMPETITION_RUN start */
static uint32_t            s_run_score_itow  = 0;

/* Lane definition: straight line from s_lane_start → config target */
static int32_t             s_lane_start_lat  = 0;
static int32_t             s_lane_start_lon  = 0;
/* Extended endpoint: 5 km beyond target along the same bearing */
static int32_t             s_lane_ext_lat    = 0;
static int32_t             s_lane_ext_lon    = 0;
							
/* Score capture */
static bool                s_pt_a_valid      = false;
static bool                s_pt_b_valid      = false;
static int32_t             s_pt_a_lat        = 0;
static int32_t             s_pt_a_lon        = 0;
static uint32_t            s_pt_a_itow       = 0;
static int32_t             s_pt_b_lat        = 0;
static int32_t             s_pt_b_lon        = 0;
static uint32_t            s_pt_b_itow       = 0;

/* Previous GNSS sample — needed for closest-crossing logic */
static int32_t             s_prev_lat        = 0;
static int32_t             s_prev_lon        = 0;
static uint32_t            s_prev_itow       = 0;
static int32_t             s_prev_alt_agl_m  = 0;

/* Score results */
static float               s_score_time_s    = 0.0f;
static float               s_score_speed_kmh = 0.0f;
static float               s_score_dist_m    = 0.0f;
static bool                s_score_valid     = false;

/* HOME_NAV: rolling velD buffer for 10-sample average */
static float               s_velD_ring[HOME_NAV_RING_SIZE];
static uint8_t             s_velD_head       = 0;
static uint8_t             s_velD_count      = 0;

/* Arrow instances */
static AL_ArrowState_t     s_target_arrow    = AL_ARROW_STATE_INIT;
static AL_ArrowState_t     s_dz_arrow        = AL_ARROW_STATE_INIT;
/* Text fields are declared as static locals inside each render function. */

/* --------------------------------------------------------------------------
   Freefall detection  (ported from freefall.js / FlyScope)
   -------------------------------------------------------------------------- */

static float ComputeVelDSlope(void)
{
    float sum_t = 0, sum_v = 0, sum_tt = 0, sum_tv = 0;
    int n = FREEFALL_WINDOW_SIZE;
    for (int i = 0; i < n; i++) {
        float t = (float)(s_ff_window[i].itow_ms - s_ff_window[0].itow_ms) / 1000.0f;
        float v = s_ff_window[i].vel_down_ms;
        sum_t += t; sum_v += v; sum_tt += t*t; sum_tv += t*v;
    }
    float denom = sum_tt - sum_t * sum_t / (float)n;
    if (fabsf(denom) < 1e-6f) return 0.0f;
    return (sum_tv - sum_t * sum_v / (float)n) / denom;
}

static void AddSampleToWindow(const FS_GNSS_Data_t *gnss)
{
    for (int i = 0; i < FREEFALL_WINDOW_SIZE - 1; i++)
        s_ff_window[i] = s_ff_window[i + 1];

    FreefallSample_t *s  = &s_ff_window[FREEFALL_WINDOW_SIZE - 1];
    s->itow_ms     = gnss->iTOW;
    s->vel_down_ms = (float)gnss->velD / 1000.0f;
    s->vert_acc_m  = (float)gnss->vAcc / 1000.0f;
    s->accel_ms2   = 0.0f;
    s->lat         = gnss->lat;
    s->lon         = gnss->lon;
    s->hMSL_mm     = gnss->hMSL;

    if (s_ff_count < FREEFALL_WINDOW_SIZE) s_ff_count++;
    if (s_ff_count == FREEFALL_WINDOW_SIZE) s->accel_ms2 = ComputeVelDSlope();
}

static bool DetectFreefallExit(void)
{
    if (s_ff_count < FREEFALL_WINDOW_SIZE) return false;

    int prev = FREEFALL_WINDOW_SIZE - 2;
    int curr = FREEFALL_WINDOW_SIZE - 1;

    float vel_delta = s_ff_window[curr].vel_down_ms - s_ff_window[prev].vel_down_ms;
    if (fabsf(vel_delta) < 1e-3f) return false;

    float interp = (EXIT_VELOCITY_THRESHOLD_MS - s_ff_window[prev].vel_down_ms) / vel_delta;
    if (interp < 0.0f || interp > 1.0f) return false;

    float vacc = s_ff_window[prev].vert_acc_m
               + interp * (s_ff_window[curr].vert_acc_m - s_ff_window[prev].vert_acc_m);
    if (vacc > GPS_VACC_MAX_METERS) return false;

    float accel = s_ff_window[prev].accel_ms2
                + interp * (s_ff_window[curr].accel_ms2 - s_ff_window[prev].accel_ms2);
    if (accel < ACCEL_MIN_THRESHOLD_MS2) return false;

    return true;
}

/* --------------------------------------------------------------------------
   Navigation helpers
   -------------------------------------------------------------------------- */

/* Project a point 5 km beyond tgt along the start→tgt bearing.
 * Coordinates are in 1e-7 degrees (u-blox raw format). */
static void ExtendLane(int32_t start_lat, int32_t start_lon,
                       int32_t tgt_lat,   int32_t tgt_lon,
                       int32_t *ext_lat,  int32_t *ext_lon)
{
    const double R   = 6371100.0;              /* Earth radius, metres */
    const double EXT = 5000.0;                 /* extension distance, metres */
    const double S   = 1e-7 * M_PI / 180.0;   /* 1e-7 deg → radians */

    double latA = start_lat * S;
    double lonA = start_lon * S;
    double latB = tgt_lat   * S;
    double lonB = tgt_lon   * S;

    double dLon    = lonB - lonA;
    double y       = sin(dLon) * cos(latB);
    double x       = cos(latA) * sin(latB) - sin(latA) * cos(latB) * cos(dLon);
    double bearing = atan2(y, x);

    double dr   = EXT / R;
    double latC = asin(sin(latB) * cos(dr) + cos(latB) * sin(dr) * cos(bearing));
    double lonC = lonB + atan2(sin(bearing) * sin(dr) * cos(latB),
                               cos(dr) - sin(latB) * sin(latC));

    *ext_lat = (int32_t)(latC / S);
    *ext_lon = (int32_t)(lonC / S);
}

static float BearingDeg(int32_t lat_a, int32_t lon_a, int32_t lat_b, int32_t lon_b)
{
    float dlat    = (float)(lat_b - lat_a);
    float lat_rad = (float)lat_a * 1e-7f * (float)M_PI / 180.0f;
    float dlon    = (float)(lon_b - lon_a) * cosf(lat_rad);
    return atan2f(dlon, dlat) * 180.0f / (float)M_PI;
}

/* Signed cross-track deviation in metres.
 * Positive = right of lane (steer left), negative = left (steer right). */
static float CrossTrackMeters(int32_t lane_lat, int32_t lane_lon,
                               int32_t tgt_lat,  int32_t tgt_lon,
                               int32_t cur_lat,  int32_t cur_lon)
{
    float b_to_tgt = BearingDeg(lane_lat, lane_lon, tgt_lat, tgt_lon);
    float b_to_cur = BearingDeg(lane_lat, lane_lon, cur_lat, cur_lon);
    float dist     = (float)calcDistance(lane_lat, lane_lon, cur_lat, cur_lon);
    return dist * sinf((b_to_cur - b_to_tgt) * (float)M_PI / 180.0f);
}

/* Relative bearing from current position/heading toward a target. */
static float RelativeBearing(int32_t cur_lat, int32_t cur_lon,
                              int32_t tgt_lat, int32_t tgt_lon,
                              int32_t heading_1e5_deg)
{
    float bearing = BearingDeg(cur_lat, cur_lon, tgt_lat, tgt_lon);
    float heading = (float)heading_1e5_deg / 100000.0f;
    float rel     = bearing - heading;
    while (rel >  180.0f) rel -= 360.0f;
    while (rel < -180.0f) rel += 360.0f;
    return rel;
}

static const char *LaneArrow(float dev_m)
{
    if      (dev_m >  LANE_MAJOR_M)    return "<<<";
    else if (dev_m >  LANE_MODERATE_M) return "  <<";
    else if (dev_m >  LANE_MINOR_M)    return "    <";
    else if (dev_m < -LANE_MAJOR_M)    return "     >>>";
    else if (dev_m < -LANE_MODERATE_M) return "     >>";
    else if (dev_m < -LANE_MINOR_M)    return "     >";
    else                               return "   ===";
}

static float AverageVelD(void)
{
    if (s_velD_count == 0) return 0.0f;
    float sum = 0.0f;
    for (uint8_t i = 0; i < s_velD_count; i++)
        sum += s_velD_ring[i];
    return sum / (float)s_velD_count;
}

/* --------------------------------------------------------------------------
   Status bar helper (common to all phases)
   -------------------------------------------------------------------------- */

static void DrawStatusBar(const FS_GNSS_Data_t *gnss, const FS_VBAT_Data_t *vbat,
                           uint8_t al_batt)
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
    snprintf(raw, sizeof(raw), "A:%s%%  F:%d%%  N:%d", al_str, fs_pct, gnss->numSV);
    AL_Draw_TextField(&tf, FONT, raw);
}

/* --------------------------------------------------------------------------
   Phase-change handler — clear screen, reset arrow state
   -------------------------------------------------------------------------- */

static void OnPhaseEnter(void)
{
    AL_Draw_ClearScreen();    /* also advances s_screen_gen — all text fields auto-invalidate */
    s_target_arrow.valid = false;
    s_dz_arrow.valid     = false;
}

/* --------------------------------------------------------------------------
   Per-phase rendering
   -------------------------------------------------------------------------- */

static void RenderIdle(const FS_GNSS_Data_t *gnss, const FS_Config_Data_t *cfg)
{
    enum {
        ALT_X = 25,   ALT_Y = 130,  ALT_FONT = 3,   /* altitude above MSL */
        HMLS_X = 25,   HMSL_Y = 83,  HMSL_FONT = 1,   /* altitude above MSL */
        DZ_X  = 25,   DZ_Y  = 60,  DZ_FONT  = 1,   /* declared DZ elevation */
        LBL_X = 55, LBL_Y = 155, LBL_FONT = 1,   /* "Run target" arrow label */
        ARROW_LX = 100, ARROW_LY = 70,
    };
    static AL_TextField_t tf_alt   = AL_TEXTFIELD_INIT(AL_TX(ALT_X), ALT_Y);
    static AL_TextField_t tf_hmsl   = AL_TEXTFIELD_INIT(AL_TX(HMLS_X), HMSL_Y);
    static AL_TextField_t tf_dz    = AL_TEXTFIELD_INIT(AL_TX(DZ_X),  DZ_Y);
    static AL_TextField_t tf_label = AL_TEXTFIELD_INIT(AL_TX(LBL_X), LBL_Y);

    char raw[16];
    if (gnss->gpsFix == 3) snprintf(raw, sizeof(raw), "Alt: %ldm", (long)((gnss->hMSL - cfg->dz_elev) / 1000));
    else                   snprintf(raw, sizeof(raw), "Alt: --");
    AL_Draw_TextField(&tf_alt, ALT_FONT, raw);

    if (gnss->gpsFix == 3) snprintf(raw, sizeof(raw), "hMSL: %ldm", (long)(gnss->hMSL / 1000));
    else                   snprintf(raw, sizeof(raw), "hMSL: --");
    AL_Draw_TextField(&tf_hmsl, HMSL_FONT, raw);

    snprintf(raw, sizeof(raw), "DZ e: %ldm", (long)(cfg->dz_elev / 1000));
    AL_Draw_TextField(&tf_dz, DZ_FONT, raw);

    bool has_target = (cfg->al_target_lat != 0 || cfg->al_target_lon != 0);
    AL_Arrow_Erase(&s_target_arrow);
    if (has_target && gnss->gpsFix == 3) {
        // float angle = (float)(gnss->iTOW / 100 % 360);  /* TEST: 10°/s rotation */

        float angle = RelativeBearing(gnss->lat, gnss->lon,
                                       cfg->al_target_lat, cfg->al_target_lon,
                                       gnss->heading);

        AL_Arrow_Draw(&s_target_arrow, ARROW_LX, ARROW_LY, angle);
        AL_Draw_TextField(&tf_label, LBL_FONT, "Run target");
    } else {
        AL_Draw_TextField(&tf_label, LBL_FONT, "");
    }
}

static void RenderAirplane(const FS_GNSS_Data_t *gnss, const FS_Config_Data_t *cfg)
{
    enum {

        ALT_X = 25,   ALT_Y = 95,  ALT_FONT = 2,   /* altitude AGL */
        HS_X  = 25,   HS_Y  = 60,  HS_FONT  = 3,   /* horizontal speed */
        LBL_X = 55, LBL_Y = 130, LBL_FONT = 1,   /* "Run target" arrow label */
        ARROW_LX = 100, ARROW_LY = 90,
    };
    static AL_TextField_t tf_alt   = AL_TEXTFIELD_INIT(AL_TX(ALT_X), ALT_Y);
    static AL_TextField_t tf_hs    = AL_TEXTFIELD_INIT(AL_TX(HS_X),  HS_Y);
    static AL_TextField_t tf_label = AL_TEXTFIELD_INIT(AL_TX(LBL_X), LBL_Y);

    char raw[16];
    int32_t alt_agl_m = (gnss->hMSL - cfg->dz_elev) / 1000;
    if (gnss->gpsFix == 3) snprintf(raw, sizeof(raw), "Alt: %ldm", (long)alt_agl_m);
    else                   snprintf(raw, sizeof(raw), "--");
    AL_Draw_TextField(&tf_alt, ALT_FONT, raw);

    if (gnss->gpsFix == 3) snprintf(raw, sizeof(raw), "Vh : %d", (int)((float)gnss->gSpeed * 0.036f));
    else                   snprintf(raw, sizeof(raw), "--");
    AL_Draw_TextField(&tf_hs, HS_FONT, raw);

    bool has_target = (cfg->al_target_lat != 0 || cfg->al_target_lon != 0);
    AL_Arrow_Erase(&s_target_arrow);
    if (has_target && gnss->gpsFix == 3) {
        float angle = RelativeBearing(gnss->lat, gnss->lon,
                                       cfg->al_target_lat, cfg->al_target_lon,
                                       gnss->heading);
        AL_Arrow_Draw(&s_target_arrow, ARROW_LX, ARROW_LY, angle);
        AL_Draw_TextField(&tf_label, LBL_FONT, "Run target");
    } else {
        AL_Draw_TextField(&tf_label, LBL_FONT, "");
    }
}

static void RenderFreefall(const FS_GNSS_Data_t *gnss, const FS_Config_Data_t *cfg)
{
    enum {
        /* Upper third (raw y 0–85): "Lane lock in" label */
        LBL_X = 25,   LBL_Y = 130,  LBL_FONT = 2,
        /* Lower two-thirds (raw y 86–255), visually centred: big countdown */
        CNT_X = 90, CNT_Y = 90, CNT_FONT = 3,
        ARROW_LX = 80, ARROW_LY = 95,
    };
    static AL_TextField_t tf_label = AL_TEXTFIELD_INIT(AL_TX(LBL_X), LBL_Y);
    static AL_TextField_t tf_count = AL_TEXTFIELD_INIT(AL_TX(CNT_X), CNT_Y);

    AL_Draw_TextField(&tf_label, LBL_FONT, "Lane lock in");

    uint32_t elapsed_ms = gnss->iTOW - s_freefall_itow;
    int countdown = 10 - (int)(elapsed_ms / 1000);
    if (countdown < 0) countdown = 0;

    char raw[4];
    snprintf(raw, sizeof(raw), "%d", countdown);
    AL_Draw_TextField(&tf_count, CNT_FONT, raw);

    // bool has_target = true;
    bool has_target = (cfg->al_target_lat != 0 || cfg->al_target_lon != 0);
    AL_Arrow_Erase(&s_target_arrow);
    if (has_target && gnss->gpsFix == 3) {
        float angle = RelativeBearing(gnss->lat, gnss->lon,
                                       cfg->al_target_lat, cfg->al_target_lon,
                                       gnss->heading);
        AL_Arrow_Draw(&s_target_arrow, ARROW_LX, ARROW_LY, angle);
    }

}

static void RenderCompetitionRun(const FS_GNSS_Data_t *gnss, const FS_Config_Data_t *cfg)
{
    enum {
        GR_X   = 25,  GR_Y   = 200,  GR_FONT   = 2,   /* glide ratio, left */
        HS_X   = 85,  HS_Y   = 140,  HS_FONT   = 3,   /* horizontal speed, right */
        LANE_X = 30,  LANE_Y = 80, LANE_FONT = 3,   /* lane deviation arrows */
    };
    static AL_TextField_t tf_gr   = AL_TEXTFIELD_INIT(AL_TX(GR_X),  GR_Y);
    static AL_TextField_t tf_hs   = AL_TEXTFIELD_INIT(AL_TX(HS_X),  HS_Y);
    static AL_TextField_t tf_lane = AL_TEXTFIELD_INIT(AL_TX(LANE_X), LANE_Y);

    char raw[16];
    if (gnss->gpsFix == 3 && gnss->velD > 0) {
        float gr = (float)gnss->gSpeed * 10.0f / (float)gnss->velD;
        snprintf(raw, sizeof(raw), "Gr: %.1f", (double)gr);
    } else {
        snprintf(raw, sizeof(raw), "Gr: --");
    }
    AL_Draw_TextField(&tf_gr, GR_FONT, raw);

    if (gnss->gpsFix == 3) snprintf(raw, sizeof(raw), "%d", (int)((float)gnss->gSpeed * 0.036f));
    // if (gnss->gpsFix == 3) snprintf(raw, sizeof(raw), "%d", 230);
    else                   snprintf(raw, sizeof(raw), "--");
    AL_Draw_TextField(&tf_hs, HS_FONT, raw);


    bool has_target = (cfg->al_target_lat != 0 || cfg->al_target_lon != 0);
    if (has_target && gnss->gpsFix == 3) {
        float dev = CrossTrackMeters(s_lane_start_lat, s_lane_start_lon,
                                     s_lane_ext_lat, s_lane_ext_lon,
                                     gnss->lat, gnss->lon);

        /* TEST: cycle through all 7 LaneArrow states every 5 s */
        // static const float k_test_devs[] = { 150.0f, 75.0f, 30.0f, 0.0f, -30.0f, -75.0f, -150.0f };
        // float dev = k_test_devs[(gnss->iTOW / 2000) % 7];

        AL_Draw_TextField(&tf_lane, LANE_FONT, LaneArrow(dev));
    } else {
        AL_Draw_TextField(&tf_lane, LANE_FONT, "");
    }
}

static void RenderRunScore(const FS_GNSS_Data_t *gnss, const FS_Config_Data_t *cfg)
{
    enum {
        TIME_X = 15,   TIME_Y = 60,  TIME_FONT = 2,   /* elapsed time  "99.9s" */
        SPD_X  = 15,   SPD_Y  = 95,  SPD_FONT  = 2,   /* average speed "999.9" */
        DIST_X = 15,   DIST_Y = 130, DIST_FONT = 2,   /* distance      "9999m" */
        LBL_X  = 130, LBL_Y  = 200, LBL_FONT  = 1,   /* "DZ" arrow label */
        ARROW_LX = 90, ARROW_LY = 70,
    };
    static AL_TextField_t tf_time = AL_TEXTFIELD_INIT(AL_TX(TIME_X),   TIME_Y);
    static AL_TextField_t tf_spd  = AL_TEXTFIELD_INIT(AL_TX(SPD_X),   SPD_Y);
    static AL_TextField_t tf_dist = AL_TEXTFIELD_INIT(AL_TX(DIST_X),   DIST_Y);
    static AL_TextField_t tf_lbl  = AL_TEXTFIELD_INIT(AL_TX(LBL_X), LBL_Y);

    char raw[AL_TEXTFIELD_MAXLEN];
    if (s_score_valid) {
        snprintf(raw, sizeof(raw), "T: %.1fs", (double)s_score_time_s);
        AL_Draw_TextField(&tf_time, TIME_FONT, raw);
        snprintf(raw, sizeof(raw), "S: %.1f",  (double)s_score_speed_kmh);
        AL_Draw_TextField(&tf_spd,  SPD_FONT,  raw);
        snprintf(raw, sizeof(raw), "D: %.1fm", (double)s_score_dist_m);
        AL_Draw_TextField(&tf_dist, DIST_FONT, raw);
    } else {
        AL_Draw_TextField(&tf_time, TIME_FONT, "T: --");
        AL_Draw_TextField(&tf_spd,  SPD_FONT,  "S: --");
        AL_Draw_TextField(&tf_dist, DIST_FONT, "D: --");
    }

    bool has_dz = (cfg->lat != 0 || cfg->lon != 0);
    AL_Arrow_Erase(&s_dz_arrow);
    if (has_dz && gnss->gpsFix == 3) {
        float angle = RelativeBearing(gnss->lat, gnss->lon, cfg->lat, cfg->lon,
                                       gnss->heading);
        AL_Arrow_Draw(&s_dz_arrow, ARROW_LX, ARROW_LY, angle);
        AL_Draw_TextField(&tf_lbl, LBL_FONT, "DZ");
    } else {
        AL_Draw_TextField(&tf_lbl, LBL_FONT, "");
    }
}

static void RenderHomeNav(const FS_GNSS_Data_t *gnss, const FS_Config_Data_t *cfg)
{
    enum {
        ALT_X  = 25,  ALT_Y  = 120,  ALT_FONT  = 3,   /* altitude AGL */
        DIST_X = 25,  DIST_Y = 60,  DIST_FONT = 2,   /* distance to DZ */
        LBL_X  = 130, LBL_Y  = 200, LBL_FONT  = 1,   /* "DZ" arrow label */
        ARROW_LX = 90, ARROW_LY = 70,
    };
    static AL_TextField_t tf_alt  = AL_TEXTFIELD_INIT(AL_TX(ALT_X),   ALT_Y);
    static AL_TextField_t tf_dist = AL_TEXTFIELD_INIT(AL_TX(DIST_X),   DIST_Y);
    static AL_TextField_t tf_lbl  = AL_TEXTFIELD_INIT(AL_TX(LBL_X),  LBL_Y);

    char raw[16];
    int32_t alt_agl_m = (gnss->hMSL - cfg->dz_elev) / 1000;
    if (gnss->gpsFix == 3) snprintf(raw, sizeof(raw), "Alt: %ldm", (long)alt_agl_m);
    else                   snprintf(raw, sizeof(raw), "Alt: --");
    AL_Draw_TextField(&tf_alt, ALT_FONT, raw);

    bool has_dz = (cfg->lat != 0 || cfg->lon != 0);
    if (gnss->gpsFix == 3 && has_dz) {
        int32_t dist_m = (int32_t)calcDistance(gnss->lat, gnss->lon, cfg->lat, cfg->lon);
        snprintf(raw, sizeof(raw), "DZ: %ldm", (long)dist_m);
    } else {
        snprintf(raw, sizeof(raw), "DZ: --");
    }
    AL_Draw_TextField(&tf_dist, DIST_FONT, raw);

    AL_Arrow_Erase(&s_dz_arrow);
    if (has_dz && gnss->gpsFix == 3) {
        float angle = RelativeBearing(gnss->lat, gnss->lon, cfg->lat, cfg->lon,
                                       gnss->heading);
        AL_Arrow_Draw(&s_dz_arrow, ARROW_LX, ARROW_LY, angle);
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
    s_phase          = COMP_PHASE_IDLE;
    s_render_phase   = COMP_PHASE_IDLE;

    s_ff_count       = 0;
    memset(s_ff_window, 0, sizeof(s_ff_window));

    s_freefall_itow  = 0;
    s_cr_start_itow  = 0;
    s_run_score_itow = 0;

    s_lane_start_lat = 0;
    s_lane_start_lon = 0;
    s_lane_ext_lat   = 0;
    s_lane_ext_lon   = 0

    s_lane_start_lat  = 0;
    s_lane_start_lon  = 0;

    s_pt_a_valid     = false;
    s_pt_b_valid     = false;
    s_score_valid    = false;

    s_prev_alt_agl_m = 0;
    s_prev_lat       = 0;
    s_prev_lon       = 0;
    s_prev_itow      = 0;

    s_velD_head      = 0;
    s_velD_count     = 0;
    memset(s_velD_ring, 0, sizeof(s_velD_ring));

    s_target_arrow.valid = false;
    s_dz_arrow.valid     = false;
}

FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode1_Setup(void)
{
    /* Direct-draw mode — no BLE objects to set up */
    return FS_AL_SETUP_DONE;
}

void FS_ActiveLook_Mode1_Update(void)
{
    const FS_GNSS_Data_t   *gnss   = FS_GNSS_GetData();
    const FS_Config_Data_t *cfg    = FS_Config_Get();
    const FS_VBAT_Data_t   *vbat   = FS_VBAT_GetData();
    uint8_t                 al_bat = FS_ActiveLook_Client_GetBatteryLevel();

    bool    has_gps  = (gnss->gpsFix == 3);
    float   velD_ms  = (float)gnss->velD / 1000.0f;
    int32_t alt_agl_m = (gnss->hMSL - cfg->dz_elev) / 1000;

    /* ==================================================================
       Phase transition logic
       ================================================================== */

    switch (s_phase)
    {
    /* ---- IDLE -------------------------------------------------------- */
    case COMP_PHASE_IDLE:
#if (DEBUG_FORCE_PHASE) >= 0
        if (has_gps) {
            s_freefall_itow = gnss->iTOW;
            s_phase = (CompetitionPhase_t)(DEBUG_FORCE_PHASE);
        }
#else
        if (has_gps && alt_agl_m >= AIRPLANE_ENTER_AGL_M)
            s_phase = COMP_PHASE_AIRPLANE;
#endif
        break;

    /* ---- AIRPLANE ---------------------------------------------------- */
    case COMP_PHASE_AIRPLANE:
        if (alt_agl_m < AIRPLANE_EXIT_AGL_M) {
            /* Descended without jumping — back to ground */
            s_phase = COMP_PHASE_IDLE;
            s_ff_count = 0;
            memset(s_ff_window, 0, sizeof(s_ff_window));
        } else if (has_gps) {
            AddSampleToWindow(gnss);
            if (DetectFreefallExit()) {
                s_freefall_itow = gnss->iTOW;
                s_phase = COMP_PHASE_FREEFALL;
            }
        }
        break;

    /* ---- FREEFALL ---------------------------------------------------- */
    case COMP_PHASE_FREEFALL:
        if (has_gps && (gnss->iTOW - s_freefall_itow) >= FREEFALL_LOCK_TIME_MS) {
            /* Lock the run lane at current position */
            s_lane_start_lat = gnss->lat;
            s_lane_start_lon = gnss->lon;
            ExtendLane(gnss->lat, gnss->lon,
                       cfg->al_target_lat, cfg->al_target_lon,
                       &s_lane_ext_lat, &s_lane_ext_lon);
            /* Initialise score capture tracking */
            s_pt_a_valid     = false;
            s_pt_b_valid     = false;
            s_score_valid    = false;
            s_prev_alt_agl_m = alt_agl_m;
            s_prev_lat       = gnss->lat;
            s_prev_lon       = gnss->lon;
            s_prev_itow      = gnss->iTOW;
            s_cr_start_itow  = gnss->iTOW;
            s_phase          = COMP_PHASE_COMPETITION_RUN;
        }
        break;

    /* ---- COMPETITION_RUN --------------------------------------------- */
    case COMP_PHASE_COMPETITION_RUN:
        if (has_gps) {
            /* ---- Score capture: closest sample to 2500 m AGL (point A) */
            if (!s_pt_a_valid &&
                s_prev_alt_agl_m >= COMP_SCORE_ALT_A_M && alt_agl_m < COMP_SCORE_ALT_A_M)
            {
                int32_t d_prev = s_prev_alt_agl_m - COMP_SCORE_ALT_A_M;
                int32_t d_curr = COMP_SCORE_ALT_A_M - alt_agl_m;
                if (d_prev <= d_curr) {
                    s_pt_a_lat = s_prev_lat; s_pt_a_lon = s_prev_lon;
                    s_pt_a_itow = s_prev_itow;
                } else {
                    s_pt_a_lat = gnss->lat;  s_pt_a_lon = gnss->lon;
                    s_pt_a_itow = gnss->iTOW;
                }
                s_pt_a_valid = true;
            }

            /* ---- Score capture: closest sample to 1500 m AGL (point B) */
            if (s_pt_a_valid && !s_pt_b_valid &&
                s_prev_alt_agl_m >= COMP_SCORE_ALT_B_M && alt_agl_m < COMP_SCORE_ALT_B_M)
            {
                int32_t d_prev = s_prev_alt_agl_m - COMP_SCORE_ALT_B_M;
                int32_t d_curr = COMP_SCORE_ALT_B_M - alt_agl_m;
                if (d_prev <= d_curr) {
                    s_pt_b_lat = s_prev_lat; s_pt_b_lon = s_prev_lon;
                    s_pt_b_itow = s_prev_itow;
                } else {
                    s_pt_b_lat = gnss->lat;  s_pt_b_lon = gnss->lon;
                    s_pt_b_itow = gnss->iTOW;
                }
                s_pt_b_valid = true;
            }

            /* Update previous sample */
            s_prev_alt_agl_m = alt_agl_m;
            s_prev_lat  = gnss->lat;
            s_prev_lon  = gnss->lon;
            s_prev_itow = gnss->iTOW;
        }

        /* ---- Transition: below 1500 m AGL and under canopy */
        if (alt_agl_m < COMP_SCORE_ALT_B_M && velD_ms < CANOPY_VELDOWN_MS) {
            /* Finalise score */
            if (s_pt_a_valid && s_pt_b_valid && s_pt_b_itow > s_pt_a_itow) {
                s_score_time_s = (float)(s_pt_b_itow - s_pt_a_itow) / 1000.0f;
                if (s_score_time_s > 0.1f) {
                    s_score_dist_m    = (float)calcDistance(s_pt_a_lat, s_pt_a_lon,
                                                             s_pt_b_lat, s_pt_b_lon);
                    s_score_speed_kmh = (s_score_dist_m / s_score_time_s) * 3.6f;
                    s_score_valid     = true;
                }
            }
            s_run_score_itow = gnss->iTOW;
            s_phase = COMP_PHASE_RUN_SCORE;
        }
        break;

    /* ---- RUN_SCORE --------------------------------------------------- */
    case COMP_PHASE_RUN_SCORE:
        if ((gnss->iTOW - s_run_score_itow) >= RUN_SCORE_DISPLAY_DURATION_MS)
            s_phase = COMP_PHASE_HOME_NAV;
        break;

    /* ---- HOME_NAV ---------------------------------------------------- */
    case COMP_PHASE_HOME_NAV:
        s_velD_ring[s_velD_head] = velD_ms;
        s_velD_head = (s_velD_head + 1) % HOME_NAV_RING_SIZE;
        if (s_velD_count < HOME_NAV_RING_SIZE) s_velD_count++;
        /* Transition to IDLE once landed (average descent < 3 m/s) */
        if (s_velD_count >= HOME_NAV_RING_SIZE && AverageVelD() < HOME_NAV_EXIT_VELDOWN_MS)
            s_phase = COMP_PHASE_IDLE;
        break;
    }

    /* ==================================================================
       Clear screen on phase change (one-time blink on transition)
       ================================================================== */

    if (s_phase != s_render_phase) {
        OnPhaseEnter();
        s_render_phase = s_phase;
    }

    /* ==================================================================
       Render current phase
       ================================================================== */

    switch (s_phase)
    {
    case COMP_PHASE_IDLE:
        DrawStatusBar(gnss, vbat, al_bat);
        RenderIdle(gnss, cfg);
        break;
    case COMP_PHASE_AIRPLANE:
        DrawStatusBar(gnss, vbat, al_bat);
        RenderAirplane(gnss, cfg);
        break;
    case COMP_PHASE_FREEFALL:        RenderFreefall(gnss, cfg);       break;
    case COMP_PHASE_COMPETITION_RUN: RenderCompetitionRun(gnss, cfg); break;
    case COMP_PHASE_RUN_SCORE:       RenderRunScore(gnss, cfg);       break;
    case COMP_PHASE_HOME_NAV:        RenderHomeNav(gnss, cfg);        break;
    }
}
