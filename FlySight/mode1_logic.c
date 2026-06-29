/***************************************************************************
**  FlySight 2 firmware — Shared competition mode flight logic            **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
**                                                                        **
**  Extracted from activelook_mode1.c so that ActiveLook and Vuzix        **
**  renderers share one state machine.  Only rendering (text / graphics)  **
**  differs between devices.                                              **
****************************************************************************/

/*
 * Debug: force the state machine to a specific phase on the first GPS fix.
 * Set DEBUG_FORCE_PHASE to a CompetitionPhase_t value, or -1 to disable.
 */
#define DEBUG_FORCE_PHASE  -1

#include "mode1_logic.h"
#include "config.h"
#include "gnss.h"
#include "nav.h"
#include "app_common.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
   Freefall detection constants
   -------------------------------------------------------------------------- */

#define FREEFALL_WINDOW_SIZE         4
#define A_GRAVITY_MS2                9.81f
#define EXIT_VELOCITY_THRESHOLD_MS   A_GRAVITY_MS2
#define ACCEL_MIN_THRESHOLD_MS2      (A_GRAVITY_MS2 / 5.0f)
#define GPS_VACC_MAX_METERS          10.0f

/* --------------------------------------------------------------------------
   Phase-transition thresholds
   -------------------------------------------------------------------------- */

#define AIRPLANE_ENTER_AGL_M              100
#define AIRPLANE_EXIT_AGL_M                50
#define FREEFALL_LOCK_TIME_MS            9000
#define CANOPY_VELDOWN_MS               10.0f
#define COMP_SCORE_ALT_A_M              2500
#define COMP_SCORE_ALT_B_M              1500
#define RUN_SCORE_DISPLAY_DURATION_MS  15000
#define HOME_NAV_EXIT_VELDOWN_MS        3.0f
#define HOME_NAV_RING_SIZE               10
#define GPS_RECOVERY_VELDOWN_MS         5.56f
#define GPS_LOSS_RECOVERY_MS           10000u

/* --------------------------------------------------------------------------
   Internal types
   -------------------------------------------------------------------------- */

typedef struct {
    uint32_t itow_ms;
    float    vel_down_ms;
    float    vert_acc_m;
    float    accel_ms2;
    int32_t  lat;
    int32_t  lon;
    int32_t  hMSL_mm;
} FreefallSample_t;

/* --------------------------------------------------------------------------
   Module-level state
   -------------------------------------------------------------------------- */

static CompetitionPhase_t  s_phase           = COMP_PHASE_IDLE;
static FreefallSample_t    s_ff_window[FREEFALL_WINDOW_SIZE];
static uint8_t             s_ff_count        = 0;

static uint32_t            s_freefall_itow   = 0;
static uint32_t            s_cr_start_itow   = 0;
static uint32_t            s_run_score_itow  = 0;

static int32_t             s_lane_start_lat  = 0;
static int32_t             s_lane_start_lon  = 0;
static int32_t             s_lane_ext_lat    = 0;
static int32_t             s_lane_ext_lon    = 0;
static bool                s_lane_valid      = false;

static bool                s_pt_a_valid      = false;
static bool                s_pt_b_valid      = false;
static int32_t             s_pt_a_lat        = 0;
static int32_t             s_pt_a_lon        = 0;
static uint32_t            s_pt_a_itow       = 0;
static int32_t             s_pt_b_lat        = 0;
static int32_t             s_pt_b_lon        = 0;
static uint32_t            s_pt_b_itow       = 0;

static int32_t             s_prev_lat        = 0;
static int32_t             s_prev_lon        = 0;
static uint32_t            s_prev_itow       = 0;
static int32_t             s_prev_alt_agl_m  = 0;
static int32_t             s_prev_hMSL       = 0;

static float               s_score_time_s    = 0.0f;
static float               s_score_speed_kmh = 0.0f;
static float               s_score_dist_m    = 0.0f;
static bool                s_score_valid     = false;

static float               s_velD_ring[HOME_NAV_RING_SIZE];
static uint8_t             s_velD_head       = 0;
static uint8_t             s_velD_count      = 0;

static uint32_t            s_last_gps_tick   = 0;

static Mode1_Data_t        s_data;

/* --------------------------------------------------------------------------
   Freefall detection helpers  (ported from freefall.js / FlyScope)
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

    FreefallSample_t *s = &s_ff_window[FREEFALL_WINDOW_SIZE - 1];
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

static void ExtendLane(int32_t start_lat, int32_t start_lon,
                       int32_t tgt_lat,   int32_t tgt_lon,
                       int32_t *ext_lat,  int32_t *ext_lon)
{
    const double R   = 6371100.0;
    const double EXT = 5000.0;
    const double S   = 1e-7 * M_PI / 180.0;

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

static float CrossTrackMeters(int32_t lane_lat, int32_t lane_lon,
                               int32_t tgt_lat,  int32_t tgt_lon,
                               int32_t cur_lat,  int32_t cur_lon)
{
    float b_to_tgt = BearingDeg(lane_lat, lane_lon, tgt_lat, tgt_lon);
    float b_to_cur = BearingDeg(lane_lat, lane_lon, cur_lat, cur_lon);
    float dist     = (float)calcDistance(lane_lat, lane_lon, cur_lat, cur_lon);
    return dist * sinf((b_to_cur - b_to_tgt) * (float)M_PI / 180.0f);
}

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

static float AverageVelD(void)
{
    if (s_velD_count == 0) return 0.0f;
    float sum = 0.0f;
    for (uint8_t i = 0; i < s_velD_count; i++)
        sum += s_velD_ring[i];
    return sum / (float)s_velD_count;
}

/* --------------------------------------------------------------------------
   Public API
   -------------------------------------------------------------------------- */

void Mode1_Logic_Init(void)
{
    s_phase           = COMP_PHASE_IDLE;
    s_ff_count        = 0;
    memset(s_ff_window, 0, sizeof(s_ff_window));

    s_freefall_itow   = 0;
    s_cr_start_itow   = 0;
    s_run_score_itow  = 0;

    s_lane_start_lat  = 0;
    s_lane_start_lon  = 0;
    s_lane_ext_lat    = 0;
    s_lane_ext_lon    = 0;
    s_lane_valid      = false;

    s_pt_a_valid      = false;
    s_pt_b_valid      = false;
    s_pt_a_lat        = 0;
    s_pt_a_lon        = 0;
    s_pt_a_itow       = 0;
    s_pt_b_lat        = 0;
    s_pt_b_lon        = 0;
    s_pt_b_itow       = 0;

    s_prev_alt_agl_m  = 0;
    s_prev_hMSL       = 0;
    s_prev_lat        = 0;
    s_prev_lon        = 0;
    s_prev_itow       = 0;

    s_score_time_s    = 0.0f;
    s_score_speed_kmh = 0.0f;
    s_score_dist_m    = 0.0f;
    s_score_valid     = false;

    s_velD_head       = 0;
    s_velD_count      = 0;
    memset(s_velD_ring, 0, sizeof(s_velD_ring));

    s_last_gps_tick   = HAL_GetTick();

    memset(&s_data, 0, sizeof(s_data));
}

bool Mode1_Logic_Update(void)
{
    const FS_GNSS_Data_t   *gnss = FS_GNSS_GetData();
    const FS_Config_Data_t *cfg  = FS_Config_Get();

    CompetitionPhase_t prev_phase = s_phase;

    bool    has_gps   = (gnss->gpsFix == 3);
    float   velD_ms   = (float)gnss->velD / 1000.0f;
    int32_t alt_agl_m = (gnss->hMSL - cfg->dz_elev) / 1000;

    bool gps_loss_recovery = false;
    if (has_gps) {
        if ((HAL_GetTick() - s_last_gps_tick) >= GPS_LOSS_RECOVERY_MS)
            gps_loss_recovery = true;
        s_last_gps_tick = HAL_GetTick();
    }

    /* ==================================================================
       Phase transition logic
       ================================================================== */

    switch (s_phase)
    {
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

    case COMP_PHASE_AIRPLANE:
        if (gps_loss_recovery && velD_ms > GPS_RECOVERY_VELDOWN_MS && alt_agl_m > COMP_SCORE_ALT_B_M) {
            s_lane_start_lat = gnss->lat;
            s_lane_start_lon = gnss->lon;
            s_lane_ext_lat   = 0;
            s_lane_ext_lon   = 0;
            s_lane_valid     = false;
            s_pt_a_valid     = false;
            s_pt_b_valid     = false;
            s_score_valid    = false;
            s_prev_alt_agl_m = alt_agl_m;
            s_prev_hMSL      = gnss->hMSL;
            s_prev_lat       = gnss->lat;
            s_prev_lon       = gnss->lon;
            s_prev_itow      = gnss->iTOW;
            s_cr_start_itow  = gnss->iTOW;
            s_phase          = COMP_PHASE_COMPETITION_RUN;
        } else if (alt_agl_m < AIRPLANE_EXIT_AGL_M) {
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

    case COMP_PHASE_FREEFALL:
        if (gps_loss_recovery && velD_ms > GPS_RECOVERY_VELDOWN_MS && alt_agl_m > COMP_SCORE_ALT_B_M) {
            s_lane_start_lat = gnss->lat;
            s_lane_start_lon = gnss->lon;
            s_lane_ext_lat   = 0;
            s_lane_ext_lon   = 0;
            s_lane_valid     = false;
            s_pt_a_valid     = false;
            s_pt_b_valid     = false;
            s_score_valid    = false;
            s_prev_alt_agl_m = alt_agl_m;
            s_prev_hMSL      = gnss->hMSL;
            s_prev_lat       = gnss->lat;
            s_prev_lon       = gnss->lon;
            s_prev_itow      = gnss->iTOW;
            s_cr_start_itow  = gnss->iTOW;
            s_phase          = COMP_PHASE_COMPETITION_RUN;
        } else if (has_gps && (gnss->iTOW - s_freefall_itow) >= FREEFALL_LOCK_TIME_MS) {
            s_lane_start_lat = gnss->lat;
            s_lane_start_lon = gnss->lon;
            bool has_target  = (cfg->target_lat != 0 || cfg->target_lon != 0);
            if (has_target) {
                ExtendLane(gnss->lat, gnss->lon,
                           cfg->target_lat, cfg->target_lon,
                           &s_lane_ext_lat, &s_lane_ext_lon);
                s_lane_valid = true;
            } else {
                s_lane_ext_lat = 0;
                s_lane_ext_lon = 0;
                s_lane_valid   = false;
            }
            s_pt_a_valid     = false;
            s_pt_b_valid     = false;
            s_score_valid    = false;
            s_prev_alt_agl_m = alt_agl_m;
            s_prev_hMSL      = gnss->hMSL;
            s_prev_lat       = gnss->lat;
            s_prev_lon       = gnss->lon;
            s_prev_itow      = gnss->iTOW;
            s_cr_start_itow  = gnss->iTOW;
            s_phase          = COMP_PHASE_COMPETITION_RUN;
        }
        break;

    case COMP_PHASE_COMPETITION_RUN:
        if (has_gps) {
            if (!s_pt_a_valid &&
                s_prev_alt_agl_m >= COMP_SCORE_ALT_A_M && alt_agl_m < COMP_SCORE_ALT_A_M)
            {
                int32_t target_hMSL = COMP_SCORE_ALT_A_M * 1000 + cfg->dz_elev;
                float span = (float)(s_prev_hMSL - gnss->hMSL);
                float t    = (span > 0.0f)
                           ? (float)(s_prev_hMSL - target_hMSL) / span
                           : 0.0f;
                s_pt_a_lat  = s_prev_lat  + (int32_t)(t * (float)(gnss->lat  - s_prev_lat));
                s_pt_a_lon  = s_prev_lon  + (int32_t)(t * (float)(gnss->lon  - s_prev_lon));
                s_pt_a_itow = s_prev_itow + (uint32_t)(t * (float)(gnss->iTOW - s_prev_itow));
                s_pt_a_valid = true;
            }

            if (s_pt_a_valid && !s_pt_b_valid &&
                s_prev_alt_agl_m >= COMP_SCORE_ALT_B_M && alt_agl_m < COMP_SCORE_ALT_B_M)
            {
                int32_t target_hMSL = COMP_SCORE_ALT_B_M * 1000 + cfg->dz_elev;
                float span = (float)(s_prev_hMSL - gnss->hMSL);
                float t    = (span > 0.0f)
                           ? (float)(s_prev_hMSL - target_hMSL) / span
                           : 0.0f;
                s_pt_b_lat  = s_prev_lat  + (int32_t)(t * (float)(gnss->lat  - s_prev_lat));
                s_pt_b_lon  = s_prev_lon  + (int32_t)(t * (float)(gnss->lon  - s_prev_lon));
                s_pt_b_itow = s_prev_itow + (uint32_t)(t * (float)(gnss->iTOW - s_prev_itow));
                s_pt_b_valid = true;
            }

            s_prev_alt_agl_m = alt_agl_m;
            s_prev_hMSL      = gnss->hMSL;
            s_prev_lat       = gnss->lat;
            s_prev_lon       = gnss->lon;
            s_prev_itow      = gnss->iTOW;
        }

        if (alt_agl_m < COMP_SCORE_ALT_B_M && velD_ms < CANOPY_VELDOWN_MS) {
            if (s_pt_a_valid && s_pt_b_valid && s_pt_b_itow > s_pt_a_itow) {
                s_score_time_s = (float)(s_pt_b_itow - s_pt_a_itow) / 1000.0f;
                if (s_score_time_s > 0.1f) {
                    s_score_dist_m    = (float)calcDistance(s_pt_a_lat, s_pt_a_lon,
                                                             s_pt_b_lat, s_pt_b_lon);
                    s_score_speed_kmh = (s_score_dist_m / s_score_time_s) * 3.6f;
                    s_score_valid     = true;
                }
                s_run_score_itow = gnss->iTOW;
                s_phase = COMP_PHASE_RUN_SCORE;
            } else {
                s_phase = COMP_PHASE_HOME_NAV;
            }
        }
        break;

    case COMP_PHASE_RUN_SCORE:
        if ((gnss->iTOW - s_run_score_itow) >= RUN_SCORE_DISPLAY_DURATION_MS)
            s_phase = COMP_PHASE_HOME_NAV;
        break;

    case COMP_PHASE_HOME_NAV:
        s_velD_ring[s_velD_head] = velD_ms;
        s_velD_head = (s_velD_head + 1) % HOME_NAV_RING_SIZE;
        if (s_velD_count < HOME_NAV_RING_SIZE) s_velD_count++;
        if (s_velD_count >= HOME_NAV_RING_SIZE && AverageVelD() < HOME_NAV_EXIT_VELDOWN_MS)
            s_phase = COMP_PHASE_IDLE;
        break;
    }

    /* ==================================================================
       Populate output struct
       ================================================================== */

    bool has_target = (cfg->target_lat != 0 || cfg->target_lon != 0);
    bool has_dz     = (cfg->lat != 0 || cfg->lon != 0);

    s_data.phase      = (uint8_t)s_phase;
    s_data.has_gps    = has_gps;
    s_data.alt_agl_m  = alt_agl_m;
    s_data.hMSL_m     = gnss->hMSL / 1000;
    s_data.dz_elev_m  = cfg->dz_elev / 1000;
    s_data.gspeed_kmh = (int32_t)((float)gnss->gSpeed * 0.036f);
    s_data.velD_ms    = velD_ms;
    s_data.glide_ratio = (has_gps && gnss->velD > 0)
                         ? (float)gnss->gSpeed * 10.0f / (float)gnss->velD
                         : 0.0f;
    s_data.num_sv     = (uint8_t)gnss->numSV;

    uint32_t elapsed_ff = gnss->iTOW - s_freefall_itow;
    int cntdn = (int)(FREEFALL_LOCK_TIME_MS / 1000) - (int)(elapsed_ff / 1000);
    s_data.freefall_cntdn_s = (cntdn < 0) ? 0 : cntdn;

    s_data.has_target        = has_target;
    s_data.rel_to_target_deg = 0.0f;
    if (has_target && has_gps)
        s_data.rel_to_target_deg = RelativeBearing(gnss->lat, gnss->lon,
                                                    cfg->target_lat, cfg->target_lon,
                                                    gnss->heading);

    s_data.has_dz        = has_dz;
    s_data.rel_to_dz_deg = 0.0f;
    s_data.dz_dist_m     = 0;
    if (has_dz && has_gps) {
        s_data.rel_to_dz_deg = RelativeBearing(gnss->lat, gnss->lon,
                                                cfg->lat, cfg->lon,
                                                gnss->heading);
        s_data.dz_dist_m = (int32_t)calcDistance(gnss->lat, gnss->lon, cfg->lat, cfg->lon);
    }

    s_data.lane_valid = s_lane_valid;
    s_data.lane_dev_m = 0.0f;
    if (s_lane_valid && has_gps)
        s_data.lane_dev_m = CrossTrackMeters(s_lane_start_lat, s_lane_start_lon,
                                             s_lane_ext_lat, s_lane_ext_lon,
                                             gnss->lat, gnss->lon);

    s_data.score_valid     = s_score_valid;
    s_data.score_time_s    = s_score_time_s;
    s_data.score_speed_kmh = s_score_speed_kmh;
    s_data.score_dist_m    = s_score_dist_m;

    return (s_phase != prev_phase);
}

const Mode1_Data_t *Mode1_Logic_GetData(void)
{
    return &s_data;
}
