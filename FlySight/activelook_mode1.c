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
 * Competition Mode (al_mode = 2)
 *
 * Aircraft exit is detected via the same linear-regression algorithm used
 * in FlyScope: a sliding window of GNSS samples is maintained, the slope
 * of velD vs. time (= vertical acceleration) is computed by least-squares
 * regression, and exit is declared when:
 *   1. velD crosses 1g (9.81 m/s) between the two most-recent samples.
 *   2. GPS vertical accuracy at the crossing is <= 10 m.
 *   3. The vertical acceleration at the crossing is >= g/5 (1.96 m/s²).
 *
 * The exit lat/lon is interpolated to the exact crossing point.
 *
 * After exit, the display shows lane deviation from the exit→target
 * corridor and the relative bearing to the configured target.
 *
 * Fixed display lines (not driven by config al_lines):
 *   Status bar : "A:XX%  F:XX%  N:XX"  (AL battery, FS battery, GPS sats)
 *   Line 1     : phase    – "IDLE" / "FREEFALL" / "CANOPY"
 *   Line 2     : lane     – "<<<" / "<<" / "<" / "===" / ">" / ">>" / ">>>"
 *   Line 3     : heading  – "<<" / "<" / "^" / ">" / ">>"
 *   Line 4     : alt AGL  – metres above configured DZ elevation
 *
 * Layout IDs 20–24 and page 11 are reserved for this mode to avoid
 * conflicting with Mode 0 (layouts 10–14, page 10).
 */

/* Set to 1 before building to enter COMP_PHASE_FREEFALL immediately using
 * the current GPS position as the exit point.  For ground testing only.
 * Restore to 0 before any real flight. */
#define FORCE_FREEFALL_FOR_TESTING    1

#include "activelook_mode1.h"
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
   Layout / page IDs
   -------------------------------------------------------------------------- */

#define COMP_LAYOUT_STATUS_BAR   20
#define COMP_LAYOUT_PHASE        21
#define COMP_LAYOUT_LANE         22
#define COMP_LAYOUT_HEADING      23
#define COMP_LAYOUT_ALTITUDE     24
#define COMP_PAGE_ID             11

/* --------------------------------------------------------------------------
   Freefall detection constants  (ported from freefall.js / FlyScope)
   -------------------------------------------------------------------------- */

#define FREEFALL_WINDOW_SIZE          4       /* sliding window depth (samples)  */
#define A_GRAVITY_MS2                 9.81f   /* m/s²                            */

/* Exit is declared when velD (m/s) crosses this value between two samples */
#define EXIT_VELOCITY_THRESHOLD_MS    A_GRAVITY_MS2

/* Vertical acceleration must exceed this to confirm a real aircraft exit */
#define ACCEL_MIN_THRESHOLD_MS2       (A_GRAVITY_MS2 / 5.0f)   /* 1.96 m/s²   */

/* GPS vertical accuracy must be within this at the crossing point */
#define GPS_VACC_MAX_METERS           10.0f

/* velD below this after exit signals canopy deployment (m/s) */
#define CANOPY_VELDOWN_THRESHOLD_MS   10.0f

/* hMSL rise above exit that triggers reset to IDLE (mm) */
#define BACK_IN_AIRCRAFT_ALTITUDE_MM  200000  /* 200 m */

/* --------------------------------------------------------------------------
   Lane and heading arrow thresholds
   -------------------------------------------------------------------------- */

/* Cross-track deviation thresholds (metres) */
#define LANE_THRESHOLD_MINOR_M        10.0f
#define LANE_THRESHOLD_MODERATE_M     50.0f
#define LANE_THRESHOLD_MAJOR_M       100.0f

/* Relative bearing thresholds (degrees) */
#define HDG_THRESHOLD_MINOR_DEG       15.0f
#define HDG_THRESHOLD_MAJOR_DEG       45.0f

/* --------------------------------------------------------------------------
   Freefall detection: sliding-window sample
   -------------------------------------------------------------------------- */

typedef struct {
    uint32_t itow_ms;       /* GPS time of week (ms) – used as time axis        */
    float    vel_down_ms;   /* velD in m/s  (positive = descending)             */
    float    vert_acc_m;    /* vAcc in m    (GPS vertical accuracy estimate)     */
    float    accel_ms2;     /* slope of velD vs time computed at this sample     */
    int32_t  lat;           /* 1e-7 degrees                                      */
    int32_t  lon;           /* 1e-7 degrees                                      */
    int32_t  hMSL_mm;       /* height above MSL in mm                            */
} FreefallSample_t;

/* --------------------------------------------------------------------------
   Competition phase state machine
   -------------------------------------------------------------------------- */

typedef enum {
    COMP_PHASE_IDLE     = 0,   /* In aircraft, waiting for exit         */
    COMP_PHASE_FREEFALL = 1,   /* In freefall; exit point has been set  */
    COMP_PHASE_CANOPY   = 2,   /* Under canopy after deployment         */
} CompetitionPhase_t;

/* --------------------------------------------------------------------------
   Module-level state
   -------------------------------------------------------------------------- */

static int                 s_setup_step          = 0;
static CompetitionPhase_t  s_competition_phase   = COMP_PHASE_IDLE;
static int32_t             s_exit_lat            = 0;
static int32_t             s_exit_lon            = 0;
static int32_t             s_exit_altitude_mm    = 0;

static FreefallSample_t    s_window[FREEFALL_WINDOW_SIZE];
static uint8_t             s_window_count        = 0;

/* --------------------------------------------------------------------------
   BLE send helper
   -------------------------------------------------------------------------- */

static void SendBlePacket(const uint8_t *data, uint16_t length)
{
    FS_ActiveLook_Client_WriteWithoutResp(data, length);
}

/* --------------------------------------------------------------------------
   Packet builders  (mirrors mode0 pattern, uses layout IDs 20–24, page 11)
   -------------------------------------------------------------------------- */

static uint8_t BuildStatusBarLayout(uint8_t layout_id, uint8_t *packet_buf)
{
    uint8_t idx = 0;
    packet_buf[idx++] = 0xFF;
    packet_buf[idx++] = 0x60;           /* layoutSave */
    packet_buf[idx++] = 0x00;
    uint8_t length_field_pos = idx++;

    packet_buf[idx++] = layout_id;
    uint8_t subcmd_size_pos = idx++;

    packet_buf[idx++] = 0x00; packet_buf[idx++] = 0x00;
    packet_buf[idx++] = 0x00; packet_buf[idx++] = 0x01;
    packet_buf[idx++] = 0x30; packet_buf[idx++] = 0x28;
    packet_buf[idx++] = 15;   packet_buf[idx++] = 0;
    packet_buf[idx++] = 1;    packet_buf[idx++] = 1;
    packet_buf[idx++] = 1;    packet_buf[idx++] = 5;
    packet_buf[idx++] = 35;   packet_buf[idx++] = 4;
    packet_buf[idx++] = 1;

    packet_buf[subcmd_size_pos] = 0;
    packet_buf[idx++] = 0xAA;
    packet_buf[length_field_pos] = idx;
    return idx;
}

static uint8_t BuildDataLineLayout(uint8_t layout_id,
                                    const char *label_text,
                                    const char *unit_text,
                                    uint8_t *packet_buf)
{
    uint8_t idx = 0;
    packet_buf[idx++] = 0xFF;
    packet_buf[idx++] = 0x60;
    packet_buf[idx++] = 0x00;
    uint8_t length_field_pos = idx++;

    packet_buf[idx++] = layout_id;
    uint8_t subcmd_size_pos = idx++;

    packet_buf[idx++] = 0x00; packet_buf[idx++] = 0x00;
    packet_buf[idx++] = 0x00; packet_buf[idx++] = 0x01;
    packet_buf[idx++] = 0x30; packet_buf[idx++] = 0x28;
    packet_buf[idx++] = 15;   packet_buf[idx++] = 0;
    packet_buf[idx++] = 2;    packet_buf[idx++] = 1;
    packet_buf[idx++] = 0;    packet_buf[idx++] = 200;
    packet_buf[idx++] = 40;   packet_buf[idx++] = 4;
    packet_buf[idx++] = 1;

    uint8_t subcmd_buf[64];
    uint8_t subcmd_idx = 0;

    subcmd_buf[subcmd_idx++] = 0x03; subcmd_buf[subcmd_idx++] = 15;  /* color */
    subcmd_buf[subcmd_idx++] = 0x04; subcmd_buf[subcmd_idx++] = 1;   /* font  */

    subcmd_buf[subcmd_idx++] = 0x09;
    subcmd_buf[subcmd_idx++] = 1;  subcmd_buf[subcmd_idx++] = 5;
    subcmd_buf[subcmd_idx++] = 0;  subcmd_buf[subcmd_idx++] = 35;
    size_t label_len = strlen(label_text);
    subcmd_buf[subcmd_idx++] = (uint8_t)label_len;
    memcpy(&subcmd_buf[subcmd_idx], label_text, label_len);
    subcmd_idx += label_len;

    subcmd_buf[subcmd_idx++] = 0x04; subcmd_buf[subcmd_idx++] = 1;   /* font  */
    subcmd_buf[subcmd_idx++] = 0x09;
    subcmd_buf[subcmd_idx++] = 0;  subcmd_buf[subcmd_idx++] = 80;
    subcmd_buf[subcmd_idx++] = 0;  subcmd_buf[subcmd_idx++] = 35;
    size_t unit_len = strlen(unit_text);
    subcmd_buf[subcmd_idx++] = (uint8_t)unit_len;
    memcpy(&subcmd_buf[subcmd_idx], unit_text, unit_len);
    subcmd_idx += unit_len;

    packet_buf[subcmd_size_pos] = subcmd_idx;
    memcpy(&packet_buf[idx], subcmd_buf, subcmd_idx);
    idx += subcmd_idx;

    packet_buf[idx++] = 0xAA;
    packet_buf[length_field_pos] = idx;
    return idx;
}

static uint8_t BuildCompetitionPage(uint8_t page_id, uint8_t *packet_buf)
{
    uint8_t idx = 0;
    packet_buf[idx++] = 0xFF;
    packet_buf[idx++] = 0x80;           /* pageSave */
    packet_buf[idx++] = 0x00;
    uint8_t length_field_pos = idx++;

    packet_buf[idx++] = page_id;

    packet_buf[idx++] = COMP_LAYOUT_STATUS_BAR;
    packet_buf[idx++] = 0x00; packet_buf[idx++] = 0x00; packet_buf[idx++] = 178;

    const uint8_t data_layout_ids[4] = {
        COMP_LAYOUT_PHASE,
        COMP_LAYOUT_LANE,
        COMP_LAYOUT_HEADING,
        COMP_LAYOUT_ALTITUDE,
    };
    const uint8_t y_positions[4] = { 133, 93, 53, 13 };

    for (int line = 0; line < 4; line++) {
        packet_buf[idx++] = data_layout_ids[line];
        packet_buf[idx++] = 0x00;
        packet_buf[idx++] = 0x00;
        packet_buf[idx++] = y_positions[line];
    }

    packet_buf[idx++] = 0xAA;
    packet_buf[length_field_pos] = idx;
    return idx;
}

static uint8_t BuildPageUpdate(uint8_t page_id,
                                const char *status_bar_text,
                                const char *phase_text,
                                const char *lane_text,
                                const char *heading_text,
                                const char *altitude_text,
                                uint8_t *packet_buf)
{
    uint8_t idx = 0;
    packet_buf[idx++] = 0xFF;
    packet_buf[idx++] = 0x86;           /* pageClearAndDisplay */
    packet_buf[idx++] = 0x00;
    uint8_t length_field_pos = idx++;

    packet_buf[idx++] = page_id;

    const char *all_strings[5] = {
        status_bar_text, phase_text, lane_text, heading_text, altitude_text,
    };
    for (int i = 0; i < 5; i++) {
        size_t text_len = strlen(all_strings[i]);
        memcpy(&packet_buf[idx], all_strings[i], text_len);
        idx += text_len;
        packet_buf[idx++] = 0;
    }

    packet_buf[idx++] = 0xAA;
    packet_buf[length_field_pos] = idx;
    return idx;
}

/* --------------------------------------------------------------------------
   Freefall detection  (ported from freefall.js / FlyScope)
   -------------------------------------------------------------------------- */

/*
 * Compute the least-squares slope of velD vs. time across the full window.
 * Time is expressed in seconds relative to the oldest sample (s_window[0]).
 * Returns slope in m/s² (vertical acceleration).
 */
static float ComputeVelDSlope(void)
{
    float sum_t = 0.0f, sum_v = 0.0f, sum_tt = 0.0f, sum_tv = 0.0f;
    int   n     = FREEFALL_WINDOW_SIZE;

    for (int i = 0; i < n; i++) {
        float t = (float)(s_window[i].itow_ms - s_window[0].itow_ms) / 1000.0f;
        float v = s_window[i].vel_down_ms;
        sum_t  += t;
        sum_v  += v;
        sum_tt += t * t;
        sum_tv += t * v;
    }

    float denominator = sum_tt - sum_t * sum_t / (float)n;
    if (fabsf(denominator) < 1e-6f)
        return 0.0f;

    return (sum_tv - sum_t * sum_v / (float)n) / denominator;
}

/*
 * Add the current GNSS sample to the sliding window (oldest sample is dropped).
 * When the window is full, computes and stores the vertical acceleration slope
 * for the newest sample (used later for interpolated acceleration at exit).
 */
static void AddSampleToWindow(const FS_GNSS_Data_t *gnss)
{
    for (int i = 0; i < FREEFALL_WINDOW_SIZE - 1; i++)
        s_window[i] = s_window[i + 1];

    FreefallSample_t *newest = &s_window[FREEFALL_WINDOW_SIZE - 1];
    newest->itow_ms     = gnss->iTOW;
    newest->vel_down_ms = (float)gnss->velD / 1000.0f;
    newest->vert_acc_m  = (float)gnss->vAcc / 1000.0f;
    newest->accel_ms2   = 0.0f;
    newest->lat         = gnss->lat;
    newest->lon         = gnss->lon;
    newest->hMSL_mm     = gnss->hMSL;

    if (s_window_count < FREEFALL_WINDOW_SIZE)
        s_window_count++;

    if (s_window_count == FREEFALL_WINDOW_SIZE)
        newest->accel_ms2 = ComputeVelDSlope();
}

/*
 * Attempt to detect aircraft exit using the FlyScope algorithm.
 *
 * Requires the window to be full (FREEFALL_WINDOW_SIZE samples).
 * Returns true and populates exit coordinates if all three conditions pass:
 *   1. velD crosses EXIT_VELOCITY_THRESHOLD_MS between the last two samples.
 *   2. GPS vertical accuracy at the crossing <= GPS_VACC_MAX_METERS.
 *   3. Vertical acceleration at the crossing >= ACCEL_MIN_THRESHOLD_MS2.
 */
static bool DetectFreefallExit(int32_t *exit_lat_out,
                                 int32_t *exit_lon_out,
                                 int32_t *exit_alt_mm_out)
{
    if (s_window_count < FREEFALL_WINDOW_SIZE)
        return false;

    int prev = FREEFALL_WINDOW_SIZE - 2;
    int curr = FREEFALL_WINDOW_SIZE - 1;

    float vel_delta = s_window[curr].vel_down_ms - s_window[prev].vel_down_ms;
    if (fabsf(vel_delta) < 1e-3f)
        return false;

    /* Interpolation factor: fraction of the way from prev to curr where velD == g */
    float interp = (EXIT_VELOCITY_THRESHOLD_MS - s_window[prev].vel_down_ms) / vel_delta;
    if (interp < 0.0f || interp > 1.0f)
        return false;

    /* Condition 2: GPS vertical accuracy */
    float vert_acc_at_crossing = s_window[prev].vert_acc_m
                                 + interp * (s_window[curr].vert_acc_m - s_window[prev].vert_acc_m);
    if (vert_acc_at_crossing > GPS_VACC_MAX_METERS)
        return false;

    /* Condition 3: vertical acceleration */
    float accel_at_crossing = s_window[prev].accel_ms2
                              + interp * (s_window[curr].accel_ms2 - s_window[prev].accel_ms2);
    if (accel_at_crossing < ACCEL_MIN_THRESHOLD_MS2)
        return false;

    /* All conditions met: interpolate exit position */
    *exit_lat_out    = s_window[prev].lat
                       + (int32_t)(interp * (float)(s_window[curr].lat    - s_window[prev].lat));
    *exit_lon_out    = s_window[prev].lon
                       + (int32_t)(interp * (float)(s_window[curr].lon    - s_window[prev].lon));
    *exit_alt_mm_out = s_window[prev].hMSL_mm
                       + (int32_t)(interp * (float)(s_window[curr].hMSL_mm - s_window[prev].hMSL_mm));

    return true;
}

/* --------------------------------------------------------------------------
   Navigation helpers
   -------------------------------------------------------------------------- */

/*
 * Absolute compass bearing from point A to point B.
 * Returns degrees in range -180..+180, 0 = North, positive = clockwise.
 * Coordinates in 1e-7 degrees.
 */
static float AbsoluteBearingDeg(int32_t lat_a, int32_t lon_a,
                                  int32_t lat_b, int32_t lon_b)
{
    float delta_lat     = (float)(lat_b - lat_a);
    float lat_a_radians = (float)lat_a * 1e-7f * (float)M_PI / 180.0f;
    float delta_lon_adj = (float)(lon_b - lon_a) * cosf(lat_a_radians);
    return atan2f(delta_lon_adj, delta_lat) * 180.0f / (float)M_PI;
}

/*
 * Signed cross-track deviation in metres along the exit→target corridor.
 *
 * Formula: XTD = d(exit, current) × sin(bearing(exit→current) − bearing(exit→target))
 *
 * Positive = athlete is RIGHT of the lane → steer LEFT.
 * Negative = athlete is LEFT of the lane  → steer RIGHT.
 */
static float CrossTrackDeviationMeters(int32_t exit_lat,    int32_t exit_lon,
                                        int32_t target_lat,  int32_t target_lon,
                                        int32_t current_lat, int32_t current_lon)
{
    float bearing_exit_to_target  = AbsoluteBearingDeg(exit_lat, exit_lon,
                                                         target_lat, target_lon);
    float bearing_exit_to_current = AbsoluteBearingDeg(exit_lat, exit_lon,
                                                         current_lat, current_lon);
    float dist_exit_to_current    = (float)calcDistance(exit_lat, exit_lon,
                                                          current_lat, current_lon);
    float angle_between_rad = (bearing_exit_to_current - bearing_exit_to_target)
                              * (float)M_PI / 180.0f;
    return dist_exit_to_current * sinf(angle_between_rad);
}

/*
 * Lane cross-track deviation → steering arrow string.
 * Arrows indicate the direction to steer to return to lane centre.
 * Number of arrows indicates severity.
 */
static const char *LaneArrow(float deviation_meters)
{
    if      (deviation_meters >  LANE_THRESHOLD_MAJOR_M)    return "<<<";
    else if (deviation_meters >  LANE_THRESHOLD_MODERATE_M) return "<<";
    else if (deviation_meters >  LANE_THRESHOLD_MINOR_M)    return "<";
    else if (deviation_meters < -LANE_THRESHOLD_MAJOR_M)    return ">>>";
    else if (deviation_meters < -LANE_THRESHOLD_MODERATE_M) return ">>";
    else if (deviation_meters < -LANE_THRESHOLD_MINOR_M)    return ">";
    else                                                      return "===";
}

/*
 * Relative bearing to target → turn-direction arrow string.
 * Arrows indicate which direction to turn to face the target.
 */
static const char *HeadingArrow(float relative_bearing_deg)
{
    if      (relative_bearing_deg >  HDG_THRESHOLD_MAJOR_DEG)  return ">>";
    else if (relative_bearing_deg >  HDG_THRESHOLD_MINOR_DEG)  return ">";
    else if (relative_bearing_deg < -HDG_THRESHOLD_MAJOR_DEG)  return "<<";
    else if (relative_bearing_deg < -HDG_THRESHOLD_MINOR_DEG)  return "<";
    else                                                          return "^";
}

/* --------------------------------------------------------------------------
   Mode 1 public API
   -------------------------------------------------------------------------- */

void FS_ActiveLook_Mode1_Init(void)
{
    s_setup_step         = 0;
    s_competition_phase  = COMP_PHASE_IDLE;
    s_exit_lat           = 0;
    s_exit_lon           = 0;
    s_exit_altitude_mm   = 0;
    s_window_count       = 0;
    memset(s_window, 0, sizeof(s_window));
}

FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode1_Setup(void)
{
    uint8_t packet_buf[128];
    uint8_t packet_length;

    switch (s_setup_step)
    {
    case 0:
        packet_length = BuildStatusBarLayout(COMP_LAYOUT_STATUS_BAR, packet_buf);
        SendBlePacket(packet_buf, packet_length);
        s_setup_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 1:
        packet_length = BuildDataLineLayout(COMP_LAYOUT_PHASE, "PHASE", "", packet_buf);
        SendBlePacket(packet_buf, packet_length);
        s_setup_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 2:
        packet_length = BuildDataLineLayout(COMP_LAYOUT_LANE, "LANE", "", packet_buf);
        SendBlePacket(packet_buf, packet_length);
        s_setup_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 3:
        packet_length = BuildDataLineLayout(COMP_LAYOUT_HEADING, "HDG", "", packet_buf);
        SendBlePacket(packet_buf, packet_length);
        s_setup_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 4:
        packet_length = BuildDataLineLayout(COMP_LAYOUT_ALTITUDE, "ALT", "m", packet_buf);
        SendBlePacket(packet_buf, packet_length);
        s_setup_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 5:
        packet_length = BuildCompetitionPage(COMP_PAGE_ID, packet_buf);
        SendBlePacket(packet_buf, packet_length);
        s_setup_step++;
        return FS_AL_SETUP_DONE;

    default:
        return FS_AL_SETUP_DONE;
    }
}

void FS_ActiveLook_Mode1_Update(void)
{
    const FS_GNSS_Data_t   *gnss_data = FS_GNSS_GetData();
    const FS_Config_Data_t *config    = FS_Config_Get();
    const FS_VBAT_Data_t   *vbat_data = FS_VBAT_GetData();
    uint8_t al_battery_pct = FS_ActiveLook_Client_GetBatteryLevel();

    /* ---- Update freefall detection window and competition phase ---- */
    switch (s_competition_phase)
    {
    case COMP_PHASE_IDLE:
#if FORCE_FREEFALL_FOR_TESTING
        /* Testing shortcut: enter freefall immediately using current position */
        if (gnss_data->gpsFix == 3) {
            s_competition_phase = COMP_PHASE_FREEFALL;
            s_exit_lat          = gnss_data->lat;
            s_exit_lon          = gnss_data->lon;
            s_exit_altitude_mm  = gnss_data->hMSL;
        }
#else
        /* Feed each GNSS fix into the sliding window and test for exit */
        if (gnss_data->gpsFix == 3) {
            AddSampleToWindow(gnss_data);

            int32_t detected_lat, detected_lon, detected_alt_mm;
            if (DetectFreefallExit(&detected_lat, &detected_lon, &detected_alt_mm)) {
                s_competition_phase = COMP_PHASE_FREEFALL;
                s_exit_lat          = detected_lat;
                s_exit_lon          = detected_lon;
                s_exit_altitude_mm  = detected_alt_mm;
            }
        }
#endif
        break;

    case COMP_PHASE_FREEFALL:
        /* Detect canopy deployment: vertical velocity drops below threshold */
        if ((float)gnss_data->velD / 1000.0f < CANOPY_VELDOWN_THRESHOLD_MS) {
            s_competition_phase = COMP_PHASE_CANOPY;
        }
        break;

    case COMP_PHASE_CANOPY:
        /* Reset to IDLE if altitude rises well above exit point (back in aircraft) */
        if (gnss_data->hMSL > s_exit_altitude_mm + BACK_IN_AIRCRAFT_ALTITUDE_MM) {
            s_competition_phase = COMP_PHASE_IDLE;
            s_exit_lat          = 0;
            s_exit_lon          = 0;
            s_exit_altitude_mm  = 0;
            s_window_count      = 0;
            memset(s_window, 0, sizeof(s_window));
        }
        break;
    }

    /* ---- Status bar: battery levels and satellite count ---- */
    char al_battery_str[5];
    if (al_battery_pct == 255)
        snprintf(al_battery_str, sizeof(al_battery_str), "??");
    else
        snprintf(al_battery_str, sizeof(al_battery_str), "%d", al_battery_pct);

    int fs_battery_pct = (100 * ((int)vbat_data->voltage - 3300)) / (4200 - 3200);
    fs_battery_pct = MAX(0, MIN(100, fs_battery_pct));

    char status_bar_str[40];
    snprintf(status_bar_str, sizeof(status_bar_str), "A:%s%%  F:%d%%  N:%d",
             al_battery_str, fs_battery_pct, gnss_data->numSV);

    /* ---- Phase label ---- */
    const char *phase_label;
    switch (s_competition_phase) {
    case COMP_PHASE_FREEFALL: phase_label = "FREEFALL"; break;
    case COMP_PHASE_CANOPY:   phase_label = "CANOPY";   break;
    default:                  phase_label = "IDLE";     break;
    }

    /* ---- Lane deviation: arrows showing direction to steer ---- */
    const char *lane_arrow = "--";
    bool target_is_configured = (config->lat != 0 || config->lon != 0);
    bool exit_point_is_set    = (s_competition_phase != COMP_PHASE_IDLE);

    if (exit_point_is_set && target_is_configured && gnss_data->gpsFix == 3) {
        float deviation_meters = CrossTrackDeviationMeters(
                s_exit_lat, s_exit_lon,
                config->lat, config->lon,
                gnss_data->lat, gnss_data->lon);
        lane_arrow = LaneArrow(deviation_meters);
    }

    /* ---- Heading: arrows showing direction to turn toward target ---- */
    const char *heading_arrow = "--";
    if (target_is_configured && gnss_data->gpsFix == 3) {
        float bearing_to_target   = AbsoluteBearingDeg(gnss_data->lat, gnss_data->lon,
                                                         config->lat,    config->lon);
        float current_heading_deg = (float)gnss_data->heading / 100000.0f;
        float relative_bearing    = bearing_to_target - current_heading_deg;

        while (relative_bearing >  180.0f) relative_bearing -= 360.0f;
        while (relative_bearing < -180.0f) relative_bearing += 360.0f;

        heading_arrow = HeadingArrow(relative_bearing);
    }

    /* ---- Altitude AGL ---- */
    char altitude_agl_str[16];
    if (gnss_data->gpsFix == 3) {
        int32_t altitude_above_dz_m = (gnss_data->hMSL - config->dz_elev) / 1000;
        snprintf(altitude_agl_str, sizeof(altitude_agl_str), "%ld", (long)altitude_above_dz_m);
    } else {
        snprintf(altitude_agl_str, sizeof(altitude_agl_str), "--");
    }

    /* ---- Send display update ---- */
    uint8_t packet_buf[160];
    uint8_t packet_length = BuildPageUpdate(COMP_PAGE_ID,
                                             status_bar_str,
                                             phase_label,
                                             lane_arrow,
                                             heading_arrow,
                                             altitude_agl_str,
                                             packet_buf);
    SendBlePacket(packet_buf, packet_length);
}
