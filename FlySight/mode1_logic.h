/***************************************************************************
**  FlySight 2 firmware — Shared competition mode flight logic            **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
**                                                                        **
**  Owns the phase state machine, freefall detection, score capture,      **
**  lane computation, and navigation derivations.  Renderers (ActiveLook, **
**  Vuzix, …) call Mode1_Logic_Update() once per GNSS frame and then read **
**  all computed values from Mode1_Logic_GetData().                       **
****************************************************************************/

#ifndef MODE1_LOGIC_H
#define MODE1_LOGIC_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    COMP_PHASE_IDLE            = 0,
    COMP_PHASE_AIRPLANE        = 1,
    COMP_PHASE_FREEFALL        = 2,
    COMP_PHASE_COMPETITION_RUN = 3,
    COMP_PHASE_RUN_SCORE       = 4,
    COMP_PHASE_HOME_NAV        = 5,
} CompetitionPhase_t;

typedef struct {
    uint8_t  phase;              /* CompetitionPhase_t */
    bool     has_gps;
    int32_t  alt_agl_m;         /* altitude above DZ elevation, metres */
    int32_t  hMSL_m;            /* altitude above MSL, metres */
    int32_t  dz_elev_m;         /* DZ elevation, metres */
    int32_t  gspeed_kmh;        /* horizontal speed, km/h */
    float    velD_ms;           /* vertical speed m/s (positive = descending) */
    float    glide_ratio;        /* gSpeed*10/velD (display units); 0 if unavailable */
    uint8_t  num_sv;            /* GNSS satellites in use */
    /* Freefall */
    int      freefall_cntdn_s;  /* seconds remaining in freefall lock countdown */
    /* Competition run target point */
    bool     has_target;
    float    rel_to_target_deg; /* relative bearing to target, -180..180 */
    /* Drop zone */
    bool     has_dz;
    float    rel_to_dz_deg;     /* relative bearing to DZ, -180..180 */
    int32_t  dz_dist_m;         /* distance to DZ in metres */
    /* Lane deviation */
    bool     lane_valid;
    float    lane_dev_m;        /* cross-track metres; positive = right of lane */
    /* Score */
    bool     score_valid;
    float    score_time_s;
    float    score_speed_kmh;
    float    score_dist_m;
} Mode1_Data_t;

void                Mode1_Logic_Init(void);
bool                Mode1_Logic_Update(void);    /* returns true when phase changes */
const Mode1_Data_t *Mode1_Logic_GetData(void);

#endif /* MODE1_LOGIC_H */
