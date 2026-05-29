/***************************************************************************
**  FlySight 2 firmware — GPS mock for SD-card replay                     **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
**                                                                        **
**  CSV column layout (row 1 = header, row 2 = units, row 3+ = data):   **
**    0  time     ISO 8601  "2026-05-24T08:35:58.74Z"                    **
**    1  lat      degrees                                                 **
**    2  lon      degrees                                                 **
**    3  hMSL     metres                                                  **
**    4  velN     m/s                                                     **
**    5  velE     m/s                                                     **
**    6  velD     m/s                                                     **
**    7  hAcc     metres                                                  **
**    8  vAcc     metres                                                  **
**    9  sAcc     m/s                                                     **
**   10  gpsFix   integer (3 = 3-D fix)                                  **
**   11  numSV    integer                                                 **
**   12  heading  degrees                                                 **
**   13  headAcc  degrees  (ignored)                                      **
****************************************************************************/

#include "gnss_mock.h"

#if GNSS_MOCK_ENABLED

#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "gnss.h"
#include "ff.h"
#include "stm32_seq.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ---- Constants ----------------------------------------------------------- */

#define MOCK_FILE_PATH  "MOCK.CSV"
#define MOCK_TIMER_MS   200u
#define MOCK_TIMER_RATE (MOCK_TIMER_MS * 1000u / CFG_TS_TICK_VAL)
#define MOCK_LINE_LEN   220

/* ---- Static state -------------------------------------------------------- */

static FIL  s_file;
static bool s_open       = false;
static bool s_eof        = false;

static FS_GNSS_Data_t s_curr;
static uint32_t       s_curr_rel_ms;

static FS_GNSS_Data_t s_next;
static uint32_t       s_next_rel_ms;
static bool           s_next_valid;

static uint32_t s_origin_ms;   /* CSV timestamp of first data row (ms from midnight) */
static uint32_t s_start_tick;  /* HAL_GetTick() at playback start */
static uint8_t  s_timer_id;

/* ---- Timestamp parser ----------------------------------------------------- */

/*
 * Parse "YYYY-MM-DDTHH:MM:SS[.frac]Z" → milliseconds from midnight.
 * Only the time portion (after 'T') is used.
 */
static uint32_t ParseTimestampMs(const char *ts)
{
    const char *p = strchr(ts, 'T');
    if (!p) return 0;
    ++p;

    int hour = (int)strtol(p, NULL, 10);
    p = strchr(p, ':'); if (!p) return 0; ++p;
    int min  = (int)strtol(p, NULL, 10);
    p = strchr(p, ':'); if (!p) return 0; ++p;
    int sec  = (int)strtol(p, NULL, 10);

    uint32_t ms = (uint32_t)hour * 3600000u
                + (uint32_t)min  *   60000u
                + (uint32_t)sec  *    1000u;

    p = strchr(p, '.');
    if (p) {
        ++p;
        int frac = 0, mult = 100;
        while (*p >= '0' && *p <= '9' && mult > 0) {
            frac += (*p++ - '0') * mult;
            mult /= 10;
        }
        ms += (uint32_t)frac;
    }
    return ms;
}

/* ---- CSV row parser ------------------------------------------------------- */

/*
 * Parse one comma-separated data row into *d and compute its ms offset from
 * s_origin_ms.  buf is modified in-place (strtok).
 * s_origin_ms must be initialised before the first call.
 */
static bool ParseRow(char *buf, uint32_t *rel_ms_out, FS_GNSS_Data_t *d)
{
    char *tok;

    memset(d, 0, sizeof(*d));

    tok = strtok(buf, ","); if (!tok) return false;
    uint32_t ts_ms = ParseTimestampMs(tok);
    if (ts_ms < s_origin_ms) ts_ms += 24u * 3600u * 1000u;  /* midnight wrap */
    *rel_ms_out = ts_ms - s_origin_ms;

    tok = strtok(NULL, ","); if (!tok) return false;
    d->lat = (int32_t)(atof(tok) * 1e7);

    tok = strtok(NULL, ","); if (!tok) return false;
    d->lon = (int32_t)(atof(tok) * 1e7);

    tok = strtok(NULL, ","); if (!tok) return false;
    d->hMSL = (int32_t)(atof(tok) * 1000.0);

    tok = strtok(NULL, ","); if (!tok) return false;
    double velN = atof(tok);
    d->velN = (int32_t)(velN * 1000.0);

    tok = strtok(NULL, ","); if (!tok) return false;
    double velE = atof(tok);
    d->velE = (int32_t)(velE * 1000.0);

    tok = strtok(NULL, ","); if (!tok) return false;
    double velD = atof(tok);
    d->velD = (int32_t)(velD * 1000.0);

    d->gSpeed = (int32_t)(sqrt(velN*velN + velE*velE) * 100.0);
    d->speed  = (int32_t)(sqrt(velN*velN + velE*velE + velD*velD) * 100.0);

    tok = strtok(NULL, ","); if (!tok) return false;
    d->hAcc = (uint32_t)(atof(tok) * 1000.0);

    tok = strtok(NULL, ","); if (!tok) return false;
    d->vAcc = (uint32_t)(atof(tok) * 1000.0);

    tok = strtok(NULL, ","); if (!tok) return false;
    d->sAcc = (uint32_t)(atof(tok) * 1000.0);

    tok = strtok(NULL, ","); if (!tok) return false;
    d->gpsFix = (uint8_t)atoi(tok);

    tok = strtok(NULL, ","); if (!tok) return false;
    d->numSV = (uint8_t)atoi(tok);

    tok = strtok(NULL, ","); if (!tok) return false;
    d->heading = (int32_t)(atof(tok) * 1e5);

    /* col 13: headAcc — skip */

    d->iTOW = *rel_ms_out;
    return true;
}

/* Read and parse the next non-empty data line from s_file. */
static bool ReadNext(uint32_t *rel_ms, FS_GNSS_Data_t *d)
{
    static char buf[MOCK_LINE_LEN];

    while (!f_eof(&s_file)) {
        if (!f_gets(buf, sizeof(buf), &s_file)) break;
        size_t len = strcspn(buf, "\r\n");
        if (len == 0) continue;
        buf[len] = '\0';
        if (ParseRow(buf, rel_ms, d)) return true;
    }
    s_eof = true;
    return false;
}

/* ---- Forward declarations ------------------------------------------------ */

static void MockTimer(void);
static void MockUpdate(void);

/* ---- Public API ---------------------------------------------------------- */

void FS_GNSS_Mock_Init(void)
{
    char buf[MOCK_LINE_LEN];

    if (f_open(&s_file, MOCK_FILE_PATH, FA_READ) != FR_OK) return;
    s_open = true;
    s_eof  = false;

    /* Skip header row 1 (column names) and row 2 (units) */
    f_gets(buf, sizeof(buf), &s_file);
    f_gets(buf, sizeof(buf), &s_file);

    /* Read first data row */
    if (!f_gets(buf, sizeof(buf), &s_file)) goto fail;
    buf[strcspn(buf, "\r\n")] = '\0';

    /* Extract origin timestamp before strtok destroys the buffer */
    {
        const char *comma = strchr(buf, ',');
        size_t ts_len = comma ? (size_t)(comma - buf) : strlen(buf);
        char ts_buf[32];
        if (ts_len >= sizeof(ts_buf)) ts_len = sizeof(ts_buf) - 1;
        memcpy(ts_buf, buf, ts_len);
        ts_buf[ts_len] = '\0';
        s_origin_ms = ParseTimestampMs(ts_buf);
    }

    /* Parse the first row (rel_ms will be 0 since origin == this row's ts) */
    {
        uint32_t rel;
        if (!ParseRow(buf, &rel, &s_curr)) goto fail;
        s_curr_rel_ms = 0;
    }

    /* Pre-load the next record for the look-ahead */
    s_next_valid = ReadNext(&s_next_rel_ms, &s_next);

    s_start_tick = HAL_GetTick();

    /* Deliver first record immediately */
    FS_GNSS_InjectData(&s_curr);

    /* Register task and start the periodic injection timer */
    UTIL_SEQ_RegTask(1 << CFG_TASK_FS_GNSS_MOCK_ID, UTIL_SEQ_RFU, MockUpdate);
    HW_TS_Create(CFG_TIM_PROC_ID_ISR, &s_timer_id, hw_ts_Repeated, MockTimer);
    HW_TS_Start(s_timer_id, MOCK_TIMER_RATE);
    return;

fail:
    f_close(&s_file);
    s_open = false;
}

void FS_GNSS_Mock_DeInit(void)
{
    HW_TS_Delete(s_timer_id);
    if (s_open) {
        f_close(&s_file);
        s_open = false;
    }
}

/* ---- Timer / task -------------------------------------------------------- */

static void MockTimer(void)
{
    UTIL_SEQ_SetTask(1 << CFG_TASK_FS_GNSS_MOCK_ID, CFG_SCH_PRIO_0);
}

/*
 * Called from the sequencer every MOCK_TIMER_MS ms.
 * Advances the current record whenever the wall-clock elapsed time has
 * passed the next record's CSV timestamp offset.
 */
static void MockUpdate(void)
{
    if (!s_open) return;

    uint32_t elapsed = HAL_GetTick() - s_start_tick;

    while (s_next_valid && elapsed >= s_next_rel_ms) {
        s_curr        = s_next;
        s_curr_rel_ms = s_next_rel_ms;

        FS_GNSS_InjectData(&s_curr);

        s_next_valid = s_eof ? false : ReadNext(&s_next_rel_ms, &s_next);
        elapsed      = HAL_GetTick() - s_start_tick;
    }
}

#endif /* GNSS_MOCK_ENABLED */
