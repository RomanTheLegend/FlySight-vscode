/***************************************************************************
**  FlySight 2 firmware — GPS mock for SD-card replay                     **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
**                                                                        **
**  Supported CSV formats                                                  **
**                                                                        **
**  Format A — plain CSV (legacy MOCK.CSV):                               **
**    Row 1 = column names, row 2 = units, rows 3+ = data                 **
**    Columns: time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,          **
**             gpsFix,numSV,heading,headAcc                               **
**                                                                        **
**  Format B — FlySight 2 track log ($GNSS rows):                         **
**    Header lines start with '$' (not '$GNSS,') and are skipped.         **
**    Data lines: $GNSS,time,lat,lon,hMSL,velN,velE,velD,                 **
**                hAcc,vAcc,sAcc,numSV                                    **
**    gpsFix is assumed 3 (3-D fix); heading defaults to 0.               **
**                                                                        **
**  The format is detected per-row: '$GNSS,' prefix → Format B,          **
**  first char is a digit → Format A data row, else skipped.             **
****************************************************************************/

#include "gnss_mock.h"

#if GNSS_MOCK_ENABLED

#include "main.h"
#include "app_common.h"
#include "gnss.h"
#include "ff.h"
#include "stm32_seq.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ---- Constants ----------------------------------------------------------- */

#define MOCK_FILE_PATH    "MOCK.CSV"
#define MOCK_TIMER_MS     200u
#define MOCK_TIMER_RATE   (MOCK_TIMER_MS * 1000u / CFG_TS_TICK_VAL)
#define MOCK_LINE_LEN     220
#define MOCK_MAX_GAP_MS   5000u   /* gaps wider than this are compressed */
#define MOCK_POST_GAP_MS   500u   /* playback delay used for a compressed gap */

/* ---- Static state -------------------------------------------------------- */

static FIL  s_file;
static bool s_open = false;
static bool s_eof  = false;

static FS_GNSS_Data_t s_curr;
static uint32_t       s_curr_csv_ts;   /* ms-from-midnight of current record */

static FS_GNSS_Data_t s_next;
static uint32_t       s_next_csv_ts;   /* ms-from-midnight of next record */
static uint32_t       s_next_play_at;  /* HAL_GetTick() when to inject s_next */
static bool           s_next_valid;

static uint8_t s_timer_id;

/* ---- Timestamp parser ----------------------------------------------------- */

/*
 * Parse "YYYY-MM-DDTHH:MM:SS[.frac]Z" → milliseconds from midnight.
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

/* ---- Gap computation ------------------------------------------------------ */

static uint32_t ComputeGapMs(uint32_t from_ts, uint32_t to_ts)
{
    uint32_t delta = (to_ts >= from_ts)
                   ? (to_ts - from_ts)
                   : (to_ts + 24u * 3600u * 1000u - from_ts);

    if (delta > MOCK_MAX_GAP_MS)
        delta = MOCK_POST_GAP_MS;

    return delta;
}

/* ---- CSV row parser ------------------------------------------------------- */

/*
 * Parse one data row.  buf is modified in-place (strtok).
 * Detects format by checking for '$GNSS,' prefix.
 * Returns the raw ms-from-midnight timestamp in *csv_ts_out.
 */
static bool ParseRow(char *buf, uint32_t *csv_ts_out, FS_GNSS_Data_t *d)
{
    char *p = buf;
    bool  fmt_b = false;

    if (strncmp(p, "$GNSS,", 6) == 0) {
        fmt_b = true;
        p += 6;
    } else if (*p < '0' || *p > '9') {
        return false;   /* not a data row */
    }

    memset(d, 0, sizeof(*d));

    char *tok;

    tok = strtok(p, ","); if (!tok) return false;
    *csv_ts_out = ParseTimestampMs(tok);

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

    if (fmt_b) {
        /* Format B: last field is numSV; gpsFix = 3, heading = 0 */
        tok = strtok(NULL, ","); if (!tok) return false;
        d->numSV  = (uint8_t)atoi(tok);
        d->gpsFix = 3;
    } else {
        /* Format A: gpsFix, numSV, heading, headAcc */
        tok = strtok(NULL, ","); if (!tok) return false;
        d->gpsFix = (uint8_t)atoi(tok);

        tok = strtok(NULL, ","); if (!tok) return false;
        d->numSV = (uint8_t)atoi(tok);

        tok = strtok(NULL, ","); if (!tok) return false;
        d->heading = (int32_t)(atof(tok) * 1e5);
        /* headAcc: skip */
    }

    d->iTOW = *csv_ts_out;
    return true;
}

/* Read and parse the next data line, skipping non-data lines. */
static bool ReadNext(uint32_t *csv_ts, FS_GNSS_Data_t *d)
{
    static char buf[MOCK_LINE_LEN];

    while (!f_eof(&s_file)) {
        if (!f_gets(buf, sizeof(buf), &s_file)) break;
        size_t len = strcspn(buf, "\r\n");
        if (len == 0) continue;
        buf[len] = '\0';
        if (ParseRow(buf, csv_ts, d)) return true;
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
    if (f_open(&s_file, MOCK_FILE_PATH, FA_READ) != FR_OK) return;
    s_open = true;
    s_eof  = false;

    /* ReadNext skips all header/meta lines in both formats */
    if (!ReadNext(&s_curr_csv_ts, &s_curr)) goto fail;

    /* Pre-load look-ahead and schedule its fire time */
    s_next_valid = ReadNext(&s_next_csv_ts, &s_next);
    if (s_next_valid) {
        uint32_t gap = ComputeGapMs(s_curr_csv_ts, s_next_csv_ts);
        s_next_play_at = HAL_GetTick() + gap;
    }

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
 * Advances playback whenever wall-clock time has reached the scheduled
 * fire time for the next record.  Gaps wider than MOCK_MAX_GAP_MS are
 * compressed to MOCK_POST_GAP_MS.
 */
static void MockUpdate(void)
{
    if (!s_open) return;

    uint32_t now = HAL_GetTick();

    while (s_next_valid && (int32_t)(now - s_next_play_at) >= 0) {
        uint32_t scheduled_at = s_next_play_at;

        s_curr        = s_next;
        s_curr_csv_ts = s_next_csv_ts;

        FS_GNSS_InjectData(&s_curr);

        uint32_t next_ts;
        s_next_valid = s_eof ? false : ReadNext(&next_ts, &s_next);
        if (s_next_valid) {
            uint32_t gap   = ComputeGapMs(s_curr_csv_ts, next_ts);
            s_next_csv_ts  = next_ts;
            s_next_play_at = scheduled_at + gap;
        }

        now = HAL_GetTick();
    }

    if (!s_next_valid && s_eof)
        HW_TS_Stop(s_timer_id);
}

#endif /* GNSS_MOCK_ENABLED */
