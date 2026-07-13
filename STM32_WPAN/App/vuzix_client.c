/***************************************************************************
**  FlySight 2 firmware — Vuzix Z100 BLE client                           **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
**                                                                        **
**  Implements GATT discovery for the Vuzix Z100 custom service plus a    **
**  minimal MessagePack encoder for the Z100 wire protocol.               **
**                                                                        **
**  UUIDs (protocol source: decompiled com.vuzix.connect APK)             **
**    Main service  39191400-BFC2-4BEE-A81A-28C23B6AC84A                  **
**    TX char       39191401-…  (Write No Response, host → glasses)       **
**    RX char       39191402-…  (Notify, glasses → host)                  **
****************************************************************************/

#include "vuzix_client.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "stm32_seq.h"
#include <string.h>
#include <stdio.h>

#ifndef UNPACK_2_BYTE_PARAMETER
#define UNPACK_2_BYTE_PARAMETER(ptr) \
    (uint16_t)(((uint16_t)(*(uint8_t *)(ptr))) | ((uint16_t)(*((uint8_t *)(ptr) + 1)) << 8U))
#endif

/* ---- UUID constants (little-endian BLE byte order) ---- */

/* 39191400-BFC2-4BEE-A81A-28C23B6AC84A */
static const uint8_t VUZIX_SVC_UUID[16] = {
    0x4A, 0xC8, 0x6A, 0x3B, 0xC2, 0x28, 0x1A, 0xA8,
    0xEE, 0x4B, 0xC2, 0xBF, 0x00, 0x14, 0x19, 0x39
};
/* 39191401-BFC2-4BEE-A81A-28C23B6AC84A  TX — Write No Response */
static const uint8_t VUZIX_TX_UUID[16] = {
    0x4A, 0xC8, 0x6A, 0x3B, 0xC2, 0x28, 0x1A, 0xA8,
    0xEE, 0x4B, 0xC2, 0xBF, 0x01, 0x14, 0x19, 0x39
};
/* 39191402-BFC2-4BEE-A81A-28C23B6AC84A  RX — Notify */
static const uint8_t VUZIX_RX_UUID[16] = {
    0x4A, 0xC8, 0x6A, 0x3B, 0xC2, 0x28, 0x1A, 0xA8,
    0xEE, 0x4B, 0xC2, 0xBF, 0x02, 0x14, 0x19, 0x39
};

#define BATTERY_SVC_UUID   0x180Fu
#define BATTERY_LEVEL_UUID 0x2A19u
#define CCCD_UUID          0x2902u

/* ---- Discovery state ---- */

typedef enum {
    DISC_IDLE = 0,
    DISC_EXCH_MTU,
    DISC_SVC,
    DISC_CHAR_VUZIX,
    DISC_CHAR_BATT,
    DISC_DESC_RX,
    DISC_NOTIFY_RX,
    DISC_DESC_BATT,
    DISC_NOTIFY_BATT,
} DiscState_t;

typedef struct {
    uint16_t  connHandle;
    DiscState_t discState;
    uint16_t  negotiatedMtu;

    uint8_t   svcFound;
    uint16_t  svcStart;
    uint16_t  svcEnd;

    uint8_t   txFound;
    uint16_t  txHandle;   /* TX characteristic value handle */

    uint8_t   rxFound;
    uint16_t  rxHandle;   /* RX characteristic value handle */
    uint16_t  rxCCCDHandle;

    uint8_t   battSvcFound;
    uint16_t  battSvcStart;
    uint16_t  battSvcEnd;
    uint8_t   battCharFound;
    uint16_t  battCharHandle;
    uint16_t  battCCCDHandle;
    uint8_t   lastBattPct;

    const FS_Vuzix_ClientCb_t *cb;
} VuzixCtx_t;

static VuzixCtx_t g_ctx;

/* ---- Transmit buffer & MessagePack encoder ---- */

/* Reserve index 0 for the Vuzix framing byte; MP bytes start at [1]. */
#define VZ_TX_BUF  244u   /* must be ≤ ATT MTU-3 (244 @ MTU=247) and fit in uint8_t */
static uint8_t  s_tx[VZ_TX_BUF];
static uint16_t s_tx_len;

/* ---- TX queue (central → glasses, Write No Response) ---- */

#define VZ_TX_WINDOW  16u

typedef struct {
    uint8_t  data[VZ_TX_BUF];
    uint16_t len;
} VzTxSlot_t;

static VzTxSlot_t s_txq[VZ_TX_WINDOW];
static uint32_t   s_txq_read;
static uint32_t   s_txq_write;
static uint8_t    s_txq_flow;   /* 1 = BLE TX pool available */

static void vz_txq_transmit(void);

static void vz_txq_reset(void)
{
    s_txq_read  = 0u;
    s_txq_write = 0u;
    s_txq_flow  = 1u;
}

static void mp_reset(void)            { s_tx_len = 1u; }
static void mp_u8(uint8_t b)         { if (s_tx_len < VZ_TX_BUF) s_tx[s_tx_len++] = b; }
static void mp_fixmap1(void)         { mp_u8(0x81u); }
static void mp_nil(void)             { mp_u8(0xC0u); }
static void mp_false(void)           { mp_u8(0xC2u); }
static void mp_true(void)            { mp_u8(0xC3u); }
static void mp_fixarray(uint8_t n)   { mp_u8((uint8_t)(0x90u | (n & 0x0Fu))); }
static void mp_fixint(uint8_t v)     { mp_u8(v & 0x7Fu); }

static void mp_str(const char *s, uint16_t len)
{
    if (len <= 31u) {
        mp_u8((uint8_t)(0xA0u | (uint8_t)len));
    } else {
        mp_u8(0xD9u);
        mp_u8((uint8_t)len);
    }
    for (uint16_t i = 0; i < len; i++) mp_u8((uint8_t)s[i]);
}

/* Compact unsigned integer — picks smallest representation. */
static void mp_uint(uint32_t v)
{
    if      (v <= 0x7Fu)   { mp_u8((uint8_t)v); }
    else if (v <= 0xFFu)   { mp_u8(0xCCu); mp_u8((uint8_t)v); }
    else if (v <= 0xFFFFu) { mp_u8(0xCDu); mp_u8((uint8_t)(v >> 8u)); mp_u8((uint8_t)v); }
    else                   { mp_u8(0xCEu);
                             mp_u8((uint8_t)(v >> 24u)); mp_u8((uint8_t)(v >> 16u));
                             mp_u8((uint8_t)(v >>  8u)); mp_u8((uint8_t)v); }
}

static void mp_bin(const uint8_t *data, uint8_t len)
{
    mp_u8(0xC4u);   /* bin8 */
    mp_u8(len);
    for (uint8_t i = 0u; i < len; i++) mp_u8(data[i]);
}

/* Compact signed integer — picks smallest representation. */
static void mp_int(int32_t v)
{
    if      (v >= 0)       { mp_uint((uint32_t)v); }
    else if (v >= -32)     { mp_u8((uint8_t)(int8_t)v); }
    else if (v >= -128)    { mp_u8(0xD0u); mp_u8((uint8_t)(int8_t)v); }
    else if (v >= -32768)  { mp_u8(0xD1u); mp_u8((uint8_t)(v >> 8)); mp_u8((uint8_t)v); }
    else                   { mp_u8(0xD2u);
                             mp_u8((uint8_t)(v >> 24)); mp_u8((uint8_t)(v >> 16));
                             mp_u8((uint8_t)(v >>  8)); mp_u8((uint8_t)v); }
}

static tBleStatus send_framed(void)
{
    if (!g_ctx.txFound || g_ctx.txHandle == 0u)
        return BLE_STATUS_INVALID_PARAMS;
    if (s_txq_write >= s_txq_read + VZ_TX_WINDOW)
        return BLE_STATUS_FAILED;   /* queue full — caller should retry */

    s_tx[0] = 0x00u;   /* last/only chunk, framing byte */
    VzTxSlot_t *slot = &s_txq[s_txq_write % VZ_TX_WINDOW];
    memcpy(slot->data, s_tx, s_tx_len);
    slot->len = s_tx_len;
    s_txq_write++;
    UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_TX_ID, CFG_SCH_PRIO_1);
    return BLE_STATUS_SUCCESS;
}

/* ---- Font slot upload (chunked, multi-write) ---- */

#define VZ_FONT_HDR_MAX 16u

typedef struct {
    const uint8_t *data;
    uint16_t       total;
    uint16_t       offset;
    uint8_t        hdr[VZ_FONT_HDR_MAX];
    uint8_t        hdr_len;
    uint8_t        hdr_sent;
    uint8_t        active;
} FontUpState_t;

static FontUpState_t s_fup;

static tBleStatus enqueue_raw(const uint8_t *chunk, uint16_t len)
{
    if (s_txq_write >= s_txq_read + VZ_TX_WINDOW)
        return BLE_STATUS_FAILED;
    VzTxSlot_t *slot = &s_txq[s_txq_write % VZ_TX_WINDOW];
    memcpy(slot->data, chunk, len);
    slot->len = len;
    s_txq_write++;
    UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_TX_ID, CFG_SCH_PRIO_1);
    return BLE_STATUS_SUCCESS;
}

/* Start or continue a font-slot upload.
 * Returns BLE_STATUS_SUCCESS when all data has been queued.
 * Returns BLE_STATUS_FAILED when the TX queue filled up — call again to continue.
 * Must only be called after a successful call to send_framed() (i.e. TX char found). */
tBleStatus FS_Vuzix_Client_SendFontSlot(uint8_t slot_idx,
                                         const uint8_t *data, uint16_t len)
{
    if (!g_ctx.txFound)
        return BLE_STATUS_INVALID_PARAMS;

    if (!s_fup.active) {
        /* First call — initialise upload state and build MP header */
        uint8_t p = 0;
        s_fup.hdr[p++] = 0x81u;                        /* fixmap(1) */
        s_fup.hdr[p++] = 0xA9u;                        /* fixstr(9) */
        memcpy(&s_fup.hdr[p], "font_slot", 9u); p += 9u;
        s_fup.hdr[p++] = 0x92u;                        /* fixarray(2) */
        s_fup.hdr[p++] = slot_idx & 0x7Fu;             /* slot (fixint 0-3) */
        s_fup.hdr[p++] = 0xC5u;                        /* bin16 */
        s_fup.hdr[p++] = (uint8_t)(len >> 8u);
        s_fup.hdr[p++] = (uint8_t)len;
        s_fup.hdr_len  = p;                             /* = 16 */
        s_fup.data     = data;
        s_fup.total    = len;
        s_fup.offset   = 0u;
        s_fup.hdr_sent = 0u;
        s_fup.active   = 1u;
        APP_DBG_MSG("Vuzix: font_slot %d upload start (%d bytes)\n", slot_idx, len);
    }

    /* Drain as many chunks into the TX queue as it will accept */
    while (s_fup.hdr_sent == 0u || s_fup.offset < s_fup.total) {
        uint8_t  chunk[VZ_TX_BUF];
        uint16_t pos = 1u;  /* byte 0 reserved for framing */
        uint8_t  new_hdr_sent = s_fup.hdr_sent;
        uint16_t new_offset   = s_fup.offset;

        /* On the first chunk, prepend the MessagePack header */
        if (!new_hdr_sent) {
            memcpy(&chunk[1], s_fup.hdr, s_fup.hdr_len);
            pos += s_fup.hdr_len;
            new_hdr_sent = 1u;
        }

        uint16_t remaining = s_fup.total - s_fup.offset;
        uint16_t capacity  = VZ_TX_BUF - pos;
        uint16_t put       = (remaining < capacity) ? remaining : capacity;
        memcpy(&chunk[pos], &s_fup.data[s_fup.offset], put);
        pos        += put;
        new_offset += put;

        chunk[0] = (new_offset >= s_fup.total) ? 0x00u : 0x80u;

        if (enqueue_raw(chunk, pos) != BLE_STATUS_SUCCESS) {
            APP_DBG_MSG("Vuzix: font_slot queue full at %d/%d, will retry\n",
                        s_fup.offset, s_fup.total);
            return BLE_STATUS_FAILED;  /* queue full — state unchanged, safe to retry */
        }

        s_fup.hdr_sent = new_hdr_sent;
        s_fup.offset   = new_offset;
    }

    s_fup.active = 0u;
    APP_DBG_MSG("Vuzix: font_slot upload complete (%d bytes queued)\n", s_fup.total);
    return BLE_STATUS_SUCCESS;
}

static void vz_txq_transmit(void)
{
    if (!s_txq_flow || s_txq_read >= s_txq_write)
        return;
    VzTxSlot_t *slot = &s_txq[s_txq_read % VZ_TX_WINDOW];
    tBleStatus s = aci_gatt_write_without_resp(g_ctx.connHandle, g_ctx.txHandle,
                                               slot->len, slot->data);
    if (s == BLE_STATUS_INSUFFICIENT_RESOURCES) {
        s_txq_flow = 0;   /* pause; resume on ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE */
    } else {
        s_txq_read++;
        /* If a font upload is in progress, wake the setup task so it can fill
         * the slot we just freed — it must not self-reschedule to avoid starving
         * this TX task (PRIO_1 vs SETUP PRIO_0). */
        if (s_fup.active)
            UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
        if (s_txq_read < s_txq_write)
            UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_TX_ID, CFG_SCH_PRIO_1);
    }
}

/* ---- Public init / registration ---- */

void FS_Vuzix_Client_Init(void)
{
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.lastBattPct = 255u;

    vz_txq_reset();
    UTIL_SEQ_RegTask(1u << CFG_TASK_FS_VUZIX_TX_ID, UTIL_SEQ_RFU, vz_txq_transmit);
}

void FS_Vuzix_Client_RegisterCb(const FS_Vuzix_ClientCb_t *cb)
{
    g_ctx.cb = cb;
}

uint8_t FS_Vuzix_Client_IsReady(void)
{
    return g_ctx.txFound;
}

uint8_t FS_Vuzix_Client_GetBatteryLevel(void)
{
    return g_ctx.lastBattPct;
}

uint16_t FS_Vuzix_Client_GetConnHandle(void)
{
    return g_ctx.connHandle;
}

/* ---- High-level message builders ---- */

tBleStatus FS_Vuzix_Client_SendConnAppInit(void)
{
    mp_reset();
    mp_fixmap1();
    mp_str("conn_app_init", 13u);
    mp_fixarray(1u);
    mp_fixint(2u);
    return send_framed();
}

tBleStatus FS_Vuzix_Client_SendControl(uint8_t layout_id)
{
    /* control: [deprecated_ancs=false, timeout_secs=0, layout=layout_id,
     *           app_id="FlySight", status_bar_hide=true, proto_ver=2] */
    mp_reset();
    mp_fixmap1();
    mp_str("control", 7u);
    mp_fixarray(6u);
    mp_false();
    mp_fixint(0u);
    mp_fixint(layout_id);
    mp_str("FlySight", 8u);
    mp_true();
    mp_fixint(2u);
    return send_framed();
}

tBleStatus FS_Vuzix_Client_SendSetTime(void)
{
    extern RTC_HandleTypeDef hrtc;
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    /* Format: "YYYYMMDDTHHmmss" (15 chars, compact ISO 8601 UTC) */
    char ts[16];
    snprintf(ts, sizeof(ts), "20%02d%02d%02dT%02d%02d%02d",
             (int)sDate.Year, (int)sDate.Month, (int)sDate.Date,
             (int)sTime.Hours, (int)sTime.Minutes, (int)sTime.Seconds);

    mp_reset();
    mp_fixmap1();
    mp_str("set_time", 8u);
    mp_str(ts, 15u);
    return send_framed();
}

tBleStatus FS_Vuzix_Client_SendKeepalive(void)
{
    mp_reset();
    mp_fixmap1();
    mp_str("keepalive", 9u);
    mp_nil();
    return send_framed();
}

tBleStatus FS_Vuzix_Client_SendText(const char *text)
{
    uint16_t tlen = (uint16_t)strlen(text);
    if (tlen > 220u) tlen = 220u;  /* guard against oversized messages */
    mp_reset();
    mp_fixmap1();
    mp_str("tbla", 4u);
    mp_fixarray(1u);
    mp_str(text, tlen);
    return send_framed();
}

tBleStatus FS_Vuzix_Client_SendLabel(uint8_t idx, int16_t x, int16_t y,
                                     uint8_t show, const char *text)
{
    /* lvgl_obj → {label: [idx, nil, color, nil, x, y, show, text]} (8 elements) */
    uint16_t tlen = (uint16_t)strlen(text);
    if (tlen > 180u) tlen = 180u;
    mp_reset();
    mp_fixmap1();
    mp_str("lvgl_obj", 8u);
    mp_fixmap1();
    mp_str("label", 5u);
    mp_fixarray(8u);
    mp_uint(idx);             /* idx: u32 */
    mp_nil();                 /* text_align: nil */
    mp_fixint(3u);            /* color: palette index 3 = white */
    mp_fixint(1u);            /* label_align: 1 = LV_ALIGN_TOP_LEFT */
    mp_int(x);                /* x: i32 */
    mp_int(y);                /* y: i32 */
    if (show) mp_true(); else mp_false();
    mp_str(text, tlen);
    return send_framed();
}

tBleStatus FS_Vuzix_Client_SendLabelFont(uint8_t idx, uint8_t font_slot_idx,
                                          int16_t x, int16_t y,
                                          uint8_t show, const char *text)
{
    /* Top-level command: {label_font: [idx, slot, nil, color, label_align, x, y, show, text]} */
    uint16_t tlen = (uint16_t)strlen(text);
    if (tlen > 170u) tlen = 170u;
    mp_reset();
    mp_fixmap1();
    mp_str("label_font", 10u);
    mp_fixarray(9u);
    mp_uint(idx);
    mp_fixint(font_slot_idx);
    mp_nil();           /* text_align: nil (use default) */
    mp_fixint(3u);      /* color: 3 = white */
    mp_fixint(1u);      /* label_align: 1 = LV_ALIGN_TOP_LEFT */
    mp_int(x);
    mp_int(y);
    if (show) mp_true(); else mp_false();
    mp_str(text, tlen);
    return send_framed();
}

tBleStatus FS_Vuzix_Client_SendCanvasRect(uint16_t x0, uint16_t y0,
                                          uint16_t x1, uint16_t y1,
                                          uint8_t color)
{
    /* canvas → {cnvs_clrblk: [x0, y0, x1, y1, color]} (5 elements) */
    mp_reset();
    mp_fixmap1();
    mp_str("canvas", 6u);
    mp_fixmap1();
    mp_str("cnvs_clrblk", 11u);
    mp_fixarray(5u);
    mp_uint(x0);
    mp_uint(y0);
    mp_uint(x1);
    mp_uint(y1);
    mp_fixint(color);
    return send_framed();
}

tBleStatus FS_Vuzix_Client_SendCanvasImg(uint16_t x, uint16_t y,
                                         uint8_t w, uint8_t h,
                                         const uint8_t *data, uint8_t data_len)
{
    /* canvas → {cnvs_img: [[x, y, w, h], bin(data)]} — raw 2-bit indexed */
    mp_reset();
    mp_fixmap1();
    mp_str("canvas", 6u);
    mp_fixmap1();
    mp_str("cnvs_img", 8u);
    mp_fixarray(2u);
    mp_fixarray(4u);
    mp_uint(x);
    mp_uint(y);
    mp_uint(w);
    mp_uint(h);
    mp_bin(data, data_len);
    return send_framed();
}

tBleStatus FS_Vuzix_Client_SendFlush(void)
{
    /* lvgl_obj → {flush: nil} */
    mp_reset();
    mp_fixmap1();
    mp_str("lvgl_obj", 8u);
    mp_fixmap1();
    mp_str("flush", 5u);
    mp_nil();
    return send_framed();
}

/* ---- Discovery: start ---- */

/* Called on connection — saves handle, fires OnConnected callback.
 * Does NOT start GATT discovery; that happens after pairing completes. */
void FS_Vuzix_Client_OnConnected(uint16_t connHandle)
{
    const FS_Vuzix_ClientCb_t *saved_cb = g_ctx.cb;
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.cb            = saved_cb;
    g_ctx.connHandle    = connHandle;
    g_ctx.lastBattPct   = 255u;
    g_ctx.negotiatedMtu = 23u;
    vz_txq_reset();
    if (g_ctx.cb && g_ctx.cb->OnConnected)
        g_ctx.cb->OnConnected();
}

/* Called after pairing completes — starts MTU exchange + GATT discovery. */
void FS_Vuzix_Client_StartDiscovery(void)
{
    g_ctx.discState = DISC_EXCH_MTU;
    tBleStatus s = aci_gatt_exchange_config(g_ctx.connHandle);
    if (s != BLE_STATUS_SUCCESS) {
        APP_DBG_MSG("Vuzix_Client: MTU exchange fail=0x%02X\n", s);
        g_ctx.discState = DISC_IDLE;
    } else {
        APP_DBG_MSG("Vuzix_Client: MTU exchange requested\n");
    }
}

/* ---- Incoming notification parser (minimal) ---- */

static void handle_rx_notification(const uint8_t *data, uint8_t len)
{
    if (len < 2u) return;
    uint8_t header   = data[0];
    uint8_t has_more = (header >> 7u) & 0x01u;

    /* Only handle single-chunk messages for now */
    if (has_more) return;

    const uint8_t *mp = &data[1];
    uint8_t mplen = len - 1u;

    if (mplen < 2u || mp[0] != 0x81u) return;  /* must be fixmap(1) */

    uint8_t key_tag = mp[1];
    if ((key_tag & 0xE0u) == 0xA0u) {
        uint8_t klen = key_tag & 0x1Fu;
        if (mplen >= 2u + klen) {
            /* Log key name for debugging */
            char key[32];
            uint8_t copy = (klen < sizeof(key) - 1u) ? klen : (uint8_t)(sizeof(key) - 1u);
            memcpy(key, &mp[2], copy);
            key[copy] = '\0';
            APP_DBG_MSG("Vuzix_Client: RX event key='%s'\n", key);
        }
    }
}

/* ---- GATT event handler ---- */

void FS_Vuzix_Client_EventHandler(void *p_blecore_evt, uint8_t hci_event_evt_code)
{
    evt_blecore_aci *evt = (evt_blecore_aci *)p_blecore_evt;

    switch (evt->ecode)
    {
    /* ------------------------------------------------------------------ */
    case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
    {
        aci_att_exchange_mtu_resp_event_rp0 *mtu =
            (aci_att_exchange_mtu_resp_event_rp0 *)evt->data;
        g_ctx.negotiatedMtu = mtu->Server_RX_MTU;
        APP_DBG_MSG("Vuzix_Client: MTU=%d\n", g_ctx.negotiatedMtu);
    }
    break;

    /* ------------------------------------------------------------------ */
    case ACI_GATT_PROC_COMPLETE_VSEVT_CODE:
    {
        aci_gatt_proc_complete_event_rp0 *pc =
            (aci_gatt_proc_complete_event_rp0 *)evt->data;
        if (pc->Connection_Handle != g_ctx.connHandle) break;

        switch (g_ctx.discState)
        {
        case DISC_EXCH_MTU:
            g_ctx.discState = DISC_SVC;
            if (aci_gatt_disc_all_primary_services(g_ctx.connHandle) != BLE_STATUS_SUCCESS)
                g_ctx.discState = DISC_IDLE;
            else
                APP_DBG_MSG("Vuzix_Client: discovering services...\n");
            break;

        case DISC_SVC:
            if (g_ctx.svcFound) {
                g_ctx.discState = DISC_CHAR_VUZIX;
                if (aci_gatt_disc_all_char_of_service(g_ctx.connHandle,
                                                       g_ctx.svcStart, g_ctx.svcEnd)
                    != BLE_STATUS_SUCCESS)
                    g_ctx.discState = DISC_IDLE;
            } else if (g_ctx.battSvcFound) {
                g_ctx.discState = DISC_CHAR_BATT;
                aci_gatt_disc_all_char_of_service(g_ctx.connHandle,
                                                  g_ctx.battSvcStart, g_ctx.battSvcEnd);
            } else {
                APP_DBG_MSG("Vuzix_Client: no known services found\n");
                g_ctx.discState = DISC_IDLE;
            }
            break;

        case DISC_CHAR_VUZIX:
            if (g_ctx.battSvcFound) {
                g_ctx.discState = DISC_CHAR_BATT;
                aci_gatt_disc_all_char_of_service(g_ctx.connHandle,
                                                  g_ctx.battSvcStart, g_ctx.battSvcEnd);
            } else {
                /* Skip straight to RX descriptor discovery */
                goto discover_rx_desc;
            }
            break;

        case DISC_CHAR_BATT:
        discover_rx_desc:
            if (g_ctx.rxFound && g_ctx.rxHandle != 0u) {
                g_ctx.discState = DISC_DESC_RX;
                aci_gatt_disc_all_char_desc(g_ctx.connHandle,
                                             g_ctx.rxHandle,
                                             g_ctx.rxHandle + 2u);
            } else {
                goto finalize_discovery;
            }
            break;

        case DISC_DESC_RX:
            /* Enable RX notifications */
            if (g_ctx.rxCCCDHandle != 0u) {
                uint8_t cccd[2] = {0x01u, 0x00u};
                g_ctx.discState = DISC_NOTIFY_RX;
                aci_gatt_write_char_desc(g_ctx.connHandle, g_ctx.rxCCCDHandle, 2u, cccd);
            } else {
                goto finalize_discovery;
            }
            break;

        case DISC_NOTIFY_RX:
            /* Enable battery notifications if available */
            if (g_ctx.battCharFound && g_ctx.battCharHandle != 0u) {
                g_ctx.discState = DISC_DESC_BATT;
                aci_gatt_disc_all_char_desc(g_ctx.connHandle,
                                             g_ctx.battCharHandle,
                                             g_ctx.battCharHandle + 2u);
            } else {
                goto finalize_discovery;
            }
            break;

        case DISC_DESC_BATT:
            if (g_ctx.battCCCDHandle != 0u) {
                uint8_t cccd[2] = {0x01u, 0x00u};
                g_ctx.discState = DISC_NOTIFY_BATT;
                aci_gatt_write_char_desc(g_ctx.connHandle, g_ctx.battCCCDHandle, 2u, cccd);
            } else {
                goto finalize_discovery;
            }
            break;

        case DISC_NOTIFY_BATT:
        finalize_discovery:
            g_ctx.discState = DISC_IDLE;
            APP_DBG_MSG("Vuzix_Client: discovery done. TX=0x%04X RX=0x%04X\n",
                        g_ctx.txHandle, g_ctx.rxHandle);
            if (g_ctx.txFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                g_ctx.cb->OnDiscoveryComplete();
            break;

        default:
            break;
        }
    }
    break;

    /* ------------------------------------------------------------------ */
    case ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE:
    {
        if (g_ctx.discState != DISC_SVC) break;
        aci_att_read_by_group_type_resp_event_rp0 *pr =
            (aci_att_read_by_group_type_resp_event_rp0 *)evt->data;

        uint8_t idx = 0u;
        while (idx < pr->Data_Length) {
            uint16_t sHdl = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx]);
            uint16_t eHdl = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx + 2u]);
            const uint8_t *uuid = &pr->Attribute_Data_List[idx + 4u];

            if (pr->Attribute_Data_Length == 20u) {
                /* 128-bit UUID */
                if (memcmp(uuid, VUZIX_SVC_UUID, 16u) == 0) {
                    APP_DBG_MSG("Vuzix_Client: found main service 0x%04X-0x%04X\n", sHdl, eHdl);
                    g_ctx.svcFound = 1u;
                    g_ctx.svcStart = sHdl;
                    g_ctx.svcEnd   = eHdl;
                }
            } else if (pr->Attribute_Data_Length == 6u) {
                /* 16-bit UUID */
                uint16_t u16 = UNPACK_2_BYTE_PARAMETER(uuid);
                if (u16 == BATTERY_SVC_UUID) {
                    APP_DBG_MSG("Vuzix_Client: found battery service 0x%04X-0x%04X\n", sHdl, eHdl);
                    g_ctx.battSvcFound = 1u;
                    g_ctx.battSvcStart = sHdl;
                    g_ctx.battSvcEnd   = eHdl;
                }
            }
            idx += pr->Attribute_Data_Length;
        }
    }
    break;

    /* ------------------------------------------------------------------ */
    case ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE:
    {
        if (g_ctx.discState != DISC_CHAR_VUZIX && g_ctx.discState != DISC_CHAR_BATT) break;
        aci_att_read_by_type_resp_event_rp0 *pr =
            (aci_att_read_by_type_resp_event_rp0 *)evt->data;

        uint8_t idx = 0u;
        while (idx < pr->Data_Length) {
            /* format: 2B decl handle, 1B props, 2B value handle, UUID bytes */
            idx += 2u;  /* skip decl handle */
            idx += 1u;  /* skip properties */
            uint16_t valHdl = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
            idx += 2u;

            uint8_t uuidLen = pr->Handle_Value_Pair_Length - 5u;
            const uint8_t *uuid = &pr->Handle_Value_Pair_Data[idx];
            idx += uuidLen;

            if (g_ctx.discState == DISC_CHAR_VUZIX) {
                if (uuidLen == 16u) {
                    if (memcmp(uuid, VUZIX_TX_UUID, 16u) == 0) {
                        APP_DBG_MSG("Vuzix_Client: TX char=0x%04X\n", valHdl);
                        g_ctx.txFound  = 1u;
                        g_ctx.txHandle = valHdl;
                    } else if (memcmp(uuid, VUZIX_RX_UUID, 16u) == 0) {
                        APP_DBG_MSG("Vuzix_Client: RX char=0x%04X\n", valHdl);
                        g_ctx.rxFound  = 1u;
                        g_ctx.rxHandle = valHdl;
                    }
                }
            } else {
                /* Battery service */
                if (uuidLen == 2u) {
                    uint16_t u16 = UNPACK_2_BYTE_PARAMETER(uuid);
                    if (u16 == BATTERY_LEVEL_UUID) {
                        APP_DBG_MSG("Vuzix_Client: battery char=0x%04X\n", valHdl);
                        g_ctx.battCharFound  = 1u;
                        g_ctx.battCharHandle = valHdl;
                    }
                }
            }
        }
    }
    break;

    /* ------------------------------------------------------------------ */
    case ACI_ATT_FIND_INFO_RESP_VSEVT_CODE:
    {
        aci_att_find_info_resp_event_rp0 *resp =
            (aci_att_find_info_resp_event_rp0 *)evt->data;

        if (resp->Format == 0x01u) {  /* 16-bit UUIDs */
            uint8_t num = (resp->Event_Data_Length - 1u) / 4u;
            uint8_t *ptr = resp->Handle_UUID_Pair;
            for (uint8_t i = 0u; i < num; i++) {
                uint16_t hdl  = UNPACK_2_BYTE_PARAMETER(ptr); ptr += 2u;
                uint16_t uuid = UNPACK_2_BYTE_PARAMETER(ptr); ptr += 2u;
                if (uuid == CCCD_UUID) {
                    if (g_ctx.discState == DISC_DESC_RX && g_ctx.rxCCCDHandle == 0u) {
                        g_ctx.rxCCCDHandle = hdl;
                        APP_DBG_MSG("Vuzix_Client: RX CCCD=0x%04X\n", hdl);
                    } else if (g_ctx.discState == DISC_DESC_BATT && g_ctx.battCCCDHandle == 0u) {
                        g_ctx.battCCCDHandle = hdl;
                        APP_DBG_MSG("Vuzix_Client: battery CCCD=0x%04X\n", hdl);
                    }
                }
            }
        }
    }
    break;

    /* ------------------------------------------------------------------ */
    case ACI_GATT_NOTIFICATION_VSEVT_CODE:
    {
        aci_gatt_notification_event_rp0 *notif =
            (aci_gatt_notification_event_rp0 *)evt->data;
        if (notif->Connection_Handle != g_ctx.connHandle) break;

        if (notif->Attribute_Handle == g_ctx.rxHandle) {
            handle_rx_notification(notif->Attribute_Value,
                                   (uint8_t)notif->Attribute_Value_Length);
        } else if (notif->Attribute_Handle == g_ctx.battCharHandle) {
            if (notif->Attribute_Value_Length >= 1u) {
                g_ctx.lastBattPct = notif->Attribute_Value[0];
                APP_DBG_MSG("Vuzix_Client: battery=%d%%\n", g_ctx.lastBattPct);
            }
        }
    }
    break;

    /* ------------------------------------------------------------------ */
    case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
        s_txq_flow = 1u;
        UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_TX_ID, CFG_SCH_PRIO_1);
        /* If the ring buffer was drained before flow was restored, wake the
         * setup task so it can refill it to restart the upload. */
        if (s_fup.active)
            UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
        break;

    default:
        break;
    }
}
