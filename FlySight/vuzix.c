/***************************************************************************
**  FlySight 2 firmware — Vuzix Z100 top-level driver                     **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
**                                                                        **
**  Mirrors activelook.c: state machine that drives the init sequence     **
**  and periodic display updates once the BLE connection is up.           **
****************************************************************************/

#include "vuzix.h"
#include "vuzix_client.h"
#include "vuzix_mode1.h"
#include "app_common.h"
#include "ble.h"
#include "config.h"
#include "dbg_trace.h"
#include "stm32_seq.h"

/* ---- State machine ---- */

typedef enum {
    VZ_STATE_INIT = 0,
    VZ_STATE_PAIR,        /* initiate Just Works pairing, wait for complete */
    VZ_STATE_DISC,        /* start GATT discovery after pairing             */
    VZ_STATE_CONN_INIT,   /* send conn_app_init: [2]                        */
    VZ_STATE_SET_TIME,    /* send set_time: "YYYYMMDDTHHmmss"               */
    VZ_STATE_CONTROL,     /* send control: request TEXT_BOTTOM_LEFT layout   */
    VZ_STATE_SETUP,       /* call mode->setup() until DONE                   */
    VZ_STATE_READY,       /* idle, waiting for timer                         */
    VZ_STATE_UPDATE,      /* timer fired — call mode->update()               */
} VZ_State_t;

static VZ_State_t s_state = VZ_STATE_INIT;
static uint8_t    s_timer_id;
static uint32_t   s_keepalive_accum_ms = 0u;

#define VZ_KEEPALIVE_INTERVAL_MS  3000u

/* ---- Mode ops table (same contract as activelook.c) ---- */

typedef struct {
    uint8_t            layout;   /* control layout_id sent to glasses */
    void               (*init)(void);
    VZ_SetupStatus_t   (*setup)(void);
    void               (*update)(void);
} VZ_ModeOps_t;

static const VZ_ModeOps_t s_modeTable[] =
{
    {   /* all hud_mode values: competition mode with LVGL canvas */
        4u,  /* LAYOUT_CUSTOM */
        FS_Vuzix_Mode1_Init,
        FS_Vuzix_Mode1_Setup,
        FS_Vuzix_Mode1_Update,
    },
};

#define VZ_NUM_MODES  (sizeof(s_modeTable) / sizeof(s_modeTable[0]))
static const VZ_ModeOps_t *s_mode = NULL;

/* Forward declarations */
static void FS_Vuzix_Task(void);
static void FS_Vuzix_TimerCb(void);
static void OnConnected(void);
static void OnDiscoveryComplete(void);
static void OnPairingComplete(uint8_t status);

static const FS_Vuzix_ClientCb_t s_vz_cb = {
    .OnConnected         = OnConnected,
    .OnDiscoveryComplete = OnDiscoveryComplete,
};

/* ---- Connection/discovery/pairing callbacks ---- */

/* Glasses connected — initiate pairing before any GATT operations. */
static void OnConnected(void)
{
    APP_DBG_MSG("Vuzix: connected, initiating pairing\n");
    s_state = VZ_STATE_PAIR;
    UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
}

/* GATT discovery finished — proceed with app init sequence. */
static void OnDiscoveryComplete(void)
{
    APP_DBG_MSG("Vuzix: discovery complete, starting app init\n");
    s_state = VZ_STATE_CONN_INIT;
    UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
}

static void OnPairingComplete(uint8_t status)
{
    if (status == 0u) {
        APP_DBG_MSG("Vuzix: pairing OK, starting GATT discovery\n");
        s_state = VZ_STATE_DISC;
        UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
    } else {
        APP_DBG_MSG("Vuzix: pairing failed (0x%02X), disconnecting\n", status);
        s_state = VZ_STATE_INIT;
        UTIL_SEQ_SetTask(1u << CFG_TASK_DISCONN_DEV_1_ID, CFG_SCH_PRIO_0);
    }
}

/* ---- Main task ---- */

static void FS_Vuzix_Task(void)
{
    switch (s_state)
    {
    case VZ_STATE_INIT:
        break;

    case VZ_STATE_PAIR:
        /* Wait for STM32WB stack to auto-pair in response to the glasses'
         * Security Request (~1500 ms delay).  ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE
         * will fire and advance state to VZ_STATE_DISC via FS_Vuzix_OnPairingComplete. */
        break;

    case VZ_STATE_DISC:
        FS_Vuzix_Client_StartDiscovery();
        /* OnDiscoveryComplete will advance state to VZ_STATE_CONN_INIT */
        break;

    case VZ_STATE_CONN_INIT:
        if (FS_Vuzix_Client_SendConnAppInit() != BLE_STATUS_SUCCESS) {
            UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
            break;
        }
        s_state = VZ_STATE_SET_TIME;
        UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
        break;

    case VZ_STATE_SET_TIME:
        if (FS_Vuzix_Client_SendSetTime() != BLE_STATUS_SUCCESS) {
            UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
            break;
        }
        s_state = VZ_STATE_CONTROL;
        UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
        break;

    case VZ_STATE_CONTROL:
        /* Select mode on first entry so we can pass the right layout_id */
        if (s_mode == NULL) {
            uint8_t mi = FS_Config_Get()->hud_mode;
            mi = (mi > 0u) ? (mi - 1u) : 0u;
            if (mi >= VZ_NUM_MODES) mi = (uint8_t)(VZ_NUM_MODES - 1u);
            s_mode = &s_modeTable[mi];
        }
        if (FS_Vuzix_Client_SendControl(s_mode->layout) != BLE_STATUS_SUCCESS) {
            UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
            break;
        }
        if (s_mode->init) s_mode->init();
        s_state = VZ_STATE_SETUP;
        UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
        break;

    case VZ_STATE_SETUP:
        if (s_mode && s_mode->setup) {
            VZ_SetupStatus_t st = s_mode->setup();
            if (st == VZ_SETUP_DONE) {
                APP_DBG_MSG("Vuzix: setup done, starting updates\n");
                s_state = VZ_STATE_READY;
                s_keepalive_accum_ms = 0u;
                HW_TS_Start(s_timer_id,
                            FS_Config_Get()->hud_rate * 1000u / CFG_TS_TICK_VAL);
            }
            /* On IN_PROGRESS do NOT self-reschedule: the TX drain task wakes us
             * after each slot is sent so it can run between upload chunks. */
        }
        break;

    case VZ_STATE_READY:
        break;

    case VZ_STATE_UPDATE:
        if (s_mode && s_mode->update) s_mode->update();

        s_keepalive_accum_ms += FS_Config_Get()->hud_rate;
        if (s_keepalive_accum_ms >= VZ_KEEPALIVE_INTERVAL_MS) {
            FS_Vuzix_Client_SendKeepalive();
            s_keepalive_accum_ms = 0u;
        }

        s_state = VZ_STATE_READY;
        break;
    }
}

/* ---- Timer callback (ISR context) ---- */

static void FS_Vuzix_TimerCb(void)
{
    if (s_state == VZ_STATE_READY) {
        s_state = VZ_STATE_UPDATE;
        UTIL_SEQ_SetTask(1u << CFG_TASK_FS_VUZIX_ID, CFG_SCH_PRIO_0);
    }
}

/* ---- Public API ---- */

void FS_Vuzix_Init(void)
{
    FS_Vuzix_Client_Init();
    FS_Vuzix_Client_RegisterCb(&s_vz_cb);

    UTIL_SEQ_RegTask(1u << CFG_TASK_FS_VUZIX_ID, UTIL_SEQ_RFU, FS_Vuzix_Task);

    s_state = VZ_STATE_INIT;

    /* Reuse the same scan task — scanning is shared with ActiveLook */
    UTIL_SEQ_SetTask(1u << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);

    HW_TS_Create(CFG_TIM_PROC_ID_ISR, &s_timer_id, hw_ts_Repeated, FS_Vuzix_TimerCb);
}

void FS_Vuzix_DeInit(void)
{
    s_state = VZ_STATE_INIT;
    HW_TS_Delete(s_timer_id);
    UTIL_SEQ_SetTask(1u << CFG_TASK_DISCONN_DEV_1_ID, CFG_SCH_PRIO_0);
}

void FS_Vuzix_HandleDisconnect(void)
{
    if (s_state == VZ_STATE_PAIR) {
        /* Disconnected before pairing completed — glasses did not accept our bond
         * (e.g. after glasses firmware update or bond reset on their side).
         * Clear the stale bond so the next Scan_Request does a full scan and
         * re-pairs from scratch. */
        APP_DBG_MSG("Vuzix: pre-pairing disconnect, clearing stale bond\n");
        aci_gap_clear_security_db();
    }
    s_state = VZ_STATE_INIT;
    s_mode  = NULL;
    HW_TS_Stop(s_timer_id);
    UTIL_SEQ_SetTask(1u << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
}

void FS_Vuzix_OnPairingComplete(uint8_t status)
{
    /* Only meaningful while we are waiting for the pairing handshake */
    if (s_state == VZ_STATE_PAIR)
        OnPairingComplete(status);
}
