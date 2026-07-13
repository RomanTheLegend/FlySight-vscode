/***************************************************************************
**  FlySight 2 firmware — Vuzix Z100 BLE client                           **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
****************************************************************************/

#ifndef VUZIX_CLIENT_H
#define VUZIX_CLIENT_H

#include <stdint.h>
#include "ble_types.h"

typedef struct {
    void (*OnConnected)(void);          /* called after connection, before pairing */
    void (*OnDiscoveryComplete)(void);  /* called after GATT discovery finishes    */
} FS_Vuzix_ClientCb_t;

void     FS_Vuzix_Client_Init(void);
void     FS_Vuzix_Client_RegisterCb(const FS_Vuzix_ClientCb_t *cb);
void     FS_Vuzix_Client_OnConnected(uint16_t connHandle);
void     FS_Vuzix_Client_StartDiscovery(void);
void     FS_Vuzix_Client_EventHandler(void *p_blecore_evt, uint8_t hci_event_evt_code);
uint8_t  FS_Vuzix_Client_IsReady(void);
uint8_t  FS_Vuzix_Client_GetBatteryLevel(void);
uint16_t FS_Vuzix_Client_GetConnHandle(void);

/* High-level MessagePack message builders + BLE write */
tBleStatus FS_Vuzix_Client_SendConnAppInit(void);
tBleStatus FS_Vuzix_Client_SendControl(uint8_t layout_id);
tBleStatus FS_Vuzix_Client_SendSetTime(void);
tBleStatus FS_Vuzix_Client_SendKeepalive(void);
tBleStatus FS_Vuzix_Client_SendText(const char *text);

/* LVGL canvas / label primitives (LAYOUT_CUSTOM mode) */
/* label_align=1 (LV_ALIGN_TOP_LEFT), color=3 (white) are hardcoded */
tBleStatus FS_Vuzix_Client_SendLabel(uint8_t idx, int16_t x, int16_t y,
                                     uint8_t show, const char *text);
/* Upload an LVGL v8 binary font (.fbin) to a persistent glasses font slot (0-3).
 * Call repeatedly until BLE_STATUS_SUCCESS — returns BLE_STATUS_FAILED while in progress. */
tBleStatus FS_Vuzix_Client_SendFontSlot(uint8_t slot_idx,
                                         const uint8_t *data, uint16_t len);
/* Render a label using a previously uploaded font slot instead of the global font. */
tBleStatus FS_Vuzix_Client_SendLabelFont(uint8_t idx, uint8_t font_slot_idx,
                                          int16_t x, int16_t y,
                                          uint8_t show, const char *text);
/* color: 2-bit palette index (0=black, 1=dark grey, 2=light grey, 3=white) */
tBleStatus FS_Vuzix_Client_SendCanvasRect(uint16_t x0, uint16_t y0,
                                          uint16_t x1, uint16_t y1,
                                          uint8_t color);
/* data: raw 2-bit indexed pixels, 4px/byte, (w*h+3)/4 bytes, no row padding */
tBleStatus FS_Vuzix_Client_SendCanvasImg(uint16_t x, uint16_t y,
                                         uint8_t w, uint8_t h,
                                         const uint8_t *data, uint8_t data_len);
tBleStatus FS_Vuzix_Client_SendFlush(void);

#endif /* VUZIX_CLIENT_H */
