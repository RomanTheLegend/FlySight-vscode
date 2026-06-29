/***************************************************************************
**  FlySight 2 firmware — Vuzix Z100 BLE client                           **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
****************************************************************************/

#ifndef VUZIX_CLIENT_H
#define VUZIX_CLIENT_H

#include <stdint.h>
#include "ble_types.h"

typedef struct {
    void (*OnDiscoveryComplete)(void);
} FS_Vuzix_ClientCb_t;

void    FS_Vuzix_Client_Init(void);
void    FS_Vuzix_Client_RegisterCb(const FS_Vuzix_ClientCb_t *cb);
void    FS_Vuzix_Client_StartDiscovery(uint16_t connectionHandle);
void    FS_Vuzix_Client_EventHandler(void *p_blecore_evt, uint8_t hci_event_evt_code);
uint8_t FS_Vuzix_Client_IsReady(void);
uint8_t FS_Vuzix_Client_GetBatteryLevel(void);

/* High-level MessagePack message builders + BLE write */
tBleStatus FS_Vuzix_Client_SendConnAppInit(void);
tBleStatus FS_Vuzix_Client_SendControl(void);
tBleStatus FS_Vuzix_Client_SendSetTime(void);
tBleStatus FS_Vuzix_Client_SendKeepalive(void);
tBleStatus FS_Vuzix_Client_SendText(const char *text);

#endif /* VUZIX_CLIENT_H */
