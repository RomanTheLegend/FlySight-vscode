/***************************************************************************
**  FlySight 2 firmware — Vuzix Z100 top-level driver                     **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
****************************************************************************/

#ifndef VUZIX_H
#define VUZIX_H

#include <stdint.h>

void FS_Vuzix_Init(void);
void FS_Vuzix_DeInit(void);
void FS_Vuzix_HandleDisconnect(void);
void FS_Vuzix_OnPairingComplete(uint8_t status);

#endif /* VUZIX_H */
