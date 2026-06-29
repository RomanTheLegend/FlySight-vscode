/***************************************************************************
**  FlySight 2 firmware — Vuzix Z100 competition mode                     **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
****************************************************************************/

#ifndef VUZIX_MODE1_H
#define VUZIX_MODE1_H

typedef enum {
    VZ_SETUP_IN_PROGRESS = 0,
    VZ_SETUP_DONE
} VZ_SetupStatus_t;

void             FS_Vuzix_Mode1_Init(void);
VZ_SetupStatus_t FS_Vuzix_Mode1_Setup(void);
void             FS_Vuzix_Mode1_Update(void);

#endif /* VUZIX_MODE1_H */
