/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2023 Bionic Avionics Inc.                                   **
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

#ifndef USB_STORAGE_CACHE_H_
#define USB_STORAGE_CACHE_H_

#include <stdint.h>

int8_t FS_USBStorageCache_Init(void);
int8_t FS_USBStorageCache_DeInit(void);
void FS_USBStorageCache_Reset(void);
void FS_USBStorageCache_SetCapacity(uint32_t block_count);
void FS_USBStorageCache_SetActivityCallbacks(void (*begin)(void), void (*end)(void));
int8_t FS_USBStorageCache_Read(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t FS_USBStorageCache_Write(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t FS_USBStorageCache_Flush(void);

#endif /* USB_STORAGE_CACHE_H_ */
