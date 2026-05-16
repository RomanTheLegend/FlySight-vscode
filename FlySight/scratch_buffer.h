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

#ifndef SCRATCH_BUFFER_H_
#define SCRATCH_BUFFER_H_

#include <stdint.h>

#define FS_SCRATCH_BUFFER_SIZE 32768U

typedef enum
{
	FS_SCRATCH_BUFFER_OWNER_NONE = 0,
	FS_SCRATCH_BUFFER_OWNER_SENSOR_BATCH,
	FS_SCRATCH_BUFFER_OWNER_USB_STORAGE_CACHE
} FS_ScratchBufferOwner_t;

uint8_t *FS_ScratchBuffer_Acquire(FS_ScratchBufferOwner_t owner);
void FS_ScratchBuffer_Release(FS_ScratchBufferOwner_t owner);
uint8_t *FS_ScratchBuffer_Get(void);
FS_ScratchBufferOwner_t FS_ScratchBuffer_GetOwner(void);

#endif /* SCRATCH_BUFFER_H_ */
