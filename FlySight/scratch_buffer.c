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

#include "scratch_buffer.h"

static uint32_t scratchBuffer[FS_SCRATCH_BUFFER_SIZE / sizeof(uint32_t)];
static FS_ScratchBufferOwner_t scratchBufferOwner = FS_SCRATCH_BUFFER_OWNER_NONE;

uint8_t *FS_ScratchBuffer_Acquire(FS_ScratchBufferOwner_t owner)
{
	if (owner == FS_SCRATCH_BUFFER_OWNER_NONE)
	{
		return 0;
	}

	if ((scratchBufferOwner != FS_SCRATCH_BUFFER_OWNER_NONE)
			&& (scratchBufferOwner != owner))
	{
		return 0;
	}

	scratchBufferOwner = owner;
	return (uint8_t *) scratchBuffer;
}

void FS_ScratchBuffer_Release(FS_ScratchBufferOwner_t owner)
{
	if (scratchBufferOwner == owner)
	{
		scratchBufferOwner = FS_SCRATCH_BUFFER_OWNER_NONE;
	}
}

uint8_t *FS_ScratchBuffer_Get(void)
{
	return (uint8_t *) scratchBuffer;
}

FS_ScratchBufferOwner_t FS_ScratchBuffer_GetOwner(void)
{
	return scratchBufferOwner;
}
