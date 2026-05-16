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

#include <stdbool.h>
#include <string.h>

#include "scratch_buffer.h"
#include "stm32_adafruit_sd.h"
#include "usb_storage_cache.h"

#define USB_STORAGE_BLOCK_SIZE        512U
#define USB_STORAGE_CACHE_BLOCK_COUNT (FS_SCRATCH_BUFFER_SIZE / USB_STORAGE_BLOCK_SIZE)
#define USB_STORAGE_WAIT_TIMEOUT      100000U

typedef enum
{
	USB_STORAGE_CACHE_INVALID = 0,
	USB_STORAGE_CACHE_READ,
	USB_STORAGE_CACHE_WRITE
} FS_USBStorageCacheMode_t;

static FS_USBStorageCacheMode_t cacheMode = USB_STORAGE_CACHE_INVALID;
static uint32_t cacheBaseBlock = 0;
static uint16_t cacheBlockCount = 0;
static uint64_t cacheDirtyMask = 0;
static uint32_t cacheTotalBlocks = 0;
static bool cacheActive = false;

static void (*beginActivityCallback)(void) = 0;
static void (*endActivityCallback)(void) = 0;

static uint64_t FS_USBStorageCache_BlockMask(uint16_t block_count)
{
	if (block_count >= USB_STORAGE_CACHE_BLOCK_COUNT)
	{
		return UINT64_MAX;
	}

	return (1ULL << block_count) - 1ULL;
}

static uint32_t FS_USBStorageCache_BaseBlock(uint32_t blk_addr)
{
	return blk_addr - (blk_addr % USB_STORAGE_CACHE_BLOCK_COUNT);
}

static uint16_t FS_USBStorageCache_BlockCount(uint32_t baseBlock)
{
	uint32_t remainingBlocks;

	if (baseBlock >= cacheTotalBlocks)
	{
		return 0;
	}

	remainingBlocks = cacheTotalBlocks - baseBlock;

	if (remainingBlocks > USB_STORAGE_CACHE_BLOCK_COUNT)
	{
		remainingBlocks = USB_STORAGE_CACHE_BLOCK_COUNT;
	}

	return (uint16_t) remainingBlocks;
}

static int8_t FS_USBStorageCache_EnsureCapacity(void)
{
	SD_CardInfo info;

	if (!cacheActive ||
			(FS_ScratchBuffer_GetOwner() != FS_SCRATCH_BUFFER_OWNER_USB_STORAGE_CACHE))
	{
		return -1;
	}

	if (cacheTotalBlocks != 0U)
	{
		return 0;
	}

	if (BSP_SD_GetCardInfo(&info) != BSP_SD_OK)
	{
		return -1;
	}

	cacheTotalBlocks = info.LogBlockNbr;

	return 0;
}

static int8_t FS_USBStorageCache_WaitCardReady(void)
{
	uint32_t timeout = USB_STORAGE_WAIT_TIMEOUT;

	while (BSP_SD_GetCardState() != BSP_SD_OK)
	{
		if (timeout-- == 0U)
		{
			return -1;
		}
	}

	return 0;
}

static int8_t FS_USBStorageCache_ReadBlocks(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
	uint8_t res;

	if (blk_len == 0U)
	{
		return 0;
	}

	if (beginActivityCallback) beginActivityCallback();
	res = BSP_SD_ReadBlocks((uint32_t *) buf, blk_addr, blk_len, SD_DATATIMEOUT);
	if (endActivityCallback) endActivityCallback();

	if (res != BSP_SD_OK)
	{
		return -1;
	}

	return FS_USBStorageCache_WaitCardReady();
}

static int8_t FS_USBStorageCache_WriteBlocks(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
	uint8_t res;

	if (blk_len == 0U)
	{
		return 0;
	}

	if (beginActivityCallback) beginActivityCallback();
	res = BSP_SD_WriteBlocks((uint32_t *) buf, blk_addr, blk_len, SD_DATATIMEOUT);
	if (endActivityCallback) endActivityCallback();

	if (res != BSP_SD_OK)
	{
		return -1;
	}

	return FS_USBStorageCache_WaitCardReady();
}

static int8_t FS_USBStorageCache_LoadReadCache(uint32_t blk_addr)
{
	uint32_t baseBlock = FS_USBStorageCache_BaseBlock(blk_addr);

	if ((cacheMode == USB_STORAGE_CACHE_READ) && (cacheBaseBlock == baseBlock))
	{
		return 0;
	}

	if (FS_USBStorageCache_Flush() < 0)
	{
		return -1;
	}

	cacheMode = USB_STORAGE_CACHE_INVALID;
	cacheBaseBlock = baseBlock;
	cacheBlockCount = FS_USBStorageCache_BlockCount(baseBlock);
	cacheDirtyMask = 0;

	if (cacheBlockCount == 0U)
	{
		return -1;
	}

	if (FS_USBStorageCache_ReadBlocks(FS_ScratchBuffer_Get(),
			cacheBaseBlock,
			cacheBlockCount) < 0)
	{
		FS_USBStorageCache_Reset();
		return -1;
	}

	cacheMode = USB_STORAGE_CACHE_READ;

	return 0;
}

static int8_t FS_USBStorageCache_EnsureWriteCache(uint32_t blk_addr)
{
	uint32_t baseBlock = FS_USBStorageCache_BaseBlock(blk_addr);

	if ((cacheMode == USB_STORAGE_CACHE_WRITE) && (cacheBaseBlock == baseBlock))
	{
		return 0;
	}

	if (FS_USBStorageCache_Flush() < 0)
	{
		return -1;
	}

	cacheMode = USB_STORAGE_CACHE_WRITE;
	cacheBaseBlock = baseBlock;
	cacheBlockCount = FS_USBStorageCache_BlockCount(baseBlock);
	cacheDirtyMask = 0;

	if (cacheBlockCount == 0U)
	{
		FS_USBStorageCache_Reset();
		return -1;
	}

	return 0;
}

int8_t FS_USBStorageCache_Init(void)
{
	if (FS_ScratchBuffer_Acquire(FS_SCRATCH_BUFFER_OWNER_USB_STORAGE_CACHE) == 0)
	{
		return -1;
	}

	cacheActive = true;

	if (FS_USBStorageCache_Flush() < 0)
	{
		cacheActive = false;
		FS_ScratchBuffer_Release(FS_SCRATCH_BUFFER_OWNER_USB_STORAGE_CACHE);
		return -1;
	}

	FS_USBStorageCache_Reset();

	return 0;
}

int8_t FS_USBStorageCache_DeInit(void)
{
	int8_t res = FS_USBStorageCache_Flush();

	FS_USBStorageCache_Reset();
	cacheActive = false;
	FS_ScratchBuffer_Release(FS_SCRATCH_BUFFER_OWNER_USB_STORAGE_CACHE);

	return res;
}

void FS_USBStorageCache_Reset(void)
{
	cacheMode = USB_STORAGE_CACHE_INVALID;
	cacheBaseBlock = 0;
	cacheBlockCount = 0;
	cacheDirtyMask = 0;
	cacheTotalBlocks = 0;
}

void FS_USBStorageCache_SetCapacity(uint32_t block_count)
{
	cacheTotalBlocks = block_count;
}

void FS_USBStorageCache_SetActivityCallbacks(void (*begin)(void), void (*end)(void))
{
	beginActivityCallback = begin;
	endActivityCallback = end;
}

int8_t FS_USBStorageCache_Read(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
	uint16_t blocksLeft = blk_len;

	if (FS_USBStorageCache_EnsureCapacity() < 0)
	{
		return -1;
	}

	while (blocksLeft > 0U)
	{
		uint32_t offset;
		uint16_t chunkBlocks;

		if (FS_USBStorageCache_LoadReadCache(blk_addr) < 0)
		{
			return -1;
		}

		offset = blk_addr - cacheBaseBlock;
		if (offset >= cacheBlockCount)
		{
			return -1;
		}

		chunkBlocks = (uint16_t) (cacheBlockCount - offset);
		if (chunkBlocks > blocksLeft)
		{
			chunkBlocks = blocksLeft;
		}

		memcpy(buf, FS_ScratchBuffer_Get() + (offset * USB_STORAGE_BLOCK_SIZE),
				chunkBlocks * USB_STORAGE_BLOCK_SIZE);

		buf += chunkBlocks * USB_STORAGE_BLOCK_SIZE;
		blk_addr += chunkBlocks;
		blocksLeft -= chunkBlocks;
	}

	return 0;
}

int8_t FS_USBStorageCache_Write(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
	uint16_t blocksLeft = blk_len;

	if (FS_USBStorageCache_EnsureCapacity() < 0)
	{
		return -1;
	}

	while (blocksLeft > 0U)
	{
		uint32_t offset;
		uint16_t chunkBlocks;

		if (FS_USBStorageCache_EnsureWriteCache(blk_addr) < 0)
		{
			return -1;
		}

		offset = blk_addr - cacheBaseBlock;
		if (offset >= cacheBlockCount)
		{
			return -1;
		}

		chunkBlocks = (uint16_t) (cacheBlockCount - offset);
		if (chunkBlocks > blocksLeft)
		{
			chunkBlocks = blocksLeft;
		}

		memcpy(FS_ScratchBuffer_Get() + (offset * USB_STORAGE_BLOCK_SIZE), buf,
				chunkBlocks * USB_STORAGE_BLOCK_SIZE);

		for (uint16_t i = 0; i < chunkBlocks; ++i)
		{
			cacheDirtyMask |= 1ULL << (offset + i);
		}

		if (cacheDirtyMask == FS_USBStorageCache_BlockMask(cacheBlockCount))
		{
			if (FS_USBStorageCache_Flush() < 0)
			{
				return -1;
			}
		}

		buf += chunkBlocks * USB_STORAGE_BLOCK_SIZE;
		blk_addr += chunkBlocks;
		blocksLeft -= chunkBlocks;
	}

	return 0;
}

int8_t FS_USBStorageCache_Flush(void)
{
	uint8_t *cache;
	uint16_t block = 0;

	if ((cacheMode != USB_STORAGE_CACHE_WRITE) || (cacheDirtyMask == 0U))
	{
		return 0;
	}

	if (!cacheActive ||
			(FS_ScratchBuffer_GetOwner() != FS_SCRATCH_BUFFER_OWNER_USB_STORAGE_CACHE))
	{
		return -1;
	}

	cache = FS_ScratchBuffer_Get();

	while (block < cacheBlockCount)
	{
		uint16_t startBlock;
		uint16_t blockCount;

		while ((block < cacheBlockCount)
				&& ((cacheDirtyMask & (1ULL << block)) == 0U))
		{
			++block;
		}

		startBlock = block;

		while ((block < cacheBlockCount)
				&& ((cacheDirtyMask & (1ULL << block)) != 0U))
		{
			++block;
		}

		blockCount = block - startBlock;
		if (blockCount == 0U)
		{
			continue;
		}

		if (FS_USBStorageCache_WriteBlocks(cache + (startBlock * USB_STORAGE_BLOCK_SIZE),
				cacheBaseBlock + startBlock,
				blockCount) < 0)
		{
			return -1;
		}

		for (uint16_t i = 0; i < blockCount; ++i)
		{
			cacheDirtyMask &= ~(1ULL << (startBlock + i));
		}
	}

	return 0;
}
