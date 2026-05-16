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

#include <math.h>
#include <stdbool.h>

#include "main.h"
#include "app_common.h"
//#include "audio.h"
#include "activelook_control.h"
#include "common.h"
#include "config.h"
#include "stm32_seq.h"

#define CONSUMER_TIMER_MSEC    200
#define CONSUMER_TIMER_TICKS   (CONSUMER_TIMER_MSEC*1000/CFG_TS_TICK_VAL)
//
#define ABS(a) (((a) < 0) ? -(a) : (a))
//
#define INVALID_VALUE   INT32_MAX
//
#define ALT_MIN         1500L // Minimum announced altitude (m)

#define FLAG_HAS_FIX         0x01
#define FLAG_FIRST_FIX       0x02
#define FLAG_BEEP_DONE       0x04
#define FLAG_SAY_ALTITUDE    0x08
#define FLAG_VERTICAL_ACC    0x10

#define TONE_MIN_PITCH 220
#define TONE_MAX_PITCH 1760

static const uint16_t sas_table[] =
{
	1024, 1077, 1135, 1197,
	1265, 1338, 1418, 1505,
	1600, 1704, 1818, 1944
};

static uint8_t timer_id;

static uint8_t cur_speech;

static uint16_t sp_counter;

static uint8_t flags;
static uint8_t prev_flags;

static int32_t prevHMSL;

static uint8_t g_suppress_tone;

static char speech_buf[16];
static char *speech_ptr;

static volatile uint32_t tonePitch;
static volatile int32_t  toneChirp;
static volatile uint16_t toneRate;
static volatile uint8_t  toneHold;

static void getValues(
	FS_GNSS_Data_t *current,
	FS_Config_Data_t *config,
	uint8_t mode,
	int32_t *val,
	int32_t *min,
	int32_t *max)
{
	uint16_t speed_mul = 1024;

	if (config->use_sas)
	{
		if (current->hMSL < 0)
		{
			speed_mul = sas_table[0];
		}
		else if (current->hMSL >= 11534336L)
		{
			speed_mul = sas_table[11];
		}
		else
		{
			int32_t h = current->hMSL / 1024;
			uint16_t i = h / 1024;
			uint16_t j = h % 1024;
			uint16_t y1 = sas_table[i];
			uint16_t y2 = sas_table[i + 1];
			speed_mul = y1 + ((y2 - y1) * j) / 1024;
		}
	}

	switch (mode)
	{
	case 0: // Horizontal speed
		*val = (current->gSpeed * 1024) / speed_mul;
		break;
	case 1: // Vertical speed
		*val = (current->velD * 1024) / speed_mul;
		break;
	case 2: // Glide ratio
		if (current->velD != 0)
		{
			*val = 10000 * (int32_t) current->gSpeed / current->velD;
			*min *= 100;
			*max *= 100;
		}
		break;
	case 3: // Inverse glide ratio
		if (current->gSpeed != 0)
		{
			*val = 10000 * current->velD / (int32_t) current->gSpeed;
			*min *= 100;
			*max *= 100;
		}
		break;
	case 4: // Total speed
		*val = (current->speed * 1024) / speed_mul;
		break;
	case 11: // Dive angle
		*val = atan2(current->velD, current->gSpeed) / M_PI * 180;
		break;
	}
}



static void displayValue(
	FS_Config_Data_t *config,
	FS_GNSS_Data_t *current)
{
	uint16_t speed_mul = 1024;
	int32_t step_size, step;

	char *end_ptr;

	if (config->use_sas)
	{
		if (current->hMSL < 0)
		{
			speed_mul = sas_table[0];
		}
		else if (current->hMSL >= 11534336L)
		{
			speed_mul = sas_table[11];
		}
		else
		{
			int32_t h = current->hMSL / 1024;
			uint16_t i = h / 1024;
			uint16_t j = h % 1024;
			uint16_t y1 = sas_table[i];
			uint16_t y2 = sas_table[i + 1];
			speed_mul = y1 + ((y2 - y1) * j) / 1024;
		}
	}

	switch (config->speech[cur_speech].units)
	{
	case FS_CONFIG_UNITS_KMH:
		speed_mul = (uint16_t) (((uint32_t) speed_mul * 18204) / 65536);
		break;
	case FS_CONFIG_UNITS_MPH:
		speed_mul = (uint16_t) (((uint32_t) speed_mul * 29297) / 65536);
		break;
	}

	// Step 0: Initialize speech pointers, leaving room at the end for one unit character

	speech_ptr = speech_buf + sizeof(speech_buf) - 1;
	end_ptr = speech_ptr;

	// Step 1: Get speech value with 2 decimal places

	switch (config->speech[cur_speech].mode)
	{
	case 0: // Horizontal speed
		speech_ptr = writeInt32ToBuf(speech_ptr, (current->gSpeed * 1024) / speed_mul, 2, 1, 0);
		break;
	case 1: // Vertical speed
		speech_ptr = writeInt32ToBuf(speech_ptr, (current->velD * 1024) / speed_mul, 2, 1, 0);
		break;
	case 2: // Glide ratio
		if (current->velD != 0)
		{
			speech_ptr = writeInt32ToBuf(speech_ptr, 100 * (int32_t) current->gSpeed / current->velD, 2, 1, 0);
		}
		else
		{
			*(--speech_ptr) = '\0';
		}
		break;
	case 3: // Inverse glide ratio
		if (current->gSpeed != 0)
		{
			speech_ptr = writeInt32ToBuf(speech_ptr, 100 * (int32_t) current->velD / current->gSpeed, 2, 1, 0);
		}
		else
		{
			*(--speech_ptr) = '\0';
		}
		break;
	case 4: // Total speed
		speech_ptr = writeInt32ToBuf(speech_ptr, (current->speed * 1024) / speed_mul, 2, 1, 0);
		break;
	case 11: // Dive angle
		speech_ptr = writeInt32ToBuf(speech_ptr, 100 * atan2(current->velD, current->gSpeed) / M_PI * 180, 2, 1, 0);
		break;
	// case 12: // Altitude
	// 	if (config->speech[cur_speech].units == FS_CONFIG_UNITS_KMH)
	// 	{
	// 		step_size = 10000 * config->speech[cur_speech].decimals;
	// 	}
	// 	else
	// 	{
	// 		step_size = 3048 * config->speech[cur_speech].decimals;
	// 	}
	// 	step = ((current->hMSL - config->dz_elev) * 10 + step_size / 2) / step_size;
	// 	speech_ptr = speech_buf + 2;
	// 	speech_ptr = numberToSpeech(step * config->speech[cur_speech].decimals, speech_ptr);
	// 	end_ptr = speech_ptr;
	// 	speech_ptr = speech_buf + 2;
	// 	break;
	}

	// Step 1.5: Include label
	if (config->num_speech > 1)
	{
		*(--speech_ptr) = config->speech[cur_speech].mode + 1;
		*(--speech_ptr) = '>';
	}

	// Step 2: Truncate to the desired number of decimal places

	if (config->speech[cur_speech].mode != 5)
	{
		if (config->speech[cur_speech].decimals == 0) end_ptr -= 4;
		else end_ptr -= 3 - config->speech[cur_speech].decimals;
	}

	// Step 3: Add units if needed, e.g., *(end_ptr++) = 'k';

	switch (config->speech[cur_speech].mode)
	{
	case 0: // Horizontal speed
	case 1: // Vertical speed
	case 2: // Glide ratio
	case 3: // Inverse glide ratio
	case 4: // Total speed
	case 11: // Dive angle
		break;
	case 12: // Altitude
		*(end_ptr++) = (config->speech[cur_speech].units == FS_CONFIG_UNITS_KMH) ? 'm' : 'f';
		break;
	}

	// Step 4: Terminate with a null

	*(end_ptr++) = '\0';
}


static void producerTask(void)
{
	FS_Config_Data_t config;
	FS_GNSS_Data_t current;

	// Copy to local variable
	memcpy(&config, FS_Config_Get(), sizeof(FS_Config_Data_t));
	memcpy(&current, FS_GNSS_GetData(), sizeof(FS_GNSS_Data_t));

	if (current.gpsFix == 3)
	{
		flags |= FLAG_HAS_FIX;


		if (!(flags & FLAG_BEEP_DONE))
		{
			flags |= FLAG_FIRST_FIX;
		}
	}
	else
	{
		flags &= ~FLAG_HAS_FIX;
//		setRate(0);
	}

	if (current.vAcc < 10000)
	{
		flags |= FLAG_VERTICAL_ACC;
	}
	else
	{
		flags &= ~FLAG_VERTICAL_ACC;
	}

	prev_flags = flags;
	prevHMSL = current.hMSL;
}

static void consumerTimer(void)
{
	static uint16_t tone_timer = 0;
	const FS_Config_Data_t *config = FS_Config_Get();

//	if (FS_Audio_IsIdle() && !toneHold && toneRate > 0 && 0x10000 - tone_timer <= toneRate)
//	{
//		FS_Audio_Beep(tonePitch, tonePitch + toneChirp, 125, config->volume * 5);
//	}
//
//	tone_timer += toneRate;

	// Call consumer task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_ACTIVELOOK_CONTROL_CONSUMER_ID, CFG_SCH_PRIO_0);
}

static void consumerTask(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();

	UTIL_SEQ_SetTask(1<<CFG_TASK_ACTIVELOOK_DISPLAY_UPDATE_ID, CFG_SCH_PRIO_0);


}

void FS_ActivelookControl_Init(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();
	uint8_t i;
	char filename[13];

	// Initialize state
	cur_speech = 0;
	sp_counter = 0;
	flags = 0;
	prev_flags = 0;
	g_suppress_tone = 0;
	speech_buf[0] = '\0';
	speech_ptr = speech_buf;
	tonePitch = 0;
	toneChirp = 0;
	toneRate = 0;
	toneHold = 0;

	// Initialize producer task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_ACTIVELOOK_CONTROL_PRODUCER_ID, UTIL_SEQ_RFU, producerTask);

	// Initialize consumer task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_ACTIVELOOK_CONTROL_CONSUMER_ID, UTIL_SEQ_RFU, consumerTask);

	// Initialize consumer timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, consumerTimer);
	HW_TS_Start(timer_id, CONSUMER_TIMER_TICKS);

	if (config->alt_step > 0)
	{
		flags |= FLAG_SAY_ALTITUDE;
	}

	for (i = 0; i < config->num_speech; ++i)
	{
		if (config->speech[i].mode == 5)
		{
			flags |= FLAG_SAY_ALTITUDE;
		}
	}


}

void FS_ActivelookControl_DeInit(void)
{
	// Delete update timer
	HW_TS_Delete(timer_id);
}

void FS_ActivelookControl_UpdateGNSS(const FS_GNSS_Data_t *current)
{
	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_ACTIVELOOK_CONTROL_PRODUCER_ID, CFG_SCH_PRIO_0);
}
