#include "main.h"
#include "config.h"
#include "rtc_util.h"
#include "sensor_time.h"
#include "time.h"

extern RTC_HandleTypeDef hrtc;

void FS_RTC_SetFromGNSS(const FS_GNSS_Time_t *gnssTime)
{
	uint32_t epoch;
	uint32_t timestamp;

	uint16_t year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  min;
	uint8_t  sec;
	uint16_t ms;

	uint64_t offset_us;
	uint32_t offset_ms;
	uint32_t ms_total;

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	if (gnssTime->week == 0) return;

	// Start of the year 2000 (gmtime epoch)
	epoch = 1042 * 7 * 24 * 3600 + 518400;

	// Calculate timestamp at start_time
	timestamp = gnssTime->week * 7 * 24 * 3600 - epoch;
	timestamp += gnssTime->towMS / 1000;

	// Calculate millisecond part of date/time
	ms = gnssTime->towMS % 1000;

	// Add the offset to milliseconds
	offset_us = FS_SensorTime_GetTicks() - gnssTime->time;
	offset_ms = (uint32_t)(offset_us / 1000);
	ms_total = ms + offset_ms;

	// Calculate new timestamp and milliseconds
	timestamp += ms_total / 1000;

	// Convert back to date/time
	gmtime_r(timestamp, &year, &month, &day, &hour, &min, &sec);

	// Update RTC
	sTime.Hours = hour;
	sTime.Minutes = min;
	sTime.Seconds = sec;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;

	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = month;
	sDate.Date = day;
	sDate.Year = year % 100;

	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	// Mark RTC as valid in backup register
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, RTC_VALID_MAGIC);
}

bool FS_RTC_IsValid(void)
{
	return (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == RTC_VALID_MAGIC);
}

void FS_RTC_AdjustToLocal(uint16_t *year, uint8_t *month, uint8_t *day,
		uint8_t *hour, uint8_t *min, uint8_t *sec)
{
	uint32_t timestamp;

	timestamp = mk_gmtime(*year, *month, *day, *hour, *min, *sec);
	timestamp += FS_Config_Get()->tz_offset;
	gmtime_r(timestamp, year, month, day, hour, min, sec);
}
