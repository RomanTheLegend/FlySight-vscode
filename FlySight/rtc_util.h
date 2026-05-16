#ifndef RTC_UTIL_H_
#define RTC_UTIL_H_

#include <stdbool.h>
#include <stdint.h>
#include "gnss.h"

#define RTC_VALID_MAGIC  0x32F5A100

void FS_RTC_SetFromGNSS(const FS_GNSS_Time_t *gnssTime);
bool FS_RTC_IsValid(void);
void FS_RTC_AdjustToLocal(uint16_t *year, uint8_t *month, uint8_t *day,
		uint8_t *hour, uint8_t *min, uint8_t *sec);

#endif /* RTC_UTIL_H_ */
