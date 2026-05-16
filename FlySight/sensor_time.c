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

#include "main.h"
#include "sensor_time.h"

static volatile uint32_t overflow_count;

#define SENSOR_TIME_TICK_HZ 1000000U

static uint32_t FS_SensorTime_GetTIM2ClockHz(void)
{
	uint32_t tim2_clock_hz = HAL_RCC_GetPCLK1Freq();

	if (LL_RCC_GetAPB1Prescaler() != LL_RCC_APB1_DIV_1)
	{
		tim2_clock_hz *= 2U;
	}

	return tim2_clock_hz;
}

void FS_SensorTime_Init(void)
{
	uint32_t tim2_clock_hz;

	/* Enable TIM2 clock */
	__HAL_RCC_TIM2_CLK_ENABLE();

	/* Configure TIM2 as free-running 1 MHz counter */
	tim2_clock_hz = FS_SensorTime_GetTIM2ClockHz();
	TIM2->PSC = (tim2_clock_hz / SENSOR_TIME_TICK_HZ) - 1U;
	TIM2->ARR = 0xFFFFFFFF;  /* Full 32-bit range */
	TIM2->CNT = 0;
	TIM2->CR1 = 0;           /* Upcounting, no auto-reload preload */

	/* Generate update event to load prescaler, then clear the flag */
	TIM2->EGR = TIM_EGR_UG;
	TIM2->SR = ~TIM_SR_UIF;

	/* Enable update interrupt */
	TIM2->DIER = TIM_DIER_UIE;

	/* Configure NVIC for TIM2 */
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void FS_SensorTime_Start(void)
{
	/* Reset counter and overflow */
	overflow_count = 0;
	TIM2->CNT = 0;
	TIM2->SR = ~TIM_SR_UIF;

	/* Start counting */
	TIM2->CR1 |= TIM_CR1_CEN;
}

void FS_SensorTime_Stop(void)
{
	/* Stop counting */
	TIM2->CR1 &= ~TIM_CR1_CEN;

	/* Disable TIM2 clock to save power */
	__HAL_RCC_TIM2_CLK_DISABLE();
}

uint64_t FS_SensorTime_GetTicks(void)
{
	uint32_t hi1, hi2, lo;

	do {
		hi1 = overflow_count;
		lo = TIM2->CNT;
		hi2 = overflow_count;
	} while (hi1 != hi2);

	return ((uint64_t)hi1 << 32) | lo;
}

void TIM2_IRQHandler(void)
{
	if (TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR = ~TIM_SR_UIF;
		++overflow_count;
	}
}
