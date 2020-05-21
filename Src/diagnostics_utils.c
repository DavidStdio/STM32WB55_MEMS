/*
 * diagnostics_utils.c
 *
 *  Created on: May 20, 2020
 *      Author: E014283
 */


#include "diagnostics_utils.h"
#include "stm32wbxx.h"


#define DWT_CTRL                  (*(volatile unsigned long*) (0xE0001000uL))   // DWT Control Register
#define NOCYCCNT_BIT              (1uL << 25)                                   // Cycle counter support bit
#define CYCCNTENA_BIT             (1uL << 0)                                    // Cycle counter enable bit

static uint32_t cycles_per_uSec = 0;

void dwt_cycle_counter_init(void)
{
	if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0)
	{
		if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
		{
			DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
		}
	}
    cycles_per_uSec = (SystemCoreClock/1000000L);
}

uint32_t cycles_to_usec(uint32_t timestamp)
{
	return timestamp/cycles_per_uSec;
}
