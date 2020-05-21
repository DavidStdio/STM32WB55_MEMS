/*
 * diagnostics_utils.h
 *
 *  Created on: May 20, 2020
 *      Author: E014283
 */

#ifndef DIAGNOSTICS_UTILS_H_
#define DIAGNOSTICS_UTILS_H_

#include <stdint.h>

void dwt_cycle_counter_init(void);
uint32_t cycles_to_usec(uint32_t timestamp);

#define GET_TIMESTAMP()      (*(uint32_t *)(0xE0001004))            // Retrieve a system timestamp. Cortex-M cycle counter.

#endif /* DIAGNOSTICS_UTILS_H_ */

