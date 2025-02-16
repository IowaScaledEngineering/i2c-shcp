#ifndef _DEBOUNCER_H_
#define _DEBOUNCER_H_

#include <stdlib.h>
#include <stdint.h>

typedef struct
{
	uint8_t clock_A;
	uint8_t clock_B;
	uint8_t debounced_state;
} DebounceState8_t;

void initDebounceState8(DebounceState8_t* d, uint8_t initialState);
uint8_t debounce8(uint8_t raw_inputs, DebounceState8_t* d);
uint8_t getDebouncedState(DebounceState8_t* d);

#endif
