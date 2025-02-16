/*************************************************************************
Title:    I2C-SHCP (Signal Head Co-Processor)
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2025 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>

#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>

#include "avr-i2c-slave.h"
#include "debouncer.h"
#include "signalHead.h"

#define LOOP_UPDATE_TIME_MS       50
#define STARTUP_LOCKOUT_TIME_MS  500

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

volatile uint8_t signalHeadOptions = 0;
volatile uint32_t millis = 0;

#define MAX_SIGNAL_HEADS 8

SignalState_t signal[MAX_SIGNAL_HEADS];

#define I2C_REGISTER_MAP_SIZE  24
volatile uint8_t i2c_registerMap[I2C_REGISTER_MAP_SIZE];
volatile uint8_t i2c_registerAttributes[I2C_REGISTER_MAP_SIZE];
const uint8_t i2c_registerMapSize= I2C_REGISTER_MAP_SIZE;


// A few hardware definitions
// Signal Port Connections
// These are in the order of:
//  Red address, bitmask
//  Yellow address, bitmask
//  Green address, bitmask

#define SIGNAL_HEAD_0_DEF   &PORTB, _BV(PB0), &PORTB, _BV(PB1), &PORTB, _BV(PB2)
#define SIGNAL_HEAD_1_DEF   &PORTB, _BV(PB3), &PORTB, _BV(PB4), &PORTB, _BV(PB5)
#define SIGNAL_HEAD_2_DEF   &PORTD, _BV(PD2), &PORTD, _BV(PD3), &PORTD, _BV(PD4)
#define SIGNAL_HEAD_3_DEF   &PORTD, _BV(PD5), &PORTD, _BV(PD6), &PORTD, _BV(PD7)
#define SIGNAL_HEAD_4_DEF   &PORTB, _BV(PB6), &PORTB, _BV(PB7), &PORTC, _BV(PC0)
#define SIGNAL_HEAD_5_DEF   &PORTC, _BV(PC1), &PORTC, _BV(PC2), &PORTC, _BV(PC3)
#define SIGNAL_HEAD_6_DEF   &PORTC, _BV(PC7), &PORTD, _BV(PD0), &PORTD, _BV(PD1)
#define SIGNAL_HEAD_7_DEF   &PORTA, _BV(PA1), &PORTA, _BV(PA2), &PORTC, _BV(PA3)


uint32_t getMillis()
{
	uint32_t retmillis;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		retmillis = millis;
	}
	return retmillis;
}

ISR(TIMER0_COMPA_vect) 
{
	static uint8_t flasherCounter = 0;
	static uint8_t flasher = 0;
	static uint8_t pwmPhase = 0;
	static uint8_t subMillisCounter = 0;
	
	// The ISR does two main things - updates the LED outputs since
	//  PWM is done through software, and updates millis which is used
	//  to trigger various events
	// We need this to run at roughly 125 Hz * number of PWM levels (32).  That makes a nice round 4kHz
	
	// First thing, output the signals so that the PWM doesn't get too much jitter

	signalHeadISR_OutputPWM(&signal[0], signalHeadOptions, pwmPhase, SIGNAL_HEAD_0_DEF);
	signalHeadISR_OutputPWM(&signal[1], signalHeadOptions, pwmPhase, SIGNAL_HEAD_1_DEF);
	signalHeadISR_OutputPWM(&signal[2], signalHeadOptions, pwmPhase, SIGNAL_HEAD_2_DEF);
	signalHeadISR_OutputPWM(&signal[3], signalHeadOptions, pwmPhase, SIGNAL_HEAD_3_DEF);
	signalHeadISR_OutputPWM(&signal[4], signalHeadOptions, pwmPhase, SIGNAL_HEAD_4_DEF);
	signalHeadISR_OutputPWM(&signal[5], signalHeadOptions, pwmPhase, SIGNAL_HEAD_5_DEF);
	signalHeadISR_OutputPWM(&signal[6], signalHeadOptions, pwmPhase, SIGNAL_HEAD_6_DEF);
	signalHeadISR_OutputPWM(&signal[7], signalHeadOptions, pwmPhase, SIGNAL_HEAD_7_DEF);

	// Now do all the counter incrementing and such
	if (++subMillisCounter >= 4)
	{
		subMillisCounter = 0;
		millis++;
	}

	pwmPhase = (pwmPhase + 1) & 0x1F;

	if (0 == pwmPhase)
	{
		pwmPhase = 0;
		flasherCounter++;
		if (flasherCounter > 94)
		{
			flasher ^= 0x01;
			flasherCounter = 0;
		}

		// We rolled over the PWM counter, calculate the next PWM widths
		// This runs at 125 frames/second essentially

		for (uint8_t i=0; i<MAX_SIGNAL_HEADS; i++)
			signalHeadISR_AspectToNextPWM(&signal[i], flasher, signalHeadOptions);
	}
}

void initializeTimer()
{
	TIMSK0 = 0;           // Timer interrupts OFF

	// Set up Timer/Counter0 for 100Hz clock
	TCCR0A = 0b00001010;  // CTC Mode
	                      // CS01 - 1:8 prescaler
	OCR0A = 250;          // 8MHz / 8 / 250 = 4kHz
	TIMSK0 = _BV(OCIE0A);
}

void initializeRegisterMap()
{
	for(uint8_t i=0; i<I2C_REGISTER_MAP_SIZE; i++)
	{
		i2c_registerMap[i] = i2c_registerAttributes[i] = 0;
	}
}

void initializeI2C()
{
	initializeRegisterMap();
	i2cSlaveInitialize(0x40, false);
}

int main(void)
{
	DebounceState8_t optionsDebouncer;
	uint32_t lastReadTime = 0;
	uint32_t currentTime = 0;
	uint8_t i=0;
	// Deal with watchdog first thing
	MCUSR = 0;              // Clear reset status
	wdt_reset();            // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);    // Enable it at a 1S timeout.

	// PORT A
	//  PA7 - Input  - B MSS - Adjacent
	//  PA6 - Input  - B MSS - Approach
	//  PA5 - Input  - B MSS - Advance Approach
	//  PA4 - Input  - A MSS - Adjacent
	//  PA3 - Input  - A MSS - Approach
	//  PA2 - Input  - A MSS - Advance Approach
	//  PA1 - Analog - Options 1
	//  PA0 - Analog - Options 0

	// PORT B
	//  PB7 - n/a    - /RESET (not I/O pin)
	//  PB6 - Output - B Signal - GREEN  (low=active)
	//  PB5 - Output - B Signal - YELLOW (low=active)
	//  PB5 - Output - B Signal - RED (low=active)
	//  PB3 - Input  - Common anode / common cathode sense (1 = common anode)
	//  PB2 - Output - A Signal - GREEN (low=active)
	//  PB1 - Output - A Signal - YELLOW (low=active)
	//  PB0 - Output - A Signal - RED (low=active)

	// PORT B
	//  PB7 - n/a    - /RESET (not I/O pin)
	//  PB6 - Output - B Signal - GREEN  (low=active)
	//  PB5 - Output - B Signal - YELLOW (low=active)
	//  PB5 - Output - B Signal - RED (low=active)
	//  PB3 - Input  - Common anode / common cathode sense (1 = common anode)
	//  PB2 - Output - A Signal - GREEN (low=active)
	//  PB1 - Output - A Signal - YELLOW (low=active)
	//  PB0 - Output - A Signal - RED (low=active)

	// PORT D
	//  PB7 - n/a    - /RESET (not I/O pin)
	//  PB6 - Output - B Signal - GREEN  (low=active)
	//  PB5 - Output - B Signal - YELLOW (low=active)
	//  PB5 - Output - B Signal - RED (low=active)
	//  PB3 - Input  - Common anode / common cathode sense (1 = common anode)
	//  PB2 - Output - A Signal - GREEN (low=active)
	//  PB1 - Output - A Signal - YELLOW (low=active)
	//  PB0 - Output - A Signal - RED (low=active)


	PORTA = 0b00000000;
	DDRA  = 0b00001110;

	PORTB = 0b00000000;
	DDRB  = 0b11111111;

	PORTC = 0b00000000;
	DDRC  = 0b11001111;

	PORTD = 0b00000000;
	DDRD  = 0b11111101;

	initializeTimer();
//	initializeOptions(&optionsDebouncer);
	initializeI2C();

	for(i=0; i<MAX_SIGNAL_HEADS; i++)
	{
		signalHeadInitialize(&signal[i]);
		signalHeadAspectSet(&signal[i], ASPECT_OFF);
	}

	// Need to set common anode or common cathode
	// as quickly as possible to prevent an initialization flash
	// initializeOptions() has already gotten the initial state of the CA/CC line
/*	optionJumpers = getDebouncedState(&optionsDebouncer);
	if (optionJumpers & OPTION_COMMON_ANODE)
		signalHeadOptions |= SIGNAL_OPTION_COMMON_ANODE;
	else 
		signalHeadOptions &= ~SIGNAL_OPTION_COMMON_ANODE;*/

	signalHeadOptions |= SIGNAL_OPTION_COMMON_ANODE;

	sei();
	wdt_reset();

	while(1)
	{
		wdt_reset();
		currentTime = getMillis();

		// Because debouncing and such is built into option reading and the MSS library, only 
		//  run the updates every 10mS or so.

		if (((uint32_t)currentTime - lastReadTime) > LOOP_UPDATE_TIME_MS && !(i2cBusy()))
		{
			lastReadTime = currentTime;

			for(i=0; i<MAX_SIGNAL_HEADS; i++)
				signalHeadAspectSet(&signal[i], i2c_registerMap[i]);
		}
	}
}




