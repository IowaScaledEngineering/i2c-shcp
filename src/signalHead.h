/*************************************************************************
Title:    MSS-CASCADE-BASIC
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     signalHead.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2024 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#ifndef _SIGNALHEAD_H_
#define _SIGNALHEAD_H_

#include <stdint.h>
#include <stdbool.h>
#include "signalAspect.h"

typedef struct
{
	SignalAspect_t startAspect;
	SignalAspect_t endAspect;
	SignalAspect_t nextAspect;
	uint8_t phase;
	uint8_t redPWM;
	uint8_t yellowPWM;
	uint8_t greenPWM;
} SignalState_t;

#define SIGNAL_OPTION_COMMON_ANODE         0x01
#define SIGNAL_OPTION_SEARCHLIGHT          0x02

#define SIGNAL_HEAD_INIT_STATE {ASPECT_OFF, ASPECT_OFF, 0, 0, 0, 0}

void signalHeadInitialize(SignalState_t* sig);
void signalHeadAspectSet(SignalState_t* sig, SignalAspect_t aspect);
SignalAspect_t signalHeadAspectGet(SignalState_t* sig);

void signalHeadISR_OutputPWM(SignalState_t* const sig, const uint8_t options, const uint8_t pwmPhase, 
	volatile uint8_t* const redPort, const uint8_t redMask, volatile uint8_t* const yellowPort, 
	const uint8_t yellowMask, volatile uint8_t* const greenPort, const uint8_t greenMask);

void signalHeadISR_AspectToNextPWM(SignalState_t* sig, uint8_t flasher, uint8_t options);

#endif

