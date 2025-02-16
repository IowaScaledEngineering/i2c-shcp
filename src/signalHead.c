/*************************************************************************
Title:    MSS-CASCADE-BASIC
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     signalHead.c
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

#include "signalHead.h"
#include "signalHeadPWM.h"

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

/*

typedef struct
{
	SignalAspect_t startAspect;
	SignalAspect_t endAspect;
	uint8_t phase;
	uint8_t redPWM;
	uint8_t yellowPWM;
	uint8_t greenPWM;
} SignalState_t;
*/

void signalHeadInitialize(SignalState_t* sig)
{
	sig->startAspect = sig->endAspect = sig->nextAspect = ASPECT_OFF;
	sig->phase = 0;
	sig->redPWM = 0;
	sig->yellowPWM = 0;
	sig->greenPWM = 0;
}

void signalHeadAspectSet(SignalState_t* sig, SignalAspect_t aspect)
{
	sig->nextAspect = aspect;
}

SignalAspect_t signalHeadAspectGet(SignalState_t* sig)
{
	return sig->nextAspect;
}

void signalHeadISR_OutputPWM(SignalState_t* const sig, const uint8_t options, const uint8_t pwmPhase, 
	volatile uint8_t* const redPort, const uint8_t redMask, volatile uint8_t* const yellowPort, 
	const uint8_t yellowMask, volatile uint8_t* const greenPort, const uint8_t greenMask)
{
	bool invert = (options & SIGNAL_OPTION_COMMON_ANODE)?false:true;
	if ((sig->redPWM > pwmPhase) != invert)
		*redPort &= ~redMask;
	else
		*redPort |= redMask;

	if ((sig->yellowPWM > pwmPhase) != invert)
		*yellowPort &= ~yellowMask;
	else
		*yellowPort |= yellowMask;

	if ((sig->greenPWM > pwmPhase) != invert)
		*greenPort &= ~greenMask;
	else
		*greenPort |= greenMask;
}



bool isGreenToYellow(SignalAspect_t startAspect, SignalAspect_t endAspect)
{
	if ((startAspect == ASPECT_GREEN || startAspect == ASPECT_FL_GREEN) 
		&& (endAspect == ASPECT_YELLOW || endAspect == ASPECT_FL_YELLOW))
		return true;
	return false;
}

bool isYellowToGreen(SignalAspect_t startAspect, SignalAspect_t endAspect)
{
	if ((startAspect == ASPECT_YELLOW || startAspect == ASPECT_FL_YELLOW)
			&& (endAspect == ASPECT_GREEN || endAspect == ASPECT_FL_GREEN))
		return true;
	return false;
}

void signalHeadISR_AspectToNextPWM(SignalState_t* sig, uint8_t flasher, uint8_t options)
{
	bool searchlightMode = (SIGNAL_OPTION_SEARCHLIGHT & options)?true:false;
	
	SignalAspect_t signalAspect = sig->nextAspect;
	
	// If it's a flashing aspect, mux the flasher in with the color
	if (signalAspect == ASPECT_FL_GREEN || signalAspect == ASPECT_FL_YELLOW || signalAspect == ASPECT_FL_RED)
		signalAspect = (flasher)?signalAspect:ASPECT_OFF;

	// If we're not currently running a transition and the aspect changed, start the transitioning
	if (sig->startAspect == sig->endAspect)
	{
		sig->phase = 0;
		sig->endAspect = signalAspect;
	}

	if (sig->startAspect != sig->endAspect)
	{
		// We're in transition towards the end aspect
		// How we do this depends upon the type of signal and the transition being made
		// For searchlights (US&S H, H2, H5 and GRS SA), green to yellow or vice versa passes through red and there's a little bounce giving
		//  a second red, but there's no long fade because the bulb never turns off.  
		// For searchlights passing between any color and red (or vice versa), it's just a quick bounce as the roundels move - no fade
		// For all other signals and searchlights going on or off, there's a fade in/out

		if (searchlightMode && sig->startAspect != ASPECT_OFF && sig->endAspect != ASPECT_OFF)
		{
			if (isGreenToYellow(sig->startAspect, sig->endAspect) || isYellowToGreen(sig->startAspect, sig->endAspect))
			{
				// *****************
				// Searchlight changing yellow-green or green-yellow through red
				// *****************

				// These are the special ones that bounce through red (at 60 frames/sec measured off my H2)
				// Yellow to green
				// ~6 frames to red
				// ~6 frames to green
				// ~12 frames back to red
				// ~8 frames to green
				
				// This uint16 is comprised of:
				//  0:4 - red channel
				//  5:9 - up channel
				//  10:14 - down channel
				
				uint16_t pwmWord = pgm_read_word(&searchlightPWMsThroughRed[sig->phase]);
				uint8_t upPhase = UP_PHASE(pwmWord);
				uint8_t downPhase = DOWN_PHASE(pwmWord);

				sig->redPWM = RED_PHASE(pwmWord);
				if (isGreenToYellow(sig->startAspect, sig->endAspect))
				{
					sig->greenPWM = downPhase;
					sig->yellowPWM = upPhase;
				} else {
					sig->greenPWM = upPhase;
					sig->yellowPWM = downPhase;
				}
				
				sig->phase++;
				if (sig->phase >= sizeof(searchlightPWMsThroughRed)/sizeof(searchlightPWMsThroughRed[0]))
				{
					// We're done
					sig->phase = 0;
					sig->startAspect = sig->endAspect;
				}
			}
			else
			{
				// *****************
				// Searchlight changing from yellow or green to red, or red to yellow or green
				// *****************
				uint16_t pwmWord = pgm_read_word(&searchlightPWMsInvolvingRed[sig->phase]);
				
				uint8_t upPhase = UP_PHASE(pwmWord);
				uint8_t downPhase = DOWN_PHASE(pwmWord);

				sig->redPWM = sig->yellowPWM = sig->greenPWM = 0;

				switch(sig->startAspect)
				{
					case ASPECT_RED:
					case ASPECT_FL_RED:
						sig->redPWM = downPhase;
						break;

					case ASPECT_YELLOW:
					case ASPECT_FL_YELLOW:
						sig->yellowPWM = downPhase;
						break;

					case ASPECT_GREEN:
					case ASPECT_FL_GREEN:
						sig->greenPWM = downPhase;
						break;

					default:
						break;
				}

				switch(sig->endAspect)
				{
					case ASPECT_RED:
					case ASPECT_FL_RED:
						sig->redPWM = upPhase;
						break;

					case ASPECT_YELLOW:
					case ASPECT_FL_YELLOW:
						sig->yellowPWM = upPhase;
						break;

					case ASPECT_GREEN:
					case ASPECT_FL_GREEN:
						sig->greenPWM = upPhase;
						break;

					default:
						break;
				}

				sig->phase++;
				if (sig->phase >= sizeof(searchlightPWMsInvolvingRed)/sizeof(searchlightPWMsInvolvingRed[0]))
				{
					// We're done
					sig->phase = 0;
					sig->startAspect = sig->endAspect;
				}
			}
			
		} else {
			uint16_t pwmWord = pgm_read_word(&fadePWMs[sig->phase]);
			uint8_t upPhase = UP_PHASE(pwmWord);
			uint8_t downPhase = DOWN_PHASE(pwmWord);

			sig->redPWM = sig->yellowPWM = sig->greenPWM = 0;
			
			switch(sig->startAspect)
			{
				case ASPECT_RED:
				case ASPECT_FL_RED:
					sig->redPWM = downPhase;
					break;

				case ASPECT_YELLOW:
				case ASPECT_FL_YELLOW:
					sig->yellowPWM = downPhase;
					break;

				case ASPECT_GREEN:
				case ASPECT_FL_GREEN:
					sig->greenPWM = downPhase;
					break;

				case ASPECT_OFF:
					// If we're going from off to on, go find the first entry
					// where the upPhase is not zero and start from there
					if (0 == upPhase)
					{
						do
						{
							pwmWord = pgm_read_word(&fadePWMs[++sig->phase]);
							upPhase = UP_PHASE(pwmWord);
						} while (upPhase == 0 && sig->phase < sizeof(fadePWMs)/sizeof(fadePWMs[0]));
						sig->phase--;
					}

				default:
					break;
			}

			switch(sig->endAspect)
			{
				case ASPECT_RED:
				case ASPECT_FL_RED:
					sig->redPWM = upPhase;
					break;

				case ASPECT_YELLOW:
				case ASPECT_FL_YELLOW:
					sig->yellowPWM = upPhase;
					break;

				case ASPECT_GREEN:
				case ASPECT_FL_GREEN:
					sig->greenPWM = upPhase;
					break;

				case ASPECT_OFF:
					// If we're going from something to off, we're done when we get the 
					//  lamp completely off
					if (downPhase == 0)
						sig->phase = sizeof(fadePWMs)/sizeof(fadePWMs[0]);

					break;

				default:
					break;
			}

			sig->phase++;
			if (sig->phase >= sizeof(fadePWMs)/sizeof(fadePWMs[0]))
			{
				// We're done
				sig->phase = 0;
				sig->startAspect = sig->endAspect;
			}
		}

/*				
		} else {
			// *****************
			// All other signals, where the bulbs fade in/out (and searchlights to or from off)
			// *****************
			uint8_t targetPWMRed = 0, targetPWMYellow = 0, targetPWMGreen = 0;
			uint8_t finalState = 0;
			
			switch(sig->endAspect)
			{
				case ASPECT_RED:
				case ASPECT_FL_RED:
					targetPWMRed = 0x1F;
					break;

				case ASPECT_YELLOW:
				case ASPECT_FL_YELLOW:
					targetPWMYellow = 0x1F;
					break;
					
				case ASPECT_GREEN:
				case ASPECT_FL_GREEN:
					targetPWMGreen = 0x1F;
					break;

				default:
					break;
			}
			
			if (targetPWMRed > sig->redPWM)
				sig->redPWM = MIN(sig->redPWM + 2, 0x1F);
			else if (targetPWMRed < sig->redPWM )
				sig->redPWM -= MIN(2, sig->redPWM);
			else
				finalState += 1;

			if (targetPWMYellow > sig->yellowPWM)
				sig->yellowPWM = MIN(sig->yellowPWM + 2, 0x1F);
			else if (targetPWMYellow < sig->yellowPWM)
				sig->yellowPWM -= MIN(2, sig->yellowPWM);
			else
				finalState += 1;

			if (targetPWMGreen > sig->greenPWM)
				sig->greenPWM = MIN(sig->greenPWM + 2, 0x1F);
			else if (targetPWMGreen < sig->greenPWM)
				sig->greenPWM -= MIN(2, sig->greenPWM);
			else
				finalState += 1;

			if (3 == finalState)
			{
				sig->phase = 0;
				sig->startAspect = sig->endAspect;
			}
		} */

	} else {
		// We're at steady state and the signal isn't changing, so 
		// just set the PWM based on the aspect for safety
		sig->redPWM = sig->yellowPWM = sig->greenPWM = 0;
		switch(sig->startAspect)
		{
			case ASPECT_RED:
			case ASPECT_FL_RED:
				sig->redPWM = 0x1F;
				break;

			case ASPECT_YELLOW:
			case ASPECT_FL_YELLOW:
				sig->yellowPWM = 0x1F;
				break;

			case ASPECT_GREEN:
			case ASPECT_FL_GREEN:
				sig->greenPWM = 0x1F;
				break;

			default:
				break;
		}
	}
}

