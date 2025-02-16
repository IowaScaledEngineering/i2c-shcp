/*************************************************************************
Title:    MSS-CASCADE-BASIC Searhlight PWM Values
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
          Based on the work of David Johnson-Davies - www.technoblogy.com - 23rd October 2017
           and used under his Creative Commons Attribution 4.0 International license
File:     $Id: $
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

#ifndef _SEARCHLIGHT_PWM_H_
#define _SEARCHLIGHT_PWM_H_

#include <stdint.h>
#include <avr/pgmspace.h>

/* The PWM values for the searchlight transitions are stored as a uint16 in program space
  11111100 00000000
  54321098 76543210
  xUUUUUDD DDDRRRRR
  
  The color going up (meaning we're transitioning to that aspect) is stored in U
  The color going down (meaning we're transitioning from that aspect) is stored in D
  The red flash between green and yellow is stored in R

	Searchlight logic
	Dark to on takes ~1/3 second, or 42 frames (120 fps)
	Changeovers take 33 frames
	End to end (yellow to green, green to yellow)
	   12 frames to red
	   12 frames to target color
	   25 frames back to red
	   15 frames back to target color


*/

#define DRU_TO_UINT16(d, r, u)   ((((d) & 0x1F)<<10) | (((u) & 0x1F)<<5) | ((r) & 0x1F))

#define UP_PHASE(w)   ((w>>5) & 0x1F)
#define DOWN_PHASE(w) ((w>>10) & 0x1F)
#define RED_PHASE(w)  (w & 0x1F)




const uint16_t const searchlightPWMsThroughRed[] PROGMEM = 
{ 
	DRU_TO_UINT16( 27,  0,  0),
	DRU_TO_UINT16( 17,  0,  0),
	DRU_TO_UINT16( 12,  0,  0),
	DRU_TO_UINT16(  0,  0,  0),
	DRU_TO_UINT16(  0, 17,  0),
	DRU_TO_UINT16(  0, 25,  0),
	DRU_TO_UINT16(  0, 25,  0),
	DRU_TO_UINT16(  0, 17,  0),
	DRU_TO_UINT16(  0,  0,  0),
	DRU_TO_UINT16(  0,  0, 12),
	DRU_TO_UINT16(  0,  0, 17),
	DRU_TO_UINT16(  0,  7, 17),
	DRU_TO_UINT16(  0,  0, 17),
	DRU_TO_UINT16(  0,  0, 17),
	DRU_TO_UINT16(  0,  0, 22),
	DRU_TO_UINT16(  0,  0, 22),
	DRU_TO_UINT16(  0,  0, 17),
	DRU_TO_UINT16(  0,  0, 17),
	DRU_TO_UINT16(  0,  0, 17),
	DRU_TO_UINT16(  0,  7, 17),
	DRU_TO_UINT16(  0,  0, 17),
	DRU_TO_UINT16(  0,  0, 12),
	DRU_TO_UINT16(  0, 12,  7),
	DRU_TO_UINT16(  0, 17,  0),
	DRU_TO_UINT16(  0, 22,  0),
	DRU_TO_UINT16(  0, 17,  0),
	DRU_TO_UINT16(  0, 12,  7),
	DRU_TO_UINT16(  0,  0, 12),
	DRU_TO_UINT16(  0,  0, 17),
	DRU_TO_UINT16(  0,  0, 22),
	DRU_TO_UINT16(  0,  0, 27),
	DRU_TO_UINT16(  0,  0, 31)
};

const uint16_t const searchlightPWMsInvolvingRed[] PROGMEM = { 
	DRU_TO_UINT16( 31,  0,  0),
	DRU_TO_UINT16( 27,  0,  0),
	DRU_TO_UINT16( 22,  0,  0),
	DRU_TO_UINT16( 17,  0,  0),
	DRU_TO_UINT16( 12,  0,  0),
	DRU_TO_UINT16(  5,  0,  0),
	DRU_TO_UINT16(  0,  0,  0),
	DRU_TO_UINT16(  0,  0,  5),
	DRU_TO_UINT16(  0,  0, 12),
	DRU_TO_UINT16(  0,  0, 12),
	DRU_TO_UINT16(  0,  0,  5),
	DRU_TO_UINT16(  5,  0,  0),
	DRU_TO_UINT16( 12,  0,  0),
	DRU_TO_UINT16(  5,  0,  0),
	DRU_TO_UINT16(  0,  0,  5),
	DRU_TO_UINT16(  0,  0, 12),
	DRU_TO_UINT16(  0,  0, 17),
	DRU_TO_UINT16(  0,  0, 22),
	DRU_TO_UINT16(  0,  0, 27),
	DRU_TO_UINT16(  0,  0, 31)
};

const uint16_t const fadePWMs[] PROGMEM = 
{ 
	DRU_TO_UINT16( 30,  0,  0),
	DRU_TO_UINT16( 28,  0,  0),
	DRU_TO_UINT16( 26,  0,  0),
	DRU_TO_UINT16( 24,  0,  0),
	DRU_TO_UINT16( 22,  0,  0),
	DRU_TO_UINT16( 20,  0,  0),
	DRU_TO_UINT16( 18,  0,  0),
	DRU_TO_UINT16( 16,  0,  0),
	DRU_TO_UINT16( 14,  0,  0),
	DRU_TO_UINT16( 12,  0,  0),
	DRU_TO_UINT16( 10,  0,  0),
	DRU_TO_UINT16(  8,  0,  0),
	DRU_TO_UINT16(  6,  0,  0),
	DRU_TO_UINT16(  4,  0,  0),
	DRU_TO_UINT16(  2,  0,  0),
	DRU_TO_UINT16(  0,  0,  0),
	DRU_TO_UINT16(  0,  0,  0),
	DRU_TO_UINT16(  0,  0,  2),
	DRU_TO_UINT16(  0,  0,  4),
	DRU_TO_UINT16(  0,  0,  6),
	DRU_TO_UINT16(  0,  0,  8),
	DRU_TO_UINT16(  0,  0, 10),
	DRU_TO_UINT16(  0,  0, 12),
	DRU_TO_UINT16(  0,  0, 14),
	DRU_TO_UINT16(  0,  0, 16),
	DRU_TO_UINT16(  0,  0, 18),
	DRU_TO_UINT16(  0,  0, 20),
	DRU_TO_UINT16(  0,  0, 22),
	DRU_TO_UINT16(  0,  0, 24),
	DRU_TO_UINT16(  0,  0, 26),
	DRU_TO_UINT16(  0,  0, 28),
	DRU_TO_UINT16(  0,  0, 31)
};

#endif

