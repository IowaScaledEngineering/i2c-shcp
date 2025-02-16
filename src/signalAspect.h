/*************************************************************************
Title:    Signal Aspects (shared definition)
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     signalAspect.h
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

#ifndef _SIGNALASPECT_H_
#define _SIGNALASPECT_H_

typedef enum
{
	ASPECT_OFF          = 0,
	ASPECT_GREEN        = 1,
	ASPECT_FL_GREEN     = 2,
	ASPECT_YELLOW       = 3,
	ASPECT_FL_YELLOW    = 4,
	ASPECT_RED          = 5,
	ASPECT_FL_RED       = 6,
	ASPECT_LUNAR        = 7,
	ASPECT_END
} SignalAspect_t;
#endif

