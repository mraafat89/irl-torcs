/***************************************************************************

    file                 : vardef.h
    created              : Thu Dec 20 01:20:19 CET 2002
    copyright            : (C) 2002 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: driver.h,v 1.0.1 2004/10/09 01:15:26 cern91 Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _VARDEF_H_
#define _VARDEF_H_

#include "car.h"
#include "raceman.h"

#define BOT_NAME (char*)"usr_2017"        // Change it -> Name of the module (short name please)

#define BT_SECT_PRIV "usr private"        // Change it -> Name of private section (xml file)

#define BT_ATT_MUFACTOR         "mufactor"
//#define BT_ATT_AMAGIC                "caero"
//#define BT_ATT_FMAGIC                "cfriction"

#define BT_ATT_K1999                "LineK1999"
#define BT_ATT_TEAMMATE                "teammate"

#define BT_ATT_PITENTRY                "pitentry"
#define BT_ATT_PITEXIT                "pitexit"
#define BT_ATT_PITTIME                 "pittime"
#define BT_ATT_BESTLAP                 "bestlap"
#define BT_ATT_WORSTLAP         "worstlap"

#define BT_ATT_FUELPERMETER         "fuelpermeter"
#define BT_ATT_FUELPERLAP         "fuelperlap"
#define BT_ATT_MAXDAMMAGE         "setmaxdammage"

#define BT_ATT_PITOFFSET         "pitoffset"
#define BT_ATT_K                 273.15f
#define BT_ATT_TC                currentTemperature
#define BT_ATT_TI                idealTemperature
#define BT_ATT_TG                currentGraining
#define BT_ATT_TW                currentWear
#define BT_ATT_CPW                car->priv.wheel
#define BT_ATT_CIW                car->info.wheel
#define BT_ATT_WT		MAX_GEARS*RE_STATE_RACE_END
#define BT_ATT_SET_ACCEL         "SetAccel"
#define BT_ATT_BRAKEDIST        "brakedelay"
#define BT_ATT_STEERPGAIN        "steerpgain"
#define BT_ATT_STEERPGAIN_MAX        "steerpgainmax"
#define BT_ATT_BUMPCAUTION        "bumpcaution"

enum { no_mode=0, normal=1, correcting=2, pitting=4, avoiding=8, avoidright=16, avoidside=32, avoidleft=64 };

/*=========================================================================*
                  global constants definition 
 *=========================================================================*/

// Status flags
                const int OPP_IGNORE			= 0x1;
                const int OPP_FRONT				= 0x2; // Opponent are in front of my car
                const int OPP_BACK				= 0x4; // Opponent are behind my car
                const int OPP_SIDE				= 0x8; // Looking to a side
                const int OPP_LEFT				= 0x10; // Opponent are at my left side
                const int OPP_RIGHT				= 0x20; // Opponent are at my right side
                const int OPP_COLL				= 0x40;
                const int OPP_COLL_WARNING		= 0x80;
                const int OPP_LETPASS			= 0x100;
                const int OPP_FRONT_FAST		= 0x200; // Drives faster than we
                const int OPP_FRONT_FOLLOW		= 0x400; // Drives slower than we
                const int OPP_OFF_TRACK			= 0x800; // don't worry about this one
                const int OPP_RACELINE_CONFLICT	= 0x1000; // don't worry about this one
                const int OPP_COLL_LINEAR		= 0x2000;
                const int OPP_BACK_CATCHING		= 0x4000;
                const int OPP_BACK_THREAT  		= 0x8000;

#endif
