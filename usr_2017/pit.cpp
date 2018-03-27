/***************************************************************************

    file                 : pit.cpp
    created              : Thu Mai 15 2:43:00 CET 2003
    copyright            : (C) 2003-2004 by Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: pit.cpp,v 1.11 2006/10/12 21:26:42 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>

#include "pit.h"
#include "xmldefs.h"

const float Pit::SPEED_LIMIT_MARGIN = 0.5;        // [m/s] safety margin to avoid pit speeding.

#define PIT_HACK_WIDTH 0.0
#define PIT_LANE_HACK_WIDTH -2.0

Pit::Pit(tSituation *s, Driver *driver, float pitoffset)
{
    track = driver->getTrackPtr();
    car = driver->getCarPtr();
    mypit = driver->getCarPtr()->_pit;
    pitinfo = &track->pits;
    pitstop = inpitlane = inpitexit = hastransoffset = false;
    pittimer = 0.0;
    float pitspeedmargin = 0.0f;
    float pitstartoverride0 = -1.0f, pitstartoverride1 = -1.0f, pitstartoverride2 = -1.0f, pitexitoverride = -1.0f;
    float pitentryoffset = 0.0f, pitexitoffset = 0.0f;
    usepitmaxspeed = false;
    pitmaxspeed = pitmaxspeedoffset = 0.0f;

    pitspeedmargin = GfParmGetNum( car->_carHandle, "hymie_2015", "pitspeedmargin", (char*)NULL, SPEED_LIMIT_MARGIN);
    pitstartextralength = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_EXTRA_LENGTH, (char*)NULL, 0.0f);
    pitstartoverride0 = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_START_OVERRIDE0, (char*)NULL, -1.0f);
    pitstartoverride1 = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_START_OVERRIDE1, (char*)NULL, -1.0f);
    pitstartoverride2 = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_START_OVERRIDE2, (char*)NULL, -1.0f);
    pitexitoverride = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_EXIT_OVERRIDE, (char*)NULL, -1.0f);
    float pityoffset = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_OFFSET, (char*)NULL, 1.0f);
    pitentryoffset = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_ENTRY_OFFSET, (char*)NULL, 0.0f);
    pitexitoffset = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_EXIT_OFFSET, (char*)NULL, 0.0f);
    pitmaxspeedoffset = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_MAX_SPEED_OFFSET, (char*)NULL, 0.0f);
    pitmaxspeed = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_MAX_SPEED, (char*)NULL, 0.0f);
    pittransoffset = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_TRANS_OFFSET, (char*)NULL, -1.0f);
	if (pittransoffset > 0.0)
		hastransoffset = true;

    usepitmaxspeed = (pitmaxspeed > 0.01f && pitmaxspeedoffset > 1.0f);

    if (mypit != NULL) 
    {
        speedlimit = pitinfo->speedLimit - pitspeedmargin;
        speedlimitsqr = speedlimit*speedlimit;
        pitspeedlimitsqr = pitinfo->speedLimit*pitinfo->speedLimit;

        // Compute pit spline points along the track.
        pp[5].x = mypit->pos.seg->lgfromstart + mypit->pos.toStart;
        pp[4].x = pp[5].x - pitinfo->len;
        pp[6].x = pp[5].x + pitinfo->len;
        dtp[0].x = pp[0].x = pitinfo->pitEntry->lgfromstart + pitoffset;
        // HACK! pit entry on corkscrew
        pp[1].x = pp[0].x;
        pp[2].x = pp[1].x;
        if (pitstartoverride0 > 0.0)
            dtp[0].x = pp[0].x = pitstartoverride0;
        if (pitstartoverride1 > 0.0)
            pp[1].x = pitstartoverride1;
        if (pitstartoverride2 > 0.0)
            pp[2].x = pitstartoverride2;
        dtp[1].x = pp[3].x = pitinfo->pitStart->lgfromstart; 
        if (pitstartoverride0 < 0.0)
            pp[1].x = pitinfo->pitEntry->lgfromstart + pitoffset + pitstartextralength;
        //pp[5].x = pitinfo->pitEnd->lgfromstart + pitinfo->len/2.0f;
        //pp[5].x = pp[3].x + (pitInfo->nMaxPits - car->index) * pitinfo->len;
        dtp[2].x = pp[7].x = pp[3].x + pitinfo->nMaxPits * pitinfo->len;
        dtp[3].x = pp[8].x = pitinfo->pitExit->lgfromstart;
        if (pitexitoverride > 0.0)
            dtp[4].x = pp[8].x = pitexitoverride;

        pitentry = pp[0].x;
        pitend = pp[7].x;
        pitexit = pp[8].x;
        pitstopentry = pp[4].x;
        pitstopexit = pp[6].x;

        // Normalizing spline segments to >= 0.0.
        int i;
        for (i = 0; i < PITPOINTS; i++) 
		{
            pp[i].s = 0.0;
            pp[i].x = toSplineCoord(pp[i].x);
        }
        for (i = 0; i < DTPOINTS; i++) 
		{
            dtp[i].s = 0.0;
            dtp[i].x = toSplineCoord(dtp[i].x);
        }

        // Fix broken pit exit.
        if (pp[8].x < pp[7].x) {
            
            dtp[3].x = pp[8].x = pp[7].x + 50.0f;
        }

        // Fix point for first pit if necessary.
        if (pp[3].x > pp[4].x) {
            pp[3].x = pp[4].x - 3.0f;
        }
        if (pp[2].x > pp[3].x) {
            pp[2].x = pp[3].x - 3.0f;
            pitentry = pp[2].x;
        }

        // Fix point for last pit if necessary.
        if (pp[6].x > pp[7].x) {
            pp[7].x = pp[6].x;
        }


        float sign = (pitinfo->side == TR_LFT) ? 1.0f : -1.0f;
        dtp[0].y = pp[0].y = pitentryoffset;
        dtp[3].y = pp[8].y = pitexitoffset;
        for (i = 1; i < PITPOINTS - 1; i++) {
            pp[i].y = fabs(pitinfo->driversPits->pos.toMiddle) - pitinfo->width;
            pp[i].y *= sign;
        }
        for (i = 1; i < DTPOINTS - 1; i++) {
            dtp[i].y = fabs(pitinfo->driversPits->pos.toMiddle) - pitinfo->width;
            dtp[i].y *= sign;
        }

        /* HACK */
        pp[2].y = pp[0].y;
        pp[1].y = pp[0].y;
        //pp[3].y = fabs(pitinfo->driversPits->pos.toMiddle)*sign;
        pp[5].y = (fabs(pitinfo->driversPits->pos.toMiddle)+3.5f*pityoffset)*sign;
        pitspline = new Spline(PITPOINTS, pp);
        dtspline = new Spline(DTPOINTS, dtp);
    }
}


Pit::~Pit()
{
    if (mypit != NULL) {
        delete pitspline;
        delete dtspline;
    }
}


// Transforms track coordinates to spline parameter coordinates.
float Pit::toSplineCoord(float x)
{
    x -= pitentry;
    while (x < 0.0f) {
        x += track->length;
    }
    return x;
}


// Computes offset to track middle for trajectory.
float Pit::getPitOffset(float offset, float fromstart)
{
  float pitoffset;

    if (mypit != NULL) {
        if (getInPit() || (getPitstop() && isBetween(fromstart))) {
            fromstart = toSplineCoord(fromstart);
            pitoffset = pitspline->evaluate(fromstart); 
            return pitoffset;
        }
    }
    return offset;
}


// Computes offset to track middle for trajectory.
float Pit::getDriveThroughOffset(float offset, float fromstart)
{
  float pitoffset;

    if (mypit != NULL) {
        if (getInPit() || (getPitstop() && isBetween(fromstart))) {
            fromstart = toSplineCoord(fromstart);
            pitoffset = dtspline->evaluate(fromstart); 
            return pitoffset;
        }
    }
    return offset;
}


// Sets the pitstop flag if we are not in the pit range.
void Pit::setPitstop(bool pitstop)
{
    if (mypit == NULL) {
        return;
    }

    float fromstart = car->_distFromStartLine;

    if (!isBetween(fromstart)) {
        this->pitstop = pitstop;
    } else if (!pitstop) {
        this->pitstop = pitstop;
        pittimer = 0.0f;
    }
}

bool Pit::isInTrans(float fromstart)
{
	if (!hastransoffset || isBetween(fromstart))
		return false;

	if (fromstart < pitentry && fromstart >= pittransoffset)
		return true;

	return false;
}

// Check if the argument fromstart is in the range of the pit.
bool Pit::isBetween(float fromstart)
{
    if (pitentry <= pitexit) {
        if (fromstart >= pitentry && fromstart <= pitexit) {
            return true;
        } else {
            return false;
        }
    } else {
        // Warning: TORCS reports sometimes negative values for "fromstart"!
        if (fromstart <= pitexit || fromstart >= pitentry) {
            return true;
        } else {
            return false;
        }
    }
}

bool Pit::isBetweenExit(float fromstart)
{
    if (pitend <= pitexit) {
        if (fromstart >= pitend && fromstart <= pitexit) {
            return true;
        } else {
            return false;
        }
    } else {
        // Warning: TORCS reports sometimes negative values for "fromstart"!
        if (fromstart <= pitexit || fromstart >= pitend) {
            return true;
        } else {
            return false;
        }
    }
}

// check if fromstart is in the actual pit area
bool Pit::isInPit(float fromstart)
{
    if (pitstopentry <= pitstopexit) {
        if (fromstart >= pitstopentry && fromstart <= pitstopexit) {
            return true;
        } else {
            return false;
        }
    } else {
        // Warning: TORCS reports sometimes negative values for "fromstart"!
        if (fromstart <= pitstopexit || fromstart >= pitstopentry) {
            return true;
        } else {
            return false;
        }
    }
}

float Pit::maxSpeed(float fromstart)
{
    if (!usepitmaxspeed)
        return 100000.0f;

    if (fromstart > pitmaxspeedoffset)
        return pitmaxspeed;
    return 100000.0f;
}


// Checks if we stay too long without getting captured by the pit.
// Distance is the distance to the pit along the track, when the pit is
// ahead it is > 0, if we overshoot the pit it is < 0.
bool Pit::isTimeout(float distance)
{
    if (car->_speed_x > 1.0f || distance > 3.0f || !getPitstop()) {
        pittimer = 0.0f;
        return false;
    } else {
        pittimer += (float) RCM_MAX_DT_ROBOTS;
        if (pittimer > 3.0f) {
            pittimer = 0.0f;
            return true;
        } else {
            return false;
        }
    }
}


// Update pit data and strategy.
void Pit::update()
{
    if (mypit != NULL) {
		inpitexit = false;
        if (isBetween(car->_distFromStartLine)) {
            if (getPitstop()) {
				if (isBetweenExit(car->_distFromStartLine))
					inpitexit = true;
                setInPit(true);
            }
        } else {
            setInPit(false);
        }

        if (getPitstop()) {
            car->_raceCmd = RM_CMD_PIT_ASKED;
        }
    }
}


float Pit::getSpeedLimitBrake(float speedsqr)
{
    return (speedsqr-speedlimitsqr)/(pitspeedlimitsqr-speedlimitsqr);
}

