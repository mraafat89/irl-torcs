/***************************************************************************

    file                 : pit.h
    created              : Thu Mai 15 2:41:00 CET 2003
    copyright            : (C) 2003-2004 by Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: pit.h,v 1.5 2004/09/22 23:20:01 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _PIT_H_
#define _PIT_H_

#include "driver.h"
#include "spline.h"

class Driver;

class Pit {
    public:
        Pit(tSituation *s, Driver *driver, float PitOffset);
        ~Pit();

        void setPitstop(bool pitstop);
        bool getPitstop() { return pitstop; }

        void setInPit(bool inpitlane) { this->inpitlane = inpitlane; }
        bool getInPit() { return inpitlane; }
		bool getInPitExit() { return inpitexit; }
		
		float getPitEntryOffset() {
			return pitspline->evaluate(pitentry);
		}

        float getPitOffset(float offset, float fromstart);
        float getDriveThroughOffset(float offset, float fromstart);

        bool isBetween(float fromstart);
        bool isBetweenExit(float fromstart);
        float maxSpeed(float fromstart);
        bool isInPit(float fromstart);
        bool isTimeout(float distance);
		bool isInTrans(float fromstart);

#if 0
        float getNPitStart() { return pp[1].x; }
        float getNPitLoc() { return pp[3].x; }
        float getNPitEnd() { return pp[5].x; }
        float getNPitEntry() { return pp[0].x; }
#endif
        float getNPitStart() { return pp[1].x; }
        float getNPitLoc() { return pp[5].x; }
        float getNPitEnd() { return pp[7].x; }
        float getNPitEntry() { return pp[0].x; }

        float toSplineCoord(float x);

        float getSpeedlimitSqr() { return speedlimitsqr; }
        float getSpeedlimit() { return speedlimit; }
        float getSpeedLimitBrake(float speedsqr);

        void update();

    private:
        tTrack *track;
        tCarElt *car;
        tTrackOwnPit *mypit;            // Pointer to my pit.
        tTrackPitInfo *pitinfo;            // General pit info.

#if 0
        enum { NPOINTS = 7 };
        SplinePoint p[NPOINTS];            // Spline points.
#endif
        enum { PITPOINTS = 9, DTPOINTS = 4 };
        SplinePoint pp[PITPOINTS];            // Spline points.
        SplinePoint dtp[DTPOINTS];            // Spline points.

        Spline *pitspline;                    // Spline.
        Spline *dtspline;                    // Spline.

        bool pitstop;                    // Pitstop planned.
        bool usepitmaxspeed;
		bool inpitexit;
        bool inpitlane;                    // We are still in the pit lane.
        float pitentry;                    // Distance to start line of the pit entry.
        float pitend;                    // Distance to start line of the pit entry.
        float pitexit;                    // Distance to the start line of the pit exit.
        float pitstopentry;
        float pitstopexit;
        float pitmaxspeedoffset;
        float pitmaxspeed;

		float pittransoffset;
		bool hastransoffset;

        float speedlimitsqr;            // Pit speed limit squared.
        float speedlimit;                // Pit speed limit.
        float pitspeedlimitsqr;            // The original speedlimit squared.
        float pitstartextralength;

        float pittimer;                    // Timer for pit timeouts.

        static const float SPEED_LIMIT_MARGIN;
};

#endif // _PIT_H_


