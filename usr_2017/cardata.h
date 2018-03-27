/***************************************************************************

    file                 : cardata.h
    created              : Thu Sep 23 12:31:33 CET 2004
    copyright            : (C) 2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: cardata.h,v 1.2 2006/03/06 22:43:50 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

/*
	This class holds global facts about cars, therefore no data relative to
	each other (for that is the class Opponents/Opponent responsible).
*/

#ifndef _BT_CARDATA_H_
#define _BT_CARDATA_H_

#include <stdio.h>
#include <math.h>
#include <car.h>
#include <robottools.h>
#include <raceman.h>

#include "linalg.h"
#include "vardef.h"


class SingleCardata {
	public:
		void init(CarElt *car);

		inline float getSpeedInTrackDirection() { return speed; }
		inline float getWidthOnTrack() { return width; }
		inline float getLengthOnTrack() { return length; }
		inline float getTrackangle() { return trackangle; }
		inline float getCarAngle() { return angle; }

		inline bool thisCar(tCarElt *car) { return (car == this->car); }
                inline tPosd *getCorner1() { return corner1; }
                inline tPosd *getCorner2() { return corner2; }
                inline tPosd *getLastSpeed() { return lastspeed; }

		void update();
		void updateModel();

	protected:
		static float getSpeed(tCarElt *car, float trackangle);

		float speed;		// speed in direction of the track.
		float width;		// the cars needed width on the track.
		float length;		// the cars needed length on the track.
		float trackangle;	// Track angle at the opponents position.
		float angle;		// The angle of the car relative to the track tangent.
		float fuelMassFactor;

                tPosd corner1[4];
                tPosd corner2[4];
                tPosd lastspeed[3];

		tdble t_m;
		tdble t_m_f;
		tdble t_m_r;
		tdble currentMass;

		double CTFactor;

		tdble RH;
		tdble CA;
		tdble CA_RW;
		tdble CA_FW;
		tdble CA_GE;

		tdble baseMass;

		tCarElt *car;		// For identification.

	public:
		double lTT;
		tdble mFTT;
		tdble aFTT;
		tdble aTT;
		double hTT;
		tdble lftOH;
		tdble rgtOH;
		double lmTT;
		double rmTT;
		tdble carMu;
		tdble fullCarMu;
		tdble offlineFuelCarMu;
		tdble baseCarMu;
		tdble fuel;
		tdble damage;
};


// TODO: use singleton pattern.
class Cardata {
	public:
		Cardata(tSituation *s);
		~Cardata();

		void update();
		SingleCardata *findCar(tCarElt *car);

	protected:
		SingleCardata *data;	// Array with car data.
		int ncars;				// # of elements in data.
};

#define CG(i) (BT_ATT_CPW[i].BT_ATT_TG)
#define CT(i) (BT_ATT_CPW[i].BT_ATT_TC-BT_ATT_K)
#define CW(i) (BT_ATT_CPW[i].BT_ATT_TW)
#define iT() (BT_ATT_CIW[0].BT_ATT_TI-BT_ATT_K)

#endif
