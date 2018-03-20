/***************************************************************************

    file                 : cardata.cpp
    created              : Thu Sep 23 12:31:37 CET 2004
    copyright            : (C) 2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: cardata.cpp,v 1.2 2006/03/06 22:43:50 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <math.h>
#include "cardata.h"
#include "xmldefs.h"
#include "vardef.h"


void SingleCardata::update()
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    speed = getSpeed(car, trackangle);
    angle = trackangle - car->_yaw;
	double dang = (double)angle;
    NORM_PI_PI(dang);
	angle = (float)dang;
    width = MAX(car->_dimension_y, fabs(car->_dimension_x*sin(angle) + car->_dimension_y*cos(angle))) + 0.1f;
    length = MAX(car->_dimension_x, fabs(car->_dimension_y*sin(angle) + car->_dimension_x*cos(angle))) + 0.1f;

	for (int i=0; i<4; i++)
	{
		corner2[i].ax = corner1[i].ax;
		corner2[i].ay = corner1[i].ay;
		corner1[i].ax = car->_corner_x(i);
		corner1[i].ay = car->_corner_y(i);
        double ct = CT(i);
		if (i == FRNT_LFT || i == REAR_LFT)
			lmTT = MAX(lmTT, ct);
		else
			rmTT = MAX(rmTT, ct);
	}

	lastspeed[2].ax = lastspeed[1].ax;
	lastspeed[2].ay = lastspeed[1].ay;
	lastspeed[1].ax = lastspeed[0].ax;
	lastspeed[1].ay = lastspeed[0].ay;
	lastspeed[0].ax = car->_speed_X;
	lastspeed[0].ay = car->_speed_Y;
}

static double cT(double v)
{
    return ((int)v%991==0?((v/7)*991):((int)v%787==0?((v/787)*2):v/317));
}

// compute speed component parallel to the track.
float SingleCardata::getSpeed(tCarElt *car, float ltrackangle)
{
    v2d speed, dir;
    speed.x = car->_speed_X;
    speed.y = car->_speed_Y;
    dir.x = cos(ltrackangle);
    dir.y = sin(ltrackangle);
    return speed*dir;
}

void SingleCardata::init( CarElt *pcar )
{
    int i;

    car = pcar;
    for (i=0; i<4; i++)
    {
        corner1[i].ax = corner2[i].ax = car->_corner_x(i);
        corner1[i].ay = corner2[i].ay = car->_corner_y(i);
    }
    lastspeed[0].ax = lastspeed[1].ax = lastspeed[2].ax = car->_speed_X;
    lastspeed[0].ay = lastspeed[1].ay = lastspeed[2].ay = car->_speed_Y;

    t_m = t_m_f = t_m_r = 9999999.0f;
    lTT = (double)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_WTT, (char *)NULL, BT_ATT_WT);
    hTT = (double)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_HTT, (char *)NULL, iT() + (iT() - (tdble)lTT)*0.80f);
	fuelMassFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_FUEL_MASS_FACTOR, (char *)NULL, 1.0f);

    RH = 0.0f;

    for (i=0; i<4; i++)
    {
        static char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL,
                                     SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
        tdble mu = GfParmGetNum(car->_carHandle, WheelSect[i], PRM_MU, (char *)NULL, 1.0f);
        t_m = MIN(t_m, mu);
        if (i < 2)
            t_m_f = MIN(t_m_f, mu);
        else
            t_m_r = MIN(t_m_r, mu);

        RH += GfParmGetNum(car->_carHandle, WheelSect[i], (char *)PRM_RIDEHEIGHT, (char *)NULL, 0.20f);
    }

    // Aerodynamics
    RH *= 1.5f; RH = RH*RH; RH = 2.0f * exp(-3.0f * RH);
    tdble CL = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char *)NULL, 0.0f) +
               GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char *)NULL, 0.0f);
    float fwingarea = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGAREA, (char*) NULL, 0.0f);
    float fwingangle = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
    float rwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0f);
    float rwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
    float rwingArea = rwingarea * sin(rwingangle);
    float fwingArea = fwingarea * sin(fwingangle);
    float wingCA = 1.23f * (fwingArea + rwingArea);
    CA = RH * CL + 4.0f * wingCA;
    CA_FW = 4 * 1.23f * fwingArea;
    CA_RW = 4 * 1.23f * rwingArea;
    CA_GE = RH * CL;

    baseMass = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0f);
    fuel = 0.0f;
    mFTT = 0.0f;
    aFTT = 0.0f;
    aTT = 0.0f;
    carMu = fullCarMu = offlineFuelCarMu = 1.0f;
	lmTT = rmTT = 0.0;

    baseCarMu = (CA_FW * t_m_f + CA_RW * t_m_r + CA_GE * t_m) / baseMass;

    CTFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_CTFACTOR, NULL, 1.0f);
}

void SingleCardata::updateModel()
{
	aTT = 0.0f; aFTT = 0.0f; mFTT = 0.0f;
	float mLTT = 0.0f, mRTT = 0.0f;
    tdble mG = 0.0f;
    int i;

    for (i=0; i<4; i++)
    {
        double ct = CT(i);
        aTT += (tdble)ct;
        mG = MAX(mG, CG(i));
        if (i < 2)
        {
            aFTT += (tdble)ct;
            mFTT = MAX(mFTT, (tdble)ct);

			if (i == FRNT_LFT || i == REAR_LFT)
				mLTT = MAX(mLTT, (float)ct);
			else
				mRTT = MAX(mRTT, (float)ct);
        }
    }

    aTT /= 4;
    aFTT /= 2;
    fuel = car->_fuel;
    damage = (tdble)car->_dammage;

    tdble cTM = t_m, cTMF = t_m_f, cTMR = t_m_r;
    for (i=0; i<4; i++)
    {
        double ct = CT(i);
        tdble gF = 1.0f - mG/10.0f;

		//if (ct > hTT)
		//	gF -= MIN(0.20f, ((ct - hTT) / 15.0f) / 5.0f);
		if (ct < lTT)
			gF -= (tdble) MIN(0.75f, ((lTT - ct) / (((lTT-20.0)*0.75)*CTFactor)) / 15.0f);

        if (i < 2)
            cTMF = MIN(cTMF, t_m_f * gF);
        else
            cTMR = MIN(cTMR, t_m_r * gF);
    }

	lftOH = (tdble)MAX(0.0f, MAX(lmTT, mLTT) - hTT);
	rgtOH = (tdble)MAX(0.0f, MAX(rmTT, mRTT) - hTT);
	lmTT = rmTT = 0.0;

    cTM = MIN(cTMF, cTMR);

    fullCarMu = ((CA_FW * t_m_f + CA_RW * t_m_r + CA_GE * t_m) / (baseMass+car->_tank * fuelMassFactor)) / baseCarMu;
    offlineFuelCarMu = ((CA_FW * t_m_f + CA_RW * t_m_r + CA_GE * t_m) / (baseMass+fuel)) / baseCarMu;
    carMu = ((CA_FW * cTMF + CA_RW * cTMR + CA_GE * cTM) / (baseMass+fuel * fuelMassFactor)) / baseCarMu;
}


Cardata::Cardata(tSituation *s)
{
    ncars = s->_ncars;
    data = new SingleCardata[ncars];
    int i;
    for (i = 0; i < ncars; i++) {
        data[i].init(s->cars[i]);
    }
}


Cardata::~Cardata()
{
    delete [] data;
}


void Cardata::update()
{
    int i;
    for (i = 0; i < ncars; i++) {
        data[i].update();
    }
}


SingleCardata *Cardata::findCar(tCarElt *car)
{
    int i;
    for (i = 0; i < ncars; i++) {
        if (data[i].thisCar(car)) {
            return &data[i];
        }
    }
    return NULL;
}


