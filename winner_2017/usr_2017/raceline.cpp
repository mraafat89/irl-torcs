////////////////////////////////////////////////////////////////////////////
//
// K1999.cpp
//
// car driver for TORCS
// created:    (c) Remi Coulom March 2000
//
// modified:    2008 Andrew Sumner novocas7rian@gmail.com
// modified:    2009 John Isham isham.john@gmail.com
// modified:       2014 Michel Luc mluc@cern91.net
// last update:    2015 Andrew Sumner novocas7rian@gmail.com
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

//#include "portability.h" 
#include "tgf.h" 
#include "track.h" 
#include "car.h"
#include "raceman.h" 
#include "robot.h" 
#include "robottools.h"

#include "raceline.h"
#include "driver.h"
#include "line.h"
#include "manual_override.h"
#include "xmldefs.h"
#include "vardef.h"
#include "spline.h"

// #define LEARNING

#ifdef LEARNING
#include "learn.h"
#else
class LLearn { 
public:
	LLearn(void *handle) {} 
};
#endif

////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////

//
// These parameters are for the computation of the path
//
//static const double DivLength = 3.0;   // Length of path elements in meters

static const double SecurityR = 100.0; // Security radius
static double SideDistExt = 2.0; // Security distance wrt outside
static double SideDistInt = 1.0; // Security distance wrt inside          

#define myModuleName (char *)"usr_2016"

/////////////////////////////////////////////////////////////////////////////
// Some utility macros and functions
/////////////////////////////////////////////////////////////////////////////

static double Mag(double x, double y)
{
    return sqrt((x * x) + (y * y));
}

static double Min(double x1, double x2)
{
    if (x1 < x2)
        return x1;
    else
        return x2;
}

static double Max(double x1, double x2)
{
    if (x1 < x2)
        return x2;
    else
        return x1;
}

static double PointDist(vec2f *p1, vec2f *p2)
{
    double dx = (p1->x - p2->x);
    double dy = (p1->y - p2->y);
    return sqrt(dx*dx + dy*dy);
}

LRaceLine::LRaceLine(Driver *pdriver)
{
    driver = pdriver;
    iterations = 100;
    side_iterations = 20;
    fDirt = 0;
    Time = -1.0;
    rl_speed_mode = -1;
    overrideCollection = NULL;
	m_pTrack = NULL;
    CornerSpeedSlow = CornerSpeedMid = CornerSpeed = turnSpeed = offlineTurnSpeed = offlineBrakeDist = 0.0;
    CornerSpeedFactor = 1.0;
    outsideCornerSpeed = insideCornerSpeed = 0.0;
    brakeDist = brakeDistMid = brakeDistSlow = 0.0;
    ExtMargin = IntMargin = speedAdjust = wheelbase = wheeltrack = curveFactor = curveAccel = curveBrake = bumpCaution = offlineBumpCaution = 0.0;
    AvoidExtMargin = AvoidIntMargin = 0.0;
    slopeFactor = offlineSlopeFactor = fulltankPercent = midtankPercent = maxfuel = edgeLineMargin = 0.0;
    Width = Length = TargetSpeed = 0.0;
    saveTrack = loadTrack = Divs = DivLength = Segs = racelineOverride = fDirt = This = Prev = Next = 0;
    steer_verbose = LineIndex = LineSave = 0;
    minTurnInverse = 0.0;
    lookAhead = lookAheadEmpty = 10.0;
	lookAheadOut = -1.0;
	lookAheadColdFactor = 1.0;
    outsteerSpeedReducer = 1.0;
    steerSkidFactor = 0.9;
    steerSkidOfflineFactor = 0.5;
    errorCorrectionFactor = 1.0;
    last_lane = 0.0;
    steerTimeFactor = 7.0;
    outsideSteeringDampener = outsideSteeringDampenerOverlap = outsideSteeringDampenerAccel = 0.0;
    car = NULL;
    last_left_steer = last_rl_steer = last_right_steer = last_steer = last_last_steer = 0.0;
    last_steer_diff = 1000.0;
    useMergedSpeed = 0;
    last_target_raceline = -1;
    cornersteer = 0.0;
    cardata = NULL;

    coldTyreFactor = 0.8;

    tSegDist = (double *)malloc(MAXSEGMENTS * sizeof(double));
    tSegIndex = (int *)malloc(MAXSEGMENTS * sizeof(int));
    tSegment = (tTrackSeg **)malloc(MAXSEGMENTS * sizeof(tTrackSeg *));
    tElemLength = (double *)malloc(MAXSEGMENTS * sizeof(double));
    tDistance = (double *)malloc(MAXDIVS * sizeof(double));
    tMaxSpeed = (double *)malloc(MAXDIVS * sizeof(double));
    tMaxSpeedCold = (double *)malloc(MAXDIVS * sizeof(double));
    tzLeft = (double *)malloc(MAXDIVS * sizeof(double));
    tzRight = (double *)malloc(MAXDIVS * sizeof(double));
    tFriction = (double *)malloc(MAXDIVS * sizeof(double));
    txRight = (double **)malloc(NUM_RACELINES * sizeof(double *));

    int i;
	for (i = 0; i < MAXDIVS; i++)
		tDistance[i] = -1.0;

    for (i=0; i<NUM_RACELINES; i++)
    {
        txRight[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(txRight[i], 0, MAXDIVS * sizeof(double));
    }
    tyRight = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tyRight[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tyRight[i], 0, MAXDIVS * sizeof(double));
    }
    txLeft = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        txLeft[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(txLeft[i], 0, MAXDIVS * sizeof(double));
    }
    tyLeft = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tyLeft[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tyLeft[i], 0, MAXDIVS * sizeof(double));
    }
    tx = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tx[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tx[i], 0, MAXDIVS * sizeof(double));
    }
    ty = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        ty[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(ty[i], 0, MAXDIVS * sizeof(double));
    }
    tz = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tz[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tz[i], 0, MAXDIVS * sizeof(double));
    }
    tzd = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tzd[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tzd[i], 0, MAXDIVS * sizeof(double));
    }
    tLane = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tLane[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tLane[i], 0, MAXDIVS * sizeof(double));
    }
    tRInverse = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tRInverse[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tRInverse[i], 0, MAXDIVS * sizeof(double));
    }
    tSpeed = (double **)malloc(NUM_RACELINE_SPEEDS * sizeof(double *));
    for (i=0; i<NUM_RACELINE_SPEEDS; i++)
    {
        tSpeed[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tSpeed[i], 0, MAXDIVS * sizeof(double));
    }
    tDivSeg = (int **)malloc(NUM_RACELINES * sizeof(int *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tDivSeg[i] = (int *)malloc(MAXDIVS * sizeof(int));
        memset(tDivSeg[i], 0, MAXDIVS * sizeof(int));
    }
}

LRaceLine::~LRaceLine()
{
    free(tSegDist);
    free(tSegIndex);
    free(tSegment);
    free(tElemLength);
    free(tDistance);
    free(tMaxSpeed);
    free(tMaxSpeedCold);
    free(tzLeft);
    free(tzRight);
    free(tFriction);

	delete learn;

    int i;
    for (i=0; i<NUM_RACELINES; i++)
        free(tyRight[i]);
    free(tyRight);
    for (i=0; i<NUM_RACELINES; i++)
        free(txRight[i]);
    free(txRight);
    for (i=0; i<NUM_RACELINES; i++)
        free(tyLeft[i]);
    free(tyLeft);
    for (i=0; i<NUM_RACELINES; i++)
        free(txLeft[i]);
    free(txLeft);
    for (i=0; i<NUM_RACELINES; i++)
        free(tx[i]);
    free(tx);
    for (i=0; i<NUM_RACELINES; i++)
        free(ty[i]);
    free(ty);
    for (i=0; i<NUM_RACELINES; i++)
        free(tz[i]);
    free(tz);
    for (i=0; i<NUM_RACELINES; i++)
        free(tzd[i]);
    free(tzd);
    for (i=0; i<NUM_RACELINES; i++)
        free(tLane[i]);
    free(tLane);
    for (i=0; i<NUM_RACELINES; i++)
        free(tRInverse[i]);
    free(tRInverse);
    for (i=0; i<NUM_RACELINE_SPEEDS; i++)
        free(tSpeed[i]);
    free(tSpeed);
    for (i=0; i<NUM_RACELINES; i++)
        free(tDivSeg[i]);
    free(tDivSeg);
}

/////////////////////////////////////////////////////////////////////////////
// Update tx and ty arrays
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::UpdateTxTy(int i, int rl)
{
    tx[rl][i] = tLane[rl][i] * txRight[rl][i] + (1 - tLane[rl][i]) * txLeft[rl][i];
    ty[rl][i] = tLane[rl][i] * tyRight[rl][i] + (1 - tLane[rl][i]) * tyLeft[rl][i];
}

/////////////////////////////////////////////////////////////////////////////
// Set segment info
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::SetSegmentInfo(const tTrackSeg *pseg, double d, int i, double l)
{
    if (pseg)
    {
        tSegDist[pseg->id] = d;
        tSegIndex[pseg->id] = i;
        tElemLength[pseg->id] = l;
        if (pseg->id >= Segs)
            Segs = pseg->id + 1;
    }
}

/////////////////////////////////////////////////////////////////////////////
// Split the track into small elements
// ??? constant width supposed
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::SplitTrack(tTrack *ptrack, int rl, bool preLoaded)
{
    DivLength = (int)GfParmGetNum( car->_carHandle, SECT_PRIVATE, "DivLength", (char *)NULL, 3.0f );
    //DivLength = 3;
    Segs = 0;
	m_pTrack = ptrack;

    tTrackSeg *psegCurrent = ptrack->seg;

    double Distance = 0;
    double Angle = psegCurrent->angle[TR_ZS];
    double xlPos = psegCurrent->vertex[TR_SL].x;
    double ylPos = psegCurrent->vertex[TR_SL].y;
    double xrPos = psegCurrent->vertex[TR_SR].x;
    double yrPos = psegCurrent->vertex[TR_SR].y;
    double xPos = (xlPos + xrPos) / 2;
    double yPos = (ylPos + yrPos) / 2;

    int i = 0;

    std::fill_n(tFriction, MAXDIVS, psegCurrent->surface->kFriction);
    std::fill_n(txLeft[rl], MAXDIVS, 0.0);
    std::fill_n(txRight[rl], MAXDIVS, 0.0);
    std::fill_n(tyLeft[rl], MAXDIVS, 0.0);
    std::fill_n(tyRight[rl], MAXDIVS, 0.0);

    do
    {
        int Divisions = 1 + int(psegCurrent->length / DivLength);
        double Step = psegCurrent->length / Divisions;

        SetSegmentInfo(psegCurrent, Distance + Step, i, Step);
        for (int j = Divisions; --j >= 0;)
        {
            tDivSeg[rl][i] = psegCurrent->id;
            tSegment[psegCurrent->id] = psegCurrent;

            if (preLoaded)
            {
                Distance += Step;
                i++;
                continue;
            }

            double cosine = cos(Angle);
            double sine = sin(Angle);

            if (psegCurrent->type == TR_STR)
            {
                xPos += cosine * Step;
                yPos += sine * Step;
            }
            else
            {
                double r = psegCurrent->radius;
                double Theta = psegCurrent->arc / Divisions;
                double L = 2 * r * sin(Theta / 2);
                double x = L * cos(Theta / 2);
                double y;
                if (psegCurrent->type == TR_LFT)
                {
                    Angle += Theta;
                    y = L * sin(Theta / 2);
                }
                else
                {
                    Angle -= Theta;
                    y = -L * sin(Theta / 2);
                }
                xPos += x * cosine - y * sine;
                yPos += x * sine + y * cosine;
            }

            double dx = -psegCurrent->width * sin(Angle) / 2;
            double dy = psegCurrent->width * cos(Angle) / 2;
            txLeft[rl][i] = xPos + dx;
            tyLeft[rl][i] = yPos + dy;
            txRight[rl][i] = xPos - dx;
            tyRight[rl][i] = yPos - dy;
            /*
            if (rl == LINE_LEFT)
            {
                txRight[rl][i] = txLeft[rl][i] + (txRight[rl][i] - txLeft[rl][i]) * edgeLineMargin;
                tyRight[rl][i] = tyLeft[rl][i] + (tyRight[rl][i] - tyLeft[rl][i]) * edgeLineMargin;
            }
            else if (rl == LINE_RIGHT)
            {
                txLeft[rl][i] = txRight[rl][i] - (txRight[rl][i] - txLeft[rl][i]) * edgeLineMargin;
                tyLeft[rl][i] = tyRight[rl][i] - (tyRight[rl][i] - tyLeft[rl][i]) * edgeLineMargin;
            }
            else if (rl == LINE_MID)
            {
                double xright = txLeft[rl][i] + (txRight[rl][i] - txLeft[rl][i]) * 0.50;
                double yright = tyLeft[rl][i] + (tyRight[rl][i] - tyLeft[rl][i]) * 0.50;
                txLeft[rl][i] = txRight[rl][i] - (txRight[rl][i] - txLeft[rl][i]) * 0.50;
                tyLeft[rl][i] = tyRight[rl][i] - (tyRight[rl][i] - tyLeft[rl][i]) * 0.50;
                txRight[rl][i] = xright;
                tyRight[rl][i] = yright;
            }
            */
            tLane[rl][i] = 0.5;
            tFriction[i] = psegCurrent->surface->kFriction;
            if (tFriction[i] < 1) // ??? ugly trick for dirt
            {
                // fprintf("%s: dirt hack segment:%d\n",__FUNCTION__,i); 

                //tFriction[i] *= 0.90;
                fDirt = 1;
                SideDistInt = -1.5;
                SideDistExt = 0.0;
            }
            UpdateTxTy(i, rl);

            Distance += Step;

            i++;
        }

        psegCurrent = psegCurrent->next;
    } while (psegCurrent != ptrack->seg);

    if (!preLoaded)
        Divs = i - 1;

    Width = psegCurrent->width;
    Length = Distance;
}

/////////////////////////////////////////////////////////////////////////////
// Compute the inverse of the radius
/////////////////////////////////////////////////////////////////////////////

double LRaceLine::GetRInverse(double prevx, double prevy, double x, double y, double nextx, double nexty)
{
    double x1 = nextx - x;
    double y1 = nexty - y;
    double x2 = prevx - x;
    double y2 = prevy - y;
    double x3 = nextx - prevx;
    double y3 = nexty - prevy;

    double det = x1 * y2 - x2 * y1;
    double n1 = x1 * x1 + y1 * y1;
    double n2 = x2 * x2 + y2 * y2;
    double n3 = x3 * x3 + y3 * y3;
    double nnn = sqrt(n1 * n2 * n3);

    return 2.0 * det / nnn;
}

double LRaceLine::GetRInverse(int prev, double x, double y, int next, double *tX, double *tY)
{
    double x1 = tX[next] - x;
    double y1 = tY[next] - y;
    double x2 = tX[prev] - x;
    double y2 = tY[prev] - y;
    double x3 = tX[next] - tX[prev];
    double y3 = tY[next] - tY[prev];

    double det = x1 * y2 - x2 * y1;
    double n1 = x1 * x1 + y1 * y1;
    double n2 = x2 * x2 + y2 * y2;
    double n3 = x3 * x3 + y3 * y3;
    double nnn = sqrt(n1 * n2 * n3);

    return 2.0 * det / nnn;
}

/////////////////////////////////////////////////////////////////////////////
// Change lane value to reach a given radius
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::AdjustRadius(int prev, int i, int next, double TargetRInverse, int rl, double Security)
{
    double OldLane = tLane[rl][i];

    //
    // Start by aligning points for a reasonable initial lane
    //
    tLane[rl][i] = (-(ty[rl][next] - ty[rl][prev]) * (txLeft[rl][i] - tx[rl][prev]) +
        (tx[rl][next] - tx[rl][prev]) * (tyLeft[rl][i] - ty[rl][prev])) /
        ((ty[rl][next] - ty[rl][prev]) * (txRight[rl][i] - txLeft[rl][i]) -
        (tx[rl][next] - tx[rl][prev]) * (tyRight[rl][i] - tyLeft[rl][i]));
    if (tLane[rl][i] < -0.2)
        tLane[rl][i] = -0.2;
    else if (tLane[rl][i] > 1.2)
        tLane[rl][i] = 1.2;
    UpdateTxTy(i, rl);

    //
    // Newton-like resolution method
    //
    const double dLane = 0.0001;

    double dx = dLane * (txRight[rl][i] - txLeft[rl][i]);
    double dy = dLane * (tyRight[rl][i] - tyLeft[rl][i]);

    double dRInverse = GetRInverse(prev, tx[rl][i] + dx, ty[rl][i] + dy, next, tx[rl], ty[rl]);
    //double dRInverse = GetRInverse(prev, tx[rl][i] + dx, ty[rl][i] + dy, next, rl);

    if (dRInverse > 0.000000001)
    {
        tLane[rl][i] += (dLane / dRInverse) * TargetRInverse;

        /****************** Get Extern/Intern Margin **********************/
#if 0
        double ExtMarginAddition = GfParmGetNum( car->_carHandle, SECT_PRIVATE, "extmargin", (char *)NULL, 0.0 );
        double IntMarginAddition = GfParmGetNum( car->_carHandle, SECT_PRIVATE, "intmargin", (char *)NULL, 0.0 );

        double ExtLane = (0.5 + ExtMarginAddition + Security) / Width;
        double IntLane = (0.5 + IntMarginAddition + Security) / Width;

#else
        double ExtLane = (getExtMargin(rl, i, TargetRInverse) + Security) / Width;
        double IntLane = (getIntMargin(rl, i, TargetRInverse) + Security) / Width;
#endif
        if (ExtLane > 0.5 && rl >= LINE_RL)
            ExtLane = 0.5;
        if (IntLane > 0.5 && rl >= LINE_RL)
            IntLane = 0.5;

        if (TargetRInverse >= 0.0)
        {
            if (tLane[rl][i] < IntLane)
                tLane[rl][i] = IntLane;
            if (1 - tLane[rl][i] < ExtLane)
            {
                if (1 - OldLane < ExtLane)
                    tLane[rl][i] = Min(OldLane, tLane[rl][i]);
                else
                    tLane[rl][i] = 1 - ExtLane;
            }
        }
        else
        {
            if (tLane[rl][i] < ExtLane)
            {
                if (OldLane < ExtLane)
                    tLane[rl][i] = Max(OldLane, tLane[rl][i]);
                else
                    tLane[rl][i] = ExtLane;
            }
            if (1 - tLane[rl][i] < IntLane)
                tLane[rl][i] = 1 - IntLane;
        }

        if (rl < LINE_RL)
        {
            tLane[rl][i] += TargetRInverse / 15;
        }
    }

    UpdateTxTy(i, rl);
}

/////////////////////////////////////////////////////////////////////////////
// Smooth path
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::Smooth(int Step, int rl)
{
    int prev = ((Divs - Step) / Step) * Step;
    int prevprev = prev - Step;
    int next = Step;
    int nextnext = next + Step;

    for (int i = 0; i <= Divs - Step; i += Step)
    {
        double ri0 = GetRInverse(prevprev, tx[rl][prev], ty[rl][prev], i, tx[rl], ty[rl]);
        double ri1 = GetRInverse(i, tx[rl][next], ty[rl][next], nextnext, tx[rl], ty[rl]);
        //double ri0 = GetRInverse(prevprev, tx[rl][prev], ty[rl][prev], i, rl);
        //double ri1 = GetRInverse(i, tx[rl][next], ty[rl][next], nextnext, rl);
        double lPrev = Mag(tx[rl][i] - tx[rl][prev], ty[rl][i] - ty[rl][prev]);
        double lNext = Mag(tx[rl][i] - tx[rl][next], ty[rl][i] - ty[rl][next]);

        double TargetRInverse = (lNext * ri0 + lPrev * ri1) / (lNext + lPrev);

        double Security = lPrev * lNext / (8 * SecurityR);

        if (rl >= LINE_RL)
        {
            if (ri0 * ri1 > 0)
            {
                double ac1 = fabs(ri0);
                double ac2 = fabs(ri1);
                {
                    double cf = getCurveFactor(i, (ac1 < ac2));
                    if (ac1 < ac2) // curve is increasing
                    {
                        ri0 += cf * (ri1 - ri0);
                    }
                    else if (ac2 < ac1) // curve is decreasing
                    {
                        ri1 += cf * (ri0 - ri1);
                    }
                }

                TargetRInverse = (lNext * ri0 + lPrev * ri1) / (lNext + lPrev);
            }
        }

        AdjustRadius(prev, i, next, TargetRInverse, rl, Security);

        prevprev = prev;
        prev = i;
        next = nextnext;
        nextnext = next + Step;
        if (nextnext > Divs - Step)
            nextnext = 0;
    }
}

/////////////////////////////////////////////////////////////////////////////
// Interpolate between two control points
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::StepInterpolate(int iMin, int iMax, int Step, int rl)
{
    int next = (iMax + Step) % Divs;
    if (next > Divs - Step)
        next = 0;

    int prev = (((Divs + iMin - Step) % Divs) / Step) * Step;
    if (prev > Divs - Step)
        prev -= Step;

    double ir0 = GetRInverse(prev, tx[rl][iMin], ty[rl][iMin], iMax % Divs, tx[rl], ty[rl]);
    double ir1 = GetRInverse(iMin, tx[rl][iMax % Divs], ty[rl][iMax % Divs], next, tx[rl], ty[rl]);
    //double ir0 = GetRInverse(prev, tx[rl][iMin], ty[rl][iMin], iMax % Divs, rl);
    //double ir1 = GetRInverse(iMin, tx[rl][iMax % Divs], ty[rl][iMax % Divs], next, rl);
    for (int k = iMax; --k > iMin;)
    {
        double x = double(k - iMin) / double(iMax - iMin);
        double TargetRInverse = x * ir1 + (1 - x) * ir0;
        AdjustRadius(iMin, k, iMax % Divs, TargetRInverse, rl);
    }
}

/////////////////////////////////////////////////////////////////////////////
// Calls to StepInterpolate for the full path
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::Interpolate(int Step, int rl)
{
    if (Step > 1)
    {
        int i;
        for (i = Step; i <= Divs - Step; i += Step)
            StepInterpolate(i - Step, i, Step, rl);
        StepInterpolate(i - Step, Divs, Step, rl);
    }
}


/*===========================================================*/
/*                  SET DATA                                 */
/*===========================================================*/

void LRaceLine::setRwData(tTrack* t, void **carParmHandle, tSituation *s)
{
    return;
}

/**********************************************************/
double LRaceLine::getMaxSpeed(int Div, int rl)
{
    if (overrideCollection)
    {
		LManualOverride *labelOverride;
		if (rl == LINE_RIGHT)
		{
			if (NULL != (labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_MAX_SPEED)))
			{
				double speed_override;
				if (labelOverride->getOverrideValue(Div, &speed_override))
					return speed_override;
			}
		}

		if (rl == LINE_LEFT)
		{
			if (NULL != (labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_MAX_SPEED)))
			{
				double speed_override;
				if (labelOverride->getOverrideValue(Div, &speed_override))
					return speed_override;
			}
		}

        labelOverride = overrideCollection->getOverrideForLabel(PRV_MAX_SPEED);
        if (labelOverride)
        {
            double speed_override;
            if (labelOverride->getOverrideValue(Div, &speed_override))
                return speed_override;
        }
    }

    return 10000.0;
}

double LRaceLine::getCurveFactor(int Div, bool isBraking)
{
    if (overrideCollection)
    {
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_RACELINECURVE);
        if (labelOverride)
        {
            double curve_override;
            if (labelOverride->getOverrideValue(Div, &curve_override))
                return curve_override;
        }
    }

    if (isBraking)
        return curveBrake;

    return curveAccel;
}

double LRaceLine::getBumpCaution(int Div, int rl)
{
    double bc_override;
    LManualOverride *labelOverride;

    if (rl >= LINE_RL)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BUMP_CAUTION);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &bc_override))
                    return bc_override;
            }
        }

        return bumpCaution;
    }
    else if (rl == LINE_LEFT)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_BUMP_CAUTION);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &bc_override))
                    return bc_override;
            }
        }
    }
    else if (rl == LINE_RIGHT)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_BUMP_CAUTION);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &bc_override))
                    return bc_override;
            }
        }
    }

    return offlineBumpCaution;
}

double LRaceLine::getCornerSpeed(int Div, int rl)
{
#ifdef LEARNING
	return learn->GetCornerSpeed(Div);
#endif

	double cornerspeed_override = CornerSpeed, cornerspeed_cold_override = CornerSpeed;
    LManualOverride *labelOverride;

	// evaluate cold factor;
	double coldfactor = 1.0;

    if (rl == LINE_RL)
    {
        if (overrideCollection)
        {
			labelOverride = overrideCollection->getOverrideForLabel(PRV_CORNERSPEED);
			if (labelOverride)
				if (!(labelOverride->getOverrideValue(Div, &cornerspeed_override)))
					cornerspeed_override = cornerspeed_cold_override = CornerSpeed;
			labelOverride = overrideCollection->getOverrideForLabel(PRV_CORNERSPEED_COLD);
			if (labelOverride)
				if (!(labelOverride->getOverrideValue(Div, &cornerspeed_cold_override)))
					cornerspeed_cold_override = cornerspeed_override;
	
			if (cardata->aFTT < 40.0)
				coldfactor = MIN(1.0, MAX(0.5, cornerspeed_cold_override / cornerspeed_override));
			else if (cardata->aFTT < cardata->lTT)
				coldfactor = MIN(1.0, MAX(0.5, (cardata->aFTT - 35) / (cardata->lTT - (cardata->lTT-35)/2)));

			if (cardata->aFTT < 40.0)
				return cornerspeed_cold_override * CornerSpeedFactor;
			else if (cardata->aFTT < cardata->lTT)
				return (cornerspeed_cold_override + ((cardata->aFTT - 35.0) / (cardata->lTT - (cardata->lTT-35.0)/2)) * (cornerspeed_override - cornerspeed_cold_override)) * CornerSpeedFactor;
			else
				return cornerspeed_override * CornerSpeedFactor;
        }

        return CornerSpeed * CornerSpeedFactor;
    }
    else if (rl == LINE_LEFT)
    {
        if (overrideCollection)
        {
			labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFTCORNERSPEED);
			if (labelOverride)
			{
				if (labelOverride->getOverrideValue(Div, &cornerspeed_override) && cornerspeed_override > 1.0)
				{
					cornerspeed_cold_override = cornerspeed_override;
					labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFTCORNERSPEED_COLD);
					if (labelOverride)
					{
						if (!(labelOverride->getOverrideValue(Div, &cornerspeed_cold_override)))
							cornerspeed_cold_override = cornerspeed_override;

						if (cardata->aFTT < 40.0)
							coldfactor = MIN(1.0, MAX(0.5, cornerspeed_cold_override / cornerspeed_override));
						else if (cardata->aFTT < cardata->lTT)
							coldfactor = MIN(1.0, MAX(0.5, (cardata->aFTT - 20.0) / (cardata->lTT - (cardata->lTT-20)/2)));
					}

					return cornerspeed_override * coldfactor * CornerSpeedFactor;
				}
			}
        }

        if (tRInverse[LINE_RL][Div] < -0.001)
            return outsideCornerSpeed * CornerSpeedFactor;
        else if (tRInverse[LINE_RL][Div] > 0.001)
            return insideCornerSpeed * CornerSpeedFactor;
    }
    else if (rl == LINE_RIGHT)
    {
        if (overrideCollection)
        {
			labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHTCORNERSPEED);
			if (labelOverride)
			{
				if (labelOverride->getOverrideValue(Div, &cornerspeed_override) && cornerspeed_override > 1.0)
				{
					cornerspeed_cold_override = cornerspeed_override;
					labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHTCORNERSPEED_COLD);
					if (labelOverride)
					{
						if (!(labelOverride->getOverrideValue(Div, &cornerspeed_cold_override)))
							cornerspeed_cold_override = cornerspeed_override;

						if (cardata->aFTT < 40)
							coldfactor = MIN(1.0, MAX(0.5, cornerspeed_cold_override / cornerspeed_override));
						else if (cardata->aFTT < cardata->lTT)
							coldfactor = MIN(1.0, MAX(0.5, (cardata->aFTT - 20.0) / (cardata->lTT - (cardata->lTT - 20)/2)));
					}

					return cornerspeed_override * coldfactor * CornerSpeedFactor;
				}
			}
        }

        if (tRInverse[LINE_RL][Div] > 0.001)
            return outsideCornerSpeed * CornerSpeedFactor;
        else if (tRInverse[LINE_RL][Div] < -0.001)
            return insideCornerSpeed * CornerSpeedFactor;
    }

    return offlineTurnSpeed * CornerSpeedFactor;
}

/**********************************************************/
double LRaceLine::getBrakeDist(int Div, int rl)
{
#ifdef LEARNING
	return learn->GetBrakeDelay(Div);
#endif

    double brakedelay_override = brakeDist;
    LManualOverride *labelOverride = NULL;

	double coldfactor = 1.0;
	if (overrideCollection)
	{
		double brakedelay_cold_override = brakeDist;
		if (rl == LINE_LEFT)
		    labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_BRAKEDELAY_COLD);
		if (rl == LINE_RIGHT)
		    labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_BRAKEDELAY_COLD);
		if (!labelOverride)
    		labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY_COLD);

		if (labelOverride)
		{
			if (!(labelOverride->getOverrideValue(Div, &brakedelay_cold_override)))
				brakedelay_cold_override = brakedelay_override;
			else
			{
				labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY);
				if (!(labelOverride->getOverrideValue(Div, &brakedelay_override)))
					brakedelay_override = brakeDist;

				if (cardata->aFTT < 40.0)
		   			coldfactor = MIN(1.0, brakedelay_cold_override / brakedelay_override);
				else if (cardata->aFTT < cardata->lTT)
					coldfactor = MIN(1.0, (cardata->aFTT - 20.0) / (cardata->lTT - (cardata->lTT - 20)/2));
			}
		}
	}

    if (rl == LINE_RL_SLOW && hasSlow)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY_SLOW);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override * coldfactor;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY_MID);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override * coldfactor;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override * coldfactor;
            }
        }

        if (brakeDistSlow > 1.0)
            return brakeDistSlow * coldfactor;
        if (brakeDistMid > 1.0)
            return brakeDistMid * coldfactor;
        return brakeDist * coldfactor;
    }
    else if (rl == LINE_RL_MID && hasMid)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY_MID);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override * coldfactor;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override * coldfactor;
            }
        }

        if (brakeDistMid > 1.0)
            return brakeDistMid * coldfactor;
        return brakeDist * coldfactor;
    }
    else if (rl == LINE_RL)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override * coldfactor;
            }
        }

        return brakeDist * coldfactor;
    }
	else if (rl == LINE_LEFT)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_BRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override * coldfactor;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_AVOIDBRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override * coldfactor;
            }
        }
    }
	else if (rl == LINE_RIGHT)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_BRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override * coldfactor;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_AVOIDBRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override * coldfactor;
            }
        }
    }

    return offlineBrakeDist * coldfactor;
}

/**********************************************************/
double LRaceLine::getIntMargin(int raceline, int Div, double rInverse)
{
	double extramargin = 0.0;// MAX(0.0, ((raceline == LINE_RL ? cardata->fuelCarMu : cardata->offlineFuelCarMu) - cardata->carMu) * 10);
    double intmargin = IntMargin;
    LManualOverride *labelOverride;

    if (raceline < LINE_RL)
        intmargin += AvoidIntMargin;

    if (rInverse <= 0.0)
    {
        if (overrideCollection)
        {
            if (raceline >= LINE_RL)
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RL_RIGHT_MARGIN);
                if (labelOverride)
                {
					if (labelOverride->getOverrideValue(Div, &extramargin))
						return (intmargin + extramargin);
                }
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (intmargin + extramargin);
                }
            }
            else
            {
                if (raceline == LINE_LEFT)
                    intmargin = MAX(intmargin, Width - (Width * edgeLineMargin));
                else if (raceline == LINE_MID)
                    intmargin = MAX(intmargin, Width * 0.4);
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (intmargin + extramargin);
                }
            }
        }
    }
    else //if (rInverse > 0.0)
    {
        if (overrideCollection)
        {
            if (raceline >= LINE_RL)
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RL_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (intmargin + extramargin);
                }
                labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (intmargin + extramargin);
                }
            }
            else
            {
                if (raceline == LINE_RIGHT)
                    intmargin = MAX(intmargin, Width - (Width * edgeLineMargin));
                else if (raceline == LINE_MID)
                    intmargin = MAX(intmargin, Width * 0.4);
                labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (intmargin + extramargin);
                }
            }
        }
    }

    return intmargin + extramargin;
}

/**********************************************************/
double LRaceLine::getExtMargin(int raceline, int Div, double rInverse)
{
	double extramargin = 0.0; // MAX(0.0, ((raceline == LINE_RL ? cardata->fuelCarMu : cardata->offlineFuelCarMu) - cardata->carMu) * 10);
    double extmargin = ExtMargin;
    LManualOverride *labelOverride;

    if (raceline < LINE_RL)
        extmargin += AvoidExtMargin;

#if 0
    tTrackSeg *pseg = car->_trkPos.seg;
    const double    MIN_MU = pseg->surface->kFriction * 0.8;
    const double    MAX_ROUGH = MX(0.005, pseg->surface->kRoughness * 1.2);
    const double    MAX_RESIST = MX(0.04, pseg->surface->kRollRes * 1.2);
    double right_margin = 0.0, left_margin = 0.0;

    for( int s = 0; s < 2; s++ )
    {
        tTrackSeg*  pSide = pseg->side[s];
        if( pSide == 0 )
            continue;

        double  extraW = 0;

        bool  done = false;
        while( !done && pSide )
        {
            double  w = pSide->startWidth +
                        (pSide->endWidth - pSide->startWidth) * t;
//          w = MX(0, w - 0.5);

//          w = MN(w, 1.0);
            if( pSide->style == TR_CURB )
            {
                // always keep 1 wheel on main track.
                w = MN(w, 1.5);
                done = true;

                if( (s == TR_SIDE_LFT && pseg->type == TR_RGT ||
                   s == TR_SIDE_RGT && pseg->type == TR_LFT) &&
                    pSide->surface->kFriction  < pseg->surface->kFriction )
                    // keep a wheel on the good stuff.
                    w = 0;//MN(w, 1.5);

                // don't go too far up raised curbs (max 2cm).
                if( pSide->height > 0 )
                    w = MN(w, 0.6);
//                  w = MN(0.05 * pSide->width / pSide->height, 1.5);

//              if( pSide->surface->kFriction  < MIN_MU )
//                  w = 0;
          }
          else if( pSide->style == TR_PLAN )
          {
              if( /* FIXME */ (inPit && pitSide == s)               ||
                  (pSide->raceInfo & (TR_SPEEDLIMIT | TR_PITLANE)) )
              {
                  w = 0;
                  done = true;
              }

              if( s == m_sideMod.side &&
                  i >= m_sideMod.start &&
                  i <= m_sideMod.end )
              {
                  if( w > 0.5 )
                    { w = 0.5; done = true; }
              }
              else
                  if( pSide->surface->kFriction  < MIN_MU    ||
                      pSide->surface->kRoughness > MAX_ROUGH  ||
                      pSide->surface->kRollRes   > MAX_RESIST  ||
                      fabs(pSide->Kzw - SLOPE) > 0.005 )
                  {
                      bool  inner = 
                                    s == TR_SIDE_LFT && pseg->type == TR_LFT ||
                                    s == TR_SIDE_RGT && pseg->type == TR_RGT;
                      w = 0;//inner ? MN(w, 0.5) : 0;
                      done = true;
                  }

                  if( (s == TR_SIDE_LFT && pseg->type == TR_RGT ||
                       s == TR_SIDE_RGT && pseg->type == TR_LFT) &&
                        pSide->surface->kFriction  < pseg->surface->kFriction )
                  {
                      // keep a wheel on the good stuff.
                      w = 0;//MN(w, 1.5);
                      done = true;
                  }
                  if (done && pSide->side[s])
                  {
                    if (pSide->side[s]->style == TR_WALL || pSide->side[s]->style == TR_FENCE)
                    {
                      // keep off the walls!
                      w -= 0.5;
                    }
                  }
                  else
                    w -= 0.5;
          }
          else
          {
            // wall of some sort.
            w = -1;
            /*
            w = pSide->style == TR_WALL ? -0.5 :
//              pSide->style == TR_FENCE ? -0.1 : 0;
              0;
              */
            done = true;
          }

          if (s == TR_SIDE_LFT)
            left_margin = w;
          else if (s == TR_SIDE_RGT)
            right_margin = w;

//          if( pSide->style != TR_PLAN || w <= 0 )
//            done = true;

          pSide = pSide->side[s];
        }
    }
#endif

    if (rInverse >= 0.0)
    {
        if (overrideCollection)
        {
            if (raceline >= LINE_RL)
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RL_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (extmargin + extramargin);
                }
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (extmargin + extramargin);
                }
            }
            else
            {
                if (raceline == LINE_LEFT)
                    extmargin = MAX(extmargin, Width - (Width * edgeLineMargin));
                else if (raceline == LINE_MID)
                    extmargin = MAX(extmargin, Width * 0.4);
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (extmargin + extramargin);
                }
            }
        }
    }
    else// if (rInverse < 0.0)
    {
        if (overrideCollection)
        {
            if (raceline >= LINE_RL)
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RL_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (extmargin + extramargin);
                }
                labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (extmargin + extramargin);
                }
            }
            else
            {
                if (raceline == LINE_RIGHT)
                    extmargin = MAX(extmargin, Width - (Width * edgeLineMargin));
                else if (raceline == LINE_MID)
                    extmargin = MAX(extmargin, Width * 0.4);
                labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return (extmargin + extramargin);
                }
            }
        }
    }

    return extmargin + extramargin;
}

/////////////////////////////////////////////////////////////////////////////
// Find changes in line height
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::CalcZCurvature(int rl)
{
    int i;

    for (i = 0; i < Divs; i++)
    {
        tTrackSeg *seg = tSegment[tDivSeg[rl][i]];

        tz[rl][i] = RtTrackHeightG(seg, (tdble)tx[rl][i], (tdble)ty[rl][i]);

        int next = (i + 1) % Divs;
        int prev = (i - 1 + Divs) % Divs;
        double rInverse = GetRInverse(prev, tx[rl][i], ty[rl][i], next, tx[rl], ty[rl]);
        //double rInverse = GetRInverse(prev, tx[rl][i], ty[rl][i], next, rl);
        tRInverse[rl][i] = rInverse;
    }

    for (i = 0; i < Divs; i++)
    {
        int j = ((i - 1) + Divs) % Divs;

        vec2f pi, pj;
        pi.x = (tdble)tx[rl][i]; pi.y = (tdble)ty[rl][i];
        pj.x = (tdble)tx[rl][j]; pj.y = (tdble)ty[rl][j];

        tzd[rl][i] = (tz[rl][i] - tz[rl][j]) / PointDist(&pi, &pj);
    }

    for (i = 0; i < Divs; i++)
    {
        double zd = 0.0;
        for (int nx = 0; nx < 4; nx++)
        {
            int nex = (i + nx) % Divs;
            if (tzd[rl][nex] < 0.0)
                zd += tzd[rl][nex] * 2;
            else
                zd += tzd[rl][nex] * 0.2;
        }

        double camber = SegCamber(rl, i) - 0.002;
        if (camber < 0.0)
        {
            camber *= 5;
            if (rl == LINE_MID)
                camber *= 2;
        }
        double slope = camber + zd / 3 * (rl >= LINE_RL ? slopeFactor : offlineSlopeFactor);
        if (rl < LINE_RL)
        {
            if (slope < 0.0)
                slope *= 1.4;
            else
                slope *= 0.2;
        }

		if (rl == LINE_RL)
	        tFriction[i] *= 1.0 + MAX(-0.4, slope);

        /*
        if (slope < 0.0)
            tBrakeFriction[rl][i] = 1.0 + MAX(-0.4, slope / 10);
        else
            tBrakeFriction[rl][i] = 1.0 + slope / 40;
            */
    }
}

double LRaceLine::SegCamber(int rl, int div)
{
    tTrackSeg *seg = tSegment[tDivSeg[rl][div]];
    double camber = (((seg->vertex[TR_SR].z - seg->vertex[TR_SL].z) / 2) + ((seg->vertex[TR_ER].z - seg->vertex[TR_EL].z) / 2)) / seg->width;
    double camber1 = ((seg->vertex[TR_SR].z - seg->vertex[TR_SL].z)) / seg->width;
    double camber2 = ((seg->vertex[TR_ER].z - seg->vertex[TR_EL].z)) / seg->width;

    if (tRInverse[rl][div] < 0.0)
    {
        camber = -camber;
        camber2 = -camber2;
        camber1 = -camber1;
    }
    if (camber2 < camber1)
        camber = camber2;

    return camber;
}

double LRaceLine::SegCamberForNext()
{
    return SegCamber(LINE_RL, Next);
}


/*===========================================================*/
/*                INIT TRACK                                 */
/*===========================================================*/
void LRaceLine::InitTrack(tTrack* track, tSituation *p)
{
	learn = new LLearn(car->_carHandle);

    iterations = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_ITERATIONS, (char *)NULL, 100.0);
    side_iterations = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_SIDE_ITERATIONS, (char *)NULL, (tdble)iterations/5.0f);
    offlineTurnSpeed = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OFFLINE_TURNSPEED, (char *)NULL, 10.0);
    outsideCornerSpeed = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OUTSIDECORNERSPEED, (char *)NULL, (tdble)offlineTurnSpeed);
    insideCornerSpeed = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_INSIDECORNERSPEED, (char *)NULL, (tdble)offlineTurnSpeed);
    CornerSpeedFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_CORNERSPEED_FACTOR, (char *)NULL, (tdble)1.0);
    minTurnInverse = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_MINTURNINVERSE, (char *)NULL, (tdble) 0.0028);
    speedAdjust = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_SPEEDADJUST, (char *)NULL, 0.0);
    brakeDist = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKEDELAY, (char *)NULL, 15.0);
    brakeDistMid = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKEDELAY_MID, (char *)NULL, 15.0);
    brakeDistSlow = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKEDELAY_SLOW, (char *)NULL, 15.0);
    offlineBrakeDist = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OFFLINE_BRAKEDELAY, (char *)NULL, 15.0);
    bumpCaution = GfParmGetNum(car->_carHandle, SECT_PRIVATE, "bumpcaution", (char *)NULL, 0.2f);
    offlineBumpCaution = GfParmGetNum(car->_carHandle, SECT_PRIVATE, "offlinebumpcaution", (char *)NULL, 0.2f);
    slopeFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, "slopefactor", (char *)NULL, 5.0);
    offlineSlopeFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, "offlineslopefactor", (char *)NULL, 5.0);
    curveFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_CURVE_FACTOR, (char *)NULL, (tdble) 0.13);
    curveAccel = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_ACCEL_CURVE, (char *)NULL, (tdble)curveFactor);
    curveBrake = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKE_CURVE, (char *)NULL, (tdble)curveFactor);
    racelineOverride = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, "raceline_override", (char *)NULL, -1.0);
    racelineDebug = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, "raceline debug", (char *)NULL, 0.0);
    fulltankPercent = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_FULLTANK_PERCENT, (char *)NULL, 2.0);
    midtankPercent = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_MIDTANK_PERCENT, (char *)NULL, 2.0);
    maxfuel = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_TANK, (char*)NULL, 100.0f) - 15.0;
    edgeLineMargin = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_EDGE_LINE_MARGIN, (char*)NULL, 0.3f);
    lookAhead = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_LOOKAHEAD, (char*)NULL, 10.0f);
    lookAheadOut = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_LOOKAHEAD_OUT, (char*)NULL, -1.0f);
    lookAheadEmpty = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_LOOKAHEAD_EMPTY, (char*)NULL, (tdble)lookAhead);
    lookAheadColdFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_LOOKAHEAD_CFACTOR, (char*)NULL, (tdble)1.0f);
    outsteerSpeedReducer = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OUTSTEER_REDUCER, (char*)NULL, 1.0f);
    steerSkidFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_STEER_SKID, (char*)NULL, 0.9f);
    steerSkidOfflineFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_STEER_SKID_OFFLINE, (char*)NULL, 0.5f);
    errorCorrectionFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_ERROR_CORRECTION, (char*)NULL, 1.0f);
    outsideSteeringDampener = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OUTSIDE_DAMPENER, (char*)NULL, 0.5f);
    outsideSteeringDampenerOverlap = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OUTSIDE_DAMPENER_O, (char*)NULL, 0.5f);
    outsideSteeringDampenerAccel = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OUTSIDE_DAMPENER_A, (char*)NULL, (tdble)outsideSteeringDampener*0.9f);
    loadTrack = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_LOAD_TRACK, (char*)NULL, 0.0f);
    saveTrack = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_SAVE_TRACK, (char*)NULL, 0.0f);
    useMergedSpeed = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_USE_MERGED_SPEED, (char*)NULL, 0.0f);
    ExtMargin = (tdble) GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_EXTMARGIN, (char *)NULL, (tdble) 1.4 );
    IntMargin = (tdble) GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_INTMARGIN, (char *)NULL, (tdble) 0.3 );
    AvoidExtMargin = (tdble) GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_AVOID_EXTMARGIN, (char *)NULL, (tdble) 1.4 );
    AvoidIntMargin = (tdble) GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_AVOID_INTMARGIN, (char *)NULL, (tdble) 1.0 );
    steerTimeFactor = (tdble) GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_STEERTIMEFACTOR, (char *)NULL, (tdble) 7.0 );
    cornersteer = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_CORNERSTEER, (char *)NULL, (tdble) 0.0 );
    coldTyreFactor = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_COLDTYREFACTOR, (char *)NULL, (tdble) coldTyreFactor );
    lastUpdateDist = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_LAST_UPDATE_DIST, (char *)NULL, (tdble) -1.0f );
    stopUpdateDist = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_STOP_UPDATE_DIST, (char *)NULL, (tdble) 500.0f );
    resumeUpdateDist = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_RESUME_UPDATE_DIST, (char *)NULL, (tdble) -1.0f );
	hasLastUpdate = false;

	m_raceType = p->_raceType;

    // split track
    for (int i=0; i<NUM_RACELINES; i++)
    {
        for (int j=0; j<MAXDIVS; j++)
        {
            tRInverse[i][j] = 0;
            tLane[i][j] = 0;
        }
    }

    bool trackPreLoaded = false;
    //hasSlow = false, hasMid = false;
    hasSlow = true, hasMid = true;

    int i;
    for (int rl = LINE_RL; rl >= LINE_LEFT; rl--)
    {
		if (rl == LINE_RL)
		{
			/*Split the track into small elements*/
			SplitTrack(track, rl, trackPreLoaded);

			if (!trackPreLoaded)
			{
				// Smoothing loop
				int Iter = (rl < LINE_RL ? side_iterations : iterations);
				for (int Step = 128; (Step /= 2) > 0;)
				{
					for (i = Iter * int(sqrt((float)Step)); --i >= 0;)
						Smooth(Step, rl);
					Interpolate(Step, rl);
				}

				CalcZCurvature(rl);
			}
		}
		else
		{
			CreateSideRaceline(rl);
			CalcZCurvature(rl);
		}
    }

	SmoothSideRacingLines();
//	for (i = 0; i < Divs; i++)
//		fprintf(stderr, "%d: L%.5f - %.5f - R%.5f\n", i, tRInverse[LINE_LEFT][i], tRInverse[LINE_RL][i], tRInverse[LINE_RIGHT][i]);
//	fflush(stderr);
}

void LRaceLine::SmoothSideRacingLines()
{
	for (int rl = LINE_LEFT; rl <= LINE_RIGHT; rl++)
	{
		int altRL = (rl == LINE_LEFT ? LINE_RIGHT : LINE_LEFT);
		for (int div = 0; div < Divs; div++)
		{
			double prevRI = tRInverse[rl][(div - 1 + Divs) % Divs];
			if (fabs(tRInverse[rl][div]) > fabs(tRInverse[altRL][div]) * 1.5)
				tRInverse[rl][div] = tRInverse[altRL][div] * 1.5;

			if (fabs(tRInverse[rl][div]) > fabs(prevRI) * 2)
			{
				// more smoothing required, get the average of 7 divs
				double rI = 0.0;
				for (int i = -4; i <= 2; i++)
				{
					rI += tRInverse[rl][(div + i + Divs) % Divs];
				}
				tRInverse[rl][div] = rI / 7;
			}
		}
	}
}

void LRaceLine::CreateSideRaceline(int rl)
{
	int i;

	double edgeLimit = 1.0 / Width;

	// adjust Lane values to fit between edge & mid
	for (i = 0; i < Divs; i++)
	{
		double leftMargin = 0.0, rightMargin = 0.0, tmp = 0.0;

		if (overrideCollection)
		{
			LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_MARGIN);
			if (labelOverride)
			{
				if (labelOverride->getOverrideValue(i, &tmp) && tmp > 0.0)
					rightMargin = tmp;
			}
			labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_MARGIN);
			if (labelOverride)
			{
				if (labelOverride->getOverrideValue(i, &tmp) && tmp > 0.0)
					leftMargin = tmp;
			}
		}

		txLeft[rl][i] = txLeft[LINE_RL][i] + (txRight[LINE_RL][i] - txLeft[LINE_RL][i]) * ((1.0 + leftMargin) / Width);
		tyLeft[rl][i] = tyLeft[LINE_RL][i] + (tyRight[LINE_RL][i] - tyLeft[LINE_RL][i]) * ((1.0 + leftMargin) / Width);
		txRight[rl][i] = txRight[LINE_RL][i] + (txLeft[LINE_RL][i] - txRight[LINE_RL][i]) * ((1.0 + rightMargin) / Width);
		tyRight[rl][i] = tyRight[LINE_RL][i] + (tyLeft[LINE_RL][i] - tyRight[LINE_RL][i]) * ((1.0 + rightMargin) / Width);

		if (rl == LINE_LEFT)
		{
			//tLane[rl][i] = MIN(tLane[LINE_RL][i], edgeLimit + (tLane[LINE_RL][i] / 1.0) * edgeLineMargin);
			tLane[rl][i] = MAX(MAX(0.0, tRInverse[LINE_RL][i] * 100 / Width), edgeLimit) + (tLane[LINE_RL][i] / 1.0) * edgeLineMargin;

			double minLane = 0.0;
			if (overrideCollection)
			{
				LManualOverride *override = overrideCollection->getOverrideForLabel(PRV_MIN_LANE);
				if (override)
					if (!(override->getOverrideValue(i, &minLane)))
						minLane = 0.0;
			}
			tLane[rl][i] = MAX(minLane, tLane[rl][i]);
		}
		else
		{
			//tLane[rl][i] = MAX(tLane[LINE_RL][i], (1.0 - edgeLimit) - ((1.0 - tLane[LINE_RL][i]) / 1.0) * edgeLineMargin);
			tLane[rl][i] = MIN(MIN(1.0, (tRInverse[LINE_RL][i] < 0.0 ? (1.0 - fabs(tRInverse[LINE_RL][i]) * 100 / Width) : 1.0)), (1.0 - edgeLimit)) - ((1.0 - tLane[LINE_RL][i]) / 1.0) * edgeLineMargin;

			double maxLane = 1.0;
			if (overrideCollection)
			{
				LManualOverride *override = overrideCollection->getOverrideForLabel(PRV_MAX_LANE);
				if (override)
					if (!(override->getOverrideValue(i, &maxLane)))
						maxLane = 1.0;
			}
			tLane[rl][i] = MIN(maxLane, tLane[rl][i]);
		}
	}

	// update xy coordinates
	for (i = 0; i < Divs; i++)
		UpdateTxTy(i, rl);

	for (i = 0; i < Divs; i++)
	{
		int delta = (rl == LINE_RL ? 1 : 2);
		tRInverse[rl][i] = GetRInverse(((i - delta) + Divs) % Divs, tx[rl][i], ty[rl][i], (i + delta) % Divs, tx[rl], ty[rl]);
	}

	/*
	for (i = 0; i < Divs; i++)
	{
		// remove sudden spikes that cause unusual harsh braking
		if (i > 0 && i < Divs - 1 && fabs(tRInverse[rl][i]) > fabs(tRInverse[rl][i - 1]) * 2 && fabs(tRInverse[rl][i]) > fabs(tRInverse[rl][i + 1]) * 2)
			tRInverse[rl][i] = (tRInverse[rl][i - 1] + tRInverse[rl][i + 1]) / 2;
	}
	*/
}

void LRaceLine::UpdateRacelineSpeeds(int raceType)
{
#ifdef LEARNING
	bool shouldUpdate = learn->ReCalcRequired;
#else
	bool shouldUpdate = false;
	if (lastUpdateDist > 0.0)
	{
		if (!hasLastUpdate && car->_distRaced >= lastUpdateDist)
		{
			hasLastUpdate = true;
			shouldUpdate = true;
		}
		else if ((car->_distRaced < MAX(500, stopUpdateDist) || car->_distRaced > resumeUpdateDist) && car->_distRaced < lastUpdateDist)
		{
			shouldUpdate = (fabs(car->_fuel - cardata->fuel) >(raceType == RM_TYPE_QUALIF ? 1.0 : 5.0) ||
				fabs(car->_dammage - cardata->damage) > 100.0f ||
				((CaTT() - cardata->aTT > 9.0 || cardata->aTT - CaTT() > 5.0) && (cardata->aFTT < cardata->lTT || cardata->aFTT > cardata->hTT)));
		}
	}
	else if (car->_speed_x > 5.0)
	{
		shouldUpdate = (fabs(car->_fuel - cardata->fuel) > (raceType == RM_TYPE_QUALIF ? 1.0 : 5.0) ||
			fabs(car->_dammage - cardata->damage) > 100.0f ||
			((CaTT() - cardata->aTT > 9.0 || cardata->aTT - CaTT() > 5.0) && (cardata->aFTT < cardata->lTT || cardata->aFTT > cardata->hTT)));
	}
#endif

	if (shouldUpdate)
    {
#ifdef LEARNING
		learn->ReCalcRequired = false;
#endif
        cardata->updateModel();

        for (int rl = LINE_LEFT; rl <= LINE_RL; rl++)
        {
            // Compute curvature and speed along the path
            int i;
            for (i = Divs; --i >= 0;)
            {
                ComputeRacelineSpeed(i, rl, tSpeed, rl);
            }
            //
            // Anticipate braking
            //
			int iter = MAX(12, MIN(25, (int)((80.0 / cardata->aFTT) * 10)));
            for (int j = iter; --j >= 0;)
            {
                for (i = Divs; --i >= 0;)
                {
                    ComputeRacelineBraking(i, rl, tSpeed, rl);
                }
            }

#if 0
            if (rl == LINE_RL)
            {
                for (int spdrl = LINE_RL_MID; spdrl <= LINE_RL_SLOW; spdrl++)
                {
                    for (i = Divs; --i >= 0;)
                    {
                        ComputeRacelineSpeed(i, rl, tSpeed, spdrl);
                    }

                    for (int j = 32; --j >= 0;)
                    {
                        for (i = Divs; --i >= 0;)
                        {
                            ComputeRacelineBraking(i, rl, tSpeed, spdrl);
                        }
                    }
                }
            }
#endif
        }
    }
}

double LRaceLine::getMinTurnInverse(int raceline)
{
    {
        return minTurnInverse;
    }

    double empty = 15.0f;
    double fuel_gauge = 1.0 - (car->_fuel <= empty ? 0.0 : (MAX(0.0, car->_fuel - 15.0f) / (maxfuel-15.0f)));
    double slowTurnInverse = 1.0 - (fuel_gauge*0.05);
    return minTurnInverse * slowTurnInverse;
}

void LRaceLine::ComputeRacelineBraking(int i, int rl, double **tSpeed, int speedrl)
{
    //double TireAccel = turnSpeed * tFriction[i];
	double TireAccel = getCornerSpeed(i, speedrl) * tFriction[i] * cardata->carMu;// cardata->fullCarMu;
	if (cardata->aFTT < 50.0)
		TireAccel *= MAX(0.65, MIN(1.0, 0.65 + ((cardata->aFTT - 20.0) / (cardata->lTT - (cardata->lTT-20)*0.70)) * 0.45));
    int prev = (i - 1 + Divs) % Divs;

    double dx = tx[rl][i] - tx[rl][prev];
    double dy = ty[rl][i] - ty[rl][prev];
    double dist = Mag(dx, dy);

    double Speed = (tSpeed[speedrl][i] + tSpeed[speedrl][prev]) / 2;

	double LatA = tSpeed[speedrl][i] * tSpeed[speedrl][i] *
		((fabs(tRInverse[rl][prev])*0.2 + fabs(tRInverse[rl][i])*0.8));

#if 1
    double TanA = MAX(0.0, TireAccel * TireAccel - LatA * LatA);
    double brakedelay = getBrakeDist(i, speedrl) + (rl != LINE_RL ? speedAdjust : 0.0);
    TanA = MIN(TanA, brakedelay);
#else
    double TanA = MAX(0.0, TireAccel * TireAccel * cardata->carMu - LatA * LatA);
    double brakedelay = getBrakeDist(i, speedrl) + (rl != LINE_RL ? speedAdjust : 0.0) * tFriction[i];
    TanA = MIN(TanA, brakedelay);
#endif

    double time = dist / Speed;
    double MaxSpeed = tSpeed[speedrl][i] + TanA * time;
    tSpeed[speedrl][prev] = Min(MaxSpeed, tMaxSpeed[prev]);
}

double LRaceLine::getFriction(int i)
{
    double empty = 15.0f;
    double fuel_status = 1.0 - (car->_fuel <= empty ? 0.0 : (MAX(0.0, car->_fuel - 15.0f) / (maxfuel-15.0f)));
    return tFriction[i] * (1.0 - fuel_status*0.1);
}

void LRaceLine::ComputeRacelineSpeed(int i, int rl, double **tSpeed, int speedrl)
{
    vec2f tmp;
    //double TireAccel = turnSpeed * tFriction[i];
    double cornerSpeed = getCornerSpeed(i, speedrl);
    double TireAccel = cornerSpeed * tFriction[i] * (cardata->carMu);
    if (rl < LINE_RL)
        TireAccel += speedAdjust;
    int next = (i + 1) % Divs;
    int prev = (i - 1 + Divs) % Divs;

    double rInverse = GetRInverse(prev, tx[rl][i], ty[rl][i], next, tx[rl], ty[rl]);
    //double rInverse = GetRInverse(prev, tx[rl][i], ty[rl][i], next, rl);
    double rI = fabs(rInverse);
    /* rinverse from clothoid curvature??  */
    if (rl == speedrl)
        tRInverse[rl][i] = rInverse;

    double MaxSpeed;

    double minturninverse = getMinTurnInverse(rl);
    if (fabs(rInverse) > minturninverse * 1.01)
        MaxSpeed = sqrt(TireAccel / (fabs(rInverse) - minturninverse));
    else
        MaxSpeed = 10000;

	if (MaxSpeed > 25.0 && fabs(rInverse) > 0.0005)
	{
		if (cardata->lftOH > 0.0 && rInverse < 0.0)
			MaxSpeed -= (MaxSpeed - 25.0) * (MIN(10.0, cardata->lftOH) / 25);
		else if (cardata->rgtOH > 0.0 && rInverse > 0.0)
			MaxSpeed -= (MaxSpeed - 25.0) * (MIN(10.0, cardata->rgtOH) / 25);
	}


    //if (fabs(rInverse) > 0.0087 )
    //printf("rInv:%f MaxSpd:%f\n", rInverse,  MaxSpeed);

    double bc = getBumpCaution(i, speedrl);

    if (bc > 0.0)
    {
        // look for a sudden downturn in approaching track segments
        double prevzd = 0.0, nextzd = 0.0;
        int range = int(MAX(40.0, MIN(100.0, MaxSpeed)) / 10.0);

        bc += rI * 80;

        for (int n = 1; n < range; n++)
        {
            int x = ((i - n) + Divs) % Divs;
            prevzd += tzd[rl][x] / MAX(1.0, double(n) / 2);
        }

        for (int n = 0; n < range; n++)
        {
            int x = ((i + n) + Divs) % Divs;
            nextzd += tzd[rl][x] / MAX(1.0, double(n + 1) / 2);
        }

        double diff = (prevzd - nextzd) * 2.2;

        if (diff > 0.10)
        {
            diff -= 0.10;
            double safespeed = MAX(15.0, 100.0 - (diff*diff) * 400 * bc);
            if (safespeed < MIN(70.0, MaxSpeed))
            {
                // do a couple of divs before this one to be sure we actually reduce speed
                for (int n = 0; n < 4; n++)
                {
                    int f = ((i - n) + Divs) % Divs;
                    tSpeed[speedrl][f] = safespeed;
                }

                for (int n = 0; n < 4; n++)
                {
                    int f = (i + n) % Divs;
                    tSpeed[speedrl][f] = safespeed;
                }

                MaxSpeed = MIN(MaxSpeed, safespeed);
            }
        }
    }

    MaxSpeed = MIN(getMaxSpeed(i, speedrl), MaxSpeed);

    tSpeed[speedrl][i] = tMaxSpeed[i] = MaxSpeed;

}

////////////////////////////////////////////////////////////////////////////
// New race
////////////////////////////////////////////////////////////////////////////
void LRaceLine::NewRace(tCarElt* newcar, tSituation *s)
{
    car = newcar;
    last_lane = MAX(0.0, MIN(1.0, car->_trkPos.toLeft / Width));

    wheelbase = (car->priv.wheel[FRNT_RGT].relPos.x +
        car->priv.wheel[FRNT_LFT].relPos.x -
        car->priv.wheel[REAR_RGT].relPos.x -
        car->priv.wheel[REAR_LFT].relPos.x) / 2;
    wheeltrack = (car->priv.wheel[FRNT_LFT].relPos.y +
        car->priv.wheel[REAR_LFT].relPos.y -
        car->priv.wheel[FRNT_RGT].relPos.y -
        car->priv.wheel[REAR_RGT].relPos.y) / 2;
}

////////////////////////////////////////////////////////////////////////////
// Car control
////////////////////////////////////////////////////////////////////////////
int LRaceLine::findNextBrakingZone()
{
    double prevspeed = tSpeed[LINE_RL][Next];
    for (int i=0; i<Divs; i++)
    {
        int div = (Next + i) % Divs;
        if (tSpeed[LINE_RL][div] < prevspeed)
            return div;
        prevspeed = tSpeed[LINE_RL][div];
    }
    return -1;
}

int LRaceLine::findNextCorner(tCarElt *theCar, int index, int *apex_div, double *dist)
{
    //tTrackSeg *seg = car->_trkPos.seg;;
    double distance = 0.0;
    int Index = (index >= 0 ? index % Divs : DivIndexForCar(theCar));
    int prefer_side = ((tRInverse[LINE_RL][Index] > 0.003) ? TR_LFT : 
                       ((tRInverse[LINE_RL][Index]) < -0.003 ? TR_RGT : TR_STR));
    //double curlane = car->_trkPos.toLeft / track->width;
    int /*next = (Next+5) % Divs,*/ i = 1, div = -1;
    //double distance = 0.0;
    double CR = tRInverse[LINE_RL][Index];

    if (car->_speed_x < 5.0)
    {
        prefer_side = TR_STR;
    }

    bool override_checked = false;
    if (overrideCollection)
    {
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_PREFERRED_SIDE);
        double override_value = 0.0;
        if (labelOverride)
        {
           if (labelOverride->getOverrideValue(Next, &override_value))
           {
               prefer_side = MIN(TR_STR, MAX(TR_RGT, (int)override_value));
               override_checked = true;
           }
        }
    }

    if (prefer_side == TR_STR && !override_checked)
    {
        int iend = (apex_div != NULL ? MIN(100, (int)car->_speed_x*5) : (int)car->_speed_x * 5);
        for (i=1; i<iend; i++)
        {
            div = (Index + i) % Divs;
            if (tRInverse[LINE_RL][div] > 0.001)
            {
                prefer_side = TR_LFT;
                break;
            }
            else if (tRInverse[LINE_RL][div] < -0.001)
            {
                prefer_side = TR_RGT;
                break;
            }
            double x1 = tx[LINE_RL][div], y1 = ty[LINE_RL][div];
            double x2 = tx[LINE_RL][(div+1)%Divs], y2 = ty[LINE_RL][(div+1)%Divs];
            distance += sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
        }
    }
	if (apex_div)
	    *apex_div = -1;

    bool dist_done = (distance > 0.1);
    if (prefer_side != TR_STR && apex_div != NULL)
    {
        if (div == -1) div = Index;

        int newdiv = div;
        int count = 0;
        do
        {
            int nextdiv = (newdiv + 1) % Divs;
            if (!dist_done)
            {
                double x1 = tx[LINE_RL][newdiv], y1 = ty[LINE_RL][newdiv];
                double x2 = tx[LINE_RL][(newdiv+1)%Divs], y2 = ty[LINE_RL][(newdiv+1)%Divs];
                distance += sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
            }
            if ((tRInverse[LINE_RL][div] > 0 && tRInverse[LINE_RL][nextdiv] < 0.0) ||
                (tRInverse[LINE_RL][div] < 0 && tRInverse[LINE_RL][nextdiv] > 0.0))
            {
                newdiv = -1;
                break;
            }
            if ((tRInverse[LINE_RL][div] < 0.0 && tRInverse[LINE_RL][nextdiv] < 0.0 && tLane[LINE_RL][nextdiv] < tLane[LINE_RL][newdiv] && tLane[LINE_RL][nextdiv] > tLane[LINE_RL][Index]) ||
                (tRInverse[LINE_RL][div] > 0.0 && tRInverse[LINE_RL][nextdiv] > 0.0 && tLane[LINE_RL][nextdiv] > tLane[LINE_RL][newdiv] && tLane[LINE_RL][nextdiv] < tLane[LINE_RL][Index]))
                break;
            newdiv = nextdiv;
            count++;
        } while (newdiv != div && count < 200);

        if (newdiv == -1 && count < 200 && !override_checked)
        {
            // corner ended before finding an apex and a new corner started, change preferred_side
            if (prefer_side == TR_LFT)
                prefer_side = TR_RGT;
            else
                prefer_side = TR_LFT;
        }

		*apex_div = newdiv;
    }

    if (dist)
        *dist = distance;
    return prefer_side;
}

int LRaceLine::DivIndexForCar(tCarElt *theCar, double catchtime)
{
	double dist = 0.0;
	if (catchtime > 0.0)
	{
		dist += catchtime * theCar->_speed_x;
		dist = MAX(0.0, dist - theCar->_dimension_x);
	}
    return DivIndexForCarDistance(theCar, dist);
}

int LRaceLine::DivIndexForCarDistance(tCarElt *theCar, double distance)
{
	double minDist = 999999.0;
	int carDiv = -1;
	double toLeft = MIN(1.0, MAX(0.0, theCar->_trkPos.toLeft / Width));
	for (int div = 0; div < Divs; div++)
	{
		double dx = (tx[LINE_LEFT][div] * (1.0 - toLeft) + tx[LINE_RIGHT][div] * toLeft) - theCar->_pos_X;
		double dy = (ty[LINE_LEFT][div] * (1.0 - toLeft) + ty[LINE_RIGHT][div] * toLeft) - theCar->_pos_Y;
		double dist = sqrt(dx*dx + dy*dy);
		if (dist < minDist)
		{
			minDist = dist;
			carDiv = div;
		}
	}

	if (distance > 0.0)
	{
		int closestDiv = carDiv, div = (carDiv + 1) % Divs, count = 0;
		double closestDist = distance - minDist;

		while (div != carDiv && count < int((distance / DivLength) * 2))
		{
			double dx = (tx[LINE_LEFT][div] * (1.0 - toLeft) + tx[LINE_RIGHT][div] * toLeft) - theCar->_pos_X;
			double dy = (ty[LINE_LEFT][div] * (1.0 - toLeft) + ty[LINE_RIGHT][div] * toLeft) - theCar->_pos_Y;
			double dist = sqrt(dx*dx + dy*dy);
			if (distance - dist < closestDist)
			{
				closestDist = distance - dist;
				closestDiv = div;
			}
			div = (div + 1) % Divs;
			count++;
		}

		carDiv = closestDiv;
	}

	return carDiv;
#if 0
	if (theCar != car && car->_laps > 1)
	{
		double dist = theCar->_distFromStartLine + distance;
		if (dist > m_pTrack->length)
			dist -= m_pTrack->length;

		double pathdist = 0.0;
		int i = Divs / 2, upper = Divs-1, lower = 0;
		while (true)
		{
			if (tDistance[i] < dist)
			{
				upper = i;
				if (upper - lower > 1)
					i = upper - (upper - lower) / 2;
			}
			else if (tDistance[i] > dist)
			{
				lower = i;
				if (upper - lower > 1)
					i = lower + (upper - lower) / 2;
			}
			else
				return i;

			if (upper - lower <= 2)
				break;
		}

		int closest = lower;
		for (i = lower+1; i <= upper; i++)
		{
			if (fabs(tDistance[i] - dist) < fabs(tDistance[closest] - dist))
				closest = i;
		}
		return closest;
	}
	else
	{
		int SegId = theCar->_trkPos.seg->id;
		tTrackSeg *seg = theCar->_trkPos.seg;
		if (distance > seg->length)
		{
			distance -= (seg->length - theCar->_trkPos.toStart);
			seg = seg->next;
			int count = 0;
			while (count < 10)
			{
				if (seg->length > distance)
					break;
				distance -= seg->length;
				seg = seg->next;
				count++;
			}
			SegId = seg->id;
		}
		if (seg->type != TR_STR)
		{
			distance *= seg->radius;
		}
		int Index = (tSegIndex[SegId] + int(distance / tElemLength[SegId])) % Divs;
		return Index;
	}
#endif
}

double LRaceLine::getLookAhead(int rl, double leftMargin, double rightMargin, bool coll)
{
    double factor = 1.0;
#if 0
	double spdmrgn = (m_raceType == RM_TYPE_RACE || driver->alone ? 2.0 : 3.0); // (coll ? 0.5 : 3.0);
	double rlfactor = (m_raceType == RM_TYPE_RACE || driver->alone ? (1.0 + fabs(tRInverse[rl][Next])) * (1.0 + fabs(tRInverse[rl][Next])) : 1.0);
	double speedfactor = (tSpeed[rl][Next] - car->_speed_x < spdmrgn ? 1.0 : MIN(1.0, (car->_speed_x + spdmrgn) / MIN(tSpeed[rl][Next], 80.0) / rlfactor));
#else
	double spdmrgn = (m_raceType == RM_TYPE_RACE ? (driver->alone ? 2.0 : 3.0) : 2.0); // (coll ? 0.5 : 3.0);
	int rlmult = (m_raceType == RM_TYPE_RACE ? (driver->alone ? 1 : 60) : 1);
	double rlfactor = 1.0 + fabs(tRInverse[rl][Next])*rlmult * (1.0 + fabs(tRInverse[rl][Next])*rlmult);
	double speedfactor = (tSpeed[rl][Next] - car->_speed_x < spdmrgn || fabs(tRInverse[rl][Next]) < 0.0005 ? 1.0 : MIN(1.0, (car->_speed_x + spdmrgn) / MIN(tSpeed[rl][Next], 80.0) / rlfactor));
#endif
	double trfactor = MIN(1.0, MAX(0.7, 0.7 + (cardata->aFTT / cardata->lTT)*0.3));
	double fuelCarMu = cardata->carMu; // (rl == LINE_RL ? cardata->fuelCarMu : cardata->offlineFuelCarMu);
	double gripfactor = MAX(0.3, MIN(1.0, (cardata->carMu / fuelCarMu) * (cardata->carMu / fuelCarMu)));
	if (tSpeed[rl][Next] > tSpeed[rl][(Next - 5 + Divs) % Divs]) gripfactor = 1.0;
#if 1
	if ((tRInverse[rl][Next] > 0.0 && driver->getAngle() < 0.0) ||
		(tRInverse[rl][Next] < 0.0 && driver->getAngle() > 0.0))
		speedfactor = 1.0;
#endif

	if ((rl == LINE_LEFT && tRInverse[rl][Next] < 0.0) || (rl == LINE_RIGHT && tRInverse[rl][Next] > 0.0))
		factor += fabs(tRInverse[rl][Next]) * 15;
	else if ((rl == LINE_LEFT && tRInverse[rl][Next] > 0.0 || (rl == LINE_RIGHT && tRInverse[rl][Next] < 0.0)))
		factor -= MIN(0.5, fabs(tRInverse[rl][Next]) * 15);

	if (driver->currentCollision && car->_accel_x < 0.0)
	{
		if ((tRInverse[LINE_RL][This] > 0.0 && driver->avgLateralMovt > 0.0) || (tRInverse[LINE_RL][This] < 0.0 && driver->avgLateralMovt < 0.0))
			factor *= 1.0 + fabs(car->_accel_x) / 20;
		else
			factor *= 0.95;
	}

#ifdef LEARNING
	return learn->GetLookahead(This) * factor * speedfactor * trfactor * gripfactor;
#endif

    if (overrideCollection)
    {
        double lookahead = lookAhead;
        if (leftMargin < 0.001 && rightMargin < 1.0)
        {
            LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_LOOKAHEAD_LEFT);
			if (labelOverride)
			{
				double leftLookahead = 0.0;
				if (labelOverride->getOverrideValue(This, &leftLookahead))
				{
					//double rlmix = MAX(0.0, (rightMargin - edgeLineMargin) / (1.0 - edgeLineMargin));
					//return (lookAhead * rlmix + leftLookahead * (1.0 - rlmix))  * factor * speedfactor * trfactor * gripfactor;
					return leftLookahead;
				}
			}
			if (tRInverse[LINE_LEFT][This] < -0.001 && lookAheadOut > 0.0)
				return lookAheadOut * factor * speedfactor * trfactor * gripfactor;
        }
        if (leftMargin > 0.001 && rightMargin == 1.0)
        {
            LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_LOOKAHEAD_RIGHT);
			if (labelOverride)
			{
				double rightLookahead = 1.0;
				if (labelOverride->getOverrideValue(This, &rightLookahead))
				{
					//double rlmix = MAX(0.0, (1.0 - leftMargin - edgeLineMargin) / (1.0 - edgeLineMargin));
					//return (lookAhead * rlmix + rightLookahead * (1.0 - rlmix)) * factor * speedfactor * trfactor * gripfactor;
					return rightLookahead;
				}
			}
			if (tRInverse[LINE_RIGHT][This] > 0.001 && lookAheadOut > 0.0)
				return lookAheadOut * factor * speedfactor * trfactor * gripfactor;
        }
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_LOOKAHEAD);
        if (labelOverride)
           if (labelOverride->getOverrideValue(This, &lookahead))
               return lookahead * factor * speedfactor * trfactor * gripfactor;
    }
    double empty = 10.0f;
    double fuel_status = (car->_fuel <= empty ? 0.0 : (MAX(0.0, car->_fuel - empty) / (maxfuel-empty)));
    double lookahead = (lookAheadEmpty - (lookAheadEmpty-lookAhead)*fuel_status) * factor * speedfactor * trfactor * gripfactor;
	if (cardata->aTT < cardata->lTT)
		lookahead *= (lookAheadColdFactor - ((lookAheadColdFactor - 1.0) * (cardata->aTT / cardata->lTT)));
	return lookahead;
}

double LRaceLine::OfflineLane(int div, double leftMargin, double rightMargin)
{
	if (leftMargin == 0.0 && rightMargin == 1.0)
		return tLane[LINE_RL][div];
	if (leftMargin == 0.0 && rightMargin == edgeLineMargin)
		return tLane[LINE_LEFT][div];
	if (leftMargin == 1.0 - edgeLineMargin && rightMargin == 1.0)
		return tLane[LINE_RIGHT][div];

	double edgeLimit = 1.0 / Width;

	if (leftMargin == 0.0)
	{
		double minLane = 0.0;
		if (overrideCollection)
		{
			LManualOverride *override = overrideCollection->getOverrideForLabel(PRV_MIN_LANE);
			if (override)
				if (!(override->getOverrideValue(Next, &minLane)))
					minLane = 0.0;
		}
		return MIN(tLane[LINE_RL][div], MAX(minLane, tLane[LINE_RL][div] * rightMargin));
	}
	else if (rightMargin == 1.0)
	{
		double maxLane = 1.0;
		if (overrideCollection)
		{
			LManualOverride *override = overrideCollection->getOverrideForLabel(PRV_MAX_LANE);
			if (override)
				if (!(override->getOverrideValue(Next, &maxLane)))
					maxLane = 1.0;
		}
		return MAX(tLane[LINE_RL][div], MIN(maxLane, 1.0 - (1.0 - tLane[LINE_RL][div]) * (1.0 - leftMargin)));
	}
		  
	return leftMargin + tLane[LINE_RL][div] * (rightMargin - leftMargin);
}

double LRaceLine::AdjustTxForMargin(int div, double lane)
{
    return lane * txRight[LINE_RL][div] + (1 - lane) * txLeft[LINE_RL][div];
}

double LRaceLine::AdjustTyForMargin(int div, double lane)
{
    return lane * tyRight[LINE_RL][div] + (1 - lane) * tyLeft[LINE_RL][div];
}

double LRaceLine::RInverseForMargin(int div, double leftMargin, double rightMargin)
{
	double prevLane = OfflineLane(((div - 1) + Divs) % Divs, leftMargin, rightMargin);
	double thisLane = OfflineLane(div, leftMargin, rightMargin);
	double nextLane = OfflineLane((div + 1) % Divs, leftMargin, rightMargin);
	double prevX = AdjustTxForMargin(((div - 1) + Divs) % Divs, prevLane);
	double prevY = AdjustTyForMargin(((div - 1) + Divs) % Divs, prevLane);
	double thisX = AdjustTxForMargin(div, thisLane);
	double thisY = AdjustTyForMargin(div, thisLane);
	double nextX = AdjustTxForMargin((div + 1) % Divs, nextLane);
	double nextY = AdjustTyForMargin((div + 1) % Divs, nextLane);
	return GetRInverse(prevX, prevY, thisX, thisY, nextX, nextY);
}

int LRaceLine::DivIndex(RaceLineDriveData *data, double leftMargin, double rightMargin, double *X, double *Y)
{
    int Index = DivIndexForCar(car);
    Index = (Index + Divs - 5) % Divs;

	double indexLane = OfflineLane(Index, leftMargin, rightMargin);
	double indexX = AdjustTxForMargin(Index, indexLane);
	double indexY = AdjustTyForMargin(Index, indexLane);

    int steer_verbose = 0;
    if (Time < 0.0)
        Time = data->s->deltaTime * steerTimeFactor;
    *X = car->_pos_X + car->_speed_X * Time / 2;
    *Y = car->_pos_Y + car->_speed_Y * Time / 2;
    data->lookahead = 0.0f;

    int tmp_Index = Index;
	int raceline = (leftMargin == 0.0 && rightMargin == 1.0 ? LINE_RL : (leftMargin > 0.0 && rightMargin < 1.0 ? LINE_MID : (leftMargin > 0.0 ? LINE_RIGHT : LINE_LEFT)));
	double minLookAhead = getLookAhead(raceline, leftMargin, rightMargin, data->coll);
    do
    {
        Next = (Index + 1) % Divs;
		double rInverse = RInverseForMargin(Next, leftMargin, rightMargin);
		double thisLane = OfflineLane(Next, leftMargin, rightMargin);
		double thisX = AdjustTxForMargin(Next, thisLane);
        double thisY = AdjustTyForMargin(Next, thisLane);
        double dx = thisX - car->_pos_X;
        double dy = thisY - car->_pos_Y;
        data->lookahead = (float)(sqrt(dx*dx + dy*dy) * MAX(0.5, 1.0 - fabs((rInverse*100)*(rInverse*100))*cornersteer));
        if (data->lookahead > minLookAhead && 
            ((thisX - indexX) * (*X - thisX) +
             (thisY - indexY) * (*Y - thisY) < 0.1))
            break;
        Index = Next;
    } while (tmp_Index != Index);

	int next = Index;
	double maxFabsRI = -1.0;
	do
	{
		next = (next + 1) % Divs;
		double rInverse = fabs(RInverseForMargin(next, leftMargin, rightMargin));
		if (rInverse > 0.01)
		{
			if (rInverse > maxFabsRI)
				maxFabsRI = rInverse;
			else if (rInverse < maxFabsRI)
			{
                   data->next_apex = next;
                   break;
			}
		}
	} while (next != Index);

    Index = tmp_Index;
    return Index;
}

int LRaceLine::DivIndex(RaceLineDriveData *data, int raceline, double *X, double *Y)
{
    int Index = DivIndexForCar(car);

    Index = (Index + Divs - 5) % Divs;

    int steer_verbose = 0;
    if (Time < 0.0)
        Time = data->s->deltaTime * steerTimeFactor; //7;     // 0.50 + turnAccel/80;
    *X = car->_pos_X + car->_speed_X * Time / 2;
    *Y = car->_pos_Y + car->_speed_Y * Time / 2;
    data->lookahead = 0.0f;

    int tmp_Index = Index;
	double lftMargin = (raceline == LINE_RIGHT ? 1.0 - edgeLineMargin : 0.0);
	double rgtMargin = (raceline == LINE_LEFT ? edgeLineMargin : 1.0);
	double minLookAhead = getLookAhead(raceline, lftMargin, rgtMargin, data->coll);
    do
    {
        Next = (Index + 1) % Divs;
        double dx = tx[raceline][Next] - car->_pos_X;
        double dy = ty[raceline][Next] - car->_pos_Y;
        data->lookahead = (float)(sqrt(dx*dx + dy*dy) * MAX(0.5, 1.0 - fabs((tRInverse[raceline][Next]*100)*(tRInverse[raceline][Next]*100))*cornersteer));
        if (data->lookahead > minLookAhead && // + (1.0 - MIN(0.1, MAX(0.0, 1.0 - car->_fuel / (maxfuel/2))/11)) &&
            ((tx[raceline][Next] - tx[raceline][Index]) * (*X - tx[raceline][Next]) +
             (ty[raceline][Next] - ty[raceline][Index]) * (*Y - ty[raceline][Next]) < 0.1))
            break;
        Index = Next;
    } while (tmp_Index != Index);

    //data->lookahead = AdjustLookahead(raceline, data->lookahead);

    if (raceline >= LINE_RL)
    {
       int next = Index;
       double maxFabsRI = -1.0;
       do
       {
           next = (next + 1) % Divs;
		   double rInverse = fabs(tRInverse[raceline][next]);
           if (rInverse > 0.01)
           {
               if (rInverse > maxFabsRI)
                   maxFabsRI = rInverse;
               else if (rInverse < maxFabsRI)
               {
                   data->next_apex = next;
                   break;
               }
           }
       } while (next != Index);
    }

    Index = tmp_Index;
    return Index;
}

float LRaceLine::AdjustLookahead(int raceline, float lookahead)
{
    if ((tRInverse[raceline][Next] > 0.0 && car->_trkPos.toMiddle < 0.0) ||
        (tRInverse[raceline][Next] < 0.0 && car->_trkPos.toMiddle > 0.0))
        lookahead *= (float)MIN(4.0f, 1.5f + fabs(car->_trkPos.toMiddle*0.3));
    else
        lookahead *= (float)MAX(0.7f, 1.5f - fabs(car->_trkPos.toMiddle*0.2));

#if 1
    if ((tRInverse[raceline][Next] < 0.0 && car->_trkPos.toMiddle > 0.0) ||
        (tRInverse[raceline][Next] > 0.0 && car->_trkPos.toMiddle < 0.0))
    {
        lookahead *= (float)MAX(1.0f, MIN(3.6f, 1.0f + (MIN(2.6f, fabs(car->_trkPos.toMiddle) / (Width / 2)) / 2) * (1.0f + fabs(tRInverse[raceline][Next]) * 65.0f + car->_speed_x / 120.0f)));
    }
    else if ((tRInverse[raceline][Next] < 0.0 && car->_trkPos.toRight < 5.0) ||
        (tRInverse[raceline][Next] > 0.0 && car->_trkPos.toLeft < 5.0))
    {
        lookahead *= (float)MAX(0.8f, MIN(1.0f, 1.0f - fabs(tRInverse[raceline][Next])*200.0 * ((5.0f - MIN(car->_trkPos.toRight, car->_trkPos.toLeft)) / 5.0f)));
    }
#endif

    return lookahead;
}

double LRaceLine::CaTT()
{
    double sum = 0.0;
    for (int i=0; i<4; i++)
        sum += CT(i);
    return sum / 4;
}

bool LRaceLine::IsSlowerThanSpeedToDiv(LLineMode *lineMode, double speed, int div, double *slowSpeed)
{
	int d = div;
	if (d >= Divs)
		d = d % Divs;
	if (d < 0) return false;

	for (int i = (Next - 5 + Divs) % Divs; i != d; i++)
	{
		if (i >= Divs)
			i = 0;
		if (i == d) return false;
		double lineSpeed = CalculateSpeedAtDiv(lineMode, i);
		if (lineSpeed < speed && lineSpeed > 10.0)
		{
			*slowSpeed = lineSpeed;
			return true;
		}
	}

	return false;
}

double LRaceLine::CalculateSpeedAtDiv(LLineMode *lineMode, int div)
{
	double tLeftMargin = lineMode->GetLeftTargetMargin();
	double tRightMargin = lineMode->GetRightTargetMargin();

	if (tLeftMargin == 0.0 && tRightMargin == 1.0)
		return tSpeed[LINE_RL][div];
	if (tLeftMargin == 0.0 && tRightMargin == edgeLineMargin)
		return tSpeed[LINE_LEFT][div];
	if (tLeftMargin == 1.0 - edgeLineMargin && tRightMargin == 1.0)
		return tSpeed[LINE_RIGHT][div];

	return CalculateOfflineSpeed((div - 3 + Divs) % Divs, div, tLeftMargin, tRightMargin);
}

double LRaceLine::CalculateOfflineSpeed(int index, int next, double leftMargin, double rightMargin)
{
//#define OLD_METHOD
#ifdef OLD_METHOD
	leftMargin = MAX(leftMargin, edgeLineMargin);
	rightMargin = MIN(rightMargin, 1.0 - edgeLineMargin);

    double X = car->_pos_X + car->_speed_X * Time / 2;
    double Y = car->_pos_Y + car->_speed_Y * Time / 2;

	double indexLane = OfflineLane(index, leftMargin, rightMargin);
	double nextLane = OfflineLane(next, leftMargin, rightMargin);
	double indexX = AdjustTxForMargin(index, indexLane);
	double indexY = AdjustTyForMargin(index, indexLane);
	double nextX = AdjustTxForMargin(next, nextLane);
	double nextY = AdjustTyForMargin(next, nextLane);

    double c0 = (nextX - indexX) * (nextX - X) +
        (nextY - indexY) * (nextY - Y);
    double c1 = (nextX - indexX) * (X - indexX) +
        (nextY - indexY) * (Y - indexY);
    double sum = c0 + c1;

    c0 /= sum;
    c1 /= sum;
#endif

	double next_speed, index_speed;

	if (leftMargin == 0.0 && rightMargin == 1.0)
	{
#ifdef OLD_METHOD
		next_speed = tSpeed[LINE_RL][next];
		index_speed = tSpeed[LINE_RL][index];
#else
		return (tSpeed[LINE_RL][index]);
#endif
	}
	else if (leftMargin == 0.0)
	{
#ifdef OLD_METHOD
		double outsteerfactor = 1.0;
		if (tRInverse[LINE_RL][next] < -0.001)
			outsteerfactor += MAX(-0.2, tRInverse[LINE_RL][next] * 50);
#endif

		if (rightMargin == edgeLineMargin)
		{
#ifdef OLD_METHOD
			next_speed = tSpeed[LINE_LEFT][next] * outsteerfactor;
			index_speed = tSpeed[LINE_LEFT][index] * outsteerfactor;
#else
			return tSpeed[LINE_LEFT][index];
#endif
		}
		else
		{
			double rightM = 1.0 - rightMargin;
			double factor = rightM / (1.0 - edgeLineMargin);
#ifdef OLD_METHOD
			next_speed = tSpeed[LINE_LEFT][next] * outsteerfactor * factor + tSpeed[LINE_RL][next] * (1.0 - factor);
			index_speed = tSpeed[LINE_LEFT][index] * outsteerfactor * factor + tSpeed[LINE_RL][index] * (1.0 - factor);
#else
			return tSpeed[LINE_LEFT][index] * factor + tSpeed[LINE_RL][index] * (1.0 - factor);
#endif
		}
	}
	else if (rightMargin == 1.0)
	{
#ifdef OLD_METHOD
		double outsteerfactor = 1.0;
		if (tRInverse[LINE_RL][next] > 0.001)
			outsteerfactor -= MIN(0.2, tRInverse[LINE_RL][next] * 50);
#endif

		if (leftMargin == 1.0 - edgeLineMargin)
		{
#ifdef OLD_METHOD
			next_speed = tSpeed[LINE_RIGHT][next] * outsteerfactor;
			index_speed = tSpeed[LINE_RIGHT][index] * outsteerfactor;
#else
			return tSpeed[LINE_RIGHT][index];
#endif
		}
		else
		{
			double factor = leftMargin / (1.0 - edgeLineMargin);
#ifdef OLD_METHOD
			next_speed = tSpeed[LINE_RIGHT][next] * outsteerfactor * factor + tSpeed[LINE_RL][next] * (1.0 - factor);
			index_speed = tSpeed[LINE_RIGHT][index] * outsteerfactor * factor + tSpeed[LINE_RL][index] * (1.0 - factor);
#else
			return tSpeed[LINE_RIGHT][index] * factor + tSpeed[LINE_RL][index] * (1.0 - factor);
#endif
		}
	}
	else
	{
		double factor = (leftMargin + rightMargin) / 2;
		double factor2 = (rightMargin - leftMargin) / 1.0;
#ifdef OLD_METHOD
		next_speed = tSpeed[LINE_RL][next] * factor2 + (tSpeed[LINE_LEFT][next] * (1.0 - factor) + tSpeed[LINE_RIGHT][next] * factor) * (1.0 - factor2);
		index_speed = tSpeed[LINE_RL][index] * factor2 + (tSpeed[LINE_LEFT][index] * (1.0 - factor) + tSpeed[LINE_RIGHT][index] * factor) * (1.0 - factor2);
#else
		return tSpeed[LINE_RL][index] * factor2 + (tSpeed[LINE_LEFT][index] * (1.0 - factor) + tSpeed[LINE_RIGHT][index] * factor) * (1.0 - factor2);
#endif
	}

#ifdef OLD_METHOD
	return (1 - c0) * next_speed + c0 * index_speed;
#endif
}

double LRaceLine::CalculateSpeed(RaceLineDriveData *data, double X, double Y, int index, int next, double leftMargin, double rightMargin, double tLeftMargin, double tRightMargin)
{
	double indexLane = OfflineLane(index, leftMargin, rightMargin);
	double nextLane = OfflineLane(next, leftMargin, rightMargin);

	double indexX = AdjustTxForMargin(index, indexLane);
	double indexY = AdjustTyForMargin(index, indexLane);
	double nextX = AdjustTxForMargin(next, nextLane);
	double nextY = AdjustTyForMargin(next, nextLane);

	if (data)
	{
		data->target->x = (float) nextX;
		data->target->y = (float) nextY;
	}

    double c0 = (nextX - indexX) * (nextX - X) +
        (nextY - indexY) * (nextY - Y);
    double c1 = (nextX - indexX) * (X - indexX) +
        (nextY - indexY) * (Y - indexY);
    double sum = c0 + c1;

    c0 /= sum;
    c1 /= sum;

	double next_speed, index_speed;

	if (tLeftMargin == 0.0 && tRightMargin == 1.0)
	{
		next_speed = tSpeed[LINE_RL][next];
		index_speed = tSpeed[LINE_RL][index];
	}
	else if (tLeftMargin == 0.0)
	{
		double outsteerfactor = 1.0;
		if (tRInverse[LINE_RL][Next] < -0.001)
			outsteerfactor += MAX(-0.2, tRInverse[LINE_RL][Next] * 10);

		if (tRightMargin == edgeLineMargin)
		{
			next_speed = tSpeed[LINE_LEFT][next] * outsteerfactor;
			index_speed = tSpeed[LINE_LEFT][index] * outsteerfactor;
		}
		else
		{
			double rightM = 1.0 - tRightMargin;
			double factor = rightM / (1.0 - edgeLineMargin);
			next_speed = tSpeed[LINE_LEFT][next] * outsteerfactor * factor + tSpeed[LINE_RL][next] * (1.0 - factor);
			index_speed = tSpeed[LINE_LEFT][index] * outsteerfactor * factor + tSpeed[LINE_RL][index] * (1.0 - factor);
		}
	}
	else if (tRightMargin == 1.0)
	{
		double outsteerfactor = 1.0;
		if (tRInverse[LINE_RL][Next] > 0.001)
			outsteerfactor -= MIN(0.2, tRInverse[LINE_RL][Next] * 10);

		if (tLeftMargin == 1.0 - edgeLineMargin)
		{
			next_speed = tSpeed[LINE_RIGHT][next] * outsteerfactor;
			index_speed = tSpeed[LINE_RIGHT][index] * outsteerfactor;
		}
		else
		{
			double factor = tLeftMargin / (1.0 - edgeLineMargin);
			next_speed = tSpeed[LINE_RIGHT][next] * outsteerfactor * factor + tSpeed[LINE_RL][next] * (1.0 - factor);
			index_speed = tSpeed[LINE_RIGHT][index] * outsteerfactor * factor + tSpeed[LINE_RL][index] * (1.0 - factor);
		}
	}
	else
	{
		double factor = (tLeftMargin + tRightMargin) / 2;
		double factor2 = (tRightMargin - tLeftMargin) / 1.0;
		next_speed = tSpeed[LINE_RL][next] * factor2 + (tSpeed[LINE_LEFT][next] * (1.0 - factor) + tSpeed[LINE_RIGHT][next] * factor) * (1.0 - factor2);
		index_speed = tSpeed[LINE_RL][index] * factor2 + (tSpeed[LINE_LEFT][index] * (1.0 - factor) + tSpeed[LINE_RIGHT][index] * factor) * (1.0 - factor2);
	}

	double targetSpeed = (1 - c0) * next_speed + c0 * index_speed;

	if (leftMargin == 0.0 && rightMargin == 1.0)
	{
		next_speed = tSpeed[LINE_RL][next];
		index_speed = tSpeed[LINE_RL][index];
	}
	else if (leftMargin == 0.0)
	{
		double outsteerfactor = 1.0;
		if (tRInverse[LINE_RL][Next] < -0.001)
			outsteerfactor += MAX(-0.2, tRInverse[LINE_RL][Next] * 50);

		if (rightMargin == edgeLineMargin)
		{
			next_speed = tSpeed[LINE_LEFT][next] * outsteerfactor;
			index_speed = tSpeed[LINE_LEFT][index] * outsteerfactor;
		}
		else
		{
			double rightM = 1.0 - rightMargin;
			double factor = rightM / (1.0 - edgeLineMargin);
			next_speed = tSpeed[LINE_LEFT][next] * outsteerfactor * factor + tSpeed[LINE_RL][next] * (1.0 - factor);
			index_speed = tSpeed[LINE_LEFT][index] * outsteerfactor * factor + tSpeed[LINE_RL][index] * (1.0 - factor);
		}
	}
	else if (rightMargin == 1.0)
	{
		double outsteerfactor = 1.0;
		if (tRInverse[LINE_RL][Next] > 0.001)
			outsteerfactor -= MIN(0.2, tRInverse[LINE_RL][Next] * 50);

		if (leftMargin == 1.0 - edgeLineMargin)
		{
			next_speed = tSpeed[LINE_RIGHT][next] * outsteerfactor;
			index_speed = tSpeed[LINE_RIGHT][index] * outsteerfactor;
		}
		else
		{
			double factor = leftMargin / (1.0 - edgeLineMargin);
			next_speed = tSpeed[LINE_RIGHT][next] * outsteerfactor * factor + tSpeed[LINE_RL][next] * (1.0 - factor);
			index_speed = tSpeed[LINE_RIGHT][index] * outsteerfactor * factor + tSpeed[LINE_RL][index] * (1.0 - factor);
		}
	}
	else
	{
		double factor = (leftMargin + rightMargin) / 2;
		double factor2 = (rightMargin - leftMargin) / 1.0;
		next_speed = tSpeed[LINE_RL][next] * factor2 + (tSpeed[LINE_LEFT][next] * (1.0 - factor) + tSpeed[LINE_RIGHT][next] * factor) * (1.0 - factor2);
		index_speed = tSpeed[LINE_RL][index] * factor2 + (tSpeed[LINE_LEFT][index] * (1.0 - factor) + tSpeed[LINE_RIGHT][index] * factor) * (1.0 - factor2);
	}

	double currentSpeed = (1 - c0) * next_speed + c0 * index_speed;

	TargetSpeed = MIN(currentSpeed, targetSpeed);
	if (data)
	    data->speed = (float)MAX(5.0f, TargetSpeed);

	return c0;
}

double LRaceLine::CalculateSpeed(RaceLineDriveData *data, double X, double Y, int index, int next, double *tSpd, double *tX, double *tY)
{
    data->target->x = (float)tX[next];
    data->target->y = (float)tY[next];

    double c0 = (tX[next] - tX[index]) * (tX[next] - X) +
        (tY[next] - tY[index]) * (tY[next] - Y);
    double c1 = (tX[next] - tX[index]) * (X - tX[index]) +
        (tY[next] - tY[index]) * (Y - tY[index]);
    double sum = c0 + c1;

    c0 /= sum;
    c1 /= sum;
    TargetSpeed = (1 - c0) * tSpd[next] + c0 * tSpd[index];

    if (TargetSpeed < 10.0f)
    {
        //fprintf(stderr,"RL=%d Index=%d Next=%d c0=%.3f sum=%.3f tSpeed[Next]=%.3f tSpeed[Index]=%.3f TargetSpeed=%.3f\n",raceline,Index,Next,c0,sum,tSpeed[raceline][Next],tSpeed[raceline][Index],TargetSpeed);
        //fflush(stderr);
    }

    /* WARNING: problems on Corkscrew pit in with TargetSpeed being negative? */
    //*speed = TargetSpeed;
    data->speed = (float)MAX(10.0f, TargetSpeed);

    return c0;
}

double LRaceLine::CalculateOffset(double *tX, double *tY, int next, int rl)
{
    double toLft = sqrt((tX[next]-txLeft[rl][Next]) + (tY[next] - tyLeft[rl][Next]));
    double toRgt = sqrt((txRight[rl][Next]-tX[next]) + (tyRight[rl][Next] - tY[next]));
    return (toLft / (toLft+toRgt));
}

double LRaceLine::CalculateOffset(int div, double leftMargin, double rightMargin)
{
	double lane = OfflineLane(div, leftMargin, rightMargin);

	if (leftMargin == 0.0 && rightMargin == 1.0)
		return CalculateOffset(tx[LINE_RL], ty[LINE_RL], div, LINE_RL);
	if (leftMargin == 0.0 && rightMargin == edgeLineMargin)
		return CalculateOffset(tx[LINE_LEFT], ty[LINE_LEFT], div, LINE_LEFT);
	if (leftMargin == 1.0 - edgeLineMargin && rightMargin == 1.0)
		return CalculateOffset(tx[LINE_RIGHT], ty[LINE_RIGHT], div, LINE_RIGHT);

	double X = AdjustTxForMargin(div, lane);
	double Y = AdjustTyForMargin(div, lane);
    double toLft = sqrt((X-txLeft[LINE_RL][div]) + (Y - tyLeft[LINE_RL][div]));
    double toRgt = sqrt((txRight[LINE_RL][div]-X) + (tyRight[LINE_RL][div] - Y));
    return (toLft / (toLft+toRgt));
}

double LRaceLine::CalculateMixedCurvature(double c0, int Index, double transition_percentage)
{
    double rInverse_next = tRInverse[LINE_LEFT][Next] + (tRInverse[LINE_RIGHT][Next] - tRInverse[LINE_LEFT][Next]) * (1.0 - transition_percentage);
    double rInverse_index = tRInverse[LINE_LEFT][Index] + (tRInverse[LINE_RIGHT][Index] - tRInverse[LINE_LEFT][Index]) * (1.0 - transition_percentage);
    double TargetCurvature = (1 - c0) * rInverse_next + c0 * rInverse_index;
    if (fabs(TargetCurvature) > 0.01)
    {
        double r = 1 / TargetCurvature;
        if (r > 0)
            r -= wheeltrack / 2;
        else
            r += wheeltrack / 2;
        TargetCurvature = 1 / r;
    }
    return TargetCurvature;
}

double LRaceLine::CalculateCurvature(double c0, double tRINext, double tRIIndex)
{
    //
    // Find target curvature (for the inside wheel)
    //
    double TargetCurvature = (1 - c0) * tRINext + c0 * tRIIndex;
    if (fabs(TargetCurvature) > 0.001)
    {
        double r = 1 / TargetCurvature;
        if (r > 0)
            r -= wheeltrack / 2;
        else
            r += wheeltrack / 2;
        TargetCurvature = 1 / r;
    }
    return TargetCurvature;
}

void LRaceLine::slowestSpeedBetweenDivs(int startdiv, int enddiv, double *rlspeed, double *leftspeed, double *rightspeed)
{
    if (enddiv < startdiv)
        enddiv += Divs;

    *rlspeed = *leftspeed = *rightspeed = 10000;

    for (int i=startdiv; i<=enddiv; i++)
    {
        int div = i % Divs;
        *rlspeed = MIN(*rlspeed, tSpeed[LINE_RL][div]);
        *leftspeed = MIN(*leftspeed, tSpeed[LINE_LEFT][div]);
        *rightspeed = MIN(*rightspeed, tSpeed[LINE_RIGHT][div]);
    }
}

void LRaceLine::updateRLSpeedMode()
{
    rl_speed_mode = LINE_RL;
    /*
    int ideal_speed_mode = LINE_RL_SLOW;
    if (car->_fuel / maxfuel < midtankPercent || (!hasSlow && !hasMid))
    {
        ideal_speed_mode = LINE_RL;
    }
    else if (car->_fuel / maxfuel < fulltankPercent || !hasSlow)
    {
        ideal_speed_mode = LINE_RL_MID;
    }

    if (rl_speed_mode == -1 || ideal_speed_mode < rl_speed_mode || (ideal_speed_mode > rl_speed_mode && fabs(tRInverse[LINE_RL][Next]) < 0.001))
    {
        rl_speed_mode = ideal_speed_mode;
    }
    */
}

float LRaceLine::smoothSteering(float steercmd, float laststeer)
{
    /* try to limit sudden changes in steering to avoid loss of control through oversteer. */
    double speedfactor = (((60.0 - (MAX(40.0, MIN(70.0, cardata->getSpeedInTrackDirection() + MAX(0.0, car->_accel_x * 5))) - 25)) / 300) * 1.2) / 0.785 * 0.75;

    if (fabs(steercmd) < fabs(laststeer) && fabs(steercmd) <= fabs(laststeer - steercmd))
        speedfactor *= 2;

    steercmd = (float)MAX(laststeer - speedfactor, MIN(laststeer + speedfactor, steercmd));
    return steercmd;
}

double LRaceLine::Point2Lane(int rl, double x, double y)
{
    double lx = txLeft[rl][Next], ly = tyLeft[rl][Next];
    double rx = txRight[rl][Next], ry = tyRight[rl][Next];
    double dx = lx - rx;
    double dy = ly - ry;
    double disttotal = sqrt((dx * dx) + (dy * dy));
    dx = lx - x;
    dy = ly - y;
    double distpoint = sqrt((dx * dx) + (dy * dy));
    return distpoint / disttotal;
}

void LRaceLine::GetRaceLineData(RaceLineDriveData *data, bool transitioning)
{
    UpdateRacelineSpeeds(data->s->_raceType);
    updateRLSpeedMode();

    double target_error = 0.0;

	bool isTransitioning = data->linemode->IsTransitioning();
	bool isOnRaceLine = data->linemode->IsOnRaceLine();

	if (isTransitioning || !isOnRaceLine)
	{
		double ti = data->linemode->GetTransitionIncrement(Next);
		SteerTheCarOffline(data);
		if (isTransitioning)
		{
			double outsteerFactor = 1.0;
			bool outsteer = false;
			double leftCurrentMargin = data->linemode->GetLeftCurrentMargin(), rightCurrentMargin = data->linemode->GetRightCurrentMargin();
			double leftPredictMargin = data->linemode->GetLeftPredictMargin(), rightPredictMargin = data->linemode->GetRightPredictMargin();
			double carMargin = car->_trkPos.toLeft / Width, leftTargetMargin = data->linemode->GetLeftTargetMargin(), rightTargetMargin = data->linemode->GetRightTargetMargin();

			// Stop the car changing direction suddenly
			if ((leftTargetMargin < leftCurrentMargin && leftPredictMargin > leftCurrentMargin) ||
				(leftTargetMargin > leftCurrentMargin && leftPredictMargin < leftCurrentMargin))
			{
				leftCurrentMargin = leftPredictMargin;
				data->linemode->SetLeftCurrentMargin(leftCurrentMargin);
			}

			if ((rightTargetMargin < rightCurrentMargin && rightPredictMargin > rightCurrentMargin) ||
				(rightTargetMargin > rightCurrentMargin && rightPredictMargin < rightCurrentMargin))
			{
				rightCurrentMargin = rightPredictMargin;
				data->linemode->SetRightCurrentMargin(rightCurrentMargin);
			}

			if ((MAX(tRInverse[LINE_RIGHT][Next], tRInverse[LINE_RL][Next]) > 0.002 && leftTargetMargin > leftCurrentMargin) ||
				(MIN(tRInverse[LINE_LEFT][Next], tRInverse[LINE_RL][Next]) < -0.002 && rightTargetMargin < rightCurrentMargin))
				outsteer = true;
			else
			{
				for (int i = 1; i < (int)(car->_speed_x * 2); i++)
				{
					int div = (Next + i) % Divs;
					if ((MAX(tRInverse[LINE_RIGHT][div], tRInverse[LINE_RL][div]) > 0.002 && leftTargetMargin > leftCurrentMargin) ||
						(MIN(tRInverse[LINE_LEFT][div], tRInverse[LINE_RL][div]) < -0.002 && rightTargetMargin < rightCurrentMargin))
					{
						outsteer = true;
						break;
					}
				}
			}


			if (outsteer)
			{
                outsteerFactor = (car->_speed_x < data->speed-3.0f ? outsideSteeringDampenerAccel : outsideSteeringDampener);
				//outsteerFactor += fabs(tRInverse[LINE_RL][Next]) * 5;
                if (overrideCollection)
                {
                    double tmp = outsteerFactor;
                    LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_OUTSIDE_DAMPENER);
                    if (labelOverride)
                       if (!labelOverride->getOverrideValue(Next, &outsteerFactor))
                           outsteerFactor = tmp;
                }
				if (car->_speed_x > data->speed)
				{
					// less outsteer if we're too fast for raceline
					outsteerFactor = MAX(0.0, outsteerFactor - (car->_speed_x - data->speed) / 30);
				}
				if (car->_speed_x < data->speed - 1.0f && car->_speed_x < 40.0 && data->s->currentTime < 20.0)
				{
					// more outsteer if very slow
					outsteerFactor = MIN(20.0, outsteerFactor + (data->speed - car->_speed_x - 1.0f) / 20);
				}
#if 0
				else if (car->_speed_x > 40.0)
				{
					// less outsteer as we're fast in general.
					outsteerFactor = MAX(0.1, outsteerFactor - ((car->_speed_x - 40.0) / 30));
				}
#endif
				data->speed -= (float)MIN(4.0, (outsteerSpeedReducer * (1.0 - outsteerFactor)));
			}
			//if (driver->avoidSqueezed > 0.0)
			//	data->speed = (float)MIN(data->speed, MAX(car->_speed_x, 2.0 - fabs(driver->avgLateralMovt) * (1.0 / data->s->deltaTime)));

			if (data->speed > car->_speed_x)
			{
				double wheelslip = MAX(fabs(car->_wheelSlipSide(FRNT_LFT)), fabs(car->_wheelSlipSide(FRNT_RGT)));
				if (wheelslip > 10.0)
					data->speed = (float) MIN(data->speed, car->_speed_x + (1.0 - MAX(0.0, MIN(1.0, (wheelslip - 10) / 5.0))));
			}

			double tiFactor = MAX(0.1, 1.0 - (fabs(tRInverse[LINE_RL][Next]) * (data->speed > car->_speed_x ? 50 : 40)));
			double futureCarMargin = ((car->_trkPos.toLeft + driver->avgLateralMovt * 1.0 / data->s->deltaTime) / Width);
			double offlineSpeed = data->speed;// CalculateOfflineSpeed(Next, Next, leftTargetMargin, rightTargetMargin);
			bool targetRaceline = (leftTargetMargin == 0.0 && rightTargetMargin == 1.0);

			if (driver->avoidCritical)
				tiFactor *= 10.0;
			else if (data->s->currentTime - driver->pitTimer < 5.0)
				tiFactor *= 1.0 + (5.0 - (data->s->currentTime - driver->pitTimer)) * (cardata->aTT < cardata->lTT ? 0.3 + (1.1 - (cardata->aTT / cardata->lTT)) : 0.3);
			else if ((targetRaceline && car->_accel_x > 0 && ((leftCurrentMargin > 0.0001 && tRInverse[LINE_RL][Next] < -0.001) ||   // recover to raceline and on the inside
				      (rightCurrentMargin < 1.0 && tRInverse[LINE_RL][Next] > 0.001))) ||                                             // on exit from a corner...
				     ((rightCurrentMargin < 1.0 && rightTargetMargin > rightCurrentMargin && tRInverse[LINE_RL][Next] < -0.001) ||   // avoiding on the outside & there's room
				      (leftCurrentMargin > 0.001 && leftTargetMargin < leftCurrentMargin && tRInverse[LINE_RL][Next] > 0.001)) ||    // on the inside to use more space...
			         (!targetRaceline && offlineSpeed > car->_speed_x - 1.0 && fabs(data->racesteer) < 0.1 && fabs(data->angle) < 0.1 &&  // car's close to target raceline, so lets
				      ((leftTargetMargin > 0.0001 && carMargin > leftTargetMargin && futureCarMargin > leftTargetMargin) ||                // adjust faster...
				      (rightTargetMargin < 1.000 && carMargin < rightTargetMargin && futureCarMargin < rightTargetMargin))))
			{
				tiFactor *= 3.0;
			}
			else
			{
				if ((leftTargetMargin > data->linemode->GetLeftCurrentMargin() && tRInverse[LINE_RIGHT][Next] < -0.001) ||
					(rightTargetMargin < data->linemode->GetRightCurrentMargin() && tRInverse[LINE_RIGHT][Next] > 0.001))
					tiFactor = MIN(tiFactor, MAX(0.3, 1.0 - MAX(fabs(tRInverse[LINE_LEFT][Next]), fabs(tRInverse[LINE_RIGHT][Next]) * 20))); // avoid steering too hard towards apex
				if (car->_speed_x >= data->left_speed - 1.0 && car->_speed_x > 25.0f)
					tiFactor = MIN(tiFactor, MAX(0.3, 1.0 - (car->_speed_x - 25.0) / 35.0)); // less increment at high speeds
				if (fabs(car->_accel_x) > 4.0)
					tiFactor = MIN(tiFactor, MAX(0.5, 1.0 - MIN(8.0, fabs(car->_accel_x) - 4.0) / 8.0)); // less increment at high acceleration/braking
				if (!outsteer && fabs(car->ctrl.steer) > 0.4)
					tiFactor = MIN(tiFactor, MAX(0.2, 1.0 - MIN(0.6, fabs(car->ctrl.steer) - 0.4) / 0.6)); // less increment when steering sharply
			}

			if (cardata->aTT < cardata->lTT)
				tiFactor *= MAX(0.3, 1.0 - (cardata->lTT - cardata->aTT) / cardata->lTT);

			data->linemode->ApplyTransition(Next, outsteerFactor * tiFactor, tRInverse[LINE_RL][Next], data->racesteer);
		}
#ifndef LEARNING
		if (true || data->s->_raceType == RM_TYPE_PRACTICE)
		{
			static char *lineName[] = { "MID", "LFT", "RGT", "RL" };
			int rl = data->linemode->GetTargetRaceline();
			fprintf(stderr, "%s TR %d:%d (%.1f) %s %s str=%.2f %.3f/%.3f %.3f/%.3f spd %.1f/%.1f/%.1f ang=%.3f vang=%.3f skidang=%.3f accx=%.3f\n", car->_name, This, Next, car->_distFromStartLine, lineName[rl], (isTransitioning ? "TRANS" : "OFFLINE"), data->racesteer, data->linemode->GetLeftCurrentMargin(), data->linemode->GetLeftTargetMargin(), data->linemode->GetRightCurrentMargin(), data->linemode->GetRightTargetMargin(), car->_speed_x, data->speed, CalculateOfflineSpeed((Next - 5 + Divs) % Divs, Next, data->linemode->GetLeftCurrentMargin(), data->linemode->GetRightCurrentMargin()), data->angle, data->speedangle, atan2(car->_speed_Y, car->_speed_X) - car->_yaw, car->_accel_x);
			fflush(stderr);
		}
#endif
	}
	else
	{
		double leftTarget = data->linemode->GetLeftTargetMargin();
		double rightTarget = data->linemode->GetRightTargetMargin();
		int rl = LINE_RL;
		if (leftTarget > 0.0)
			rl = LINE_RIGHT;
		else if (rightTarget < 1.0)
			rl = LINE_LEFT;
		SteerTheCar(data, rl);
#ifndef LEARNING
		if (true || data->s->_raceType == RM_TYPE_PRACTICE)
		{
			fprintf(stderr, "%s LN %d:%d (%.1f %.1f) %s spd %.1f/%.1f/%.1f err=%.3f vne=%.3f skidang=%.3f accx=%.3f\n", car->_name, This, Next, car->_distFromStartLine, car->_distRaced, (rl == LINE_RL ? "RL" : (rl == LINE_LEFT ? "LFT" : "RGT")), car->_speed_x, data->speed, CalculateOfflineSpeed((Next - 5 + Divs) % Divs, Next, leftTarget, rightTarget), data->error, data->vnerror, atan2(car->_speed_Y, car->_speed_X) - car->_yaw, car->_accel_x);
			fflush(stderr);
		}
#endif
	}

	if (overrideCollection && ((tRInverse[LINE_RL][This] > 0.0 && data->error < -0.6) || (tRInverse[LINE_RL][This] < 0.0 && data->error > 0.6)))
	{
		LManualOverride *labelOverride;
		if (NULL != (labelOverride = overrideCollection->getOverrideForLabel(PRV_SPEEDERROR)))
		{
			double se;
			if (labelOverride->getOverrideValue(This, &se))
				data->speed -= (fabs(data->error) - 0.6) * se;
		}
	}

    last_last_steer = last_steer;
    last_steer = data->laststeer = data->racesteer;
}


void LRaceLine::SteerTheCarOffline(RaceLineDriveData *data)
{
	double leftMargin = data->linemode->GetLeftCurrentMargin();
	double rightMargin = data->linemode->GetRightCurrentMargin();
	double targetLeftMargin = data->linemode->GetLeftTargetMargin();
	double targetRightMargin = data->linemode->GetRightTargetMargin();
	double transitionInc = data->linemode->GetTransitionIncrement(Next);

	if (leftMargin == targetLeftMargin && rightMargin == targetRightMargin)
	{
		/* if (targetLeftMargin == 0.0 && targetRightMargin == 1.0)
		{
			fprintf(stderr, "RL STEER\n");
			SteerTheCar(data, LINE_RL);
			return;
		}
		else */ if (targetLeftMargin == 0.0 && targetRightMargin == edgeLineMargin)
		{
			SteerTheCar(data, LINE_LEFT);
			return;
		}
		else if (targetLeftMargin == 1.0 - edgeLineMargin && targetRightMargin == 1.0)
		{
			SteerTheCar(data, LINE_RIGHT);
			return;
		}
	}

    // 
    // Find index in data arrays
    //
    /*====================================================*/
    double X, Y;
    int Index = This = DivIndex(data, leftMargin, rightMargin, &X, &Y);
	if (car->_laps <= 1 && data->s->raceInfo.type == RM_TYPE_RACE)
		tDistance[Index] = car->_distFromStartLine;
    int diff = Next - Index;
    if (diff < 0)
        diff = (Next + Divs) - Index;
    NextNextNext = (Next + diff) % Divs;
    /*====================================================*/

    double TargetCurvature, c0;
    {
        //c0 = CalculateSpeed(data, X, Y, Index, raceline);
		c0 = CalculateSpeed(data, X, Y, Index, Next, leftMargin, rightMargin, targetLeftMargin, targetRightMargin);
        data->raceoffset = (float)CalculateOffset(Next, leftMargin, rightMargin);
		double rInvNext = RInverseForMargin(Next, leftMargin, rightMargin);
		double rInvIndex = RInverseForMargin(Index, leftMargin, rightMargin);
		TargetCurvature = CalculateCurvature(c0, rInvNext, rInvIndex);
		//fprintf(stderr, "RInv %.3f %.3f Curv %.3f\n", rInvIndex, rInvNext, TargetCurvature);
    }

    //
    // Steering control
    //
    double Error = 0;
    double VnError = 0;
    double carspeed = Mag(car->_speed_X, car->_speed_Y);

    //
    // Ideal value
    //
    double steer = atan(wheelbase * TargetCurvature) / car->_steerLock;

    //
    // Servo system to stay on the pre-computed path
    //
    Prev = (Index + Divs - 1) % Divs;
    int NextNext = (Next + 1) % Divs;
	double indexLane = OfflineLane(Index, leftMargin, rightMargin);
	double nextLane = OfflineLane(Next, leftMargin, rightMargin);
	double prevLane = OfflineLane(Prev, leftMargin, rightMargin);
	double nextnextLane = OfflineLane(NextNext, leftMargin, rightMargin);
	//fprintf(stderr, "lane: %.2f %.2f %.2f %.2f s1 %.3f ", prevLane,indexLane,nextLane,nextnextLane,steer);

	double txPrev = AdjustTxForMargin(Prev, prevLane);
	double tyPrev = AdjustTyForMargin(Prev, prevLane);
	double txIndex = AdjustTxForMargin(Index, indexLane);
	double tyIndex = AdjustTyForMargin(Index, indexLane);
	double txNext = AdjustTxForMargin(Next, nextLane);
	double tyNext = AdjustTyForMargin(Next, nextLane);
	double txNextNext = AdjustTxForMargin(NextNext, nextnextLane);
	double tyNextNext = AdjustTyForMargin(NextNext, nextnextLane);

    {
        double dx = txNext - txIndex;
        double dy = tyNext - tyIndex;
        Error = (dx * (Y - tyIndex) - dy * (X - txIndex)) / Mag(dx, dy);

        double Prevdx = txNext - txPrev;
        double Prevdy = tyNext - tyPrev;
        double Nextdx = txNextNext - txIndex;
        double Nextdy = tyNextNext - tyIndex;
        dx = c0 * Prevdx + (1 - c0) * Nextdx;
        dy = c0 * Prevdy + (1 - c0) * Nextdy;
        double n = Mag(dx, dy);
        dx /= n;
        dy /= n;
        double sError = (dx * car->_speed_Y - dy * car->_speed_X) / (carspeed == 0.0 ? carspeed+0.01 : carspeed);
        double cError = (dx * car->_speed_X + dy * car->_speed_Y) / (carspeed == 0.0 ? carspeed+0.01 : carspeed);

        VnError = asin(sError);

        VnError = MAX((PI / -2.0), MIN(VnError, (PI / 2.0)));

        if (cError < 0)
        {
            VnError = PI - VnError;
            //VnError = (2.0 *PI) - VnError;
            //VnError = -1.0 * VnError;
        }

        NORM_PI_PI(VnError);

        data->error = VnError;
    }

    // Correcting Steer:
    steer -= ((atan(Error * (300.0 / (carspeed + 300.0)) / 15.0) + VnError) * errorCorrectionFactor) / car->_steerLock;
	//fprintf(stderr, "s2 %.3f Err %.3f/%.3f ", steer, Error, VnError);

    //
    // Steer into the skid
    //
    double vx = car->_speed_X;
    double vy = car->_speed_Y;
    double dirx = cos(car->_yaw);
    double diry = sin(car->_yaw);
    double Skid = (dirx * vy - vx * diry) / (carspeed == 0.0 ? carspeed+0.1 : carspeed);
    if (Skid > 0.9)
        Skid = 0.9;
    if (Skid < -0.9)
        Skid = -0.9;
    data->skid = ((float)asin(Skid) / car->_steerLock);
	double rlmix = MAX(0.0, MIN(1.0, MIN((rightMargin - edgeLineMargin) / (1.0 - edgeLineMargin), (1.0 - leftMargin - edgeLineMargin) / (1.0 - edgeLineMargin))));
	steer += data->skid * (steerSkidFactor * rlmix + steerSkidOfflineFactor * (1.0 - rlmix));

    double yr = carspeed * TargetCurvature;
    double yawdiff = car->_yaw_rate - yr;
    steer -= (0.08 * (100 / (carspeed + 100)) * yawdiff) / car->_steerLock;
	//fprintf(stderr, "s4 %.3f ", steer);
    data->racesteer = (float)steer;

	// don't accelerate if car is skidding as it'll cause a spinout
	if (data->speed > car->_speed_x)
		data->speed = (float)MAX(car->_speed_x, data->speed - fabs(data->skid * 10));
}

void LRaceLine::SteerTheCar(RaceLineDriveData *data, int target_raceline)
{
    int rl = MIN(LINE_RL, target_raceline);
    //int target_raceline = data->linemode->getTargetLine();

    // 
    // Find index in data arrays
    //
    /*====================================================*/
    double X, Y;
    int Index = This = DivIndex(data, rl, &X, &Y);
	if (car->_laps <= 1 && data->s->raceInfo.type == RM_TYPE_RACE)
		tDistance[Index] = car->_distFromStartLine;
    int diff = Next - Index;
    if (diff < 0)
        diff = (Next + Divs) - Index;
    NextNextNext = (Next + diff) % Divs;
    /*====================================================*/


#if 0
    double dx = X4 - car->_pos_X;
    double dy = Y4 - car->_pos_Y;
    *lookahead = sqrt(dx*dx + dy*dy);
#endif

    double c0, TargetCurvature, transition_percentage = 0.0;
    bool transition_required = false;

    {
        //c0 = CalculateSpeed(data, X, Y, Index, raceline);
        c0 = CalculateSpeed(data, X, Y, Index, Next, tSpeed[rl], tx[rl], ty[rl]);
        data->raceoffset = (float)CalculateOffset(tx[rl], ty[rl], Next, rl);
        TargetCurvature = CalculateCurvature(c0, tRInverse[rl][Next], tRInverse[rl][Index]);
		//fprintf(stderr, "RInv %.3f %.3f Curv %.3f\n", tRInverse[rl][Index], tRInverse[rl][Next], TargetCurvature);
    }

    //
    // Steering control
    //
    double Error = 0;
    double VnError = 0;
    double carspeed = Mag(car->_speed_X, car->_speed_Y);

    //
    // Ideal value
    //
    double steer = atan(wheelbase * TargetCurvature) / car->_steerLock;

    if (0 && steer_verbose)
        GfOut("Steer A:%f", steer);

    //
    // Servo system to stay on the pre-computed path
    //
    Prev = (Index + Divs - 1) % Divs;
    int NextNext = (Next + 1) % Divs;

    {
        double dx = tx[rl][Next] - tx[rl][Index];
        double dy = ty[rl][Next] - ty[rl][Index];
        Error = (dx * (Y - ty[rl][Index]) - dy * (X - tx[rl][Index])) / Mag(dx, dy);

        double Prevdx = tx[rl][Next] - tx[rl][Prev];
        double Prevdy = ty[rl][Next] - ty[rl][Prev];
        double Nextdx = tx[rl][NextNext] - tx[rl][Index];
        double Nextdy = ty[rl][NextNext] - ty[rl][Index];
        dx = c0 * Prevdx + (1 - c0) * Nextdx;
        dy = c0 * Prevdy + (1 - c0) * Nextdy;
        double n = Mag(dx, dy);
        dx /= n;
        dy /= n;
        double sError = (dx * car->_speed_Y - dy * car->_speed_X) / (carspeed == 0.0 ? carspeed+0.01 : carspeed);
        double cError = (dx * car->_speed_X + dy * car->_speed_Y) / (carspeed == 0.0 ? carspeed+0.01 : carspeed);

        VnError = asin(sError);

        VnError = MAX((PI / -2.0), MIN(VnError, (PI / 2.0)));

        if (cError < 0)
        {
            VnError = PI - VnError;
            //VnError = (2.0 *PI) - VnError;
            //VnError = -1.0 * VnError;
        }

        NORM_PI_PI(VnError);

        data->error = Error;
        data->vnerror = VnError;
#ifdef LEARNING
		learn->Update(car, data->s->currentTime, VnError, This);
#endif
    }

    // Correcting Steer:
    steer -= ((atan(Error * (300.0 / (carspeed + 300.0)) / 15.0) + VnError) * errorCorrectionFactor) / car->_steerLock;

    if (steer_verbose)
        GfOut("steer: B:%f\n", steer);

    //
    // Steer into the skid
    //
    double vx = car->_speed_X;
    double vy = car->_speed_Y;
    double dirx = cos(car->_yaw);
    double diry = sin(car->_yaw);
    double Skid = (dirx * vy - vx * diry) / (carspeed == 0.0 ? carspeed+0.1 : carspeed);
    if (Skid > 0.9)
        Skid = 0.9;
    if (Skid < -0.9)
        Skid = -0.9;
    data->skid = ((float)asin(Skid) / car->_steerLock);
    steer += data->skid * steerSkidFactor;//0.9;
    //steer += (asin(Skid) / car->_steerLock) * steerSkidFactor;//0.9;

    if (0 && steer_verbose)
        GfOut(" C:%f", steer);

    double yr = carspeed * TargetCurvature;
    double yawdiff = car->_yaw_rate - yr;
    steer -= (0.08 * (100 / (carspeed + 100)) * yawdiff) / car->_steerLock;
    data->racesteer = (float)steer;
}

/**********************************************************/
double LRaceLine::getSlowestSpeedForDistance(double distance, int raceline, double toLeft, int *slowdiv)
{
	if ((raceline == LINE_LEFT && toLeft < edgeLineMargin*Width+1.0) ||
		(raceline == LINE_RIGHT && toLeft > Width - edgeLineMargin*Width - 1.0))
	{
		*slowdiv = Next;
		return 0.0;
	}

    double dist = car->_trkPos.toStart + distance;
    int distancediv = (distance > 0.0 ? DivIndexForCarDistance(car, dist) : Next);
    int div = Next;
    double minspeed = 100000;
    double dist_covered = 0.0;

    do {
		double factor = MAX(0.0, MIN(1.0, toLeft / Width));
		double speed;
		if (raceline == LINE_LEFT)
			speed = tSpeed[LINE_RL][div] * factor + (tSpeed[raceline][div] * (1.0 - factor));
		else
			speed = tSpeed[LINE_RL][div] * (1.0 - factor) + (tSpeed[raceline][div] * factor);

        if (speed < minspeed)
        {
			minspeed = speed;
            if (slowdiv)
                *slowdiv = div;
        }
        int ldiv = div;
        div = (div + 1) % Divs;
        dist_covered += Mag(tx[LINE_RL][div]-tx[LINE_RL][ldiv], ty[LINE_RL][div]-ty[LINE_RL][ldiv]);
        if (dist_covered >= distance) 
            break;
    } while (div != Next);

    return minspeed;
}

int LRaceLine::isOnLine(int line)
{
    double lane2left = tLane[line][Next] * Width;

    double pos2middle = fabs(car->_trkPos.toLeft - lane2left);

    if (steer_verbose)
        fprintf(stderr, "Lane ToLeft:%.3f, Car ToLeft:%.3f, Car ToMiddle:%.3f\n", lane2left, car->_trkPos.toLeft, car->_trkPos.toMiddle);

    if (pos2middle < MAX(0.1, 1.0 - (car->_speed_x * (car->_speed_x / 10)) / 600))
    {
        if (steer_verbose)
            fprintf(stderr, "%s is OnLine: PosToMiddle: %.3f \n", car->_name, pos2middle);
        return 1;
    }

    return 0;
}

/**********************************************************/
void LRaceLine::GetPoint(float offset, float lookahead, vec2f *rt)
{
#if 0
    double lane = (Width / 2 - offset) / Width;
#if 0
    int ndiv = (Next + 1 + int((lookahead)/DivLength)) % Divs;
    rt->x = (float) (lane * txRight[ndiv] + (1 - lane) * txLeft[lane][ndiv]);
    rt->y = (float) (lane * tyRight[ndiv] + (1 - lane) * tyLeft[lane][ndiv]);
#endif
    double length = 0.0;
    //double la = (double) lookahead * MAX(0.8, car->_speed_x/TargetSpeed); //0.8;
    double la = (double)lookahead * MIN(1.0, MAX(0.8, car->_speed_x / TargetSpeed)); //0.8;
    vec2f last;
    last.x = (float)(lane * txRight[lane][Next] + (1 - lane) * txLeft[lane][Next]);
    last.y = (float)(lane * tyRight[lane][Next] + (1 - lane) * tyLeft[lane][Next]);
    //last.x = car->_pos_X;
    //last.y = car->_pos_Y;
    //int ndiv = (Next + 1 + int((lookahead)/DivLength)) % Divs;
    int ndiv = Next, count = 0;
    while (length < la && count < (int)(la / DivLength))
    {
        rt->x = (float)(lane * txRight[lane][ndiv] + (1 - lane) * txLeft[lane][ndiv]);
        rt->y = (float)(lane * tyRight[lane][ndiv] + (1 - lane) * tyLeft[lane][ndiv]);
        double dx = rt->x - last.x;
        double dy = rt->y - last.y;
        double thislength = sqrt(dx*dx + dy*dy);
        length += thislength;
        ndiv = (ndiv + 1) % Divs;
        count++;
        last.x = rt->x;
        last.y = rt->y;
    }
#endif
}

/**********************************************************/
double LRaceLine::correctLimit(int line)
{
    // this returns true if we're approaching a corner & are significantly
    // inside the ideal racing line. The idea is to prevent a sudden outwards
    // movement at a time when we should be looking to turn in.

    double nlane2left = tLane[line][Next] * Width;
    if ((tRInverse[line][Next] > 0.001 && car->_trkPos.toLeft < nlane2left - 2.0) ||
        (tRInverse[line][Next] < -0.001 && car->_trkPos.toLeft > nlane2left + 2.0))
        //return MAX(0.2, MIN(1.0, 1.0 - fabs(tRInverse[line][Next]) * 80.0));
        return MAX(0.2, MIN(1.0, 1.0 - fabs(tRInverse[line][Next]) * 100.0));

    int nnext = (Next + (int)(car->_speed_x / 3)) % Divs;
    double nnlane2left = tLane[line][nnext] * Width;
    if ((tRInverse[line][nnext] > 0.001 && car->_trkPos.toLeft < nnlane2left - 2.0) ||
        (tRInverse[line][nnext] < -0.001 && car->_trkPos.toLeft > nnlane2left + 2.0))
        //return MAX(0.2, MIN(1.0, 1.0 - fabs(tRInverse[line][nnext]) * 40.0));
        return MAX(0.3, MIN(1.0, 1.0 - fabs(tRInverse[line][nnext]) * 40.0));

    /* OK, we're not inside the racing line! */
#if 0
    // Check and see if we're significantly outside it and turning a corner, 
    // in which case we don't want to correct too much either.
    if ((tRInverse[line][Next] > 0.001 && car->_trkPos.toLeft > nlane2left + 2.0) ||
        (tRInverse[line][Next] < -0.001 && car->_trkPos.toLeft < nlane2left - 2.0))
        return MAX(0.2, MIN(1.0, 1.0 - fabs(tRInverse[line][Next]) * (car->_speed_x-40.0)*4));

    if ((tRInverse[line][nnext] > 0.001 && car->_trkPos.toLeft > nlane2left + 2.0) ||
        (tRInverse[line][nnext] < -0.001 && car->_trkPos.toLeft < nlane2left - 2.0))
        return MAX(0.2, MIN(1.0, 1.0 - fabs(tRInverse[line][nnext]) * (car->_speed_x-40.0)*2));
#endif
    // Check and see if we're outside it and turning into a corner,
    //  in which case we want to correct more to try and get closer to the apex.
    if ((tRInverse[line][Next] > 0.001 && tLane[Next] <= tLane[(Next - 5 + Divs) % Divs] && car->_trkPos.toLeft > nlane2left + 2.0) ||
        (tRInverse[line][Next] < -0.001 && tLane[Next] >= tLane[(Next - 5 + Divs) % Divs] && car->_trkPos.toLeft < nlane2left - 2.0))
        return MAX(1.0, MIN(1.5, 1.0 + fabs(tRInverse[line][Next])));

    return 1.0;
}

/**********************************************************/
double LRaceLine::getAvoidSpeed(float distance1, float distance2)
{
    int i;
    //double speed1 = 1000.0, speed2 = 1000.0, speed3 = 1000.0;
    double speed1 = 1000.0, speed2 = 1000.0;

    int count = 0;
    for (i = Next; count < (int)(distance1 / DivLength); i++)
    {
        count++;
        i = (i % Divs);
        speed1 = MIN(speed1, (tSpeed[LINE_LEFT][i] + tSpeed[LINE_RIGHT][i])/2);
    }

    count = 0;
    distance2 = (MIN(distance2, distance1 * 3) - distance1) / DivLength;
    for (; count < (int)distance2; i++)
    {
        count++;
        i = (i % Divs);
        speed2 = MIN(speed2, (tSpeed[LINE_LEFT][i] + tSpeed[LINE_RIGHT][i])/2 + (double)count * 0.25);
    }

    return MIN(speed1, speed2);
}

/********************************************************************/
/*                                                                  */
/*            STORE K1999 DATA                            */
/*        Raceline K1999 (aka Andrew's Locus)                 */
/*                                                                  */
/********************************************************************/
/* Set "savedata" to 1 in the xml file
               Not used in race, just for look at */

// saved in your Local directory .torcs/drivers/[module_name]/tracks/[track_name].data

//void LRaceLine::storeData( void **carParmHandle )
void LRaceLine::StoreData(tTrack* t)
{
    int rlLine = 0;


    rlLine = (int) GfParmGetNum(car->_carHandle, SECT_PRIVATE, (char*)"savedata", (char*)NULL, 0);

    if (rlLine)
    {
        WriteLine(t);
        WriteTrack(t);
    }
    else
        return;

}

/**********************************************************/
void LRaceLine::WriteLine(tTrack* track)
{
#if 0
    FILE *tfile = NULL;
    int i;
    int rl = LINE_RL;

    char filename[256];
    sprintf(filename, "%sdrivers/%s/tracks/%s_line.data", GetLocalDir(), myModuleName, track->internalname);

    //tfile = fopen( "/home/gamer/TRB/torcs_data/hymie_2015/tracks/wheel-2.learn", "w");

    tfile = fopen(filename, "w");

    if (tfile == NULL)
        return;

    fprintf(tfile, "#==========================================================================================\n");
    fprintf(tfile, "# K1999 RaceLine data\n");
    fprintf(tfile, "# i tDistance[i] tLane[rl][i] tx[rl][i] ty[rl][i] tRInverse[rl][i] tSpeed[rl][i]\n");
    fprintf(tfile, "# Divs:%d\n", Divs);
    for (i = 0; i < Divs; i++)
    {
        fprintf(tfile, "%d %f %f %f %f %f %f\n",
            i,
            tDistance[i],
            tLane[rl][i],
            tx[rl][i], ty[rl][i],
            tRInverse[rl][i],
            tSpeed[rl][i]
            );
    }
    fprintf(tfile, "#End\n");
    fprintf(tfile, "#===============\n");
    fprintf(tfile, "\n");

    fflush(tfile);
    fclose(tfile);
#endif
}

/**********************************************************/

void LRaceLine::WriteTrack(tTrack* track)
{
    /*
    FILE *tfile = NULL;
    int i;

    char filename[256];
    sprintf(filename, "%sdrivers/%s/tracks/%s_track.data", GetLocalDir(), myModuleName, track->internalname);
    tfile = fopen(filename, "w");

    if (tfile == NULL)
        return;

    fprintf(tfile, "#=============================================================================\n");
    fprintf(tfile, "# K1999 Track data\n");
    fprintf(tfile, "# i tDistance[i] txLeft[i] tyLeft[i] txRight[i] tyRight[i] tFriction[i]\n");
    fprintf(tfile, "# Divs:%d\n", Divs);

    for (i = 0; i < Divs; i++)
    {
        fprintf(tfile, "%d %f %f %f %f %f %f\n",
            i,
            tDistance[i],
            txLeft[i], tyLeft[i],
            txRight[i], tyRight[i],
            tFriction[i]
            );
    }

    fprintf(tfile, "#Done\n");
    fprintf(tfile, "#=================================\n");
    fprintf(tfile, "\n");

    fflush(tfile);
    fclose(tfile);

    */
}

/*==========================================================*/

void LRaceLine::removeNewLineCharacters(char *text)
{
    char *p = text + (strlen(text)-1);
    while (p >= text && (*p == 13 || *p == 10 || *p == ' ' || *p == '\t'))
    {
        *p = 0;
        p--;
    }
}

double LRaceLine::readDouble(FILE *fp)
{
    if (!fp) return -1000.0;
    char buffer[257];
    if (!fgets(buffer, 256, fp)) return -1000.0;

    removeNewLineCharacters(buffer);
    return atof(buffer);
}

int LRaceLine::readInt(FILE *fp)
{
    if (!fp) return -1000;
    char buffer[257];
    if (!fgets(buffer, 256, fp)) return -1000;

    removeNewLineCharacters(buffer);
    return atoi(buffer);
}

bool LRaceLine::ApproachingBrakeZone(tCarElt *pCar)
{
	int div = Next;
	if (pCar)
		div = DivIndexForCar(pCar);

	if (fabs(tRInverse[LINE_RL][div]) > 0.01) return false;
	if (overrideCollection)
	{
		LManualOverride *labelOverride;
		if (NULL != (labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEZONE)))
		{
			double bz;
			if (labelOverride->getOverrideValue(div, &bz) && (int)bz == 1)
				return true;
		}
	}

	int i;
	bool brakingDetected = false;
	for (i = 0; i < (int)(car->_speed_x * 3); i++)
	{
		if (tSpeed[LINE_RL][(div + i) % Divs] < car->_speed_x)
		{
			brakingDetected = true;
			break;
		}
	}

	if (!brakingDetected) return false;
	double prevSpeed = tSpeed[LINE_RL][(div + i) % Divs];

	while (i < car->_speed_x * 6)
	{
		if (tSpeed[LINE_RL][(div + i) % Divs] < car->_speed_x * 0.7)
			return true;
		if (tSpeed[LINE_RL][(div + i) % Divs] > prevSpeed)
			return false;
		prevSpeed = tSpeed[LINE_RL][(div + i) % Divs];
		i++;
	}

	return false;
}
