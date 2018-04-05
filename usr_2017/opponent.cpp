/***************************************************************************

    file                 : opponent.cpp
    created              : Thu Apr 22 01:20:19 CET 2003
    copyright            : (C) 2003-2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: opponent.cpp,v 1.9 2006/03/06 22:43:50 berniw Exp $

    ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

//#define OPP_DEBUG
//#define BRAKE_DEBUG
//#include "quadratic.h"
#include "opponent.h"
#include "Vec2d.h"
#include "vardef.h"

// class variables and constants.
tTrack* Opponent::track;
const float Opponent::FRONTCOLLDIST = 200.0f;            // [m] distance to check other cars in front me.
const float Opponent::BACKCOLLDIST = 15.0f;            // [m] distance to check other cars behind me.

const float Opponent::EXACT_DIST = 12.0f;            // [m] if the estimated distance is smaller, compute it more accurate
const float Opponent::LAP_BACK_TIME_PENALTY = -30.0f;        // [s]
const float Opponent::OVERLAP_WAIT_TIME = 3.0f;            // [s] overlaptimer must reach this time before we let the opponent pass.
const float Opponent::SPEED_PASS_MARGIN = 2.0f;            // [m/s] avoid overlapping opponents to stuck behind me.
const int Opponent::MAX_DAMAGE_DIFF = 1000;

Opponent::Opponent()
{
    t_impact = t_toside = 0.0;
    toLeft = 0.0f;
    prevToLeft = -200.0f;
    brake_overtake_filter = 1.0f;
	currentLft = currentRgt = impactLft = impactRgt = -10.0;
    teammate = hasSlowerSpeed = false;
    team = -1;
    FRONTCOLL_MARGIN = 0.0;            // [m] front safety margin.
    SIDECOLL_MARGIN = 3.0;            // [m] side safety margin.
    brake_multiplier = 0.3;
    brake_warn_multiplier = 0.3;
    relativeangle = 0.0f;
    distance = distance2 = distance3 = trueSpeedDiff = 0.0;
    prev_speed_X = prev_speed_Y = d_prev_speed_X = d_prev_speed_Y = 0.0;
    average_AX = average_AY = collspeed = 0.0;
    for (int i=0; i<4; i++)
        speedAngle[i] = 0;
    deltamult = deltaTime = speedDelta = prevSpeed = simTime = 0;
	firstTime = true;
	left_speed_y = right_speed_y = 0.0;
	last_left_toMid = last_right_toMid = left_toMid = right_toMid = 0.0;
	withinBrakeDist = false;
}



void Opponent::update(tSituation *s, Driver *driver)
{
    //tCarElt *mycar = driver->getCarPtr();
    mycar = driver->getCarPtr();
	if (firstTime)
	{
		teammate = !strncmp(car->_teamname, mycar->_teamname, 12);
		firstTime = false;
	}

    toLeft = car->_trkPos.toLeft;
    if (prevToLeft < -100.0f)
        avgLateralMovt = 0;
    else
        avgLateralMovt = avgLateralMovt*0.75 + (toLeft-prevToLeft)*0.25;
    prevToLeft = car->_trkPos.toLeft;

	simTime = s->currentTime;

    if (team == -1)
    {
        deltamult = (float)(1.0 / s->deltaTime);
        deltaTime = s->deltaTime;
    }

    if (teammate && team == -1) {
        team = TEAM_FRIEND;
    }
    else if (!teammate) {
        team = TEAM_FOE;
    }

    // Set state
    initState();
    // If the car is out of the simulation ignore it.
    //if (car->_state & (RM_CAR_STATE_NO_SIMU & ~RM_CAR_STATE_PIT)) 
	if ((car->_state & RM_CAR_STATE_NO_SIMU) && !(car->_state & RM_CAR_STATE_PIT))
    {
        return;
    }

	float trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
	double angle = trackangle - car->_yaw;
	NORM_PI_PI(angle);
	angle = fabs(angle);
	if (angle > 1.6)
		angle = 1.6 - (angle - 1.6);

	double extra_width = (car->_dimension_x - car->_dimension_y) / 2 * (angle / 1.6);

	left_toMid = car->_trkPos.toMiddle + car->_dimension_y/2 + extra_width;
	right_toMid = car->_trkPos.toMiddle - car->_dimension_y/2 - extra_width;
	left_speed_y = (left_toMid - last_left_toMid) / s->deltaTime;
	right_speed_y = (right_toMid - last_right_toMid) / s->deltaTime;
	last_left_toMid = left_toMid;
	last_right_toMid = right_toMid;

    /* Is opponent in relevant range */

    calcDist();
    calcSpeed();
    oppSpeed = (float)speed;
    mySpeed = driver->getSpeed();

    //calcState(driver->getSpeed());
    calcState(mySpeed, driver);

    // Opponent is faster mycar
    if (oppSpeed > mySpeed + SPEED_PASS_MARGIN) {
        oppFaster = true;
    }

    updateOverlapTimer(s, mycar);

    // Opponent is behind mycar
    if (state & OPP_BACK) {
		// Check if we should let overtake the opponent.
		if (oppSpeed > mySpeed - 1.0f && overlaptimer > OVERLAP_WAIT_TIME && car->_pos < mycar->_pos) {
		    state = OPP_LETPASS;
		}
		//fprintf(stderr, "%s opp=%s %d %d %d dist=%.1f olt=%.1f\n", mycar->_name, car->_name, oppSpeed > mySpeed - 1.0f, overlaptimer > OVERLAP_WAIT_TIME, car->_pos < mycar->_pos, distance, overlaptimer);
    }

    double dt = s->deltaTime;
    brake_overtake_filter *= (float) exp(-dt * .5);

    brakedistance = (float)(distance - car->_dimension_x);

    prev_speed_X = car->_speed_X;
    prev_speed_Y = car->_speed_Y;
    d_prev_speed_X = mycar->_speed_X;
    d_prev_speed_Y = mycar->_speed_Y;
}

void Opponent::initState()
{
    distance = DBL_MAX;
    //oppFront = false;
    //oppSlower = false;
    oppFaster = false;
    oppLetPass = false;
    oppFaraway = false;
    state = OPP_IGNORE;
}


// Updating distance along the middle.
void Opponent::calcDist()
{
    distance3 = distance2;
    distance2 = distance;
    //distance = car->_distRaced - mycar->_distRaced;
    distance = (car->_trkPos.seg->lgfromstart + getDistToSegStart(car)) - (mycar->_trkPos.seg->lgfromstart + getDistToSegStart(mycar));
    //float oppToStart = car->_trkPos.seg->lgfromstart + getDistToSegStart();
    //distance = oppToStart - mycar->_distFromStartLine;

    if (distance > track->length / 2.0f) {
        distance -= track->length;
    }
    else if (distance < -track->length / 2.0f) {
        distance += track->length;
    }

    trueSpeedDiff = (distance3 - distance) * deltamult / 2;

    double sa = (float)-(cardata->getTrackangle() - atan2(car->_speed_Y, car->_speed_X));
    NORM_PI_PI(sa);
	speedangle = (float)sa;
}


// Compute the length to the start of the segment.
float Opponent::getDistToSegStart(tCarElt *theCar)
{
    if (theCar->_trkPos.seg->type == TR_STR) {
        return theCar->_trkPos.toStart;
    }
    else {
        return theCar->_trkPos.toStart*theCar->_trkPos.seg->radius;
    }
}


void Opponent::calcSpeed()
{
    double ra = car->_yaw - mycar->_yaw;
    NORM_PI_PI(ra);
	relativeangle = (float)ra;
    if (fabs(distance) < 20.0) {
        if (fabs(relativeangle) > 0.5) {
            speed = getSpeedAngle(mycar->_yaw);
        }
        else {
            speed = car->_speed_x;
        }
    }
    else {
        speed = getSpeedAngle(RtTrackSideTgAngleL(&(car->_trkPos)));
    }

    speedDelta = speedDelta * 0.75 + (car->_speed_x - prevSpeed)*0.25;
    prevSpeed = car->_speed_x;

    average_AX = average_AX * 0.75 + car->pub.DynGCg.vel.x * 0.25;
    average_AY = average_AY * 0.75 + car->pub.DynGCg.vel.y * 0.25;

    {
        for (int j=3; j>0; j--)
            speedAngle[j] = speedAngle[j-1];
        double newx = car->_corner_x(FRNT_LFT) + car->_speed_X;
        double newy = car->_corner_y(FRNT_LFT) + car->_speed_Y;
        double dx = newx - car->_corner_x(FRNT_LFT);
        double dy = newy - car->_corner_y(FRNT_LFT);
        speedAngle[0] = atan2(dx, dy);
    }
}

double Opponent::getSpeedAngle(double angle)
{
    Vec2d speed, dir;
    speed.x = car->_speed_X;
    speed.y = car->_speed_Y;
    dir.x = cos(angle);
    dir.y = sin(angle);
    return speed * dir;
}

static bool LineIntersectsLine(tPosd *l1p1, tPosd *l1p2, tPosd *l2p1, tPosd *l2p2)
{
	float q = (l1p1->ay - l2p1->ay) * (l2p2->ax - l2p1->ax) - (l1p1->ax - l2p1->ax) * (l2p2->ay - l2p1->ay);
	float d = (l1p2->ax - l1p1->ax) * (l2p2->ay - l2p1->ay) - (l1p2->ay - l1p1->ay) * (l2p2->ax - l2p1->ax);

	if (d == 0)
	{
		return false;
	}

	float r = q / d;

	q = (l1p1->ay - l2p1->ay) * (l1p2->ax - l1p1->ax) - (l1p1->ax - l2p1->ax) * (l1p2->ay - l1p1->ay);
	float s = q / d;

	if (r < 0 || r > 1 || s < 0 || s > 1)
	{
		return false;
	}

	return true;
}

static bool LineIntersectsRect(tPosd *p1, tPosd *p2, tPosd *r0, tPosd *r1, tPosd *r2, tPosd *r3)
{
	return LineIntersectsLine(p1, p2, r0, r1) ||
		LineIntersectsLine(p1, p2, r1, r2) ||
		LineIntersectsLine(p1, p2, r2, r3) ||
		LineIntersectsLine(p1, p2, r3, r0);
}

void Opponent::calcState(float Speed, Driver *driver)
{
    //float COLLDIST = MAX(car->_dimension_x, mycar->_dimension_x);
    float COLLDIST = car->_dimension_x + 0.1f;
    state = 0;
	t_toside = 999.0;
	currentLft = currentRgt = impactLft = impactRgt = -10.0;
	withinBrakeDist = hasSlowerSpeed = false;

    if (distance > -BACKCOLLDIST && distance < FRONTCOLLDIST) 
    {
		currentLft = currentRgt = impactLft = impactRgt = -10.0;

		float tA = RtTrackSideTgAngleL(&(car->_trkPos));
		double oA = tA - car->_yaw;
		NORM_PI_PI(oA);
		oA = fabs(oA);
		if (oA > 1.6)
			oA = 1.6 - (oA - 1.6);

		double extraWidth = (car->_dimension_y + MAX(0.0, car->_dimension_x / 2 * (oA / 1.6) - (car->_dimension_y/2)));
		currentLft = impactLft = car->_trkPos.toLeft - extraWidth;
		currentRgt = impactLft = car->_trkPos.toLeft + extraWidth;

        // Is opponent aside.
        if (distance > -COLLDIST && distance < COLLDIST) 
		{
			if (distance < -car->_dimension_x + 1.0f)
			{
				tPosd p1, p2, r[4];
				for (int i = 0; i < 4; i++)
				{
					r[i].ax = mycar->_corner_x(i);
					r[i].ay = mycar->_corner_y(i);
				}

				if (car->_trkPos.toLeft > mycar->_trkPos.toLeft)
				{
					p1.ax = car->_corner_x(REAR_LFT);
					p1.ay = car->_corner_y(REAR_LFT);
					p2.ax = p1.ax - (car->_corner_x(REAR_RGT) - p1.ax) * 10;
					p2.ay = p1.ay - (car->_corner_y(REAR_RGT) - p1.ay) * 10;
				}
				else
				{
					p1.ax = car->_corner_x(REAR_RGT);
					p1.ay = car->_corner_y(REAR_RGT);
					p2.ax = p1.ax - (car->_corner_x(REAR_LFT) - p1.ax) * 10;
					p2.ay = p1.ay - (car->_corner_y(REAR_LFT) - p1.ay) * 10;
				}
				if (LineIntersectsRect(&p1, &p2, &r[0], &r[1], &r[2], &r[3]))
					state = OPP_SIDE;
			}
			else
				state = OPP_SIDE;
		}

		if (state == OPP_SIDE)
        {
//fprintf(stderr, "%s SIDE\n",car->_name);fflush(stderr);

            sidedist = car->_trkPos.toMiddle - mycar->_trkPos.toMiddle;
            state = OPP_SIDE;
            if (distance > 0.0) // && Speed > oppSpeed)
            {
                double newdistance = getCornerDist();
				if (distance > car->_dimension_x * 0.3) // && newdistance < 1.0 && (Speed - oppSpeed) < 2.0)
				{
					int coll = testLinearCollision2(driver);
					if (coll > 0 && (coll & OPP_COLL))
						state |= coll;
				}
                distance = newdistance;
                t_impact = MAX(0.0, MIN(1.5, distance / MAX(0.05, Speed - oppSpeed)));
                t_impact = (distance / (Speed - oppSpeed));
                catchdist = (float)(Speed * distance / (Speed - oppSpeed));
#ifdef BRAKE_DEBUG
				if (state & OPP_COLL)
				{
					fprintf(stderr, "%s %s: SIDE COLLISION\n", mycar->_name, car->_name); fflush(stderr);
				}
#endif
            }
        }
        // Is opponent in front.
        else if (distance >= 0.0) 
        {
//fprintf(stderr, "%s FRONT\n",car->_name);fflush(stderr);
            state = OPP_FRONT;
            state |= testQuadraticCollision(driver);
#ifdef BRAKE_DEBUG
			if (state & OPP_COLL)
			{
				fprintf(stderr, "%s %s: FRONT COLLISION\n", mycar->_name, car->_name); fflush(stderr);
			}
#endif
			if ((state & OPP_COLL) && driver->raceline->InBrakingZone(LINE_RL))
				collspeed -= 1.0;
            //oppFront = true;
            // Is opponent slower.
            if (oppSpeed <= Speed) 
            {
                //oppSlower = true;
                if (team == TEAM_FRIEND && car->_dammage - MAX_DAMAGE_DIFF < mycar->_dammage)
                    state |= OPP_FRONT_FOLLOW;
                //distance -= MAX(car->_dimension_x, mycar->_dimension_x);
				float colldistance = distance;// (float)(distance - FRONTCOLL_MARGIN);
                // If the distance is small we compute it more accurate.
				/*
                if (colldistance < MAX(EXACT_DIST, Speed-oppSpeed)) 
                {
                    distance = getCornerDist();
                    //if (mindist < distance) 
                    {
                        brakedistance = colldistance = (float)(distance - FRONTCOLL_MARGIN);
                    }
                }*/

                catchdist = (float)(Speed * colldistance / MAX(0.05, (Speed - oppSpeed)));

#if 1
                t_impact = MAX(0.0, colldistance / MAX(0.05, (Speed - oppSpeed)));
				t_toside = (MAX(0.0, (distance + car->_dimension_x) / MAX(0.05, ((Speed+0.5) - oppSpeed))));

				impactLft = currentLft - getAvgLateralMovt() * t_impact / deltaTime;
				impactRgt = currentRgt + getAvgLateralMovt() * t_impact / deltaTime;

                float cardist = car->_trkPos.toMiddle - mycar->_trkPos.toMiddle;
                sidedist = cardist;
                cardist = fabs(cardist) - fabs(getWidth() / 2.0f) - mycar->_dimension_y / 2.0f;
                double speed_diff = (Speed - oppSpeed);
                double brake_distance = driver->getBrakeCoefficient() * ((Speed+1.0) * (Speed+1.0));
                bool off_the_track = (fabs(car->_trkPos.toMiddle) > car->_trkPos.seg->width/2 + 3.0 && fabs(car->_trkPos.toMiddle - mycar->_trkPos.toMiddle) > 5.0);

				//testQuadraticCollision(driver);
#else
                if (catchdist < 5.0f || colldistance < 1.2)
                {
                    state = OPP_COLL;
                }
#endif
            }

            // Is opponent behind and faster.
        }
        else if (distance < 0.0 && distance >= -BACKCOLLDIST)
		{
			distance -= MAX(car->_dimension_x, mycar->_dimension_x);
			distance -= FRONTCOLL_MARGIN;

			if (oppSpeed > Speed - SPEED_PASS_MARGIN) {
				//fprintf(stderr, "%s BACK\n",car->_name);fflush(stderr);
				oppFaster = true;
				state = OPP_BACK;
				catchdist = (float)(Speed * distance / (oppSpeed - Speed));

				double dist = (mycar->_distRaced - car->_distRaced) - mycar->_dimension_x;
				if (team != TEAM_FRIEND && car->_speed_x > mycar->_speed_x && dist > 0)
				{
					t_impact = ((dist / (car->_speed_x - mycar->_speed_x)) * (1.0 + fabs(driver->raceline->tRInverse[LINE_RL][driver->raceline->Next]) * 2000));
					if (t_impact < 10.0)
					{
						state |= OPP_BACK_CATCHING;
						//					fprintf(stderr, "A %s: %.1f CATCHING!\n", car->_name, t_impact); fflush(stderr);
					}
				}
			}

			if (team != TEAM_FRIEND && car->_pos > mycar->_pos)
				state |= OPP_BACK_THREAT;
        }
        // Is opponent in front and faster.
        else if (distance >= COLLDIST && oppSpeed > Speed) {
//fprintf(stderr, "%s FRONT FAST\n",car->_name);fflush(stderr);
            oppFaster = true;
            state = OPP_FRONT_FAST;
            distance -= MAX(car->_dimension_x, mycar->_dimension_x);
            if (distance < EXACT_DIST) 
                distance = getCornerDist();
            if (team == TEAM_FRIEND && car->_dammage - MAX_DAMAGE_DIFF < mycar->_dammage) {
                state = OPP_FRONT_FOLLOW;
            }
            if (distance < 20.0 - (oppSpeed - Speed) * 4) {
                state = OPP_FRONT;
            }
        }
    }
    else if (distance <= -BACKCOLLDIST || distance >= FRONTCOLLDIST) {
//fprintf(stderr, "%s FARAWAY\n",car->_name);fflush(stderr);
		double dist = (mycar->_distRaced - car->_distRaced) - mycar->_dimension_x;
		if (team != TEAM_FRIEND && car->_speed_x > mycar->_speed_x && dist > 0)
		{
			t_impact = ((dist / (car->_speed_x - mycar->_speed_x)) * (1.0 + fabs(driver->raceline->tRInverse[LINE_RL][driver->raceline->Next])*2000));
			if (t_impact < 10.0)
			{
				state |= OPP_BACK_CATCHING;
//				fprintf(stderr, "B %s: %.1f CATCHING!\n", car->_name, t_impact); fflush(stderr);
			}
			else
				state = OPP_IGNORE;
		}
		else
		{
			oppFaraway = true;
			state = OPP_IGNORE;
		}
    }

#ifdef OPP_DEBUG
    if (state)// && state != OPP_IGNORE)
    {
        fprintf(stderr, "%s - %s: %.1f %s %s %s\n", mycar->_name, car->_name, distance, (state & OPP_SIDE) ? "SIDE" : ((state & OPP_FRONT) ? "FRONT" : ((state & OPP_BACK) ? "BACK" : "")), (state & OPP_COLL) ? "COLL" : "",(state & OPP_IGNORE)?"IGNORE":"");
	fflush(stderr);
    }
#endif
}


float Opponent::getCornerDist()
{
	return getCornerDist(mycar, car);
}

static float DistFromPoint2Line(float x, float y, float x1, float y1, float x2, float y2)
{
	float A = x - x1;
	float B = y - y1;
	float C = x2 - x1;
	float D = y2 - y1;

	float dot = A * C + B * D;
	float len_sq = C * C + D * D;
	float param = -1;
	if (len_sq != 0) //in case of 0 length line
		param = dot / len_sq;

	float xx, yy;

	if (param < 0) {
		xx = x1;
		yy = y1;
	}
	else if (param > 1) {
		xx = x2;
		yy = y2;
	}
	else {
		xx = x1 + param * C;
		yy = y1 + param * D;
	}

	float dx = x - xx;
	float dy = y - yy;
	return (dx * dx + dy * dy);
}

float Opponent::getCornerDist(tCarElt *dCar, tCarElt *oCar)
{
	float minDist = FLT_MAX;
	for (int i = 0; i < 4; i++)
	{
		minDist = MIN(minDist, DistFromPoint2Line(dCar->_corner_x(i), dCar->_corner_y(i), oCar->_corner_x(FRNT_LFT), oCar->_corner_y(FRNT_LFT), oCar->_corner_x(FRNT_RGT), oCar->_corner_y(FRNT_RGT)));
		minDist = MIN(minDist, DistFromPoint2Line(dCar->_corner_x(i), dCar->_corner_y(i), oCar->_corner_x(FRNT_RGT), oCar->_corner_y(FRNT_RGT), oCar->_corner_x(REAR_RGT), oCar->_corner_y(REAR_RGT)));
		minDist = MIN(minDist, DistFromPoint2Line(dCar->_corner_x(i), dCar->_corner_y(i), oCar->_corner_x(REAR_LFT), oCar->_corner_y(REAR_LFT), oCar->_corner_x(REAR_RGT), oCar->_corner_y(REAR_RGT)));
		minDist = MIN(minDist, DistFromPoint2Line(dCar->_corner_x(i), dCar->_corner_y(i), oCar->_corner_x(REAR_LFT), oCar->_corner_y(REAR_LFT), oCar->_corner_x(FRNT_LFT), oCar->_corner_y(FRNT_LFT)));
		minDist = MIN(minDist, DistFromPoint2Line(oCar->_corner_x(i), oCar->_corner_y(i), dCar->_corner_x(FRNT_LFT), dCar->_corner_y(FRNT_LFT), dCar->_corner_x(FRNT_RGT), dCar->_corner_y(FRNT_RGT)));
		minDist = MIN(minDist, DistFromPoint2Line(oCar->_corner_x(i), oCar->_corner_y(i), dCar->_corner_x(FRNT_RGT), dCar->_corner_y(FRNT_RGT), dCar->_corner_x(REAR_RGT), dCar->_corner_y(REAR_RGT)));
		minDist = MIN(minDist, DistFromPoint2Line(oCar->_corner_x(i), oCar->_corner_y(i), dCar->_corner_x(REAR_LFT), dCar->_corner_y(REAR_LFT), dCar->_corner_x(REAR_RGT), dCar->_corner_y(REAR_RGT)));
		minDist = MIN(minDist, DistFromPoint2Line(oCar->_corner_x(i), oCar->_corner_y(i), dCar->_corner_x(REAR_LFT), dCar->_corner_y(REAR_LFT), dCar->_corner_x(FRNT_LFT), dCar->_corner_y(FRNT_LFT)));
	}

	if (minDist > 0.0f)
		minDist = sqrt(minDist) - 1.0f;
	return MAX(0.0f, minDist);

#if 0
    straight2d frontLine(
        dCar->_corner_x(FRNT_LFT),
        dCar->_corner_y(FRNT_LFT),
        dCar->_corner_x(FRNT_RGT) - dCar->_corner_x(FRNT_LFT),
        dCar->_corner_y(FRNT_RGT) - dCar->_corner_y(FRNT_LFT)
        );
    straight2d rearLine(
        dCar->_corner_x(REAR_LFT),
        dCar->_corner_y(REAR_LFT),
        dCar->_corner_x(REAR_RGT) - dCar->_corner_x(REAR_LFT),
        dCar->_corner_y(REAR_RGT) - dCar->_corner_y(REAR_LFT)
        );
    straight2d leftLine(
        dCar->_corner_x(FRNT_LFT),
        dCar->_corner_y(FRNT_LFT),
        dCar->_corner_x(REAR_LFT) - dCar->_corner_x(FRNT_LFT),
        dCar->_corner_y(REAR_LFT) - dCar->_corner_y(FRNT_LFT)
        );
    straight2d rightLine(
        dCar->_corner_x(FRNT_RGT),
        dCar->_corner_y(FRNT_RGT),
        dCar->_corner_x(REAR_RGT) - dCar->_corner_x(FRNT_RGT),
        dCar->_corner_y(REAR_RGT) - dCar->_corner_y(FRNT_RGT)
        );
    double mindist = DBL_MAX;
    bool left[4];
    bool right[4];
    for (int i = 0; i < 4; i++) {
        Vec2d corner(oCar->_corner_x(i), oCar->_corner_y(i));
        double frontdist = frontLine.dist(corner);
        double reardist = rearLine.dist(corner);
        double leftdist = leftLine.dist(corner);
        double rightdist = rightLine.dist(corner);
        bool front = frontdist < reardist && reardist > dCar->_dimension_x ? true : false;
        bool rear = reardist < frontdist && frontdist > dCar->_dimension_x ? true : false;
        left[i] = leftdist < rightdist && rightdist > dCar->_dimension_y ? true : false;
        right[i] = rightdist < leftdist && leftdist > dCar->_dimension_y ? true : false;
        double dist = DBL_MAX;
        if (front) {
            dist = frontdist;
        }
        else if (rear) {
            dist = -reardist;
        }
        if (fabs(dist) < fabs(mindist)) {
            mindist = dist;
        }
    }
    if (fabs(mindist) > 3.0) {
        mindist -= SIGN(mindist) * 2.99;
    }
    else {
        mindist = SIGN(mindist) * 0.01;
    }
    /*
    bool lft = true;
    bool rgt = true;
    for (int j = 0; j < 4; j++) {
        if (!left[j]) {
            lft = false;
        }
    }
    for (int k = 0; k < 4; k++) {
        if (!right[k]) {
            rgt = false;
        }
    }
    if (lft || rgt) {
        // opponent aside
        mindist = 0.0;
    }
    */
    return (float)mindist;
#endif
}


// Update overlaptimers of opponents.
void Opponent::updateOverlapTimer(tSituation *s, tCarElt *mycar)
{
#if 0
    float mycarFromStartLane = mycar->_distRaced;
    float oppFromStartLane = car->_distRaced;
    float distOpponentCar = oppFromStartLane - mycarFromStartLane;
    fprintf(stderr,"DistOffmyCar %.2f DistOffOpp %.2f OppCarDist %.2f\n", mycarFromStartLane, oppFromStartLane, distOpponentCar);
#endif
    if (car->_pos < mycar->_pos ||
        ((team == TEAM_FRIEND) && mycar->_dammage > car->_dammage + MAX_DAMAGE_DIFF))
    {
        if (getState() & (OPP_BACK | OPP_SIDE)) {
            overlaptimer += s->deltaTime;
        }
        else if (getState() & OPP_FRONT) {
            overlaptimer = LAP_BACK_TIME_PENALTY;
        }
        else {
            if (overlaptimer > 0.0) {
                if (getState() & OPP_FRONT_FAST) {
                    overlaptimer = MIN(0.0, overlaptimer);
                }
                else {
                    overlaptimer -= s->deltaTime;
                }
            }
            else {
                overlaptimer += s->deltaTime;
            }
        }
    }
    else {
        overlaptimer = 0.0;
    }
}



int Opponent::polyOverlap(tPosd *op, tPosd *dp)
{
    int i, j;

    // need this to ensure corners are used in the right order
    int cpos[4] = { 1, 0, 2, 3 };

    for (j = 0; j < 4; j++)
    {
        tPosd *j1 = op + cpos[j];
        tPosd *j2 = op + cpos[((j + 1) % 4)];

        for (i = 0; i < 4; i++)
        {
            tPosd *i1 = dp + cpos[i];
            tPosd *i2 = dp + cpos[((i + 1) % 4)];

            double aM, bM, aB, bB, isX = 0, isY = 0;
            double lineAx1 = j1->ax;
            double lineAx2 = j2->ax;
            double lineAy1 = j1->ay;
            double lineAy2 = j2->ay;
            double lineBx1 = i1->ax;
            double lineBx2 = i2->ax;
            double lineBy1 = i1->ay;
            double lineBy2 = i2->ay;

            if ((lineAx2 - lineAx1) == 0.0)
            {
                if ((lineBx2 - lineBx1) == 0.0)
                    continue;
                isX = lineAx1;
                bM = (lineBy2 - lineBy1) / (lineBx2 - lineBx1);
                bB = lineBy2 - bM * lineBx2;
                isY = bM * isX + bB;
            }
            else if ((lineBx2 - lineBx1) == 0.0)
            {
                isX = lineBx1;
                aM = (lineAy2 - lineAy1) / (lineAx2 - lineAx1);
                aB = lineAy2 - aM * lineAx2;
                isY = aM * isX + aB;
            }
            else
            {
                aM = (lineAy2 - lineAy1) / (lineAx2 - lineAx1);
                bM = (lineBy2 - lineBy1) / (lineBx2 - lineBx1);
                aB = lineAy2 - aM * lineAx2;
                bB = lineBy2 - bM * lineBx2;
                isX = MAX(((bB - aB) / (aM - bM)), 0);
                isY = aM * isX + aB;
            }

            if (isX < MIN(lineAx1, lineAx2) || isX < MIN(lineBx1, lineBx2) || isX > MAX(lineAx1, lineAx2) || isX > MAX(lineBx1, lineBx2))
                continue;
            if (isY < MIN(lineAy1, lineAy2) || isY < MIN(lineBy1, lineBy2) || isY > MAX(lineAy1, lineAy2) || isY > MAX(lineBy1, lineBy2))
                continue;

            return 1;
        }
    }

    return 0;
}

static double Mag(double x, double y)
{
    return sqrt((x*x) + (y*y));
}

static double findDistanceBetweenCars(tPosd *c1, tPosd *c2)
{
    double mindist = 10000000;
    
    for (int i=0; i<4; i++)
    {
        for (int j=0; j<4; j++)
        {
            double x = (c2[j].ax-c1[i].ax), y = (c2[j].ay - c1[i].ay);
            double thisdist = sqrt((x * x) + (y * y));
            mindist = MIN(thisdist, mindist);
        }
    }

    return mindist;
}


int Opponent::testLinearCollision2(Driver *driver)
{
    tPosd o_cur[4], d_cur[4], o_new[4], d_new[4], o_new2[4], d_new2[4];
    int i;
    // set up car current positions
    for (i = 0; i < 4; i++)
    {
        o_new[i].ax = o_cur[i].ax = car->_corner_x(i);
        o_new[i].ay = o_cur[i].ay = car->_corner_y(i);
        d_new2[i].ax = d_new[i].ax = d_cur[i].ax = mycar->_corner_x(i);
        d_new2[i].ay = d_new[i].ay = d_cur[i].ay = mycar->_corner_y(i);
    }

    double myta = fabs(RtTrackSideTgAngleL(&(mycar->_trkPos))) * 180 / 3.14159;
    double ota = fabs(RtTrackSideTgAngleL(&(car->_trkPos))) * 180 / 3.14159;
    double tadiff = fabs(myta - ota);
    brakedistance = distance = getCornerDist();
	double trackdistance = brakedistance;
    //distance = (tdble)findDistanceBetweenCars(o_cur, d_cur);
#if 0
    if (distance > MAX(5.0, 6 * ((90 - tadiff) / 18)))
    {
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d NO COLL due to track angle\n",car->_name,mycar->_dammage);fflush(stderr);
#endif
        return 0;
    }
#endif
    double brake_coefficient = driver->getBrakeCoefficient();
    if (fabs(relativeangle) > 0.3f)
        brake_coefficient /= (1.0 + (fabs(relativeangle) - 0.3f));
    //if (car->_speed_x < 10 && distance < 3)
    //    brake_coefficient = MIN(brake_coefficient, 0.02);
#if 1
    if (!(state & OPP_SIDE))
    {
		collspeed = (float)CalcCollSpeed(driver);
    }
    else
        collspeed = car->_speed_x - 1.0f;
#else
    collspeed = (float)(car->_speed_x + (_impact * MAX(1.0, timpact/2) * (brake_coefficient*10)));
#endif
    double ospeed = car->_speed_x + (distance < 2.0 ? MIN(0.0, car->_accel_x * t_impact * 2.0) : 0.0);
    double speedDiff = mycar->_speed_x - car->_speed_x;
    double timpact = (distance) / speedDiff;
    if (distance < 5.0 && car->_accel_x < 0.0)
        speedDiff = mycar->_speed_x - (car->_speed_x + MAX(-2.0f, car->_accel_x) * t_impact * 4);
    timpact = (distance) / speedDiff;

	if (speedDiff >= 0.0 && distance <= 0.0 && fabs(car->_trkPos.toLeft - mycar->_trkPos.toLeft) < MAX(cardata->getWidthOnTrack(), driver->getWidthOnTrack()) + 0.5)
	{
#ifdef BRAKE_DEBUG
		fprintf(stderr, "%s %d COLL A: sdiff %.3f and distance %.1f > 1.0\n", car->_name, mycar->_dammage, speedDiff, distance); fflush(stderr);
#endif
		collspeed = MIN(collspeed, car->_speed_x - 2.0f);
		return OPP_COLL;
	}

	if (timpact < 0.0 && distance > 2.0)
	{
#ifdef BRAKE_DEBUG
		fprintf(stderr, "%s %d NO COLL A: t_impact %.2f <= 0.0 and distance %.1f > 1.0\n", car->_name, mycar->_dammage, timpact, distance); fflush(stderr);
#endif
        return -1;
	}

    //if (timpact > MAX(1.0, MIN(distance*2, car->_speed_x / 20)) / (simTime < 2.0 ? 2 : 1) && distance > 0.5)
	if (mycar->_speed_x <= collspeed && distance > 0.5 && !(state & OPP_SIDE))
    {
#ifdef BRAKE_DEBUG
		//fprintf(stderr, "%s %d NO COLL B: t_impact %.2f > speed factor %.2f, dist = %.2f\n", car->_name, mycar->_dammage, timpact, MAX(1.0, MIN(trackdistance * 2, car->_speed_x / 20)) / (simTime < 2.0 ? 2 : 1), distance); fflush(stderr);
		fprintf(stderr, "%s %d NO COLL B: speed %.2f < collspeed %.2f\n", car->_name, mycar->_dammage, mycar->_speed_x, collspeed); fflush(stderr);
#endif
        return -1;
    }

	double slowSpeed = car->_speed_x;
	if (distance > MIN(mycar->_speed_x / 3, MAX(1.0, (mycar->_speed_x - car->_speed_x) * 3)))
	{
		int div = driver->raceline->DivIndexForCar(car, timpact);
		if (driver->raceline->IsSlowerThanSpeedToDiv(driver->linemode, car->_speed_x + MIN(0.0, car->_accel_x / 4), div, &slowSpeed))
		{
#ifdef BRAKE_DEBUG
			fprintf(stderr, "%s %d NO COLL C: Slower Speed %.2f between opponent and collision point\n", car->_name, mycar->_dammage, slowSpeed); fflush(stderr);
#endif
			if (distance > MAX(2, 4.0 - fabs(driver->raceline->tRInverse[LINE_RL][driver->raceline->This]) * 1000))
				hasSlowerSpeed = true;
			return -1;
		}
	}

	speedDiff = ((mycar->_speed_x + slowSpeed) / 2) - car->_speed_x;
	timpact = distance / speedDiff;

	if (timpact < 0.0 && distance > 2.0 + MAX(0.0, MIN(2.0, -car->_accel_x)))
	{
#ifdef BRAKE_DEBUG
		fprintf(stderr, "%s %d NO COLL D: t_impact %.2f <= 0.0 and distance %.1f > 1.0\n", car->_name, mycar->_dammage, timpact, distance); fflush(stderr);
#endif
        return -1;
	}

#if 0
    if (timpact > MAX(1.0, MIN(distance*2, car->_speed_x / 20)) / (simTime < 2.0 ? 2 : 1) && distance > 0.5)
    {
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d NO COLL E: t_impact %.2f > speed factor %.2f, dist = %.2f\n",car->_name,mycar->_dammage,timpact,MAX(1.0, car->_speed_x/20), distance);fflush(stderr);
#endif
        return -1;
    }
#endif

	withinBrakeDist = true;

    double timpact2 = (tdble)distance / speedDiff;

    if (car->_speed_x > 15)
    {
        // make the opponent a little bigger so we stop short of actually hitting
        tdble s1 = ((float)driver->getBrakeMargin() / mycar->_dimension_x)/4, s2 = s1*0.7f;
        if (distance < 2.0)
        {
            if (car->_speed_x > 5.0)
            {
                s1 *= (tdble)MIN(2.0, (1.0+(fabs(relativeangle))));
                s2 *= (tdble)MIN(2.0, (1.0+(fabs(relativeangle))));
            }
            else if (fabs(relativeangle) > 0.5)
            {
                s1 *= (tdble)MIN(2.0, (1.0+(fabs(relativeangle)-0.5)));
                s2 *= (tdble)MIN(2.0, (1.0+(fabs(relativeangle)-0.7)));
            }
        }
        o_new[REAR_LFT].ax = o_cur[REAR_LFT].ax + (o_cur[REAR_LFT].ax-o_cur[REAR_RGT].ax)*s1;
        o_new[REAR_LFT].ay = o_cur[REAR_LFT].ay + (o_cur[REAR_LFT].ay-o_cur[REAR_RGT].ay)*s1;
        o_new[REAR_RGT].ax = o_cur[REAR_RGT].ax + (o_cur[REAR_RGT].ax-o_cur[REAR_LFT].ax)*s1;
        o_new[REAR_RGT].ay = o_cur[REAR_RGT].ay + (o_cur[REAR_RGT].ay-o_cur[REAR_LFT].ay)*s1;
        o_new[FRNT_LFT].ax = o_cur[FRNT_LFT].ax + (o_cur[FRNT_LFT].ax-o_cur[FRNT_RGT].ax)*s1;
        o_new[FRNT_LFT].ay = o_cur[FRNT_LFT].ay + (o_cur[FRNT_LFT].ay-o_cur[FRNT_RGT].ay)*s1;
        o_new[FRNT_RGT].ax = o_cur[FRNT_RGT].ax + (o_cur[FRNT_RGT].ax-o_cur[FRNT_LFT].ax)*s1;
        o_new[FRNT_RGT].ay = o_cur[FRNT_RGT].ay + (o_cur[FRNT_RGT].ay-o_cur[FRNT_LFT].ay)*s1;
        o_new[FRNT_LFT].ax = o_cur[FRNT_LFT].ax + (o_cur[FRNT_LFT].ax-o_cur[REAR_LFT].ax)*s2;
        o_new[FRNT_LFT].ay = o_cur[FRNT_LFT].ay + (o_cur[FRNT_LFT].ay-o_cur[REAR_LFT].ay)*s2;
        o_new[FRNT_RGT].ax = o_cur[FRNT_RGT].ax + (o_cur[FRNT_RGT].ax-o_cur[REAR_RGT].ax)*s2;
        o_new[FRNT_RGT].ay = o_cur[FRNT_RGT].ay + (o_cur[FRNT_RGT].ay-o_cur[REAR_RGT].ay)*s2;
        o_new[REAR_LFT].ax = o_cur[REAR_LFT].ax + (o_cur[REAR_LFT].ax-o_cur[FRNT_LFT].ax)*s2;
        o_new[REAR_LFT].ay = o_cur[REAR_LFT].ay + (o_cur[REAR_LFT].ay-o_cur[FRNT_LFT].ay)*s2;
        o_new[REAR_RGT].ax = o_cur[REAR_RGT].ax + (o_cur[REAR_RGT].ax-o_cur[FRNT_RGT].ax)*s2;
        o_new[REAR_RGT].ay = o_cur[REAR_RGT].ay + (o_cur[REAR_RGT].ay-o_cur[FRNT_RGT].ay)*s2;
    }

    d_new2[FRNT_LFT].ax += (d_new[FRNT_LFT].ax - d_new[REAR_LFT].ax) / 3;
    d_new2[FRNT_LFT].ay += (d_new[FRNT_LFT].ay - d_new[REAR_LFT].ay) / 3;
    d_new2[FRNT_RGT].ax += (d_new[FRNT_RGT].ax - d_new[REAR_RGT].ax) / 3;
    d_new2[FRNT_RGT].ay += (d_new[FRNT_RGT].ay - d_new[REAR_RGT].ay) / 3;

    if (polyOverlap(o_new, d_new2))
    {
        collspeed = (float) MIN(collspeed, car->_speed_x-2.0);
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d CLOSE OVERLAP COLLISION collspeed=%.2f\n",car->_name,mycar->_dammage,collspeed);fflush(stderr);
#endif
        return OPP_COLL | OPP_COLL_WARNING; // cars are overlapping now.
    }

    //collspeed = (float)(car->_speed_x + (car->_accel_x * timpact2) + timpact * (brake_coefficient*10));
    //collspeed = MIN(collspeed, (collspeed - fabs(relativeangle)*3) + MAX(0.0, (distance-1.0)*2.3));

    if (!(state & OPP_SIDE) && speedDiff <= 0 && car->_speed_x > 5.0f) 
    {
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d NO COLL: speedDiff %.1f <= 0 and speed %.2f > 5.0\n",car->_name,mycar->_dammage,speedDiff,car->_speed_x);fflush(stderr);
#endif
        return -1;
    }
    //speedDiff *= MIN(1.0, speedDiff/3);

#if 0
    LRaceLine *raceline = driver->getRaceLine();
    int rl = driver->currentRaceline();
    int div = raceline->DivIndexForCar(car, timpact);
    double rlspeed = raceline->tSpeed[rl][div];

    if (driver->isOnRaceline() && rlspeed > collspeed && rlspeed <= car->_speed_x)
    {
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d NO COLL: speedDiff=%.1f collspeed=%.2f\n",car->_name,mycar->_dammage,speedDiff,collspeed);fflush(stderr);
#endif
        return 0;
    }
#endif

    if (timpact > ((state & OPP_SIDE) ? 1.5 : 4.0))//MIN(3.0, MAX(0.7, speedDiff)))
    {
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d SIDE - NO COLL: t_impact %.2f > 5.0, dist=%.1f speedDiff=%.1f collspeed=%.2f\n",car->_name,mycar->_dammage,timpact,distance,speedDiff,collspeed);fflush(stderr);
#endif
        return -1;
    }

    // move the opponent back a little towards our car, and project everything forward in time
    double deltax = 0.0, deltay = 0.0;
    for (i = 0; i < 4; i++)
    {
        // project opponent car by linear velocity, and make o_new2 moved back a little
        // in the direction of our car.
        double thisdist = sqrt((deltax * deltax) + (deltay * deltay));
        o_new2[i].ax = (tdble)(o_cur[i].ax + car->_speed_X * timpact2);
        o_new2[i].ay = (tdble)(o_cur[i].ay + car->_speed_Y * timpact2);
        deltax = o_new2[i].ax - o_cur[i].ax;
        deltay = o_new2[i].ay - o_cur[i].ay;
        o_new[i].ax = o_cur[i].ax + (tdble)(thisdist * sin(getProjectedSpeedAngle(timpact2)));
        o_new[i].ay = o_cur[i].ay + (tdble)(thisdist * cos(getProjectedSpeedAngle(timpact2)));

        if (i == FRNT_LFT || i == FRNT_RGT)
        {
            // project front of our car by speed and turn direction
            d_new[i].ax += (tdble)(mycar->_speed_X * timpact2);
            d_new[i].ay += (tdble)(mycar->_speed_Y * timpact2);
            deltax = d_new[i].ax - d_cur[i].ax;
            deltay = d_new[i].ay - d_cur[i].ay;
            //theta = atan2(deltax, deltay) - ((mycar->_yaw_rate*0.8) * timpact2);
            thisdist = sqrt((deltax * deltax) + (deltay * deltay));
            double newax = d_cur[i].ax + thisdist * sin(driver->getSpeedAngle(timpact2));
            double neway = d_cur[i].ay + thisdist * cos(driver->getSpeedAngle(timpact2));
            if ((mycar->_yaw_rate < 0.0 && i == FRNT_RGT) || (mycar->_yaw_rate > 0.0 && i == FRNT_LFT))
            {
                d_new2[i].ax = (tdble)newax;
                d_new2[i].ay = (tdble)neway;
            }
            else
            {
                d_new2[i].ax = (tdble)(d_new[i].ax * 0.3 + newax * 0.7);
                d_new2[i].ay = (tdble)(d_new[i].ay * 0.3 + neway * 0.7);
            }
//fprintf(stderr, "%s: old=%.2f/%.2f new1=%.2f/%.2f yr=%.5f theta=%.5f new2=%.2f/%.2f\n",car->_name,d_cur[i].ax,d_cur[i].ay,new_ax,new_ay,mycar->_yaw_rate,theta,d_new[i].ax,d_new[i].ay);fflush(stderr);
        }
    }

    deltax = ((d_new2[FRNT_LFT].ax-o_new[REAR_LFT].ax)+(d_new2[FRNT_RGT].ax-o_new[REAR_RGT].ax))/2;
    deltay = ((d_new2[FRNT_LFT].ay-o_new[REAR_LFT].ay)+(d_new2[FRNT_RGT].ay-o_new[REAR_RGT].ay))/2;
    double theta = atan2(deltax, deltay);
    double movedist = 0.5 + MIN(1.5, speedDiff/15);
    for (i = 0; i < 4; i++)
    {
        o_new2[i].ax = (tdble)(o_new[i].ax + movedist * sin(theta));
        o_new2[i].ay = (tdble)(o_new[i].ay + movedist * cos(theta));
    }

    double trackdist = car->_distFromStartLine - mycar->_distFromStartLine;
    double sidedist = fabs(car->_trkPos.toLeft - mycar->_trkPos.toLeft);
    if ((!(MIN(car->_trkPos.toLeft, car->_trkPos.toRight) < -3 && MIN(mycar->_trkPos.toLeft, mycar->_trkPos.toRight) >= 0)) &&
        (sidedist / 2 < MAX(timpact, trackdist/2) || sidedist < 2.0 || speedDiff > 15))
        if (polyOverlap(o_new, d_new2) || polyOverlap(o_new2, d_new2))
        {
#ifdef BRAKE_DEBUG
            fprintf(stderr, "%s %d PROJECTED OVERLAP COLLISION dist=%.2f ospd=%.2f spd=%.2f collspeed=%.2f t_impact=%.2f sidedist=%.2f\n",car->_name,mycar->_dammage,distance,car->_speed_x,mycar->_speed_x,collspeed,timpact,fabs(car->_trkPos.toLeft - mycar->_trkPos.toLeft));fflush(stderr);
#endif
            return OPP_COLL | OPP_COLL_WARNING;
        }

    //if (driver->isOnRaceline())// && car->_pos > mycar->_pos)
    if ((state & OPP_SIDE))
        return 0;

#if 0
	if (collspeed < mycar->_speed_x && timpact < 5.0)
    {
        int coll = testRacelineCollision(driver, distance, timpact);
        if (coll)
        {
#ifdef BRAKE_DEBUG
            if (coll & OPP_COLL)
                fprintf(stderr, "%s %d projected %s collision, collspeed=%.2f\n",car->_name,mycar->_dammage,((coll & OPP_RACELINE_CONFLICT) ? "raceline" : "linear"),collspeed);fflush(stderr);
#endif
            return coll;
        }
    }
#endif
#ifdef BRAKE_DEBUG
    fprintf(stderr, "%s %d NO collision: t_impact %.4f, dist=%.2f speedDiff=%.1f collspeed=%.2f\n",car->_name,mycar->_dammage,timpact,distance,speedDiff,collspeed);fflush(stderr);
#endif

#if 0 // not used in driver.cpp, so no need to process warnings
    if (mycar->_yaw_rate < 0.0)
    {
        d_new2[FRNT_LFT].ax = d_new[FRNT_LFT].ax;
        d_new2[FRNT_LFT].ay = d_new[FRNT_LFT].ay;
    }
    else
    {
        d_new2[FRNT_RGT].ax = d_new[FRNT_RGT].ax;
        d_new2[FRNT_RGT].ay = d_new[FRNT_RGT].ay;
    }
    o_new[REAR_LFT].ax = (o_new2[REAR_LFT].ax + (o_new2[REAR_LFT].ax-o_new2[REAR_RGT].ax)/3);
    o_new[REAR_LFT].ay = (o_new2[REAR_LFT].ay + (o_new2[REAR_LFT].ay-o_new2[REAR_RGT].ay)/3);
    o_new[REAR_RGT].ax = (o_new2[REAR_RGT].ax + (o_new2[REAR_RGT].ax-o_new2[REAR_LFT].ax)/3);
    o_new[REAR_RGT].ay = (o_new2[REAR_RGT].ay + (o_new2[REAR_RGT].ay-o_new2[REAR_LFT].ay)/3);
    o_new[FRNT_LFT].ax = (o_new2[FRNT_LFT].ax + (o_new2[FRNT_LFT].ax-o_new2[FRNT_RGT].ax)/3);
    o_new[FRNT_LFT].ay = (o_new2[FRNT_LFT].ay + (o_new2[FRNT_LFT].ay-o_new2[FRNT_RGT].ay)/3);
    o_new[FRNT_RGT].ax = (o_new2[FRNT_RGT].ax + (o_new2[FRNT_RGT].ax-o_new2[FRNT_LFT].ax)/3);
    o_new[FRNT_RGT].ay = (o_new2[FRNT_RGT].ay + (o_new2[FRNT_RGT].ay-o_new2[FRNT_LFT].ay)/3);
    o_new[FRNT_LFT].ax = (o_new2[FRNT_LFT].ax + (o_new2[FRNT_LFT].ax-o_new2[REAR_LFT].ax)/3);
    o_new[FRNT_LFT].ay = (o_new2[FRNT_LFT].ay + (o_new2[FRNT_LFT].ay-o_new2[REAR_LFT].ay)/3);
    o_new[FRNT_RGT].ax = (o_new2[FRNT_RGT].ax + (o_new2[FRNT_RGT].ax-o_new2[REAR_RGT].ax)/3);
    o_new[FRNT_RGT].ay = (o_new2[FRNT_RGT].ay + (o_new2[FRNT_RGT].ay-o_new2[REAR_RGT].ay)/3);
    o_new[REAR_LFT].ax = (o_new2[REAR_LFT].ax + (o_new2[REAR_LFT].ax-o_new2[FRNT_LFT].ax)/3);
    o_new[REAR_LFT].ay = (o_new2[REAR_LFT].ay + (o_new2[REAR_LFT].ay-o_new2[FRNT_LFT].ay)/3);
    o_new[REAR_RGT].ax = (o_new2[REAR_RGT].ax + (o_new2[REAR_RGT].ax-o_new2[FRNT_RGT].ax)/3);
    o_new[REAR_RGT].ay = (o_new2[REAR_RGT].ay + (o_new2[REAR_RGT].ay-o_new2[FRNT_RGT].ay)/3);

    if (polyOverlap(o_new, d_new2))
    {
        fprintf(stderr, "%s WARNING COLLISION\n",car->_name);fflush(stderr);
        return OPP_COLL_WARNING;
    }
#endif

    return 0;
}

double Opponent::CalcCollSpeed(Driver *driver)
{
	double mySpd = hypot(mycar->_speed_X, mycar->_speed_Y);
	if (fabs(mySpd) < 0.01) mySpd = 0.01;
	double myDirX = car->_speed_X / mySpd;
	double myDirY = car->_speed_Y / mySpd;

	double dPX = car->pub.DynGCg.pos.x - mycar->pub.DynGCg.pos.x;
	double dPY = car->pub.DynGCg.pos.y - mycar->pub.DynGCg.pos.y;
	double dVX = car->_speed_X - mycar->_speed_X;
	double dVY = car->_speed_Y - mycar->_speed_Y;
	double rdPX = myDirX * dPX + myDirY * dPY;
	double rdPY = myDirY * dPX - myDirX * dPY;
	double rdVX = myDirX * dVX + myDirY * dVY;
	double rdVY = myDirY * dVX - myDirX * dVY;
	double oVX = car->_speed_x + rdVX;
	double minDY = (mycar->_dimension_y + car->_dimension_y) / 2;
	double minDX = (mycar->_dimension_x + car->_dimension_x) / 2;

	double currentEk = driver->mass() * mycar->_speed_x * mycar->_speed_x / 2;
    double ospeed = car->_speed_x + (distance < 2.0 ? MIN(0.0, car->_accel_x * t_impact * 2.0) : 0.0);
	double targetEk = driver->mass() * ospeed * ospeed / 2;
    double brake_coefficient = driver->getBrakeCoefficient();
	if (currentEk - targetEk > t_impact * 750000 / brake_coefficient)
	{
		if (state & OPP_SIDE)
		{
			if (rdPY < minDY)      // colliding now
				collspeed = (float)oVX - 3.0f;
			else                   // colliding soon
			{
				double t = (fabs(rdPY) - minDY) / fabs(rdVY);
				double collX = rdPX + rdVX * t;
				if (collX > minDX*0.5 && collX < minDX)
					collspeed = (float)oVX - 3.0f;
			}
		}
		else
			collspeed = (float)oVX;
		//collspeed = MAX(ospeed, mycar->_speed_x - (speedDiff * ((currentEk - (timpact * 100000 / brake_coefficient)) / (currentEk - targetEk))));
	}
	else
		collspeed = mycar->_speed_x + 2.0f;

	return collspeed;
}

int Opponent::testLinearCollision(Driver *driver, double ti, double speed_difference, double catch_distance, double multiplier)
{
    if (mycar->_speed_x < 15 || true) return testLinearCollision2(driver);
#if 0
    double o_accel_X = (car->_speed_X - prev_speed_X) * deltamult;
    double o_accel_Y = (car->_speed_Y - prev_speed_Y) * deltamult;
    double d_accel_X = (mycar->_speed_X - d_prev_speed_X) * deltamult;
    double d_accel_Y = (mycar->_speed_Y - d_prev_speed_Y) * deltamult;
    double velX = (mycar->_speed_X + d_accel_X) - (car->_speed_X + o_accel_X);
    double velY = (mycar->_speed_Y + d_accel_Y) - (car->_speed_Y + o_accel_Y);
    speed_difference = Mag(velX, velY);
    //speed_difference = MAX(1.0, speed_difference);
    double brake_coefficient = driver->getBrakeCoefficient();
    double brake_distance = brake_coefficient * (speed_difference * speed_difference);
    //if (brake_distance < catch_distance - 2.0 && speed_difference < 2.0)
     //   return 0;

    tCarElt *mycar = driver->getCarPtr();
    int i;
    tCarElt *dcar = driver->getCarPtr();
    double o_speedX = car->_speed_X;
    double o_speedY = car->_speed_Y;
    double d_speedX = dcar->_speed_X;
    double d_speedY = dcar->_speed_Y;

    ti *= 1.0;

    tPosd o_cur[4], d_cur[4], o_new[4], d_new[4], o_new2[4], d_new2[4];

    // set up car current positions
    for (i = 0; i < 4; i++)
    {
        o_cur[i].ax = car->_corner_x(i);
        o_cur[i].ay = car->_corner_y(i);
        d_cur[i].ax = dcar->_corner_x(i);
        d_cur[i].ay = dcar->_corner_y(i);
    }

    //tPosd carCenter, ocarCenter;
    //centerPoint(&d_cur[0], &d_cur[1], &d_cur[2], &d_cur[3], &carCenter);
    //centerPoint(&o_cur[0], &o_cur[1], &o_cur[2], &o_cur[3], &ocarCenter);

    //double angleFromOCarToCar = AngleBetweenPoints(&carCenter, &ocarCenter);

    double sizefactor = MAX(0.1, speed_difference*speed_difference/40);
    //if (dcar->_speed_x < 7.0)
    //    sizefactor = 0.1;
    double rear_lft_ax = d_cur[REAR_LFT].ax + dcar->_speed_X*ti;
    double rear_lft_ay = d_cur[REAR_LFT].ay + dcar->_speed_Y*ti;
    double rear_rgt_ax = d_cur[REAR_RGT].ax + dcar->_speed_X*ti;
    double rear_rgt_ay = d_cur[REAR_RGT].ay + dcar->_speed_Y*ti;
    d_new[REAR_LFT].ax = d_cur[REAR_LFT].ax;
    d_new[REAR_LFT].ay = d_cur[REAR_LFT].ay;
    d_new[REAR_RGT].ax = d_cur[REAR_RGT].ax;
    d_new[REAR_RGT].ay = d_cur[REAR_RGT].ay;
    d_new[FRNT_LFT].ax = d_cur[FRNT_LFT].ax + (d_cur[FRNT_LFT].ax-d_cur[REAR_LFT].ax)*sizefactor + dcar->_speed_X*(ti);// + sizefactor/3);
    d_new[FRNT_LFT].ay = d_cur[FRNT_LFT].ay + (d_cur[FRNT_LFT].ay-d_cur[REAR_LFT].ay)*sizefactor + dcar->_speed_Y*(ti);// + sizefactor/3);
    d_new[FRNT_RGT].ax = d_cur[FRNT_RGT].ax + (d_cur[FRNT_RGT].ax-d_cur[REAR_RGT].ax)*sizefactor + dcar->_speed_X*(ti);// + sizefactor/3);
    d_new[FRNT_RGT].ay = d_cur[FRNT_RGT].ay + (d_cur[FRNT_RGT].ay-d_cur[REAR_RGT].ay)*sizefactor + dcar->_speed_Y*(ti);// + sizefactor/3);
    //d_new[FRNT_LFT].ax += (d_new[FRNT_LFT].ax-d_new[REAR_LFT].ax)*sizefactor;
    //d_new[FRNT_LFT].ay += (d_new[FRNT_LFT].ay-d_new[REAR_LFT].ay)*sizefactor;
    //d_new[FRNT_RGT].ax += (d_new[FRNT_RGT].ax-d_new[REAR_RGT].ax)*sizefactor;
    //d_new[FRNT_RGT].ay += (d_new[FRNT_RGT].ay-d_new[REAR_RGT].ay)*sizefactor;

    /*
    sizefactor = MIN(1.0, speed_difference / (multiplier * 3));
    double lft_ax = d_new[FRNT_LFT].ax, lft_ay = d_new[FRNT_LFT].ay;
    d_new[FRNT_LFT].ax += (d_new[FRNT_LFT].ax - d_new[FRNT_RGT].ax) * sizefactor;
    d_new[FRNT_LFT].ay += (d_new[FRNT_LFT].ay - d_new[FRNT_RGT].ay) * sizefactor;
    d_new[FRNT_RGT].ax += (d_new[FRNT_RGT].ax - lft_ax) * sizefactor;
    d_new[FRNT_RGT].ay += (d_new[FRNT_RGT].ay - lft_ay) * sizefactor;
    */

    o_new[REAR_LFT].ax = o_cur[REAR_LFT].ax + car->_speed_X*ti;
    o_new[REAR_LFT].ay = o_cur[REAR_LFT].ay + car->_speed_Y*ti;
    o_new[REAR_RGT].ax = o_cur[REAR_RGT].ax + car->_speed_X*ti;
    o_new[REAR_RGT].ay = o_cur[REAR_RGT].ay + car->_speed_Y*ti;
    o_new[FRNT_LFT].ax = o_cur[FRNT_LFT].ax + car->_speed_X*ti;
    o_new[FRNT_LFT].ay = o_cur[FRNT_LFT].ay + car->_speed_Y*ti;
    o_new[FRNT_RGT].ax = o_cur[FRNT_RGT].ax + car->_speed_X*ti;
    o_new[FRNT_RGT].ay = o_cur[FRNT_RGT].ay + car->_speed_Y*ti;

#if 1
    {
        // the faster the speed difference the larger the opponent car is
        double sizefactor = MAX(0.5, speed_difference/7);
        double o_rear_lft_ax = o_new[REAR_LFT].ax, o_rear_lft_ay = o_new[REAR_LFT].ay;
        double o_rear_rgt_ax = o_new[REAR_RGT].ax, o_rear_rgt_ay = o_new[REAR_RGT].ay;
        o_new[REAR_LFT].ax += (o_new[REAR_LFT].ax - o_new[FRNT_RGT].ax) * brake_multiplier * sizefactor;
        o_new[REAR_LFT].ay += (o_new[REAR_LFT].ay - o_new[FRNT_RGT].ay) * brake_multiplier * sizefactor;
        o_new[REAR_RGT].ax += (o_new[REAR_RGT].ax - o_new[FRNT_LFT].ax) * brake_multiplier * sizefactor;
        o_new[REAR_RGT].ay += (o_new[REAR_RGT].ay - o_new[FRNT_LFT].ay) * brake_multiplier * sizefactor;
        o_new[FRNT_LFT].ax += (o_new[FRNT_LFT].ax - o_rear_rgt_ax) * brake_multiplier * sizefactor;
        o_new[FRNT_LFT].ay += (o_new[FRNT_LFT].ay - o_rear_rgt_ay) * brake_multiplier * sizefactor;
        o_new[FRNT_RGT].ax += (o_new[FRNT_RGT].ax - o_rear_lft_ax) * brake_multiplier * sizefactor;
        o_new[FRNT_RGT].ay += (o_new[FRNT_RGT].ay - o_rear_lft_ay) * brake_multiplier * sizefactor;
        /*
        if (fabs(relativeangle) < 0.7)
        {
            o_new[REAR_LFT].ax += (o_new[REAR_LFT].ax - o_new[FRNT_LFT].ax) * MAX(0.3, sizefactor);
            o_new[REAR_LFT].ay += (o_new[REAR_LFT].ay - o_new[FRNT_LFT].ay) * MAX(0.3, sizefactor);
            o_new[REAR_RGT].ax += (o_new[REAR_RGT].ax - o_new[FRNT_RGT].ax) * MAX(0.3, sizefactor);
            o_new[REAR_RGT].ay += (o_new[REAR_RGT].ay - o_new[FRNT_RGT].ay) * MAX(0.3, sizefactor);
        }
        else if (fabs(relativeangle) > 1.45)
        {
            o_new[FRNT_LFT].ax += (o_new[FRNT_LFT].ax - o_new[REAR_LFT].ax) * MAX(0.3, sizefactor);
            o_new[FRNT_LFT].ay += (o_new[FRNT_LFT].ay - o_new[REAR_LFT].ay) * MAX(0.3, sizefactor);
            o_new[FRNT_RGT].ax += (o_new[FRNT_RGT].ax - o_new[REAR_RGT].ax) * MAX(0.3, sizefactor);
            o_new[FRNT_RGT].ay += (o_new[FRNT_RGT].ay - o_new[REAR_RGT].ay) * MAX(0.3, sizefactor);
        }
        else if (relativeangle < 0.0)
        {
            o_new[REAR_RGT].ax += (o_new[REAR_RGT].ax - o_new[REAR_LFT].ax) * MAX(0.3, sizefactor);
            o_new[REAR_RGT].ay += (o_new[REAR_RGT].ay - o_new[REAR_LFT].ay) * MAX(0.3, sizefactor);
            o_new[FRNT_RGT].ax += (o_new[FRNT_RGT].ax - o_new[FRNT_LFT].ax) * MAX(0.3, sizefactor);
            o_new[FRNT_RGT].ay += (o_new[FRNT_RGT].ay - o_new[FRNT_LFT].ay) * MAX(0.3, sizefactor);
        }
        else
        {
            o_new[REAR_LFT].ax += (o_new[REAR_LFT].ax - o_new[REAR_RGT].ax) * MAX(0.3, sizefactor);
            o_new[REAR_LFT].ay += (o_new[REAR_LFT].ay - o_new[REAR_RGT].ay) * MAX(0.3, sizefactor);
            o_new[FRNT_LFT].ax += (o_new[FRNT_LFT].ax - o_new[FRNT_RGT].ax) * MAX(0.3, sizefactor);
            o_new[FRNT_LFT].ay += (o_new[FRNT_LFT].ay - o_new[FRNT_RGT].ay) * MAX(0.3, sizefactor);
        }
        */
    }
#endif

    // test for collision
    if (polyOverlap(o_new, d_new))
    {
        if (brake_distance > catch_distance + 1.0)
            return 2;
        return 1;
    }
    d_new[REAR_LFT].ax = rear_lft_ax;
    d_new[REAR_LFT].ay = rear_lft_ay;
    d_new[REAR_RGT].ax = rear_rgt_ax;
    d_new[REAR_RGT].ay = rear_rgt_ay;
    if (polyOverlap(o_new, d_new))
    {
        if (brake_distance > catch_distance + 1.0)
            return 2;
        return 1;
    }

#endif // 0
    return 0;
}

int Opponent::testQuadraticCollision(Driver *driver)
{
    int collision = -1;
	if (!(car->_state & RM_CAR_STATE_PIT) || fabs(car->_trkPos.toLeft - mycar->_trkPos.toLeft) < 3.0)
	{
		collision = testLinearCollision2(driver);
		if (collision <= 0) return 0;
		if ((collision & OPP_COLL))
			return collision;
		return testCalculatedCollision(driver);
	}
	return 0;
#if 0
    // quadratics from mouse/sieger ... except I can't get them to work :/
    double d_Spd = Mag(mycar->_speed_X, mycar->_speed_Y);
    double o_Spd = Mag(car->_speed_X, car->_speed_Y);
    double d_DirX = mycar->_speed_X / d_Spd;
    double d_DirY = mycar->_speed_Y / d_Spd;
    double o_DirX = car->_speed_X / o_Spd;
    double o_DirY = car->_speed_Y / o_Spd;

    double d_ragAX = d_DirX * driver->average_AX + d_DirY * driver->average_AY;
    double o_ragAX = o_DirX * average_AX + o_DirY * average_AY;
    double d_ragAY = d_DirY * driver->average_AX - d_DirX * driver->average_AY;
    double o_ragAY = o_DirY * average_AX - o_DirX * average_AY;

    double dPX = car->pub.DynGCg.pos.x - mycar->pub.DynGCg.pos.x;
    double dPY = car->pub.DynGCg.pos.y - mycar->pub.DynGCg.pos.y;
    double dVX  = car->_speed_X - mycar->_speed_X;
    double dVY  = car->_speed_Y - mycar->_speed_Y;

    double o_rdPX = d_DirX * dPX + d_DirY * dPY;
    double o_rdPY = d_DirY * dPX - d_DirX * dPY;
    double o_rdVX = d_DirX * dVX + d_DirY * dVY;
    double o_rdVY = d_DirY * dVX - d_DirX * dVY;

    double minDX = (car->_dimension_x + mycar->_dimension_x) / 2;
    double minDY = (car->_dimension_y + mycar->_dimension_y) / 2;

    Quadratic myPar(0, 0, 0, d_ragAY);
    Quadratic oPar(0, o_rdPY, o_rdVY, o_ragAY);
    Quadratic relPar = oPar - myPar;

    double acc = o_ragAX;
    Quadratic q(acc/2, o_rdVX, o_rdPX - minDX);
    double t = 0.0;
    if (q.SmallestNonNegativeRoot(t))
    {
        double catchY = relPar.CalcY(t);
        if (fabs(catchY) < minDY)
            collision = 1;
        else
        {
            q.Setup( acc/2, o_rdVX, o_rdPX + minDX );
            if (q.SmallestNonNegativeRoot(t))
            {
                catchY = relPar.CalcY(t);
                if (fabs(catchY) < minDY || catchY * o_rdPY < 0)
                    collision = 1;
            }
        }
    }

    return collision;
#endif
}

int Opponent::testCalculatedCollision(Driver *driver)
{
	double future_left, future_right;

	tPosd o_cur[4], d_cur[4];
    for (int i = 0; i < 4; i++)
    {
        o_cur[i].ax = car->_corner_x(i);
        o_cur[i].ay = car->_corner_y(i);
        d_cur[i].ax = mycar->_corner_x(i);
        d_cur[i].ay = mycar->_corner_y(i);
    }

	double dist = (tdble)findDistanceBetweenCars(o_cur, d_cur);
    t_impact = MAX(-1.0, (dist - 1.0) / (mycar->_speed_x - oppSpeed));

	collspeed = (float)CalcCollSpeed(driver);

	if (t_impact >= 0.0 && dist < 1.0)
	{
		// close collision
		future_left = left_toMid;
		future_right = right_toMid;
        collspeed = (float) MIN(collspeed, car->_speed_x-2.0);
	}
	else if (t_impact > 0.0 && t_impact < 10.0)
	{
		future_left = left_toMid + left_speed_y * t_impact;
		future_right = right_toMid + right_speed_y * t_impact;
	}
	else
		return 0;

	// ignore if totally off the track
	if (future_right > track->width/2 + 0.5 || future_left < -(track->width/2 + 0.5))
		return 0;
		
	double my_future_left = driver->futureLeftToMid(t_impact);
	double my_future_right = driver->futureRightToMid(t_impact);

	if ((brakedistance > t_impact * 2.0) &&
		((my_future_right < future_left && my_future_left >= future_right) ||
		 (my_future_left > future_right && my_future_right <= future_left)))
	{
#ifdef BRAKE_DEBUG
		fprintf(stderr, "%s CCOLL %s: myrgt=%.1f mylft=%.1f oprgt=%.1f (%.1f) oplft=%.1f (%.1f) t_imp=%.3f ospd=%.2f/%.2f\n",car->_name,mycar->_name,my_future_right,my_future_left,future_right,right_toMid,future_left,left_toMid,t_impact,right_speed_y,left_speed_y); fflush(stderr);
#endif
		return OPP_COLL;
	}
#ifdef BRAKE_DEBUG
	fprintf(stderr, "%s NO CCOLL %s: myrgt=%.1f mylft=%.1f oprgt=%.1f (%.1f) oplft=%.1f (%.1f) t_imp=%.3f ospd=%.2f/%.2f\n",car->_name,mycar->_name,my_future_right,my_future_left,future_right,right_toMid,future_left,left_toMid,t_impact,right_speed_y,left_speed_y); fflush(stderr);
#endif

	return testRacelineCollision(driver, dist, t_impact, future_left, future_right);
}

static void centerPoint(tPosd *p1, tPosd *p2, tPosd *p3, tPosd *p4, tPosd *pt)
{
    pt->ax = (p1->ax + p2->ax + p3->ax + p4->ax) / 4;
    pt->ay = (p1->ay + p2->ay + p3->ay + p4->ay) / 4;
}

static double AngleBetweenPoints(tPosd *target, tPosd *origin)
{
    return atan2(target->ay - origin->ay, target->ax - origin->ax);
}

static void FindPointAlongAngle(tPosd *origin, double angle, double distance, tPosd *target)
{
    tdble deltax = (tdble)(distance * cos(angle));
    tdble deltay = (tdble)(distance * sin(angle));
    target->ax = origin->ax + deltax;
    target->ay = origin->ay + deltay;
}

static double sign(tPosd *p1, tPosd *p2, tPosd *p3)
{
    return (p1->ax - p3->ax) * (p2->ay - p3->ay) - (p2->ax - p3->ax) * (p1->ay - p3->ay);
}

static int pointInTriangle(tPosd *pt, tPosd *v1, tPosd *v2, tPosd *v3)
{
#if 0
    // Barycentric method
    double s = v1->ay * v3->ax - v1->ax * v3->ay + (v3->ay - v1->ay) * pt->ax + (v1->ax - v3->ax) * pt->ay;
    double t = v1->ax * v2->ay - v1->ay * v2->ax + (v1->ay - v2->ay) * pt->ax + (v2->ax - v1->ax) * pt->ay;

    if ((s < 0) != (t < 0))
        return 0;

    double A = -v2->ay * v3->ax + v1->ay * (v3->ax - v2->ax) + v1->ax * (v2->ay - v3->ay) + v2->ax * v3->ay;
    if (A < 0.0)
    {
        s = -s;
        t = -t;
        A = -A;
    }
    return s > 0 && t > 0 && (s + t) < A;
#else
    int i1, i2, i3;

    i1 = (sign(pt, v1, v2) < 0.0);
    i2 = (sign(pt, v2, v3) < 0.0);
    i3 = (sign(pt, v3, v1) < 0.0);

    return ((i1 == i2) && (i2 == i3));
#endif
}

int Opponent::testRacelineCollision(Driver *driver, double distance, double timpact, double future_left, double future_right)
{
	if (driver->linemode->IsTransitioning()) return 0;

    int coll = 0;
    LRaceLine *raceline = driver->getRaceLine();
	int div = raceline->DivIndexForCar(car); // , timpact);

	double rl_toLeft = raceline->OfflineLane(div, driver->linemode->GetLeftCurrentMargin(), driver->linemode->GetRightCurrentMargin()) * track->width;
	if (future_left < -900)
	{
		double o_toLeft = car->_trkPos.toLeft;
		if (fabs(o_toLeft - rl_toLeft) < car->_dimension_y + 0.5)
		{
#ifdef BRAKE_DEBUG
			fprintf(stderr, "%s => %s RCOLL A\n", mycar->_name, car->_name); fflush(stderr);
#endif
			return OPP_COLL;
		}
	}
	else
	{
		double my_toMid = track->width - (rl_toLeft)-track->width / 2;
		double my_future_right = my_toMid - car->_dimension_y / 2 - 0.5;
		double my_future_left = my_toMid + car->_dimension_y / 2 + 0.5;

		if ((my_future_right < future_left && my_future_left >= future_right) ||
			(my_future_left > future_right && my_future_right <= future_left))
		{
#ifdef BRAKE_DEBUG
			fprintf(stderr, "%s => %s RCOLL B\n", mycar->_name, car->_name); fflush(stderr);
#endif
			return OPP_COLL;
		}
	}
#if 0
    if (car->_trkPos.toLeft >= 0.0)
        o_toLeft = MAX(o_toLeft, 0.0);
    if (car->_trkPos.toRight <= track->width)
        o_toLeft = MIN(o_toLeft, track->width);
    double speedDiff = distance / timpact;
    double factor = 1.0;

    if (driver->currentRaceline() >= LINE_RL)
    {
        double lane = raceline->tLane[rl][div];
        double lane2left = lane * track->width;
        if (distance < 5.0 && (lane2left < o_toLeft && raceline->tRInverse[rl][raceline->Next] > 0.005) || (lane2left > o_toLeft && raceline->tRInverse[rl][raceline->Next] < -0.005))
            factor += fabs(raceline->tRInverse[rl][raceline->Next]) * 20;

        if (distance > 2.0 && timpact < 3.0 && fabs(o_toLeft - lane2left) < 2.0 * factor - MIN((15 - MIN(10.0, (distance-10)/4)) / 5, MAX(0, 2 - speedDiff/10)))
            return OPP_COLL | OPP_COLL_WARNING | OPP_RACELINE_CONFLICT;
        else if (fabs(o_toLeft - lane2left) < 6.0 && timpact < 5.0)
            return OPP_COLL_WARNING;
    }
    
    {
        double m_toLeft = mycar->_trkPos.toLeft + (driver->avgLateralMovt * timpact * deltamult);
        if (mycar->_trkPos.toLeft >= 0.0)
            m_toLeft = MAX(m_toLeft, 0.0);
        if (mycar->_trkPos.toRight <= track->width)
            m_toLeft = MIN(m_toLeft, track->width);
        if (distance < 5.0 && ((m_toLeft < o_toLeft && raceline->tRInverse[LINE_RL][raceline->Next] > 0.005) || (m_toLeft > o_toLeft && raceline->tRInverse[LINE_RL][raceline->Next] < -0.005)))
            factor += fabs(raceline->tRInverse[LINE_RL][raceline->Next]) * 20;

        if (distance > 2.0 && timpact < 3.0 && fabs(o_toLeft - m_toLeft) < 2.0 * factor - MIN((15 - MIN(10.0, (distance-10)/4)) / 5, MAX(0, 2 - speedDiff/10)))
            return OPP_COLL | OPP_COLL_WARNING | OPP_COLL_LINEAR;
        else if (fabs(o_toLeft - m_toLeft) < 6.0 && timpact < 5.0)
            return OPP_COLL_WARNING;
    }
#endif

    return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// OPPONENTS CLASS
///////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize the list of opponents.
Opponents::Opponents(tSituation *s, Driver *driver, Cardata *c, double brake_multiplier, double brake_warn_multiplier)
{
    opponent = new Opponent[s->_ncars - 1];
    int i, j = 0;
    for (i = 0; i < s->_ncars; i++) {
        if (s->cars[i] != driver->getCarPtr()) {
            opponent[j].setCarPtr(s->cars[i]);
            opponent[j].setCarDataPtr(c->findCar(s->cars[i]));
            opponent[j].setIndex(i);
            opponent[j].brake_multiplier = brake_multiplier;
            opponent[j].brake_warn_multiplier = brake_warn_multiplier;
            j++;
        }
    }
    Opponent::setTrackPtr(driver->getTrackPtr());
    nopponents = s->_ncars - 1;
}


Opponents::~Opponents()
{
    delete[] opponent;
}


void Opponents::update(tSituation *s, Driver *driver)
{
    int i;
    for (i = 0; i < s->_ncars - 1; i++) {
        opponent[i].update(s, driver);
    }
}


void Opponents::setTeamMate(const char *teamname)
{
    int i;
    for (i = 0; i < nopponents; i++) {
        if (strcmp(opponent[i].getCarPtr()->_teamname, teamname) == 0) {
            opponent[i].markAsTeamMate();
            break;    // Name should be unique, so we can stop.
        }
    }
}

tCarElt *Opponents::getTeamMateCar()
{
    for (int i=0; i<nopponents; i++)
    {
        if (opponent[i].isTeamMate())
            return opponent[i].getCarPtr();
    }
    return NULL;
}

bool Opponents::isBehindFriend(int pos)
{
	for (int i = 0; i < nopponents; i++)
	{
		if (opponent[i].isTeamMate() && opponent[i].getCarPtr()->_pos < pos)
			return true;
	}
	return false;
}




