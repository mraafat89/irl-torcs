/***************************************************************************

    file                 : linemode.cpp
    created              : Sat Feb 07 19:53:00 CET 2015
    copyright            : (C) 2015 by Andrew Sumner
    email                : novocas7rian@gmail.com
    version              : $Id: linemode.cpp,v 1.0 2015/02/07 20:11:49 andrew Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <iostream>
#include <math.h>
#include "car.h"
#include "track.h"
#include "linemode.h"
#include "raceline.h"
#include "xmldefs.h"

//#define LINEMODE_DEBUG

//
// Public methods
//
LLineMode::LLineMode(tCarElt *theCar, LManualOverrideCollection *oc, double trackWidth, LRaceLine *raceline)
{
	m_IsTransitioning = true;
	m_HasTransitioned = false;
	m_UnderThreat = false;
	m_OverrideCollection = oc;
	m_Car = theCar;
	m_Raceline = raceline;
	m_TimerStarted = m_CurrentTime = 0.0;
	m_HoldApexDiv = -1;
	m_PreferLine = LINE_MID;
	m_TargetRaceline = LINE_MID;
	m_TransitionIncrement = GfParmGetNum(m_Car->_carHandle, SECT_PRIVATE, PRV_TRANSITION_INC, (char *) NULL, 0.14f);
	m_EdgeMargin = GfParmGetNum(m_Car->_carHandle, SECT_PRIVATE, PRV_EDGE_LINE_MARGIN, (char *) NULL, 0.15f);
	m_LeftCurrentMargin = m_LeftPredictMargin = m_Car->_trkPos.toLeft / trackWidth;
	m_RightCurrentMargin = m_LeftPredictMargin = m_Car->_trkPos.toLeft / trackWidth;
	m_LeftTargetMargin = m_PrevLeftTargetMargin = m_Car->_trkPos.toLeft / trackWidth;
	m_RightTargetMargin = m_PrevRightTargetMargin = m_Car->_trkPos.toLeft / trackWidth;
	m_OnHold = false;
	m_TrackWidth = trackWidth;
}

LLineMode::~LLineMode()
{
}

double LLineMode::GetTransitionIncrement(int div)
{
	double ti = m_TransitionIncrement;
	/*
	if (fabs(m_Car->_accel_x) > 3.0)
	{
		double tiFactor1 = MIN(8.0, fabs(m_Car->_accel_x));
		tiFactor1 *= tiFactor1 / 2;
		double tiFactor2 = MAX(0.0, MIN(1.0, fabs(m_Car->ctrl.steer)) - 0.5);
		ti *= MAX(0.1, MIN(1.0 - tiFactor1 / 32.0, 1.0 - tiFactor2 / 0.5));
	}
	*/
    if (m_OverrideCollection)
    {
        LManualOverride *labelOverride = m_OverrideCollection->getOverrideForLabel(PRV_TRANSITION_INC);
        if (labelOverride)
        {
			double tti = ti;
			if (labelOverride->getOverrideValue(div, &tti))
				ti = tti;
        }
    }
    if (m_CurrentTime - m_TimerStarted < 0.5)
        return MAX(0.0, ti * ((m_CurrentTime - m_TimerStarted) / 0.5));
    return ti;
}

void LLineMode::UpdateSituation(tSituation *s, bool underThreat)
{
    m_CurrentTime = s->currentTime;
	m_UnderThreat = underThreat;
#ifdef LINEMODE_DEBUG
//    fprintf(stderr, "LineMode: Source=%s Target=%s hold_div=%d / %d\n",(source_line==LINE_NONE?"None":(source_line==LINE_LEFT?"Left":(source_line==LINE_RIGHT?"Right":"RL"))),(target_line==LINE_NONE?"None":(target_line==LINE_LEFT?"Left":(target_line==LINE_RIGHT?"Right":"RL"))),hold_apex_div,(raceline?raceline->This:-1));fflush(stderr);
#endif
}

bool LLineMode::IsOnHold(int div, bool stay_inside)
{
    //return false;
    if (m_IsTransitioning && m_LeftTargetMargin == 0.0 && m_RightTargetMargin == 1.0 && m_UnderThreat)
    { 
        double distance = 0.0;
        int hold_div = m_HoldApexDiv;
        int this_prefer_line = m_Raceline->findNextCorner(m_Car, m_Raceline->This, &hold_div, &distance);
        if (distance > m_Car->_speed_x * 5 || hold_div < 0 ||
            (m_HoldApexDiv > 0 && hold_div != m_HoldApexDiv && m_PreferLine != this_prefer_line))
        {
            m_OnHold = false;
			m_HoldApexDiv = -1;
#ifdef LINEMODE_DEBUG
			fprintf(stderr, "%s: NO HOLD (A %d %d %d) = dist (%.1f > %.1f) || div %d != %d && line %s != %s\n", m_Car->_name, (distance > m_Car->_speed_x * 5), (m_HoldApexDiv < 0), (hold_div > 0 && hold_div != m_HoldApexDiv && m_PreferLine != this_prefer_line), distance, m_Car->_speed_x * 5, m_HoldApexDiv, hold_div, m_PreferLine == TR_STR ? "STR" : m_PreferLine == TR_RGT ? "RGT" : "LFT", this_prefer_line == TR_STR ? "STR" : this_prefer_line == TR_RGT ? "RGT" : "LFT"); fflush(stderr);
#endif
            return false;
        }
        if ((this_prefer_line == TR_LFT && (m_LeftCurrentMargin > 0.001 || m_RightCurrentMargin == 1.0)) ||
			(this_prefer_line == TR_RGT && (m_LeftCurrentMargin < 0.001 || m_RightCurrentMargin < 1.0)))
        {
            m_OnHold = false;
			m_HoldApexDiv = -1;
#ifdef LINEMODE_DEBUG
			fprintf(stderr,"%s: NO HOLD (2)\n",m_Car->_name);fflush(stderr);
#endif
            return false;
        }
        if ((this_prefer_line == TR_LFT && m_RightCurrentMargin <= 0.83 && m_LeftCurrentMargin == 0.0) || 
			(this_prefer_line == TR_RGT && m_LeftCurrentMargin >= 0.17 && m_RightCurrentMargin == 1.0))
        {
            ResetTimer();
			m_HoldApexDiv = hold_div;
#ifdef LINEMODE_DEBUG
			fprintf(stderr,"%s: HOLDING!!\n",m_Car->_name);fflush(stderr);
#endif
            return true;
        }
#ifdef LINEMODE_DEBUG
		fprintf(stderr, "%s: NO HOLD (C) line=%s, lftMarg=%.3f rgtMarg=%.3f\n", m_Car->_name, this_prefer_line == TR_STR ? "STR" : this_prefer_line == TR_RGT ? "RGT" : "LFT",m_LeftCurrentMargin,m_RightCurrentMargin); fflush(stderr);
#endif
    
    }
#ifdef LINEMODE_DEBUG
	else
	{
		fprintf(stderr, "%s: NO HOLD (D), threat=%d\n", m_Car->_name, m_UnderThreat); fflush(stderr);
	}
#endif
    m_HoldApexDiv = -1;
    m_OnHold = false;
    return false;
}

bool LLineMode::IsOnRaceLine()
{
	if ((m_LeftTargetMargin == 0.0 && (m_RightTargetMargin == 1.0 || m_RightTargetMargin == m_EdgeMargin)) ||
		(m_LeftTargetMargin == 1.0 - m_EdgeMargin && m_RightTargetMargin == 1.0))
		return true;
	return false;
}

void LLineMode::ApplyTransition(int div, double factor, double rInverse, float steer)
{
	if (!m_IsTransitioning)
		return;
	if (m_LeftTargetMargin == 0.0 && m_RightTargetMargin == 1.0 && IsOnHold(div, true))
		return;

	double change = (1.0 / m_TrackWidth) * GetTransitionIncrement(div) * factor;

	double oldLeftCurrentMargin = m_LeftCurrentMargin;
	double oldRightCurrentMargin = m_RightCurrentMargin;

	if (m_LeftTargetMargin < m_LeftCurrentMargin)
		m_LeftCurrentMargin = MAX(m_LeftTargetMargin, m_LeftCurrentMargin - change);
	else if (m_LeftTargetMargin > m_LeftCurrentMargin)
		m_LeftCurrentMargin = MIN(m_LeftTargetMargin, m_LeftCurrentMargin + change);

	if (m_RightTargetMargin < m_RightCurrentMargin)
		m_RightCurrentMargin = MAX(m_RightTargetMargin, m_RightCurrentMargin - change);
	else if (m_RightTargetMargin > m_RightCurrentMargin)
		m_RightCurrentMargin = MIN(m_RightTargetMargin, m_RightCurrentMargin + change);

	m_LeftPredictMargin = MAX(0.0, MIN(1.0 - m_EdgeMargin, m_LeftCurrentMargin + (m_LeftCurrentMargin - oldLeftCurrentMargin)));
	m_RightPredictMargin = MAX(m_EdgeMargin, MIN(1.0, m_RightCurrentMargin + (m_RightCurrentMargin - oldRightCurrentMargin)));

	if (fabs(rInverse) < 0.001 && fabs(steer) < 0.1f &&
		m_LeftCurrentMargin == m_LeftTargetMargin && m_RightCurrentMargin == m_RightTargetMargin)
	{
		m_IsTransitioning = false;
	}
#if 0
	else
	{
		fprintf(stderr, "Still Transitioning: %.4f > 0.001, %.2f >= 0.1, %.4f != %.4f, %.4f != %.4f (%d %d %d %d)\n", fabs(rInverse), fabs(steer), m_LeftCurrentMargin, m_LeftTargetMargin, m_RightCurrentMargin, m_RightTargetMargin, fabs(rInverse) < 0.001, fabs(steer) < 0.1f, m_LeftCurrentMargin == m_LeftTargetMargin, m_RightCurrentMargin == m_RightTargetMargin);
	}
#endif
}

void LLineMode::SetTargetMargins(int raceline, double leftMargin, double rightMargin, bool force)
{
	bool change = false;

	m_TargetRaceline = raceline;

	leftMargin = MIN(leftMargin, 1.0 - m_EdgeMargin);
	rightMargin = MAX(rightMargin, m_EdgeMargin);

	if (leftMargin != m_LeftTargetMargin)
	{
		m_PrevLeftTargetMargin = m_LeftTargetMargin;
		m_LeftTargetMargin = leftMargin;
		change = true;
	}
	if (rightMargin != m_RightTargetMargin)
	{
		m_PrevRightTargetMargin = m_RightTargetMargin;
		m_RightTargetMargin = rightMargin;
		change = true;
	}

	if (change)
	{
		double leftChange = fabs(m_PrevLeftTargetMargin - m_LeftTargetMargin), rightChange = fabs(m_PrevRightTargetMargin - m_RightTargetMargin);
		if (MAX(leftChange, rightChange) > 2.0)
			ResetTimer();
		if (m_LeftTargetMargin != 0.0 || m_RightTargetMargin != 1.0)
			m_HoldApexDiv = -1;
		//fprintf(stderr, "New Margins: %.3f %.3f (%d)\n", m_LeftTargetMargin, m_RightTargetMargin,force);
		m_IsTransitioning = true;
	}
}

void LLineMode::SetRecoverToRaceLine()
{
#ifdef LINEMODE_DEBUG
    fprintf(stderr, "setRecoveryToRaceLine\n");fflush(stderr);
#endif
    if (m_LeftTargetMargin != 0.0 || m_RightTargetMargin != 1.0)
    {
        m_HoldApexDiv = -1;
        int prefer_line = m_Raceline->findNextCorner(m_Car, m_Raceline->This, &m_HoldApexDiv);
        if ((prefer_line == TR_LFT && m_RightTargetMargin <= 0.7) || (prefer_line == TR_RGT && m_LeftTargetMargin >= 0.3))
        {
            if (m_HoldApexDiv > -1)
            {
                if (m_HoldApexDiv < m_Raceline->This)
                    m_HoldApexDiv += m_Raceline->Divs;
                double distance = (m_HoldApexDiv - m_Raceline->This) * 3.0;
                if (distance < m_Car->_speed_x * 5)
                {
                    if (distance > 6.0 || m_Raceline->tSpeed[LINE_RL][m_Raceline->Next] <= m_Raceline->tSpeed[LINE_RL][m_Raceline->This])
                    {
//fprintf(stderr, "DEFERRED CORRECTION TO RL!\n");
                        return;
                    }
                }
            }
        }
    }

	SetTargetMargins(LINE_RL, 0.0, 1.0);
}

void LLineMode::SetPitting()
{
    m_HoldApexDiv = -1;
    ResetTimer();
	SetTargetMargins(LINE_MID, m_Car->_trkPos.toLeft / m_TrackWidth, m_Car->_trkPos.toLeft / m_TrackWidth, true);
    m_IsTransitioning = true;
}

//
// Private methods
//
void LLineMode::ResetTimer()
{
    m_TimerStarted = m_CurrentTime;
    //prefer_line = LINE_MID;
}
