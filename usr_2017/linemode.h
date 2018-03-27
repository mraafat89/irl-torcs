/***************************************************************************

    file                 : linemode.h
    created              : Sat Feb 07 19:53:00 CET 2015
    copyright            : (C) 2015 by Andrew Sumner
    email                : novocas7rian@gmail.com
    version              : $Id: linemode.h,v 1.0 2015/02/07 20:11:49 andrew Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef __LINEMODE_H
#define __LINEMODE_H

#include <raceman.h>
#include <tgf.h>
#include <car.h>
#include <robottools.h>
#include <robot.h>

#include "vardef.h"
#include "manual_override.h"
#include "xmldefs.h"

//enum { LINE_MID=0, LINE_LEFT, LINE_RIGHT, LINE_RL, LINE_RL_MID, LINE_RL_SLOW, LINE_LEFT_OUTSTEER, LINE_RIGHT_OUTSTEER, LINE_MID_COLD, LINE_LEFT_COLD, LINE_RIGHT_COLD, LINE_RL_COLD, LINE_RL_MID_COLD, LINE_RL_SLOW_COLD, LINE_LEFT_OUTSTEER_COLD, LINE_RIGHT_OUTSTEER_COLD, LINE_NONE };
enum { LINE_MID=0, LINE_LEFT, LINE_RIGHT, LINE_RL, LINE_RL_MID, LINE_RL_SLOW, LINE_MID_COLD, LINE_LEFT_COLD, LINE_RIGHT_COLD, LINE_RL_COLD, LINE_RL_MID_COLD, LINE_RL_SLOW_COLD, LINE_NONE };

class LRaceLine;
class LLineMode {
    public:
        LLineMode(tCarElt *thecar, LManualOverrideCollection *overrideCollection, double trackWidth, LRaceLine *raceline);
        ~LLineMode();

		void SetTargetMargins(int raceline, double leftMargin, double rightMargin, bool force = false);
		void SetRecoverToRaceLine();
		void SetMinLeftCurrentMargin(double margin) { m_LeftCurrentMargin = MIN(1.0 - m_EdgeMargin, MAX(m_LeftCurrentMargin, margin)); }
		void SetMinRightCurrentMargin(double margin) { m_RightCurrentMargin = MAX(m_EdgeMargin, MIN(m_RightCurrentMargin, margin)); }
		void SetLeftCurrentMargin(double margin) { m_LeftCurrentMargin = margin; }
		void SetRightCurrentMargin(double margin) { m_RightCurrentMargin = margin; }
        void SetPitting();
		void ApplyTransition(int div, double factor, double rInverse, float steer);
		double GetTransitionIncrement(int Div);
		double GetRightTargetMargin() { return m_RightTargetMargin; }
		double GetLeftTargetMargin() { return m_LeftTargetMargin; }
		double GetRightCurrentMargin() { return m_RightCurrentMargin; }
		double GetLeftCurrentMargin() { return m_LeftCurrentMargin; }
		double GetRightPredictMargin() { return m_RightPredictMargin; }
		double GetLeftPredictMargin() { return m_LeftPredictMargin; }
        void UpdateSituation(tSituation *s, bool underThreat);
		void SetOnHold() { 
			if (!m_OnHold) ResetTimer();
				m_OnHold = true;
		}
        double GetTransitionTime() { return m_CurrentTime - m_TimerStarted; }
        bool IsOnHold(int div, bool stay_inside);
		bool IsOnRaceLine();
		bool IsTransitioning() { return m_IsTransitioning; }
		bool SetTransitioning(bool value) { m_IsTransitioning = value; }
		bool HasFinishedTransition(double targetSteer, double steer);
		int GetTargetRaceline() { return m_TargetRaceline;  }

    private:
        bool m_IsTransitioning;
		bool m_HasTransitioned;
		bool m_UnderThreat;
		LManualOverrideCollection *m_OverrideCollection;
        tCarElt *m_Car;
        LRaceLine *m_Raceline;
        double m_TimerStarted;
		double m_CurrentTime;
		double m_EdgeMargin;
        int m_HoldApexDiv;
		int m_PreferLine;
		int m_TargetRaceline;
        double m_TransitionIncrement;
		double m_LeftCurrentMargin;
		double m_RightCurrentMargin;
		double m_LeftPredictMargin;
		double m_RightPredictMargin;
		double m_LeftTargetMargin;
		double m_RightTargetMargin;
		double m_PrevLeftTargetMargin;
		double m_PrevRightTargetMargin;
		bool m_OnHold;
		double m_TrackWidth;

        void ResetTimer();
};

#endif // __LINEMODE_H
