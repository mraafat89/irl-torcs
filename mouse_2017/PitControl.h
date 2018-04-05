/***************************************************************************

    file        : PitControl.h
    created     : 18 Apr 2017
    copyright   : (C) 2017 Tim Foden

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// PitControl.h: interface for the PitControl class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PITCONTROL_H__81207FE5_EA2F_4DC9_B253_1625C2B60DBF__INCLUDED_)
#define AFX_PITCONTROL_H__81207FE5_EA2F_4DC9_B253_1625C2B60DBF__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "MyTrack.h"
#include "PitPath.h"
#include "TeamInfo.h"

class PitControl  
{
public:
	enum
	{
		PT_NORMAL,
		PT_DRIVE_THROUGH,
	};

public:
	PitControl( const MyTrack& track, const PitPath& pitPath );
	~PitControl();

	void	SetDamageLimits( int warnDamage, int dangerDamage );
	void	Process( CarElt* pCar, TeamInfo::Item* pMyInfo );

	bool	WantToPit() const;
	int		PitType() const;		// type of pit requested.

	double	FuelPerM( const CarElt* pCar ) const;

private:
	enum
	{
		PIT_NONE,
		PIT_ENABLED,
		PIT_ENTER,
		PIT_ASKED,
		PIT_EXIT,
	};

private:
	const MyTrack&	m_track;
	const PitPath&	m_pitPath;

	int				m_warnDamageLimit;
	int				m_dangerDamageLimit;

	int				m_state;

	double			m_lastFuel;
	double			m_totalFuel;
	double			m_lastDamage;
	double			m_totalDamage;
	double			m_lastTyreWear;
	double			m_totalTyreWear;
	int				m_lastLap;
	int				m_totalLaps;
	int				m_pitType;
};

#endif // !defined(AFX_PITCONTROL_H__81207FE5_EA2F_4DC9_B253_1625C2B60DBF__INCLUDED_)
