/***************************************************************************

    file        : Seg.h
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

// Seg.h: interface for the Seg class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_SEG_H__4BC3412A_2FA2_4240_B62C_5A65660A76D7__INCLUDED_)
#define AFX_SEG_H__4BC3412A_2FA2_4240_B62C_5A65660A76D7__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <track.h>
#include "Vec3d.h"

class Seg  
{
public:
	Seg();
	~Seg();

public:
	double		segDist;
	tTrackSeg*	pSeg;		// main track segment.
	double		wl;			// width to left.
	double		wr;			// width to right.
	double		el;			// extent to left.
	double		er;			// extent to right.
	double		midOffs;	// offset to "mid" (nominal centre -- e.g. pitlane)
	double		t;			// relative position of pt within trackSeg [0..1]
	Vec3d		pt;			// centre point.
	Vec3d		norm;		// normal left to right (unit vector in xy, slope in z).
	int			bendId;		// id of bend this seg is a part of.
};

#endif // !defined(AFX_SEG_H__4BC3412A_2FA2_4240_B62C_5A65660A76D7__INCLUDED_)
