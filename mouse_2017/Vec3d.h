/***************************************************************************

    file        : Vec3d.h
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

// Vec3d.h: interface for the Vec3d class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_VEC3D_H__4801AC0B_D727_469E_9260_87DC0EA58243__INCLUDED_)
#define AFX_VEC3D_H__4801AC0B_D727_469E_9260_87DC0EA58243__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <tmath/v3_t.h>
#include <tgf.h>
#include "Vec2d.h"

class Vec3d : public v3t<double>
{
public:
	Vec3d() {}
	Vec3d( const v3t<double>& v ) : v3t<double>(v) {}
	Vec3d( double x, double y, double z ) : v3t<double>(x, y, z) {};
	Vec3d( const t3Dd& v ) : v3t<double>(v.x, v.y, v.z) {}

	Vec3d&	operator=( const v3t<double>& v )
	{
		v3t<double>::operator=(v);
		return *this;
	}

	Vec2d	GetXY() const { return Vec2d(x, y); }
};

#endif // !defined(AFX_VEC3D_H__4801AC0B_D727_469E_9260_87DC0EA58243__INCLUDED_)
