/***************************************************************************

    file        : Path.h
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

// Path.h: interface for the Path class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_LINEPATH_H__0A22D193_004E_40A8_8D71_E0B6F6A51171__INCLUDED_)
#define AFX_LINEPATH_H__0A22D193_004E_40A8_8D71_E0B6F6A51171__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <vector>

#include "MyTrack.h"
#include "CarModel.h"

class Path
{
public:
	struct PathPt
	{
		const Seg*	pSeg;		// track seg that contains this Seg.
		double		k;			// curvature in xy.
		double		kz;			// curvature in z direction... e.g. bumps.
		double		offs;		// offs from centre point.
		Vec3d		pt;			// actual pt (same as CalcPt())
		double      ap;         // pitch angle along the path.
		double      ar;         // roll angle perpendicular to path.
		double		maxSpd;		// max speed through this pt.
		double		spd;		// speed through this pt (braking only).
		double		accSpd;		// speed through this pt, with modelled accel.
		double		h;			// predicted height of car above track (flying).
		double		lBuf;		// buffer from left for safety.
		double		rBuf;		// buffer from right for safety.
		double		fwdK;
		bool		fixed;		// offset is fixed.

		double		Dist() const	{ return pSeg->segDist; }
		double		Wl() const		{ return pSeg->wl; }
		double		Wr() const		{ return pSeg->wr; }
		double		Extl() const	{ return pSeg->el; }
		double		Extr() const	{ return pSeg->er; }
		const Vec3d&Pt() const		{ return pSeg->pt; }
		const Vec3d&Norm() const	{ return pSeg->norm; }
		Vec3d		CalcPt() const	{ return pSeg->pt + pSeg->norm * offs; }
		Vec3d		CalcPt(double o) const	{ return pSeg->pt + pSeg->norm * o; }
	};

public:
	Path();
	virtual ~Path();

	void	Clear();
	int		Size() const { return m_pts.size(); }

	virtual Path&	operator=( const Path& path );

public:
	virtual bool	ContainsPos( double trackPos ) const;
	virtual bool	GetPtInfo( double trackPos, PtInfo& pi ) const;

	bool	GotPath() const;
	void	Initialise( MyTrack* pTrack, double maxL = 999, double maxR = 999 );

	int				GetSize() const;
	const PathPt&	GetAt( int idx ) const;
	PathPt&			GetAt( int idx );

	void	CalcCurvaturesXY( int start, int len, int step = 1 );
	void	CalcCurvaturesZ( int start, int len, int step = 1 );
	void	CalcAngles( int start, int len, int step = 1 );
	void	CalcMaxSpeeds( int start, int len, const CarModel& cm, int step = 1 );
	void	PropagateBraking( int start, int len, const CarModel& cm, int step = 1 );
	void	PropagateAcceleration( int start, int len, const CarModel& cm, int step = 1 );

	void	CalcCurvaturesXY( int step = 1 );
	void	CalcCurvaturesZ( int step = 1 );
	void	CalcAngles( int step = 1 );
	void	CalcMaxSpeeds( const CarModel& cm, int step = 1 );
	void	PropagateBraking( const CarModel& cm, int step = 1 );
	void	PropagateAcceleration( const CarModel& cm, int step = 1 );
	void	CalcFwdAbsK( int range, int step = 1 );

	void	SetOffset( const CarModel& cm, double offset, PathPt* l3 );

	void	InterpolateBetweenLinear( const CarModel& cm, int step );
	void	GenShortest( const CarModel& cm );
	void	GenMiddle();

	void	Average( const CarModel& cm );
	void	AverageSection( const CarModel& cm, int from, int len );
	void	ModifySection( int from, int len, double delta, int important, double lBuf = 1, double rBuf = 1 );
	void	QuadraticFilter( int idx );

	void	SetEstimatedTime( double time );
	double	GetEstimatedTime() const;
	
	double	CalcEstimatedTime( int start, int len ) const;
	double	CalcEstimatedLapTime() const;

    double  GetRollAngle( int idx ) const;
    double  GetPitchAngle( int idx ) const;

	// for interfacing with "springs" data.
	bool	LoadPath( const char* pDataFile );
	bool	SavePath( const char* pDataFile ) const;

//	const PathPt&	GetAt( int index ) const;

protected:
    int                 NSEG;
	MyTrack*			m_pTrack;
	std::vector<PathPt>	m_pts;

	double		m_estimatedTime;
	double		m_maxL;
	double		m_maxR;
};

#endif // !defined(AFX_LINEPATH_H__0A22D193_004E_40A8_8D71_E0B6F6A51171__INCLUDED_)
