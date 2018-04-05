/***************************************************************************

    file        : ClothoidPath.h
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

// ClothoidPath.h: interface for the ClothoidPath class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CLOTHOIDPATH_H__E1689BA0_5D2E_4D10_954C_92DC51D23523__INCLUDED_)
#define AFX_CLOTHOIDPATH_H__E1689BA0_5D2E_4D10_954C_92DC51D23523__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <vector>

#include "Path.h"
#include "MyTrack.h"

class ClothoidPath : public Path
{
public:
	enum
	{
		FLAG_FLYING		= 0x01,
	};

	struct Options
	{
		int					bumpMod;
		double				safetyLimit;
		double				safetyMultiplier;
		int					quadSmoothIters;
		double				maxL;
		double				maxR;
		std::vector<double>	factors;

		Options() : bumpMod(0), safetyLimit(1.5), safetyMultiplier(100), quadSmoothIters(0), maxL(999), maxR(999) {}
		Options( int bm, double limit = 1.5, double mult = 100, double ml = 999, double mr = 999 )
		:	bumpMod(bm), safetyLimit(limit), safetyMultiplier(mult), quadSmoothIters(0), maxL(ml), maxR(mr), factors(1, 1.005) {}
	};

	class ICalcTimeFunc
	{
	public:
		virtual double operator()( const Path& path ) const = 0;
	};
	
	class EstimateTimeFunc : public ICalcTimeFunc
	{
	public:
		virtual double operator()( const Path& path ) const
		{
			return path.CalcEstimatedLapTime();
		}
	};

public:
	ClothoidPath();
	virtual ~ClothoidPath();

	virtual ClothoidPath&	operator=( const Path& other );
	ClothoidPath&			operator=( const ClothoidPath& other );

	const Options&	GetOptions() const;

	void	MakeSmoothPath( MyTrack* pTrack, const CarModel& cm,
							const Options& opts );

	void	Search( const CarModel& cm );
	void	Search( const CarModel& cm, const ICalcTimeFunc& calcTimeFunc );

//private:
	void	AnalyseBumps( const CarModel& cm, bool dumpInfo = false );
	void	SmoothBetween( int step );
	using Path::SetOffset;
	void	SetOffset( const CarModel& cm, double k, double t,
					   PathPt* l3, const PathPt* l2, const PathPt* l4 );
	void	OptimiseLine( const CarModel& cm, int idx, int step, double hLimit,
						  PathPt* l3, const PathPt* l2, const PathPt* l4 );
	void	Optimise(	const CarModel& cm, double factor,
						int idx, PathPt* l3,
						const PathPt* l0, const PathPt* l1,
						const PathPt* l2, const PathPt* l4,
						const PathPt* l5, const PathPt* l6,
						int	bumpMod );
	void	OptimisePath(	const CarModel& cm,
							int step, int nIterations, int bumpMod );

private:
	Options	m_options;
};

#endif // !defined(AFX_CLOTHOIDPATH_H__E1689BA0_5D2E_4D10_954C_92DC51D23523__INCLUDED_)
