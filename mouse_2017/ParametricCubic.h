// ParametricCubic.h: interface for the ParametricCubic class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PARAMETRICCUBIC_H__EF867A55_5F05_4790_B3C0_DA5740F7135F__INCLUDED_)
#define AFX_PARAMETRICCUBIC_H__EF867A55_5F05_4790_B3C0_DA5740F7135F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Cubic.h"
#include "Vec2d.h"

class ParametricCubic  
{
public:
	ParametricCubic();
	~ParametricCubic();

	void	SetPoints( Vec2d p0, Vec2d p1, Vec2d p2, Vec2d p3 );
	void	SetPointsAndTangents( Vec2d p0, Vec2d v0, Vec2d p1, Vec2d v1 );
	void	SetHalitePointsAndTangents( Vec2d p0, Vec2d v0, Vec2d p1, Vec2d v1 );

    static ParametricCubic FromPoints( Vec2d p0, Vec2d p1, Vec2d p2, Vec2d p3 );
    static ParametricCubic FromPointsAndTangents( Vec2d p0, Vec2d v0, Vec2d p1, Vec2d v1 );
	static ParametricCubic HaliteFromPointsAndTangents( Vec2d p0, Vec2d v0, Vec2d p1, Vec2d v1 );

	Vec2d	Calc( double t ) const;
	Vec2d	CalcGradient( double t ) const;
	double	CalcCurvature( double t ) const;

    bool    Calc1stLineCrossingPt( const Vec2d& linePoint, const Vec2d& lineTangent, double* t ) const;

private:
	Cubic	m_x;
	Cubic	m_y;
};

#endif // !defined(AFX_PARAMETRICCUBIC_H__EF867A55_5F05_4790_B3C0_DA5740F7135F__INCLUDED_)
