#include "ParametricCubicSpline.h"

ParametricCubicSpline::ParametricCubicSpline( int size, const Vec2d* points, const Vec2d* tangents )
{
    {for( int i = 0; i + 1 < size; i++ )
    {
//        _curves.push_back( ParametricCubic::FromPointsAndTangents(points[i],   tangents[i],
//                                                                  points[i+1], tangents[i+1]) );
        _curves.push_back( ParametricCubic::HaliteFromPointsAndTangents(
									points[i], tangents[i], points[i+1], tangents[i+1]) );
    }}
}

ParametricCubicSpline::~ParametricCubicSpline()
{
}

bool ParametricCubicSpline::CalcLineCrossingPt( const Vec2d& linePoint, const Vec2d& lineTangent, double* t )
{
    // slow method... try each curve in turn until a solution is found.
    {for( int i = 0; i < (int)_curves.size(); i++ )
    {
        if( _curves[i].Calc1stLineCrossingPt(linePoint, lineTangent, t) )
            return true;
    }}
    
    return false;
}
