#ifndef PARAMETRIC_CUBIC_SPLINE_H_
#define PARAMETRIC_CUBIC_SPLINE_H_

#include "ParametricCubic.h"

#include <vector>

class ParametricCubicSpline
{
public:
    ParametricCubicSpline( int size, const Vec2d* points, const Vec2d* tangents );
    ~ParametricCubicSpline();

    bool CalcLineCrossingPt( const Vec2d& linePoint, const Vec2d& lineTangent, double* t );
    
private:
    std::vector<ParametricCubic> _curves;
};

#endif  // PARAMETRIC_CUBIC_SPLINE_H_
