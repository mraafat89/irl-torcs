
#include "CarBounds2d.h"
#include "Utils.h"

using namespace std;

static const int s_next_corner[] = {FRNT_LFT, REAR_LFT, REAR_RGT, FRNT_RGT};

CarBounds2d::CarBounds2d( const tCarElt* car )
{
/*
	xAxis = Vec2d(car->pub.DynGC.pos.az);
	yAxis = xAxis.GetNormal();

	Vec2d middle(car->pub.DynGC.pos.x, car->pub.DynGC.pos.y);

	pts[FRNT_LFT] = middle + xAxis * car->pub.corner[FRNT_LFT].x + yAxis * car->pub.corner[FRNT_LFT].y;
	pts[FRNT_RGT] = middle + xAxis * car->pub.corner[FRNT_RGT].x + yAxis * car->pub.corner[FRNT_RGT].y;
	pts[REAR_LFT] = middle + xAxis * car->pub.corner[REAR_LFT].x + yAxis * car->pub.corner[REAR_LFT].y;
	pts[REAR_RGT] = middle + xAxis * car->pub.corner[REAR_RGT].x + yAxis * car->pub.corner[REAR_RGT].y;
*/
	pts[FRNT_LFT] = Vec2d(car->pub.corner[FRNT_LFT].ax, car->pub.corner[FRNT_LFT].ay);
	pts[FRNT_RGT] = Vec2d(car->pub.corner[FRNT_RGT].ax, car->pub.corner[FRNT_RGT].ay);
	pts[REAR_LFT] = Vec2d(car->pub.corner[REAR_LFT].ax, car->pub.corner[REAR_LFT].ay);
	pts[REAR_RGT] = Vec2d(car->pub.corner[REAR_RGT].ax, car->pub.corner[REAR_RGT].ay);
	
	xAxis = Vec2d(pts[FRNT_LFT] - pts[REAR_LFT]).GetUnit();
	yAxis = Vec2d(pts[FRNT_LFT] - pts[FRNT_RGT]).GetUnit();
}

double	CarBounds2d::distToSide( int side, double maxDist, const CarBounds2d& other ) const
{
	vector<Vec2d> pts;
	pts.push_back( other.pts[FRNT_RGT] );
	pts.push_back( other.pts[FRNT_LFT] );
	pts.push_back( other.pts[REAR_LFT] );
	pts.push_back( other.pts[REAR_RGT] );
	pts.push_back( other.pts[FRNT_RGT] );

	return distToSide(side, maxDist, pts);
}

double	CarBounds2d::distToSide( int side, double maxDist, const vector<Vec2d>& pts ) const
{
	CarBounds2d temp(*this);

	Vec2d	midPt;
	double	midLen = 0;
	switch( side )
	{
		case SIDE_FRONT:
			temp.pts[REAR_LFT] = temp.pts[FRNT_LFT];
			temp.pts[REAR_RGT] = temp.pts[FRNT_RGT];
			midPt  = (temp.pts[FRNT_LFT] + temp.pts[FRNT_RGT]) * 0.5;
			midLen = temp.pts[FRNT_LFT].dist(temp.pts[FRNT_RGT]);
			break;

		case SIDE_REAR:
			temp.pts[FRNT_LFT] = temp.pts[REAR_LFT];
			temp.pts[FRNT_RGT] = temp.pts[REAR_RGT];
			midPt  = (temp.pts[REAR_LFT] + temp.pts[REAR_RGT]) * 0.5;
			midLen = temp.pts[REAR_LFT].dist(temp.pts[REAR_RGT]);
			break;

		case SIDE_LEFT:
			temp.pts[FRNT_RGT] = temp.pts[FRNT_LFT];
			temp.pts[REAR_RGT] = temp.pts[REAR_LFT];
			midPt  = (temp.pts[FRNT_LFT] + temp.pts[REAR_LFT]) * 0.5;
			midLen = temp.pts[FRNT_LFT].dist(temp.pts[REAR_LFT]);
			break;

		case SIDE_RIGHT:
			temp.pts[FRNT_LFT] = temp.pts[FRNT_RGT];
			temp.pts[REAR_LFT] = temp.pts[REAR_RGT];
			midPt  = (temp.pts[FRNT_RGT] + temp.pts[REAR_RGT]) * 0.5;
			midLen = temp.pts[FRNT_RGT].dist(temp.pts[REAR_RGT]);
			break;
	}

	double filterDistSq = (midLen + maxDist) * (midLen + maxDist);

	if( temp.collidesWith(pts, midPt, filterDistSq) )
		return 0;

	temp.inflateSide( side, maxDist );
	if( !temp.collidesWith(pts, midPt, filterDistSq) )
		return maxDist;

	double	incr = maxDist * 0.5;
	double	dist = maxDist - incr;
	temp.inflateSide( side, -incr );

	while( incr > 0.01 )
	{
		if( temp.collidesWith(pts, midPt, filterDistSq) )
		{
			incr *= 0.5;
			dist -= incr;
			temp.inflateSide( side, -incr );
		}
		else
		{
			incr *= 0.5;
			dist += incr;
			temp.inflateSide( side, incr );
		}
	}
	
	if( !temp.collidesWith(pts, midPt, filterDistSq) )
		dist -= incr;

	return dist;
}

void	CarBounds2d::inflateSide( int side, double delta )
{
	switch( side )
	{
		case SIDE_FRONT:	
			pts[FRNT_LFT] += xAxis * delta;
			pts[FRNT_RGT] += xAxis * delta;
			break;

		case SIDE_REAR:	
			pts[REAR_LFT] -= xAxis * delta;
			pts[REAR_RGT] -= xAxis * delta;
			break;

		case SIDE_LEFT:	
			pts[FRNT_LFT] += yAxis * delta;
			pts[REAR_LFT] += yAxis * delta;
			break;

		case SIDE_RIGHT:	
			pts[FRNT_RGT] -= yAxis * delta;
			pts[REAR_RGT] -= yAxis * delta;
			break;
	}
}

void	CarBounds2d::inflate( double deltaX, double deltaY )
{
	inflate( deltaX, deltaX, deltaY, deltaY );
}

void	CarBounds2d::inflate( double deltaFront, double deltaRear, double deltaLeft, double deltaRight )
{
	pts[FRNT_LFT] +=  xAxis * deltaFront + yAxis * deltaLeft;
	pts[REAR_LFT] += -xAxis * deltaRear  + yAxis * deltaLeft;
	pts[REAR_RGT] += -xAxis * deltaRear  - yAxis * deltaRight;
	pts[FRNT_RGT] +=  xAxis * deltaFront - yAxis * deltaRight;
}

bool	CarBounds2d::contains( const Vec2d& pt ) const
{
	for( int i = 0; i < 4; i++ )
	{
		if( Vec2d(pts[s_next_corner[i]] - pts[i]).GetNormal() * (pt - pts[i]) > 0 )
			return false;
	}
	
	return true;
}

bool	CarBounds2d::collidesWith( const CarBounds2d& other ) const
{
	// test for points contained.
	for( int i = 0; i < 4; i++ )
	{
		if( contains(other.pts[i]) || other.contains(pts[i]) )
			return true;
	}
	
	// test the edges crossing too.
	double	t1, t2;
	for( int i = 0; i < 4; i++ )
	{
		Vec2d	v = pts[s_next_corner[i]] - pts[i];
		for( int j = 0; j < 4; j++ )
		{
			Vec2d	ov = other.pts[s_next_corner[j]] - other.pts[j];
			if( Utils::LineCrossesLine(pts[i], v, other.pts[j], ov, t1, t2) )
			{
				if( t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1 )
					return true;
			}
		}
	}

	return false;
}

bool	CarBounds2d::collidesWith( const Vec2d& pt1, const Vec2d& pt2 ) const
{
	double	t1, t2;
	Vec2d	lv = pt2 - pt1;

	for( int i = 0; i < 4; i++ )
	{
		Vec2d	v = pts[s_next_corner[i]] - pts[i];
		if( Utils::LineCrossesLine(pts[i], v, pt1, lv, t1, t2) )
		{
			if( t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1 )
				return true;
		}
	}

	return false;
}

bool	CarBounds2d::collidesWith( const std::vector<Vec2d>& otherPts, const Vec2d& filterPt, double filterDistSqLimit ) const
{
	if( otherPts.empty() )
		return false;

	bool lastIsOk =	filterDistSqLimit < 0 || otherPts[0].DistSq(filterPt) <= filterDistSqLimit;

	double	t1, t2;
	for( int i = 1; i < (int)otherPts.size(); i++ )
	{
		bool currIsOk =	filterDistSqLimit < 0 || otherPts[i].DistSq(filterPt) <= filterDistSqLimit;

		if( lastIsOk && currIsOk )
		{
			Vec2d	otherV = otherPts[i] - otherPts[i - 1];

			for( int j = 0; j < 4; j++ )
			{
				Vec2d	v = pts[s_next_corner[j]] - pts[j];
				if( Utils::LineCrossesLine(otherPts[i - 1], otherV, pts[j], v, t1, t2) )
				{
					if( t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1 )
						return true;
				}
			}
		}
		
		lastIsOk = currIsOk;
	}

	return false;
}
