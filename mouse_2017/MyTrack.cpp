/***************************************************************************

    file        : MyTrack.cpp
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

// MyTrack.cpp: implementation of the MyTrack class.
//
//////////////////////////////////////////////////////////////////////

#include "MyTrack.h"
#include "Utils.h"

#include <robottools.h>

using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

MyTrack::MyTrack()
:	NSEG(0),
	m_delta(3),
	m_pSegs(0),
	m_pCurTrack(0),
	m_nBends(0)
{
}

MyTrack::~MyTrack()
{
	delete [] m_pSegs;
}

void	MyTrack::Clear()
{
	delete [] m_pSegs;
	NSEG = 0;
	m_pSegs = 0;
	m_pCurTrack = 0;
	m_innerMod.clear();
	m_nBends = 0;
}

void	MyTrack::NewTrack( tTrack* pNewTrack, const vector<double>* pInnerMod, bool pit, SideMod* pSideMod )
{
	if( m_pCurTrack != pNewTrack )
	{
		delete [] m_pSegs;
		m_pSegs = 0;
		NSEG = 0;
	}

	m_pCurTrack = pNewTrack;

	if( pInnerMod )
		m_innerMod = *pInnerMod;
	else
	{
		m_innerMod.clear();
	}

	if( pSideMod )
		m_sideMod = *pSideMod;

	if( m_pSegs == 0 )
	{
		// make new segs ... roughly every NOMINAL_SEG_LEN metres apart.
		const double	NOMINAL_SEG_LEN = 3;//10;
		NSEG = int(floor(pNewTrack->length / NOMINAL_SEG_LEN));
		m_pSegs = new Seg[NSEG];
		m_delta = pNewTrack->length / NSEG;

//		DEBUGF( "   ### NSEG %d\n", NSEG );

		tTrackSeg*	pseg = pNewTrack->seg;
		while( pseg->lgfromstart > pNewTrack->length / 2 )
			pseg = pseg->next;
		double		tsend = pseg->lgfromstart + pseg->length;
//		DEBUGF( "   ### tsend %g len %g fromstart %g\n",
//					tsend, pseg->length, pseg->lgfromstart );

		int	pitEntry = -1;
		int	pitExit  = -1;
		int pitSide  = pNewTrack->pits.side == TR_LFT ? TR_SIDE_LFT : TR_SIDE_RGT;

		{for( int i = 0; i < NSEG; i++ )
		{
			double	segDist = i * m_delta;
			while( segDist >= tsend )
			{
				pseg = pseg->next;
//				DEBUGF( "   ### segDist %g tsend %g len %g fromstart %g\n",
//						segDist, tsend, pseg->length, pseg->lgfromstart );
//				tsend += pseg->length;
				tsend = pseg->lgfromstart + pseg->length;
			}

			const double	MIN_MU = pseg->surface->kFriction * 0.8;
			const double	MAX_ROUGH = MX(0.005, pseg->surface->kRoughness * 1.2);
			const double	MAX_RESIST = MX(0.02, pseg->surface->kRollRes * 1.2);

//			DEBUGF( "   ### segDist %g   tsend %g\n",
//					segDist, tsend );

			double	t = (segDist - pseg->lgfromstart) / pseg->length;
			double	width = pseg->startWidth + (pseg->endWidth - pseg->startWidth) * t;

			m_pSegs[i].segDist = segDist;
			m_pSegs[i].pSeg = pseg;
			m_pSegs[i].wl = pseg->width / 2;
			m_pSegs[i].wr = pseg->width / 2;
			m_pSegs[i].midOffs = 0;
			m_pSegs[i].bendId = -1;

			if( pitEntry < 0 && (pseg->raceInfo & TR_PITENTRY) )
				pitEntry = i;
			if( (pseg->raceInfo & TR_PITEXIT) )
			//if( pitExit < 0 && (pseg->raceInfo & TR_PITEXIT) )
				pitExit  = i;
		}}

//		DEBUGF( "pit entry %d  pit exit %d \n", pitEntry, pitExit );
/*
		if( pNewTrack->pits.pitStart )
		{
			DEBUGF( "pit entry %d  pit exit %d \n",
				pNewTrack->pits.pitEntry->id, pNewTrack->pits.pitExit->id );
			DEBUGF( "pit start %d  pit end  %d \n",
				pNewTrack->pits.pitStart->id, pNewTrack->pits.pitEnd->id );
			DEBUGF( "pit side %d   pit len %g\n", pitSide, pNewTrack->pits.len );
			pseg = pNewTrack->pits.pitEntry->prev;
			do
			{
				pseg = pseg->next;
				DEBUGF( " %7.2fm %4d %5.1fm %4.1fm..%4.1fm",
						pseg->lgfromstart, pseg->id, pseg->length,
						pseg->startWidth, pseg->endWidth );

				tTrackSeg*	pSide = pseg->side[pitSide];
				while( pSide )
				{
					DEBUGF( "  %4.1f-%4.1fm %d %d %3x", pSide->startWidth, pSide->endWidth,
							pSide->type2, pSide->style, pSide->raceInfo );
					pSide = pSide->side[pitSide];
				}

				DEBUGF( "\n" );
			}
			while( pseg != pNewTrack->pits.pitExit );
		}
*/
		int		lastStart	= 0;
		double	lastK		= 0;
		int		lastSign	= 1;

		const double STRAIGHT_K = 0.00005;
		bool	increasing	= true;

		vector<int>	bends;

		{for( int i = 0; i < NSEG; i++ )
		{
			const tTrackSeg* pseg = m_pSegs[i].pSeg;
			double k =  pseg->type == TR_LFT ?  1.0 / pseg->radius :
						pseg->type == TR_RGT ? -1.0 / pseg->radius : 0;
			//if( fabs(k) < STRAIGHT_K )
			//	k = 0;
			double absK = fabs(k);
			
			if( k != lastK )
			{
				//PRINTF("[%d] t=%d, r=%g, k=%g, s=%d\n", i, pseg->type, pseg->radius, k, lastSign );
				if( lastSign * lastK > 0 && lastSign * k < lastSign * lastK )
				{
					PRINTF("bend[%d..%d] r=%g, k=%g, s=%d\n", lastStart, i, pseg->radius, lastK, lastSign );
					bends.push_back( (lastStart + i) / 2 );
				}

				lastStart	= i;
				lastSign	= k < lastK ? -1 : 1;
				lastK		= k;
			}
		}}

		int nBends = bends.size();
		PRINTF( "number of bends identified: %d\n", nBends );

		{for( int bend = 0; bend < nBends; bend++ )
		{
			int begin	= bends[bend];
			int end		= bends[(bend + 1) % nBends];
			int len		= (end - begin + NSEG) % NSEG;
			int half	= len / 2;

			int type	= m_pSegs[begin].pSeg->type;
			int nextId  = (1 + bend * 2) % (nBends * 2);
			for( int j = 0; j < half; j++ )
			{
				int index = (begin + j + NSEG) % NSEG;
				if( type != m_pSegs[index].pSeg->type )
				{
					PRINTF( "[%db] end=%d\n", nextId / 2, index );
					break;
				}
 				m_pSegs[index].bendId = nextId;
			}

			type	= m_pSegs[end].pSeg->type;
			nextId  = (1 + bend * 2 + 1) % (nBends * 2);
			for( int j = 0; j < half; j++ )
			{
				int index = (end - j + NSEG) % NSEG;
				if( type != m_pSegs[index].pSeg->type )
				{
					PRINTF( "[%da] begin=%d\n", nextId / 2, index );
					break;
				}
 				m_pSegs[index].bendId = nextId;
			}
		}}

		{for( int i = 0; i < NSEG; i++ )
		{
			pseg = m_pSegs[i].pSeg;

//			DEBUGF( "   ### segDist %g   tsend %g\n",
//					segDist, tsend );

			double	segDist = m_pSegs[i].segDist;
			double	t = (segDist - pseg->lgfromstart) / pseg->length;

			bool	inPit = (pitEntry < pitExit && pitEntry <= i && i <= pitExit ||
							 pitEntry > pitExit && (i <= pitExit || i >= pitEntry));

			const double	MIN_MU = pseg->surface->kFriction * 0.8;
			const double	MAX_ROUGH = MX(0.005, pseg->surface->kRoughness * 1.2);
			const double	MAX_RESIST = MX(0.02, pseg->surface->kRollRes * 1.2);
			const double	SLOPE = pseg->Kzw;

			{for( int s = 0; s < 2; s++ )
			{
				tTrackSeg*	pSide = pseg->side[s];
				if( pSide == 0 )
					continue;

				double	extraW = 0;

				bool	done = false;
				bool	edge = false;
				while( !done )
				{
					if( !pSide )
					{
						// got to edge .. there's always a wall here...
						extraW = MX(0, extraW - 0.5);
						break;
					}

					double	w = pSide->startWidth +
								(pSide->endWidth - pSide->startWidth) * t;
//					w = MX(0, w - 0.5);

//					w = MN(w, 1.0);
					if( pSide->style == TR_CURB )
					{
						if( s == m_sideMod.side &&
							i >= m_sideMod.start &&
							i <= m_sideMod.end )
							;
						else
						{
						// always keep 1 wheel on main track.
						w = MN(w, 1.5);
						done = true;

						if( (s == TR_SIDE_LFT && pseg->type == TR_RGT ||
							 s == TR_SIDE_RGT && pseg->type == TR_LFT) &&
							pSide->surface->kFriction  < pseg->surface->kFriction )
							// keep a wheel on the good stuff.
							w = 0;//MN(w, 1.5);

						// don't go too far up raised curbs (max 2cm).
						if( pSide->height > 0 )
							w = MN(w, 0.6);
//							w = MN(w, 1.0);
//							w = MN(0.05 * pSide->width / pSide->height, 1.5);

//						if( pSide->surface->kFriction  < MIN_MU )
//							w = 0;
						}
					}
					else if( pSide->style == TR_PLAN )
					{
						if( (inPit && pitSide == s)							 ||
							false )//(pSide->raceInfo & (TR_SPEEDLIMIT | TR_PITLANE)) )
						{
							w = 0;
							done = true;
						}

						if( s == m_sideMod.side &&
							i >= m_sideMod.start &&
							i <= m_sideMod.end )
						{
							if( w > 0.5 )
								{ w = 0.5; done = true; }
						}
						else
						if( pSide->surface->kFriction  < MIN_MU		||
							pSide->surface->kRoughness > MAX_ROUGH	||
							pSide->surface->kRollRes   > MAX_RESIST	||
							fabs(pSide->Kzw - SLOPE) > 0.005 )
						{
							bool	inner = 
										s == TR_SIDE_LFT && pseg->type == TR_LFT ||
										s == TR_SIDE_RGT && pseg->type == TR_RGT;
							w = 0;//inner ? MN(w, 0.5) : 0;
							done = true;
						}

						if( (s == TR_SIDE_LFT && pseg->type == TR_RGT ||
							 s == TR_SIDE_RGT && pseg->type == TR_LFT) &&
							pSide->surface->kFriction  < pseg->surface->kFriction )
						{
							// keep a wheel on the good stuff.
							w = 0;//MN(w, 1.5);
							done = true;
						}
					}
					else
					{
						// wall of some sort.
//						w = 0;
						w = pSide->style == TR_WALL ? -0.5 :
//							pSide->style == TR_FENCE ? -0.1 :
							0;
						done = true;
					}

					extraW += w;

//					if( pSide->style != TR_PLAN || w <= 0 )
//						done = true;

					pSide = pSide->side[s];
				}

				bool	inner = s == TR_SIDE_LFT && pseg->type == TR_LFT ||
								s == TR_SIDE_RGT && pseg->type == TR_RGT;
				if( inner )
				{
					int innerId = m_pSegs[i].bendId;
					if( innerId >= 0 && innerId < (int)m_innerMod.size() )
					{
						double innerMod = m_innerMod[innerId];
						extraW += innerMod;
					}
				}

//				extraW = MX(0, extraW - 0.1);
				if( s == TR_SIDE_LFT )
					m_pSegs[i].wl += extraW;
				else
					m_pSegs[i].wr += extraW;

				double extent = pseg->width / 2;
				pSide = pseg->side[s];
				while( pSide )
				{
					if( pSide->style >= TR_WALL )
						break;
						
					extent += pSide->width;

					pSide = pSide->side[s];
				}
				
				if( s == TR_SIDE_LFT )
					m_pSegs[i].el = extent;
				else
					m_pSegs[i].er = extent;
			}}

//			DEBUGF( "\n" );

//			m_pSegs[i].wl = 1;
//			m_pSegs[i].wr = 1;
//			m_pSegs[i].wl *= 0.6;
//			m_pSegs[i].wr *= 0.6;

			CalcPtAndNormal( pseg, segDist - pseg->lgfromstart,
//								m_pSegs[i].wl + m_pSegs[i].wr,
								m_pSegs[i].t,
								m_pSegs[i].pt, m_pSegs[i].norm );

//			DEBUGF( "%4d  p(%7.2f, %7.2f, %7.2f)  n(%7.4f, %7.4f, %7.4f)\n",
//					i, m_pSegs[i].pt.x, m_pSegs[i].pt.y, m_pSegs[i].pt.z,
//					m_pSegs[i].norm.x, m_pSegs[i].norm.y, m_pSegs[i].norm.z );
		}}
		
		{for( int i = 0; i < NSEG; i++ )
		{
		    int j = (i + 1) % NSEG;

			{for( int s = 0; s < 2; s++ )
			{
				double  currW = s == TR_SIDE_LFT ? m_pSegs[i].el : m_pSegs[i].er;
				double  nextW = s == TR_SIDE_LFT ? m_pSegs[j].el : m_pSegs[j].er;

                if( currW > nextW + 1 )

				if( s == TR_SIDE_LFT )
					m_pSegs[i].el = nextW;
				else
					m_pSegs[i].er = nextW;
			}}
		}}

		{for( int i = NSEG - 1; i >= 0; i-- )
		{
		    int j = (i + NSEG - 1) % NSEG;

			{for( int s = 0; s < 2; s++ )
			{
				double  currW = s == TR_SIDE_LFT ? m_pSegs[i].el : m_pSegs[i].er;
				double  prevW = s == TR_SIDE_LFT ? m_pSegs[j].el : m_pSegs[j].er;

                if( currW > prevW + 1 )

				if( s == TR_SIDE_LFT )
					m_pSegs[i].el = prevW;
				else
					m_pSegs[i].er = prevW;
			}}
		}}
	
/*
		{for( int i = 0; i < NSEG; i++ )
		{
			int	in = (i + 1) % NSEG;

			if( m_pSegs[i].wl > m_pSegs[in].wl )
			{
				if( m_pSegs[i].wl > m_pSegs[in].wl - 0.25 )
					m_pSegs[i].wl = m_pSegs[in].wl - 1;
				else
					m_pSegs[i].wl = m_pSegs[in].wl;
			}
			if( m_pSegs[i].wr > m_pSegs[in].wr )
			{
				if( m_pSegs[i].wr > m_pSegs[in].wr - 0.25 )
					m_pSegs[i].wr = m_pSegs[in].wr - 1;
				else
					m_pSegs[i].wr = m_pSegs[in].wr;
			}
		}}

		{for( int i = NSEG - 1; i >= 0; i-- )
		{
			int	ip = (i - 1 + NSEG) % NSEG;

			if( m_pSegs[i].wl > m_pSegs[ip].wl )
				m_pSegs[i].wl = m_pSegs[ip].wl;
			if( m_pSegs[i].wr > m_pSegs[ip].wr )
				m_pSegs[i].wr = m_pSegs[ip].wr;
		}}*/
	}
}

tTrack*	MyTrack::GetTrack()
{
	return m_pCurTrack;
}

const tTrack*	MyTrack::GetTrack() const
{
	return m_pCurTrack;
}

double	MyTrack::GetLength() const
{
	return m_pCurTrack->length;
}

int		MyTrack::GetSize() const
{
	return NSEG;
}

double	MyTrack::GetWidth() const
{
	return m_pCurTrack->width;
}

double	MyTrack::NormalisePos( double trackPos ) const
{
	while( trackPos < 0 )
		trackPos += m_pCurTrack->length;
	while( trackPos >= m_pCurTrack->length )
		trackPos -= m_pCurTrack->length;
	return trackPos;
}

bool	MyTrack::PosInRange( double pos, double rangeStart, double rangeLength ) const
{
	double delta = NormalisePos(pos - rangeStart);
	return delta < rangeLength;
}

int		MyTrack::IndexFromPos( double trackPos ) const
{
	int	idx = (int(floor(trackPos / m_delta)) + NSEG) % NSEG;
/*	if( idx < 0 )
		idx += NSEG;
	else if( idx >= NSEG )
		idx -= NSEG;*/
	return idx;
}

const Seg&	MyTrack::operator[]( int index ) const
{
	return m_pSegs[index];
}

const Seg&	MyTrack::GetAt( int index ) const
{
	return m_pSegs[index];
}

double	MyTrack::GetDelta() const
{
	return m_delta;
}

double	MyTrack::CalcPos( const tTrkLocPos& trkPos, double offset ) const
{
	double	pos = RtGetDistFromStart2(const_cast<tTrkLocPos*>(&trkPos)) + offset;
	return NormalisePos(pos);
}

double	MyTrack::CalcPos( const tCarElt* car, double offset ) const
{
	double	pos = RtGetDistFromStart(const_cast<tCarElt*>(car)) + offset;
	return NormalisePos(pos);
}

double	MyTrack::CalcPos( double x, double y, const Seg* hint, bool sides ) const
{
	tTrackSeg*	pTrackSeg = m_pSegs[0].pSeg;
	if( hint != 0 )
		pTrackSeg = hint->pSeg;

	tTrkLocPos	pos;
	RtTrackGlobal2Local( pTrackSeg, tdble(x), tdble(y), &pos, sides );
	double	dist = RtGetDistFromStart2(&pos);
	return dist;
}

double	MyTrack::CalcForwardAngle( double trackPos ) const
{
	int					idx = IndexFromPos(trackPos);
	const tTrackSeg*	pSeg = m_pSegs[idx].pSeg;

	double	t;
	Vec3d	pt;
	Vec3d	norm;
	CalcPtAndNormal( pSeg, NormalisePos(trackPos - pSeg->lgfromstart), t, pt, norm );

	return Utils::VecAngXY(norm) + PI / 2;
}

Vec2d	MyTrack::CalcNormal( double trackPos ) const
{
	int					idx = IndexFromPos(trackPos);
	const tTrackSeg*	pSeg = m_pSegs[idx].pSeg;

	double	t;
	Vec3d	pt;
	Vec3d	norm;
	CalcPtAndNormal( pSeg, NormalisePos(trackPos - pSeg->lgfromstart), t, pt, norm );

	return norm.GetXY();
}

double	MyTrack::GetFriction( int index, double offset ) const
{
	const tTrackSeg*	pSeg = m_pSegs[index].pSeg;
	double	friction = pSeg->surface->kFriction;
//	if( pSeg->surface->kRoughness > 0.005 )
//		friction *= 0.9;

/*
	double	w = pSeg->width / 2;
	if( offset < -w && pSeg->lside )
	{
		// on side to left
		friction = pSeg->lside->surface->kFriction;
	}
	else if( offset > w )
	{
		// on side to right
		friction = pSeg->rside->surface->kFriction;
	}
*/
	return friction;
}

void	MyTrack::CalcPtAndNormal(
	const tTrackSeg*	pSeg,
	double				toStart,
	double&				t,
	Vec3d&				pt,
	Vec3d&				norm ) const
{
	if( pSeg->type == TR_STR )
	{
		Vec3d	s = (Vec3d(pSeg->vertex[TR_SL]) + Vec3d(pSeg->vertex[TR_SR])) / 2;
		Vec3d	e = (Vec3d(pSeg->vertex[TR_EL]) + Vec3d(pSeg->vertex[TR_ER])) / 2;
		t = toStart / pSeg->length;
		pt = s + (e - s) * t;

		double hl = pSeg->vertex[TR_SL].z +
					(pSeg->vertex[TR_EL].z - pSeg->vertex[TR_SL].z) * t;
		double hr = pSeg->vertex[TR_SR].z +
					(pSeg->vertex[TR_ER].z - pSeg->vertex[TR_SR].z) * t;
		norm = -Vec3d(pSeg->rgtSideNormal);
		norm.z = (hr - hl) / pSeg->width;
	}
	else
	{
		double d = pSeg->type == TR_LFT ? 1 : -1;
		double deltaAng = d * toStart / pSeg->radius;
		double ang = pSeg->angle[TR_ZS] - PI / 2 + deltaAng;
		double c = cos(ang);
		double s = sin(ang);
		double r = d * pSeg->radius;
		t = toStart / pSeg->length;
		double hl = pSeg->vertex[TR_SL].z +
					(pSeg->vertex[TR_EL].z - pSeg->vertex[TR_SL].z) * t;
		double hr = pSeg->vertex[TR_SR].z +
					(pSeg->vertex[TR_ER].z - pSeg->vertex[TR_SR].z) * t;
		pt = Vec3d(pSeg->center.x + c * r, pSeg->center.y + s * r, (hl + hr) / 2);
		norm = Vec3d(c, s, (hr - hl) / pSeg->width);
	}
}
