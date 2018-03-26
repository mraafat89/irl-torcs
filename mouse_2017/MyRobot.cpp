/***************************************************************************

    file        : MyRobot.cpp
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

// MyRobot.cpp: implementation of the MyRobot class.
//
//////////////////////////////////////////////////////////////////////

#include "MyRobot.h"
#include "Quadratic.h"
#include "Utils.h"
#include "Avoidance.h"
#include "CarBounds2d.h"

#include <portability.h>
#include <robottools.h>

#include <algorithm>
#include <string>

using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define SECT_PRIV					"mouse private"
#define SECT_PRIV_LEFT				"mouse private/left"
#define SECT_PRIV_RIGHT				"mouse private/right"
#define PRV_GEAR_UP_RPM				"gear up rpm"
#define PRV_MU_SCALE				"mu scale"
#define PRV_BRAKE_MU_SCALE			"brake mu scale"
#define PRV_FLY_HEIGHT				"fly height"
#define PRV_FACTOR					"factor"
#define PRV_INNER_MOD				"inner mod"
#define PRV_CARMODEL_FLAGS			"carmodel flags"
#define PRV_BUMP_MOD				"bump mod"
#define PRV_SIDE_MOD				"side mod"
#define PRV_QUAD_SMOOTH_ITERS		"quad smooth iters"
#define PRV_KZ_SCALE				"kz scale"
#define PRV_ACC_MAX_SPIN_SPEED		"acc max spin speed"
#define PRV_DEC_MAX_SPIN_SPEED		"dec max spin speed"
#define PRV_STEER_K_ACC				"steer k acc"
#define PRV_STEER_K_DEC				"steer k dec"
#define PRV_AVOID_WIDTH				"avoid width"
#define PRV_SPDC_NORMAL				"spd ctrl normal"
#define PRV_SPDC_TRAFFIC			"spd ctrl traffic"
#define PRV_STAY_TOGETHER			"stay together"
#define PRV_PIT_ENTRY_OFFS			"pit entry offset"
#define PRV_PIT_EXIT_OFFS			"pit exit offset"
#define PRV_PIT_DAMAGE_WARN			"pit damage warn limit"
#define PRV_PIT_DAMAGE_DANGER		"pit damage danger limit"
#define PRV_SKID_FACTOR				"skid factor"
#define PRV_SKID_FACTOR_TRAFFIC		"skid factor traffic"
#define PRV_REAR_LAT_SLIP_FACTOR	"rear lat slip factor"
#define PRV_REAR_LAT_SLIP_LIMIT		"rear lat slip limit"
#define PRV_REAR_LAT_SLIP_DSCALE	"rear lat slip dscale"
#define PRV_STEER_0_LINE_SCALE		"steer 0 line scale"
#define PRV_TCL_TARGET_SPEED		"tcl target speed"
#define PRV_SAFETY_LIMIT			"safety limit"
#define PRV_SAFETY_MULTIPLIER		"safety multiplier"
#define PRV_BRAKE_LIMIT				"brake limit"

#define FLY_COUNT		20

#define	STEER_SPD_IDX(x)	(int(floor((x) / 5)))
#define	STEER_K_IDX(k)		(MX(0, MN(int(20 + floor((k) * 500 + 0.5)), 40)))

static const double	s_sgMin[] = { 0.00,  10 };
static const double	s_sgMax[] = { 0.03, 100 };
static const int	s_sgSteps[] = { 10,  18 };

void	MyRobot::PathRange::AddGreater(
	double pos,
	double offs,
	bool incRL,
	const MyRobot& me )
{
/*	double	pu, pv;
	me.CalcBestPathUV( pos, offs, pu, pv );
	if( !gotL )
	{
		u  = pu;
		vL = pv;
	}
	else
	{
		if( u != pu )
		{
			if( u < pu )
			{
				// need to re-calc pv using u;
				pv = rgtT = me.CalcPathTarget(pos, offs, u);
				if( vL < pv )
				{
					vL = pv;
				}
			}
			else
			{
				// need to re-calc vL using pu;
				double vL2 = me.CalcPathTarget(pos, offs, pu);
				if( vL2 < pv )
				{
					u  = pu;
					vL = pv;
				}
			}
		}
		else if( vL < pv )
		{
			vL = pv;
		}
	}*/
}

void	MyRobot::PathRange::AddLesser(
	double pos,
	double offs,
	bool incRL,
	const MyRobot& me )
{
}

MyRobot::MyRobot()
:	m_pitControl(m_track, m_pitPath[PATH_NORMAL][0]),
	m_driveType(cDT_RWD),
	m_gearUpRpm(8000),
	m_prevYawError(0),
	m_prevLineError(0),
	m_flying(0),
	m_steerLimit(0),
	m_avgAY(0),
	m_raceStart(false),
	m_avoidS(1),
	m_avoidT(0),
	m_followPath(PATH_NORMAL),
	m_lastB(0),
	m_lastBrk(0),
	m_lastTargV(0),
	m_maxAccel(0, 150, 30, 1),
	m_steerGraph(2, s_sgMin, s_sgMax, s_sgSteps, 0),
	m_stuck(NOT_STUCK),
	m_stuckTime(0),
	_acc(0),
	_tctrlAcc(0),
	_deltaCounter(0),
	_prevDelta(0),
	_lastSpd0(0)
{
	{for( int i = 0; i < 50; i++ )
	{
		m_brkCoeff[i] = 0.5;//0.12;
	}}

	{for( int i = 0; i < STEER_SPD_MAX; i++ )
	{
		for( int j = 0; j < STEER_K_MAX; j++ )
		{
			m_steerCoeff[i][j] = 0;
		}
	}}

	memset( m_angle, 0, sizeof(m_angle) );
}

MyRobot::~MyRobot()
{
}

static void*	MergeParamFile( void* hParams, const char* fileName )
{
//	PRINTF( "loading: '%s'\n", fileName );
	void*	hNewParams = GfParmReadFile(fileName, GFPARM_RMODE_STD);
	if( hNewParams == NULL )
		return hParams;

	if( hParams == NULL )
	{
		PRINTF( "loaded: '%s'\n", fileName );
		return hNewParams;
	}

	PRINTF( "merging: '%s'\n", fileName );
	return GfParmMergeHandles(hParams, hNewParams,
				GFPARM_MMODE_SRC    | GFPARM_MMODE_DST |
				GFPARM_MMODE_RELSRC | GFPARM_MMODE_RELDST);
}

static double	SafeParmGetNum( void *handle, const char *path, const char *key, const char *unit, tdble deflt )
{
	if( handle == NULL )
		return deflt;
	return GfParmGetNum(handle, path, key, unit, deflt);
}

void	MyRobot::SetShared( Shared* pShared )
{
	m_pShared = pShared;
}

// Called for every track change or new race.
void	MyRobot::InitTrack(
	int			index,
	tTrack*		pTrack,
	void*		pCarHandle,
	void**		ppCarParmHandle,
	tSituation*	pS )
{
//	DEBUGF( "mouse:initTrack()\n" );

	//
	//	get the name of the car (e.g. "clkdtm").
	//
	//	this is a sucky way to get this, but it's the only way I've managed
	//	to come up with so far.  it basically gets the name of the car .acc
	//	file, and strips off the ".acc" part, to get the name of the car.  yuk.
	//

	char*	path = SECT_GROBJECTS "/" LST_RANGES "/" "1";
	char*	key  = PRM_CAR;
	strncpy( m_carName, GfParmGetStr(pCarHandle, path, key, ""), sizeof(m_carName) );
	char*	p = strrchr(m_carName, '.');
	if( p )
		*p = '\0';
//	DEBUGF( "   m_carName: '%s'\n", m_carName );

	//
	//	get the name of the track (e.g. "e-track-1")
	//

	strncpy( m_trackName, strrchr(pTrack->filename, '/') + 1, sizeof(m_trackName) );
	*strrchr(m_trackName, '.') = '\0';

	//
	//	set up race type array.
	//

	const char*	raceTypeStr[] = { "practice", "qualify", "race" };
	int			raceType = pS->raceInfo.type;
	if( raceType == RM_TYPE_PRACTICE && (pS->raceInfo.totLaps == 3 || pS->raceInfo.totLaps == 10) )
		raceType =  RM_TYPE_QUALIF;

	//
	//	set up the base param path.
	//

	char	baseParamPath[] = "drivers/mouse_2017";

	//
	//	ok, lets read/merge the car parms.
	//

	void*	hCarParm = 0;
	char	buf[1024];

	// default params for car type (e.g. clkdtm)
    snprintf( buf, sizeof(buf), "%s/cars/%s/%s.xml",
				baseParamPath, m_carName, m_carName );
	hCarParm = MergeParamFile(hCarParm, buf);

	// override params for car type on track.
    snprintf( buf, sizeof(buf), "%s/cars/%s/track-%s.xml",
				baseParamPath, m_carName, m_trackName );
	hCarParm = MergeParamFile(hCarParm, buf);

	// override params for car type on track of specific race type.
    snprintf( buf, sizeof(buf), "%s/cars/%s/track-%s-%s.xml",
				baseParamPath, m_carName, m_trackName, raceTypeStr[raceType] );
	hCarParm = MergeParamFile(hCarParm, buf);

	// override params for car type on track with specific driver.
    snprintf( buf, sizeof(buf), "%s/cars/%s/track-%s-drv=%d.xml",
				baseParamPath, m_carName, m_trackName, index );
	hCarParm = MergeParamFile(hCarParm, buf);

	// override params for car type on track of specific race type and driver.
    snprintf( buf, sizeof(buf), "%s/cars/%s/track-%s-%s-drv=%d.xml",
				baseParamPath, m_carName, m_trackName, raceTypeStr[raceType], index );
	hCarParm = MergeParamFile(hCarParm, buf);

	// setup the car param handle to be returned.

	*ppCarParmHandle = hCarParm;

	// get the private parameters now.

	double rpm = SafeParmGetNum(hCarParm, SECT_PRIV, PRV_GEAR_UP_RPM, "rpm", 8300);
	m_gearUpRpm = rpm / 9.5;//10.0;
	PRINTF( "*** gear up rpm: %g (%g)\n", rpm, m_gearUpRpm );

	START_HOLD_LINE_TIME = SafeParmGetNum(hCarParm, SECT_PRIV, "start hold line time", 0, 5.0f);

	MyTrack::SideMod	sideMod[N_PATHS];
	string sect;
	for( int p = 0; p < N_PATHS; p++ )
	{
		switch( p )
		{
			case PATH_NORMAL:	sect = SECT_PRIV;		break;
			case PATH_LEFT:		sect = SECT_PRIV_LEFT;	break;
			case PATH_RIGHT:	sect = SECT_PRIV_RIGHT;	break;
		}

		if( p == PATH_NORMAL )
		{
			m_priv[p].FACTORS.clear();
			m_cm[p].FLAGS = 0;
			m_cm[p].MU_SCALE = 1.0;
			m_cm[p].BRAKE_MU_SCALE = 0.95;
			m_cm[p].KZ_SCALE = 0.22;
		}
		else
		{
			m_priv[p] = m_priv[PATH_NORMAL];
			m_cm[p]   = m_cm[PATH_NORMAL];
		}

		m_cm[p].FLAGS = (int)SafeParmGetNum(hCarParm, sect.c_str(), PRV_CARMODEL_FLAGS, 0, (tdble)m_cm[PATH_NORMAL].FLAGS);
		m_cm[p].MU_SCALE = SafeParmGetNum(hCarParm, sect.c_str(), PRV_MU_SCALE, NULL, m_cm[PATH_NORMAL].MU_SCALE);
		m_cm[p].BRAKE_MU_SCALE = SafeParmGetNum(hCarParm, sect.c_str(), PRV_BRAKE_MU_SCALE, NULL, m_cm[PATH_NORMAL].BRAKE_MU_SCALE);
		m_cm[p].KZ_SCALE = SafeParmGetNum(hCarParm, sect.c_str(), PRV_KZ_SCALE, NULL, m_cm[PATH_NORMAL].KZ_SCALE);

		{for( int i = 0; ; i++ )
		{
			snprintf( buf, sizeof(buf), "%s %d", PRV_FACTOR, i );
			double	factor = SafeParmGetNum(hCarParm, sect.c_str(), buf, 0, -1);
			DEBUGF( "%s: %g\n", buf, factor );
			if( factor == -1 )
				break;
			if( i == 0 )
				m_priv[p].FACTORS.clear();
			m_priv[p].FACTORS.push_back( factor );
		}}

		if( m_priv[p].FACTORS.size() == 0 )
			m_priv[p].FACTORS.push_back( 1.005 );
			
		{for( int bend = 0; bend < 100; bend++ )
		{
			snprintf( buf, sizeof(buf), "%s %d", PRV_INNER_MOD, bend );
			double	innerModA = SafeParmGetNum(hCarParm, sect.c_str(), buf, 0, 999);
			double	innerModB = innerModA;
			innerModA = SafeParmGetNum(hCarParm, sect.c_str(), (string(buf) + "a").c_str(), 0, innerModA);
			innerModB = SafeParmGetNum(hCarParm, sect.c_str(), (string(buf) + "b").c_str(), 0, innerModA);

			if( innerModA == 999 && innerModB == 999 )
			    continue;

			DEBUGF( "%s: %g, %g\n", buf, innerModA, innerModB );

			while( m_priv[p].INNER_MOD.size() < bend * 2 + 2 )
    			m_priv[p].INNER_MOD.push_back( 0 );

            if( innerModA != 999 )
			    m_priv[p].INNER_MOD[bend * 2] = innerModA;
			if( innerModB != 999 )
			    m_priv[p].INNER_MOD[bend * 2 + 1] = innerModB;
		}}

		if( m_priv[p].INNER_MOD.size() == 0 )
			m_priv[p].INNER_MOD.push_back( 0.0 );
			
		m_priv[p].FLY_HEIGHT = SafeParmGetNum(hCarParm, sect.c_str(), PRV_FLY_HEIGHT, "m", m_priv[p].FLY_HEIGHT);
		m_priv[p].BUMP_MOD = int(SafeParmGetNum(hCarParm, sect.c_str(), PRV_BUMP_MOD, 0, (float)m_priv[p].BUMP_MOD));
		m_priv[p].QUAD_SMOOTH_ITERS = int(SafeParmGetNum(hCarParm, sect.c_str(), PRV_QUAD_SMOOTH_ITERS, 0, (float)m_priv[p].QUAD_SMOOTH_ITERS) + 0.5f);
		m_priv[p].SPDC_NORMAL = int(SafeParmGetNum(hCarParm, sect.c_str(), PRV_SPDC_NORMAL, 0, (float)m_priv[p].SPDC_NORMAL));
//		m_priv[p].SPDC_NORMAL = 4;
		m_priv[p].SPDC_TRAFFIC = int(SafeParmGetNum(hCarParm, sect.c_str(), PRV_SPDC_TRAFFIC, 0, (float)m_priv[p].SPDC_TRAFFIC));
		m_priv[p].ACC_MAX_SPIN_SPEED = SafeParmGetNum(hCarParm, sect.c_str(), PRV_ACC_MAX_SPIN_SPEED, 0, m_priv[p].ACC_MAX_SPIN_SPEED);
		m_priv[p].DEC_MAX_SPIN_SPEED = SafeParmGetNum(hCarParm, sect.c_str(), PRV_DEC_MAX_SPIN_SPEED, 0, m_priv[p].DEC_MAX_SPIN_SPEED);
		m_priv[p].AVOID_WIDTH = SafeParmGetNum(hCarParm, sect.c_str(), PRV_AVOID_WIDTH, 0, m_priv[p].AVOID_WIDTH);
		m_priv[p].STAY_TOGETHER = SafeParmGetNum(hCarParm, sect.c_str(), PRV_STAY_TOGETHER, 0, m_priv[p].STAY_TOGETHER);
		m_priv[p].STEER_K_ACC = SafeParmGetNum(hCarParm, sect.c_str(), PRV_STEER_K_ACC, 0, m_priv[p].STEER_K_ACC);
		m_priv[p].STEER_K_DEC = SafeParmGetNum(hCarParm, sect.c_str(), PRV_STEER_K_DEC, 0, m_priv[p].STEER_K_DEC);
		m_priv[p].PIT_ENTRY_OFFSET = SafeParmGetNum(hCarParm, sect.c_str(), PRV_PIT_ENTRY_OFFS, 0, m_priv[p].PIT_ENTRY_OFFSET);
		m_priv[p].PIT_EXIT_OFFSET = SafeParmGetNum(hCarParm, sect.c_str(), PRV_PIT_EXIT_OFFS, 0, m_priv[p].PIT_EXIT_OFFSET);
		m_priv[p].PIT_DAMAGE_WARN = (int)SafeParmGetNum(hCarParm, sect.c_str(), PRV_PIT_DAMAGE_WARN, 0, (float)m_priv[p].PIT_DAMAGE_WARN);
		m_priv[p].PIT_DAMAGE_DANGER = (int)SafeParmGetNum(hCarParm, sect.c_str(), PRV_PIT_DAMAGE_DANGER, 0, (float)m_priv[p].PIT_DAMAGE_DANGER);
		m_priv[p].SKID_FACTOR = SafeParmGetNum(hCarParm, sect.c_str(), PRV_SKID_FACTOR, 0, m_priv[p].SKID_FACTOR);
		m_priv[p].SKID_FACTOR_TRAFFIC = SafeParmGetNum(hCarParm, sect.c_str(), PRV_SKID_FACTOR_TRAFFIC, 0, m_priv[p].SKID_FACTOR_TRAFFIC);
		m_priv[p].REAR_LAT_SLIP_FACTOR = SafeParmGetNum(hCarParm, sect.c_str(), PRV_REAR_LAT_SLIP_FACTOR, 0, m_priv[p].REAR_LAT_SLIP_FACTOR);
		m_priv[p].REAR_LAT_SLIP_LIMIT = SafeParmGetNum(hCarParm, sect.c_str(), PRV_REAR_LAT_SLIP_LIMIT, 0, m_priv[p].REAR_LAT_SLIP_LIMIT);
		m_priv[p].REAR_LAT_SLIP_DSCALE = SafeParmGetNum(hCarParm, sect.c_str(), PRV_REAR_LAT_SLIP_DSCALE, 0, m_priv[p].REAR_LAT_SLIP_DSCALE);
		m_priv[p].STEER_0_LINE_SCALE = SafeParmGetNum(hCarParm, sect.c_str(), PRV_STEER_0_LINE_SCALE, 0, m_priv[p].STEER_0_LINE_SCALE);
		m_priv[p].TCL_TARGET_SPEED = SafeParmGetNum(hCarParm, sect.c_str(), PRV_TCL_TARGET_SPEED, 0, m_priv[p].TCL_TARGET_SPEED);
		m_priv[p].SAFETY_LIMIT = SafeParmGetNum(hCarParm, sect.c_str(), PRV_SAFETY_LIMIT, 0, m_priv[p].SAFETY_LIMIT);
		m_priv[p].SAFETY_MULTIPLIER = SafeParmGetNum(hCarParm, sect.c_str(), PRV_SAFETY_MULTIPLIER, 0, m_priv[p].SAFETY_MULTIPLIER);
		m_priv[p].BRAKE_LIMIT = SafeParmGetNum(hCarParm, sect.c_str(), PRV_BRAKE_LIMIT, 0, m_priv[p].BRAKE_LIMIT);

		DEBUGF( "FLY_HEIGHT %g\n", m_priv[p].FLY_HEIGHT );
		DEBUGF( "BUMP_MOD %d\n", m_priv[p].BUMP_MOD );

		sideMod[p].side = -1;
		const char*	pStr = GfParmGetStr(hCarParm, sect.c_str(), PRV_SIDE_MOD, "");
		if( pStr == 0 ||
			sscanf(pStr, "%d , %d , %d",
					&sideMod[p].side, &sideMod[p].start, &sideMod[p].end) != 3 )
		{
			sideMod[p].side = -1;
		}

		DEBUGF( "SIDE MOD %d %d %d\n", sideMod[p].side, sideMod[p].start, sideMod[p].end );
		DEBUGF( "STAY_TOGETHER %g\n", m_priv[p].STAY_TOGETHER );
	}

	m_track.NewTrack( pTrack, &m_priv[PATH_NORMAL].INNER_MOD, false, &sideMod[PATH_NORMAL] );

	// setup initial fuel for race.
	double	fuelPerM        = SafeParmGetNum(hCarParm, SECT_PRIV, "fuel per m", 0, 0.001f);
	double	maxFuel			= SafeParmGetNum(hCarParm, SECT_CAR, PRM_TANK, (char*) NULL, 100.0f);
	double	fullRaceFuel	= 1.02 * pS->_totLaps * pTrack->length * fuelPerM;
	double	fuel			= fullRaceFuel;
	if( raceType == RM_TYPE_PRACTICE )
	{
		fuel = SafeParmGetNum(hCarParm, SECT_PRIV, "practice init fuel", "Kg", fuel);
		PRINTF( "practice initial fuel: %g\n", fuel );
	}
//	fuel = 3;
	if( fuel > maxFuel )
	{
		// pit required, so work out how much fuel per pit if divided equally.
		int nTanks = int(ceil(fullRaceFuel / maxFuel));
		fuel = fullRaceFuel / nTanks + fuelPerM * pTrack->length * (nTanks - 1);
		PRINTF( "number of pitstops: %d\n", nTanks - 1 );
	}
	PRINTF( "initial fuel per m: %g\n", fuelPerM );
	PRINTF( "intiial fuel: %g\n", fuel );
	GfParmSetNum( hCarParm, SECT_CAR, PRM_FUEL, (char*) NULL, fuel );

	m_pitControl.SetDamageLimits( m_priv[PATH_NORMAL].PIT_DAMAGE_WARN,
								  m_priv[PATH_NORMAL].PIT_DAMAGE_DANGER );
}

// Start a new race.
void	MyRobot::NewRace( int index, tCarElt* pCar, tSituation* pS )
{
//	DEBUGF( "mouse:newRace()\n" );

//	if( pS == 0 )
//		DEBUGF( "!!!!! situation ptr is null !!!!!\n" );
	m_nCars = pS->_ncars;
	m_myOppIdx = -1;
	{for( int i = 0; i < m_nCars; i++ )
	{
//		if( pS->cars[i] == 0 )
//			DEBUGF( "!!!!! car ptr is null !!!!!\n" );
//		m_oppPath[i].Initialise( &m_track, pS->cars[i] );
//		m_opp[i].Initialise( &m_track, pS->cars[i] );
		m_opp[i].Initialise( &m_track, pS->cars[i] );
		if( pS->cars[i] == pCar )
			m_myOppIdx = i;
	}}

//	m_cm[PATH_NORMAL].configWheels( pCar );
	m_cm[PATH_NORMAL].config( pCar );

	m_cm[PATH_NORMAL].MASS = SafeParmGetNum(pCar->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);

    float fwingarea = SafeParmGetNum(pCar->_carHandle, SECT_FRNTWING,
                                       PRM_WINGAREA, (char*) NULL, 0.0);
    float fwingangle = SafeParmGetNum(pCar->_carHandle, SECT_FRNTWING,
                                        PRM_WINGANGLE, (char*) NULL, 0.0);
    float rwingarea = SafeParmGetNum(pCar->_carHandle, SECT_REARWING,
                                       PRM_WINGAREA, (char*) NULL, 0.0);
    float rwingangle = SafeParmGetNum(pCar->_carHandle, SECT_REARWING,
                                        PRM_WINGANGLE, (char*) NULL, 0.0);
	float fwingArea = rwingarea * sin(rwingangle);
	float rwingArea = fwingarea * sin(fwingangle);
    float wingca = 1.23 * (fwingArea + rwingArea);

    float cl = SafeParmGetNum(pCar->_carHandle, SECT_AERODYNAMICS,
                             PRM_FCL, (char*) NULL, 0.0) +
                SafeParmGetNum(pCar->_carHandle, SECT_AERODYNAMICS,
                             PRM_RCL, (char*) NULL, 0.0);
    float h = 0.0;
    char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL,
                          SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
    {for( int i = 0; i < 4; i++ )
	{
        h += SafeParmGetNum(pCar->_carHandle, WheelSect[i],
                          PRM_RIDEHEIGHT, (char*) NULL, 0.20f);
	}}
    h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
    m_cm[PATH_NORMAL].CA = h*cl + 4.0*wingca;
	m_cm[PATH_NORMAL].CA_FW = 4 * 1.23 * fwingArea;
	m_cm[PATH_NORMAL].CA_RW = 4 * 1.23 * rwingArea;
	m_cm[PATH_NORMAL].CA_GE = h * cl;

	DEBUGF( "CA %g   CA_FW %g   CA_RW %g   CA_GE %g\n",
			m_cm[PATH_NORMAL].CA, m_cm[PATH_NORMAL].CA_FW,
			m_cm[PATH_NORMAL].CA_RW, m_cm[PATH_NORMAL].CA_GE );

	double	cx = SafeParmGetNum(pCar->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*)NULL, 0.0);
	double	frontArea = SafeParmGetNum(pCar->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*)NULL, 0.0);

	m_cm[PATH_NORMAL].TYRE_MU   = 9999;
	m_cm[PATH_NORMAL].TYRE_MU_F = 9999;
	m_cm[PATH_NORMAL].TYRE_MU_R = 9999;
	{for( int i = 0; i < 4; i++ )
	{
		double	mu = SafeParmGetNum(pCar->_carHandle, WheelSect[i],
								  PRM_MU, (char*)NULL, 1.0);
		m_cm[PATH_NORMAL].TYRE_MU = MN(m_cm[PATH_NORMAL].TYRE_MU, mu);
		if( i < 2 )
			m_cm[PATH_NORMAL].TYRE_MU_F = MN(m_cm[PATH_NORMAL].TYRE_MU_F, mu);
		else
			m_cm[PATH_NORMAL].TYRE_MU_R = MN(m_cm[PATH_NORMAL].TYRE_MU_R, mu);
	}}

	double gripScaleF = GripFactor(pCar, true);
	double gripScaleR = GripFactor(pCar, false);
	m_cm[PATH_NORMAL].GRIP_SCALE_F	= gripScaleF;
	m_cm[PATH_NORMAL].GRIP_SCALE_R	= gripScaleR;
	m_cm[PATH_LEFT].GRIP_SCALE_F	= gripScaleF;
	m_cm[PATH_LEFT].GRIP_SCALE_R	= gripScaleR;
	m_cm[PATH_RIGHT].GRIP_SCALE_F	= gripScaleF;
	m_cm[PATH_RIGHT].GRIP_SCALE_R	= gripScaleR;

	DEBUGF( "CARMASS %g   TYRE_MU %g   TYRE_MU_F %g   TYRE_MU_R %g \n",
			m_cm[PATH_NORMAL].MASS, m_cm[PATH_NORMAL].TYRE_MU,
			m_cm[PATH_NORMAL].TYRE_MU_F, m_cm[PATH_NORMAL].TYRE_MU_R );
	DEBUGF( "NORMAL: MU_SC %g   KZ_SCALE %g   FLY_HEIGHT %g\n",
			m_cm[PATH_NORMAL].MU_SCALE, m_cm[PATH_NORMAL].KZ_SCALE, m_priv[PATH_NORMAL].FLY_HEIGHT );
	DEBUGF( "LEFT:   MU_SC %g   KZ_SCALE %g   FLY_HEIGHT %g\n",
			m_cm[PATH_LEFT].MU_SCALE, m_cm[PATH_LEFT].KZ_SCALE, m_priv[PATH_LEFT].FLY_HEIGHT );
	DEBUGF( "RIGHT:  MU_SC %g   KZ_SCALE %g   FLY_HEIGHT %g\n",
			m_cm[PATH_RIGHT].MU_SCALE, m_cm[PATH_RIGHT].KZ_SCALE, m_priv[PATH_RIGHT].FLY_HEIGHT );

//	m_track.CalcMaxSpeeds( CARMASS, CA, TYRE_MU, MU_SCALE, MU_DF_SCALE, pCar->_fuel );
//	m_track.PropagateBraking();

	m_cm[PATH_NORMAL].FUEL = 0;//pCar->_fuel;
	m_cm[PATH_NORMAL].CD_BODY = 0.645 * cx * frontArea;
	m_cm[PATH_NORMAL].CD_WING = wingca;
	m_cm[PATH_NORMAL].WIDTH = pCar->_dimension_y;
//	cm.CD = cx * (frontArea + wingArea);
//	cm.CD = cx * frontArea + wingca;
//	m_cm.CD = 0.645 * cx * frontArea;
	for( int p = PATH_LEFT; p <= PATH_RIGHT; p++ )
	{
		m_cm[p].FUEL	= m_cm[PATH_NORMAL].FUEL;
		m_cm[p].CD_BODY	= m_cm[PATH_NORMAL].CD_BODY;
		m_cm[p].CD_WING	= m_cm[PATH_NORMAL].CD_WING;
		m_cm[p].WIDTH	= m_cm[PATH_NORMAL].WIDTH;

		m_cm[p].MASS	= m_cm[PATH_NORMAL].MASS;

		m_cm[p].CA		= m_cm[PATH_NORMAL].CA;
		m_cm[p].CA_FW	= m_cm[PATH_NORMAL].CA_FW;
		m_cm[p].CA_RW	= m_cm[PATH_NORMAL].CA_RW;
		m_cm[p].CA_GE	= m_cm[PATH_NORMAL].CA_GE;

		m_cm[p].TYRE_MU		= m_cm[PATH_NORMAL].TYRE_MU;
		m_cm[p].TYRE_MU_F	= m_cm[PATH_NORMAL].TYRE_MU_F;
		m_cm[p].TYRE_MU_R	= m_cm[PATH_NORMAL].TYRE_MU_R;
	}

//	DEBUGF( "CD %g   wing CA %g\n", m_cm.CD_BODY, wingca );
	DEBUGF( "WING F %g    WING R %g   SPDC N %d    SPDC T %d\n",
			fwingangle * 180 / PI, rwingangle * 180 / PI,
			m_priv[PATH_NORMAL].SPDC_NORMAL, m_priv[PATH_NORMAL].SPDC_TRAFFIC );

	if( m_pShared->m_path[PATH_NORMAL].GetOptions().factors != m_priv[PATH_NORMAL].FACTORS ||
		m_pShared->m_path[PATH_LEFT].GetOptions().factors   != m_priv[PATH_LEFT].FACTORS ||
		m_pShared->m_path[PATH_RIGHT].GetOptions().factors  != m_priv[PATH_RIGHT].FACTORS ||
		m_pShared->m_pTrack != m_track.GetTrack() )
	{
		if( m_pShared->m_pTrack != m_track.GetTrack() )
		{
			m_pShared->m_pTrack = m_track.GetTrack();
			m_pShared->m_teamInfo.Empty();
		}

//		DEBUGF( "Generating smooth paths...\n" );
//		double	w = 0.5;//m_track.GetWidth() / 6;
//		w = MX(0.5, m_track.GetWidth() / 2 - 3);
//		double	w = m_priv[PATH_NORMAL].AVOID_WIDTH;

		ClothoidPath::Options options(m_priv[PATH_NORMAL].BUMP_MOD, 
									  m_priv[PATH_NORMAL].SAFETY_LIMIT,
									  m_priv[PATH_NORMAL].SAFETY_MULTIPLIER);
		options.factors = m_priv[PATH_NORMAL].FACTORS;
		options.quadSmoothIters = m_priv[PATH_NORMAL].QUAD_SMOOTH_ITERS;
		m_pShared->m_path[PATH_NORMAL].MakeSmoothPath( &m_track, m_cm[PATH_NORMAL], options);
		//m_pShared->m_path[PATH_NORMAL].Search( m_cm[PATH_NORMAL] );

		options = ClothoidPath::Options(m_priv[PATH_LEFT].BUMP_MOD, 
										m_priv[PATH_LEFT].SAFETY_LIMIT,
										m_priv[PATH_LEFT].SAFETY_MULTIPLIER, 999, 0);
		options.factors = m_priv[PATH_LEFT].FACTORS;
		options.quadSmoothIters = m_priv[PATH_LEFT].QUAD_SMOOTH_ITERS;
		m_pShared->m_path[PATH_LEFT].MakeSmoothPath( &m_track, m_cm[PATH_LEFT], options);

		options = ClothoidPath::Options(m_priv[PATH_RIGHT].BUMP_MOD, 
										m_priv[PATH_RIGHT].SAFETY_LIMIT,
										m_priv[PATH_RIGHT].SAFETY_MULTIPLIER, 0, 999);
		options.factors = m_priv[PATH_RIGHT].FACTORS;
		options.quadSmoothIters = m_priv[PATH_RIGHT].QUAD_SMOOTH_ITERS;
		m_pShared->m_path[PATH_RIGHT].MakeSmoothPath( &m_track, m_cm[PATH_RIGHT], options);
	}

//	m_path[PATH_NORMAL].MakeSmoothPath( &m_track );
	m_path[PATH_NORMAL] = m_pShared->m_path[PATH_NORMAL];
//	m_path[PATH_NORMAL].Initialise( &m_track, m_cm[PATH_NORMAL], 999, 999 );

	//if( index == 0 )
	{
		char	buf[1024];
		sprintf( buf, "drivers/mouse_2017/cars/%s/track-%s.spr", m_carName, m_trackName );
		bool loadedOk = m_path[PATH_NORMAL].LoadPath(buf);
		PRINTF( "loaded springs data: %s\n", loadedOk ? "true" : "false" );
	}
	m_path[PATH_NORMAL].CalcMaxSpeeds( m_cm[PATH_NORMAL] );
	m_path[PATH_NORMAL].PropagateBraking( m_cm[PATH_NORMAL] );

//#define BRAKING_TEST
#if defined(DEV) && defined(BRAKING_TEST)
	// code for track Tim Test 1
	m_path[PATH_NORMAL].GenMiddle();
	m_path[PATH_NORMAL].CalcMaxSpeeds( m_cm[PATH_NORMAL] );
	m_path[PATH_NORMAL].GetAt(700).maxSpd = m_path[PATH_NORMAL].GetAt(700).spd = 0;
	m_path[PATH_NORMAL].PropagateBraking(m_cm[PATH_NORMAL] );
#endif

//	m_path[PATH_LEFT].MakeSmoothPath( &m_track, 999, -0.5 );
	m_path[PATH_LEFT] = m_pShared->m_path[PATH_LEFT];
	m_path[PATH_LEFT].CalcMaxSpeeds( m_cm[PATH_LEFT] );
	m_path[PATH_LEFT].PropagateBraking( m_cm[PATH_LEFT] );

//	m_path[PATH_RIGHT].MakeSmoothPath( &m_track, -0.5, 999 );
	m_path[PATH_RIGHT] = m_pShared->m_path[PATH_RIGHT];
	m_path[PATH_RIGHT].CalcMaxSpeeds( m_cm[PATH_RIGHT] );
	m_path[PATH_RIGHT].PropagateBraking( m_cm[PATH_RIGHT] );

	for( int p = PATH_NORMAL; p <= PATH_RIGHT; p++ )
	{
		for( int i = 0; i < 2; i++ )
		{
			CarModel	pitCm(m_cm[p]);
			pitCm.BRAKE_MU_SCALE = 0.8;//0.95;
			m_pitPath[p][i].MakePath( pCar->race.pit, &m_path[p], pitCm, i == 1,
									  m_priv[p].PIT_ENTRY_OFFSET, m_priv[p].PIT_EXIT_OFFSET );
		}
	}
/*
	{
		bool	same = true;
		int		via  = -1;
		const int	NSEG = m_track.GetSize();
		{for( int i = 0; i < NSEG; i++ )
		{
			const Path::PathPt&	n = m_path[PATH_NORMAL].GetAt(i);
			const Path::PathPt&	l = m_path[PATH_LEFT].GetAt(i);
			bool	newSame = fabs(n.offs - l.offs) < 0.25;
			if( same & !newSame )
			{
				via = i;
				DEBUGF( "%4d  VIA L\n", i );
			}
			else if( !same && newSame )
			{
				DEBUGF( "%4d  JOIN L     %.4f    %.4f\n", i,
						m_path[PATH_NORMAL].CalcEstimatedTime(via, i - via),
						m_path[PATH_LEFT].CalcEstimatedTime(via, i - via) );
			}
			same = newSame;
		}}
	}
*/
//	double	lapTime = m_path[PATH_NORMAL].CalcEstimatedLapTime();
//	DEBUGF( "Estimated lap time (%g): %02d:%02d.%03d\n",
//		lapTime, int(floor(lapTime / 60)), int(floor(lapTime)) % 60,
//		int((lapTime - floor(lapTime)) * 1000) );

    const char* traintype = GfParmGetStr(pCar->_carHandle,
										 SECT_DRIVETRAIN, PRM_TYPE, VAL_TRANS_RWD);
	m_driveType = cDT_RWD;
    if( strcmp(traintype, VAL_TRANS_RWD) == 0 )
        m_driveType = cDT_RWD;
    else if( strcmp(traintype, VAL_TRANS_FWD) == 0 )
        m_driveType = cDT_FWD;
    else if( strcmp(traintype, VAL_TRANS_4WD) == 0 )
        m_driveType = cDT_4WD;

	m_flying = 0;
	m_raceStart = true;
//	DEBUGF( "m_avoidS = %g (race start)\n", 0.0 );
	m_avoidS = 0;
	m_avoidT = 0;

	m_accBrkCoeff.Clear();
	m_accBrkCoeff.Sample( 0, 0 );
	m_accBrkCoeff.Sample( 1, 0.5 );

	TeamInfo::Item*	pItem = new TeamInfo::Item();
	pItem->index = pCar->index;
//	pItem->team = pCar->index / 2;
//	pItem->team = -1;
	pItem->teamName = pCar->_teamname;
	pItem->damage = pCar->_dammage;
//	pItem->pitState = TeamInfo::PIT_NOT_SHARED;
	pItem->lapsUntilPit = 999;
	pItem->usingPit = false;
	pItem->pOther = 0;
	pItem->pCar = pCar;
	m_pShared->m_teamInfo.Add( pCar->index, pItem );
}

bool	MyRobot::Pitting( int path, double pos ) const
{
	return	m_pitControl.WantToPit() &&
			m_pitPath[path][m_pitControl.PitType()].ContainsPos(pos);
}

bool	MyRobot::Pitting( tCarElt* car ) const
{
	double	pos = m_track.CalcPos(car);
	return Pitting(PATH_NORMAL, pos);
}

void	MyRobot::GetPtInfo( int path, double pos, PtInfo& pi ) const
{

	if( Pitting(path, pos) )
		m_pitPath[path][m_pitControl.PitType()].GetPtInfo( pos, pi );
	else
		m_path[path].GetPtInfo( pos, pi );
	// !!FOR TESTING ONLY!!
	//pi.spd = MN(pi.spd, 25);
}

void	InterpPtInfo( PtInfo& pi0, const PtInfo& pi1, double t )
{
//		pi.k	= piL.k    * (1 - t) + piR.k    * t;
		pi0.k	= Utils::InterpCurvature(pi0.k, pi1.k, t);
		double	deltaOAng = pi1.oang - pi0.oang;
		NORM_PI_PI(deltaOAng);
		pi0.oang = pi0.oang + deltaOAng * t;
		pi0.offs = pi0.offs * (1 - t) + pi1.offs * t;
		pi0.spd	 = pi0.spd  * (1 - t) + pi1.spd  * t;
		pi0.acc  = pi0.acc  * (1 - t) + pi1.acc  * t;
//		pi.toL
}

void	MyRobot::GetPosInfo(
	double		pos,
	PtInfo&		pi,
	double		u,
	double		v ) const
{
	GetPtInfo( PATH_NORMAL, pos, pi );

	PtInfo	piL, piR;

	if( u != 1 )
	{
		GetPtInfo( PATH_LEFT,  pos, piL );
		GetPtInfo( PATH_RIGHT, pos, piR );

		double	s = u;
		double	t = (v + 1) * 0.5;

		InterpPtInfo( piL, pi, s );
		InterpPtInfo( piR, pi, s );

		pi = piL;

		InterpPtInfo( pi, piR, t );
	}
}

void	MyRobot::GetPosInfo(
	double		pos,
	PtInfo&		pi ) const
{
	GetPosInfo(pos, pi, m_avoidS, m_avoidT);
}

double	MyRobot::CalcPathTarget( double pos, double offs, double s ) const
{
	PtInfo	pi, piL, piR;
	GetPtInfo( PATH_NORMAL, pos, pi );
	GetPtInfo( PATH_LEFT, pos, piL );
	GetPtInfo( PATH_RIGHT, pos, piR );

	InterpPtInfo( piL, pi, s );
	InterpPtInfo( piR, pi, s );

	double	t = (offs - piL.offs) / (piR.offs - piL.offs);

	return MX(-1, MN(t, 1)) * 2 - 1;
}

double	MyRobot::CalcPathTarget( double pos, double offs ) const
{
	return CalcPathTarget(pos, offs, m_avoidS);
}

Vec2d	MyRobot::CalcPathTarget2( double pos, double offs ) const
{
	PtInfo	pi, piL, piR;
	GetPtInfo( PATH_NORMAL, pos, pi );
	GetPtInfo( PATH_LEFT, pos, piL );
	GetPtInfo( PATH_RIGHT, pos, piR );

	double	s = m_avoidS;

	InterpPtInfo( piL, pi, s );
	InterpPtInfo( piR, pi, s );

	double	t = (offs - piL.offs) / (piR.offs - piL.offs);

	return Vec2d(MX(-1, MN(t, 1)) * 2 - 1, 1);
}

double	MyRobot::CalcPathOffset( double pos, double s, double t ) const
{
	PtInfo	pi, piL, piR;
	GetPtInfo( PATH_NORMAL, pos, pi );
	GetPtInfo( PATH_LEFT, pos, piL );
	GetPtInfo( PATH_RIGHT, pos, piR );

	InterpPtInfo( piL, pi, s );
	InterpPtInfo( piR, pi, s );

	InterpPtInfo( piL, piR, (t + 1) * 0.5 );

	return piL.offs;
}

void	MyRobot::CalcBestPathUV( double pos, double offs, double& u, double& v ) const
{
	PtInfo	pi, piL, piR;
	GetPtInfo( PATH_NORMAL, pos, pi );

	if( fabs(offs - pi.offs) < 0.01 )
	{
		u = 1;
		v = 0;
		return;
	}

	GetPtInfo( PATH_LEFT,  pos, piL );
	GetPtInfo( PATH_RIGHT, pos, piR );

	double	doffs = offs - pi.offs;
	if( doffs < 0 )
	{
		double	den = piL.offs - pi.offs;
		if( fabs(den) > 0.001 )
			u = 1 - MN(1, doffs / den);
		else
			u = 0;
		v = -1;
	}
	else
	{
		double	den = piR.offs - pi.offs;
		if( fabs(den) > 0.001 )
			u = 1 - MN(1, doffs / den);
		else
			u = 0;
		v = 1;
	}
//	double	out = CalcPathOffset(pos, u, v);
//	DEBUGF( "CalcBestPathUV %5.1f:%4.1f  doffs %5.1f  %5.1f/%5.1f/%5.1f  %5.3f %6.3f : %6.3f\n",
//			pos, offs, doffs, piL.offs, pi.offs, piR.offs, u, v, out );
}

double	MyRobot::CalcBestSpeed( double pos, double offs ) const
{
	double	u, v;
	CalcBestPathUV( pos, offs, u, v );

	PtInfo	pi;
	GetPosInfo( pos, pi, u, v );

	return pi.spd;
}

void	MyRobot::GetPathToLeftAndRight( const CarElt* pCar, double& toL, double& toR ) const
{
	double	pos = pCar->_distFromStartLine;
	double	offs = -pCar->_trkPos.toMiddle;

	PtInfo	pi;
	GetPtInfo( PATH_LEFT, pos, pi );
	toL = -(pi.offs - offs);
	GetPtInfo( PATH_RIGHT, pos, pi );
	toR = pi.offs - offs;
}

double	MyRobot::GripFactor( const CarElt* pCar, bool front ) const
{
	const double initialTemperature	= 273.15 + 20.0;

//	if( pCar->priv.wheel[2].currentTemperature != initialTemperature )
//		return 1.0;

	double	gripFactor = 0;
	int		wheel_base = front ? 0 : 2;
	for( int i = 0; i < 2; i++ )
	{
		int w = i + wheel_base;
		double currentTemperature	= pCar->priv.wheel[w].currentTemperature;
		double idealTemperature		= pCar->info.wheel[w].idealTemperature;
		double currentGraining		= pCar->priv.wheel[w].currentGraining;

		tdble di = (currentTemperature - idealTemperature)/(idealTemperature - initialTemperature);
		gripFactor += ((1.0f-(MIN((di*di), 1.0f)))/4.0f + 3.0f/4.0f)*(1.0f - currentGraining/10.0f);
	}

	return gripFactor * 0.5;
}

double	CalcMaxSlip( double fRatio )
{
//	double	stmp = MIN(slip, 1.5f);
//	Bx = wheel->mfB * stmp;
//	F = sin(wheel->mfC * atan(Bx * (1.0f - wheel->mfE) +
//								wheel->mfE * atan(Bx))) *
//			(1.0f + stmp * simSkidFactor[car->carElt->_skillLevel]);
//
//	f = sin(C + atan(Bx * (1 - E) + E * atan(Bx)))
//
//	s = 0, f = 0
//	s = 0.75, f = 0.75
//	s = 1.25, f = 1
//	s = 10, f = 1
//
//	sa = yaw - velAng;
//	slip = tan(sa) * vel;
//	sr = slip / vel == tan(sa) * vel / vel == tan(sa)
	return 0;
}

double	MyRobot::SteerAngle0( tCarElt* car, PtInfo& pi, PtInfo& aheadPi, const Private& priv )
{
	// work out current car speed.
	double	spd0 = hypot(car->_speed_x, car->_speed_y);
//	double	spd0 = car->_speed_x;

	// get current pos on track.
	double	pos = m_track.CalcPos(car);//, car->_dimension_x * 0.025);
//	double	pos = m_track.CalcPos(car, car->_dimension_x * 0.2);

	// look this far ahead.
	double	aheadDist = car->_dimension_x * 0.5 + spd0 * 0.02;
//	if( m_flying )
//		aheadDist += 10;
	double	aheadPos = m_track.CalcPos(car, aheadDist);

	// get info about pts on track.
	GetPosInfo( pos, pi );
	GetPosInfo( aheadPos, aheadPi );

	PtInfo	piOmega;
	double	aheadOmega = car->_dimension_x * 0.5 + spd0 * 0.02;// * 10;
	double	aheadOmegaPos = m_track.CalcPos(car, aheadOmega);
	GetPosInfo( aheadOmegaPos, piOmega );
//	static double avgDelta = 0;

	// work out basic steering angle.
	double	velAng = atan2(car->_speed_Y, car->_speed_X);
	double	angle = aheadPi.oang - car->_yaw;
	double	delta = pi.offs + car->_trkPos.toMiddle;
//	avgDelta = avgDelta * 0.5 + delta * 0.5;
//	delta = avgDelta;
	NORM_PI_PI(angle);
//	angle *= 0.75;
//	if( car->_speed_y * delta > 0 )
//		angle *= 1.01;
//	angle = tanh(2 * angle) * 0.5;

	// control rotational velocity.
//	double	avgK = (pi.k + aheadPi.k) * 0.5;
	double	avgK = (pi.k + piOmega.k) * 0.5;
//	double	avgK = pi.k;
//	double	avgK = pi.k;//(pi.k + aheadPi.k) * 0.5;
//	double	avgK = Utils::InterpCurvatureLin(pi.k, aheadPi.k, 0.5);
//	double	avgK = Utils::InterpCurvatureRad(pi.k, aheadPi.k, 0.5);
	double	targetOmega = car->_speed_x * avgK;//aheadPi.k;
	double	o2 = (aheadPi.k - pi.k) * spd0 / aheadDist;
	static PidController	yawCtrl;
	yawCtrl.m_p = 0.12;
	yawCtrl.m_d = 0.012;
	
//	if( fabs(pi.k - aheadPi.k) < 0.0025 )
	{
//		double omega = spd0 * avgK;//aheadPi.k;
//		angle += 0.02 * (targetOmega - car->_yaw_rate);
//		angle += 0.04 * (targetOmega - car->_yaw_rate);
//		angle += 0.05 * (targetOmega - car->_yaw_rate);
		angle += 0.08 * (targetOmega - car->_yaw_rate);
//		angle += 0.10 * (targetOmega - car->_yaw_rate);
//		angle += 0.12 * (targetOmega - car->_yaw_rate);
//		angle += 0.20 * (targetOmega - car->_yaw_rate);
//		angle += 0.25 * (targetOmega - car->_yaw_rate);
//		angle += yawCtrl.Sample(targetOmega - car->_yaw_rate);
	}

	angle += o2 * 0.08;

	if( car->_accel_x > 0 )
		angle += avgK * m_priv[PATH_NORMAL].STEER_K_ACC;
	else
		angle += avgK * m_priv[PATH_NORMAL].STEER_K_DEC;

	{
		double	velAng = atan2(car->_speed_Y, car->_speed_X);
		double	ang = car->_yaw - velAng;
		NORM_PI_PI(ang);

		int	k = int(floor((pi.k - K_MIN) / K_STEP));
		int	s = int(floor((spd0 - SPD_MIN) / SPD_STEP));
		double	ae = 0;
		if( k >= 0 && k < K_N && s >= 0 && s < SPD_N )
		{
			ae = m_angle[s][k] - ang;
//			NORM_PI_PI(ae);
		}

//		DEBUGF( "k %g  delta %g\n", pi.k, delta );
//		if( fabs(delta) > 0.2 && pi.k * delta < 0 )
//		{
//			double	ang = MX(0, MN((fabs(delta) - 0.2) * 0.2, 0.05));
//			angle += ang * SGN(pi.k);
//		}

//		m_angControl.m_p = 1;
//		m_angControl.m_d = 0;//10;
//		angle -= atan(m_angControl.Sample(ae));
//		angle += (ae + ang) * 0.5;
	}

//	DEBUGF( "k change %7.4f  omega %7.4f  o2 %7.4f   yaw_rate %7.4f\n",
//			pi.k - aheadPi.k, targetOmega, o2, car->_yaw_rate );

	// control offset from path.
	m_lineControl.m_p = 1.0;
	m_lineControl.m_d = 10;
//	const double SC = 0.15;
//	const double SC = 0.2;
	const double SC = priv.STEER_0_LINE_SCALE;
	double lineAngle = SC * atan(m_lineControl.Sample(delta));
/*	if( lineAngle < -0.1 )
		lineAngle = -0.1;
	else if( lineAngle > 0.1 )
		lineAngle = 0.1;*/
	angle -= lineAngle;

	double	frontSlipSide = (car->priv.wheel[0].slipSide  + car->priv.wheel[1].slipSide)  / 2;
	if( fabs(frontSlipSide) > 8 )
		DEBUGF( "slip: front(tan=%7.3f side=%7.3f) rear(tan=%7.3f side=%7.3f) acc(tan=%7.3f side=%7.3f)  steer=%g\n",
				(car->priv.wheel[0].slipAccel + car->priv.wheel[1].slipAccel) / 2,
				frontSlipSide,
				(car->priv.wheel[2].slipAccel + car->priv.wheel[3].slipAccel) / 2,
				(car->priv.wheel[2].slipSide  + car->priv.wheel[3].slipSide)  / 2,
				car->pub.DynGC.acc.x, car->pub.DynGC.acc.y, angle * 180 / PI);

	const double steerHardFactor = 0.5;
	bool steeringHard = fabs(angle) > car->info.steerLock * steerHardFactor;
	bool limitSteering =	fabs(frontSlipSide) > 6 &&
							steeringHard &&
							frontSlipSide * angle < 0;
	if( limitSteering )
	{
		angle = (angle < 0 ? -car->info.steerLock : car->info.steerLock) * steerHardFactor;
	}
	
	return angle;
}

double	MyRobot::SteerAngle1( tCarElt* car, PtInfo& pi, PtInfo& aheadPi )
{
	// work out current car speed.
	double	spd0 = hypot(car->_speed_x, car->_speed_y);
//	double	spd0 = car->_speed_x;

	// calc x,y coords of mid point on frt axle.
	double	midPt = 1.37;//1;//car->_dimension_x * 0.5;
//	double	midPt = car->_dimension_x * 0.5;
	double	x = car->pub.DynGCg.pos.x + midPt * cos(car->_yaw);
	double	y = car->pub.DynGCg.pos.y + midPt * sin(car->_yaw);

	tTrkLocPos	trkPos;
	RtTrackGlobal2Local(car->_trkPos.seg, x, y, &trkPos, 0);
	double	toMiddle = trkPos.toMiddle;

	// get curret pos on track.
	double	pos = m_track.CalcPos(trkPos);

	// look this far ahead.
//	double	aheadDist = 2 * spd0 * 0.02;
	double	aheadDist = spd0 * 0.02;
	double	aheadPos = m_track.CalcPos(trkPos, aheadDist);

	// get info about pts on track.
	GetPosInfo( pos, pi );
	GetPosInfo( aheadPos, aheadPi );

	double	angle = aheadPi.oang - car->_yaw;
	NORM_PI_PI(angle);

//	DEBUGF( "**** offs %6.3f  delta %6.3f   avoidT %5.2f\n",
//			pi.offs, delta, m_avoidT );

//	double	avgK = (pi.k + aheadPi.k) * 0.5;
//	double	avgK = pi.k;//(pi.k + aheadPi.k) * 0.5;
	double	avgK = aheadPi.k;
//	double	avgK = Utils::InterpCurvatureLin(pi.k, aheadPi.k, 0.5);
//	double	avgK = Utils::InterpCurvatureRad(pi.k, aheadPi.k, 0.5);
	double omega = car->_speed_x * avgK;//aheadPi.k;
//	double omega = spd0 * avgK;//aheadPi.k;
//	angle += 0.02 * (omega - car->_yaw_rate);
	angle += 0.08 * (omega - car->_yaw_rate);
/*
	omega = spd0 * avgK - car->_yaw_rate;//aheadPi.k;
	double	dist1s = spd0;
//	omega = car->_speed_x * avgK - car->_yaw_rate;//aheadPi.k;
//	double	dist1s = car->_speed_x;
	double	len = 2.63;	// dist between front/back wheel centres.
	double	radiusRear = dist1s / omega;
	angle += atan(len / radiusRear);
*/
	// control offset from path.
	m_lineControl.m_p = 1.0;
	m_lineControl.m_d = 10;
	m_lineControl.m_i = 0;//0.02;
	m_lineControl.m_totalRate = 0;
	m_lineControl.m_maxTotal = 2;
//	const double SC = 0.15;//0.15;//0.15;//0.2;//0.15;
	const double SC = MN(1, 8.5 / spd0);
	double	delta = pi.offs + toMiddle;
//	angle -= SC * atan(m_lineControl.Sample(delta));
	angle -= SC * tanh(m_lineControl.Sample(delta));

//	DEBUGF( "offset %6.2f tot %g\n", delta, m_lineControl.m_total );

	return angle;
}

double	MyRobot::SteerAngle2( tCarElt* car, PtInfo& pi, PtInfo& aheadPi )
{
	// work out current car speed.
	double	spd0 = hypot(car->_speed_x, car->_speed_y);
//	double	spd0 = car->_speed_x;

	// calc x,y coords of mid point on frt axle.
	double	midPt = 1.37;//1;//car->_dimension_x * 0.5;
//	double	midPt = car->_dimension_x * 0.5;
	double	x = car->pub.DynGCg.pos.x + midPt * cos(car->_yaw);
	double	y = car->pub.DynGCg.pos.y + midPt * sin(car->_yaw);

	static double	oldX = x;
	static double	oldY = y;
	double	velX = (x - oldX) / 0.02;
	double	velY = (y - oldY) / 0.02;
	oldX = x;
	oldY = y;

//	DEBUGF( "%g %g    %g %g    yaw %g\n",
//			car->pub.DynGCg.pos.x, car->pub.DynGCg.pos.y, x, y, car->_yaw );

	tTrkLocPos	trkPos;
	RtTrackGlobal2Local(car->_trkPos.seg, x, y, &trkPos, 0);
	double	toMiddle = trkPos.toMiddle;


	// get curret pos on track.
	double	pos = m_track.CalcPos(trkPos);

	// look this far ahead.
//	double	aheadDist = car->_dimension_x * 0.5 + spd0 * 0.02;
//	double	aheadDist = (car->_dimension_x - 1.37) * 0.5 + spd0 * 0.02;
//	double	aheadDist = 1.37 + spd0 * 0.02;
//	double	aheadDist = 2 * spd0 * 0.02;
	double	aheadDist = spd0 * 0.02;
//	double	aheadDist = 2.0 + spd0 * 0.02;
	double	aheadPos = m_track.CalcPos(trkPos, aheadDist);

	// get info about pts on track.
	GetPosInfo( pos, pi );
	GetPosInfo( aheadPos, aheadPi );

	double	angle = aheadPi.oang - car->_yaw;
	NORM_PI_PI(angle);

//	double	velAng = atan2(velY, velX);
	double	velAng = atan2(car->_speed_Y, car->_speed_X);
	double	velAngCtrl = aheadPi.oang - velAng;
//	double	yawAng = car->_yaw - velAng;
	NORM_PI_PI(velAngCtrl);
	m_velAngControl.m_p = 1;//0.5;//1;
	m_velAngControl.m_d = 10;//25;
	velAngCtrl = m_velAngControl.Sample(velAngCtrl);
//	angle += velAngCtrl;
	angle += tanh(velAngCtrl);
//	angle += 0.5 * velAngCtrl;
//	angle += 0.05 * tanh(20 * velAngCtrl);

//	DEBUGF( "**** offs %6.3f  delta %6.3f   avoidT %5.2f\n",
//			pi.offs, delta, m_avoidT );

//	double	avgK = (pi.k + aheadPi.k) * 0.5;
//	double	avgK = pi.k;//(pi.k + aheadPi.k) * 0.5;
	double	avgK = aheadPi.k;
//	double	avgK = Utils::InterpCurvatureLin(pi.k, aheadPi.k, 0.5);
//	double	avgK = Utils::InterpCurvatureRad(pi.k, aheadPi.k, 0.5);
	double omega = car->_speed_x * avgK;//aheadPi.k;
//	double omega = spd0 * avgK;//aheadPi.k;
	angle += 0.02 * (omega - car->_yaw_rate);

	// control offset from path.
	m_lineControl.m_p = 1.0;
	m_lineControl.m_d = 10;
	const double SC = 0.15;//0.15;//0.15;//0.2;//0.15;
	double	delta = pi.offs + toMiddle;
//	angle -= SC * atan(m_lineControl.Sample(delta));
	angle -= SC * tanh(m_lineControl.Sample(delta));

	return angle;
}

double	MyRobot::SteerAngle3( tCarElt* car, PtInfo& pi, PtInfo& aheadPi )
{
	// work out current car speed.
	double	spd0 = hypot(car->_speed_x, car->_speed_y);

	// get curret pos on track.
	double	pos = m_track.CalcPos(car);

	// look this far ahead.
	double	aheadTime = 0.2;//5;
	double	aheadDist = spd0 * aheadTime;

	double	aheadPos = m_track.CalcPos(car, aheadDist);

	// get info about pts on track.
	GetPosInfo( pos, pi );
	GetPosInfo( aheadPos, aheadPi );

	// we are trying to control 4 things with the steering...
	//	1. the distance of the car from the driving line it is to follow.
	//	2. the gradient of the distance of car from driving line.
	//	3. the angle of the car (yaw).
	//	4. the rotation speed (omega) of the car.

	// we want the angle of the car to be aheadPi.oang in aheadTime seconds.
	//	our current rotation speed is car->_yaw_rate.  our current rotation
	//	angle is car->_yaw.

//	DEBUGF( "*****    yaw %g    g_yaw %g\n", car->_yaw, car->pub.DynGCg.pos.az );
//	DEBUGF( "*****    yaw %g    yaw_rate %g\n", car->_yaw, car->_yaw_rate );

//	static double	avgYawU = 0;
//	static double	avgAhdK = 0;
//	static double	avgYawS = 0;
//	avgYawU = avgYawU * 0.5 + car->_yaw_rate * 0.5;
//	avgAhdK = avgAhdK * 0.5 + aheadPi.k * 0.5;
//	double	ys = aheadPi.oang - car->_yaw;
//	NORM_PI_PI(ys);
//	avgYawS = avgYawS * 0.5 + ys * 0.5;

	// current yaw rate.
	double	yawU = car->_yaw_rate;
//	double	yawU = avgYawU;

	// future yaw rate required ahead.
	double	yawV = aheadPi.k * spd0;
//	double	yawV = avgAhdK * spd0;

	// future yaw required ahead (assuming current yaw to be 0).
	double	yawS = aheadPi.oang - car->_yaw;
//	double	yawS = avgYawS;
	NORM_PI_PI(yawS);

	// acceleration to apply.
	double	yawA = 2 * (yawS - yawU * aheadTime) / (aheadTime * aheadTime);
//	double	yawA = (yawS / aheadTime) - yawU;

	double	yaw1s = yawU + 0.5 * yawA;

//	DEBUGF( "**** yaw %.3f  yawU %.3f  yawV %.3f  yawS %.3f  yawA %.3f\n",
//			car->_yaw, yawU, yawV, yawS, yawA );

	// angle to steer to get desired yaw angle after timestep.
	double	dist1s = spd0;
	double	len = 2.63;	// dist between front/back wheel centres.
	double	radiusRear = dist1s / yaw1s;
//	double	angle = MX(-0.5, MN(atan(len / radiusRear), 0.5));
	double	angle = atan(len / radiusRear);

	if( spd0 < 1 )
		angle = 0;

//	DEBUGF( "**** yaw1s %g  distTs %.3f  radiusRear %.3f  angle %.3f\n",
//			yaw1s, dist1s, radiusRear, angle );

//	double omega = spd0 * avgK;//aheadPi.k;
//	angle += 0.02 * (omega - car->_yaw_rate);

	// control offset from path.
	const double SC1 = 1;
	m_lineControl.m_p = SC1 * 0.25;	// 1.0 == oscillates
	m_lineControl.m_d = SC1 * 2.5;	// 9.5 == oscillates
	double	delta = pi.offs + car->_trkPos.toMiddle;
	const double SC2 = 1.0 / SC1;
	angle -= SC2 * atan(m_lineControl.Sample(delta));
//	angle -= 0.75 * atan(m_lineControl.Sample(delta));
//	angle -= 0.15 * atan(delta);

	double	frontSlipSide = (car->priv.wheel[0].slipSide  + car->priv.wheel[1].slipSide)  / 2;
	if( fabs(frontSlipSide) > 8 )
		DEBUGF( "slip: front(tan=%7.3f side=%7.3f) rear(tan=%7.3f side=%7.3f) acc(tan=%7.3f side=%7.3f)  steer=%g\n",
				(car->priv.wheel[0].slipAccel + car->priv.wheel[1].slipAccel) / 2,
				frontSlipSide,
				(car->priv.wheel[2].slipAccel + car->priv.wheel[3].slipAccel) / 2,
				(car->priv.wheel[2].slipSide  + car->priv.wheel[3].slipSide)  / 2,
				car->pub.DynGC.acc.x, car->pub.DynGC.acc.y, angle * 180 / PI);

   {
        double  zforce  = car->_reaction[0];
        double  s       = car->_skid[0] / (zforce * 0.0002f);
        double  stmp	= MIN(s, 1.5f);
        double  sv      = hypot(car->_wheelSlipSide(0), car->_wheelSlipAccel(0));
        double  v       = sv / s;
        double  sx      = car->_wheelSlipAccel(0) / v;
        double  sy      = car->_wheelSlipSide(0) / v;
        double  sa      = asin(sy);

        double  wrl     = car->_wheelSpinVel(0) * car->_wheelRadius(0);
        //double  waz   = car->_steer;// + wheel->staticPos.az;
        //double  CosA  = cos(waz);
        //double  SinA  = sin(waz);
        if( s > 0.1754 )
			GfOut( "acc %6.2f  zf %6.1f  s %.6f  v %6.2f  sx %.6f  sy %.6f\n",
					car->pub.DynGC.acc.x, zforce, s, v, sx, sy );
    }

	const double steerHardFactor = 0.3;
	bool steeringHard = fabs(angle) > car->info.steerLock * steerHardFactor;
	bool limitSteering =	fabs(frontSlipSide) > 6 &&
							steeringHard &&
							frontSlipSide * angle < 0;
	if( limitSteering )
	{
		angle = (angle < 0 ? -car->info.steerLock : car->info.steerLock) * steerHardFactor;
	}
	
	return angle;
}

double	MyRobot::SteerAngle4( tCarElt* car, PtInfo& pi, PtInfo& aheadPi )
{
	// work out current car speed.
	double	spd0 = hypot(car->_speed_x, car->_speed_y);
//	double	spd0 = car->_speed_x;

	// get curret pos on track.
	double	pos = m_track.CalcPos(car);

	// look this far ahead.
	double	aheadDist = car->_dimension_x * 0.5 + spd0 * 0.02;
//	double	aheadDist = 1 + spd0 * 0.04;
	double	aheadPos = m_track.CalcPos(car, aheadDist);

	// get info about pts on track.
	GetPosInfo( pos, pi );
	GetPosInfo( aheadPos, aheadPi );

	//
	//	deal with yaw.
	//

    double	yawError = aheadPi.oang - car->_yaw;
	NORM_PI_PI(yawError);
//	double	yawERate = yawError - m_prevYawError;
	double	yawERate = car->pub.DynGC.vel.az;

	// PID with no integration term.
	const double Y_PROP = 0.1;//1;//2;//1.0;
	const double Y_DIFF = 2.5;//1;//10;
    const double Y_SC = 1;
	double angle = Y_SC * atan((Y_PROP * yawError + Y_DIFF * yawERate) / Y_SC);

	//
	//	deal with line.
	//

	double	lineError = -(pi.offs + car->_trkPos.toMiddle);
	double	lineERate = lineError - m_prevLineError;
	m_prevLineError = lineError;

//	DEBUGF( "**** offs %6.3f  delta %6.3f   avoidT %5.2f\n",
//			pi.offs, delta, m_avoidT );

	// PID with no integration term.
//	const double L_PROP = 0.025;//005;
//	const double L_DIFF = 0.5;	// 2 -- osc;
	const double L_PROP = 0;//0.15;//005;
	const double L_DIFF = 0;//1.5;	// 2 -- osc;
    const double L_SC = 0.15;//0.2;//0.15;
	angle += L_SC * atan((L_PROP * lineError + L_DIFF * lineERate) / L_SC);

	return angle;
}

void	MyRobot::SpeedControl0(
	double	targetSpd,
	double	spd0,
	double&	acc,
	double&	brk )
{
	if( m_lastBrk && m_lastTargV )
	{
		if( m_lastBrk > 0 )//|| (car->ctrl.accelCmd == -m_lastBrk) )
		{
			double	err = m_lastTargV - spd0;
			m_accBrkCoeff.Sample( err, m_lastBrk );
//			DEBUGF( "accbrk sample %g -> %g\n", err, m_lastBrk );
		}
		m_lastBrk = 0;
		m_lastTargV = 0;
	}

	if( spd0 - 0.25 > targetSpd )
	{
//		if( spd0 - 2 > targetSpd )
		if( spd0 > targetSpd )
//		if( spd0 * 0.9 > targetSpd )
//		if( spd0 - 1 > targetSpd )
		{
#if 1
			acc = 0.0;
			brk = spd0 < 50 ? 0.5 : 0.6;
#elif 1
			double	MAX_BRK = 0.9;
//			double	MAX_BRK = 0.75;
			double	err = spd0 - targetSpd;
			brk = MX(0, MN(m_accBrkCoeff.CalcY(err), MAX_BRK));
//			DEBUGF( "accbrk calcy  %g -> %g\n", err, t );
			acc = 0;

			m_lastBrk = brk;
			m_lastTargV = 0;
	
			if( brk > 0 )
			{
				if( targetSpd > 0 )
					m_lastTargV = spd0;
			}
#else
			if( spd0 - 2 < targetSpd )
				brk = 0.07;
			else if( spd0 - 3 < targetSpd )
				brk = 0.14;
			else if( spd0 - 4 < targetSpd )
				brk = 0.20;
			else if( spd0 - 5 < targetSpd )
				brk = 0.35;
			else if( spd0 - 6 < targetSpd )
				brk = 0.5;
			else
				brk = 0.9;//targetSpd == 0 ? 0.5 : 0.75;
//			brk = spd0 - 3 < targetSpd ? 0.14 : spd0 - 5 < targetSpd ? 0.25 : 0.5;
			acc = 0;
#endif
		}
		else
		{
			if( targetSpd > 1 )
			{
				// slow naturally.
				acc = MN(acc, 0.1);
			}
			else
			{
				acc = 0;
				brk = 0.1;
			}
		}
	}
#if 1
	else
	{
		double x = (10 + spd0) * (targetSpd - spd0) / 20;
		if (x > 0)
			acc = x;
	}
#endif
/*
	if( m_lastBrk > 0 && brk > 0 && spd0 - 6 < targetSpd )
	{
//			brk -= 0.3;
//			if( brk < 0 )
		{
//				acc = MN(0.25, -brk);
			brk = 0;
			acc = 0.1;
		}
	}
*/
	m_lastBrk = brk;
	m_lastTargV = 0;
}

void	MyRobot::SpeedControl5(
	double	targetSpd,
	double	spd0,
	double	targetAcc,
	double	acc0,
	double	slip0,
	double&	acc,
	double&	brk,
	bool	traffic )
{
//	targetSpd -= 0.5;
//	if( spd0 - 0.25 > targetSpd )
	double ta = MN(m_brk.adjAccForSpdError(targetAcc, spd0, targetSpd), 0);
	if( ta < 0 )
	{
		if( spd0 > targetSpd )
		{
			m_brk.execute5( acc0, ta, slip0, m_priv[PATH_NORMAL].BRAKE_LIMIT, traffic );
			acc = 0.0;
			brk = m_brk.targetBrk;
		}
		else
		{
			m_brk.clear();
			if( targetSpd > 1 )
			{
				// slow naturally.
//				acc = MN(acc, 0.1);
				acc = MN(acc, traffic ? 0.1 : 0.11 * (targetSpd - spd0));
			}
			else
			{
				acc = 0;
				brk = 0.1;
			}
		}
	}
	else
	{
		m_brk.clear();
		double x = (10 + spd0) * (targetSpd - spd0) / 20;
		if (x > 0)
			acc = MN(x, 1);
	}
}

void	MyRobot::SpeedControl6(
	double	targetSpd,
	double	spd0,
	double	targetAcc,
	double	acc0,
	double	slip0,
	double&	acc,
	double&	brk,
	bool	traffic )
{
	if( spd0 > targetSpd || spd0 + 0.5 > targetSpd && targetAcc < -5 )
	{
		double ta = MN(m_brk.adjAccForSpdError(targetAcc, spd0, targetSpd), 0);

		m_brk.execute6( acc0, ta, slip0, m_priv[PATH_NORMAL].BRAKE_LIMIT, traffic );

		acc = 0.0;
		brk = m_brk.targetBrk;
	}
	else
	{
		m_brk.clear();
		double x = (10 + spd0) * (targetSpd - spd0) / 20;
		if (x > 0)
			acc = MN(x, 1);
	}

	m_lastBrk = brk;
	m_lastTargV = 0;
}

void	MyRobot::SpeedControl7(
	double	targetSpd,
	double	spd0,
	double	targetAcc,
	double	acc0,
	double	xslip0,
	double	slip0,
	double&	acc,
	double&	brk,
	bool	traffic )
{
//	targetSpd -= 0.5;
//	if( spd0 - 0.25 > targetSpd )
	double ta = MN(m_brk.adjAccForSpdError(targetAcc, spd0, targetSpd), 0);
	if( ta < 0 )
	{
		if( spd0 > targetSpd )
		{
			m_brk.execute7( acc0, ta, xslip0, slip0, m_priv[PATH_NORMAL].BRAKE_LIMIT, traffic );
			acc = 0.0;
			brk = m_brk.targetBrk;
		}
		else
		{
			m_brk.clear();
			if( targetSpd > 1 )
			{
				// slow naturally.
//				acc = MN(acc, 0.1);
				acc = MN(acc, traffic ? 0.1 : 0.11 * (targetSpd - spd0));
			}
			else
			{
				acc = 0;
				brk = 0.1;
			}
		}
	}
	else
	{
		m_brk.clear();
		double x = (10 + spd0) * (targetSpd - spd0) / 20;
		if (x > 0)
			acc = MN(x, 1);
	}
}

void	MyRobot::SpeedControl8(
	double	targetSpd,
	double	spd0,
	double	targetAcc,
	double	acc0,
	double	fslip0,
	double	rslip0,
	double&	acc,
	double&	brk,
	bool	traffic )
{
//	targetSpd -= 0.5;
//	if( spd0 - 0.25 > targetSpd )
	double ta = MN(m_brk.adjAccForSpdError(targetAcc, spd0, targetSpd), 0);
	if( ta < 0 )
	{
		if( spd0 > targetSpd )
		{
			m_brk.execute8( acc0, ta, fslip0, rslip0, m_priv[PATH_NORMAL].BRAKE_LIMIT, traffic );
			acc = m_brk.acc;
			brk = m_brk.targetBrk;
		}
		else
		{
			m_brk.clear();
			if( targetSpd > 1 )
			{
				// slow naturally.
//				acc = MN(acc, 0.1);
				acc = MN(acc, traffic ? 0.1 : 0.11 * (targetSpd - spd0));
			}
			else
			{
				acc = 0;
				brk = 0.1;
			}
		}
	}
	else
	{
		m_brk.clear();
		double x = (10 + spd0) * (targetSpd - spd0) / 20;
		if (x > 0)
			acc = x;
	}
}

void	MyRobot::SpeedControl1(
	double	targetSpd,
	double	spd0,
	double&	acc,
	double&	brk )
{
	if( spd0 > targetSpd )
	{
		if( spd0 - 1 > targetSpd )
		{
			if( spd0 - 2 < targetSpd )
				brk = 0.07;
			else if( spd0 - 3 < targetSpd )
				brk = 0.14;
			else if( spd0 - 4 < targetSpd )
				brk = 0.20;
			else if( spd0 - 5 < targetSpd )
				brk = 0.25;
			else if( spd0 - 5 < targetSpd )
				brk = 0.5;
			else
				brk = 0.5;//targetSpd == 0 ? 0.5 : 0.75;
//			brk = spd0 - 3 < targetSpd ? 0.14 : spd0 - 5 < targetSpd ? 0.25 : 0.5;
			acc = 0;
		}
		else
		{
			if( targetSpd > 1 )
				// slow naturally.
				acc = MN(acc, 0.25);
			else
			{
				acc = 0;
				brk = 0.1;
			}
		}
	}

	m_lastTargV = 0;
}

void	MyRobot::SpeedControl2(
	double	targetSpd,
	double	spd0,
	double&	acc,
	double&	brk )
{
	if( m_lastBrk && m_lastTargV )
	{
		if( m_lastBrk > 0 )//|| (car->ctrl.accelCmd == -m_lastBrk) )
		{
			double	err = m_lastTargV - spd0;
			m_accBrkCoeff.Sample( err, m_lastBrk );
//			DEBUGF( "accbrk sample %g -> %g\n", err, m_lastBrk );
		}
		m_lastBrk = 0;
		m_lastTargV = 0;
	}

	if( spd0 > targetSpd )
//	if( targetSpd < 100 )
	{
//		if( spd0 - 1 > targetSpd )
		{
			double	MAX_BRK = 0.5;
//			double	MAX_BRK = 0.75;
			double	err = spd0 - targetSpd;
			brk = MX(0, MN(m_accBrkCoeff.CalcY(err), MAX_BRK));
//			DEBUGF( "accbrk calcy  %g -> %g\n", err, t );
			acc = 0;

			m_lastBrk = brk;
			m_lastTargV = 0;

			if( brk > 0 )
			{
				if( targetSpd > 0 )
					m_lastTargV = spd0;
			}

//			DEBUGF( "*** brake ***  spd0 %g   targ %g  brk %g   acc %g\n",
//					spd0, targetSpd, brk, acc );
		}
//		else
//			acc = MN(acc, 0.1);
	}
}

void	MyRobot::SpeedControl3(
	double	targetSpd,
	double	spd0,
	double&	acc,
	double&	brk )
{
	if( m_lastBrk && m_lastTargV )
	{
		double	err = spd0 - m_lastTargV;
		m_brkCoeff[m_lastB] += err * 0.001;
//		DEBUGF( "*** bfk err %g   coeff %g\n", err, m_brkCoeff[m_lastB] );
		m_lastBrk = 0;
		m_lastTargV = 0;
	}

	if( spd0 > targetSpd )
	{
//		if( spd0 - 1 > targetSpd )
		{
			int		b = int(floor(spd0 / 2));
			double	MAX_BRK = 0.5;
			brk = MX(0, MN(m_brkCoeff[b] * (spd0 - targetSpd), MAX_BRK));
			acc = 0;
			m_lastB = b;
			m_lastBrk = brk;
			m_lastTargV = 0;

			if( brk > 0 && brk < MAX_BRK)
			{
				if( targetSpd > 0 )
					m_lastTargV = targetSpd;
			}

//			DEBUGF( "*** brake ***  spd0 %g   targ %g  brk %g   acc %g\n",
//					spd0, targetSpd, brk, acc );
		}
//		else
//			acc = 0.1;
	}
}

void	MyRobot::SpeedControl4(
	double	targetSpd,
	double	spd0,
	double	k,
	CarElt*	car,
	double&	acc,
	double&	brk )
{
	if( m_lastBrk && m_lastTargV )
	{
		if( m_lastBrk > 0 || (car->ctrl.accelCmd == -m_lastBrk) )
		{
			double	err = m_lastTargV - spd0;
			m_accBrkCoeff.Sample( err, m_lastBrk );
//			DEBUGF( "accbrk sample %g -> %g\n", err, m_lastBrk );
		}
		m_lastBrk = 0;
		m_lastTargV = 0;
	}

//	if( spd0 > targetSpd )
	//if( targetSpd < 100 )
	{
//		if( spd0 - 1 > targetSpd )
		{
//			double	MAX_BRK = 0.5;
//			double	MAX_BRK = 0.75;
			double	MAX_BRK = fabs(k) < 0.0015 ? 0.9 :
							  fabs(k) < 0.0035 ? 0.6 : 0.5;
			double	err = spd0 - targetSpd;
			double	t = m_accBrkCoeff.CalcY(err);
//			DEBUGF( "accbrk calcy  %g -> %g\n", err, t );
//			DEBUGF( "brk k %9.6f  err %6.3f  t=%6.3f\n", k, err, t );
			if( t > 0 )
			{
				brk = MN(t, MAX_BRK);
				acc = 0;
			}
			else
			{
				brk = 0;
				acc = MN(-t, 1);
			}
			m_lastBrk = t;
			m_lastTargV = 0;

//			if( brk > 0 && brk < MAX_BRK )
			if( t > -1 && t < MAX_BRK )
			{
				if( targetSpd > 0 )
					m_lastTargV = spd0;
			}

//			DEBUGF( "*** brake ***  spd0 %g   targ %g  brk %g   acc %g\n",
//					spd0, targetSpd, brk, acc );
		}
//		else
//			acc = 0.1;
	}
	
	double w1_speed = car->priv.wheel[0].spinVel * car->info.wheel[0].wheelRadius;
	double w2_speed = car->priv.wheel[1].spinVel * car->info.wheel[1].wheelRadius;
	double w3_speed = car->priv.wheel[2].spinVel * car->info.wheel[2].wheelRadius;
	double w4_speed = car->priv.wheel[3].spinVel * car->info.wheel[3].wheelRadius;

	double front_speed = (w1_speed + w2_speed) * 0.5;	
	double left_spin_speed  = w3_speed - front_speed;
	double right_spin_speed = w4_speed - front_speed;
	double max_spin_speed = MX(left_spin_speed, right_spin_speed);

	if( max_spin_speed > 2 )
	{
		_acc -= 0.01 * (max_spin_speed - 2);
	}
	else if( targetSpd > spd0 )
	{
		_acc += 0.01 * (targetSpd - spd0);
	}
	else if( targetSpd < spd0 )
	{
		_acc -= 0.01 * (targetSpd - spd0);
	}
	
	_acc = MX(0, MN(_acc, 1));
	//acc = _acc;
}

void	MyRobot::SpeedControl(
	int		which,
	double	targetSpd,
	double	spd0,
	double	targetAcc,
	double	acc0,
	double	fslip0,
	double	rxslip0,
	double	rslip0,
	double	k,
	CarElt*	car,
	double&	acc,
	double&	brk,
	bool	traffic )
{
//	SpeedControl2(targetSpd, spd0, acc, brk);
//	SpeedControl4(targetSpd, spd0, car, acc, brk);
	switch( which )
	{
		case 0:		SpeedControl0(targetSpd, spd0, acc, brk);			break;
		case 1:		SpeedControl1(targetSpd, spd0, acc, brk);			break;
		case 2:		SpeedControl2(targetSpd, spd0, acc, brk);			break;
		case 3:		SpeedControl3(targetSpd, spd0, acc, brk);			break;
		case 4:		SpeedControl4(targetSpd, spd0, k, car, acc, brk);	break;
		case 5:		SpeedControl5(targetSpd, spd0, targetAcc, acc0, rxslip0, acc, brk, traffic);			break;
		case 6:		SpeedControl6(targetSpd, spd0, targetAcc, acc0, rxslip0, acc, brk, traffic);			break;
		case 7:		SpeedControl7(targetSpd, spd0, targetAcc, acc0, rxslip0, MX(fslip0, rslip0), acc, brk, traffic);	break;
		case 8:		SpeedControl8(targetSpd, spd0, targetAcc, acc0, fslip0, rslip0, acc, brk, traffic);		break;
		default:	SpeedControl3(targetSpd, spd0, acc, brk);			break;
	}
}

void	MyRobot::launchControlClutch( tCarElt* car, tSituation* s )
{
	static bool accel = true;
	static int gearChangeCounter = 0;

	if( car->pub.speed > 250.0 / 3.6 )
	{
		// hit 200 kph...
		accel = false;
	}

	if( !accel )
	{
		car->ctrl.accelCmd = 0.0;
		car->ctrl.brakeCmd = 1.0;
	}
	//car->ctrl.steer = -tan(car->pub.trkPos.toMiddle * 0.01);
	car->ctrl.steer = -car->pub.DynGC.pos.az;

	if( s->currentTime < 0 )
	{
		accel = true;
		car->ctrl.accelCmd = 1.0f;
		car->ctrl.brakeCmd = 0.0;
		car->ctrl.clutchCmd = 1.0;
		gearChangeCounter = 10;
		return;
	}

	double	wv = 0;
	int		count = 0;

	if( m_driveType == cDT_FWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(FRNT_LFT) * car->priv.wheel[FRNT_LFT].spinVel +
				car->_wheelRadius(FRNT_RGT) * car->priv.wheel[FRNT_RGT].spinVel;
		count += 2;
	}

	if( m_driveType == cDT_RWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(REAR_LFT) * car->priv.wheel[REAR_LFT].spinVel +
				car->_wheelRadius(REAR_RGT) * car->priv.wheel[REAR_RGT].spinVel;
		count += 2;
	}
	wv /= count;

	double delta = wv - car->pub.speed;
//	static double avgVal = 1.0;
//	double value = MX(0, MN(-(m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5, 1.5));
//	avgVal = value * 0.25 + avgVal * 0.75;
//	double delta = avgVal;
	double ddiff = delta - _prevDelta;
	_prevDelta = delta;

	//PRINTF( "wv %g, spd %g, diff %g, dc %d\n", wv, car->pub.speed, delta, _deltaCounter );
/*
	if( accel )
	{
		if( s->currentTime < 1 )
		{
			double rpmErr = car->priv.enginerpmRedLine * 0.98 - car->priv.enginerpm;
			car->ctrl.accelCmd = MX(0, MN(car->ctrl.accelCmd + 0.001 * rpmErr, 1.0));
		}
		else
			car->ctrl.accelCmd = 1;
	}
*/
	if( car->ctrl.clutchCmd > 0 || gearChangeCounter > 0 || delta > 4 )
	{
		if( s->currentTime < 0.2 )
		{
			car->ctrl.clutchCmd = 0.75f;
			_deltaCounter = 5;
		}
		else if( (delta > 0.1 || _deltaCounter > 0) && car->priv.gear < 3 )
		{
			if( delta > 3 )
				_deltaCounter = 5;
			else
				_deltaCounter = MX(0, _deltaCounter - 1);

			//if( _deltaCounter > 0 )
				car->ctrl.clutchCmd = MX(0, MN(car->ctrl.clutchCmd + (delta - 4) * 0.02 + ddiff * 0.04, 1));
			//else
				//car->ctrl.clutchCmd = 0;
//			DEBUGF("adj clutch err %g  ddiff %g\n", err, ddiff );
		}
		else if( gearChangeCounter > 0 )
		{
			//car->ctrl.clutchCmd = MX(0, MN(car->ctrl.clutchCmd - 0.05, 0.5));
			//car->ctrl.clutchCmd = MX(0, MN(car->ctrl.clutchCmd - 0.01, 0.1));
			//car->ctrl.clutchCmd = 0.015 * gearChangeCounter;
			car->ctrl.clutchCmd = MX(0, gearChangeCounter * 0.02 - 0.05);
			//car->ctrl.clutchCmd = MX(0, MN(car->ctrl.clutchCmd - 0.1, 0.5));
			//car->ctrl.clutchCmd = MX(0, car->ctrl.clutchCmd - 0.5);
			//car->ctrl.clutchCmd = 0;
		}
		else
		{
			car->ctrl.clutchCmd = 0;
		}
	}

/*
	if( car->ctrl.clutchCmd > 0 )
	{
		//car->ctrl.clutchCmd = MX(0, MN(car->ctrl.clutchCmd - 0.05, 0.5));
		//car->ctrl.clutchCmd = MX(0, MN(car->ctrl.clutchCmd - 0.01, 0.1));
		//car->ctrl.clutchCmd = MX(0, MN(car->ctrl.clutchCmd - 0.02, 0.15));
		//car->ctrl.clutchCmd = MX(0, gearChangeCounter * 0.02 - 0.05);
		car->ctrl.clutchCmd = gearChangeCounter * 0.015;
		//car->ctrl.clutchCmd = MX(0, MN(car->ctrl.clutchCmd - 0.1, 0.5));
		//car->ctrl.clutchCmd = MX(0, car->ctrl.clutchCmd - 0.5);
		//car->ctrl.clutchCmd = 0;
	}
*/
/*
	double	wr = 0;
	int		count = 0;

	if( m_driveType == cDT_FWD || m_driveType == cDT_4WD )
	{
		wr += car->_wheelRadius(FRNT_LFT) + car->_wheelRadius(FRNT_RGT);
		count += 2;
	}

	if( m_driveType == cDT_RWD || m_driveType == cDT_4WD )
	{
		wr += car->_wheelRadius(REAR_LFT) + car->_wheelRadius(REAR_RGT);
		count += 2;
	}
	wr /= count;

	double	gr = car->_gearRatio[car->_gear + car->_gearOffset];
	double	rpmForSpd = gr * car->_speed_x / wr;
	double	rpm = car->_enginerpm;
//	DEBUGF( "RPM %3.0f  RPM2 %3.0f  %g\n", rpm, rpmForSpd, rpmForSpd / rpm );

	if( car->ctrl.clutchCmd > 0.5 )
	{
		car->ctrl.clutchCmd = 0.5;
	}
	else if( car->ctrl.clutchCmd == 0.5 )
	{
		if( rpmForSpd / rpm > 0.82 )
			car->ctrl.clutchCmd = 0.49f;
	}
	else
	{
		car->ctrl.clutchCmd -= 0.04f;
		if( car->ctrl.clutchCmd < 0 )
			car->ctrl.clutchCmd = 0;
	}
*/

	gearChangeCounter = MX(0, gearChangeCounter - 1);

	double acc = 0;
	int newGear = CalcGear(car, acc);
	if( newGear > car->ctrl.gear )
		gearChangeCounter = 10;
	car->ctrl.gear = newGear;

	//PRINTF( "time: %g  speed: %g  rpm: %g\n",
	//		s->currentTime, car->pub.speed, car->priv.enginerpm );

	static double lastSpd = 0;
	PRINTF( "%1.3f,%d,%6.3f,%4.0f,%5.3f,%5.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\n",
			s->currentTime,
			car->priv.gear,
			car->pub.speed,
			car->priv.enginerpm * 60 / (2 * PI),
			car->ctrl.accelCmd,
			car->ctrl.clutchCmd,
			(car->pub.DynGC.vel.x - lastSpd) / s->deltaTime,//car->pub.DynGC.acc.x,
			delta,
			car->priv.wheel[2].slipAccel,
			(m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5,
			(wv - car->pub.DynGC.vel.x) / car->pub.DynGC.vel.x );
	lastSpd = car->pub.DynGC.vel.x;
}

void	MyRobot::launchControlSimple( tCarElt* car, tSituation* s )
{
	static int ctrlCount = 0;

	car->ctrl.accelCmd = 1.0;
	car->ctrl.brakeCmd = 0.0;

	if( s->currentTime < 0 )
	{
		car->ctrl.clutchCmd = 0.75;
	}
	else
	{
		if( ctrlCount == 0 )
		{
			car->ctrl.gear = 1;
			car->ctrl.clutchCmd = 1.0;
		}
		else if( ctrlCount < 10 )
		{
			car->ctrl.clutchCmd = 1 - ctrlCount * 0.1;
		}
		
		ctrlCount++;
	}	

	double	wv = 0;
	int		count = 0;

	if( m_driveType == cDT_FWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(FRNT_LFT) * car->priv.wheel[FRNT_LFT].spinVel +
				car->_wheelRadius(FRNT_RGT) * car->priv.wheel[FRNT_RGT].spinVel;
		count += 2;
	}

	if( m_driveType == cDT_RWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(REAR_LFT) * car->priv.wheel[REAR_LFT].spinVel +
				car->_wheelRadius(REAR_RGT) * car->priv.wheel[REAR_RGT].spinVel;
		count += 2;
	}
	wv /= count;

	double delta = wv - car->pub.speed;
	double ddiff = delta - _prevDelta;
	_prevDelta = delta;

	PRINTF( "%1.3f,%d,%5.2f,%3.0f,%5.3f,%5.3f,%6.3f,%6.3f\n",
			s->currentTime,
			car->priv.gear,
			car->pub.speed,
			car->priv.enginerpm * 60 / (2 * PI),
			car->ctrl.accelCmd,
			car->ctrl.clutchCmd,
			car->pub.DynGC.acc.x,
			delta );
}

void	MyRobot::launchControlAcclerator( tCarElt* car, tSituation* s )
{
	static bool accel = true;
	static int gearChangeCounter = 0;

	if( car->pub.speed > 250.0 / 3.6 )
	{
		// hit 200 kph...
		accel = false;
	}

	if( !accel )
	{
		car->ctrl.accelCmd = 0.0;
		car->ctrl.brakeCmd = 0.5;
	}
	//car->ctrl.steer = -tan(car->pub.trkPos.toMiddle * 0.01);
	car->ctrl.steer = -car->pub.DynGC.pos.az;

	if( s->currentTime < 0 )
	{
		accel = true;
		car->ctrl.accelCmd = 1.0;
		car->ctrl.brakeCmd = 0.0;
		car->ctrl.clutchCmd = 1.0;
		gearChangeCounter = 10;
		return;
	}
	
	double	wv = 0;
	int		count = 0;

	if( m_driveType == cDT_FWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(FRNT_LFT) * car->priv.wheel[FRNT_LFT].spinVel +
				car->_wheelRadius(FRNT_RGT) * car->priv.wheel[FRNT_RGT].spinVel;
		count += 2;
	}

	if( m_driveType == cDT_RWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(REAR_LFT) * car->priv.wheel[REAR_LFT].spinVel +
				car->_wheelRadius(REAR_RGT) * car->priv.wheel[REAR_RGT].spinVel;
		count += 2;
	}
	wv /= count;

	if( car->ctrl.clutchCmd > 0 || gearChangeCounter > 0 )
	{
		if( gearChangeCounter > 0 )
		{
			car->ctrl.clutchCmd = MX(0, gearChangeCounter * 0.02 - 0.05);
		}
		else
		{
			car->ctrl.clutchCmd = 0;
		}
	}

	double delta = wv - car->pub.speed;
//	double delta = -(car->priv.wheel[2].slipAccel + car->priv.wheel[3].slipAccel) / 2;
//	double delta = -(m_cm[PATH_NORMAL][PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL][PATH_NORMAL].wheel(3).slipX()) * 0.5;
	double ddiff = delta - _prevDelta;
	_prevDelta = delta;


//	PRINTF( "wv %g, spd %g, diff %g, dc %d\n", wv, car->pub.speed, delta, _deltaCounter );

	if( accel )
	{
		/*if( s->currentTime < 0.1 )
			car->ctrl.clutchCmd = 0.75;
		else if( car->priv.enginerpm < car->priv.enginerpmRedLine * 0.8 && car->priv.gear == 1 )
		{
			double rpmErr = car->priv.enginerpmRedLine * 0.8 - car->priv.enginerpm;
			car->ctrl.clutchCmd = 0.5 + rpmErr * 0.001;
		}
		else
		{
			//car->ctrl.clutchCmd = 0;
		}*/
			
		/*if( s->currentTime < 0.2 )
		{
			car->ctrl.clutchCmd = 0.75 - s->currentTime * 2;
			car->ctrl.accelCmd = 1.0;
		}
		else*/ if( s->currentTime >= 0.2 && car->priv.gear < 3 )
		{
			double slipErr = 4.0 - delta;
			car->ctrl.accelCmd = MX(0, MN(car->ctrl.accelCmd + 0.013 * slipErr - 0.05 * ddiff, 1.0));

//			double slipErr = 0.175 - delta;
//			car->ctrl.accelCmd = MX(0, MN(car->ctrl.accelCmd + 0.1 * slipErr, 1.0));
		}
		else
		{
			car->ctrl.accelCmd = 1.0;
		}
	}

	gearChangeCounter = MX(0, gearChangeCounter - 1);

	double acc = 0;
	int newGear = CalcGear(car, acc);
	if( newGear > car->ctrl.gear )
		gearChangeCounter = 10;
	car->ctrl.gear = newGear;

	//PRINTF( "time: %g  speed: %g  rpm: %g\n",
	//		s->currentTime, car->pub.speed, car->priv.enginerpm );

	static double lastSpd = 0;
	PRINTF( "%1.3f,%d,%6.3f,%4.0f,%5.3f,%5.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\n",
			s->currentTime,
			car->priv.gear,
			car->pub.speed,
			car->priv.enginerpm * 60 / (2 * PI),
			car->ctrl.accelCmd,
			car->ctrl.clutchCmd,
			(car->pub.DynGC.vel.x - lastSpd) / s->deltaTime,//car->pub.DynGC.acc.x,
			delta,
			car->priv.wheel[2].slipAccel,
			(m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5,
			(wv - car->pub.DynGC.vel.x) / car->pub.DynGC.vel.x );
	lastSpd = car->pub.DynGC.vel.x;
}

void	MyRobot::launchControlAccSlip( tCarElt* car, tSituation* s )
{
	static bool accel = true;
	static int gearChangeCounter = 0;

	if( car->pub.speed > 250.0 / 3.6 )
	{
		// hit 200 kph...
		accel = false;
	}

	if( !accel )
	{
		car->ctrl.accelCmd = 0.0;
		car->ctrl.brakeCmd = 0.5;
	}
	//car->ctrl.steer = -tan(car->pub.trkPos.toMiddle * 0.01);
	car->ctrl.steer = -car->pub.DynGC.pos.az;

	if( s->currentTime < 0 )
	{
		accel = true;
		car->ctrl.accelCmd = 1.0;
		car->ctrl.brakeCmd = 0.0;
		car->ctrl.clutchCmd = 1.0;
		gearChangeCounter = 10;
		return;
	}
	
	double	wv = 0;
	int		count = 0;

	if( m_driveType == cDT_FWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(FRNT_LFT) * car->priv.wheel[FRNT_LFT].spinVel +
				car->_wheelRadius(FRNT_RGT) * car->priv.wheel[FRNT_RGT].spinVel;
		count += 2;
	}

	if( m_driveType == cDT_RWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(REAR_LFT) * car->priv.wheel[REAR_LFT].spinVel +
				car->_wheelRadius(REAR_RGT) * car->priv.wheel[REAR_RGT].spinVel;
		count += 2;
	}
	wv /= count;

	if( car->ctrl.clutchCmd > 0 || gearChangeCounter > 0 )
	{
		if( gearChangeCounter > 0 )
		{
			car->ctrl.clutchCmd = MX(0, gearChangeCounter * 0.02 - 0.05);
		}
		else
		{
			car->ctrl.clutchCmd = 0;
		}
	}

	double delta = wv - car->pub.speed;
//	double delta = -(car->priv.wheel[2].slipAccel + car->priv.wheel[3].slipAccel) / 2;
//	double delta = -(m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5;
	double ddiff = delta - _prevDelta;
	_prevDelta = delta;


//	PRINTF( "wv %g, spd %g, diff %g, dc %d\n", wv, car->pub.speed, delta, _deltaCounter );

	if( accel )
	{
		/*if( s->currentTime < 0.1 )
			car->ctrl.clutchCmd = 0.75;
		else if( car->priv.enginerpm < car->priv.enginerpmRedLine * 0.8 && car->priv.gear == 1 )
		{
			double rpmErr = car->priv.enginerpmRedLine * 0.8 - car->priv.enginerpm;
			car->ctrl.clutchCmd = 0.5 + rpmErr * 0.001;
		}
		else
		{
			//car->ctrl.clutchCmd = 0;
		}*/
			
		/*if( s->currentTime < 0.2 )
		{
			car->ctrl.clutchCmd = 0.75 - s->currentTime * 2;
			car->ctrl.accelCmd = 1.0;
		}
		else*/ if( s->currentTime >= 0.0 && car->priv.gear < 3 )
		{
//			double slipErr = 4.0 - delta;
			double targSpd = 0.195 * MX(5, car->pub.DynGC.vel.x);
			double slipErr = targSpd - delta;
			car->ctrl.accelCmd = MX(0, MN(car->ctrl.accelCmd + 0.013 * slipErr - 0.05 * ddiff, 1.0));

//			double slipErr = 0.175 - delta;
//			car->ctrl.accelCmd = MX(0, MN(car->ctrl.accelCmd + 0.1 * slipErr, 1.0));
		}
		else
		{
			car->ctrl.accelCmd = 1.0;
		}
	}

	gearChangeCounter = MX(0, gearChangeCounter - 1);

	double acc = 0;
	int newGear = CalcGear(car, acc);
	if( newGear > car->ctrl.gear )
		gearChangeCounter = 10;
	car->ctrl.gear = newGear;

	//PRINTF( "time: %g  speed: %g  rpm: %g\n",
	//		s->currentTime, car->pub.speed, car->priv.enginerpm );

	static double lastSpd = 0;
	PRINTF( "%1.3f,%d,%6.3f,%4.0f,%5.3f,%5.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\n",
			s->currentTime,
			car->priv.gear,
			car->pub.speed,
			car->priv.enginerpm * 60 / (2 * PI),
			car->ctrl.accelCmd,
			car->ctrl.clutchCmd,
			(car->pub.DynGC.vel.x - lastSpd) / s->deltaTime,//car->pub.DynGC.acc.x,
			delta,
			car->priv.wheel[2].slipAccel,
			(m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5,
			(wv - car->pub.DynGC.vel.x) / car->pub.DynGC.vel.x );
	lastSpd = car->pub.DynGC.vel.x;
}

void	MyRobot::launchControlAccSlip2( tCarElt* car, tSituation* s )
{
	static bool accel = true;
	static int gearChangeCounter = 0;

	if( car->pub.speed > 250.0 / 3.6 )
	{
		// hit 200 kph...
		accel = false;
	}

	if( !accel )
	{
		car->ctrl.accelCmd = 0.0;
		car->ctrl.brakeCmd = 0.5;
	}
	//car->ctrl.steer = -tan(car->pub.trkPos.toMiddle * 0.01);
	car->ctrl.steer = -car->pub.DynGC.pos.az;

	if( s->currentTime < 0 )
	{
		accel = true;
		car->ctrl.accelCmd = 1.0;
		car->ctrl.brakeCmd = 0.0;
		car->ctrl.clutchCmd = 1.0;
		gearChangeCounter = 10;
		return;
	}
	
	double	wv = 0;
	int		count = 0;

	if( m_driveType == cDT_FWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(FRNT_LFT) * car->priv.wheel[FRNT_LFT].spinVel +
				car->_wheelRadius(FRNT_RGT) * car->priv.wheel[FRNT_RGT].spinVel;
		count += 2;
	}

	if( m_driveType == cDT_RWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(REAR_LFT) * car->priv.wheel[REAR_LFT].spinVel +
				car->_wheelRadius(REAR_RGT) * car->priv.wheel[REAR_RGT].spinVel;
		count += 2;
	}
	wv /= count;

	if( car->ctrl.clutchCmd > 0 || gearChangeCounter > 0 )
	{
		if( gearChangeCounter > 0 )
		{
//			if( car->ctrl.gear <= 1 )
//				car->ctrl.clutchCmd = 0.7f;
//			else
				car->ctrl.clutchCmd = MX(0, gearChangeCounter * 0.02 - 0.05);
		}
		else
		{
			car->ctrl.clutchCmd = 0;
		}
	}

	double delta = wv - car->pub.speed;
//	double delta = -(car->priv.wheel[2].slipAccel + car->priv.wheel[3].slipAccel) / 2;
//	double delta = -(m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5;
	double ddiff = delta - _prevDelta;
	_prevDelta = delta;


//	PRINTF( "wv %g, spd %g, diff %g, dc %d\n", wv, car->pub.speed, delta, _deltaCounter );

	if( accel )
	{
		/*if( s->currentTime < 0.1 )
			car->ctrl.clutchCmd = 0.75;
		else if( car->priv.enginerpm < car->priv.enginerpmRedLine * 0.8 && car->priv.gear == 1 )
		{
			double rpmErr = car->priv.enginerpmRedLine * 0.8 - car->priv.enginerpm;
			car->ctrl.clutchCmd = 0.5 + rpmErr * 0.001;
		}
		else
		{
			//car->ctrl.clutchCmd = 0;
		}*/
			
		/*if( s->currentTime < 0.2 )
		{
			car->ctrl.clutchCmd = 0.75 - s->currentTime * 2;
			car->ctrl.accelCmd = 1.0;
		}
		else*/ if( s->currentTime >= 0.0 && car->priv.gear < 3 )
		{
//			double slipErr = 4.0 - delta;
			double targSpd = (car->priv.gear <= 1 ? 0.300 : 0.195) * MX(5, car->pub.DynGC.vel.x);
			double slipErr = targSpd - delta;
			car->ctrl.accelCmd = MX(0, MN(car->ctrl.accelCmd + 0.013 * slipErr - 0.05 * ddiff, 1.0));

//			double slipErr = 0.175 - delta;
//			car->ctrl.accelCmd = MX(0, MN(car->ctrl.accelCmd + 0.1 * slipErr, 1.0));
		}
		else
		{
			car->ctrl.accelCmd = 1.0;
		}
	}

	gearChangeCounter = MX(0, gearChangeCounter - 1);

	double acc = 0;
	int newGear = CalcGear(car, acc);
	if( newGear > car->ctrl.gear )
		gearChangeCounter = 10;
	car->ctrl.gear = newGear;

	//PRINTF( "time: %g  speed: %g  rpm: %g\n",
	//		s->currentTime, car->pub.speed, car->priv.enginerpm );
	if( s->currentTime < 1 )
	{
		static double cl = 0.9;
		car->ctrl.accelCmd = 1.0;
		car->ctrl.clutchCmd = cl;
//		if( car->priv.enginerpm * 60 / (2 * PI) > 8200 )
//			cl -= 0.05;
		car->ctrl.gear = 1;
	}

	static double lastSpd = 0;
	PRINTF( "%1.3f,%d,%6.3f,%4.0f,%5.3f,%5.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\n",
			s->currentTime,
			car->priv.gear,
			car->pub.speed,
			car->priv.enginerpm * 60 / (2 * PI),
			car->ctrl.accelCmd,
			car->ctrl.clutchCmd,
			(car->pub.DynGC.vel.x - lastSpd) / s->deltaTime,//car->pub.DynGC.acc.x,
			delta,
			car->priv.wheel[2].slipAccel,
			(m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5,
			(wv - car->pub.DynGC.vel.x) / car->pub.DynGC.vel.x );
	lastSpd = car->pub.DynGC.vel.x;

}

void	MyRobot::launchControlFullThrottle( tCarElt* car, tSituation* s )
{
	static bool accel = true;
	static int gearChangeCounter = 0;
	static int bcount = 0;

	if( car->pub.speed > 300.0 / 3.6 )
	{
		// hit 200 kph...
		accel = false;
	}

	if( !accel )
	{
		static double brk = 1.0;
		static double lastSlip = 0;
		double fslip = (m_cm[PATH_NORMAL].wheel(0).slipX() + m_cm[PATH_NORMAL].wheel(1).slipX()) * 0.5;
		double rslip = (m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5;
		double err = 0.175 - fslip;
		double dslip = fslip - lastSlip;
		brk += err * 0.2 - dslip * 1.5;
		brk = MX(0, MN(brk, 1));
		lastSlip = fslip;
/*
		//car->ctrl.brakeCmd = 0.4;
		//car->ctrl.brakeCmd = bcount % 4 < 3 ? 1.0 : 0.0;
		double vx = 3.6 * car->pub.DynGCg.vel.x;
		const double a = 0.000008702598466;
		const double b = -0.000595387978;
		const double c = 0.37;//0.4;
		double fslip = (m_cm[PATH_NORMAL].wheel(0).slipX() + m_cm[PATH_NORMAL].wheel(1).slipX()) * 0.5;
		double rslip = (m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5;
		double brk = a * vx * vx + b * vx + c;
		brk -= ((fslip + rslip) * 0.5 - 0.175) * 0.1;
*/
		car->ctrl.accelCmd = rslip < 0.175 ? 0.0 : 0.1;
		car->ctrl.brakeCmd = MX(0, MN(brk, 1));
		bcount += 1;
	}
	//car->ctrl.steer = -tan(car->pub.trkPos.toMiddle * 0.01);
	car->ctrl.steer = -car->pub.DynGC.pos.az;

	if( s->currentTime < 0 )
	{
		accel = true;
		car->ctrl.accelCmd = 1.0;
		car->ctrl.brakeCmd = 0.0;
		car->ctrl.clutchCmd = 1.0;
		gearChangeCounter = 10;
		return;
	}
	
	double	wv = 0;
	int		count = 0;

	if( m_driveType == cDT_FWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(FRNT_LFT) * car->priv.wheel[FRNT_LFT].spinVel +
				car->_wheelRadius(FRNT_RGT) * car->priv.wheel[FRNT_RGT].spinVel;
		count += 2;
	}

	if( m_driveType == cDT_RWD || m_driveType == cDT_4WD )
	{
		wv +=	car->_wheelRadius(REAR_LFT) * car->priv.wheel[REAR_LFT].spinVel +
				car->_wheelRadius(REAR_RGT) * car->priv.wheel[REAR_RGT].spinVel;
		count += 2;
	}
	wv /= count;

	if( car->ctrl.clutchCmd > 0 || gearChangeCounter > 0 )
	{
		if( gearChangeCounter > 0 )
		{
			car->ctrl.clutchCmd = MX(0, gearChangeCounter * 0.02 - 0.05);
//			car->ctrl.clutchCmd = gearChangeCounter > 0 ? 0.01 : 0.0;
		}
		else
		{
			car->ctrl.clutchCmd = 0;
		}
	}

	if( accel )
	{
		car->ctrl.accelCmd = 1.0;
	}

	gearChangeCounter = MX(0, gearChangeCounter - 1);

	double acc = 0;
	int newGear = CalcGear(car, acc);
	if( newGear != car->ctrl.gear )
		gearChangeCounter = 10;
	car->ctrl.gear = newGear;

	static double lastPos = 0;
	static double lastSpd = 0;
	//		  t    del-t  acc   brk   clut gr  rpm   pos    spd   cspd  acc   cacc rslpv  fslp  rslp freac rreac
	PRINTF( "%1.3f,%7.5f,%5.3f,%5.3f,%5.3f,%d,%4.0f,%12.7f,%5.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%5.0f,%5.0f\n",
			s->currentTime,
			s->deltaTime,
			car->ctrl.accelCmd,
			car->ctrl.brakeCmd,
			car->ctrl.clutchCmd,
			car->priv.gear,
			car->priv.enginerpm * 60 / (2 * PI),
			car->pub.DynGCg.pos.x,
			car->pub.DynGCg.vel.x,
			(car->pub.DynGCg.pos.x - lastPos) / s->deltaTime,
			car->pub.DynGC.acc.x,
			(car->pub.DynGC.vel.x - lastSpd) / s->deltaTime,
			car->priv.wheel[2].slipAccel,
			(m_cm[PATH_NORMAL].wheel(0).slipX() + m_cm[PATH_NORMAL].wheel(1).slipX()) * 0.5,
			(m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5,
			car->priv.reaction[0] + car->priv.reaction[1],
			car->priv.reaction[2] + car->priv.reaction[3] );
	lastSpd = car->pub.DynGC.vel.x;
	lastPos = car->pub.DynGC.pos.x;
}

void	MyRobot::Drive( int index, tCarElt* car, tSituation* s )
{
	m_cm[PATH_NORMAL].update( car, s );

#if EXPERIMENTAL
//#if 1
	launchControlFullThrottle(car, s);
//	launchControlClutch(car, s);
//	launchControlAcclerator(car, s);
//	launchControlAccSlip(car, s);
//	launchControlAccSlip2(car, s);
//	launchControlSimple(car, s);
	return;
#endif
	
	double	h[4];
	{for( int i = 0; i < 4; i++ )
	{
		tTrkLocPos	wp;
//		double		wx = car->_pos_X;
//		double		wy = car->_pos_Y;
		double		wx = car->pub.DynGCg.pos.x;
		double		wy = car->pub.DynGCg.pos.y;
		RtTrackGlobal2Local(car->_trkPos.seg, wx, wy, &wp, TR_LPOS_SEGMENT);
		h[i] = car->_pos_Z - RtTrackHeightL(&wp) - car->_wheelRadius(i);
	}}

	if( m_raceStart || s->currentTime <= 0.5 )
	{
		if( m_raceStart )
		{
			Vec2d	thisPt(car->_pos_X, car->_pos_Y);
			for( int i = 0; i + 1 < HIST; i++ )
				m_lastPts[i] = m_lastPts[i + 1];
			m_lastSpd = 0;
			m_lastAng = 0;
			m_lastLap = car->race.laps;
			m_steerGraph.SetBeta( 0.1 );
		}

		m_raceStart = false;
//		DEBUGF( "m_avoidS = %g (race start 2)\n", 0.0 );
		m_avoidS = 0;
		m_avoidSVel = 0;
		m_avoidT = CalcPathTarget(m_track.CalcPos(car), -car->_trkPos.toMiddle);
		m_avoidTVel = 0;
	}

	if( car->race.laps != m_lastLap )
	{
		m_lastLap = car->race.laps;
		PRINTF( "[%d] Average fuel/m: %g\n", car->index, m_pitControl.FuelPerM(car) );
	}

	double	carFuel = car->_fuel;
	double	gripScaleF = GripFactor(car, true);
	double	gripScaleR = GripFactor(car, false);
//	double	carFuel = MN(car->_fuel, 50);
//	double	carFuel = 0;
//	if( fabs(m_cm[PATH_NORMAL].FUEL			- car->_fuel)		> 5		||
	if( fabs(m_cm[PATH_NORMAL].FUEL			- carFuel)			> 2		||
		fabs(m_cm[PATH_NORMAL].DAMAGE		- car->_dammage)	> 250	||
		fabs(m_cm[PATH_NORMAL].GRIP_SCALE_F	- gripScaleF)		> 0.05	||
		fabs(m_cm[PATH_NORMAL].GRIP_SCALE_R	- gripScaleR)		> 0.05 )
	{
//		m_cm[PATH_NORMAL].FUEL			= 5 * floor(car->_fuel / 5);
		m_cm[PATH_NORMAL].FUEL			= 2 * floor(carFuel / 2);
		m_cm[PATH_NORMAL].DAMAGE		= car->_dammage;
		m_cm[PATH_NORMAL].GRIP_SCALE_F	= gripScaleF;
		m_cm[PATH_NORMAL].GRIP_SCALE_R	= gripScaleR;
		for( int p = PATH_LEFT; p <= PATH_RIGHT; p++ )
		{
			m_cm[p].FUEL			= m_cm[PATH_NORMAL].FUEL;
			m_cm[p].DAMAGE			= m_cm[PATH_NORMAL].DAMAGE;
			m_cm[p].GRIP_SCALE_F	= m_cm[PATH_NORMAL].GRIP_SCALE_F;
			m_cm[p].GRIP_SCALE_R	= m_cm[PATH_NORMAL].GRIP_SCALE_R;
		}
/*		for( int path = PATH_NORMAL; path <= PATH_RIGHT; path++ )
		{
			m_path[path].CalcMaxSpeeds( m_cm );
			m_path[path].PropagateBraking( m_cm );
			m_path[path].PropagateAcceleration( m_cm );
//			m_pitPath[path].CalcMaxSpeeds( m_cm );
//			m_pitPath[path].PropagateBraking( m_cm );
//			m_pitPath[path].PropagateAcceleration( m_cm );
		}*/
#if !(defined(DEV) && defined(BRAKING_TEST))
		for( int p = PATH_NORMAL; p <= PATH_RIGHT; p++ )
		{
			m_path[p].CalcMaxSpeeds( m_cm[p] );
			m_path[p].PropagateBraking( m_cm[p] );
			m_path[p].PropagateAcceleration( m_cm[p] );
		}
#endif
	}

//	DEBUGF( "**** wheel fz %g %g %g %g\n", h[0], h[1], h[2], h[3] );

	if( h[0] > m_priv[PATH_NORMAL].FLY_HEIGHT )
	{
//		m_flying = FLY_COUNT;
		m_flying = MN(FLY_COUNT, m_flying + (FLY_COUNT / 2));
	}
	else if( m_flying > 0 )
	{
		m_flying--;
	}

	// get curret pos on track.

	PtInfo	pi, aheadPi;
	double	angle = SteerAngle0(car, pi, aheadPi, m_priv[PATH_NORMAL]);
//	double	angle = SteerAngle1(car, pi, aheadPi);
//	double	angle = SteerAngle2(car, pi, aheadPi);
//	double	angle = SteerAngle3(car, pi, aheadPi);

	double	steer = angle / car->_steerLock;

	double	offset = -car->pub.trkPos.toMiddle - pi.offs;
	double	oangle = car->pub.DynGC.pos.az     - pi.oang;
	double	ospeed = car->pub.DynGC.vel.x      - pi.spd;
	double	xfslip = (m_cm[PATH_NORMAL].wheel(0).slipX() +
					  m_cm[PATH_NORMAL].wheel(1).slipX()) * 0.5;
	double	xrslip = (m_cm[PATH_NORMAL].wheel(2).slipX() +
					  m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5;
	double	yfslip = (m_cm[PATH_NORMAL].wheel(0).slipY() +
					  m_cm[PATH_NORMAL].wheel(1).slipY()) * 0.5;
	double	yrslip = (m_cm[PATH_NORMAL].wheel(2).slipY() +
					  m_cm[PATH_NORMAL].wheel(3).slipY()) * 0.5;
	double	fslip = hypot(xfslip, yfslip);
	double	rslip = hypot(xrslip, yrslip);
	NORM_PI_PI(oangle);

//	DEBUGF( "   *** pos %7.2f  angle %6.3f\n", pos, angle );

	// work out current car speed.
	double	spd0 = hypot(car->_speed_x, car->_speed_y);
//	double	spd0 = car->_speed_x;
	double	acc0 = (spd0 - _lastSpd0) / s->deltaTime;
	_lastSpd0 = spd0;
//	spd0 = car->pub.speed;

//	spd0 = hypot(m_cm[PATH_NORMAL].VEL_L.x, m_cm[PATH_NORMAL].VEL_L.y);
//	acc0 = hypot(m_cm[PATH_NORMAL].ACC_L.x, m_cm[PATH_NORMAL].ACC_L.y);

//	DEBUGF( "[%d] eo%6.2f  ea%6.1f  es %6.2f  acc%6.2f  pa%6.2f fsl %6.3f,%6.3f=%6.3f  rsl %6.3f,%6.3f=%6.3f\n",
//			car->index, offset, oangle * 180 / PI, ospeed, acc0, pi.acc, xfslip, yfslip, fslip, xrslip, yrslip, rslip );

//	PRINTF( "[%d] a(x%6.2f c%6.2f p%6.2f t%6.2f) dspd%8.3f; yo%6.3f xfs; %6.3f; fs; %6.3f; rs; %6.3f; a-b; %5.3f %5.3f\n",
//			car->index, car->pub.DynGC.acc.x, acc0, m_cm[PATH_NORMAL].ACC_L.x, pi.acc,
//			car->pub.speed - pi.spd, pi.offs + car->pub.trkPos.toMiddle, xfslip, fslip, rslip,
//			car->ctrl.accelCmd, car->ctrl.brakeCmd );
/*	PRINTF( "[%d] react: %4.0f, %4.0f\n", 
			car->index, car->_reaction[0] + car->_reaction[1], car->_reaction[2] + car->_reaction[3] );
*/
	double	targetSpd = pi.spd;
	double	targetAcc = pi.acc;
	double	avoidTargetSpd = pi.spd;
	double	avoidTargetAcc = pi.acc;

//	if( pi.idx >= 461 && pi.idx <= 484 )
//		targetSpd = 100;

	bool	close = false;
	bool	lapper = false;
	AvoidOtherCars( index, car, s, pi.k, &avoidTargetSpd, &avoidTargetAcc, &close, &lapper );
	bool	slowing = false;
	if( car->pub.speed > avoidTargetSpd && targetSpd > avoidTargetSpd )
	{
		PRINTF( "[%d] slowing for avoidance.  curr %g  targ %g  avoid spd %g, avoid acc %g, curr acc %g\n",
				car->index, car->pub.speed, targetSpd, avoidTargetSpd, avoidTargetAcc, car->pub.DynGC.acc.x );
		slowing = true;
	}
	//if( targetSpd > pi.spd )
		targetSpd = avoidTargetSpd;
		targetAcc = avoidTargetAcc;

#if 1
	//
	// steer into the skid
	if( !m_pitControl.WantToPit())
	{
		double skidfactor = (m_avoidS != 1 ? m_priv[PATH_NORMAL].SKID_FACTOR_TRAFFIC :
											 m_priv[PATH_NORMAL].SKID_FACTOR);
		if (skidfactor > 0.0)
		{
			double vx = car->_speed_X;
			double vy = car->_speed_Y;
			double dirx = cos(car->_yaw);
			double diry = sin(car->_yaw);
			double Skid = (dirx * vy - vx * diry) / (spd0 == 0.0 ? 0.1 : spd0);
			Skid = MN(0.9, MX(-0.9, Skid));
			steer += (asin(Skid) / car->_steerLock) * skidfactor;
		}
		else
		{
			static double last_srslip[MAX_MOD_ITF] = {0};

//			const double slip_limit = 0.15;
			const double slip_limit  = m_priv[PATH_NORMAL].REAR_LAT_SLIP_LIMIT;
			const double slip_factor = m_priv[PATH_NORMAL].REAR_LAT_SLIP_FACTOR;
			const double slip_dscale = m_priv[PATH_NORMAL].REAR_LAT_SLIP_DSCALE;

			double srslip = SGN(yrslip) * rslip;
			double delta_srslip = srslip - last_srslip[index];
			last_srslip[index] = srslip;
			steer += atan(delta_srslip) * slip_dscale;// 0.1;

			if( rslip > slip_limit && fabs(yrslip) > slip_limit * 0.5 )
			{
				double slip_delta = yrslip > 0 ? rslip - slip_limit : slip_limit - rslip;
				//PRINTF( "slip delta %g\n", slip_delta );
				steer += atan(slip_delta) * slip_factor;// * 0.01;
			}
		}
	}
#endif

//	DEBUGF( "   *** pos %7.2f  angle %6.3f\n", pos, angle );
	{
		Vec2d	thisPt(car->_pos_X, car->_pos_Y);
		if( (thisPt - m_lastPts[0]).len() > 0.1 &&
			car->ctrl.accelCmd == 1.0 )
		{
			double	x[2];
			x[0] = Utils::CalcCurvature(m_lastPts[0], m_lastPts[HIST / 2], thisPt);
			x[0] = fabs(x[0]);
			x[1] = m_lastSpd;
			m_steerGraph.Learn( x, fabs(m_lastAng) );
//			DEBUGF( "*** Learn %8.6f, %5.2f -> %8.6f  \n",
//					x[0], m_lastSpd, fabs(m_lastAng) );
		}

		for( int i = 0; i + 1 < HIST; i++ )
		{
			m_lastPts[i] = m_lastPts[i + 1];
		}

		m_lastPts[HIST - 1] = thisPt;
		m_lastSpd = spd0;
		m_lastAng = m_lastAng * 0.75 + angle * 0.25;
	}

	const double G = 9.81;

	double	acc = 1.0;
	double	brk = 0;
/*
	double	targetSpd = pi.spd;
//	if( pi.idx >= 461 && pi.idx <= 484 )
//		targetSpd = 100;

	bool	close = false;
	bool	lapper = false;
	AvoidOtherCars( index, car, pi.k, targetSpd, s, close, lapper );
*/
//	DEBUGF( "$$ tv=%5.1f - %5.1f == %5.1f  acc=%5.1f  slip=%5.2f,%5.2f  zf=%5.2f\n",
//			targetSpd, spd0, targetSpd - spd0, car->pub.DynGC.acc.x,
//			car->priv.wheel[2].slipAccel, car->priv.wheel[2].slipSide, car->priv.reaction[2] );
	int spdCtrl = m_priv[PATH_NORMAL].SPDC_NORMAL;
	bool traffic = close;// || m_pitControl.WantToPit();
	if( traffic )
		spdCtrl = m_priv[PATH_NORMAL].SPDC_TRAFFIC;

//	if( car->race.distRaced > 310 )
//		spdCtrl = 8;

//	double xslipf0 = (m_cm[PATH_NORMAL].wheel(0).slipX() + m_cm[PATH_NORMAL].wheel(1).slipX()) * 0.5;
//	double yslipf0 = (m_cm[PATH_NORMAL].wheel(0).slipY() + m_cm[PATH_NORMAL].wheel(1).slipY()) * 0.5;
//	double slipf0  = hypot(xslipf0, yslipf0);
//	SpeedControl( spdCtrl, targetSpd, spd0, targetAcc, acc0, xslipf0, slipf0, pi.k, car, acc, brk, traffic );
	SpeedControl( spdCtrl, targetSpd, spd0, targetAcc, acc0, fslip, xfslip, rslip, pi.k, car, acc, brk, traffic );

//	PRINTF( "speed control[%d]: acc %0.3f  brk %0.3f :: fslip %0.3f  rslip %0.3f :: fxslip %0.3f  rxslip %0.3f\n",
//			spdCtrl, acc, brk, fslip, rslip, xfslip, xrslip );

	if( lapper )
		acc = MN(acc, 0.6);
/*
	DEBUGF( "### (%8.3f, %8.3f)  (%8.3f, %8.3f)\n",
			car->priv.wheel[2].slipSide,
			car->priv.wheel[2].slipAccel,
			car->priv.wheel[3].slipSide,
			car->priv.wheel[3].slipAccel  );
*/
#if 0
	const double TCL_SLIP  = 8;
	const double TCL_RANGE = 8;
	double drivenWheelSpeed = (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) * car->_wheelRadius(REAR_LFT) / 2.0;
	double slip = drivenWheelSpeed - car->_speed_x;
	double min_slip = (car->_speed_x > 10.0f ? TCL_SLIP : TCL_RANGE/10);
	if (slip > min_slip)
	{
		float friction = MIN(car->_wheelSeg(REAR_RGT)->surface->kFriction, car->_wheelSeg(REAR_LFT)->surface->kFriction);
		if (friction >= 0.95f)
			friction = pow(friction+0.06f, 3.0f);
		else
			friction = pow(friction, 3.0f);
		float this_slip = MIN(TCL_RANGE, (TCL_RANGE/2) * friction);
		acc = MIN(acc, 1.0f - MIN(0.95f, (slip - min_slip) / TCL_RANGE));
	}
#endif
#if 1
	{
//		double delta = -(car->priv.wheel[2].slipAccel + car->priv.wheel[3].slipAccel) / 2;

		double w1_speed = car->priv.wheel[0].spinVel * car->info.wheel[0].wheelRadius;
		double w2_speed = car->priv.wheel[1].spinVel * car->info.wheel[1].wheelRadius;
		double w3_speed = car->priv.wheel[2].spinVel * car->info.wheel[2].wheelRadius;
		double w4_speed = car->priv.wheel[3].spinVel * car->info.wheel[3].wheelRadius;

		double front_speed = (w1_speed + w2_speed) * 0.5;	
		double left_spin_speed  = w3_speed - front_speed;
		double right_spin_speed = w4_speed - front_speed;
		double max_spin_speed = MX(left_spin_speed, right_spin_speed);
		double avg_spin_speed = (left_spin_speed + right_spin_speed) * 0.5;
		double delta = max_spin_speed;

		double ddiff = delta - _prevDelta;
		_prevDelta = delta;


		double slipErr = m_priv[PATH_NORMAL].TCL_TARGET_SPEED - delta;
//		double targSpd = 0.205 * MX(5, front_speed);
//		double slipErr = targSpd - delta;
		_tctrlAcc = MX(0, MN(_tctrlAcc + 0.013 * slipErr - 0.05 * ddiff, 1.0));

//		double	rSlipX = (m_cm[PATH_NORMAL].wheel(2).slipX() + m_cm[PATH_NORMAL].wheel(3).slipX()) * 0.5;
//		double	rSlipY = (m_cm[PATH_NORMAL].wheel(2).slipY() + m_cm[PATH_NORMAL].wheel(3).slipY()) * 0.5;
//		double	rSlip = hypot(rSlipX, rSlipY);
//		_tctrlAcc = MX(0, MN(_tctrlAcc + 0.1 * (0.175 - rSlip), 1.0));

	  if( spdCtrl != 8 )
	  {		
		if( brk == 0 && acc > _tctrlAcc )//&& _tctrlAcc < 0.75 )
			acc = _tctrlAcc;
		if( brk != 0 && max_spin_speed > m_priv[PATH_NORMAL].DEC_MAX_SPIN_SPEED )
		{
			acc = MX(0.1, 0.5 - (max_spin_speed - 2) * 0.1);
			//acc = 0.2;
			//acc = MX(0.1, MN(0.2 - (max_spin_speed - DEC_MAX_SPIN_SPEED) * 0.02, 0.2));
		}
	  }
		if( s->currentTime < 0 )
		{
			acc = 1.0;
			brk = 1.0;
			car->ctrl.clutchCmd = 1.0;
		}

//		PRINTF( "[%5.2f]%d brk %.2f%c  acc %.2f  tca %.2f  acc.x %6.3f  targ %6.3f  mx.sp %6.3f  av.sp %6.3f  "
//				"sxL %6.3f  sxR %6.3f\n",
//				s->currentTime, spdCtrl, brk, brk == 0 ? 'z' : 'n', acc, _tctrlAcc, car->pub.DynGC.acc.x, targetSpd, max_spin_speed,
//				avg_spin_speed, m_cm[PATH_NORMAL].wheel(2).slipX(), m_cm[PATH_NORMAL].wheel(3).slipX() );
	}
#endif
#if 0
	//if( acc && !brk )
	{
		double w1_speed = car->priv.wheel[0].spinVel * car->info.wheel[0].wheelRadius;
		double w2_speed = car->priv.wheel[1].spinVel * car->info.wheel[1].wheelRadius;
		double w3_speed = car->priv.wheel[2].spinVel * car->info.wheel[2].wheelRadius;
		double w4_speed = car->priv.wheel[3].spinVel * car->info.wheel[3].wheelRadius;

		double front_speed = (w1_speed + w2_speed) * 0.5;	
		double left_spin_speed  = w3_speed - front_speed;
		double right_spin_speed = w4_speed - front_speed;
		double max_spin_speed = MX(left_spin_speed, right_spin_speed);
		/*double limit = 3;
		if( max_spin_speed > limit )
			if( brk == 0 )
				acc = MX(0.1, 0.5 - (max_spin_speed - 3) * 0.1);
			else
				acc = MX(0.1, 0.5 - (max_spin_speed - 2) * 0.1);*/

		if( brk == 0 && max_spin_speed > ACC_MAX_SPIN_SPEED )
			acc = MX(0.1, 0.5 - (max_spin_speed - 2) * 0.1);
		if( brk != 0 && max_spin_speed > DEC_MAX_SPIN_SPEED )
			acc = MX(0.1, 0.5 - (max_spin_speed - 2) * 0.1);
	}
#endif
#if 0
	{
	//	double delta = wv - car->pub.speed;
		double slipTan = -(car->priv.wheel[2].slipAccel + car->priv.wheel[3].slipAccel) / 2;
		double slipLat = -(car->priv.wheel[2].slipSide  + car->priv.wheel[3].slipSide)  / 2;
		double delta = hypot(slipTan, slipLat * 2.5);
		double ddiff = delta - _prevDelta;
		_prevDelta = delta;

//		PRINTF( "wv %g, spd %g, diff %g, dc %d\n", wv, car->pub.speed, delta, _deltaCounter );
		PRINTF( "st %6.3f, sl %6.3f, delta %6.3f\n", slipTan, slipLat, delta );

		if( slipTan > 0 )
		{
			if( car->priv.gear < 3 )
			{
				double slipErr = 4.0 - delta;
				double newAcc = MX(0, MN(car->ctrl.accelCmd + 0.013 * slipErr - 0.05 * ddiff, 1.0));
				acc = newAcc;
			}
		}
	}
#endif
//	if( car->priv.wheel[2].slipAccel > 5 || car->priv.wheel[3].slipAccel > 5 )
//		acc = acc * 0.5;

	double	delta = pi.offs + car->_trkPos.toMiddle;
//	if( acc == 1.0 && car->_speed_y * delta > 0 &&
//		car->_speed_x > 0 && fabs(car->_speed_y / car->_speed_x) > 0.1 )
//		acc = 0.95;

//	acc = 1.0;
//	brk = 0;
	// out of control??
	double	skidAng = atan2(car->_speed_Y, car->_speed_X) - car->_yaw;
	NORM_PI_PI(skidAng);
  if( spdCtrl != 8 )
  {
//	if( car->_speed_x > 5 && fabs(car->_speed_y / car->_speed_x) > 0.15 )
	if( /*brk == 0 &&*/ car->_speed_x > 5 && fabs(skidAng) > 0.2 )
	{
//		DEBUGF( "******* skidAng %g    x_spd %g    ratio %g\n",
//				skidAng, car->_speed_x, car->_speed_y / car->_speed_x );
		acc = MN(acc, 0.25 + 0.75 * cos(skidAng));
	}

	if( /*brk != 0 &&*/ car->_speed_x > 5 )
	{
		skidAng = MN(skidAng * 2, PI);
		brk *= MX(0, fabs(cos(skidAng)));
	}
  }
	// too far off line?
	double	fDelta = fabs(delta);
	const double	offDist = 2;//0.5;
	if( fDelta > offDist && car->_distRaced > 50 )
	{
		double	mx = MX(1.0 - (fDelta - offDist) * 0.2, 0.3);
		acc = MN(mx, acc);
	}

//	DEBUGF( "****** slip %6.3f m/s    sy %6.3f\n", spd0 * sin(skidAng),
//				sin(skidAng) );
//	if( fabs(sin(skidAng)) > 0.125 )
//		acc = MN(acc, 0.2);

/*
	// stuck?
	double	carRelAng = car->_yaw
	if( car->_speed_x < 5 && (pi.toL < 3 || pi.toR < 3) &&
		car->
	{
		m_stuck = true;
	}

	// get unstuck if stuck.
	if( m_stuck )
	{
	}
*/

//	DEBUGF( "acc %g  brk %g\n", acc, brk );

	if( car->ctrl.clutchCmd > 0 )
	{
/*		car->ctrl.clutchCmd -= 0.085f;
		if( car->ctrl.clutchCmd < 0 )
			car->ctrl.clutchCmd = 0;
*/
		double	wr = 0;
		int		count = 0;

		if( m_driveType == cDT_FWD || m_driveType == cDT_4WD )
		{
			wr += car->_wheelRadius(FRNT_LFT) + car->_wheelRadius(FRNT_RGT);
			count += 2;
		}

		if( m_driveType == cDT_RWD || m_driveType == cDT_4WD )
		{
			wr += car->_wheelRadius(REAR_LFT) + car->_wheelRadius(REAR_RGT);
			count += 2;
		}
		wr /= count;
		double	gr = car->_gearRatio[car->_gear + car->_gearOffset];
		double	rpmForSpd = gr * car->_speed_x / wr;
		double	rpm = car->_enginerpm;
//		DEBUGF( "RPM %3.0f  RPM2 %3.0f  %g\n", rpm, rpmForSpd, rpmForSpd / rpm );
#ifdef OLD_CLUTCH
		if( car->ctrl.clutchCmd > 0.5 )
		{
			car->ctrl.clutchCmd = 0.5;
		}
		else if( car->ctrl.clutchCmd == 0.5 )
		{
			if( rpmForSpd / rpm > 0.82 )
				car->ctrl.clutchCmd = 0.49f;
		}
		else
		{
			car->ctrl.clutchCmd -= 0.04f;
			if( car->ctrl.clutchCmd < 0 )
				car->ctrl.clutchCmd = 0;
		}
#else
		const float dec = 0.02f;
		if( car->ctrl.clutchCmd > dec * 7.5f )
		{
			car->ctrl.clutchCmd = dec * 7.5f;
		}
		else
		{
			car->ctrl.clutchCmd -= dec;
			if( car->ctrl.clutchCmd < 0 )
				car->ctrl.clutchCmd = 0;
		}
	}
#endif
	int gear = CalcGear(car, acc);

	// facing the wrong way on the track??
	double	dirAng = pi.oang - car->_yaw;
	NORM_PI_PI(dirAng);

	if( fabs(car->_speed_x) > 2 || s->currentTime < 0 )
		m_stuckTime = 0;
	else
		m_stuckTime += s->deltaTime;

	const Opponent::Sit& mySit = m_opp[car->index].GetInfo().sit;

#ifdef DO_OLD_STUCK
//	if( gear > 0 && fabs(dirAng) > 75 * PI / 180 )//car->_speed_x < 0 )
//	if( !Pitting(car) && gear > 0 && fabs(dirAng) > 25 * PI / 180 )//car->_speed_x < 0 )
	if( m_stuck != NOT_STUCK ||
		!Pitting(car) && gear > 0 && fabs(dirAng) > 30 * PI / 180 && car->_speed_x < 10 ||
		m_stuckTime > 2 )
	{
		double sideAhead  = dirAng > 0 ? pi.extR + car->pub.trkPos.toMiddle : pi.extL - car->pub.trkPos.toMiddle;
		double sideBehind = dirAng > 0 ? pi.extL - car->pub.trkPos.toMiddle : pi.extR + car->pub.trkPos.toMiddle;

//		PRINTF( "el %5.2f  er %5.2f  tomid %5.2f  ttl %5.2f ttr %5.2f\n",
//				pi.extL, pi.extR, car->pub.trkPos.toMiddle, car->pub.trkPos.toLeft, car->pub.trkPos.toRight );
//		PRINTF( "stuck %d  stuck_time %1.2f  speed %1.2f  ahead %1.2f  behind %1.2f\n",
//				m_stuck, m_stuckTime, car->_speed_x, sideAhead, sideBehind );

		double	toLeft  = -999;
		double	toRight =  999;
		
		double	aheadLeft  = mySit.offs;
		double	aheadRight = mySit.offs;

		double	distAhead = 25;
		double	distBehind = 25;

		int		oppCount = 0;

		vector<Opponent*> sorted_opps;
		for( int i = 0; i < m_nCars; i++)
			if( (m_opp[i].GetCar()->pub.state & RM_CAR_STATE_NO_SIMU) == 0 )
				sorted_opps.push_back( &m_opp[i] );
		std::sort(sorted_opps.begin(), sorted_opps.end(), Opponent::compareSitRelPos);

		double	lastPos = 0;
		for( int i = 0; i < (int)sorted_opps.size(); i++ )
		{
			const Opponent* pOpp = sorted_opps[i];
			if( pOpp == &m_opp[car->index] )
				continue;

			const tCarElt*			oCar  = pOpp->GetCar();
			const Opponent::Info&	oInfo = pOpp->GetInfo();
			const Opponent::Sit&	oSit  = oInfo.sit;

			if( oSit.relPos > lastPos + 10 )
				// found a gap.
				break;

			lastPos = MX(lastPos, oSit.relPos);

			if( oSit.stuckTime > 1 && oSit.relPos > -2 && oSit.relPos < 32 )
			{
				double left  = oSit.offs - 3;
				double right = oSit.offs + 3;
				
				if( oSit.relPos > -2 && oSit.relPos < 4 )
				{
					// got car to side.
					if( oSit.offs < mySit.offs )//oInfo.GotFlags(Opponent::F_TRK_LEFT) )
						toLeft = max(toLeft, right);
					if( oSit.offs > 0 )//oInfo.GotFlags(Opponent::F_TRK_RIGHT) )
						toRight = min(toRight, left);
				}
				else
				{
				}

				oppCount++;

				if( car->index == 12 )
					PRINTF( "[%d] [%d] left %1.3f  right %1.3f\n", car->index, i, left, right );
			}
			else
			{
//				if( index == 8 )
//					PRINTF( "[%d] [%d] st %1.3f  mydist %1.3f  odist %1.3f\n",
//							car->index, i, oSit.stuckTime,
//							car->race.distFromStartLine, oCar->race.distFromStartLine );
			}
			
			const CarBounds2d	oBounds(oCar);

			CarBounds2d	bounds(car);
			distAhead  = bounds.distToSide(CarBounds2d::SIDE_FRONT, distAhead,  oBounds);
			distBehind = bounds.distToSide(CarBounds2d::SIDE_REAR,  distBehind, oBounds);
			//PRINTF( "[%d] [%d] distAhead %g, distBehind %g\n", car->index, oCar->index, distAhead, distBehind );
		}

		PRINTF( "[%d] distAhead %g, distBehind %g, sideAhead %g, sideBehind %g, toLeft %g, toRight %g\n",
				car->index, distAhead, distBehind, sideAhead, sideBehind, toLeft, toRight );

		double myOffs = -car->pub.trkPos.toMiddle;
		toLeft  -= myOffs;
		toRight -= myOffs;
		//double best = fabs(bestLeft) < fabs(bestRight) ? toLeft : toRight;
		if( fabs(dirAng) < 30 * PI / 180 )// && bestLeft < bestRight )
		{
			//dirAng = -atan(best);// - pi.oang;
			dirAng = 0;
			brk = 0;
			if( oppCount == 0 || !slowing )
				m_stuck = NOT_STUCK;
			else
				m_stuck = STUCK_GO_FORWARDS;
		}
		else
		switch( m_stuck )
		{
			case NOT_STUCK:
				m_stuck = dirAng * car->_trkPos.toMiddle < 0 ?
								STUCK_GO_BACKWARDS : STUCK_GO_FORWARDS;
				m_stuckTime = 0;
				break;
		
			case STUCK_GO_BACKWARDS:
				gear = -1;
	//			acc = 0.5;
				brk = 0;
				if( distBehind < 0.2 || sideBehind < m_track.GetWidth() / 2 || m_stuckTime > 1 )
				{
					m_stuck = STUCK_GO_FORWARDS;
					m_stuckTime = 0;
				}
				break;
				
			case STUCK_GO_FORWARDS:
				if( distAhead < 0.2 || sideAhead < m_track.GetWidth() / 2 || m_stuckTime > 1 )
				{
					m_stuck = STUCK_GO_BACKWARDS;
					m_stuckTime = 0;
				}
				break;
		}
//		if( car->index == 12 )
//			PRINTF( "[%d] bL %.3f  bR %.3f  best %.3f  ang %.3f\n", car->index, bestLeft, bestRight, best, dirAng );
/*
//		DEBUGF( "Backwards (dirAng %lg) (offs %lg)\n", dirAng, pi.offs );

//		if( dirAng * pi.offs < 0 )
		if( dirAng * car->_trkPos.toMiddle < 0 )
//		if( dirAng > 0 && car->_trkPos.toLeft  > 3 ||
//			dirAng < 0 && car->_trkPos.toRight > 3 )
		{
//			DEBUGF( "Reversing a bit\n", dirAng );
			gear = -1;
//			acc = 0.5;
			brk = 0;
			steer = -SGN(dirAng);
		}
		else
		{
			steer = SGN(dirAng);
		}
*/
		if( car->_speed_x < 0 )
			steer = -SGN(dirAng);
		else
			steer = SGN(dirAng);

		acc = fabs(car->_speed_x) < 3 ? 1.0 : 0.2;
		
		double w1_speed = car->priv.wheel[0].spinVel * car->info.wheel[0].wheelRadius;
		double w2_speed = car->priv.wheel[1].spinVel * car->info.wheel[1].wheelRadius;
		double w3_speed = car->priv.wheel[2].spinVel * car->info.wheel[2].wheelRadius;
		double w4_speed = car->priv.wheel[3].spinVel * car->info.wheel[3].wheelRadius;

		double front_speed = (w1_speed + w2_speed) * 0.5;	
		if( gear > 0 && (w3_speed > front_speed + 2 || w4_speed > front_speed + 2) ||
		    gear < 0 && (w3_speed < front_speed - 2 || w4_speed < front_speed - 2) )
			acc = 0.1;
	}
	else
	{
		m_stuck = NOT_STUCK;
	}
#endif

  if( spdCtrl != 8 )
  {
	if( gear > 0 && car->_speed_x < -0.01 )
	{
		gear = 1;
		brk = car->_speed_x < -0.5 ? 0.25 : 0;
		acc = 0.25;
	}

	if( gear == 1 && car->_speed_x >= -0.01 && car->_speed_x < 10 &&
		acc == 1.0 && brk == 0 )
	{
		if( fabs(dirAng) > 10 )
		{
			acc = 0.5;
		}
		else
		{
			double	rpm = car->_enginerpm;
	//		double	clutch = (800 - rpm) / 850;
			double	clutch = (850 - rpm) / 400;
			if( car->_speed_x < 0.05 )
				clutch = 0.5;

	//		if( car->race.curTime <= 0.01 )
	//			clutch = 1.0;
			car->ctrl.clutchCmd = MX(0, MN(clutch, 0.9));
		}
	}
  }
//	if( car->_speed_x > 10 )
//		acc = 0.2;

/*
	if( m_flying )
	{
//		m_flying--;

		// steer in direction of car movement.
		double	velAng = atan2(car->_speed_Y, car->_speed_X);
		double	angle = velAng - car->_yaw;
		NORM_PI_PI(angle);

//		DEBUGF( "*** flying %d   h %.3f  ang%g\n", m_flying, h[0], angle );

		int		f = FLY_COUNT - m_flying;
		double	t = MX(0, MN(1.0 * f / FLY_COUNT, 1));
//		t = 0.25;
		steer = steer * t + (1 - t) * angle / car->_steerLock;//0.25;//angle *= 0.5;//0.1;//5;
//		steer = steer * t;
		acc = MN(acc, MX(t, 0.8 + 0.2 * t));
	}
*/
	if( fabs(pi.offs) > 5 )
	{
//		acc = MN(acc, 0.25);
	}

	double	toL = pi.toL - car->_trkPos.toMiddle;
	double	toR = pi.toR + car->_trkPos.toMiddle;
//	DEBUGF( "toL %g   toR %g\n", toL, toR );
	if( toL < -1 || toR < -1 )
	{
		// off track.
//		acc = MN(acc, 0.2);
	}

	if( car->ctrl.accelCmd == 1 && car->ctrl.brakeCmd == 0 )
	{
		m_maxAccel.Learn( car->_speed_x, car->_accel_x );
//		DEBUGF( "%4.1f %4.1f ", car->_speed_x, car->_accel_x );
//		for( int i = 0; i < 12; i++ )
//			DEBUGF( " %4.1f", m_maxAccel.GetY(i) );
//		DEBUGF( "\n" );
	}

	if( fabs(pi.k * spd0 - car->_yaw_rate) < 0.02 )
	{
		double	velAng = atan2(car->_speed_Y, car->_speed_X);
		double	ang = car->_yaw - velAng;
		NORM_PI_PI(ang);

		int	k = int(floor((pi.k - K_MIN) / K_STEP));
		int	s = int(floor((spd0 - SPD_MIN) / SPD_STEP));
		double	ae = 0;
		if( k >= 0 && k < K_N && s >= 0 && s < SPD_N )
		{
			ae = ang - m_angle[s][k];
			m_angle[s][k] += ae * 0.1;
			ae = m_angle[s][k] - ang;
		}

//		DEBUGF( "Steady state  s %.1f, k %.6f, a %.4f  ae %.4f  o %g\n",
//				spd0, pi.k, ang, ae, pi.offs + car->_trkPos.toMiddle );
	}

//	if( acc > 0.25 )
//		acc = 0.25;
//	if( car->ctrl.accelCmd + 0.1 < acc )
//	{
//		acc = car->ctrl.accelCmd + 0.1;
//	}

	if( spdCtrl != 5 && spdCtrl != 6 && spdCtrl != 7 && spdCtrl != 8 )
		brk = ApplyAbs(car, brk);
//	acc = ApplyTractionControl(car, acc);

//	if( car->_speed_x > 10 )
//		acc = 0.0;

//	DEBUGF( "accel cmd: %.2f    steer ang: %.3f\n", acc, steer * car->_steerLock * 180 / PI );
/*
	if( car->race.laps > 1 && pi.idx >= 330 )
	{
		acc = 0.0;
		brk = 1.0;
	}
*/
    // set up the values to return
    car->ctrl.steer = steer;
    car->ctrl.gear = gear;
    car->ctrl.accelCmd = acc;
//	car->ctrl.accelCmd = brk > 0 ? MN(acc, 0.2) : acc;
    car->ctrl.brakeCmd = brk;

////if( car->race.laps > 3 )
	bool doStuckThing = true;
	if( Pitting(car) )
	{
		const TeamInfo::Item* pTeamMateInfo = m_pShared->m_teamInfo.GetTeamMate(car);
		if( pTeamMateInfo != NULL && (pTeamMateInfo->pCar->pub.state & RM_CAR_STATE_PIT) )
			doStuckThing = false;
	}
	if( doStuckThing )
		m_stuckThing.execute( m_track, s, car, mySit );

	m_pitControl.Process( car, m_pShared->m_teamInfo.GetAt(car->index) );

//	PRINTF( "[%d] final control : accel %0.3f  brake %0.3f  clutch %0.3f  gear %d  steer %0.3f  [%8.1fm]\n",
//			car->index, car->ctrl.accelCmd, car->ctrl.brakeCmd, car->ctrl.clutchCmd, car->ctrl.gear, car->ctrl.steer,
//			car->race.distRaced );

//	DEBUGF( "%g %g %g\n", car->priv.wheel[0].Fx, car->priv.wheel[0].Fy, car->priv.wheel[0].Fz );
//	DEBUGF( "%g %g %08x\n", car->priv.wheel[0].slipSide, car->priv.wheel[0].slipAccel, car->priv.wheel[0].seg );
//	DEBUGF( "%d %g %g --  \r", pi.idx, car->_pos_X, car->_pos_Y );
}

// Pitstop callback.
int		MyRobot::PitCmd( int index, tCarElt* car, tSituation* s )
{
	m_pitControl.Process( car, m_pShared->m_teamInfo.GetAt(car->index) );
	return false;
}


// End of the current race.
void	MyRobot::EndRace( int index, tCarElt* car, tSituation* s )
{
}


// Called before the module is unloaded.
void	MyRobot::Shutdown( int index )
{
	// dump out some interesting info....
//	const int	NSEG = m_track.GetSize();
//	{for( int i = 0; i < NSEG; i++ )
//	{
//		DEBUGF( "dist %7.1f  spd %7.1f  rec %7.1f\n",
//				m_track[i].segDist,
//				m_path[PATH_NORMAL].GetAt(i).spd,
//				m_oppPath[m_myOppIdx].GetAt(i).avgV );
//	}}
}

void	MyRobot::AvoidOtherCars(
	int					index,
	const tCarElt*		car,
	const tSituation*	s,
	double				k,
	double*				carTargetSpd,
	double*				carTargetAcc,
	bool*				close,
	bool*				lapper )
{
	m_pShared->m_teamInfo.GetAt(car->index)->damage = car->_dammage;

	double	trackLen = m_track.GetLength();
	double	myPos = RtGetDistFromStart(const_cast<tCarElt*>(car));
	double	mySpd = hypot(car->_speed_X, car->_speed_Y);

	double	myDirX, myDirY;
	if( fabs(mySpd) < 0.01 )
	{
		myDirX = cos(car->_yaw);
		myDirY = sin(car->_yaw);
	}
	else
	{
		myDirX = car->_speed_X / mySpd;
		myDirY = car->_speed_Y / mySpd;
	}

	int		myIdx = 0;

	{for( int i = 0; i < m_nCars; i++ )
	{
		m_opp[i].UpdatePath();
/*		double	conf = m_oppPath[i].CalcConfidence();
		if( conf > 0.95 )
		{
			double	dist = RtGetDistFromStart(m_oppPath[i].GetCar());
			DEBUGF( "***** %2d   dist %.2f   conf %g\n",
					m_oppPath[i].GetCar()->_laps, dist, conf );
		}*/

		PtInfo	oppPi;
		const tCarElt* oCar = m_opp[i].GetCar();
		GetPosInfo( oCar->race.distFromStartLine, oppPi );
		m_opp[i].UpdateSit( car, s, &m_pShared->m_teamInfo, myDirX, myDirY,	oppPi );
	}}

	const Opponent::Sit&	mySit = m_opp[m_myOppIdx].GetInfo().sit;

	{for( int i = 0; i < m_nCars; i++ )
	{
		m_opp[i].ProcessMyCar( s, &m_pShared->m_teamInfo, car, mySit, *this,
								m_maxAccel.CalcY(car->_speed_x), i );
	}}

	// accumulate all the collision the flags...
	Avoidance::Info	ai;
	double	minCatchTime = 99;
	double	minCatchAccTime = 99;
	double	minVCatTime = 99;
	*lapper = false;

	double	width = m_track.GetWidth();
//	ai.aheadSpan = Span(-width / 2, width / 2);
//	ai.sideSpan = ai.aheadSpan;

	PtInfo	pi;
	GetPtInfo( PATH_NORMAL, car->_distFromStartLine, pi );
	ai.bestPathOffs = pi.offs;

	{for( int i = 0; i < m_nCars; i++ )
	{
		Opponent::Info&	oi = m_opp[i].GetInfo();
		CarElt*			oCar = m_opp[i].GetCar();

		if( oCar != car && fabs(oi.sit.rdPX) < 25 )
			ai.nearbyCars++;
	}}
	
	{for( int i = 0; i < m_nCars; i++ )
	{
		Opponent::Info&	oi = m_opp[i].GetInfo();
		CarElt*			oCar = m_opp[i].GetCar();

		ai.flags |= oi.flags;

		if( oi.GotFlags(Opponent::F_FRONT) )
		{
			if( oi.flags & (Opponent::F_COLLIDE | Opponent::F_CATCHING |
							Opponent::F_CATCHING_ACC) )
			{
//				DEBUGF( "*** flags " );
//				if( oi.GotFlags(Opponent::F_COLLIDE) )		DEBUGF( "COLLIDE " );
//				if( oi.GotFlags(Opponent::F_CATCHING) )		DEBUGF( "CATCH " );
//				if( oi.GotFlags(Opponent::F_CATCHING_ACC) )	DEBUGF( "CATCH_A " );
//				if( oi.GotFlags(Opponent::F_CLOSE) )		DEBUGF( "CLOSE " );
//				DEBUGF( "\n" );
//				DEBUGF( "t %g   catchY %g  catchSpd %g  catchDec %g\n",
//						oi.catchTime, oi.catchY, oi.catchSpd, oi.catchDecel );
			}

			bool dangerous = oi.GotFlags(Opponent::F_DANGEROUS);
			if( oi.GotFlags(Opponent::F_COLLIDE) &&
//				oi.catchTime < 3 && fabs(oi.catchY) < oi.sit.minDY )
//				oi.catchDecel > 5 & fabs(oi.catchY) < oi.sit.minDY )
//				(dangerous ||
//				 oi.catchDecel > 12.5 * car->_trkPos.seg->surface->kFriction) )
				true )
			{
				PRINTF( "[%d] catching: (%s) cdec %g (dgr %d), cspd %g (in %g s)\n",
						car->index, oCar->info.name, oi.catchDecel, dangerous, oi.catchSpd, oi.catchTime );

				// this is a really shitty estimate of the speed we need to be going
				// to slow down for the being caught ahead... would really like to work out a better
				// way.
//				ai.spdF = MN(ai.spdF, oi.catchSpd);
//				ai.accF = MN(ai.accF, -oi.catchDecel);

				bool followingClosely = oi.sit.rdPX > 0 &&
										oi.sit.rdPX < oi.sit.minDXa + 3 &&
										fabs(oi.sit.rdPY) < oi.sit.minDY;

				bool dangerous = oi.GotFlags(Opponent::F_DANGEROUS);
				const double MIN_DECEL = 15.0;
				double decel =  (car->pub.speed - oi.catchSpd) / MX(oi.catchTime, 0.1);
				PRINTF( "[%d] catching: (%s) decel %g (dgr %d)\n", car->index, oCar->info.name, oi.catchDecel, dangerous );
				PRINTF( "[%d] catching: (%s) path spd: %g (%g) %g  path acc: %g (%g) %g\n",
						car->index, oCar->info.name, oi.sit.pi.spd, oi.sit.pi.spd - oCar->pub.DynGC.vel.x, oCar->pub.DynGC.vel.x,
						oi.sit.pi.acc, oi.sit.pi.acc - oCar->pub.DynGC.acc.x, oCar->pub.DynGC.acc.x );
				if( decel > MIN_DECEL || dangerous || followingClosely )
//				if( dangerous || oi.catchTime < 1.25 ||
//					oCar->pub.DynGC.vel.x < oi.sit.pi.spd - 10 )
//				if( oi.sit.rdPX < 10 || dangerous )
				{
					double decelSpeed = MX(oi.catchSpd, car->pub.speed - 2 * s->deltaTime * decel);
					if( dangerous )
						decelSpeed = oi.catchSpd;
					PRINTF( "[%d] catching: (%s) myspd %g, decel spd %g, hisspd %g, catch (time %g  spd %g  dec %g)\n",
							car->index, oCar->info.name, car->pub.speed, decelSpeed, oCar->pub.speed, oi.catchTime, oi.catchSpd, oi.catchDecel );
					if( car->pub.speed > decelSpeed )
						PRINTF( "[%d] slowing to avoid car: (%s) spd=%g\n",
								car->index, oCar->info.name, oCar->pub.speed );
//					ai.spdF = MN(ai.spdF, decelSpeed);
					ai.spdF = MN(ai.spdF, oi.catchSpd);
					ai.accF = MN(ai.accF, -oi.catchDecel);
				}
			}

			if( oi.flags & (Opponent::F_COLLIDE | Opponent::F_CATCHING) )
				minCatchTime = MN(minCatchTime, oi.catchTime);
			if( oi.flags & Opponent::F_CATCHING_ACC )
				minCatchAccTime = MN(minCatchAccTime, oi.catchAccTime);
			if( oi.sit.rdVX < 0 )
			{
				double	vCatTime = -(oi.sit.rdPX - oi.sit.minDXa) / oi.sit.rdVX;
				if( vCatTime > 0 )
					minVCatTime = MN(minVCatTime, vCatTime);
			}

			bool	ignoreTeamMate = oi.GotFlags(Opponent::F_TEAMMATE) &&
										(car->_laps < oCar->_laps ||
										 car->_dammage + 200 >= oi.tmDamage);
			//ignoreTeamMate = true;

			oi.avoidLatchTime = MX(0, oi.avoidLatchTime - s->deltaTime);

			double	maxSpdK = 15.0 / (110 * 110);
			double	colTime = fabs(k) > maxSpdK ? 0.5 : 0.7;
			double	catTime = fabs(k) > maxSpdK ? 0.5 :	2.5;
			double	cacTime = fabs(k) > maxSpdK ? 0.5 : 2.5;
			bool	catching = 
				 oi.catchTime    < colTime && fabs(oi.catchY)    < 5 && oi.GotFlags(Opponent::F_COLLIDE)  ||
				 oi.catchTime    < catTime && fabs(oi.catchY)    < 5 && oi.GotFlags(Opponent::F_CATCHING) ||
				 oi.catchAccTime < cacTime && fabs(oi.catchAccY) < 5 && oi.GotFlags(Opponent::F_CATCHING_ACC);
			if( !ignoreTeamMate &&
//				oi.catchTime < 10 &&
//				(oi.GotFlags(Opponent::F_CATCHING) ||
//				 oi.GotFlags(Opponent::F_CATCHING_ACC)) )
				(//oi.GotFlags(Opponent::F_CLOSE) ||
				 oi.avoidLatchTime > 0 || catching ||
				 oi.GotFlags(Opponent::F_DANGEROUS)) )
			{
				DEBUGF( "%.3f catch %d (dgr %d)  catch[%d%d t %.3f y %.4g]  acc[%d t %.3f y %.4g]\n",
						oi.avoidLatchTime, catching, oi.GotFlags(Opponent::F_DANGEROUS),
						oi.GotFlags(Opponent::F_CATCHING), oi.GotFlags(Opponent::F_COLLIDE), oi.catchTime, oi.catchY,
						oi.GotFlags(Opponent::F_CATCHING_ACC), oi.catchAccTime, oi.catchAccY );
				double	toL, toR;
				GetPathToLeftAndRight( oCar, toL, toR );
				toL += oi.sit.tVY * oi.catchTime;
				toR -= oi.sit.tVY * oi.catchTime;
				bool	spaceL = toL > oi.sit.minDY;// + 0.25;
				bool	spaceR = toR > oi.sit.minDY;// + 0.25;
				bool	avoidL = oi.sit.rdPY < 0 && spaceR;
				bool	avoidR = oi.sit.rdPY > 0 && spaceL;

				if( catching )
					oi.avoidLatchTime = fabs(k) < maxSpdK ? 0.5 : 0.1;

//				bool	avoidL = oi.sit.rdPY < -oi.sit.minDY && spaceR;
//				bool	avoidR = oi.sit.rdPY >  oi.sit.minDY && spaceL;

				if( fabs(k) < maxSpdK )
				{
					if( !avoidL && !avoidR )
					{
						avoidL = !spaceL && spaceR;
						avoidR = !spaceR && spaceL;
					}
/*					if( spaceL && spaceR )
					{
						if( oi.sit.rdPY < -oi.sit.minDY )
							avoidR = true;
						else if( oi.sit.rdPY >  oi.sit.minDY )
							avoidL = true;
						else if( spaceL && spaceR )
						{
							avoidL = toL < toR;
							avoidR = !avoidL;
						}
					}*/
				}

//				bool	avoidL = oi.sit.rdPY < 0 && oCar->pub.trkPos.toRight > 4.0 ||
//								 oi.sit.rdPY > 0 && oCar->pub.trkPos.toLeft  < 4.0;
//				bool	avoidL = oi.sit.offs < 0;

				if( avoidL )
				{
					ai.avoidAhead |= Opponent::F_LEFT;
//					ai.aheadSpan.ExcludeLeftOf(-oCar->_trkPos.toMiddle + oi.sit.minDY);
				}
				if( avoidR )
				{
					ai.avoidAhead |= Opponent::F_RIGHT;
//					ai.aheadSpan.ExcludeRightOf(-oCar->_trkPos.toMiddle - oi.sit.minDY);
				}

				if( avoidL )
//					minLDist = MN(oi.sit.rdPX, minLDist);
					ai.minLDist = MN(oi.sit.ragVX, ai.minLDist);
				if( avoidR )
//					minRDist = MN(oi.sit.rdPX, minRDist);
					ai.minRDist = MN(oi.sit.ragVX, ai.minRDist);
			}
		}

		if( oi.GotFlags(Opponent::F_TO_SIDE) )
		{
			int	av = oi.sit.rdPY < 0 ? Opponent::F_LEFT : Opponent::F_RIGHT;

			ai.avoidToSide |= av;

			if( oi.sit.rdPY < 0 )
			{
				ai.minLSideDist = MN(ai.minLSideDist, -oi.sit.rdPY - oi.sit.minDY);
//				ai.sideSpan.ExcludeLeftOf(-oCar->_trkPos.toMiddle + oi.sit.minDY);
			}
			else
			{
				ai.minRSideDist = MN(ai.minRSideDist,  oi.sit.rdPY - oi.sit.minDY);
//				ai.sideSpan.ExcludeRightOf(-oCar->_trkPos.toMiddle - oi.sit.minDY);
			}

		}

		if( oi.GotFlags(Opponent::F_AHEAD) )
		{
			if( ai.pClosestAhead == 0 ||
				ai.pClosestAhead->sit.rdPX > oi.sit.rdPX )
			{
				ai.pClosestAhead = &oi;
			}
		}


		bool	treatTeamMateAsLapper =
					oi.GotFlags(Opponent::F_TEAMMATE | Opponent::F_REAR) &&
					oi.sit.relPos > -25 &&
					car->_laps == oCar->_laps &&
					car->_dammage > oi.tmDamage + 300 &&
					ai.nearbyCars <= 1;
/*
		if( STAY_TOGETHER > 50 &&
			oi.GotFlags(Opponent::F_TEAMMATE | Opponent::F_REAR) &&
			oi.sit.relPos < -25 && oi.sit.relPos > -STAY_TOGETHER &&
			car->_dammage + 2000 > oi.tmDamage )
		{
//			treatTeamMateAsLapper = true;
			lapper = true; 
		}
*/
		if( oi.GotFlags(Opponent::F_LAPPER) || treatTeamMateAsLapper )
		{
			int	av = oi.sit.rdPY < 0 ? Opponent::F_LEFT : Opponent::F_RIGHT;
			ai.avoidLapping |= av;
			*lapper = true;
		}
	}}

	ai.k = k;
	ai.nextK = k;

	double	pos = car->_distFromStartLine;
	int		carIdx = m_track.IndexFromPos(m_track.CalcPos(car));
	ai.k = 	m_path[PATH_NORMAL].GetAt(carIdx).k;
	int		NSEG = m_track.GetSize();
	for( int i = 1; i < NSEG; i++ )
	{
		int	idx = (carIdx + i) % NSEG;
		double	thisK = m_path[PATH_NORMAL].GetAt(idx).k;
		if( fabs(thisK) > 0.01 )
		{
			ai.nextK = thisK;
			break;
		}
	}

	Avoidance avoidance;
	Vec2d	target = avoidance.calcTarget(ai, car, *this);

//	target.x = 1;
//	target.y = 1;

//	DEBUGF( "tx %g  ty %g\n", target.x, target.y );
//	DEBUGF( "****** avoid: %d%d%d  k %6.3f  nextK %6.3f  (%.2f,%.2f)  target: %6.3f\n",
//			ai.avoidToSide, ai.avoidLapping, ai.avoidAhead, ai.k, ai.nextK,
//			m_avoidS, m_avoidT, target );

	*carTargetSpd = MN(*carTargetSpd, ai.spdF);
	*carTargetAcc = MN(*carTargetAcc, ai.accF);
	*close = (ai.flags & Opponent::F_CLOSE) != 0;
//	lapper = ai.avoidLapping != 0;

	if( m_flying || s->currentTime < START_HOLD_LINE_TIME )
		return;

//	double	w = width;
	double	w = m_priv[PATH_NORMAL].AVOID_WIDTH * 2 + width;
	double	scale = 25.0 / w;
//	scale *= (w * 0.5 + 1) / (m_priv[PATH_NORMAL].AVOID_WIDTH * 2 + w * 0.5);
	double	avoidSMaxA = 0.00075 * scale;
	double	avoidSMaxV = 0.005 * scale;
//	double	avoidSMaxA = 0.00025 * scale;
//	double	avoidSMaxV = 0.0015 * scale;

//	double	avoidTMaxA = 0.0002;
	double	avoidTMaxA = 0.0003 * scale;
//	double	avoidTMaxA = 0.001 * scale;
	double	avoidTMaxV = 0.2 * scale;

//	avoidSMaxA *= 0.5;
//	avoidSMaxV *= 0.5;
//	avoidTMaxA *= 0.66;
//	avoidTMaxV *= 0.66;

//	if( avoidToSide && fabs(minSideDist) < 4 )
//	{
//		avoidMaxA    *= 2;
//		avoidMaxVNeg *= 2;
//		avoidMaxVPos *= 2;
//	}

/*
	m_attractor = 0.0;
	if( m_followPath == PATH_LEFT && m_avoidT <= 0 )
		m_attractor = -1.0;
	else if( m_followPath == PATH_RIGHT && m_avoidT >= 0 )
		m_attractor = 1.0;
*/
//	if( m_avoidT * target >= 0 )
		m_attractor = target.x;

//	if( m_avoidS == 1 && m_attractor != 0 )
//		m_avoidT = m_attractor;

	double	targetS = 1 - target.y;
	if( m_avoidS != 1 && m_attractor == 0 ||
		m_avoidS != targetS && m_attractor != 0 )
	{
		targetS = (m_attractor == 0) ? 1 : 0;//0.35;
		double	avoidA = targetS > m_avoidS ? avoidSMaxA : -avoidSMaxA;

		double	dist = targetS - m_avoidS;
		if( fabs(dist) < 0.0005 )
			m_avoidSVel = 0;
		else
		{
			double	slowS = (m_avoidSVel * m_avoidSVel) / (2 * avoidSMaxA);
			if( fabs(dist) <= slowS )
			{
				avoidA = -(m_avoidSVel * m_avoidSVel) / (2 * dist);
			}

			m_avoidSVel += avoidA;
		}
	}
	else
		m_avoidSVel = 0;

	if( m_avoidSVel > avoidSMaxV )
		m_avoidSVel = avoidSMaxV;
	else if( m_avoidSVel < -avoidSMaxV )
		m_avoidSVel = -avoidSMaxV;

	double	oldAvoidS = m_avoidS;
	m_avoidS += m_avoidSVel;
	if( m_avoidS < 0.0005 && m_avoidSVel < 0 )
	{
//		DEBUGF( "m_avoidS %g (v %g)\n", m_avoidS, m_avoidSVel );
//		DEBUGF( "m_avoidS = %g (min limit)\n", 0.0 );
		m_avoidS = 0;
		m_avoidSVel = 0;
	}
	else if( m_avoidS >= 0.9995 && m_avoidSVel > 0 )
	{
//		DEBUGF( "m_avoidS = %g (max limit)\n", 1.0 );
		m_avoidS = 1;
//		m_avoidT = 0;
		m_avoidSVel = 0;
	}
	else if( oldAvoidS < targetS && m_avoidS >= targetS ||
			 oldAvoidS > targetS && m_avoidS <= targetS ||
			 fabs(targetS - m_avoidS) < 0.0005 )
	{
//		DEBUGF( "m_avoidS = %g (hit target)\n", targetS );
		m_avoidS = targetS;
		m_avoidSVel = 0;
	}

//	avoidMaxVNeg *= 2;
//	avoidMaxVPos *= 2;

//	m_avoidS = 1;
//	DEBUGF( "**** avoidS %g\n", m_avoidS );

//	DEBUGF( "at %8.5f ts %3.1f as %8.5f asv %8.5f att %5.2f minCat %5.2f/%5.2f/%5.2f\r",
//			m_avoidT, targetS, m_avoidS, m_avoidSVel, m_attractor,
//			minCatchTime, minCatchAccTime, minVCatTime );

	double	attractT = m_attractor;
//	if( attractT == 0 )
//		attractT = m_avoidT;
	double	avoidA = 0;
	if( attractT != m_avoidT )
	{
		double	tMaxA = avoidTMaxA / MX(0.2, 1 - m_avoidS);
//		avoidA = attractT > m_avoidT ? avoidTMaxA : -avoidTMaxA;
		avoidA = attractT > m_avoidT ? tMaxA : -tMaxA;
//		double	slowA  = -(m_avoidTVel * m_avoidTVel) / (2 * (attractor - m_avoidT));
//		if( fabs(slowA) >= avoidMaxA )
//			avoidA = SGN(slowA) * avoidMaxA;

		double	dist = attractT - m_avoidT;
		double	slowS = (m_avoidTVel * m_avoidTVel) / (2 * avoidTMaxA);
		if( dist * m_avoidTVel > 0 && fabs(dist) <= slowS )
		{
			avoidA = -(m_avoidTVel * m_avoidTVel) / (2 * dist);
		}

//		DEBUGF( "##### avoidA %7.4f   attractor %g\n", avoidA, attractor );
		if( avoidA > avoidTMaxA )
			avoidA = avoidTMaxA;
		else if( avoidA < -avoidTMaxA )
			avoidA = -avoidTMaxA;

		m_avoidTVel += avoidA;
	}
	else
		m_avoidTVel = 0;

	double	tMaxV = avoidTMaxV / MX(0.2, 1 - m_avoidS);
	double	pos_tMaxV =  tMaxV;
	double	neg_tMaxV = -tMaxV;
	if( target.y != 0 &&
		(ai.flags & Opponent::F_DANGEROUS) == 0 && 
		(ai.flags & Opponent::F_TO_SIDE)   == 0 )
	{
		if( k >  0.0025 )
		{
			pos_tMaxV = 0;//*= 0.001;
			PRINTF( "-- right movement disallowed\n" );
		}
		if( k < -0.0025 )
		{
			neg_tMaxV = 0;//*= 0.001;
			PRINTF( "-- left movement disallowed\n" );
		}
	}
	
	if( m_avoidTVel > pos_tMaxV )
		m_avoidTVel = pos_tMaxV;
	else if( m_avoidTVel < neg_tMaxV )
		m_avoidTVel = neg_tMaxV;

	double	oldAvoidT = m_avoidT;
	m_avoidT += m_avoidTVel;
//	if( fabs(m_avoidT - attractor) <= 0.005 )
/*	if( fabs(m_avoidT - attractT) <= avoidTMaxA * 1.01 ||
		m_avoidS >= 0.99 )
	{
		m_avoidT = attractT;
		m_avoidTVel = 0;
	}
*/
	if( m_avoidT < -1 )
	{
		m_avoidT = -1;
		m_avoidTVel = 0;
	}
	else if( m_avoidT > 1 )
	{
		m_avoidT = 1;
		m_avoidTVel = 0;
	}
	else if( oldAvoidT < attractT && m_avoidT >= attractT ||
			 oldAvoidT > attractT && m_avoidT <= attractT )
	{
		m_avoidT = attractT;
		m_avoidTVel = 0;
	}

	double	offs = CalcPathOffset(pos, m_avoidS, m_avoidT);
//	CalcBestPathUV(pos, offs, m_avoidU, m_avoidV);

//	PRINTF( "***** targ (%5.2f, %5.2f)  k %9.6f  offs  %5.2f  avS %.3f  avT %.4f  avTV %.5f  avTA %.5f\n",
//			target.x, target.y, pi.k, offs - ai.bestPathOffs, m_avoidS, m_avoidT, m_avoidTVel, avoidA );
}

int		MyRobot::CalcGear( tCarElt* car, double& acc )
{
    if( car->_gear <= 0 )
    {
		//car->ctrl.clutchCmd = 1.0;
		return 1;
	}

//	const int	MAX_GEAR = 7;
	const int	MAX_GEAR = car->_gearNb - 1;

	double	gr_dn = car->_gear > 1 ?
				car->_gearRatio[car->_gear + car->_gearOffset - 1] :
				1e5;
	double	gr_this = car->_gearRatio[car->_gear + car->_gearOffset];

	double	wr = (car->_wheelRadius(2) + car->_wheelRadius(3)) / 2;
	double	rpm = gr_this * car->_speed_x / wr;
//	double	rpm = car->_enginerpm;

	double	rpmUp = m_gearUpRpm;
	double	rpmDn = rpmUp * gr_this * 0.95 / gr_dn;

//	DEBUGF( "gear %d    rpm %6.1f %6.1f    dist %6.1f  up dist %6.1f   down dist %6.1f\n",
//		car->_gear, rpm, car->_enginerpm, grDist, upDist, dnDist );

    if( car->_gear < MAX_GEAR && rpm > rpmUp )
	{
		car->ctrl.clutchCmd = 0.5;//1.0;
//		acc = 0.5;
        return car->_gear + 1;
    }
	else if( car->_gear > 1 && rpm < rpmDn )
	{
		car->ctrl.clutchCmd = 1.0;
//		acc = 1.0;
		return car->_gear - 1;
	}

    return car->_gear;
}

double	MyRobot::ApplyAbs( tCarElt* car, double brake )
{
	if( car->_speed_x < 10 )
		return brake;

	double	slip = 0.0;
	for( int i = 0; i < 4; i++ )
		slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / car->_speed_x;
	slip /= 4.0;

//	if( slip < 0.9 )//ABS_SLIP )
//	if( (m_cm[PATH_NORMAL].wheel(0).slipX() + m_cm[PATH_NORMAL].wheel(1).slipX()) * 0.5 > 0.14 )
	if( (m_cm[PATH_NORMAL].wheel(0).slipX() + m_cm[PATH_NORMAL].wheel(1).slipX()) * 0.5 > 0.175 )
	{
//		DEBUGF( "ABS ABS ABS ABS ABS ABS ABS  slip %g\n", slip );
//		brake *= slip;
		brake *= 0.5;
	}

	return brake;
}

double	MyRobot::ApplyTractionControl( tCarElt* car, double acc )
{
	double	spin = 0;
	double	wr = 0;
	int		count = 0;

	if( m_driveType == cDT_FWD || m_driveType == cDT_4WD )
	{
		spin += car->_wheelSpinVel(FRNT_LFT) * car->_wheelRadius(FRNT_LFT);
		spin += car->_wheelSpinVel(FRNT_RGT) * car->_wheelRadius(FRNT_RGT);
		wr += car->_wheelRadius(FRNT_LFT) + car->_wheelRadius(FRNT_RGT);
		count += 2;
	}

	if( m_driveType == cDT_RWD || m_driveType == cDT_4WD )
	{
		spin += car->_wheelSpinVel(REAR_LFT) * car->_wheelRadius(REAR_LFT);
		spin += car->_wheelSpinVel(REAR_RGT) * car->_wheelRadius(REAR_RGT);
		wr += car->_wheelRadius(REAR_LFT) + car->_wheelRadius(REAR_RGT);
		count += 2;
	}

	static double	tract = 1.0;

	spin /= count;

	if( car->_speed_x < 0.01 )
		return acc;

	double	slip = car->_speed_x / spin;

    if( slip > 1.1 )
	{
		tract = 0.1;

		wr /= count;
		double	gr = car->_gearRatio[car->_gear + car->_gearOffset];
		double	rpmForSpd = gr * car->_speed_x / wr;
//		DEBUGF( "TC:  slip %g   rpmForSpd %g\n",
//					slip, rpmForSpd );
//		acc = 0.5 * rpmForSpd / car->_enginerpmRedLine;
		acc = 0;
	}
	else
	{
		tract = MN(1.0, tract + 0.1);
	}

	acc = MN(acc, tract);

	return acc;
}

