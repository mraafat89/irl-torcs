/***************************************************************************

    file        : MyRobot.h
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

// MyRobot.h: interface for the MyRobot class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MYROBOT_H__AA11EF80_E44F_4A64_B4ED_36FDD5FE6C69__INCLUDED_)
#define AFX_MYROBOT_H__AA11EF80_E44F_4A64_B4ED_36FDD5FE6C69__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <track.h>
#include <car.h>
#include <robot.h>

#include <vector>

#include "MyTrack.h"
#include "Shared.h"
#include "ClothoidPath.h"
#include "PitPath.h"
#include "PitControl.h"
#include "Opponent.h"
#include "PidController.h"
#include "LearnedGraph.h"
#include "LinearRegression.h"
#include "Utils.h"
#include "Stuck.h"

const double	SPD_MIN = 0;
const double	SPD_MAX = 100;
const int		SPD_N = 20;
const double	SPD_STEP = (SPD_MAX - SPD_MIN) / SPD_N;
const double	K_MIN = -0.1;
const double	K_MAX = 0.1;
const int		K_N = 100;
const double	K_STEP = (K_MAX - K_MIN) / K_N;

class MyRobot  
{
public:
	enum	// paths
	{
		PATH_NORMAL,
		PATH_LEFT,
		PATH_RIGHT,

		N_PATHS,
	};

	enum
	{
		STEER_SPD_MAX = 20,
		STEER_K_MAX = 41,
		HIST = 20,
	};

public:
	MyRobot();
	~MyRobot();

	void	SetShared( Shared* pShared );

	void	InitTrack( int index, tTrack* track, void* carHandle,
						void** carParmHandle, tSituation* s);
	void	NewRace( int index, tCarElt* car, tSituation* s );
	void	Drive( int index, tCarElt* car, tSituation* s );
	int		PitCmd( int index, tCarElt* car, tSituation* s );
	void	EndRace( int index, tCarElt* car, tSituation* s );
	void	Shutdown( int index);

	bool	Pitting( int path, double pos ) const;
	bool	Pitting( tCarElt* car ) const;
	
	int		PitType() const;

	void	GetPtInfo( int path, double pos, PtInfo& pi ) const;
	void	GetPosInfo( double pos, PtInfo& pi, double u, double v ) const;
	void	GetPosInfo( double pos, PtInfo& pi ) const;
	double	CalcPathTarget( double pos, double offs, double s ) const;
	double	CalcPathTarget( double pos, double offs ) const;
	Vec2d	CalcPathTarget2( double pos, double offs ) const;
	double	CalcPathOffset( double pos, double s, double t ) const;
	void	CalcBestPathUV( double pos, double offs, double& u, double& v ) const;
	double	CalcBestSpeed( double pos, double offs ) const;
	void	GetPathToLeftAndRight( const CarElt* pCar, double& toL, double& toR ) const;
	double	GripFactor( const CarElt* pCar, bool front ) const;

private:
	struct Private;

	double	SteerAngle0( tCarElt* car, PtInfo& pi, PtInfo& aheadPi, const Private& priv );
	double	SteerAngle1( tCarElt* car, PtInfo& pi, PtInfo& aheadPi );
	double	SteerAngle2( tCarElt* car, PtInfo& pi, PtInfo& aheadPi );
	double	SteerAngle3( tCarElt* car, PtInfo& pi, PtInfo& aheadPi );
	double	SteerAngle4( tCarElt* car, PtInfo& pi, PtInfo& aheadPi );

	void	SpeedControl0( double targetSpd, double spd0, double& acc, double& brk );
	void	SpeedControl1( double targetSpd, double spd0, double& acc, double& brk );
	void	SpeedControl2( double targetSpd, double spd0, double& acc, double& brk );
	void	SpeedControl3( double targetSpd, double spd0, double& acc, double& brk );
	void	SpeedControl4( double targetSpd, double spd0, double k,
						   CarElt* car, double& acc, double& brk );
	void	SpeedControl5( double targetSpd, double spd0, double targetAcc, double acc0,
						   double slip0, double& acc, double& brk, bool traffic );
	void	SpeedControl6( double targetSpd, double spd0, double targetAcc, double acc0,
						   double slip0, double& acc, double& brk, bool traffic );
	void	SpeedControl7( double targetSpd, double spd0, double targetAcc, double acc0,
						   double xslip0, double slip0, double& acc, double& brk, bool traffic );
	void	SpeedControl8( double targetSpd, double	spd0, double targetAcc, double	acc0,
						   double fslip0, double rslip0, double& acc, double& brk, bool	traffic );
	void	SpeedControl( int which, double targetSpd, double spd0, double targetAcc, double acc0,
						  double fslip0, double rxslip0, double rslip0, double k, CarElt* car, double& acc, double& brk,
						  bool traffic );

	void	launchControlClutch( tCarElt* car, tSituation* s );
	void	launchControlSimple( tCarElt* car, tSituation* s );
	void	launchControlAcclerator( tCarElt* car, tSituation* s );
	void	launchControlAccSlip( tCarElt* car, tSituation* s );
	void	launchControlAccSlip2( tCarElt* car, tSituation* s );
	void	launchControlFullThrottle( tCarElt* car, tSituation* s );

private:
	void	ProcessOtherCars( int index, tCarElt* car, double spd, tSituation* s );
	void	AvoidOtherCars( int index, const tCarElt* car, const tSituation* s, double k,
							double* carTargetSpd, double* carTargetAcc, bool* inTraffic,
							bool* lapper );

	int		CalcGear( tCarElt* car, double& acc );
	double	ApplyAbs( tCarElt* car, double brake );
	double	ApplyTractionControl( tCarElt* car, double acc );

private:
	enum	// drive types
	{
		cDT_RWD, cDT_FWD, cDT_4WD,
	};

	enum
	{
		cMAX_OPP = 20,
	};

	enum StuckAction
	{
		NOT_STUCK, STUCK_GO_BACKWARDS, STUCK_GO_FORWARDS,
	};

	struct	PathRange
	{
		double	u;
		double	vL;
		double	vR;
		bool	gotL;
		bool	gotR;
		bool	racingLine;

		PathRange() : u(1), vL(-1), vR(1),
					  gotL(false), gotR(false), racingLine(true) {}

		void	AddGreater( double pos, double offs, bool incRL, const MyRobot& me );
		void	AddLesser( double pos, double offs, bool incRL, const MyRobot& me );
	};

	struct Braking
	{
		double	targetSlip;
		double	targetBrk;
		double	acc;

		double	internalBrk;

		double	lastSlip;
		double	lastAccErr;
		double	lastSlipErr;

		Braking() { clear(); }

		double adjAccForSpdError( double targetAcc, double spd0, double targetSpd )
		{
			return targetAcc + (targetSpd - spd0) * 2.0;
		}

		void	clear()
		{
			targetSlip = 0;
			targetBrk  = 0;
			acc = 0;
			internalBrk = 0;
			lastSlip = 0;
			lastAccErr = 0;
			lastSlipErr  = 0;
		}

		void    execute5( double acc0, double targetAcc, double slip0, double brake_limit, bool in_traffic )
		{
			if( targetAcc >= 0 )
			{
				// no longer decelerating -- reset braking.
				clear();
				return;
			}
			else if( internalBrk == 0 )
			{
				// make a 1st estimate of how hard to brake.
				//targetSlip = 0.125;
				internalBrk = MN(-targetAcc * 0.1, 0.5);
			}

			double    accErr = acc0 - targetAcc;
			targetSlip = MX(0, MN(targetSlip + accErr * 0.01, 0.175));

			double    slipErr = targetSlip - slip0;
			internalBrk  = MX(0, MN(internalBrk + slipErr * 0.5, brake_limit));

			if( !in_traffic && slip0 > 0.18 )
			{
				//targetSlip = 0.175;
				internalBrk = internalBrk * 0.5;
			}
			targetBrk = internalBrk;
		}

		void    execute6( double acc0, double targetAcc, double slip0, double brake_limit, bool in_traffic )
		{
			if( targetAcc >= 0 )
			{
				// no longer decelerating -- reset braking.
				clear();
				return;
			}
			else if( internalBrk == 0 )
			{
				// make a 1st estimate of how hard to brake.
				//targetSlip = 0.125;
				internalBrk = MN(-targetAcc * 0.1, 0.5);
			}

			double    accErr = acc0 - targetAcc;
			targetSlip = MX(0, MN(targetSlip + accErr * 0.01, 0.175));

			double    slipErr = targetSlip - slip0;
			internalBrk  = MX(0, MN(internalBrk + slipErr * 0.5, brake_limit));

			targetBrk = internalBrk;
			if( !in_traffic && slip0 > 0.1 )
			{
				//targetSlip = 0.175;
				targetBrk = internalBrk * 0.5;
//				targetBrk = internalBrk * 0.15 / slip0;
			}
		}

		void    execute7( double acc0, double targetAcc, double xslip0, double slip0, double brake_limit, bool in_traffic )
		{
			if( targetAcc >= 0 )
			{
				// no longer decelerating -- reset braking.
				clear();
				return;
			}
			else if( internalBrk == 0 )
			{
				// make a 1st estimate of how hard to brake.
				//targetSlip = 0.125;
				internalBrk = MN(-targetAcc * 0.1, 0.5);
			}

			double    accErr = acc0 - targetAcc;
			targetSlip = MX(0, MN(targetSlip + accErr * 0.01, 0.175));

			double    slipErr = targetSlip - slip0;
			internalBrk  = MX(0, MN(internalBrk + slipErr * 0.5, brake_limit));

			targetBrk = internalBrk;
			if( !in_traffic && slip0 > 0.19 )
			{
				//targetSlip = 0.175;
//				targetBrk = internalBrk * 0.5;
				targetBrk = internalBrk * 0.11 / slip0;
			}
		}

		void    execute8( double acc0, double targetAcc, double fslip0, double rslip0, double brake_limit, bool in_traffic )
		{
			if( targetAcc >= 0 )
			{
				// no longer decelerating -- reset braking.
				clear();
				return;
			}
			else if( internalBrk == 0 )
			{
				// make a 1st estimate of how hard to brake.
				//targetSlip = 0.125;
				internalBrk = MN(-targetAcc * 0.1, 1.0);
				//internalBrk = 1.0;
			}

			double err = 0.175 - fslip0;
			double dfslip = fslip0 - lastSlip;
			internalBrk += err * 0.2 - dfslip * 1.5;
			internalBrk = MX(0, MN(internalBrk, 1));
			lastSlip = fslip0;

			targetBrk = internalBrk;
			acc = rslip0 < 0.175 ? 0.0 : 0.1;

			PRINTF( "%6.2f,%6.2f,%6.3f,%6.3f,%5.3f,%5.3f\n",
					acc0, targetAcc, fslip0, rslip0, internalBrk, acc );
		}
	};

private:
	Shared*			m_pShared;
	MyTrack			m_track;
	ClothoidPath	m_path[N_PATHS];
	PitPath			m_pitPath[N_PATHS][2];
	PitControl		m_pitControl;

	CarModel		m_cm[N_PATHS];
	Braking			m_brk;

	double			_acc;

	char			m_carName[100];
	char			m_trackName[100];

	struct Private
	{
		double				FLY_HEIGHT;
		std::vector<double>	FACTORS;
		std::vector<double>	INNER_MOD;
		int					QUAD_SMOOTH_ITERS;
		int					BUMP_MOD;
		int					SPDC_NORMAL;
		int					SPDC_TRAFFIC;
		double				ACC_MAX_SPIN_SPEED;
		double				DEC_MAX_SPIN_SPEED;
		double				STEER_K_ACC;
		double				STEER_K_DEC;
		double				STAY_TOGETHER;			// dist in m.
		double				AVOID_WIDTH;			// in m.
		double				PIT_ENTRY_OFFSET;		// dist in m.
		double				PIT_EXIT_OFFSET;		// dist in m.
		int					PIT_DAMAGE_WARN;		// fix damage if no extra pit stops required.
		int					PIT_DAMAGE_DANGER;		// fix damage even if extra pit stops required.
		double				SKID_FACTOR;	
		double				SKID_FACTOR_TRAFFIC;
		double				REAR_LAT_SLIP_FACTOR;
		double				REAR_LAT_SLIP_LIMIT;
		double				REAR_LAT_SLIP_DSCALE;
		double				STEER_0_LINE_SCALE;
		double				TCL_TARGET_SPEED;		// speed in m/s
		double				SAFETY_LIMIT;			// from edge in m.
		double				SAFETY_MULTIPLIER;		// dist = k * MULT.
		double				BRAKE_LIMIT;
		
		Private() { clear(); }

		void clear()
		{
			FLY_HEIGHT = 0.15;
			FACTORS.clear();
			BUMP_MOD = 0;
			QUAD_SMOOTH_ITERS = 0;
			SPDC_NORMAL = 5;
			SPDC_TRAFFIC = 0;
			ACC_MAX_SPIN_SPEED = 3.5;
			DEC_MAX_SPIN_SPEED = 2;
			STEER_K_ACC = 0;
			STEER_K_DEC = 0;
			STAY_TOGETHER = 0;			// dist in m.
			AVOID_WIDTH = 0.5;			// in m.
			PIT_ENTRY_OFFSET = 0;		// dist in m.
			PIT_EXIT_OFFSET = 0;		// dist in m.
			PIT_DAMAGE_WARN = 5000;		// fix damage if no extra pit stops required.
			PIT_DAMAGE_DANGER = 7000;	// fix damage even if extra pit stops required.
			SKID_FACTOR = 0.0;	
			SKID_FACTOR_TRAFFIC = 0.0;	
			REAR_LAT_SLIP_FACTOR = 2.5;
			REAR_LAT_SLIP_LIMIT = 0.15;
			REAR_LAT_SLIP_DSCALE = 0.1;
			STEER_0_LINE_SCALE = 0.15;
			TCL_TARGET_SPEED = 4.2;		// speed in m/s
			SAFETY_LIMIT = 1.5;			// from edge in m.
			SAFETY_MULTIPLIER = 100;	// dist = k * MULT.
			BRAKE_LIMIT = 1.0;
		}
	};

/*
	double			FLY_HEIGHT;
	std::vector<double>	FACTORS;
	int				BUMP_MOD;
	int				SPDC_NORMAL;
	int				SPDC_TRAFFIC;
	double			ACC_MAX_SPIN_SPEED;
	double			DEC_MAX_SPIN_SPEED;
	double			STEER_K_ACC;
	double			STEER_K_DEC;
	double			STAY_TOGETHER;			// dist in m.
	double			AVOID_WIDTH;			// in m.
	double			PIT_ENTRY_OFFSET;		// dist in m.
	double			PIT_EXIT_OFFSET;		// dist in m.
	double			SKID_FACTOR;	
	double			SKID_FACTOR_TRAFFIC;	
	double			TCL_TARGET_SPEED;		// speed in m/s
*/
	double			START_HOLD_LINE_TIME;	// hold inital line on track, in s.

	Private			m_priv[N_PATHS];

	int				m_driveType;
	double			m_gearUpRpm;			// for gear changing.
	PidController	m_lineControl;			// controller for line error.
	PidController	m_velAngControl;		// controller for direction of car.
	PidController	m_angControl;			// controller for attack angle error.
	double			m_prevYawError;
	double			m_prevLineError;
	int				m_steerLimit;
	int				m_flying;				// for landing better.
	int				m_nCars;
	int				m_myOppIdx;
	Opponent		m_opp[cMAX_OPP];		// info about other cars.
	double			m_avgAY;
	bool			m_raceStart;
	double			m_avoidS;				// where we are LR->T (0..1).
	double			m_avoidSVel;
	double			m_avoidT;				// where we are L->R (-1..1).
	double			m_avoidTVel;
	double			m_avoidU;
	double			m_avoidV;
	double			m_attractor;			// where we want to be.
	int				m_followPath;			// path we want to follow;
	Stuck			m_stuckThing;
	StuckAction		m_stuck;
	double			m_stuckTime;

	LearnedGraph	m_maxAccel;
//	LearnedGraph	m_maxDecel;
	double			m_angle[SPD_N][K_N];

	int				m_lastLap;
	Vec2d			m_lastPts[HIST];
	double			m_lastSpd;
	double			m_lastAng;

	LinearRegression	m_accBrkCoeff;
	LinearRegression	m_accCoeff;
	double			m_brkCoeff[50];
	double			m_steerCoeff[STEER_SPD_MAX][STEER_K_MAX];
	LearnedGraph	m_steerGraph;
	int				m_lastB;
	double			m_lastBrk;
	double			m_lastAcc;
	double			m_lastTargV;

	double			_tctrlAcc;
	int				_deltaCounter;
	double			_prevDelta;
	double			_lastSpd0;
};

#endif // !defined(AFX_MYROBOT_H__AA11EF80_E44F_4A64_B4ED_36FDD5FE6C69__INCLUDED_)
