/***************************************************************************

    file        : CarModel.cpp
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

// CarModel.cpp: implementation of the CarModel class.
//
//////////////////////////////////////////////////////////////////////

#include "CarModel.h"
#include "Quadratic.h"

#include <math.h>
#include "Utils.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CarModel::CarModel()
:	FLAGS(0),
	MASS(0),
	FUEL(0),
	DAMAGE(0),
	TYRE_MU(0),
	TYRE_MU_F(0),
	TYRE_MU_R(0),
	MU_SCALE(1),
	BRAKE_MU_SCALE(0.95),
	GRIP_SCALE_F(1),
	GRIP_SCALE_R(1),
	CA(0),
	CA_FW(0),
	CA_RW(0),
	CA_GE(0),
	CD_BODY(0),
	CD_WING(0),
	KZ_SCALE(0),
	WIDTH(2),
	POS_AZ(0),
	VEL_AZ(0),
	F_AXLE_X(1.5),
	R_AXLE_X(-1.5),
	F_AXLE_WB(0.5),
	R_AXLE_WB(0.5),
	F_AXLE_CG(0),
	R_AXLE_CG(0),
	F_WING_X(1.5),
	R_WING_X(-1.5)
{
	for( int w = 0; w < 4; w++ )
		_wheel[w].setWheel( w );
}

//===========================================================================

CarModel::~CarModel()
{
}

//===========================================================================

void    CarModel::config( const tCarElt* car )
{
    configWheels( car );

	MASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);

    float fwingarea  = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGAREA,  NULL, 0.0);
    float fwingangle = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGANGLE, NULL, 0.0);
    float rwingarea  = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA,  NULL, 0.0);
    float rwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, NULL, 0.0);
	float fwingArea = rwingarea * sin(rwingangle);
	float rwingArea = fwingarea * sin(fwingangle);
    float wingca = 1.23f * (fwingArea + rwingArea);

    float cl =  GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                             PRM_FCL, NULL, 0.0) +
                GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                             PRM_RCL, NULL, 0.0);
    float h  =  GfParmGetNum(car->_carHandle, SECT_FRNTRGTWHEEL, PRM_RIDEHEIGHT, NULL, 0.20f) +
                GfParmGetNum(car->_carHandle, SECT_FRNTLFTWHEEL, PRM_RIDEHEIGHT, NULL, 0.20f) +
                GfParmGetNum(car->_carHandle, SECT_REARRGTWHEEL, PRM_RIDEHEIGHT, NULL, 0.20f) +
                GfParmGetNum(car->_carHandle, SECT_REARLFTWHEEL, PRM_RIDEHEIGHT, NULL, 0.20f);
    h *= 1.5f; h = h*h; h = h*h; h = 2.0f * (float)exp(-3.0*h);
    CA = h*cl + 4.0f*wingca;
	CA_FW = 4 * 1.23f * fwingArea;
	CA_RW = 4 * 1.23f * rwingArea;
	CA_GE = h * cl;

	DEBUGF( "CA %g   CA_FW %g   CA_RW %g   CA_GE %g\n", CA, CA_FW, CA_RW, CA_GE );

	double	cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, NULL, 0.0);
	double	frontArea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, NULL, 0.0);

	CD_BODY = 0.645 * cx * frontArea;
	CD_WING = wingca;

	WIDTH = car->_dimension_y;

    TYRE_MU_F = MN(GfParmGetNum(car->_carHandle, SECT_FRNTRGTWHEEL, PRM_MU, NULL, 1.0),
                   GfParmGetNum(car->_carHandle, SECT_FRNTLFTWHEEL, PRM_MU, NULL, 1.0));
    TYRE_MU_R = MN(GfParmGetNum(car->_carHandle, SECT_REARRGTWHEEL, PRM_MU, NULL, 1.0),
                   GfParmGetNum(car->_carHandle, SECT_REARLFTWHEEL, PRM_MU, NULL, 1.0));
	TYRE_MU   = MN(TYRE_MU_R, TYRE_MU_R);

	F_AXLE_X  = GfParmGetNum(car->_carHandle, SECT_FRNTAXLE, PRM_XPOS, NULL, 0);
	R_AXLE_X  = GfParmGetNum(car->_carHandle, SECT_REARAXLE, PRM_XPOS, NULL, 0);
	F_AXLE_WB = GfParmGetNum(car->_carHandle, SECT_FRNTAXLE, PRM_FRWEIGHTREP, NULL, 0.5f);
	R_AXLE_WB = 1 - F_AXLE_WB;
	F_AXLE_CG = h * GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, NULL, 0);
	R_AXLE_CG = h * GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, NULL, 0);

	F_WING_X  = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_XPOS, NULL, 0);
	R_WING_X  = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_XPOS, NULL, 0);
}

//===========================================================================

void	CarModel::update( const tCarElt* car, const tSituation* sit )
{
	Vec3d	new_pos_g = Vec3d(car->pub.DynGCg.pos.x, car->pub.DynGCg.pos.y, car->pub.DynGCg.pos.z);
	Vec3d	new_vel_g = (new_pos_g - POS_G) / sit->deltaTime;
	Vec3d	new_acc_g = (new_vel_g - VEL_G) / sit->deltaTime;
	
	POS_G = new_pos_g;
	VEL_G = new_vel_g;
	ACC_G = new_acc_g;

	const sgMat4& m = car->pub.posMat;

	VEL_L.x = VEL_G * Vec3d(m[0][0], m[0][1], m[0][2]);
	VEL_L.y = VEL_G * Vec3d(m[1][0], m[1][1], m[1][2]);
	VEL_L.z = VEL_G * Vec3d(m[2][0], m[2][1], m[2][2]);

	ACC_L.x = ACC_G * Vec3d(m[0][0], m[0][1], m[0][2]);
	ACC_L.y = ACC_G * Vec3d(m[1][0], m[1][1], m[1][2]);
	ACC_L.z = ACC_G * Vec3d(m[2][0], m[2][1], m[2][2]);
	
	double	new_pos_az = car->pub.DynGCg.pos.az;
	double	new_vel_az = Utils::NormPiPi(new_pos_az - POS_AZ) / sit->deltaTime;

	POS_AZ = new_pos_az;
	VEL_AZ = new_vel_az;

//	DEBUGF( "vx %7.4f %7.4f  vy %7.4f %7.4f  vz %7.4f %7.4f  vaz %7.4f %7.4f\n",
//			VEL_L.x, car->pub.DynGC.vel.x,
//			VEL_L.y, car->pub.DynGC.vel.y,
//			VEL_L.z, car->pub.DynGC.vel.z,
//			VEL_AZ, car->pub.DynGC.vel.az );

	updateWheels( car, sit );
}

//===========================================================================

const WheelModel& CarModel::wheel( int wheel ) const
{
	return _wheel[wheel];
}

//===========================================================================

void	CarModel::configWheels( const tCarElt* car )
{
	for( int w = 0; w < 4; w++ )
		_wheel[w].config( car );
}

//===========================================================================

void	CarModel::updateWheels( const tCarElt* car, const tSituation* s )
{
	for( int w = 0; w < 4; w++ )
		_wheel[w].update( car, s, *this );
}

//===========================================================================

double	CarModel::CalcMaxSpeed(
	double k,
	double kz,
	double trackMu,
	double rollAngle,
	double pitchAngle ) const
{
	if( FLAGS & F_SEPARATE_FRONT_REAR )
	    return CalcMaxSpeedAeroNew(k, kz, trackMu, rollAngle, pitchAngle);
	else
	    return CalcMaxSpeedAeroOld(k, kz, trackMu, rollAngle, pitchAngle);
}

//===========================================================================

double	CarModel::CalcMaxSpeedAeroOld(
	double k,
	double kz,
	double trackMu,
	double trackRollAngle,
	double trackPitchAngle ) const
{
	//
	//	Here we calculate the theoretical maximum speed at a point on the
	//	path.  This takes into account the curvature of the path (k), the
	//	grip on the road (mu), the downforce from the wings and the ground
	//	effect (CA), the tilt of the road (left to right slope) (sn)
	//	and the curvature of the road in z (kz).
	//
	//	There are still a few silly fudge factors to make the theory match
	//	with the reality (the car goes too slowly otherwise, aarrgh!).
	//

	double	M  = MASS + FUEL;

	double	mua, muf, mur;

	if( FLAGS & F_OLD_AERO_1 )
	{
		double	MU_F = trackMu * TYRE_MU_F;
		double	MU_R = trackMu * TYRE_MU_R;

		muf = MU_F * MU_SCALE;
		mur = MU_R * MU_SCALE;
		mua = (MU_F + MU_R) * 0.5;
	}
	else
	{
		double	MU = trackMu * TYRE_MU;

		mua   = MU * MU_SCALE;// * 0.975;
	}

//	mua *= (GRIP_SCALE_F + GRIP_SCALE_R) * 0.5;
	mua *= MN(GRIP_SCALE_F, GRIP_SCALE_R);

	double	cs = cos(trackRollAngle) * cos(trackPitchAngle);
	double	sn = sin(trackRollAngle);

	double	absK = MX(0.001, fabs(k));
	double	sgnK = SGN(k);

	double	num, den;

	if( FLAGS & F_OLD_AERO_1 )
	{
		num = M * (cs * G * mua + sn * G * sgnK);
//		den = M * (absK - 0.1 * kz) -
		den = M * (absK - KZ_SCALE * kz) -
					(CA_FW * muf + CA_RW * mur + CA_GE * mua);
	}
	else
	{
//		num = M * (G * mu + sn * G * sgnK);
		num = M * (cs * G * mua + sn * G * sgnK);
//		den = M * (absK - 0.00 * kz) - CA * mu_df;
//		den = M * (absK - 0.10 * kz) - CA * mu_df;
//		den = M * (absK - 0.20 * kz) - CA * mu_df;
//		den = M * (absK - 0.25 * kz) - CA * mu_df;
//		den = M * (absK - 0.29 * kz) - CA * mu_df;
//		den = M * (absK - 0.33 * kz) - CA * mu_df;
//		den = M * (absK - 0.42 * kz) - CA * mu_df;
		den = M * (absK - KZ_SCALE * kz) - CA * mua; //mu_df;
	}

	if( den < 0.00001 )
		den = 0.00001;

	double	spd = sqrt(num / den);

	if( spd > 200 )
		spd = 200;

	return spd;
}

#if 0
double	CarModel::CalcMaxSpeed(
	double k,
	double kz,
	double trackMu,
	double trackRollAngle ) const
{
	//
	//	Here we calculate the theoretical maximum speed at a point on the
	//	path.  This takes into account the curvature of the path (k), the
	//	grip on the road (mu), the downforce from the wings and the ground
	//	effect (CA), the tilt of the road (left to right slope) (sn)
	//	and the curvature of the road in z (kz).
	//
	//	There are still a few silly fudge factors to make the theory match
	//	with the reality (the car goes too slowly otherwise, aarrgh!).
	//

	double	M  = MASS + FUEL;

	double	MU = trackMu * TYRE_MU;
//	MU = MU * (1 - m_pPath[i].pSeg->pSeg->surface->kRoughness * 5);

	double	cs = cos(trackRollAngle);
	double	sn = sin(trackRollAngle);

	double	absK = MX(0.001, fabs(k));
	double	sgnK = SGN(k);

	double	mu    = MU * MU_SCALE;// * 0.975;
	double	mu_df = MU * MU_SCALE;//MU_DF_SCALE;

//	double	beta = 0.5 * 3 * absK;	// estimate of turning angle of front wheels.
//	mu = mu_df = 0.5 * mu * (1 + fabs(cos(beta)));
//	double	beta = 3 * absK;	// estimate of turning angle of front wheels.
//	mu = mu * fabs(cos(beta));

//	double	num = M * (G * mu + sn * G * sgnK);
	double	num = M * (cs * G * mu + sn * G * sgnK);
//	double	den = M * (absK - 0.00 * kz) - CA * mu_df;
//	double	den = M * (absK - 0.10 * kz) - CA * mu_df;
//	double	den = M * (absK - 0.20 * kz) - CA * mu_df;
//	double	den = M * (absK - 0.25 * kz) - CA * mu_df;
//	double	den = M * (absK - 0.29 * kz) - CA * mu_df;
//	double	den = M * (absK - 0.33 * kz) - CA * mu_df;
//	double	den = M * (absK - 0.42 * kz) - CA * mu_df;
	double	den = M * (absK - KZ_SCALE * kz) - CA * mu_df;

	if( den < 0.00001 )
		den = 0.00001;

	double	spd = sqrt(num / den);

	if( spd > 200 )
		spd = 200;

	return spd;
}
#endif

//===========================================================================

double	CarModel::CalcMaxSpeedAeroNew(
	double k,
	double kz,
	double trackMu,
	double rollAngle,
	double pitchAngle ) const
{
	double maxSpeedFrontAxle = AxleCalcMaxSpeed(k, kz, trackMu,
	                                            rollAngle, pitchAngle,
	                                            GRIP_SCALE_F,
//	                                            (GRIP_SCALE_F + GRIP_SCALE_R) * 0.5,
	                                            TYRE_MU_F, F_AXLE_X, F_WING_X,
												F_AXLE_WB, CA_FW, F_AXLE_CG);
	double maxSpeedRearAxle  = AxleCalcMaxSpeed(k, kz, trackMu,
	                                            rollAngle, pitchAngle,
	                                            GRIP_SCALE_R,
//	                                            (GRIP_SCALE_F + GRIP_SCALE_R) * 0.5,
	                                            TYRE_MU_R, R_AXLE_X, R_WING_X,
												R_AXLE_WB, CA_RW, R_AXLE_CG);
	return MN(maxSpeedFrontAxle, maxSpeedRearAxle);
}

//===========================================================================

double	CarModel::AxleCalcMaxSpeed(
	double k,
	double kz,
	double trackMu,
	double trackRollAngle,
	double trackPitchAngle,
	double gripScale,
	double tyreMu,
	double ax,	// axle x position
	double wx,	// wing x position
	double wf,	// axle weight balance factor (e.g. 0.47 for car6)
	double Cw,	// wing downforce constant
	double Cg ) const	// axle ground effect downforce constant -- assumes nominal ride height.
{
	//	This function calculates the theoretical maximum speed for a single axle of 
	//	the car at a point on the path.	 This takes into account the curvature of
	//	the path (k), the grip on the road (mu), the downforce from the nearest
	//	wing and the axle's ground effect (CA), the tilt of the road (angle left to
	//	right), the pitch of the road (angle rear to front), and the curvature of
	//	the road in z (kz).
	//
	//	At this point the car is assumed to be travelling at a constant speed, and
	//	thus any additional weight transfer (apart from the static weight balance)
	//	due to accleration or braking is not taken into consideration.
	//
	//	Also makes a simplifying assumption that the downforce from the other wing
	//	on this axle is small enough to be ignored.
    //
    //  ax      -- axle x position (relative to COG)
    //  wx      -- wing x position (relative to COG)
    //  wf      -- weight factor for this axle (0.47 for car6)
    //  Cw      -- wing downforce constant
    //  Cg      -- axle ground effect downforce constant -- but is dependent on the ride height.
    //  a       -- roll angle
    // 
    //  af = wx / ax            -- wing downforce factor for this axle
    //  Ma = wf M               -- mass supported by axle
    //  Ca = Cg + Cw af         -- total downforce constant for axle
    //  Gz = G * cos(a)         -- downwards acceleration due to gravity
    //  Gy = G * sin(a)         -- sideways acceleration due to gravity
    // 
    //  Az = v^2 Kz             -- intent is that Kz is +ve if resultant acceleration is down
    //  Fg = Ma Gz              -- downwards force due to gravity
    //  Fz = Ma Az = v^2 Ma Kz  -- downwards force due to vertical curvature along track
    //  Fa = v^2 Ca             -- downwards force due to aero on axle
    //  Fy = Ma Gy              -- sideways force due to gravity
    // 
    //  Fd = Fg + Fz + Fa       -- total downwards force
    //  Fd = Ma Gz + v^2 Ma Kz + v^2 Ca
    // 
    //  Fh = MU Fd + Fy         -- maximum horizontal (lateral) force
    //  Fh = MU Fd + Ma Gy
    //  a  = Fh / Ma            -- maximum horizontal (lateral) acceleration
    // 
    //  a = v^2 K
    //  Fh / Ma = v^2 K
    //  v^2 Ma K = Fh
    //  v^2 Ma K = MU Fd + Ma Gy
    //  v^2 Ma K = MU (Ma Gz + v^2    Ma Kz + v^2    Ca) + Ma Gy
    //  v^2 Ma K = MU  Ma Gz + v^2 MU Ma Kz + v^2 MU Ca  + Ma Gy

    //  v^2  Ma K - v^2 MU Ma Kz - v^2 MU Ca  = MU Ma Gz + Ma Gy
    //  v^2 (Ma K -     MU Ma Kz -     MU Ca) = MU Ma Gz + Ma Gy

    //  v^2 = (MU Ma Gz + Ma Gy) / (Ma K - MU Ma Kz - MU Ca)
    //  v^2 = Ma (MU Gz + Gy)    / (Ma K - MU Ma Kz - MU Ca)

	double	absK = MX(0.001, fabs(k));
	double	sgnK = SGN(k);

	double	af = wx / ax;
	double	Ma = wf * (MASS + FUEL);
	double	Ca = Cg + Cw * af;

	double	cs_roll	 = cos(trackRollAngle);
	double	sn_roll	 = sin(trackRollAngle);
	double	cs_pitch = cos(trackPitchAngle);

	double	mu	= trackMu * tyreMu * MU_SCALE * gripScale;
	double	Gz  = cs_roll * cs_pitch * G;
	double	Gy	= sn_roll * G * sgnK;

	double  num = Ma * (mu * Gz + Gy);
	double  den = MX(0.000001, Ma * absK - /*mu * */ Ma * kz * KZ_SCALE - mu * Ca);
	double	spd = sqrt(num / den);

	return MN(spd, 200);
}

//===========================================================================

double	CarModel::CalcBraking(
	double k0, double kz0, double k1, double kz1,
	double spd1, double dist, double trackMu,
	double trackRollAngle, double trackPitchAngle ) const
{
	double	M  = MASS + FUEL;

	double	MU = trackMu * TYRE_MU;
	double	MU_F = MU;
	double	MU_R = MU;
	if( FLAGS & F_OLD_AERO_1 )
	{
		MU_F = trackMu * TYRE_MU_F;
		MU_R = trackMu * TYRE_MU_R;
		MU   = (MU_F + MU_R) * 0.5;
	}

	double	CD = CD_BODY * (1.0 + DAMAGE / 10000.0) + CD_WING;

	// when under braking we keep some grip in reserve.
	MU *= BRAKE_MU_SCALE;

//	MU *= (GRIP_SCALE_F + GRIP_SCALE_R) * 0.5;
	MU *= MN(GRIP_SCALE_F, GRIP_SCALE_R);

	double	K  = (k0  + k1)  * 0.5;
	double	Kz = (kz0 + kz1) * 0.5;// * KZ_SCALE;
//	double	Kz = (kz0 + kz1) * 0.5 * KZ_SCALE;
	if( Kz > 0 )
		Kz = 0;

	double	Gdown = G * cos(trackRollAngle) * cos(trackPitchAngle);
	double	Glat  = G * sin(trackRollAngle);
	double	Gtan  = G * -sin(trackPitchAngle);

	double	v = spd1;
	double	u = v;

//	double	dist = Utils::VecLenXY(m_pPath[i].CalcPt() -
//								   m_pPath[j].CalcPt());

	double	axle_r = (fabs(F_AXLE_X) + fabs(R_AXLE_X)) * 0.5;

	for( int count = 0; count < 100; count++ )
	{
		double	avgV = (u + v) * 0.5;
		double	avgVV = avgV * avgV;

		double	Froad;
		if( FLAGS & F_OLD_AERO_1 )
		{
			double	Fdown = M * Gdown + M * Kz * avgVV + CA_GE * avgVV;
			double	Ffrnt = CA_FW * avgVV;
			double	Frear = CA_RW * avgVV;

			Froad = Fdown * MU + Ffrnt * MU_F + Frear * MU_R;
		}
		else
		{
			double	Fdown = M * Gdown + M * Kz * avgVV + CA * avgVV;

			Froad = Fdown * MU;
		}
		double	Flat  = M * Glat;
		double	Ftan  = M * Gtan - CD * avgVV;

		// Frot = Iz (w1 - w0) / (r t)
		// t  = dist / avgV
		// w1 = v * k1
		// w0 = u * k0
		// r  = average distance of axles from COG.
		// Iz = inertia around z axis.
		double t = dist / avgV;
		double INERTIA_Z = 1000;
		double Frot = INERTIA_Z * fabs(v * k1 - u * k0) / (axle_r * t);
		Froad -= Frot;	// change in rotation speed uses some grip.

		double	Flatroad = fabs(M * avgVV * K - Flat);
		if( Flatroad > Froad )
			Flatroad = Froad;
		double	Ftanroad = -sqrt(Froad * Froad - Flatroad * Flatroad) + Ftan;

		double	acc = Ftanroad / M;// * 0.95;

//		DEBUGF( "%4d K %7.4f  Glat %.3f\n",
//				i, K, Glat );
//		DEBUGF( "%4d K %7.4f  Fr %.3f  Fl %.3f  Ft %.3f  u %.1f  acc %.1f\n",
//				i, K, Froad, Flat, Ftan, u, acc );
//		DEBUGF( "%4d K %7.4f  Flr %.3f  Ftr %.3f  u %.1f  acc %.1f\n",
//				i, K, Flatroad, Ftanroad, u, acc );

		double	inner = MX(0, v * v - 2 * acc * dist );
		double	oldU = u;
		u = sqrt(inner);
		if( fabs(u - oldU) < 0.001 )
			break;
	}

	return u;
}

//===========================================================================

double	CarModel::CalcAcceleration(
	double k0, double kz0, double k1, double kz1,
	double spd0, double dist, double trackMu,
	double trackRollAngle, double trackPitchAngle ) const
{
	double	M  = MASS + FUEL;
	double	MU = trackMu * TYRE_MU;
	double	CD = CD_BODY * (1.0 + DAMAGE / 10000.0) + CD_WING;

	// when under braking we keep some grip in reserve.
	MU *= BRAKE_MU_SCALE;

//	MU *= (GRIP_SCALE_F + GRIP_SCALE_R) * 0.5;
	MU *= MN(GRIP_SCALE_F, GRIP_SCALE_R);

	double	K  = (k0  + k1)  * 0.5;
	double	Kz = (kz0 + kz1) * 0.5;// * KZ_SCALE;
//	double	Kz = (kz0 + kz1) * 0.5 * KZ_SCALE;
	if( Kz > 0 )
		Kz = 0;

	double	Gdown = G * cos(trackRollAngle) * cos(trackPitchAngle);
	double	Glat  = G * sin(trackRollAngle);
	double	Gtan  = G * -sin(trackPitchAngle);

	double	u = spd0;
	double	v = u;

	// 30m/ss @ 0m/s
	//  3m/ss @ 60m/s
	//	1m/ss @ 75m/s
	//	0m/ss @ 85m/s
	Quadratic	accFromSpd(21.0/5400, -43.0/60, 30);	// approx. clkdtm

	// Power (kW) = Torque (Nm) x Speed (RPM) / 9.5488

	double	axle_r = (fabs(F_AXLE_X) + fabs(R_AXLE_X)) * 0.5;

	for( int count = 0; count < 100; count++ )
	{
		double	avgV = (u + v) * 0.5;
		double	vv = avgV * avgV;

		double	Fdown = M * Gdown + M * Kz * vv + CA * vv;
		double	Froad = Fdown * MU;
		double	Flat  = M * Glat;
		double	Ftan  = M * Gtan - CD * vv;

		// Frot = Iz (w1 - w0) / (r t)
		// t  = dist / avgV
		// w1 = v * k1
		// w0 = u * k0
		// r  = average distance of axles from COG.
		// Iz = inertia around z axis.
		double t = dist / avgV;
		double INERTIA_Z = 1000;
		double Frot = INERTIA_Z * fabs(v * k1 - u * k0) / (axle_r * t);
		Froad -= Frot;	// change in rotation speed uses some grip.

		double	Flatroad = fabs(M * vv * K - Flat);
		if( Flatroad > Froad )
			Flatroad = Froad;
		double	Ftanroad = sqrt(Froad * Froad - Flatroad * Flatroad) + Ftan;

		double	acc = Ftanroad / M;
		double	maxAcc = accFromSpd.CalcY(avgV);
		if( acc > maxAcc )
			acc = maxAcc;

		double	inner = MX(0, u * u + 2 * acc * dist );
		double	oldV = v;
		v = sqrt(inner);
		if( fabs(v - oldV) < 0.001 )
			break;
	}

	if( v < u )
		int f = 1;

	return MX(u, v);
}

//===========================================================================

double	CarModel::CalcMaxSpdK() const
{
	const double	MAX_SPD = 110;	// ~400 kph
	double	maxSpdK = G * TYRE_MU / (MAX_SPD * MAX_SPD);
	return maxSpdK;
}

//===========================================================================

double	CarModel::CalcMaxLateralF( double spd, double trackMu ) const
{
	double	M  = MASS + FUEL;
	double	MU = trackMu * TYRE_MU;	// * GRIP_SCALE; ???

	double	vv = spd * spd;

	double	Fdown = M * G + /*M * Kz * vv*/ + CA * vv;
	double	Flat  = Fdown * MU;

	return Flat;
}

//===========================================================================

void	CarModel::CalcSimuSpeeds(
	double	spd0,
	double	dy,
	double	dist,
	double	trackMu,
	double&	minSpd,
	double&	maxSpd ) const
{
	// simple speed calc for use in simulation for path optimisation... the
	//	overriding pre-requisite of which is speed of calculation.
	//
	// a = v*v/r
	// max_a = M * G * MU;
	// max_spd = sqrt(max_a r) = sqrt(M * G * MU / k)

	double	M  = MASS + FUEL;
//	double	MU = trackMu * TYRE_MU * (GRIP_SCALE_F + GRIP_SCALE_R) * 0.5;
	double	MU = trackMu * TYRE_MU * MN(GRIP_SCALE_F, GRIP_SCALE_R);

	double	max_acc = G * MU;
//	double	max_spd = k == 0 ? 200 : MN(200, sqrt(max_acc / k));

	//	s = ut + 0.5 att = dy
	//	a = 2(dy - ut) / tt      ... but lateral u = 0
	double	estT = dist / spd0;

	double	lat_acc = 2 * dy / (estT * estT);
	if( lat_acc > max_acc )
		lat_acc = max_acc;
	double	lin_acc = sqrt(max_acc * max_acc - lat_acc * lat_acc);

	//
	// accelerate
	//

	// acceleration is limited by engine power... and this quadratic
	//	is an estimation (poor, but hopefully good enough for our purposes).
	static const Quadratic	accFromSpd(21.0/5400, -43.0/60, 30);
	double	eng_acc = accFromSpd.CalcY(spd0) * trackMu;
	if( eng_acc > lin_acc )
		eng_acc = lin_acc;

	maxSpd = sqrt(spd0 * spd0 + 2 * eng_acc * dist);
//	if( maxSpd > max_spd )
//		maxSpd = max_spd;

	//
	// brake
	//

	minSpd = sqrt(spd0 * spd0 - 2 * lin_acc * dist);
}

//===========================================================================

void	CarModel::CalcSimuSpeedRanges(
	double	spd0,
	double	dist,
	double	trackMu,
	double&	minSpd,
	double&	maxSpd,
	double&	maxDY ) const
{
	// simple speed calc for use in simulation for path optimisation... the
	//	overriding pre-requisite of which is speed of calculation.
	//
	// a = v*v/r
	// max_a = M * G * MU;
	// max_spd = sqrt(max_a r) = sqrt(M * G * MU / k)

	double	M  = MASS + FUEL;
//	double	MU = trackMu * TYRE_MU * (GRIP_SCALE_F + GRIP_SCALE_R) * 0.5;
	double	MU = trackMu * TYRE_MU * MN(GRIP_SCALE_F, GRIP_SCALE_R);

	double	max_acc = G * MU;

	//
	// accelerate
	//

	// acceleration is limited by engine power... and this quadratic
	//	is an estimation (poor, but hopefully good enough for our purposes).
	static const Quadratic	accFromSpd(21.0/5400, -43.0/60, 30);
	double	eng_acc = accFromSpd.CalcY(spd0) * trackMu;
	if( eng_acc > max_acc )
		eng_acc = max_acc;

	maxSpd = sqrt(spd0 * spd0 + 2 * eng_acc * dist);

	//
	// brake
	//

	minSpd = sqrt(spd0 * spd0 - 2 * max_acc * dist);

	//
	// turn (turning is symmetrical)
	//

	// s = ut + 1/2 att    u = 0, as we're looking along vel vector.
	// t = dist / spd0;
	double	turnT = dist / spd0;
	maxDY = 0.5 * max_acc * turnT * turnT;
}

//===========================================================================
