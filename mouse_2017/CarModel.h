/***************************************************************************

    file        : CarModel.h
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

// CarModel.h: interface for the CarModel class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CARMODEL_H__60AA9DE0_703F_4684_BCBE_35873DC5C9A6__INCLUDED_)
#define AFX_CARMODEL_H__60AA9DE0_703F_4684_BCBE_35873DC5C9A6__INCLUDED_

#include "WheelModel.h"
#include "Vec3d.h"

class CarModel  
{
public:
	enum
	{
		F_NONE					= 0x00,
		F_OLD_AERO_1			= 0x01,
		F_SEPARATE_FRONT_REAR	= 0x02,
		F_USE_PATH_PITCH		= 0x04,
	};

public:
	CarModel();
	~CarModel();

    void    config( const tCarElt* car );
	void	update( const tCarElt* car, const tSituation* sit );

	double	CalcMaxSpeed( double k, double kz, double kFriction,
						  double rollAngle, double pitchAngle ) const;

	double	CalcBraking( double k0, double kz0, double k1, double kz1,
						  double spd1, double dist, double kFriction,
						  double trackRollAngle, double pitchAngle ) const;

	double	CalcAcceleration( double k0, double kz0, double k1, double kz1,
						  double spd0, double dist, double kFriction,
						  double trackRollAngle, double pitchAngle ) const;
	double	CalcMaxSpdK() const;
	double	CalcMaxLateralF( double spd, double kFriction ) const;

	void	CalcSimuSpeeds( double spd0, double dy, double dist, double kFriction,
							double& minSpd, double& maxSpd ) const;
	void	CalcSimuSpeedRanges( double spd0, double dist, double kFriction,
								 double& minSpd, double& maxSpd, double& maxDY ) const;

	const WheelModel&	wheel( int wheel ) const;
	void				configWheels( const tCarElt* car );
	void				updateWheels( const tCarElt* car, const tSituation* s );

private:
	double	CalcMaxSpeedAeroOld(double k, double kz, double kFriction,
						        double rollAngle, double pitchAngle ) const;

	double	CalcMaxSpeedAeroNew(double k, double kz, double kFriction,
						        double rollAngle, double pitchAngle ) const;

	double	AxleCalcMaxSpeed(   double k, double kz, double kFriction,
				                double trackRollAngle, double trackPitchAngle,
	                            double gripScale, double tyreMu, double axleX, double wingX,
	                            double weightBalanceFactor,
				                double wingDownforceConst, double axleGroundEffectConst ) const;

public:
	int		FLAGS;			// options that modify calculations
	double	MASS;			// fixed mass of car.
	double	FUEL;			// mass of fuel in car.
	double	DAMAGE;			// damage of this car.
	double	TYRE_MU;		// mu value of tyres (min of those avail).
	double	TYRE_MU_F;		// mu value of front tyres.
	double	TYRE_MU_R;		// mu value of rear  tyres.
	double	MU_SCALE;		// scaling of MU to use for this car.
	double	BRAKE_MU_SCALE;	// extra scaling of MU to apply when braking.
	double	GRIP_SCALE_F;	// scaling of grip due to condition of front tyres.
	double	GRIP_SCALE_R;	// scaling of grip due to condition of rear  tyres.
	double	CA;				// aerodynamic downforce constant -- total.
	double	CA_FW;			// aerodynamic downforce constant -- front wing.
	double	CA_RW;			// aerodynamic downforce constant -- rear wing.
	double	CA_GE;			// aerodynamic downforce constant -- ground effect.
	double	CD_BODY;		// aerodynamic drag constant -- car body.
	double	CD_WING;		// aerodynamic drag constant -- wings.
	double	KZ_SCALE;		// bump sensitivity.
	double	WIDTH;			// width of car (m).

	Vec3d	POS_G;			// position in global coords.
	Vec3d	VEL_G;			// velocity in global coords.
	Vec3d	ACC_G;			// acceleration in global coords.
	Vec3d	VEL_L;			// velocity in local  coords.
	Vec3d	ACC_L;			// acceleration in local  coords.
	double	POS_AZ;			// angle around z axis.
	double	VEL_AZ;			// speed around z axis.
	
	double	F_AXLE_X;		// front axle x position
	double	R_AXLE_X;		// rear  axle x position
	double	F_AXLE_WB;		// front axle weight balance (fraction of mass on axle)
	double	R_AXLE_WB;		// rear  axle weight balance (fraction of mass on axle)
	double	F_AXLE_CG;		// front axle ground effect constant
	double	R_AXLE_CG;		// rear  axle ground effect constant

	double	F_WING_X;		// front wing x position
	double	R_WING_X;		// rear  wing x position

	WheelModel	_wheel[4];
};

#endif // !defined(AFX_CARMODEL_H__60AA9DE0_703F_4684_BCBE_35873DC5C9A6__INCLUDED_)
