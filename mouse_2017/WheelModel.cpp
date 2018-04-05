
#include <robottools.h>

#include "WheelModel.h"
#include "CarModel.h"
#include "Utils.h"

WheelModel::WheelModel()
:   _w(0),
    _X(0),
    _Y(0),
    _Z(0),
    _B(0),
    _C(0),
    _E(0),
    _MU(1),
    _x(0),
    _y(0),
    _z(0),
    _sx(0),
    _sy(0),
    _sa(0)
{
    memset( &_tp, 0, sizeof(_tp));
}

WheelModel::~WheelModel()
{
}

void    WheelModel::setWheel( int wheel )
{
    _w = wheel;
}

void    WheelModel::config( const tCarElt* car )
{
    static const char* axleSect[2] = {SECT_FRNTAXLE, SECT_REARAXLE};
    static const char* wheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};

	_X = GfParmGetNum(car->_carHandle, axleSect[_w/2], PRM_XPOS, (char*)NULL, 0.0f);
	_Y = GfParmGetNum(car->_carHandle, wheelSect[_w], PRM_YPOS, (char*)NULL, 0.0f);
	_Z = 0;
	
	_X -= car->info.statGC.x;
	_Y -= car->info.statGC.y;
	_Z -= car->info.statGC.z;
	
	_MU = GfParmGetNum(car->_carHandle, wheelSect[_w], PRM_MU, (char*)NULL, 1.0);

	double  Ca      = GfParmGetNum(car->_carHandle, wheelSect[_w], PRM_CA, (char*)NULL, 30.0f);
	double  RFactor = GfParmGetNum(car->_carHandle, wheelSect[_w], PRM_RFACTOR, (char*)NULL, 0.8f);
	double  EFactor = GfParmGetNum(car->_carHandle, wheelSect[_w], PRM_EFACTOR, (char*)NULL, 0.7f);

//	_lfMax          = GfParmGetNum(car->_carHandle, wheelSect[_w], PRM_LOADFMAX, (char*)NULL, 1.6f);
//	_lfMin          = GfParmGetNum(car->_carHandle, wheelSect[_w], PRM_LOADFMIN, (char*)NULL, 0.8f);
//	_opLoad         = GfParmGetNum(car->_carHandle, wheelSect[_w], PRM_OPLOAD, (char*)NULL, wheel->weight0 * 1.2f);

    _C = 2 - asin(RFactor) * 2 / PI;
    _B = Ca / _C;
    _E = EFactor;
}

void    WheelModel::update( const tCarElt* car, const tSituation* sit, const CarModel& cm )
{
    updatePosition( car, sit );
    updateSlip( car, sit, cm );
}

void    WheelModel::updatePosition( const tCarElt* car, const tSituation* sit )
{
	const sgMat4& m = car->pub.posMat;
//	_x = car->pub.DynGCg.pos.x + m[0][0] * _X + m[1][0] * _Y + m[2][0] * _Z;
//	_y = car->pub.DynGCg.pos.y + m[0][1] * _X + m[1][1] * _Y + m[2][1] * _Z;
//	_z = car->pub.DynGCg.pos.z + m[0][2] * _X + m[1][2] * _Y + m[2][2] * _Z;
	_x = car->pub.DynGCg.pos.x + m[0][0] * _X + m[0][1] * _Y + m[0][2] * _Z;
	_y = car->pub.DynGCg.pos.y + m[1][0] * _X + m[1][1] * _Y + m[1][2] * _Z;
	_z = car->pub.DynGCg.pos.z + m[2][0] * _X + m[2][1] * _Y + m[2][2] * _Z;
	
	RtTrackGlobal2Local(car->pub.trkPos.seg, (tdble)_x, (tdble)_y, &_tp, TR_LPOS_SEGMENT);
//  _friction = _p.seg->surface->kFriction;
	_vay = (_vay + car->_wheelSpinVel(_w)) * 0.5;
}

void    WheelModel::updateSlip( const tCarElt* car, const tSituation* sit, const CarModel& cm )
{
    double  zforce  = car->_reaction[_w];
    if( zforce == 0 )
    {
        // in air?
        _sx = _sy = _sa = 0;
        return;
    }

	if( car->pub.speed < 0.5 )
	{
        _sx = _w < 2 ? 0 : car->ctrl.accelCmd * 0.5;
        _sy = _sa = 0;
        return;
	}

//	double  bvx = car->pub.DynGC.vel.x - car->pub.DynGC.vel.az * _Y;
//	double  bvy = car->pub.DynGC.vel.y + car->pub.DynGC.vel.az * _X;
	double  bvx = cm.VEL_L.x - cm.VEL_AZ * _Y;
	double  bvy = cm.VEL_L.y + cm.VEL_AZ * _X;
	double  bv  = hypot(bvx, bvy);

	double  waz = _w < 2 ? car->ctrl.steer * car->info.steerLock : 0;
    double  wrl = _vay * car->_wheelRadius(_w);

	if( bv < 0.000001f )
	{
		_sa = 0;
		_sx = wrl;
		_sy = 0;
	}
	else
	{
	    _sa = atan2(bvy, bvx) - waz;
	    NORM_PI_PI(_sa);

	    double  vt = bvx * cos(waz) + bvy * sin(waz);
		_sx = (vt - wrl) / fabs(vt);
		_sy = sin(_sa); // should be tan(_sa)???
	}
	
//	double  sv      = hypot(car->_wheelSlipSide(_w), car->_wheelSlipAccel(_w));
//	double  s       = car->_skid[_w] / (zforce * 0.0002f);
//	GfOut( "[%d] sv(%7.3f %6.3f) zf:%4.0f v(b%7.3f w%7.3f s%7.3f) s(%6.3f x%7.3f y%6.3f a%6.1f)\n",
//			_w, car->_wheelSlipAccel(_w), car->_wheelSlipSide(_w), zforce, bv, wrl, sv, s, _sx, _sy, _sa * 180 / PI );
}

