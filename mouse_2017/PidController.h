/***************************************************************************

    file        : PidController.h
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

// PidController.h: interface for the PidController class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PIDCONTROLLER_H__568AA054_D484_4EA4_A1F1_8E77B673207E__INCLUDED_)
#define AFX_PIDCONTROLLER_H__568AA054_D484_4EA4_A1F1_8E77B673207E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class PidController  
{
public:
	PidController();
	virtual ~PidController();

	double	Sample( double propValue );
	double	Sample( double propValue, double diffValue );

public:
	double	m_lastPropValue;	// for calculating differential (if not supplied)
	double	m_total;			// for integral.
	double	m_maxTotal;			// for integral.
	double	m_totalRate;		// for integral.

	double	m_p;
	double	m_i;
	double	m_d;
};

#endif // !defined(AFX_PIDCONTROLLER_H__568AA054_D484_4EA4_A1F1_8E77B673207E__INCLUDED_)
