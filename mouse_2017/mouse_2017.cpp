/***************************************************************************

    file        : mouse_2017.cpp
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

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "Vec3d.h"
#include "MyRobot.h"
#include "Shared.h"

static const int	NBOTS = MAX_MOD_ITF;//10;//2;
static MyRobot		s_robot[NBOTS];
static Shared		s_shared;

static void initTrack( int index, tTrack* track, void* carHandle,
						void** carParmHandle, tSituation* s);
static void newRace( int index, tCarElt* car, tSituation* s );
static void drive( int index, tCarElt* car, tSituation* s );
static int pitcmd( int index, tCarElt* car, tSituation* s );
static void endRace( int index, tCarElt* car, tSituation* s );
static void shutdown( int index);
static int InitFuncPt( int index, void* pt );


// Module entry point.
extern "C" int mouse_2017( tModInfo* modInfo )
{
	// Clear all structures.
	memset( modInfo, 0, MAX_MOD_ITF * sizeof(tModInfo) );
//	memset( modInfo, 0, sizeof(tModInfo) );

	static char	name[NBOTS][20];
	static char	desc[NBOTS][20];

	printf( "mouse %d bots\n", NBOTS );
	for( int i = 0; i < NBOTS; i++ )
	{
		sprintf( name[i], "Mouse 2017 %d", i + 1 );
		sprintf( desc[i], "Mouse 2017 %d", i + 1 );

		modInfo[i].name    = strdup((char*)&name[i]);	// name of the module (short).
		modInfo[i].desc    = strdup((char*)&desc[i]); // Description of the module (can be long).
		modInfo[i].fctInit = InitFuncPt;		// Init function.
		modInfo[i].gfId    = ROB_IDENT;		// Supported framework version.
		modInfo[i].index   = i;				// Indexes
	}

	return 0;
}


// Module interface initialization.
static int InitFuncPt( int index, void* pt )
{
//	DEBUGF( "mouse_2017:InitFuncPt()\n" );

	tRobotItf* itf = (tRobotItf*)pt;

	// Create robot instance for index.
	itf->rbNewTrack = initTrack;	// Give the robot the track view called.
	itf->rbNewRace  = newRace;		// Start a new race.
	itf->rbDrive    = drive;		// Drive during race.
	itf->rbPitCmd   = pitcmd;		// Pit commands.
	itf->rbEndRace  = endRace;		// End of the current race.
	itf->rbShutdown = shutdown;		// Called before the module is unloaded.
	itf->index      = index;		// Index used if multiple interfaces.

//	DEBUGF( "  itf->rbNewTrack    %p\n", itf->rbNewTrack );
//	DEBUGF( "  itf->rbNewRace     %p\n", itf->rbNewRace );
//	DEBUGF( "  itf->rbDrive       %p\n", itf->rbDrive );
//	DEBUGF( "  itf->rbPitCmd      %p\n", itf->rbPitCmd );
//	DEBUGF( "  itf->rbEndRace     %p\n", itf->rbEndRace );
//	DEBUGF( "  itf->rbShutdown    %p\n", itf->rbShutdown );
//	DEBUGF( "  itf->index         %d\n", itf->index );

	return 0;
}


// Called for every track change or new race.
static void initTrack( int index, tTrack* track, void* carHandle,
						void** carParmHandle, tSituation* s )
{
	s_robot[index].SetShared( &s_shared );
	s_robot[index].InitTrack(index, track, carHandle, carParmHandle, s);
}


// Start a new race.
static void newRace( int index, tCarElt* car, tSituation* s )
{
	s_robot[index].NewRace( index, car, s );
}

// Drive during race.
static void drive( int index, tCarElt* car, tSituation* s )
{
	s_robot[index].Drive( index, car, s );
}


// Pitstop callback.
static int pitcmd( int index, tCarElt* car, tSituation* s )
{
	return s_robot[index].PitCmd(index, car, s);
}


// End of the current race.
static void endRace( int index, tCarElt* car, tSituation* s )
{
	s_robot[index].EndRace( index, car, s );
}


// Called before the module is unloaded.
static void shutdown( int index )
{
	s_robot[index].Shutdown( index );
}

