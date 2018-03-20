/***************************************************************************

    file                 : driver.cpp
    created              : Thu Dec 20 01:21:49 CET 2002
    copyright            : (C) 2002-2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: driver.cpp,v 1.16 2006/04/27 22:32:29 berniw Exp $

    ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

//#define DRV_DEBUG 
//#define OVERTAKE_DEBUG

#include <stdio.h>
#include <string>
#include <tgfclient.h>

#include "driver.h"
#include "vardef.h"
#include "xmldefs.h"

#ifdef _MSC_VER
#if _MSC_VER <= 1500
#define IsNan _isnan
#define IsFinite _finite
#endif
#endif

#ifndef IsNan
#define IsNan isnan
#define IsFinite isfinite
#endif

const float Driver::MAX_UNSTUCK_ANGLE = (float)(15.0f / 180.0f*PI);    // [radians] If the angle of the car on the track is smaller, we assume we are not stuck.
const float Driver::MAX_REALLYSTUCK_ANGLE = 1.6;
const float Driver::UNSTUCK_TIME_LIMIT = 5.0f;                // [s] We try to get unstuck after this time.
const float Driver::UNSTUCK2_TIME_LIMIT = 15.0f;                // [s] We try to get unstuck after this time.
const float Driver::UNSTUCK3_TIME_LIMIT = 30.0f;                // [s] We try to get unstuck after this time.
const float Driver::MAX_UNSTUCK_SPEED = 1.5f;                // [m/s] Below this speed we consider being stuck.
const float Driver::MIN_UNSTUCK_DIST = 1.0f;                // [m] If we are closer to the middle we assume to be not stuck. <2.0>
const float Driver::G = 9.81f;                        // [m/(s*s)] Welcome on Earth.
const float Driver::FULL_ACCEL_MARGIN = 1.0f;                // [m/s] Margin reduce oscillation of brake/acceleration.

const float Driver::SHIFT = 0.98f;                    // [-] (% of rpmredline) When do we like to shift gears. <0.96>
const float Driver::SHIFT_UP = 0.99f;                    // [-] (% of rpmredline) 
const float Driver::SHIFT_DOWN = 120;
const float Driver::SHIFT_MARGIN = 4.0f;                // [m/s] Avoid oscillating gear changes.

const float Driver::ABS_SLIP = 2.5f;                    // [m/s] range [0..10]
const float Driver::ABS_RANGE = 5.0f;                    // [m/s] range [0..10]
const float Driver::ABS_MINSPEED = 3.0f;                // [m/s] Below this speed the ABS is disabled (numeric, division by small numbers).
const float Driver::TCL_SLIP = 2.0f;                    // [m/s] range [0..10]
const float Driver::TCL_RANGE = 10.0f;                    // [m/s] range [0..10]

const float Driver::LOOKAHEAD_CONST = 15.0f;                // [m]
const float Driver::LOOKAHEAD_FACTOR = 0.33f;                // [-]
const float Driver::WIDTHDIV = 2.0f;                    // [-] Defines the percentage of the track to use (2/WIDTHDIV). <3.0>
const float Driver::SIDECOLL_MARGIN = 3.0f;                // [m] Distance between car centers to avoid side collisions. 
const float Driver::BORDER_OVERTAKE_MARGIN = 0.5f;            // [m]
const float Driver::OVERTAKE_OFFSET_SPEED = 5.0f;            // [m/s] Offset change speed.
const float Driver::PIT_LOOKAHEAD = 10.0f;                // [m] Lookahead to stop in the pit.
const float Driver::PIT_BRAKE_AHEAD = 200.0f;                // [m] Workaround for "broken" pitentries.
const float Driver::PIT_MU = 0.4f;                    // [-] Friction of pit concrete.
const float Driver::MAX_SPEED = 80.0f;                    // [m/s] Speed to compute the percentage of brake to apply.
const float Driver::MAX_FUEL_PER_METER = 0.0007f;            // [liter/m] fuel consumtion.
const float Driver::CLUTCH_SPEED = 5.0f;                    // [m/s]
const float Driver::CENTERDIV = 0.1f;                        // [-] (factor) [0.01..0.6].
const float Driver::DISTCUTOFF = 200.0f;                    // [m] How far to look, terminate while loops.
const float Driver::MAX_INC_FACTOR = 6.0f;                    // [m] Increment faster if speed is slow [1.0..10.0].
const float Driver::CATCH_FACTOR = 3.0f;                    // [-] select MIN(catchdist, dist*CATCH_FACTOR) to overtake.
const float Driver::CLUTCH_FULL_MAX_TIME = 2.0f;            // [s] Time to apply full clutch.
const float Driver::USE_LEARNED_OFFSET_RANGE = 0.2f;            // [m] if offset < this use the learned stuff

const float Driver::TEAM_REAR_DIST = 50.0f;                    //
const int Driver::TEAM_DAMAGE_CHANGE_LEAD = 8000;            // When to change position in the team?

// Static variables.
static int pitstatus[128] = { 0 };
Cardata *Driver::cardata = NULL;
double Driver::currentsimtime;

/***************************************************************************/

Driver::Driver(int index)
{
    INDEX = index;
    overtake_test_timer = (index ? 0.0 : 0.15);
    moduleName = BOT_NAME;
    stucksteer = -20.0;
    brakemargin = 0.0;
    stuck_stopped_timer = stuck_reverse_timer = -1.0f;
    stuck_damage = 0;
    mode = no_mode;
    racelineDrivedata = NULL;
    overrideCollection = NULL;
    linemode = NULL;
    avoidmode = stuck = 0;
	lastmode = LINE_NONE;
    speedangle = angle = myoffset = laststeer = lastNSasteer = coll_brake_timpact = coll_brake_boost = 0.0f;
    car_Mass = ftank_Mass = brake_coefficient = 0.0;
    stucksteer = 0.0f;
    situation = NULL;
    car = NULL;
    raceline = NULL;
    line = NULL;
    opponents = NULL;
    opponent = NULL;
    pit = NULL;
    strategy = NULL;
    cardata = NULL;
    mycardata = NULL;
    currentsimtime = 0.0;
    test_raceline = test_rnd_raceline = 0;
    outside_overtake_inhibitor = 1.0f;
    simtime = avoidtime = correcttimer = correctlimit = overtake_timer = 0.0;
	pitTimer = -30.0;
    brakedelay = CornerSpeed = setAccel = LetPass = m_fuelPerMeter = 0.0;
    displaySetting = modeVerbose = m_fuelStrat = m_maxDammage = m_testPitstop = 0;
    m_testQualifTime = m_lineIndex = m_strategyverbose = LineK1999 = bumpCaution = 0;
    tcl_slip = tcl_range = abs_slip = abs_range = 0.0;
    brakemargin = currentspeedsqr = clutchtime = oldlookahead = racesteer = 0.0f;
    rlookahead = raceoffset = avoidlftoffset = avoidrgtoffset = racespeed = 0.0f;
    avoidspeed = accelcmd = brakecmd = PitOffset = brakeratio = 0.0f;
    racetarget.x = 0.0; racetarget.y = 0.0;
    radius = NULL;
    carindex = 0;
	alone = aloneTeam = underThreat = avoidCritical = false;
    suspHeight = 0.0;
    gear_shift = gear_shift_up = gear_shift_down = 0.0;
    MAX_UNSTUCK_COUNT = MAX_UNSTUCK_COUNT2 = MAX_UNSTUCK_COUNT3 = 0;
    CARMASS = FUEL_FACTOR = FCA = RCA = FWA = CR = 0.0;
    CA = CW = TIREMU = OVERTAKE_OFFSET_INC = MU_FACTOR = COAST_ACCEL = 0.0f;
    avgLateralMovt = avgYawRateDelta = prevYawRate = prevToLeft = average_AX = average_AY = 0.0;
    deltaTime = 0.0;
    for (int i=0; i<4; i++)
        speedAngle[i] = 0;
    speedAdvance = 10;
    speedDivisor = 200;
    m_lastWSide = -1;
    m_lastWCount = 0;
	tclf = 0.9;
    left_overtake_caution = right_overtake_caution = overtake_caution = baseBrake = 1.0;
}

/***************************************************************************/

Driver::~Driver()
{
    delete opponents;
    delete pit;
    delete[] radius;
    delete strategy;
    delete raceline;
    if (line)
        delete line;
    if (linemode)
        delete linemode;
    if (overrideCollection)
        delete overrideCollection;
    if (racelineDrivedata)
        delete racelineDrivedata;
    if (cardata != NULL) {
        delete cardata;
        cardata = NULL;
    }
}

/***************************************************************************/


// Called for every track change or new race.
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
    track = t;

    const int BUFSIZE = 256;
    char buffer[BUFSIZE];
    char carName[BUFSIZE];
    /*------------------------------------------------------*/
    /*     Load a custom setup if one is available.    */
    /*------------------------------------------------------*/
    // Get a pointer to the first char of the track filename.
    char* trackname = strrchr(track->filename, '/') + 1;

    // Setup for this robot
    void *newParmHandle;
    *carParmHandle = NULL;
    newParmHandle = NULL;
    sprintf(buffer, "drivers/%s/%d/setup.xml", moduleName, INDEX);
    //newParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
    *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);

    if (*carParmHandle != NULL) {
        m_fuelPerMeter = GfParmGetNum(*carParmHandle, SECT_PRIVATE, BT_ATT_FUELPERMETER, (char*)NULL, 0.00068);
        modeVerbose = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_VERBOSE, (char*)NULL, 0.0);
        displaySetting = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_DISPLAYSETTING, (char*)NULL, 0.0);
        m_testPitstop = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_PIT_TEST, (char*)NULL, 0.0);
        m_testQualifTime = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_QUALIF_TEST, (char*)NULL, 0.0);
        m_lineIndex = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_LINE_INDEX, (char*)NULL, 0.0);
        m_strategyverbose = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_STRATEGY_VERBOSE, (char*)NULL, 0.0);
        m_steerverbose = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_STEER_VERBOSE, (char*)NULL, 0.0);
        newParmHandle = *carParmHandle;
    }
    else {
        m_fuelPerMeter = 0.00068;
        m_fuelStrat = 1;
        m_maxDammage = 5000;
        m_testPitstop = 0;
        m_testQualifTime = 0;
        m_lineIndex = 0;
        m_strategyverbose = 0;
        m_steerverbose = 0;
        modeVerbose = 0;
        displaySetting = 0;
    }

    const char *car_sect = SECT_GROBJECTS "/" LST_RANGES "/" "1";
    strncpy(carName, GfParmGetStr(carHandle, car_sect, PRM_CAR, ""), sizeof(carName));
    char *p = strrchr(carName, '.');
    if (p)
        *p = '\0';
    *carParmHandle = NULL;
    switch (s->_raceType)
    {
    case RM_TYPE_PRACTICE:
        sprintf(buffer, "drivers/%s/%s/practice/%s", moduleName, carName, trackname);
        break;
    case RM_TYPE_QUALIF:
        sprintf(buffer, "drivers/%s/%s/qualifying/%s", moduleName, carName, trackname);
        break;
    case RM_TYPE_RACE:
        sprintf(buffer, "drivers/%s/%s/race/%s", moduleName, carName, trackname);
        break;
    default:
        break;
    }

    *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);

    // if no xml file in race type folder, load the parameters from  race directory
    if (*carParmHandle == NULL) {
        sprintf(buffer, "drivers/%s/%s/race/%s", moduleName, carName, trackname);
        *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
    }

    // or Parameters by defaulft for all tracks
    if (*carParmHandle == NULL) {
        std::cout << "Can't load the xml! " << buffer << std::endl;
        sprintf(buffer, "drivers/%s/%s/default.xml", moduleName, carName);
        *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
        if (*carParmHandle)
            std::cout << "Default XML loaded" << std::endl;
    }
    else
        std::cout << "XML loaded - " << buffer << std::endl;
    if (*carParmHandle != NULL && newParmHandle != NULL)
    {
        *carParmHandle = GfParmMergeHandles(*carParmHandle, newParmHandle, (GFPARM_MMODE_SRC | GFPARM_MMODE_DST | GFPARM_MMODE_RELSRC | GFPARM_MMODE_RELDST));
    }

    /*-----------------------------------------------------*/
    /*              Define a Strategy         */
    /*-----------------------------------------------------*/
    // Simple management of fuel and repairs
    strategy = new SimpleStrategy();
    strategy->setFuelAtRaceStart(t, carHandle, carParmHandle, s, INDEX);

    MU_FACTOR = 0.69f;
    FUEL_FACTOR = 1.5;
    m_maxDammage = 3500;
    brakedelay = 10.0;
    PitOffset = 10.0;
    LetPass = 0.5;
    tcl_slip = TCL_SLIP;
    tcl_range = TCL_RANGE;
    abs_slip = ABS_SLIP;
    abs_range = ABS_RANGE;
    bumpCaution = 0;
    setAccel = 0;
    CornerSpeed = 15.0;

    CARMASS = 1150.0;
    ftank_Mass = 85.0;
    car_Mass = 1250.0;
    brakeratio = 1.0f;

    double CornerSpeedMid = 15.0, CornerSpeedSlow = 15.0;

    if (*carParmHandle != NULL)
    {
        /* Load data from the xml file and set parameters. */
        MU_FACTOR = GfParmGetNum(*carParmHandle, SECT_PRIVATE, BT_ATT_MUFACTOR, (char*)NULL, 0.69f);
        FUEL_FACTOR = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_FUEL_FACTOR, (char*)NULL, 1.5f);
        COAST_ACCEL = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_COAST_ACCEL, (char*)NULL, 0.0f);
        brakedelay = GfParmGetNum(*carParmHandle, SECT_PRIVATE, BT_ATT_BRAKEDIST, (char*)NULL, 10.0f);
        m_maxDammage = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, BT_ATT_MAXDAMMAGE, (char*)NULL, 5000.0f);
        PitOffset = GfParmGetNum(*carParmHandle, SECT_PRIVATE, BT_ATT_PITOFFSET, (char*)NULL, 10.0f);
        m_fuelStrat = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_PIT_STRATEGY, (char*)NULL, 0.0f);
        LetPass = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_LETPASS, (char*)NULL, 0.6f);
        tcl_slip = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_TCL_SLIP, (char*)NULL, TCL_SLIP);
        tcl_range = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_TCL_RANGE, (char*)NULL, TCL_RANGE);
        abs_slip = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_ABS_SLIP, (char*)NULL, ABS_SLIP);
        abs_range = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_ABS_RANGE, (char*)NULL, ABS_RANGE);
        outside_overtake_inhibitor = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_OUTSIDE_OVERTAKE, (char*)NULL, 1.0f);
        baseBrake = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_BASE_BRAKE, (char*)NULL, 1.0f);
        gear_shift = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_SHIFT, (char*)NULL, SHIFT);
        gear_shift_up = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_SHIFT_UP, (char*)NULL, SHIFT_UP);
        gear_shift_down = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_SHIFT_DOWN, (char*)NULL, SHIFT_DOWN);
        WB4DIST = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_WB4DIST, (char *)NULL, -1000.0f);
        WB4COUNT = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_WB4COUNT, (char *)NULL, 5.0f);
        HMDIST = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_HOLDMIDDIST, (char *)NULL, -1000.0f);

        if (s->_raceType == RM_TYPE_PRACTICE)
        {
            const char *test_raceline_str = GfParmGetStr(*carParmHandle, SECT_PRIVATE, PRV_TEST_RACELINE, "");
            if (!strcmp(test_raceline_str, "left"))
                test_raceline = TR_LFT;
            else if (!strcmp(test_raceline_str, "right"))
                test_raceline = TR_RGT;
            else if (strlen(test_raceline_str) > 2)
                test_raceline = TR_STR;
        }

        CornerSpeed = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_TURN_SPEED, (char *)NULL, -1.0);
        if (CornerSpeed < 0.0)
            CornerSpeed = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_CORNERSPEED, (char *)NULL, 15.0);
        CornerSpeedMid = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_CORNERSPEED_MID, (char *)NULL, CornerSpeed);
        CornerSpeedSlow = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_CORNERSPEED_SLOW, (char *)NULL, CornerSpeed);
        coll_brake_timpact = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_COLLBRAKE_TIMPACT, (char *)NULL, 0.50);
        coll_brake_boost = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_COLLBRAKE_BOOST, (char *)NULL, 1.0);
        double brkpressure = (GfParmGetNum( *carParmHandle, SECT_BRKSYST, PRM_BRKPRESS, (char *) NULL, 0.0f ) / 1000);
        brakeratio -= MIN(0.5, MAX(0.0, brkpressure - 20000.0) / 100000);
        overtake_caution = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_OVERTAKE, (char *)NULL, 1.00);
        left_overtake_caution = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_LEFT_OVERTAKE, (char *)NULL, overtake_caution);
        right_overtake_caution = GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_RIGHT_OVERTAKE, (char *)NULL, overtake_caution);
    }

    /* K1999 raceline */
    raceline = new LRaceLine(this);
    raceline->setCornerSpeeds(CornerSpeed, CornerSpeedMid, CornerSpeedSlow);
    raceline->setBrakeDist(brakedelay);
    raceline->setRwData(track, carParmHandle, s);
    overrideCollection = NULL;
    //int loadTrack = (int)GfParmGetNum(*carParmHandle, SECT_PRIVATE, PRV_LOAD_TRACK, (char *)NULL, 0.0);
    //if (!loadTrack)
    {
        overrideCollection = new LManualOverrideCollection();
        overrideCollection->loadFromFile(trackname, (const char *)carName, s->_raceType);
        //overrideCollection->saveToFile();
        raceline->setOverrides(overrideCollection);
    }
}

/***************************************************************************/

// Start a new race.
void Driver::newRace(tCarElt* car, tSituation *s)
{
    float deltaTime = (float)RCM_MAX_DT_ROBOTS;
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT / deltaTime);
    MAX_UNSTUCK_COUNT2 = int(UNSTUCK2_TIME_LIMIT / deltaTime);
    MAX_UNSTUCK_COUNT3 = int(UNSTUCK3_TIME_LIMIT / deltaTime);
    OVERTAKE_OFFSET_INC = OVERTAKE_OFFSET_SPEED*deltaTime;
	left_toMid = last_left_toMid = car->_trkPos.toMiddle + car->_dimension_y / 2;
	right_toMid = last_right_toMid = car->_trkPos.toMiddle - car->_dimension_y / 2;
	left_speed_y = right_speed_y = 0.0;
    stuck = 0;
    alone = true;
    clutchtime = 0.0f;
    oldlookahead = laststeer = lastNSasteer = 0.0f;
    this->car = car;
    CARMASS = GfParmGetNum(car->_carHandle, (char *)SECT_CAR, (char *)PRM_MASS, NULL, 1150.0f);
    suspHeight = GfParmGetNum(car->_carHandle, SECT_REARLFTSUSP, PRM_SUSPCOURSE, NULL, 0.0f);
    brakemargin = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKE_MARGIN, (char *)NULL, 0.0f);
    brake_coefficient = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKE_COEFFICIENT, (char *)NULL, 1.900f);
    double brake_multiplier = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKE_MULTIPLIER, (char *)NULL, 0.3f);
    double brake_warn_multiplier = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_WARN_MULTIPLIER, (char *)NULL, 0.5f);
    speedAdvance = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_SPEED_ADVANCE, (char *)NULL, 0.0f);
    speedDivisor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_SPEED_DIVISOR, (char *)NULL, 200.0f);
	spinDist = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_SPINDIST, (char *)NULL, -1.0f);
	spinDir = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_SPINDIR, (char *)NULL, 0.0f);

    prevToLeft = car->_trkPos.toLeft;

    myoffset = 0.0f;
    simtime = correcttimer = 0.0;
    correctlimit = 1000.0;
    initCa();
    initCw();
    initCR();
    initTireMu();
    initTCLfilter();
    if (displaySetting) {
        showSetup();
    }

    racelineDrivedata = new RaceLineDriveData();
    memset(racelineDrivedata, 0, sizeof(RaceLineDriveData));

    // Create just one instance of cardata shared by all drivers.
    if (cardata == NULL) {
        cardata = new Cardata(s);
    }
    mycardata = cardata->findCar(car);
    currentsimtime = s->currentTime;
    overtake_timer = 0.0;

    // initialize the list of opponents.
    opponents = new Opponents(s, this, cardata, brake_multiplier, brake_warn_multiplier);
    opponent = opponents->getOpponentPtr();

    strategy->setOpponents(opponents);
    strategy->setTrack(track);

    // Set team mate.
    opponents->setTeamMate(car->_teamname);
    strategy->setTeamMate(opponents->getTeamMateCar());
	//fprintf(stderr, "%s: team=%s teamcar=%p (%p)\n", car->_name, car->_teamname, opponents->getTeamMateCar(), car); fflush(stderr);

    // create the pit object.
    pit = new Pit(s, this, PitOffset);

    if (modeVerbose)
        printf("%s enter in correcting mode\n", car->_name);
    carindex = 0;

    for (int i = 0; i < s->_ncars; i++)
    {
        if (s->cars[i] == car)
        {
            carindex = i;
            break;
        }
    }

    /* K1999 raceline */
    raceline->setCar(car, mycardata);
    raceline->NewRace(car, s);
    raceline->InitTrack(track, s);

    line = new Line();
    line->setCar(car);
    line->InitTrack(track, s);

    // setup the line mode class
    linemode = new LLineMode(car, overrideCollection, track->width, raceline);
    linemode->UpdateSituation(s, false);

	//if (HMDIST > 0 && opponents->isBehindFriend(car->_pos))
	//	mde = (avoidright | avoidleft);
    lastmode = LINE_NONE;

    // Initialize radius of segments.
    radius = new float[track->nseg];
    //computeRadius( ref_line, radius);
    computeRadius(2, radius);
}

/***************************************************************************/

double Driver::getBrakeMargin()
{
    double brk = (double)brakemargin;
    if (overrideCollection)
    {
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKE_MARGIN);
        if (labelOverride)
        {
            if (!labelOverride->getOverrideValue(raceline->Next, &brk))
                brk = (double)brakemargin;
        }
    }
    return brk;
}

double Driver::getBrakeCoefficient()
{
    double brk = brake_coefficient;
    if (overrideCollection)
    {
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKE_COEFFICIENT);
        if (labelOverride)
        {
            if (!labelOverride->getOverrideValue(raceline->Next, &brk))
                brk = brake_coefficient;
        }
    }
    return brk;
}

bool Driver::calcSpeed()
{
    accelcmd = brakecmd = 0.0f;
    double speed = racelineDrivedata->speed;
    currentCollision = potentialOvertake = false;

    if (pit->getPitstop())
        speed = MIN(speed, pit->maxSpeed(car->_distFromStartLine));

    for (int i = 0; i < opponents->getNOpponents(); i++)
        if (opponent[i].getState() & OPP_COLL)
        {
#ifdef OVERTAKE_DEBUG	
			if (speed > opponent[i].getCollSpeed())
			{
				fprintf(stderr, "%s %s: COLLISION - Reducing speed from %.2f to %.2f\n", car->_name, opponent[i].getCarPtr()->_name, speed, opponent[i].getCollSpeed()); fflush(stderr);
			}
#endif
            speed = MIN(speed, opponent[i].getCollSpeed());
            currentCollision = true;
        }

    double x = (speedAdvance + car->_speed_x) * (speed - car->_speed_x) / speedDivisor;

    if (x > 0 && (!currentCollision || speed > car->_speed_x))
    {
        accelcmd = MAX(COAST_ACCEL, (float)x);
        if (car->_speed_x < 10.0 && car->_gear == 1)
            accelcmd = MAX(accelcmd, 0.9f);
    }
    else
    {
        if (currentCollision && speed < 5.0)
            brakecmd = 1.0f;
		else
		{
			double factor = 30.0;
			if (mycardata->aTT < mycardata->lTT)
				factor *= 1.0 + (1.0 - mycardata->aTT / mycardata->lTT) / 2;
			brakecmd = MIN(1.0f, factor * fabs(x));
		}
    }

    return currentCollision;
}

/***************************************************************************/

// Drive during race.
void Driver::drive(tSituation *s)
{
    situation = s;
    memset(&car->ctrl, 0, sizeof(tCarCtrl));
    linemode->UpdateSituation(s, underThreat);
	if (lastmode == LINE_NONE)
	{
		SetMode(LINE_MID, car->_trkPos.toLeft / track->width, car->_trkPos.toLeft / track->width);
		linemode->SetLeftCurrentMargin(car->_trkPos.toLeft / track->width);
		linemode->SetRightCurrentMargin(car->_trkPos.toLeft / track->width);
		linemode->SetTargetMargins(LINE_MID, car->_trkPos.toLeft / track->width, car->_trkPos.toLeft / track->width, true);
	}

    static double line_timer = s->currentTime;
    
    if (s->raceInfo.type == RM_TYPE_PRACTICE && test_raceline)
    {
        if (s->currentTime - line_timer > 8.0)
        {
            if (test_raceline == TR_LFT)
                SetMode(avoidright, 0.0, 0.0);
            else if (test_raceline == TR_RGT)
                SetMode(avoidleft, 1.0, 1.0);
            else
            {
#if 1
                static char *lineName[] = { "LINE_MID", "LINE_LEFT", "LINE_RIGHT", "LINE_RL", "LINE_RL", "LINE_RL" };
				int rnd = rand();
				int target_line = rnd % 4;
				if (target_line == LINE_MID)
				{
					rnd = rand();
					target_line = rnd % 4;
				}
                if (target_line != test_rnd_raceline)
                {
					fprintf(stderr, "%d/%d ", rnd, target_line);
                    line_timer = s->currentTime;
                    if (target_line == LINE_RL)
                    {
                        //linemode->setRecoveryToRaceLine();
                        SetMode(correcting, 0.0, 1.0);
                        fprintf(stderr, " Switching to %s from %s\n", lineName[target_line], lineName[test_rnd_raceline]);fflush(stderr);
                    }
                    else if (target_line == LINE_LEFT)
                    {
                        //linemode->setAvoidanceToLeft();
						double rightMargin = 0.9 - (double)(rand() % 75) / 100.0;
                        SetMode(avoidright, 0.0, rightMargin);
                        fprintf(stderr, "Switching to %s from %s (rgt %.3f)\n", lineName[target_line], lineName[test_rnd_raceline], rightMargin);fflush(stderr);
                    }
                    else if (target_line == LINE_RIGHT)
                    {
                        //linemode->setAvoidanceToRight();
						double leftMargin = 0.1 + (double)(rand() % 75) / 100.0;
                        SetMode(avoidleft, leftMargin, 1.0);
                        fprintf(stderr, "Switching to %s from %s (lft %.3f)\n", lineName[target_line], lineName[test_rnd_raceline], leftMargin);fflush(stderr);
                    }
                    else if (target_line == LINE_MID)
                    {
                        //linemode->setAvoidanceToRight();
						double leftMargin = 0.1 + (double)(rand() % 55) / 100.0;
						double rightMargin = 0.9 - (double)(rand() % 55) / 100.0;
						if (leftMargin > rightMargin)
							leftMargin = rightMargin;
                        SetMode(avoidleft|avoidright, leftMargin, rightMargin);
                        fprintf(stderr, "Switching to %s from %s (%.3f %.3f)\n", lineName[target_line], lineName[test_rnd_raceline], leftMargin, rightMargin);fflush(stderr);
                    }
					test_rnd_raceline = target_line;
                }
#endif
            }
        }

#if 0
        if (linemode->IsTransitioning())
        {
			if (linemode->GetLeftTargetMargin() > 0.00001 || linemode->GetRightTargetMargin() < 1.0)
                SetMode(avoiding);
        }
#endif
    }

    update(s);

    //pit->setPitstop(true);

    //if ( ! isStuck() )
    //stucksteer = -20.0;

	if (spinDist > 0 && car->_distRaced > spinDist)
	{
		while (mycardata->rmTT < 60.0) {
			car->_accelCmd = (car->_speed_x > 5.0 ? 0.0 : 0.8);
			car->_brakeCmd = (car->_speed_x > 5.0 ? 1.0 : 0.0);
			car->_steerCmd = (spinDir > 0 ? 1 : -1);
			car->_gearCmd = 1;
			return;
		}
	}
    if (isStuck()) {

        //if ( stucksteer < -19.0 )
        if (car->_speed_x <= 0.0)
        {
            stucksteer = -mycardata->getCarAngle() / car->_steerLock;
            car->_accelCmd = MAX(0.3f, (car->_speed_x > -5.0f ? 0.6f : 0.6f - (fabs(car->_speed_x)-7.0)/10));
            laststeer = -stucksteer;
        }
        else
        {
            stucksteer = laststeer = getSteer(s);
            if (car->_speed_x < 1.0f)
                car->_accelCmd = 0.4f;
            else if (car->_speed_x < 10.0f)
                car->_accelCmd = MAX(0.2f, 0.4f - fabs(stucksteer)/3);
        }

        if (stucksteer < 0.0)
            stucksteer = MIN(-0.5f, stucksteer);
        else
            stucksteer = MAX(0.5f, stucksteer);

        car->_steerCmd = stucksteer;
//fprintf(stderr, "stuck steer = %.3f\n", car->_steerCmd);
        car->_gearCmd = -1;    // Reverse gear.
        car->_brakeCmd = 0.0f;    // No brakes.
        car->_clutchCmd = 0.0f;    // Full clutch (gearbox connected with engine).
    }
    else 
    {
        laststeer = car->_steerCmd = filterTrk(getSteer(s));

        car->_gearCmd = getGear();
        bool collision_brake = calcSpeed();

        if (!collision_brake && car->_gearCmd == 1 && car->_speed_x < 5.0)
        {
            brakecmd = 0.0;
            accelcmd = 1.0;
            if (car->_speed_x > 1.0 && simtime > 10.0f)
                accelcmd -= MIN(0.5, fabs(car->_steerCmd)/2);
        }

        if (car->_distRaced < WB4DIST && MIN(car->_trkPos.toLeft, car->_trkPos.toRight) > 1.0 && fabs(car->_steerCmd) < 0.7f)
        {
            float steerDelta = MAX(40.0f, 70.0f - car->_speed_x) / 130;
			int count = MAX(0, WB4COUNT - (int)(raceline->tRInverse[LINE_RL][raceline->Next] * 100));
            if (accelcmd > 0.5 && (fabs(car->_steerCmd) < 0.1))
                accelcmd = 1.0;
            if (m_lastWSide > 0)
                car->_steerCmd = laststeer = car->_steerCmd - steerDelta;
            else
                car->_steerCmd = laststeer = car->_steerCmd + steerDelta;
            m_lastWCount++;
			if (m_lastWCount > count)
            {
                m_lastWSide -= m_lastWSide*2;
                m_lastWCount = 0;
            }
        }

        car->_brakeCmd = filterABS(filterBrakeSpeed(filterBColl(filterBPit(getBrake()))));
        if (simtime < 0.5f || car->_brakeCmd <= 0.0001f) {
            car->_accelCmd = filterOverlap(filterTCL(getAccel(), s->_raceType));
            if (car->_speed_x < 7.0f && simtime > 3.0f)
			{
				double minAccel = MAX(0.1, 0.5 - fabs(car->_yaw_rate) / 2);
				car->_accelCmd = MAX(minAccel, car->_accelCmd - fabs(car->_steerCmd) / 2);
			}
            car->_brakeCmd = 0.0f;
        }
        else {
            car->_accelCmd = 0.0f;
        }
        car->_clutchCmd = getClutch();

    }

    lastmode = mode;
}

/***************************************************************************/

// Set pitstop commands.
int Driver::pitCommand(tSituation *s)
{
    if (strategy->pitStopPenalty() == RM_PENALTY_STOPANDGO)
    {
        car->pitcmd.stopType = RM_PIT_STOPANDGO;
    }
    else
    {
        car->_pitRepair = strategy->pitRepair(car, s);
        car->_pitFuel = strategy->pitRefuel(car, s);
        car->pitcmd.tireChange = strategy->pitTyres(car, s);
    }

	// force an update
	raceline->lastUpdateDist = -1.0;

    // This should be the only place where the pit stop is set to false!
    pit->setPitstop(false);

    stuck_reverse_timer = stuck_stopped_timer = -1.0f;

    return ROB_PIT_IM; // return immediately.
}

/***************************************************************************/

// End of the current race.
void Driver::endRace(tSituation *s)
{
    // Nothing for now.
}


/***************************************************************************
 *
 *             utility functions
 *
 ***************************************************************************/


/*============== Print Parameters at Setup ===================*/
void Driver::showSetup()
{
    fprintf(stderr, "######### %s #########\n", car->_name);
    fprintf(stderr, "# %s: Mode verbose= %d\n", car->_name, modeVerbose);
    fprintf(stderr, "# %s: Strategy verbose= %d\n", car->_name, m_strategyverbose);
    fprintf(stderr, "# %s: Steering verbose= %d\n", car->_name, m_steerverbose);
    fprintf(stderr, "# %s: Check QualifTime= %d\n", car->_name, m_testQualifTime);
    fprintf(stderr, "# %s: Check Pitstop= %d\n", car->_name, m_testPitstop);
    fprintf(stderr, "# \n");
    fprintf(stderr, "# %s: fuelPerMeter= %.5f\n", car->_name, m_fuelPerMeter);
    fprintf(stderr, "# %s: PitDamage= %d\n", car->_name, m_maxDammage);
    fprintf(stderr, "# %s: Fuel strategy= %d\n", car->_name, m_fuelStrat);
    fprintf(stderr, "# \n");
    fprintf(stderr, "# %s: Brake delay= %.1f\n", car->_name, brakedelay);
    fprintf(stderr, "# %s: Corner speed= %.1f\n", car->_name, CornerSpeed);
    fprintf(stderr, "# %s: Pit Offset= %.1f\n", car->_name, PitOffset);
    fprintf(stderr, "# %s: Let Pass= %.2f\n", car->_name, LetPass);
}
/*========================================================*/

/* Compute Radius */
void Driver::computeRadius(int ref_line, float *radius)
{
    tTrackSeg *currentseg, *startseg = track->seg;
    currentseg = startseg;

    do {
        if (currentseg->type == TR_STR)
        {
            radius[currentseg->id] = FLT_MAX;
        }
        else  /* turn */
        {
            radius[currentseg->id] = currentseg->radius + currentseg->width / 2.0;
        }
        currentseg = currentseg->next;

    } while (currentseg != startseg);


    /* override old radii with new */
    /* this needs major cleanup */

    currentseg = startseg;

    do
    {
        //radius[currentseg->id] = line.lineRadius(ref_line, currentseg->lgfromstart);
        radius[currentseg->id] = line->lineRadius(2, currentseg->lgfromstart);

        if (radius[currentseg->id] < 0)
            radius[currentseg->id] = FLT_MAX;

        currentseg = currentseg->next;
    } while (currentseg != startseg);


}


/******************************************************************************/

// Compute the length to the end of the segment.
float Driver::getDistToSegEnd()
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    }
    else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
    }
}


/******************************************************************************/

// Compute fitting acceleration.
float Driver::getAccel()
{
    if (car->_gear > 0) {
        if (simtime < 2.0f || car->_speed_x < 5.0f)
           accelcmd = 1.0f;
        else
        {
            accelcmd = MIN(1.0f, accelcmd);
            if (fabs(angle) > 0.8 && getSpeed() > 10.0f)
                accelcmd = MAX(0.0f, MIN(accelcmd, 1.0f - getSpeed() / 100.0f * fabs(angle)));
            else if (car->_gear <= 2 && (car->_trkPos.toLeft < -1.0 || car->_trkPos.toRight < -1.0))
            {
                if (car->_speed_x < 4.0)
                    accelcmd = MAX(accelcmd, 0.9f);
                else 
                    accelcmd = MIN(accelcmd, MIN(car->_wheelSeg(REAR_RGT)->surface->kFriction, car->_wheelSeg(REAR_LFT)->surface->kFriction));
            }
            else if (car->_speed_x < 10.0 && car->_gear == 1)
                accelcmd = MAX(accelcmd, 0.9f);
        }
        return accelcmd;
    }
    else {
        return 1.0;
    }
}


/******************************************************************************/

// If we get lapped reduce accelerator.
float Driver::filterOverlap(float accel)
{
    int i;
    for (i = 0; i < opponents->getNOpponents(); i++) 
    {
        if ((opponent[i].getState() & OPP_LETPASS) && fabs(car->_trkPos.toMiddle - opponent[i].getCarPtr()->_trkPos.toMiddle) > 3.0)
        {
//fprintf(stderr, "%s LETPASS!!!\n",opponent[i].getCarPtr()->_name);fflush(stderr);

            //return MIN(accel, 0.5f);
            return MIN(accel, LetPass);
        }
    }
    return accel;
}

/******************************************************************************/

// Compute initial brake value.
float Driver::getBrake()
{
//#define BRAKETEST
#ifdef BRAKETEST
static bool brakenow = false;
static double brakedist = 0.0;
if (car->_speed_x > 86)
{
    brakenow = true;
    brakedist = car->_trkPos.seg->lgfromstart + car->_trkPos.toStart;
}
if (brakenow)
{
    if (car->_speed_x > 0.001) { fprintf(stderr, "%.1f %.1f\n", car->_speed_x, (car->_trkPos.seg->lgfromstart + car->_trkPos.toStart) - brakedist);fflush(stderr);}
    return 1.0;
}
#endif
    // Car drives backward?
    if (car->_speed_x < -MAX_UNSTUCK_SPEED) {
        // Yes, brake.
        return 1.0;
    }
    else {
        // We drive forward, normal braking.
        return brakecmd;
    }
}


/******************************************************************************/

// Compute gear.
int Driver::getGear()
{
    if (car->_gear <= 0) {
        return 1;
    }
#define BT_GEARS 
//#define K1999_GEARS 
//#define MOUSE_GEARS 

#ifdef BT_GEARS
    /*BT gear changing */
    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float up_omega = car->_enginerpmRedLine / gr_up;
    float wr = car->_wheelRadius(2);
    float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
	float shift_margin = SHIFT_MARGIN*1.5 / MAX(1, car->_gear - 1);
    float dn_omega = car->_enginerpmRedLine / gr_down;

	bool change_down = (car->_gear > 1 && dn_omega*wr*gear_shift > car->_speed_x + shift_margin);
	bool change_up = ((car->_gear > 1 && up_omega*wr*gear_shift < car->_speed_x) ||
		(car->_gear < 3 && car->_enginerpm / car->_enginerpmRedLine > SHIFT_UP && car->_gear < car->_gearNb - 2));

	if (change_up && !change_down)
    {
        return car->_gear + 1;
    }
	else if (change_down)
    {
        return car->_gear - 1;
    }
#elif defined(MOUSE_GEARS)
    const int        MAX_GEAR = car->_gearNb - 1;

    double        gr_dn = car->_gear > 1 ?
                                car->_gearRatio[car->_gear + car->_gearOffset - 1] :
                                1e5;
    double        gr_this = car->_gearRatio[car->_gear + car->_gearOffset];

    double        wr = (car->_wheelRadius(2) + car->_wheelRadius(3)) / 2;
    double        rpm = gr_this * car->_speed_x / wr;
//    double        rpm = car->_enginerpm;

    double        rpmUp = 830.0;
    double        rpmDn = rpmUp * gr_this * 0.9 / gr_dn;

//  GfOut( "gear %d    rpm %6.1f %6.1f    dist %6.1f  up dist %6.1f   down dist %6.1f\n",
//          car->_gear, rpm, car->_enginerpm, grDist, upDist, dnDist );

    if( car->_gear < MAX_GEAR && rpm > rpmUp )
    {
        car->ctrl.clutchCmd = 1.0;
//      acc = 0.5;
        return car->_gear + 1;
    }
    else if( car->_gear > 1 && rpm < rpmDn )
    {
//      car->ctrl.clutchCmd = 1.0;
//      acc = 1.0;
        return car->_gear - 1;
    }
#elif defined(K1999_GEARS)
    float *tRatio = car->_gearRatio + car->_gearOffset;
    double rpm = (car->_speed_x + SHIFT_MARGIN/2) * tRatio[car->_gear] / car->_wheelRadius(2);

    if (rpm > car->_enginerpmRedLine * (1.0 - ((double)car->_gear/300)))
        return car->_gear + 1;

    if (car->_gear > 1 && rpm / tRatio[car->_gear] * tRatio[car->_gear-1] < car->_enginerpmRedLine * (0.92 + ((double)car->_gear/200.0)))
        return car->_gear - 1;

#endif

    return car->_gear;
}

/******************************************************************************/
void Driver::SetMode(int newmode, double leftMargin, double rightMargin, bool force)
{
    if (newmode == pitting)
    {
		//fprintf(stderr, "Set to Pitting\n");
        linemode->SetPitting();
		return;
    }

    //if (mode == newmode && !force)
    //    return;

    if (mode == normal || mode == pitting)
    {
        correcttimer = simtime + 7.0;
        correctlimit = 1000.0;
		double pos = car->_trkPos.toLeft / track->width;
		if (pit->getInPit() && !(pit->getInPitExit() || fabs(car->_trkPos.toMiddle) > track->width/2 - 1.0))
		{
			if (pos < 0.0)
			{
				linemode->SetLeftCurrentMargin(pos);
				linemode->SetRightCurrentMargin(1.0);
			}
			else
			{
				linemode->SetRightCurrentMargin(pos);
				linemode->SetLeftCurrentMargin(0.0);
			}
		}
		//if (mode == normal && !pit->getInPit())
		{
			//fprintf(stderr, "Set to Normal\n");
			linemode->SetTargetMargins(LINE_RL, 0.0, 1.0, force);
		}
#if 0
		else
		{
			//fprintf(stderr, "Set to Pitting\n");
			linemode->SetTargetMargins(LINE_MID, pos, pos, force);
		}
#endif
    }

	if (newmode == correcting)
	{
		//fprintf(stderr, "Set to Correcting\n");
		linemode->SetRecoverToRaceLine();
	}
	else if ((newmode & avoidleft) || (newmode & avoidright))
	{
		int rl = LINE_MID;
		if (leftMargin == 0.0)
			rl = LINE_RIGHT;
		else
			rl = LINE_LEFT;
		//fprintf(stderr, "Set to %s\n", (rl == LINE_MID ? "MID" : (rl == LINE_LEFT ? "LFT" : "RGT")));
		linemode->SetTargetMargins(rl, leftMargin, rightMargin, force);
	}

    mode = newmode;
}

/******************************************************************************/
// Compute steer value.
float Driver::getSteer(tSituation *s)
{
    if (simtime <= 0.0)
        return 0.0f;

    float targetAngle;
    float offset = (s->currentTime < 1.0 ? car->_trkPos.toMiddle : getOffset());


    /* K1999 raceline */
    memset(racelineDrivedata, 0, sizeof(RaceLineDriveData));
    racelineDrivedata->s = s;
    racelineDrivedata->target = &racetarget;
    racelineDrivedata->linemode = linemode;
    racelineDrivedata->laststeer = laststeer;
    racelineDrivedata->angle = angle;
    racelineDrivedata->avoidmode = avoidmode;
    racelineDrivedata->speedangle = (car->_speed_x < 5.0 ? 0.0 : speedangle);
    for (int i = 0; i < opponents->getNOpponents(); i++)
        if ((opponent[i].getState() & OPP_COLL) && opponent[i].getCollSpeed() <= car->_speed_x && opponent[i].getDistance() < 2.0)
        {
            racelineDrivedata->coll = true;
            break;
        }
    raceline->GetRaceLineData(racelineDrivedata, (mode == normal ? false : true));

    vec2f    target = getTargetPoint(racelineDrivedata->target_lane);
    float avoidsteer = 0.0f;
    float steer = 0.0f;

    if (mode == pitting || simtime < 1.0f)
    {
        targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
        avoidsteer = calcSteer(targetAngle, 0, racelineDrivedata->racesteer);
        if (mode == pitting)
            linemode->SetPitting();
        return avoidsteer;
    }
    if (car->_speed_x < 0.0)
    {
        return -mycardata->getCarAngle() / car->_steerLock;
    }

    /* K1999 steering */
    targetAngle = atan2(racetarget.y - car->_pos_Y, racetarget.x - car->_pos_X);

    /* BT steering,  uncomment the following to use BT steering rather than K1999*/
    // racelineDrivedata->racesteer = calcSteer( targetAngle, 1 );

    NORM_PI_PI(targetAngle);

    /*
    if ((mode & avoiding) &&
        (!avoidmode ||
        (avoidmode == avoidright && racelineDrivedata->raceoffset >= myoffset && racelineDrivedata->raceoffset < avoidlftoffset) ||
        (avoidmode == avoidleft && racelineDrivedata->raceoffset <= myoffset && racelineDrivedata->raceoffset > avoidrgtoffset)))
    {
        // we're avoiding, but trying to steer somewhere the raceline takes us.
        // hence we'll just correct towards the raceline instead.
        setMode(correcting);
        if (modeVerbose)
            fprintf(stderr, "%s enter in correcting mode\n", car->_name);
    }
    */


    /* K1999 raceline */
    int line;
    line = LINE_RL;

    if (mode == correcting &&
        (lastmode == normal ||
        (fabs(angle) < 0.2f &&
        fabs(racelineDrivedata->racesteer) < 0.4f &&
        fabs(laststeer - racelineDrivedata->racesteer) < 0.05 &&
        ((fabs(car->_trkPos.toMiddle) < car->_trkPos.seg->width / 2 - 1.0) || car->_speed_x < 10.0) &&
        (raceline->isOnLine(line)))))
    {
        // we're correcting & are now close enough to the raceline to
        // switch back to 'normal' mode...
        SetMode(normal, 0.0, 1.0);
        if (modeVerbose)
            printf("%s enter in Normal mode\n", car->_name);
    }

	if (pit->getInPit() && (!pit->getInPitExit() || fabs(car->_trkPos.toMiddle) > track->width/2 - 0.5))
    {
        float targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
        targetAngle -= car->_yaw;
        NORM_PI_PI(targetAngle);
        steer = targetAngle / car->_steerLock;

#if 1
        double spd0 = hypot(car->_speed_x, car->_speed_y);
        double vx = car->_speed_X;
        double vy = car->_speed_Y;
        double dirx = cos(car->_yaw);
        double diry = sin(car->_yaw);
        double Skid = (dirx * vy - vx * diry) / (spd0 == 0.0 ? 0.1 : spd0);
        Skid = MIN(0.9, MAX(-0.9, Skid));
        steer += (asin(Skid) / car->_steerLock) * 0.06;
#endif
		pitTimer = simtime;
		linemode->SetLeftCurrentMargin(car->_trkPos.toLeft / track->width);
		linemode->SetRightCurrentMargin(car->_trkPos.toLeft / track->width);
    }
	else
    {
        steer = racelineDrivedata->racesteer;
        lastNSasteer = racelineDrivedata->racesteer*0.8;
    }

//fprintf(stderr,"getSteer2 %.3f\n",steer);
    return steer;
}

/******************************************************************************/
float Driver::calcSteer(float targetAngle, int rl, float racesteer)
{
    double steer_direction = targetAngle - car->_yaw;

    NORM_PI_PI(steer_direction);

    float steer = (float)(steer_direction / car->_steerLock);

    // smooth steering.  I know there's a separate function for this, but what the hey!
    if (mode != pitting)
    {
        //double minspeedfactor = (((60.0 - (MAX(40.0, MIN(70.0, getSpeed() + MAX(0.0, car->_accel_x*5))) - 25)) / 300) * MAX(10.0, 30.0+car->_accel_x));
        double minspeedfactor = (((80.0 - (MAX(40.0, MIN(70.0, getSpeed() + MAX(0.0, car->_accel_x * 5))) - 25)) / 300) * (5.0 + MAX(0.0, (CA - 1.9) * 20)));

        double maxspeedfactor = minspeedfactor;
        double rInverse = raceline->getRInverse();

        if (rInverse > 0.0)
        {
            //minspeedfactor = MAX(minspeedfactor/3, minspeedfactor - rInverse*getSpeed() + MAX(0.0f, angle/100));
            minspeedfactor = MAX(minspeedfactor / 3, minspeedfactor - rInverse*80.0);//getSpeed() + MAX(0.0f, angle/100));
            maxspeedfactor = MAX(maxspeedfactor / 3, maxspeedfactor + rInverse*20.0);//getSpeed() + MIN(0.0f, angle/100));
        }
        else
        {
            //maxspeedfactor = MAX(maxspeedfactor/3, maxspeedfactor + rInverse*getSpeed() + MIN(0.0f, angle/100));
            maxspeedfactor = MAX(maxspeedfactor / 3, maxspeedfactor + rInverse*80.0);//getSpeed() + MIN(0.0f, angle/100));
            minspeedfactor = MAX(minspeedfactor / 3, minspeedfactor + rInverse*20.0);//getSpeed() + MAX(0.0f, angle/100));
        }

        steer = (float)MAX(lastNSasteer - minspeedfactor, MIN(lastNSasteer + maxspeedfactor, steer));
    }

    lastNSasteer = steer;

    if (fabs(angle) > fabs(speedangle))
    {
        // steer into the skid
        double sa = MAX(-0.3, MIN(0.3, speedangle / 3));
        //double anglediff = (sa - angle) * 0.7;
        double anglediff = (sa - angle) * (0.7 - MAX(0.0, MIN(0.3, car->_accel_x / 100)));
        //anglediff += raceline->getRInverse() * 10;
        steer += (float)(anglediff*0.7);
    }

    if (fabs(angle) > 1.2)
    {
        if (steer > 0.0f)
            steer = 1.0f;
        else
            steer = -1.0f;
    }
    else if (fabs(car->_trkPos.toMiddle) - car->_trkPos.seg->width / 2 > 2.0)
    {
        steer = (float)MIN(1.0f, MAX(-1.0f, steer * (1.0f + (fabs(car->_trkPos.toMiddle) - car->_trkPos.seg->width / 2) / 14 + fabs(angle) / 2)));
    }


    if (mode != pitting && simtime > 1.5f)
    {
        // limit how far we can steer against raceline 
        double limit = (90.0 - MAX(40.0, MIN(60.0, car->_speed_x))) / (50 + fabs(angle)*fabs(angle) * 3);
        steer = MAX(racesteer - limit, MIN(racesteer + limit, steer));
    }

    return steer;
}

/******************************************************************************/
float Driver::correctSteering(float avoidsteer, float racesteer)
{
    /* K1999 raceline */
    int line;
    line = LINE_RL;

    float steer = avoidsteer;
    //float accel = MIN(0.0f, car->_accel_x); // Not used

    double speed = MAX(50.0, getSpeed());
    //double changelimit = MIN(1.0, raceline->correctLimit(line));
    double changelimit = MIN(raceline->correctLimit(line), (((120.0 - getSpeed()) / 6000) * (0.5 + MIN(fabs(avoidsteer), fabs(racesteer)) / 10))) * 0.50;

    if (mode == correcting && simtime > 2.0f)
    {
        // move steering towards racesteer...
        if (correctlimit < 900.0)
        {
            if (steer < racesteer)
            {
                if (correctlimit >= 0.0)
                {
                    //steer = (float) MIN(racesteer, steer + correctlimit);
                    steer = racesteer;
                }
                else
                {
                    steer = (float)MIN(racesteer, MAX(steer, racesteer + correctlimit));
                }
            }
            else
            {
                if (correctlimit <= 0.0)
                {
                    //steer = (float) MAX(racesteer, steer-correctlimit);
                    steer = racesteer;
                }
                else
                {
                    steer = (float)MAX(racesteer, MIN(steer, racesteer + correctlimit));
                }
            }
        }

        speed -= car->_accel_x / 10;
        speed = MAX(55.0, MIN(150.0, speed + (speed*speed / 55)));
        double rInverse = raceline->getRInverse() * (car->_accel_x<0.0 ? 1.0 + fabs(car->_accel_x) / 10.0 : 1.0);
        double correctspeed = 0.5;
        if ((rInverse > 0.0 && racesteer > steer) || (rInverse < 0.0 && racesteer < steer))
            correctspeed += rInverse * 110;

        if (racesteer > steer)
            steer = (float)MIN(racesteer, steer + changelimit);
        else
            steer = (float)MAX(racesteer, steer - changelimit);

        //correctlimit = (steer - racesteer) * 1.08;
        correctlimit = (steer - racesteer);
    }

    return steer;
}

/******************************************************************************/
float Driver::smoothSteering(float steercmd)
{
    /* try to limit sudden changes in steering to avoid loss of control through oversteer. */
    double speedfactor = (((60.0 - (MAX(40.0, MIN(70.0, getSpeed() + MAX(0.0, car->_accel_x * 5))) - 25)) / 300) * 1.2) / 0.785 * 0.75;
    //double speedfactor = (((60.0 - (MAX(40.0, MIN(70.0, getSpeed() + MAX(0.0, car->_accel_x * 5))) - 25)) / 300) * 1.2) / 0.785;

    if (fabs(steercmd) < fabs(laststeer) && fabs(steercmd) <= fabs(laststeer - steercmd))
        speedfactor *= 2;

    steercmd = (float)MAX(laststeer - speedfactor, MIN(laststeer + speedfactor, steercmd));
    return steercmd;
}

/******************************************************************************/
// Compute the clutch value.
float Driver::getClutch()
{
//#define MOUSE_CLUTCH
#ifndef MOUSE_CLUTCH
    float maxtime = (car->_gearCmd == 1 ? 0.36f : MAX(0.06f, 0.32f - ((float)car->_gearCmd / 65.0f)));
    if (car->_gear != car->_gearCmd)
        clutchtime = maxtime;
	float delta = (car->_gearCmd == 1 && simtime < 5.0f ? 0.005f : 0.03f);
	if (car->_enginerpm / car->_enginerpmRedLine > 0.7)
        clutchtime = MAX(0.0f, clutchtime - (float)(RCM_MAX_DT_ROBOTS * (delta + ((float)car->_gearCmd / 5.0f))));
	else //if (car->_gearCmd <= 2)
		clutchtime = MIN(maxtime, clutchtime + (float)(RCM_MAX_DT_ROBOTS * (0.03f + ((float)car->_gearCmd / 5.0f))));
    return 2.0f * clutchtime;
#else
    // Mouse clutch
    if( car->ctrl.clutchCmd > 0 )
    {
/*        car->ctrl.clutchCmd -= 0.085f;
        if( car->ctrl.clutchCmd < 0 )
            car->ctrl.clutchCmd = 0;
*/
        double    wr = 0;
        int        count = 0;

        {
            wr += car->_wheelRadius(REAR_LFT) + car->_wheelRadius(REAR_RGT);
            count += 2;
        }
        wr /= count;
        double    gr = car->_gearRatio[car->_gear + car->_gearOffset];
        double    rpmForSpd = gr * car->_speed_x / wr;
        double    rpm = car->_enginerpm;
//        GfOut( "RPM %3.0f  RPM2 %3.0f  %g\n", rpm, rpmForSpd, rpmForSpd / rpm );

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
    }

    if (car->_gearCmd == 1 && car->_speed_x >= -0.01 && car->_speed_x < 10 && car->_accelCmd > 0.1 && car->_brakeCmd == 0)
    {
        double    rpm = car->_enginerpm;
        double    clutch = (850 - rpm) / 400;
        if( car->_speed_x < 0.05 )
            clutch = 0.5;

        car->ctrl.clutchCmd = MAX(0, MIN(clutch, 0.9));
    }
    if (car->_gear < car->_gearNb - 1 && car->_gearCmd > car->_gear)
        car->ctrl.clutchCmd = 1.0;
    return car->ctrl.clutchCmd;
#endif

}

/****************************************/

// Compute target point for steering.
vec2f Driver::getTargetPoint(double lane)
{
    tTrackSeg *seg = car->_trkPos.seg;
    float lookahead;
    float length = getDistToSegEnd();
    float offset = car->_trkPos.toMiddle;
    float pitoffset = -100.0f;

    if (pit->getInPit()) {
        // To stop in the pit we need special lookahead values.
        if (currentspeedsqr > pit->getSpeedlimitSqr()) {
            lookahead = PIT_LOOKAHEAD + car->_speed_x*LOOKAHEAD_FACTOR;
        }
        else {
            lookahead = PIT_LOOKAHEAD;
        }
    }
    else {
        // Usual lookahead.
#if 0
        lookahead = racelineDrivedata->lookahead;
#if 0
        lookahead = LOOKAHEAD_CONST + car->_speed_x*LOOKAHEAD_FACTOR;
        lookahead = MAX(lookahead, LOOKAHEAD_CONST + ((car->_speed_x*(car->_speed_x/2)) / 60.0));
#endif
#if 1
        double speed = MAX(20.0, getSpeed());// + MAX(0.0, car->_accel_x));
        lookahead = (float)(LOOKAHEAD_CONST * 1.2 + speed * 0.60);
        lookahead = MIN(lookahead, (float)(LOOKAHEAD_CONST + ((speed*MAX(1, speed / 7)) * 0.15)));
#endif

        // Prevent "snap back" of lookahead on harsh braking.
        float cmplookahead = oldlookahead - car->_speed_x*RCM_MAX_DT_ROBOTS;
        if (lookahead < cmplookahead) {
            lookahead = cmplookahead;
        }
#else 
        // original BT method
        //lookahead = LOOKAHEAD_CONST + car->_speed_x/6;//*(LOOKAHEAD_FACTOR/30);
        //lookahead = LOOKAHEAD_CONST + car->_speed_x/6 + MAX(0.0, car->_speed_x-70) * MAX(0.0, fabs(raceline->tRInverse[LINE_RL][raceline->Next])-0.003)*1000;
        lookahead = MAX(20, racelineDrivedata->lookahead*0.6) + MAX(0.0, car->_speed_x-40)/6;
        // Prevent "snap back" of lookahead on harsh braking.
        //float cmplookahead = oldlookahead - car->_speed_x*RCM_MAX_DT_ROBOTS;
        if (lookahead < oldlookahead) {
            //lookahead = cmplookahead;
            lookahead = (lookahead * 0.8 + oldlookahead * 0.2);
        }
#endif
    }

    oldlookahead = lookahead;

    // Search for the segment containing the target point.
    while (length < lookahead) {
        seg = seg->next;
        length += seg->length;
    }
    // Length now > lookahead

    length = lookahead - length + seg->length;
    // length now distance past start of seg

    float fromstart = seg->lgfromstart;
    fromstart += length;

    // Compute the target point.
    if (strategy->pitStopPenalty() == RM_PENALTY_DRIVETHROUGH)
        pitoffset = pit->getDriveThroughOffset(pitoffset, fromstart);
    else
        pitoffset = pit->getPitOffset(pitoffset, fromstart);
    if ((pit->getPitstop() || pit->getInPit()) && pitoffset != -100.0f)
    {
        if (pit->getPitstop() || fabs(car->_trkPos.toMiddle) > track->width/2)
        {
            SetMode(pitting, 0, 0);
            offset = myoffset = pitoffset;
            if (modeVerbose)
                printf("%s enter in pitting mode\n", car->_name);
        }
        else
        {
            SetMode(correcting, 0.0, 1.0);
        }
    }
    else if (mode == pitting)
    {
        SetMode(correcting, 0.0, 1.0);
        if (modeVerbose)
            printf("%s enter in correcting mode\n", car->_name);
    }
    else if (linemode->IsTransitioning())
        offset = (track->width/2) - (track->width * lane);

    vec2f s;
#if 0
    if (mode != pitting && simtime > 1.5)
    {
        /* BT raceline */
        //s = line.getTargetPoint( 2, fromstart, offset);

        /* K1999 raceline */
        raceline->GetPoint(offset, lookahead, &s);
        return s;
    }
#endif

    s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x) / 2.0f;
    s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y) / 2.0f;

    if (seg->type == TR_STR) {
        vec2f d, n;
        n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x) / seg->length;
        n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y) / seg->length;
        n.normalize();
        d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x) / seg->length;
        d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y) / seg->length;
        return s + d*length + offset*n;
    }
    else {
        vec2f c, n, t, rt;
        c.x = seg->center.x;
        c.y = seg->center.y;
        float arc = length / seg->radius;
        float arcsign = (seg->type == TR_RGT) ? -1.0f : 1.0f;
        arc = arc*arcsign;
        s = s.rotate(c, arc);

        n = c - s;
        n.normalize();
        t = s + arcsign*offset*n;

#if 0
        if (mode != pitting)
        {
            /* bugfix - calculates target point a different way, thus
               bypassing an error in the BT code that sometimes made
               the target closer to the car than lookahead...*/
            /* BT raceline */
            //rt = line.getTargetPoint( 2, fromstart, offset);

            /* K1999 raceline */
            raceline->GetPoint(offset, lookahead, &rt);

            double dx = t.x - car->_pos_X;
            double dy = t.y - car->_pos_Y;
            double dist1 = sqrt(dx*dx + dy*dy);
            dx = rt.x - car->_pos_X;
            dy = rt.y - car->_pos_Y;
            double dist2 = sqrt(dx*dx + dy*dy);
            if (dist2 > dist1)
                t = rt;
        }
#endif

        return t;
    }
}


/******************************************************************************/

int Driver::checkSwitch( int side, Opponent *o, tCarElt *ocar, double catchdist)
{
    double radius = 0.0;

    if (catchdist < 15.0)
    {
        switch (side)
        {
            case TR_LFT:
                if (ocar->_trkPos.toRight < ocar->_dimension_y + 1.5 + car->_dimension_y)
                {
                    side = TR_RGT;
                }
                break;

            case TR_RGT:
            default:
                if (ocar->_trkPos.toLeft < ocar->_dimension_y + 1.5 + car->_dimension_y)
                {
                    side = TR_LFT;
                }
                break;
        }
    }

    return side;
}


// Compute offset to normal target point for overtaking or let pass an opponent.
float Driver::getOffset()
{
    int i;
    float mincatchdist = 1000.0f, mindist = -1000.0f;
    //Opponent *ret = NULL;
    Opponent *o = NULL;
    avoidmode = 0;
    //myoffset = car->_trkPos.toMiddle;

    myoffset = car->_trkPos.toMiddle;
    avoidlftoffset = MAX(myoffset, car->_trkPos.seg->width / 2 - 1.5);
    avoidrgtoffset = MIN(myoffset, -(car->_trkPos.seg->width / 2 - 1.5));

    /* Increment speed dependent. */
    //float incfactor = (MAX_INC_FACTOR - MIN(fabs(car->_speed_x)/MAX_INC_FACTOR, (MAX_INC_FACTOR - 1.0f))) * 4.0f;
    double rInverse = raceline->getRInverse();
    double incspeed = MIN(60.0, MAX(45.0, getSpeed())) - 20.0;
    double incfactor = (MAX_INC_FACTOR - MIN(fabs(incspeed) / MAX_INC_FACTOR, (MAX_INC_FACTOR - 1.0f))) * (12.0f + MAX(0.0, (CA - 1.9) * 14));
    double rgtinc = incfactor * MIN(1.3, MAX(0.4, 1.0 + rInverse * (rInverse < 0.0 ? 20 : 50)));
    double lftinc = incfactor * MIN(1.3, MAX(0.4, 1.0 - rInverse * (rInverse > 0.0 ? 20 : 50)));


    {
        int offlft = (myoffset > car->_trkPos.seg->width / 2 - 1.5);
        int offrgt = (myoffset < -(car->_trkPos.seg->width / 2 - 1.5));

        if (offlft)
            myoffset -= OVERTAKE_OFFSET_INC*rgtinc / 2;
        else if (offrgt)
            myoffset += OVERTAKE_OFFSET_INC*lftinc / 2;

        avoidlftoffset = MAX(avoidlftoffset, myoffset - OVERTAKE_OFFSET_INC*rgtinc*(offlft ? 6 : 2));
        avoidrgtoffset = MIN(avoidrgtoffset, myoffset + OVERTAKE_OFFSET_INC*lftinc*(offrgt ? 6 : 2));
    }

    double maxoffset = track->width / 2 - (car->_dimension_y); // limit to the left
    double minoffset = -(track->width / 2 - (car->_dimension_y)); // limit to the right

    if (myoffset < minoffset) // we're already outside right limit, bring us back towards track
    {
        minoffset = myoffset + OVERTAKE_OFFSET_INC*lftinc;
        maxoffset = MIN(maxoffset, myoffset + OVERTAKE_OFFSET_INC*lftinc * 2);
    }
    else if (myoffset > maxoffset) // outside left limit, bring us back
    {
        maxoffset = myoffset - OVERTAKE_OFFSET_INC*rgtinc;
        minoffset = MAX(minoffset, myoffset - OVERTAKE_OFFSET_INC*rgtinc * 2);
    }
    else
    {
        /* set tighter limits based on how far we're allowed to move */
        maxoffset = MIN(maxoffset, myoffset + OVERTAKE_OFFSET_INC*lftinc * 2);
        minoffset = MAX(minoffset, myoffset - OVERTAKE_OFFSET_INC*rgtinc * 2);
    }

    // Side Collision.
    avoidmode = 0;
	avoidCritical = false;
	avoidSqueezed = -1.0;
	double leftMargin = 0.0, rightMargin = 1.0;
    for (i = 0; i < opponents->getNOpponents(); i++)
    {
        tCarElt *ocar = opponent[i].getCarPtr();

        /* Ignore Pitting or Eliminated Cars */
        if (ocar->_state & RM_CAR_STATE_NO_SIMU)
            continue;

        /* Ignore offtrack cars */
        if ((fabs(ocar->_trkPos.toMiddle) > car->_trkPos.seg->width / 2 + 5.0) &&
            (fabs(ocar->_trkPos.toMiddle - car->_trkPos.toMiddle) >= 7.0))
            continue;

        if ((opponent[i].getState() & OPP_SIDE) || (opponent[i].getState() & OPP_LETPASS))
        {
#ifdef OVERTAKE_DEBUG
fprintf(stderr, "Side-avoiding %s\n",ocar->_name);fflush(stderr);
#endif
            if (modeVerbose)
                printf("%s enter in Avoiding mode\n", ocar->_name);
            o = &opponent[i];
            overtake_timer = 0.0;

			double oppCarTL = o->getCurrentLft();
			double oppCarTR = o->getCurrentRgt();

			oppCarTL -= (ocar->_laps > car->_laps ? 3 : 1);
			oppCarTR += (ocar->_laps > car->_laps ? 3 : 1);

            float sidedist = fabs(ocar->_trkPos.toLeft - car->_trkPos.toLeft);
            float sidemargin = opponent[i].getWidth() + getWidth() + 2.0f;
            float side = car->_trkPos.toMiddle - ocar->_trkPos.toMiddle;

            if ((side > 0.0 && rInverse < 0.0) ||
                (side < 0.0 && rInverse > 0.0))
            {
                /* avoid more if on the outside of opponent on a bend.
                  Stops us from cutting in too much and colliding...*/
                sidemargin += fabs(rInverse) * 150;
            }

            double sidedist2 = sidedist;
            if (side > 0.0)
            {
                sidedist2 -= (o->getSpeedAngle() - speedangle) * 40;
                sidemargin -= MIN(0.0, rInverse * 100);
            }
            else
            {
                sidedist2 -= (speedangle - o->getSpeedAngle()) * 40;
                sidemargin += MAX(0.0, rInverse * 100);
            }
            sidedist = MIN(sidedist, sidemargin);

            if (sidedist < sidemargin)
            {
                //float side = car->_trkPos.toMiddle - ocar->_trkPos.toMiddle;
                //float w = car->_trkPos.seg->width/WIDTHDIV-BORDER_OVERTAKE_MARGIN;
                double sdiff = (3.0 - (sidemargin - sidedist) / sidemargin);

                if (side > 0.0f) {
                    myoffset += (float)(OVERTAKE_OFFSET_INC*lftinc*MAX(0.2f, MIN(1.0f, sdiff)));
                }
                else {
                    myoffset -= (float)(OVERTAKE_OFFSET_INC*rgtinc*MAX(0.2f, MIN(1.0f, sdiff)));
                }
            }
            else if (sidedist > sidemargin + 3.0 && racelineDrivedata->raceoffset > myoffset + OVERTAKE_OFFSET_INC*incfactor)
            {
                myoffset += OVERTAKE_OFFSET_INC*lftinc / 4;
            }
            else if (sidedist > sidemargin + 3.0 && racelineDrivedata->raceoffset < myoffset - OVERTAKE_OFFSET_INC*incfactor)
            {
                myoffset -= OVERTAKE_OFFSET_INC*rgtinc / 4;
            }

#if 1
            if (ocar->_trkPos.toLeft > car->_trkPos.toLeft)
            {
                avoidmode |= avoidright;
				rightMargin = MAX(leftMargin, MIN(rightMargin, (oppCarTL + MIN(0.0, o->getAvgLateralMovt() * 2)) / track->width));
				if (sidedist < car->_dimension_y + 1.0 || o->getAvgLateralMovt() < avgLateralMovt && (avgLateralMovt - o->getAvgLateralMovt()) / deltaTime * 2.0 > sidedist - 0.5)
				{
					if (rightMargin < car->_trkPos.toLeft / track->width)
						avoidCritical = true;
					if (raceline->tRInverse[LINE_RL][raceline->Next] > 0.001 && avgLateralMovt > 0.0)
						avoidSqueezed = MAX(0.0, sidedist - car->_dimension_y - 1.0);
					//linemode->SetMinRightCurrentMargin((oppCarTL - 0.5) / track->width);
				}
#ifdef OVERTAKE_DEBUG
				fprintf(stderr, "%s - %s: SIDE-AVOID to the Left%s%s\n", car->_name, ocar->_name, avoidCritical ? ", Critical" : "", avoidSqueezed > 0.0 ? ", being squeezed" : ""); fflush(stderr);
#endif
            }
            else
            {
                avoidmode |= avoidleft;
				leftMargin = MIN(rightMargin, MAX(leftMargin, (oppCarTR + MAX(0.0, o->getAvgLateralMovt() * 2)) / track->width));
				if (sidedist < car->_dimension_y + 1.0 || o->getAvgLateralMovt() > avgLateralMovt || (o->getAvgLateralMovt() - avgLateralMovt) / deltaTime * 2.0 > sidedist - 0.5)
				{
					if (leftMargin > car->_trkPos.toLeft / track->width)
						avoidCritical = true;
					if (raceline->tRInverse[LINE_RL][raceline->Next] < -0.001 && avgLateralMovt < 0.0)
						avoidSqueezed = MAX(0.0, sidedist - car->_dimension_y - 1.0);
					//linemode->SetMinLeftCurrentMargin((oppCarTR + 0.5) / track->width);
				}
#ifdef OVERTAKE_DEBUG
				fprintf(stderr, "%s - %s: SIDE-AVOID to the Right%s%s\n", car->_name, ocar->_name, avoidCritical ? ", Critical" : "", avoidSqueezed > 0.0 ? ", being squeezed" : ""); fflush(stderr);
#endif
            }
            avoidmode |= avoidside;
#endif
        }
    }

    if ((avoidmode) & (avoidleft | avoidright))
    {
#ifdef OVERTAKE_DEBUG
fprintf(stderr, "side avoidmode = %s%s\n",(avoidmode & avoidleft) ? "right " : "",(avoidmode & avoidright) ? "left" : "");fflush(stderr);
#endif
        if (avoidmode & avoidside)
        {
            overtake_test_timer = 0.0;
            SetMode(avoidmode, leftMargin, rightMargin, true);
            myoffset = MIN(avoidlftoffset, MAX(avoidrgtoffset, myoffset));
            myoffset = MIN(maxoffset, MAX(minoffset, myoffset));
            // return myoffset;
        }
    }

	if (car->_pit && pit->getPitstop() && pit->isInTrans(car->_distFromStartLine))
	{
		double margin = 0.2;
		avoidmode = avoidright;
		if (track->pits.side == TR_RGT)
		{
			avoidmode = avoidleft;
			margin = 0.8;
		}

		SetMode(avoidmode, (avoidmode == avoidleft ? margin : 0.0), (avoidmode == avoidright ? margin : 1.0), true);
		return 0.0;
	}

    /* Overtake. */
#if 1
	if (CheckOvertaking(leftMargin, rightMargin))
	{
        overtake_timer = simtime;
        return 0.0;
    }

	if (avoidmode & avoidside)
		return 0.0;
#else
#endif

#ifdef OVERTAKE_DEBUG
    if (overtake_test_timer == simtime)
    {
        fprintf(stderr, "OVERTAKE - nothing to overtake\n");
    }
    fflush(stderr);
#endif

    /*==============================================================================*/
    /*                CHANGE LEAD                    */
    /*         Let overlap or let less damaged team mate pass.          */
    /*==============================================================================*/
    o = NULL;
	mindist = -99999.0;

    for (i = 0; i < opponents->getNOpponents(); i++)
    {

#ifdef OVERTAKE_DEBUG // check Team mate name and situation
        Opponent *opp = &opponent[i];
        //if(opponent[i].isTeamMate())
		/*
        if(opp->isTeamMate())
        {
            printf("%s >> TeamMate:%s\n", car->_name, opponent[i].getCarPtr()->_name);
            // if teammate
            if(opp->getDistance() < 0.0f)
            {
                printf("TEAMMATE %s, laps: %i olaps: %i diff_dam: %i dist: %.3f %.3f\n", opponent[i].getCarPtr()->_name, 
                    car->race.laps, opp->getCarPtr()->race.laps, car->_dammage - opp->getDamage(), opp->getDistance(), TEAM_REAR_DIST);
            }

        } 
        else if(opp->getState() & OPP_LETPASS)
        {
            fprintf(stderr, "%s >> Opponent:%s LETPASS=%d\n", car->_name, opponent[i].getCarPtr()->_name,(opp->getState()&OPP_LETPASS));
			fflush(stderr);
        }
		*/
#endif

        /* Let the teammate with less damage overtake to use slipstreaming.
           The position change happens when the damage difference is greater than  TEAM_DAMAGE_CHANGE_LEAD. */
        if (((opponent[i].getState() & OPP_LETPASS) && !opponent[i].isTeamMate()) ||
            (opponent[i].isTeamMate() && (car->_dammage - opponent[i].getDamage() > TEAM_DAMAGE_CHANGE_LEAD) &&
            (opponent[i].getDistance() > -TEAM_REAR_DIST) && (opponent[i].getDistance() < -car->_dimension_x) &&
            car->race.laps == opponent[i].getCarPtr()->race.laps))
        {
            // Behind, larger distances are smaller ("more negative").
            if (opponent[i].getDistance() > mindist && opponent[i].getDistance() < 0.0) {
#ifdef OVERTAKE_DEBUG
				fprintf(stderr, "OVERLAPPER: %s distance %.1f\n", opponent[i].getCarPtr()->_name,opponent[i].getDistance()); fflush(stderr);
#endif
                mindist = opponent[i].getDistance();
                o = &opponent[i];
            }
        }
    }

    if (o != NULL) 
    {
        tCarElt *ocar = o->getCarPtr();
		double oppCarTL = o->getCurrentLft();
		double oppCarTR = o->getCurrentRgt();
		if (raceline->tRInverse[LINE_LEFT][raceline->Next] < -0.001)
			oppCarTL += MIN(3.0, raceline->tRInverse[LINE_LEFT][raceline->Next] * 1000);
		if (raceline->tRInverse[LINE_RIGHT][raceline->Next] > 0.001)
			oppCarTR += MIN(3.0, raceline->tRInverse[LINE_RIGHT][raceline->Next] * 1000);

		rightMargin = 1.0, leftMargin = 0.0;

        float side = car->_trkPos.toMiddle - ocar->_trkPos.toMiddle;
        float w = car->_trkPos.seg->width / WIDTHDIV - BORDER_OVERTAKE_MARGIN;
        if (side > 0.0f) {
			// I'm to his left
            avoidmode |= avoidright;
			rightMargin = MAX(leftMargin, MIN(rightMargin, (oppCarTL + MIN(0.0, o->getAvgLateralMovt() * 2)) / track->width));
        }
        else {
			// I'm to his right
            avoidmode |= avoidleft;
			leftMargin = MIN(rightMargin, MAX(leftMargin, (oppCarTR + MAX(0.0, o->getAvgLateralMovt() * 2)) / track->width));
        }
#ifdef OVERTAKE_DEBUG
fprintf(stderr, "back avoidmode = %s%s\n",(avoidmode & avoidleft) ? "right " : "",(avoidmode & avoidright) ? "left" : "");fflush(stderr);
#endif
        if (avoidmode)
        {
            SetMode(avoidmode, leftMargin, rightMargin, true);

            myoffset = MIN(avoidlftoffset, MAX(avoidrgtoffset, myoffset));
            myoffset = MIN(maxoffset, MAX(minoffset, myoffset));
            return myoffset;
        }
    }

	if (!avoidmode && overrideCollection)
	{
		int defend_line = -1;
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_DEFEND_LINE);
        if (labelOverride)
        {
			double value = 0.0;
			if (labelOverride->getOverrideValue(raceline->Next, &value))
				defend_line = value;
		}

		if (defend_line >= 0)
		{
			if (defend_line > 2)
				defend_line = LINE_MID;

			// look for someone to defend against
			for (i = 0; i < opponents->getNOpponents(); i++)
			{
				Opponent *opp = &opponent[i];
				tCarElt *ocar = opp->getCarPtr();
				if (ocar->_pos > car->_pos + 2 || ocar->_pos < car->_pos || !(opp->getState() & OPP_BACK_CATCHING))
					continue;

				if (defend_line == LINE_MID)
					SetMode(avoidright | avoidleft, 0.3, 0.7);
				else if (defend_line == LINE_RIGHT)
					SetMode(avoidleft, 0.6, 1.0);
				else
					SetMode(avoidright, 0.0, 0.4);
				overtake_timer = simtime + 0.3;
//				fprintf(stderr, "%s: Avoiding\n", car->_name); fflush(stderr);
				break;
			}
		}
	}
#if 0       
    // no-one to avoid, work back towards raceline
    //if (mode != normal && fabs(myoffset) > car->_trkPos.seg->width/2 + 0.5)
    if (mode != normal && fabs(myoffset - racelineDrivedata->raceoffset) > 1.0)
    {
        //if (myoffset > racelineDrivedata->raceoffset)
        //    myoffset -= OVERTAKE_OFFSET_INC * rgtinc/2;
        //else
        //    myoffset += OVERTAKE_OFFSET_INC * lftinc/2;
        if (myoffset > racelineDrivedata->raceoffset + OVERTAKE_OFFSET_INC * rgtinc / 4)
            myoffset -= OVERTAKE_OFFSET_INC * rgtinc / 4;
        else if (myoffset < racelineDrivedata->raceoffset + OVERTAKE_OFFSET_INC * lftinc / 4)
            myoffset += OVERTAKE_OFFSET_INC * lftinc / 4;
    }
#else
    if (!potentialOvertake || situation->currentTime - overtake_timer > MAX(0.2, (car->_speed_x / 50) - fabs(raceline->tRInverse[LINE_RL][raceline->Next])*10))
    {
        overtake_timer = 0.0;
		if (!test_raceline && situation->currentTime > 4.0)
        {
#ifdef OVERTAKE_DEBUG
			if (linemode->GetLeftTargetMargin() != 0.0 || linemode->GetRightTargetMargin() != 1.0)
				fprintf(stderr, "Set Correcting...\n");
#endif
            SetMode(correcting, 0.0, 1.0, true);
        }
    }
    else
    {
#ifdef OVERTAKE_DEBUG
        fprintf(stderr, "Can't correct as %.3f - %.3f < %.2f\n",situation->currentTime,overtake_timer, MAX(0.2, (car->_speed_x / 50) - fabs(raceline->tRInverse[LINE_RL][raceline->Next])*10));
#endif
    }
        
#endif
    if (simtime > 2.0f)
    {
        if (myoffset > racelineDrivedata->raceoffset)
            myoffset = MAX(racelineDrivedata->raceoffset, myoffset - OVERTAKE_OFFSET_INC*incfactor / 2);
        else
            myoffset = MIN(racelineDrivedata->raceoffset, myoffset + OVERTAKE_OFFSET_INC*incfactor / 2);
    }

    myoffset = MIN(maxoffset, MAX(minoffset, myoffset));
    return myoffset;
}

/***************************/
double Driver::GetOvertakeSpeedDiff()
{
	double minSpeedDiff = 0.0;

    if (overrideCollection)
    {
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_OVERTAKE_SPD_DIFF);
        if (labelOverride)
        {
            if (!labelOverride->getOverrideValue(raceline->Next, &minSpeedDiff))
                minSpeedDiff = 0.0;
		}
    }

	return minSpeedDiff;
}

bool Driver::CheckOvertaking(double minLeftMargin, double maxRightMargin)
{
	Opponent *o = NULL;
	tCarElt *ocar = NULL;

	double minSpeedDiff = GetOvertakeSpeedDiff();
    int oppList[128], oppCount = 0, i, j, k;
	double oppCatchTime[128], oppDistance[128], oppRanking[128], oppLeftSpeed[128], oppRightSpeed[128];
	bool oppIsBrakeZone[128];
	double myTmp = AverageTmpForCar(car);
	bool approachingBrakeZone = raceline->ApproachingBrakeZone(NULL);
	double rInverse = raceline->tRInverse[LINE_RL][raceline->Next];
	potentialOvertake = false;

	// Create list of opponents sorted by their importance
	for (i = 0; i < opponents->getNOpponents(); i++)
	{
		ocar = opponent[i].getCarPtr();

		if ((opponent[i].getState() & OPP_FRONT_FOLLOW))
		{
#ifdef OVERTAKE_DEBUG
			fprintf(stderr, "%s >> %s: IGNORED, Following\n", car->_name, ocar->_name); fflush(stderr);
#endif
			continue;
		}

		if (opponent[i].getTeam() == TEAM_FRIEND)
		{
			double oTmp = AverageTmpForCar(ocar);
			if ((oTmp > 60.0 || (myTmp < 70 && myTmp - oTmp < 40)) && (ocar->_fuel < 0.2 || ocar->_dammage < car->_dammage + 2000 || !aloneTeam))
				continue;
		}

		if (fabs(ocar->_trkPos.toMiddle) > car->_trkPos.seg->width / 2 + 4.0 &&
			fabs(car->_trkPos.toLeft - ocar->_trkPos.toLeft) >= 8.0 &&
			!(opponent[i].getState() & OPP_COLL))
		{
#ifdef OVERTAKE_DEBUG
			fprintf(stderr, "%s >> %s: IGNORED, Off Track\n", car->_name, ocar->_name); fflush(stderr);
#endif
			continue;
		}

		if (ocar->_state & RM_CAR_STATE_NO_SIMU)
			continue;

		if ((opponent[i].getState() & OPP_FRONT))
		{
			double distance = opponent[i].getDistance();
			double mySpeed = getSpeed() + 0.5;
			double ospeed = opponent[i].getSpeed();

			if (mySpeed < ospeed - (fabs(rInverse) > 0.001 ? MAX(0.3, 1.0 - fabs(rInverse) * 50) : 1.0 + car->_speed_x/50.0))
			{
#ifdef OVERTAKE_DEBUG
				fprintf(stderr, "%s >> %s: IGNORED, too fast for us\n", car->_name, ocar->_name); fflush(stderr);
#endif
				continue;
			}

			if (opponent[i].getHasSlowerSpeed())
			{
#ifdef OVERTAKE_DEBUG
				fprintf(stderr, "%s >> %s: IGNORED, slower speed between us and collision point\n", car->_name, ocar->_name); fflush(stderr);
#endif
				continue;
			}

			double lftSpeed = raceline->CalculateOfflineSpeed((raceline->Next - 5 + raceline->Divs) % raceline->Divs, raceline->Next, 0.0, (opponent[i].getCurrentLft() - 2.0) / track->width);
			double rgtSpeed = raceline->CalculateOfflineSpeed((raceline->Next - 5 + raceline->Divs) % raceline->Divs, raceline->Next, (opponent[i].getCurrentRgt() + 2.0) / track->width, 1.0);
			if (lftSpeed > car->_speed_x && accelcmd > 0.9)
				lftSpeed = MIN(lftSpeed, car->_speed_x + 3.0);
			if (rgtSpeed > car->_speed_x && accelcmd > 0.9)
				rgtSpeed = MIN(rgtSpeed, car->_speed_x + 3.0);
			double speedDiff = MAX(0.0, MAX(lftSpeed, MAX(rgtSpeed, mySpeed)) - ospeed);

			double catchtime = MAX(0.0, MIN(opponent[i].getTimeToSide(), MAX(lftSpeed, rgtSpeed)));
			double ranking = MIN(distance*5, catchtime);
			bool nearBrakingZone = (opponent[i].getDistance() - speedDiff * 2 < 5.0 && approachingBrakeZone && raceline->ApproachingBrakeZone(ocar));

			if (speedDiff < minSpeedDiff && !nearBrakingZone && !opponent[i].getWithinBrakeDist())
			{
#ifdef OVERTAKE_DEBUG
				fprintf(stderr, "%s >> %s: IGNORED, speed diff %.2f < %.2f\n", car->_name, ocar->_name, speedDiff, minSpeedDiff); fflush(stderr);
#endif
				continue;
			}

			//double importance = fabs((distance / 3) / (distance / 3 * distance / 4) * (MAX(1.0, speedDiff / 6) + (car->_pos < ocar->_pos ? 0.1 : 0.0)));
			double importance = (distance / (distance * distance)) * 75;
			if (nearBrakingZone)
				importance *= 2;
			else if (!opponent[i].getWithinBrakeDist() && distance > 3.0) 
				importance /= 4;

			//if ((catchtime < 0.0 || (catchtime > 1.0 && catchtime/10 > importance)))// && !nearBrakingZone)
			if ((catchtime < 0.0 || (MIN(distance, catchtime) > importance)))
			{
#ifdef OVERTAKE_DEBUG
				fprintf(stderr, "%s >> %s: IGNORED, catchtime %.4f > importance %.4f dist=%.3f\n", car->_name, ocar->_name, catchtime, importance, distance); fflush(stderr);
#endif
				continue;
			}

#ifdef OVERTAKE_DEBUG
			fprintf(stderr, "%s >> %s: CONSIDERING, catchtime/10 %.4f < importance %.4f, nbz %d wBD %d\n", car->_name, ocar->_name, catchtime/10, importance, nearBrakingZone, opponent[i].getWithinBrakeDist()); fflush(stderr);
#endif

			for (j = 0; j < oppCount; j++)
			{
				if (oppRanking[j] > ranking)
					break;
			}
			for (k = oppCount; k > j; k--)
			{
				oppList[k] = oppList[k - 1];
				oppCatchTime[k] = oppCatchTime[k - 1];
				oppDistance[k] = oppDistance[k - 1];
				oppRanking[k] = oppRanking[k - 1];
				oppIsBrakeZone[k] = oppIsBrakeZone[k - 1];
				oppLeftSpeed[k] = oppLeftSpeed[k - 1];
				oppRightSpeed[k] = oppRightSpeed[k - 1];
			}
			oppList[j] = i;
			oppCatchTime[j] = catchtime;
			oppDistance[j] = distance;
			oppRanking[j] = ranking;
			oppIsBrakeZone[j] = (nearBrakingZone && catchtime / 10 > importance);
			oppLeftSpeed[j] = lftSpeed;
			oppRightSpeed[j] = rgtSpeed;
            oppCount++;
		}
	}

	if (oppCount == 0)
		return false;

	potentialOvertake = true;

	int preferLine = raceline->findNextCorner(car, raceline->Next, NULL, NULL);
	double lft_caution = overtake_caution, rgt_caution = overtake_caution;
	if (overrideCollection)
	{
		bool lft_set = false, rgt_set = false;
		LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_OVERTAKE);
		if (labelOverride)
			if (!(lft_set = labelOverride->getOverrideValue(raceline->Next, &lft_caution)))
				lft_caution = overtake_caution;
		labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_OVERTAKE);
		if (labelOverride)
			if (!(rgt_set = labelOverride->getOverrideValue(raceline->Next, &rgt_caution)))
				rgt_caution = overtake_caution;

		if (!lft_set || !rgt_set)
		{
			double tmp = 0.0;
			labelOverride = overrideCollection->getOverrideForLabel(PRV_OVERTAKE);
			if (labelOverride)
				if (labelOverride->getOverrideValue(raceline->Next, &tmp))
				{
					if (!lft_set)
						lft_caution = tmp;
					if (!rgt_set)
						rgt_caution = tmp;
				}
		}
	}

	// look through list for cars we can overtake...
	for (i = 0; i < oppCount; i++)
	{
		// how much space is there on each side of the opponent?
		ocar = opponent[oppList[i]].getCarPtr();
		double minLft = MAX(minLeftMargin * track->width, raceline->edgeLineMargin * track->width + 1.0), maxRgt = MIN(maxRightMargin * track->width, track->width - minLft - 1.0);
		double oppCurrentLft = opponent[oppList[i]].getCurrentLft();
		double oppCurrentRgt = opponent[oppList[i]].getCurrentRgt();
		double impactLft = opponent[oppList[i]].getImpactLft();
		double impactRgt = opponent[oppList[i]].getImpactRgt();
		double oppAvoidLft = MIN(oppCurrentLft, impactLft);
		double oppAvoidRgt = MAX(oppCurrentRgt, impactRgt);

		if (raceline->tRInverse[LINE_LEFT][raceline->Next] > 0.001)
			oppAvoidRgt = MIN(maxRgt, oppAvoidRgt + 1.0);
		if (raceline->tRInverse[LINE_RIGHT][raceline->Next] < -0.001)
			oppAvoidLft = MAX(minLft, oppAvoidLft - 1.0);

		// do we have room to pass?
		if (oppAvoidLft < minLft && oppAvoidRgt > maxRgt)
		{
			oppAvoidLft = oppCurrentLft;
			oppAvoidRgt = oppCurrentRgt;
			if (oppAvoidLft < minLft && oppAvoidRgt > maxRgt)
			{
#ifdef OVERTAKE_DEBUG
				fprintf(stderr, "%s >> %s: BLOCKED! %.2f < %.2f and %.2f > %.2f\n", car->_name, ocar->_name, oppAvoidLft, minLft, oppAvoidRgt, maxRgt); fflush(stderr);
#endif
				continue;
			}
		}

		bool space2Left = oppAvoidLft >= minLft;
		bool space2Right = oppAvoidRgt <= maxRgt;

		// calculate how far we'll travel according to twice the catchtime
		double catchdist = MIN(200.0, (car->_speed_x + (ocar->_pos < car->_pos ? 0.5 : 1.0)) * MIN(3.0, oppCatchTime[i]));
		if (ocar->_speed_x >= car->_speed_x)
			catchdist = car->_speed_x * 2;

		// Find the worst ratio between raceline speed and offline speed, and use that to compare to current 
		// ratio between our current speed (or current offline speed, whichever's bigger) and the opponent.

		// get the minimum speed for each side (if there's room)
		double minLspeed = -1.0, minRspeed = 1.0;
		double curLspeed = -1.0, curRspeed = -1.0;
		double leftSlowSpeedRatio = 999.0, rightSlowSpeedRatio = 999.0;

		int lftDiv = -1, rgtDiv = -1;

		if (oppIsBrakeZone[i])
		{
			if (preferLine == TR_LFT)
				space2Right = oppAvoidRgt <= maxRgt - 3.0;
			else if (preferLine == TR_RGT)
				space2Left = oppAvoidLft >= minLft + 3.0;
		}

		if (space2Left)
		{
			minLspeed = MIN(car->_speed_x + 10.0, raceline->getSlowestSpeedForDistance(catchdist, LINE_LEFT, oppAvoidLft, &lftDiv));
			minLspeed = MIN(car->_speed_x + 10.0, raceline->CalculateOfflineSpeed(lftDiv, (lftDiv + 3) % raceline->Divs, 0.0, (oppAvoidLft - 2.0) / track->width));
			curLspeed = MIN(car->_speed_x + 10.0, oppLeftSpeed[i]);

			if (curLspeed < ocar->_speed_x && curLspeed > 5.0)
				space2Left = false;
			else
			{
				double lSpeed = (curLspeed + car->_speed_x) / 2;
				double newCatchTime = MAX(0.0, oppDistance[i] + car->_dimension_x) / MAX(0.05, (lSpeed - ocar->_speed_x));
				minLspeed = raceline->getSlowestSpeedForDistance(newCatchTime, LINE_LEFT, oppAvoidLft, &lftDiv);
				if (minLspeed < ocar->_speed_x * (ocar->_pos > car->_pos ? 0.9 : 1.0) && minLspeed > 5.0)
					space2Left = false;
			}
#if 0
			if (preferLine == TR_LFT)
				minLspeed *= 1.2;
			else if (preferLine == TR_RGT)
				minLspeed *= 0.9;
#endif
			leftSlowSpeedRatio = raceline->tSpeed[LINE_RL][lftDiv] / minLspeed;
		}
		if (space2Right)
		{
			minRspeed = MIN(car->_speed_x + 10.0, raceline->getSlowestSpeedForDistance(catchdist, LINE_RIGHT, oppAvoidRgt, &rgtDiv));
			minRspeed = MIN(car->_speed_x + 10.0, raceline->CalculateOfflineSpeed(rgtDiv, (rgtDiv + 3) % raceline->Divs, (oppAvoidRgt + 2.0)/ track->width, 1.0));
			curRspeed = MIN(car->_speed_x + 10.0, oppRightSpeed[i]);
			if (curRspeed < ocar->_speed_x && curRspeed > 5.0)
				space2Right = false;
			else
			{
				double rSpeed = (curRspeed + car->_speed_x) / 2;
				double newCatchTime = MAX(0.0, oppDistance[i] + car->_dimension_x) / MAX(0.05, (rSpeed - ocar->_speed_x));
				minRspeed = raceline->getSlowestSpeedForDistance(newCatchTime, LINE_RIGHT, oppAvoidRgt, &rgtDiv);
				if (minRspeed < ocar->_speed_x * (ocar->_pos > car->_pos ? 0.9 : 1.0) && minRspeed > 5.0)
					space2Right = false;
			}
#if 0
			if (preferLine == TR_RGT)
				minRspeed *= 1.2;
			else if (preferLine == TR_LFT)
				minRspeed *= 0.9;
#endif
			rightSlowSpeedRatio = raceline->tSpeed[LINE_RL][rgtDiv] / minRspeed;
		}

		if (IsNan(leftSlowSpeedRatio) || !IsFinite(leftSlowSpeedRatio))
			leftSlowSpeedRatio = 1.0;
		if (IsNan(rightSlowSpeedRatio) || !IsFinite(rightSlowSpeedRatio))
			rightSlowSpeedRatio = 1.0;

		bool avoid2Rgt = (space2Right && curRspeed >= ocar->_speed_x && (rightSlowSpeedRatio <= (curRspeed * rgt_caution) / MAX(1.0, ocar->_speed_x)));
		bool avoid2Lft = (space2Left && curLspeed >= ocar->_speed_x && (leftSlowSpeedRatio <= (curLspeed * lft_caution) / MAX(1.0, ocar->_speed_x)));
#ifdef OVERTAKE_DEBUG
		fprintf(stderr, "%s -> %s spd=%.3f/%.3f a2L=%d %d spd=%.3f/%.3f [%d (%.3f %.3f) %d %d] (%.6f <= (%.3f/%.1f)=%.3f) a2R=%d %d spd=%.3f/%.3f [%d (%.3f %.3f) %d %d] (%.6f <= (%.3f/%.1f)=%.3f\n", car->_name, ocar->_name, car->_speed_x,ocar->_speed_x,avoid2Lft, space2Left, curLspeed, minLspeed, oppAvoidLft >= minLft, oppAvoidLft, minLft, curLspeed >= ocar->_speed_x, leftSlowSpeedRatio <= (curLspeed * lft_caution) / ocar->_speed_x, leftSlowSpeedRatio,curLspeed*lft_caution,ocar->_speed_x,(curLspeed*lft_caution)/ocar->_speed_x,avoid2Rgt, space2Right, curRspeed,minRspeed,oppAvoidRgt <= maxRgt, oppAvoidRgt, maxRgt, curRspeed >= ocar->_speed_x, rightSlowSpeedRatio <= (curRspeed * rgt_caution) / ocar->_speed_x,rightSlowSpeedRatio,curRspeed*rgt_caution,ocar->_speed_x,(curRspeed*rgt_caution)/ocar->_speed_x);
#endif

		// choose side with highest minimum speed provided its still above minSpeedDiff
		int avoidSide = 0;
		if (avoid2Lft && avoid2Rgt)
		{
			if (oppCatchTime[i] > fabs(car->_trkPos.toLeft - ocar->_trkPos.toLeft)/2)
				avoidSide = (ocar->_trkPos.toMiddle > 0.0 ? avoidleft : avoidright);
			else if (car->_trkPos.toLeft > ocar->_trkPos.toLeft + 1.0)
				avoidSide = avoidleft;
			else if (car->_trkPos.toLeft < ocar->_trkPos.toLeft - 1.0)
				avoidSide = avoidright;
			else if (oppAvoidLft > track->width - oppAvoidRgt)
				avoidSide = avoidright;
			else
				avoidSide = avoidleft;

			if (avoidSide == avoidleft)
			{
				SetMode(avoidSide, MAX(minLeftMargin, (oppAvoidRgt+2.0)/track->width), maxRightMargin, true);
#ifdef OVERTAKE_DEBUG
				fprintf(stderr, "%s >> %s: OVERTAKE to RIGHT (1)\n", car->_name, ocar->_name, oppAvoidRgt); fflush(stderr);
#endif
			}
			else
			{
				SetMode(avoidSide, minLeftMargin, MIN(maxRightMargin, (oppAvoidLft-2.0)/track->width), true);
#ifdef OVERTAKE_DEBUG
				fprintf(stderr, "%s >> %s: OVERTAKE to LEFT (1)\n", car->_name, ocar->_name); fflush(stderr);
#endif
			}
		}
		else if (avoid2Lft)
		{
			avoidSide = avoidright;
			SetMode(avoidSide, minLeftMargin, MIN(maxRightMargin, (oppAvoidLft-2.0)/track->width), true);
#ifdef OVERTAKE_DEBUG
			fprintf(stderr, "%s >> %s: OVERTAKE to LEFT (2)\n", car->_name, ocar->_name); fflush(stderr);
#endif
		}
		else if (avoid2Rgt)
		{
			avoidSide = avoidleft;
			SetMode(avoidSide, MAX(minLeftMargin, (oppAvoidRgt+2.0)/track->width), maxRightMargin, true);
#ifdef OVERTAKE_DEBUG
			fprintf(stderr, "%s >> %s: OVERTAKE to RIGHT (2)\n", car->_name, ocar->_name); fflush(stderr);
#endif
		}

		if (avoidSide > 0)
		{
			// todo: save details of what we're avoiding for later comparison
			overtake_timer = simtime + 0.5;
			if (car->_speed_x - ocar->_speed_x > 15.0f && ocar->_speed_x < 10.0f && oppCatchTime[i] < 3.0)
				avoidCritical = true;

			return true;
		}

#ifdef OVERTAKE_DEBUG
		fprintf(stderr, "%s >> %s: DISCARDED aLft=%.2f aRgt=%.2f oSpd=%.2f lSpd=%.2f->%.2f rSpd=%.2f->%.2f ratio l%.2f r%.2f > l%.2f r%.2f\n", car->_name, ocar->_name,oppAvoidLft,oppAvoidRgt,ocar->_speed_x,curLspeed,minLspeed,curRspeed,minRspeed,leftSlowSpeedRatio,rightSlowSpeedRatio,curLspeed/ocar->_speed_x,curRspeed/ocar->_speed_x); fflush(stderr);
#endif
	}

	if (oppCount == 0)
		overtake_timer = 0.0;
	return false;
}


int Driver::GetAvoidSide(Opponent *oppnt, int allowed_sides, double t_impact, double *leftMargin, double *rightMargin)
{
    if (!allowed_sides) return 0;
	t_impact = MIN(2.0, t_impact);
    int avoidSide = 0;
    float mincatchdist = getSpeed()*2;
    tCarElt *oppCar = oppnt->getCarPtr();
    float oppLeftMovt = oppnt->getAvgLateralMovt() * t_impact / deltaTime;
    float myLeftMovt = avgLateralMovt * t_impact / deltaTime;

	float tA = RtTrackSideTgAngleL(&(oppCar->_trkPos));
	float oA = tA - oppCar->_yaw;
	NORM_PI_PI(oA);
	oA = fabs(oA);
	if (oA > 1.6)
		oA = 1.6 - (oA - 1.6);

	double extra_width = car->_dimension_y + oppCar->_dimension_y + oppCar->_dimension_x/2 * (oA / 1.6) + 2.0;

	left_toMid = car->_trkPos.toMiddle + car->_dimension_y / 2 + extra_width;
	right_toMid = car->_trkPos.toMiddle - car->_dimension_y / 2 - extra_width;
    /* Compute the opponents distance to the middle.*/
	float oppExtWidth = extra_width;// fabs(sin(oppCar->_yaw));
    float oppCarTL = oppCar->_trkPos.toLeft - extra_width - MIN(0.0, oppLeftMovt);
    float oppCarTR = oppCar->_trkPos.toLeft + extra_width + MAX(0.0, oppLeftMovt);
    float myCarTL = MAX(0.5, MIN(track->width-0.5, car->_trkPos.toLeft + myLeftMovt));
    float myCarTR = MAX(0.5, MIN(track->width-0.5, car->_trkPos.toRight - myLeftMovt));
    float sidedist = fabs(oppCarTL - myCarTL);
    float base2left = track->width * raceline->tLane[LINE_LEFT][raceline->Next] + car->_dimension_y;
    float base2right = track->width * raceline->tLane[LINE_RIGHT][raceline->Next] - car->_dimension_y;

	if ((allowed_sides & avoidright) && oppCarTL > base2left)
	{
		avoidSide |= avoidright;
		if (oppnt->getState() & OPP_COLL)
			oppCarTL -= 2.0;
		if (rightMargin)
			*rightMargin = MAX(0.0, oppCarTL / track->width);
	}
#ifdef OVERTAKE_DEBUG
	else
		fprintf(stderr, "Can't avoid to the left, oppTL=%.1f base=%.1f\n", oppCarTL, base2left);
#endif
	if ((allowed_sides & avoidleft) && oppCarTR < base2right)
	{
		avoidSide |= avoidleft;
		if (oppnt->getState() & OPP_COLL)
			oppCarTR += 2.0;
		if (leftMargin)
			*leftMargin = MIN(1.0, oppCarTR / track->width);
	}
#ifdef OVERTAKE_DEBUG
	else
		fprintf(stderr, "Can't avoid to the right, oppTR=%.1f base=%.1f\n", oppCarTR, base2right);
#endif

#if 0
    if ((allowed_sides & avoidright) && oppCarTL > racelineDrivedata->leftlane_2left + car->_dimension_y + 2 + MIN((lft_rInverse > 0 ? 2.0 : 2.0), fabs(lft_rInverse)*50) && (car->_trkPos.toLeft < oppCarTL + 2.0 || oppCarTR < 5.0))
    {
        //myoffset -= OVERTAKE_OFFSET_INC*rgtinc;
        avoidSide |= avoidright;
    } 
    else if ((allowed_sides & avoidleft) && oppCarTR > racelineDrivedata->rightlane_2right + car->_dimension_y + 2 + MIN((rgt_rInverse < 0 ? 2.0 : 2.0), fabs(rgt_rInverse)*50) && (car->_trkPos.toRight < oppCarTR + 2.0 || oppCarTL < 5.0))
    {
        //myoffset += OVERTAKE_OFFSET_INC*lftinc;
        avoidSide |= avoidleft;
    } 
#endif

    return avoidSide;
}

/******************************************************************************/
// Update my private data every timestep.
void Driver::update(tSituation *s)
{
    {
        deltaTime = s->deltaTime;
        for (int j=3; j>0; j--)
            speedAngle[j] = speedAngle[j-1];
        double newx = car->_corner_x(FRNT_LFT) + car->_speed_X;
        double newy = car->_corner_y(FRNT_LFT) + car->_speed_Y;
        double dx = newx - car->_corner_x(FRNT_LFT);
        double dy = newy - car->_corner_y(FRNT_LFT);
        speedAngle[0] = atan2(dx, dy);
    }

	float trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
	float ang = trackangle - car->_yaw;
	NORM_PI_PI(ang);
	ang = fabs(ang);
	if (ang > 1.6)
		ang = 1.6 - (ang - 1.6);

	double extra_width = (car->_dimension_x - car->_dimension_y) / 2 * (ang / 1.6);

	double left_toMid = car->_trkPos.toMiddle + car->_dimension_y/2 + extra_width;
	double right_toMid = car->_trkPos.toMiddle - car->_dimension_y/2 - extra_width;
	left_speed_y = (left_toMid - last_left_toMid) / s->deltaTime;
	right_speed_y = (right_toMid - last_right_toMid) / s->deltaTime;
	last_left_toMid = left_toMid;
	last_right_toMid = right_toMid;
       
#if 0
static double thetimer = 0.0;
if (s->currentTime - thetimer >= 1.0)
{
    thetimer = s->currentTime;
    double newx = car->_corner_x(FRNT_LFT) + car->_speed_X;
    double newy = car->_corner_y(FRNT_LFT) + car->_speed_Y;
    double dx = newx - car->_corner_x(FRNT_LFT);
    double dy = newy - car->_corner_y(FRNT_LFT);
    double dist = sqrt((dx*dx) + (dy*dy));
    double adjx = car->_corner_x(FRNT_LFT) + dist * sin(getSpeedAngle(1.0));
    double adjy = car->_corner_y(FRNT_LFT) + dist * cos(getSpeedAngle(1.0));
    fprintf(stderr, "Current=%.2f/%.2f Projected=%.2f/%.2f -> %.2f/%.2f sA=%.2f/%.2f - %.2f %.2f %.2f\n",car->_corner_x(FRNT_LFT),car->_corner_y(FRNT_LFT),newx,newy,adjx,adjy,speedAngle[0],getSpeedAngle(1.0),speedAngle[1],speedAngle[2],speedAngle[3]);fflush(stderr);
}
#endif
    // Update global car data (shared by all instances) just once per timestep.
    if (currentsimtime != s->currentTime) {
        simtime = currentsimtime = s->currentTime;
        cardata->update();
    }

    average_AX = average_AX * 0.75 + car->pub.DynGCg.vel.x * 0.25;
    average_AY = average_AY * 0.75 + car->pub.DynGCg.vel.y * 0.25;
    avgLateralMovt = avgLateralMovt * 0.75 + (car->_trkPos.toLeft - prevToLeft)*0.25;
    avgYawRateDelta = avgYawRateDelta * 0.75 + (car->_yaw_rate - prevYawRate)*0.25;
    prevYawRate = car->_yaw_rate;
    prevToLeft = car->_trkPos.toLeft;

    // Update the local data rest.
    speedangle = (float)-(mycardata->getTrackangle() - atan2(car->_speed_Y, car->_speed_X));
    NORM_PI_PI(speedangle);
    ftank_Mass = car->_fuel * FUEL_FACTOR;
    car_Mass = CARMASS + ftank_Mass;
    //####
    //fprintf(stderr,"# carMass=%.2f with fuelTankMass=%.2f\n", car_Mass, ftank_Mass);
    //####
    currentspeedsqr = car->_speed_x*car->_speed_x;
    opponents->update(s, this);
    strategy->update(car, s);

    //if (car->_state <= RM_CAR_STATE_PIT)
    {
        if (!pit->getPitstop() && (car->_distFromStartLine < pit->getNPitEntry() || car->_distFromStartLine > pit->getNPitEnd()))
        {
            pit->setPitstop(strategy->needPitstop(car, s));
        }

        if (pit->getPitstop() && car->_pit)
        {
            pitstatus[carindex] = 1;
            for (int i = 0; i < opponents->getNOpponents(); i++)
            {
                int idx = opponent[i].getIndex();
                if (opponent[i].getTeam() != TEAM_FRIEND) continue;
                if (opponent[i].getCarPtr() == car) continue;
                if (opponent[i].getCarPtr()->_state > RM_CAR_STATE_PIT)
                    continue;

                if (pitstatus[idx] == 1 ||
                    ((pitstatus[idx] || (opponent[i].getCarPtr()->_fuel < car->_fuel - 1.0 && car->_dammage < 5000))
                    && fabs(car->_trkPos.toMiddle) <= car->_trkPos.seg->width / 2))
                {
                    pit->setPitstop(0);
                    pitstatus[carindex] = 0;
                }
                break;
            }
        }
        else if (!pit->getInPit())
            pitstatus[carindex] = 0;
    }

    pit->update();
    isAlone();
    simtime = s->currentTime;

    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
    angle = -angle;
}

/******************************************************************************/

void Driver::isAlone()
{
    int i;
	aloneTeam = alone = true;
	underThreat = false;
    for (i = 0; i < opponents->getNOpponents(); i++) {
        if ((opponent[i].getState() & (OPP_COLL | OPP_LETPASS)) ||
            ((opponent[i].getState() & (OPP_FRONT)) && opponent[i].getDistance() < MAX(50.0, car->_speed_x*1.5)) ||
            (fabs(opponent[i].getDistance()) < 50.0))
        {
			if (!(opponent[i].getTeam() == TEAM_FRIEND))
				aloneTeam = false;
			alone = false;
        }
		if ((opponent[i].getState() & OPP_BACK_THREAT) && opponent[i].getCarPtr()->_speed_x > car->_speed_x - 10.0)
			underThreat = true;

		if (underThreat && !alone && !aloneTeam) return;
    }

	return;
}

double	Driver::AverageTmpForCar(CarElt *car)
{
	double tmp = 0.0;
	for (int i = 0; i<4; i++)
		tmp += CT(i);

	return tmp / 4;
}


/******************************************************************************/

// Check if I'm stuck.
bool Driver::isStuck()
{
    if (simtime < 0.0f || (pit->getInPit() && fabs(angle) < 0.65 && car->_state == RM_CAR_STATE_PIT))
    {
        // starting the race or in pit, no reversing allowed
        stuck_reverse_timer = stuck_stopped_timer = -1.0f;
		if (car->_state == RM_CAR_STATE_PIT)
			stuck_stopped_timer = simtime;
        return false;
    }
    if (stuck_reverse_timer > 0.0f)
    {
        // already reversing, do we keep reversing?
        if (simtime - stuck_reverse_timer > MIN(3.0, MAX(2.0f, fabs(angle)*10)) || car->_dammage > stuck_damage + 100)
        {
            stuck_reverse_timer = -1.0f;
            stuck_stopped_timer = simtime;
            return false;
        }
        return true;
    }

    if (fabs(mycardata->getCarAngle()) > MAX_REALLYSTUCK_ANGLE ||
        fabs(car->_speed_x) < MAX_UNSTUCK_SPEED ||
        (fabs(mycardata->getCarAngle()) > MAX_UNSTUCK_ANGLE &&
         fabs(car->_speed_x) < MAX_UNSTUCK_SPEED &&
         fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST))
    {
        if (stuck_stopped_timer > 0.0f && car->_speed_x > MAX_UNSTUCK_SPEED)
            stuck_stopped_timer = -1.0f;

        if (stuck_stopped_timer < 0.0f)
        {
            // we have to wait for stopped timer to exceed limit before we can reverse
            if (fabs(car->_speed_x) < MAX_UNSTUCK_SPEED)
                stuck_stopped_timer = simtime;
            return false;
        }
        if ((angle > 0.0 && car->_trkPos.toMiddle < 0.0 && laststeer < 0.0) ||
            (angle < 0.0 && car->_trkPos.toMiddle > 0.0 && laststeer > 0.0))
            return false;
        else if (simtime - stuck_stopped_timer > (fabs(mycardata->getCarAngle()) > MAX_UNSTUCK_ANGLE ? 2.5f : 4.5f))
        {
            stuck_stopped_timer = -1.0f;
            stuck_reverse_timer = simtime;
            stuck_damage = car->_dammage;
            return true;
        }
        return false;
    }

    stuck_stopped_timer = -1.0f;
    return false;
}

/******************************************************************************/

// Compute aerodynamic downforce coefficient CA.
void Driver::initCa()
{
    char *WheelSect[4] = { (char *)SECT_FRNTRGTWHEEL, (char *)SECT_FRNTLFTWHEEL, (char *)SECT_REARRGTWHEEL, (char *)SECT_REARLFTWHEEL };
    double rearwingarea = GfParmGetNum(car->_carHandle, (char *)SECT_REARWING, (char *)PRM_WINGAREA, (char*)NULL, 0.0);
    double rearwingangle = GfParmGetNum(car->_carHandle, (char *)SECT_REARWING, (char *)PRM_WINGANGLE, (char*)NULL, 0.0);
    double frontwingarea = GfParmGetNum(car->_carHandle, (char *)SECT_FRNTWING, (char *)PRM_WINGAREA, (char*)NULL, 0.0);
    double frontwingangle = GfParmGetNum(car->_carHandle, (char *)SECT_FRNTWING, (char *)PRM_WINGANGLE, (char*)NULL, 0.0);
    double frontclift = GfParmGetNum(car->_carHandle, (char *)SECT_AERODYNAMICS, (char *)PRM_FCL, (char*)NULL, 0.0);
    double rearclift = GfParmGetNum(car->_carHandle, (char *)SECT_AERODYNAMICS, (char *)PRM_RCL, (char*)NULL, 0.0);
    double rearwingca = 1.23*rearwingarea*sin(rearwingangle);
    double frntwingca = 1.23*frontwingarea*sin(frontwingangle);

    double cl = frontclift + rearclift;
    double h = 0.0;

    int i;
    for (i = 0; i < 4; i++)
    {
        h += GfParmGetNum(car->_carHandle, WheelSect[i], (char *)PRM_RIDEHEIGHT, (char*)NULL, 0.20f);


    }
    h *= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
    CA = h*cl + 4.0*((frntwingca + rearwingca) / 2);
    //fprintf(stderr,"CA=%.3f\n",CA);
    FCA = h*frontclift + 4.0*frntwingca;
    RCA = h*rearclift + 4.0*rearwingca;

}

/******************************************************************************/

// Compute aerodynamic drag coefficient CW.
void Driver::initCw()
{
    float cx = GfParmGetNum(car->_carHandle, (char *)SECT_AERODYNAMICS, (char *)PRM_CX, (char*)NULL, 0.0f);
    float frontarea = GfParmGetNum(car->_carHandle, (char *)SECT_AERODYNAMICS, (char *)PRM_FRNTAREA, (char*)NULL, 0.0f);
    CW = 0.645f*cx*frontarea;

}

/******************************************************************************/

void Driver::initCR()
{
    CR = GfParmGetNum(car->_carHandle, (char *)SECT_CAR, (char *)PRM_FRWEIGHTREP, (char *)NULL, 0.50);

}

/******************************************************************************/

// Init the friction coefficient of the the tires.
void Driver::initTireMu()
{
    char *WheelSect[4] = { (char *)SECT_FRNTRGTWHEEL, (char *)SECT_FRNTLFTWHEEL, (char *)SECT_REARRGTWHEEL, (char *)SECT_REARLFTWHEEL };
    float tm = FLT_MAX;
    int i;

    for (i = 0; i < 4; i++) {
        tm = MIN(tm, GfParmGetNum(car->_carHandle, WheelSect[i], (char *)PRM_MU, (char*)NULL, 1.0f));


    }
    TIREMU = tm;

}

/******************************************************************************/

// Reduces the brake value such that it fits the speed (more downforce -> more braking).
float Driver::filterBrakeSpeed(float brake)
{
    bool coll = (brake > 1.5f);
    brake = MIN(brake, 1.0f);
    float weight = (CARMASS + car->_fuel)*G;
    float maxForce = weight + CA*MAX_SPEED*MAX_SPEED;
    float force = weight + CA*currentspeedsqr;
    return (brake*force / maxForce) * (coll ? 1.5f : 1.0f * baseBrake);
}

/******************************************************************************/

// Brake filter for pit stop.
float Driver::filterBPit(float brake)
{

	float dl = 0.0, dw;
	if (pit->getPitstop()) {
		RtDistToPit(car, track, &dl, &dw);
        if (dl < PIT_BRAKE_AHEAD) {
            float mu = car->_trkPos.seg->surface->kFriction*TIREMU*PIT_MU;
            if (strategy->pitStopPenalty() != RM_PENALTY_DRIVETHROUGH && (brakedist(0.0f, mu) > dl || dl < 3.0))
                return (car->_speed_x < 3.0f ? 1.0f : 0.6f);
        }
    }

    if (pit->getInPit()) 
    {
        float s = pit->toSplineCoord(car->_distFromStartLine);
        // Pit entry.
        if (pit->getPitstop()) 
        {
            float mu = car->_trkPos.seg->surface->kFriction*TIREMU*PIT_MU;

            if (s >= pit->getNPitEnd())
            {
                if (strategy->pitStopPenalty() == RM_PENALTY_DRIVETHROUGH)
                {
                    pit->setPitstop(false);
                    return 0.0f;
                }
            }

            if (s < pit->getNPitStart()) 
            {
                // Brake to pit speed limit.
                float dist = pit->getNPitStart() - s;
                if (brakedist(pit->getSpeedlimit(), mu) > dist)
                    return 0.7f;
            }
            else 
            {
                // Hold speed limit.
                if (currentspeedsqr > pit->getSpeedlimitSqr()) 
                    return pit->getSpeedLimitBrake(currentspeedsqr);
            }
            if (strategy->pitStopPenalty() != RM_PENALTY_DRIVETHROUGH)
            {
                // Brake into pit (speed limit 0.0 to stop)
                float dist = pit->getNPitLoc() - s;
                if (pit->isTimeout(dist)) 
                {
                    pit->setPitstop(false);
                    return 0.0f;
                }
                else 
                {
                    if (brakedist(0.0f, mu) > dist || dl < 3.0f) 
                    {
                        stuck_reverse_timer = stuck_stopped_timer = -1.0f;
                        return 1.0f;
                    }
                    else if (s > pit->getNPitLoc()) 
                    {
                        // Stop in the pit.
                        stuck_reverse_timer = stuck_stopped_timer = -1.0f;
                        return 1.0f;
                    }
                }
            }
        }
        else 
        {
            // Pit exit.
            if (s < pit->getNPitEnd()) 
            {
                // Pit speed limit.
                if (currentspeedsqr > pit->getSpeedlimitSqr()) 
                    return pit->getSpeedLimitBrake(currentspeedsqr);
            }
        }
    }

    return brake;
}

/******************************************************************************/

// Brake filter for collision avoidance.
float Driver::filterBColl(float brake)
{
    return brake;
    if (simtime < 1.5)
        return brake;

    float mu = car->_trkPos.seg->surface->kFriction;
    int i;
    float thisbrake = 0.0f, collision = 0.0f;;
    for (i = 0; i < opponents->getNOpponents(); i++)
    {
        if (opponent[i].getState() & OPP_COLL)
        {
            return 2.0f;
        }
    }
    return MIN(1.0f, MAX(thisbrake, brake));
}

/******************************************************************************/

// Antilocking filter for brakes.
float Driver::filterABS(float brake)
{
    if (car->_speed_x < ABS_MINSPEED) return brake;
    float origbrake = brake;
    float rearskid = MAX(0.0f, MAX(car->_skid[2], car->_skid[3]) - MAX(car->_skid[0], car->_skid[1]));
    int i;
    float slip = 0.0f;
    for (i = 0; i < 4; i++) {
        slip += car->_wheelSpinVel(i) * car->_wheelRadius(i);
    }
    //slip *= 1.0f + MAX(rearskid, MAX(fabs(car->_yaw_rate) / 5, fabs(angle) / 6));
    slip = car->_speed_x - slip / 4.0f;
	double this_abs_slip = ((currentCollision || mode != normal) ? (abs_slip + (abs_range - abs_slip) / 2) : abs_slip);
    if (slip > this_abs_slip) {
        brake = brake - MIN(brake, (slip - this_abs_slip) / abs_range);
    }

#if 1
    if (car->_speed_x > 5.0)
    {
        double skidAng = atan2(car->_speed_Y, car->_speed_X) - car->_yaw;
        NORM_PI_PI(skidAng);
        skidAng = MIN(skidAng * 2, PI);
        brake *= MAX(0.5, MIN(1.0, (fabs(cos(skidAng))*2)));
    }
#endif

    brake = MAX(brake, MIN(origbrake, 0.1f));
    return brake;
}

/******************************************************************************/

// TCL filter for accelerator pedal.
float Driver::filterTCL(float accel, int raceType)
{
    //if (simtime < 4.0 && car->_gear <= 1)
    //    return accel;

    accel = MIN(1.0f, accel);
    float accel1 = accel, accel2 = accel, accel3 = accel, accel4 = accel;

    double slip = (this->*GET_DRIVEN_WHEEL_SPEED)() - (car->_speed_x+0.1);
    double this_tcl_slip = (double)tcl_slip;//(car->_speed_x > 10.0f ? tcl_slip : tcl_range/10);
	double this_tcl_range = (double)tcl_range;
	if (overrideCollection)
	{
		LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_TCL_SLIP);
		if (labelOverride)
			if (!labelOverride->getOverrideValue(raceline->Next, &this_tcl_slip))
				this_tcl_slip = (double)tcl_slip;
		labelOverride = overrideCollection->getOverrideForLabel(PRV_TCL_RANGE);
		if (labelOverride)
			if (!labelOverride->getOverrideValue(raceline->Next, &this_tcl_range))
				this_tcl_range = (double)tcl_range;
	}

	//if (simtime < 4.0 && car->_gear <= 1)
	//	this_tcl_slip /= 2;
    if (slip > this_tcl_slip && this_tcl_slip < this_tcl_range)
    {
		if (simtime < 10.0f)
			this_tcl_range *= 2.0f;
#if 1
		accel3 *= tclf;
		tclf = MAX(0.1, tclf - 0.05);
#else
        //accel3 = accel3 - MIN(accel3, (slip - this_tcl_slip) / (tcl_range - this_tcl_slip));
        accel3 = accel3 - MIN(accel3, (slip - this_tcl_slip) / tcl_range) * tclf;
		tclf = MAX(0.1, tclf - 0.05);
#endif
	}
	else
		tclf = MIN(0.9, tclf + 0.2);

#if 1
    //if (raceType != RM_TYPE_QUALIF)
    {
        double height = 0.0;
        tTrkLocPos wp;
        double wx = car->pub.DynGCg.pos.x;
        double wy = car->pub.DynGCg.pos.y;
        RtTrackGlobal2Local(car->_trkPos.seg, wx, wy, &wp, TR_LPOS_SEGMENT);
        height = car->_pos_Z - RtTrackHeightL(&wp) - car->_wheelRadius(REAR_LFT) - suspHeight*2;

        if (height > 0.0)
        {
            accel1 = MAX(accel3, 1.0);
            accel2 = MAX(accel3, 1.0);
            accel3 = MAX(accel3, 1.0);
            accel4 = MAX(accel3, 1.0);
	    accel1 = MIN(accel1, 0.2);
        }
    }
#endif

#if 1
    double skidAng = atan2(car->_speed_Y, car->_speed_X) - car->_yaw;
    NORM_PI_PI(skidAng);
    if (car->_speed_x > 5.0 && fabs(skidAng) > 0.35)// && ((laststeer > 0.8 && skidAng > 0.0) || (laststeer < -0.8 && skidAng < 0.0)))
        accel4 = MIN(accel4, 0.25 - (fabs(skidAng)-0.35) * 10);
    if (car->_speed_x > 0.0 && car->_speed_x < 12.0 && fabs(car->_yaw_rate) > 1.8 && ((laststeer > 0.0 && car->_yaw_rate < 0.0) || (laststeer < 0.0 && car->_yaw_rate > 0.0)))
        accel4 = MIN(accel4, 0.25 - (fabs(car->_yaw_rate) - 1.8)/2);
#endif
    return MIN(accel1, MIN(accel2, MIN(accel3, accel4)));
}

/******************************************************************************/

// Traction Control (TCL) setup.
void Driver::initTCLfilter()
{
    const char *traintype = GfParmGetStr(car->_carHandle, (char *)SECT_DRIVETRAIN, (char *)PRM_TYPE, (char *)VAL_TRANS_RWD);
    if (strcmp(traintype, VAL_TRANS_RWD) == 0) {
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_RWD;
    }
    else if (strcmp(traintype, VAL_TRANS_FWD) == 0) {
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_FWD;
    }
    else if (strcmp(traintype, VAL_TRANS_4WD) == 0) {
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_4WD;
    }
}

/******************************************************************************/

// TCL filter plugin for rear wheel driven cars.
float Driver::filterTCL_RWD()
{
    return (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
        car->_wheelRadius(REAR_LFT) / 2.0f;
}

/******************************************************************************/

// TCL filter plugin for front wheel driven cars.
float Driver::filterTCL_FWD()
{
    return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
        car->_wheelRadius(FRNT_LFT) / 2.0f;
}

/******************************************************************************/

// TCL filter plugin for all wheel driven cars.
float Driver::filterTCL_4WD()
{
    return ((car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
        car->_wheelRadius(FRNT_LFT) +
        (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
        car->_wheelRadius(REAR_LFT)) / 4.0f;
}

/******************************************************************************/

// Hold car on the track.
float Driver::filterTrk(float targetAngle)
{
    float angle = targetAngle;

#if 1
    float tw = (car->_trkPos.seg->width - 2.0 * car->_dimension_y) / 2.0f;
    //float tw2 = car->_trkPos.seg->width/2.0f;
    float tm = car->_trkPos.toMiddle;

    /* filter steering to stay on track */
    //if ( (fabs(tm) > tw) && (fabs(tm) < tw2) ) 
    if (fabs(tm) > tw)
    {
        float adjust;
        adjust = -2.5 * PI / 180.0 * tm / tw;

        if (fabs(adjust) > 5.0 * PI / 180.0)
        {
            if (adjust > 0.0)
                adjust = 5.0 * PI / 180.0;
            else
                adjust = -5.0 * PI / 180.0;
        }
        NORM_PI_PI(adjust);
        angle += adjust;
    }

    double maxchange = 0.05 + MAX(0.0, (100 - car->_speed_x) / 100) * 0.5;
    angle = MAX(targetAngle - maxchange, MIN(targetAngle + maxchange, angle));
#endif

    return angle;
}

/******************************************************************************/

// Compute the needed distance to brake.
float Driver::brakedist(float allowedspeed, float mu)
{
    float c = mu*G;
    float d = (CA*mu + CW) / car_Mass;
    float v1sqr = currentspeedsqr;
    float v2sqr = allowedspeed*allowedspeed;
    return -log((c + v2sqr*d) / (c + v1sqr*d)) / (2.0f*d);
}

