/***************************************************************************

    file                 : manual_override.cpp
    created              : Sat Feb 07 19:53:00 CET 2015
    copyright            : (C) 2015 by Andrew Sumner
    email                : novocas7rian@gmail.com
    version              : $Id: manual_override.cpp,v 1.0 2015/02/07 20:11:49 andrew Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//#include <portability.h>
#include <raceman.h>

#include "manual_override.h"
#include "vardef.h"

#if defined(WIN32)
#define snprintf _snprintf
#endif

#define SECT_OVERRIDE "overrides"

LManualOverride::LManualOverride(const char *theLabel)
{
    overrideValues = (OverrideValue *)malloc(20 * sizeof(OverrideValue));
    overrideValueCount = 0;
    overrideValueAlloc = 20;
    label = theLabel;
}

LManualOverride::~LManualOverride()
{
    free(overrideValues);
}

void LManualOverride::loadFromFile(char **file_contents, int linecount)
{
    char testlabel[64];
    overrideValueCount = 0;

    snprintf(testlabel, 63, "%s<", label);

    for (int line=0; line<linecount; line++)
    {
        char *text = file_contents[line];
        if (isHeading(text))
        {
            if (!strcmp(text, testlabel) && line < linecount-1)
            {
                line++;
                char *workingText = (char *) malloc(strlen(file_contents[line])+1);
                strcpy(workingText, file_contents[line]);
                int sdist = 0, edist = 0;
                double value = 0.0;
                if (readValues(workingText, &sdist, &edist, &value))
                {
                    if (overrideValueCount >= overrideValueAlloc)
                    {
                        overrideValueAlloc *= 2;
                        overrideValues = (OverrideValue *)realloc(overrideValues, overrideValueAlloc * sizeof(OverrideValue));
                    }
                    overrideValues[overrideValueCount].startdist = sdist;
                    overrideValues[overrideValueCount].enddist = edist;
                    overrideValues[overrideValueCount].value = value;
                    overrideValueCount++;
                }
                free(workingText);
            }
        }
    }
}

bool LManualOverride::readValues(char *text, int *sdist, int *edist, double *value)
{
    bool success = false;
    *value = -1000.0;
    *sdist = *edist = -1000;

    char *token = strtok(text, " ");
    if (token)
    {
        *sdist = atoi(token);
        token = strtok(NULL, " ");
        if (token)
        {
            *edist = atoi(token);
            token = strtok(NULL, " ");
            if (token)
            {
                *value = atof(token);
                success = true;
            }
        }
    }
    return success;
}

bool LManualOverride::isHeading(char *text)
{
    int len = strlen(text);
    if (len > 0)
    {
        if (text[len-1] == '<')
        {
            return true;
        }
    }
    return false;
}

void LManualOverride::saveToFile(FILE *filepointer)
{
    if (!filepointer)
        return;

    char str[128];

    for (int i=0; i<overrideValueCount; i++)
    {
        snprintf(str, 127, "%s<\n", label);
        fprintf(filepointer, str);
        snprintf(str, 127, "%.4f %.4f %.4f\n\n", overrideValues[i].startdist, overrideValues[i].enddist, overrideValues[i].value);
        fprintf(filepointer, str);
    }
}

bool LManualOverride::getOverrideValue(int dist, double *value)
{
    for (int i=0; i<overrideValueCount; i++)
    {
        if (distInRange(dist, overrideValues[i].startdist, overrideValues[i].enddist))
        {
            *value = overrideValues[i].value;
            return true;
        }
    }
    return false;
}

void LManualOverride::setOverrideValue(int startdist, int enddist, double value)
{
    if (overrideValueCount >= overrideValueAlloc)
    {
        overrideValueAlloc *= 2;
        overrideValues = (OverrideValue *)realloc(overrideValues, overrideValueAlloc * sizeof(OverrideValue));
    }

    overrideValues[overrideValueCount].startdist = startdist;
    overrideValues[overrideValueCount].enddist = enddist;
    overrideValues[overrideValueCount].value = value;
    overrideValueCount++;
}


bool LManualOverride::distInRange(int dist, int startdist, int enddist)
{
    if (startdist < enddist)
    {
        if (dist >= startdist && dist <= enddist)
            return true;
    }
    else
    {
        if (dist >= startdist || dist <= enddist)
            return true;
    }
    return false;
}


#define OVERRIDE_SIZE 64
static const char *overrideLabels[OVERRIDE_SIZE] = { 
    PRV_CORNERSPEED, 
    PRV_CORNERSPEED_MID, 
    PRV_CORNERSPEED_SLOW, 
    PRV_CORNERSPEED_COLD, 
    PRV_BRAKEDELAY, 
    PRV_BRAKEDELAY_MID, 
    PRV_BRAKEDELAY_SLOW, 
    PRV_RIGHTCORNERSPEED, 
    PRV_LEFTCORNERSPEED, 
    PRV_RIGHTCORNERSPEED_COLD, 
    PRV_LEFTCORNERSPEED_COLD, 
    PRV_AVOIDBRAKEDELAY, 
    PRV_RACELINECURVE, 
    PRV_BUMP_CAUTION, 
    PRV_LEFT_BUMP_CAUTION, 
    PRV_RIGHT_BUMP_CAUTION, 
    PRV_AVOID_BUMPCAUTION, 
    PRV_SLOPE_FACTOR, 
    PRV_AVOID_SLOPE, 
    PRV_LEFT_MARGIN, 
    PRV_RL_LEFT_MARGIN, 
    PRV_LEFT_MARGIN_MID, 
    PRV_LEFT_MARGIN_SLOW, 
    PRV_RL_RIGHT_MARGIN,
    PRV_RIGHT_MARGIN,
    PRV_RIGHT_MARGIN_MID,
    PRV_RIGHT_MARGIN_SLOW,
    PRV_MAX_SPEED,
    PRV_LEFT_MAX_SPEED,
    PRV_RIGHT_MAX_SPEED,
    PRV_OUTSIDE_DAMPENER,
    PRV_PREFERRED_SIDE,
    PRV_LOOKAHEAD,
    PRV_LEFT_OVERTAKE,
    PRV_RIGHT_OVERTAKE,
    PRV_OVERTAKE,
    PRV_TRANSITION_INC,
    PRV_LFT_TRANS_INC,
    PRV_RGT_TRANS_INC,
    PRV_RL_FOR_OVERTAKE,
    PRV_LOOKAHEAD_LEFT,
    PRV_LOOKAHEAD_RIGHT,
    PRV_BRAKE_COEFFICIENT,
    PRV_OVERTAKE_SPD_DIFF,
    PRV_STAY_INSIDE,
    PRV_COLLBRAKE_TIMPACT,
	PRV_LEFT_BRAKEDELAY_COLD,
	PRV_RIGHT_BRAKEDELAY_COLD,
	PRV_LEFT_BRAKEDELAY,
	PRV_RIGHT_BRAKEDELAY,
	PRV_BRAKEDELAY_COLD,
	PRV_DEFEND_LINE,
	PRV_TCL_SLIP,
	PRV_TCL_RANGE,
	PRV_MIN_LANE,
	PRV_MAX_LANE,
	PRV_BRAKEZONE,
	PRV_SPEEDERROR,
    NULL
};

LManualOverrideCollection::LManualOverrideCollection()
{
	int i;
    overrides = (LManualOverride **) malloc(OVERRIDE_SIZE * sizeof(LManualOverride *));
	for (i = 0; i < OVERRIDE_SIZE; i++)
	{
		if (!(overrideLabels[i])) break;
		overrides[i] = new LManualOverride(overrideLabels[i]);
	}

	override_count = i;
}

LManualOverrideCollection::~LManualOverrideCollection()
{
    for (int i=0; i<override_count; i++)
        delete overrides[i];
    free(overrides);
}

void LManualOverrideCollection::loadFromFile(char *trackName, const char *carName, int raceType)
{
    char buffer[1025];
    char truncatedTrackName[512];
    int len = strlen(trackName);
    strcpy(truncatedTrackName, trackName);
    if (len > 4)
        truncatedTrackName[len-4] = 0;
    snprintf(buffer, 1024, "%sdrivers/%s/%s/%s/%s.dat", GetDataDir(), BOT_NAME, carName, (raceType == RM_TYPE_RACE ? "race" : (raceType == RM_TYPE_QUALIF ? "qualifying" : "practice")), truncatedTrackName); 
    FILE *filepointer = fopen(buffer, "r");

    if (!filepointer)
    {
        fprintf(stderr, "Unable to open data file %s\n", buffer);
        fflush(stderr);
        return;
    }

    int linecount = 1, i;
    while (fgets(buffer, 1022, filepointer) != NULL)
    {
        linecount++;
    }

    if (linecount)
    {
        fseek(filepointer, 0, SEEK_SET);
        char **file_contents = (char **) malloc(linecount * sizeof(char *));

        linecount = 0;
        while (fgets(buffer, 1022, filepointer) != NULL)
        {
            file_contents[linecount] = (char *) malloc(strlen(buffer)+1);
            strcpy(file_contents[linecount], buffer);
            removeNewLineCharacters(file_contents[linecount]);
            linecount++;
            file_contents[linecount] = NULL;
        }

        for (i=0; i<override_count; i++)
        {
            overrides[i]->loadFromFile(file_contents, linecount);
        }

        for (i=0; i<linecount; i++)
            if (file_contents[i])
                free(file_contents[i]);
        free(file_contents);
        fprintf(stderr, "%d lines loaded from data file\n", linecount);
        fflush(stderr);
    }

    fclose(filepointer);
}

void LManualOverrideCollection::saveToFile()
{
#if 0
    char buffer[1025];
    snprintf(buffer, 1024, "%sdrivers/%s/%s/%s.dat_save", GetDataDir(), BOT_NAME, carName, trackName); 
    FILE *filepointer = fopen(buffer, "w");

    if (filepointer)
    {
        for (int i=0; i<OVERRIDE_COUNT; i++)
        {
            overrides[i]->saveToFile(filepointer);
        }

        fclose(filepointer);
    }
#endif
}

LManualOverride *LManualOverrideCollection::getOverrideForLabel(char *label)
{
    if (label)
    {
        for (int i=0; i<override_count; i++)
        {
            if (!strcmp(label, overrideLabels[i]))
                return overrides[i];
        }
    }
    return NULL;
}

void LManualOverrideCollection::removeNewLineCharacters(char *text)
{
    char *p = text + (strlen(text)-1);
    while (p >= text && (*p == 13 || *p == 10 || *p == ' ' || *p == '\t'))
    {
        *p = 0;
        p--;
    }
}

