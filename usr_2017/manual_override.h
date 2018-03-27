/***************************************************************************

    file                 : manual_override.h
    created              : Sat Feb 07 19:53:00 CET 2015
    copyright            : (C) 2015 by Andrew Sumner
    email                : novocas7rian@gmail.com
    version              : $Id: manual_override.h,v 1.0 2015/02/07 20:11:49 andrew Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#ifndef __MANUAL_OVERRIDE_H
#define __MANUAL_OVERRIDE_H

#include <stdio.h>
#include <tgf.h>

#include "xmldefs.h"

typedef struct {
    int startdist;
    int enddist;
    double value;
} OverrideValue;

class LManualOverride {
    public:
        LManualOverride(const char *theLabel);
        ~LManualOverride();

        void loadFromFile(char **file_contents, int linecount);
        void saveToFile(FILE *filepointer);
        bool getOverrideValue(int dist, double *value);
        void setOverrideValue(int startdist, int enddist, double value);
        int valueCount() { return overrideValueCount; }

    private:
        OverrideValue *overrideValues;
        int overrideValueCount;
        int overrideValueAlloc;
        const char *label;

        bool distInRange(int dist, int startdist, int enddist);
        bool isHeading(char *text);
        bool readValues(char *text, int *sdist, int *edist, double *value);
};

class LManualOverrideCollection {
    public:
        LManualOverrideCollection();
        ~LManualOverrideCollection();

        void loadFromFile(char *trackname, const char *carname, int racetype);
        void saveToFile();
        LManualOverride *getOverrideForLabel(char *label);

    private:
        int divCount;
        int override_count;
        LManualOverride **overrides;

        void removeNewLineCharacters(char *text);
};

#endif  /// __MANUAL_OVERRIDE_H
