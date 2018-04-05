##############################################################################
#
#    file                 : Makefile
#    created              : 9 May 2017
#    copyright            : (C) 2017 Tim Foden
#    version              : $Id$
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = mouse_2017
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp \
		Avoidance.cpp CarBounds2d.cpp CarModel.cpp \
		ClothoidPath.cpp Cubic.cpp CubicSpline.cpp \
		LearnedGraph.cpp LinearRegression.cpp MyRobot.cpp MyTrack.cpp \
		Opponent.cpp ParametricCubic.cpp ParametricCubicSpline.cpp \
		Path.cpp PathRecord.cpp PidController.cpp PitControl.cpp \
		PitPath.cpp PtInfo.cpp Quadratic.cpp Seg.cpp Shared.cpp \
		Stuck.cpp TeamInfo.cpp Utils.cpp Vec2d.cpp Vec3d.cpp WheelModel.cpp
SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml logo.rgb
SHIPSUBDIRS = 0 1 cars

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-mouse_2017_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-mouse_2017_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
