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
SHIPDIR     = drivers/${ROBOT}/cars
SHIP        = 
SHIPSUBDIRS = $(shell find * -maxdepth 0 -type d -print)

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-mouse_2017_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-mouse_2017_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
