/** \file gpsSocket.h
 *  \ingroup hwmodule
 *  \brief GPS Module serial and socket
 *
 * GPS Module for RHD. 
 * 
 * Based on HAKOD by Asbjørn Mejnertsen and Anders Reeske Nielsen
 * and AuGps by Lars Mogensen and Christian Andersen
 * 
 * The module supports both standard NMEA GPS and RTK GPS through
 * serial port
 *
 *  \author Anders Billesø Beck
 *  modified by Claes Jaeger
 *  $Rev: 173 $
 *  $Date: 2013-03-28 14:14:50 +0100 (Thu, 28 Mar 2013) $
 */
 /***************************************************************************
 *                  Copyright 2008 Anders Billesø Beck                     *
 *                       anders.beck@get2net.dk                            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#ifndef GPS_H
  #define GPS_H

int initXML(char *filename);

#endif

