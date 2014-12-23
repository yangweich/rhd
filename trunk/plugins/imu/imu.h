/** \file gps.h
 *  \ingroup hwmodule
 *  \brief Serial GPS Module
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
 *  $Rev: 59 $
 *  $Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $
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

#define IMU_DEV_MAX 4
#define DEF_STR_LEN 128

int initXML(char *filename);

struct imuData
{
  int  insDev;      ///INS Port file pointer
  char insDevStr[DEF_STR_LEN];
  int connection;
  //int iAttitude; 
  int iAccRaw;
  int iGyroRaw; 
  int iMagRaw;
  int iConnected;
  char buf[DEF_STR_LEN];
};

#endif

