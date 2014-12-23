/** \file cruizcore.c
 *  \ingroup hwmodule
 *  \brief Driver for the Microinfinity CruizCore Gyro module
 *
 *  This driver interfaces the Microinfinity CruizCore XG1010
 *
 *  Interface to the hardware is rather simplistic, so data is directly
 *  forwarded from the gyro and only calibration reset is avaliable in
 *  the opposite direction
 * 
 *  \author Anders Billesø Beck
 *  $Rev: 59 $
 *  $Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $
 *  
 */
/***************************************************************************
 *                  Copyright 2010 Anders Billesø Beck, DTU / DTI          *
 *                       anbb@teknologisk.dk                               *
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

#ifndef CRUIZCORE_H
  #define CRUIZCORE_H

	//Plugin function definition
  extern int initXML(char*);
  extern int periodic(int );
  extern int terminate(void);
#endif

