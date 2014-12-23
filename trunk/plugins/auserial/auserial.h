/** \file auserial.h
 *  \ingroup hwmodule
 *  \brief Hardware abstraction layer for AU Serial bus
 *
 *  This liberary inplements the hardware abstraction layer
 *  for communicating using the serial bus protocol used on the 
 *  SMR platform in the Institute of Automation (AU) on the Technical
 *  University of Denmark
 * 
 *  Initialization is done by XML that creates the desired serial busses
 *  devices and defines the commands and variables associated by the devices
 *  
 *  Interface is done through the variable pool, that ensures thread 
 *  safety and open access.
 *
 *  Recieving from the serial bus is done in seperate threads, and 
 *  automatically transferred to the variable pool.
 *
 *  \author Anders Billesø Beck
 *  $Rev: 59 $
 *  $Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $
 */
/***************************************************************************
 *                  Copyright 2008 Anders Billesø Beck, DTU                *
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

#include <stdint.h>


#ifndef AUSERIAL_H
  #define AUSERIAL_H 1

extern int initXML(char *);
extern int periodic(int);
	//extern int terminate (void) ; //No shutdown function

#endif


