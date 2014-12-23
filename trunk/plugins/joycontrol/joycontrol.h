/** \file joystickcontrol.c
 *  \ingroup hwmodule
 *  \brief Remotecontrol plugin using joystick
 *
 *  This plug-in provides remote control possibilities using a
 *  standard HID joystick.
 * 
 *  The joystick driver should be initialized in safety-mode
 *  to be able to override the automatic computerized control. 
 * 
 *  \author Anders Billesø Beck
 *  $Rev: 59 $
 *  $Date: 2011-07-13 10:30:58 +0200 (Wed, 13 Jul 2011) $
 *  
 */
/***************************************************************************
 *                  Copyright 2011 Anders Billesø Beck, DTU                *
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

#ifndef JOYCONTROL_H
  #define JOYCONTROL_H

/********* Joystick definitions  *********/

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */


  //Plugin function definition
  extern int initXML(char*);
  extern int periodic(int );
  extern int terminate(void);
#endif

