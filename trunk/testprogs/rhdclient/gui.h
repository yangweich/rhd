/** \file gui.c
 *  \brief Graphical user interface for RHDTest client program
 *
 *  \author Anders Billesø Beck
 *  $Rev: 841 $
 *  $Date: 2010-02-12 13:55:42 +0100 (Fri, 12 Feb 2010) $
 */
 /***************************************************************************
 *                  Copyright 2009 Anders Billesø Beck                     *
 *                  anders.billeso.beck@teknologisk.dk                     *
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

#include <pthread.h>

#ifndef GUI_H
	#define GUI_H

#ifdef	__cplusplus
extern "C" {
#endif


extern volatile int running;

void initGUI(pthread_mutex_t *);
void clearGui(void);
void drawFrame(void);


void printStatus(void);
void printVariables(void);
void printPeriod(void);
void printMessage(void);
void popup(char *, double);

//Interface functions
int getMaxY(void);
int getMaxX(void);
int setRhdStatus(char);
int setRhdHost(char *rhdh);

#ifdef	__cplusplus
}
#endif
#endif




