/** \file cmdparser.h
 *  \brief Command input-handler and parser for rhdtest
 *
 *  \author Anders Billesø Beck
 *  $Rev: 602 $
 *  $Date: 2009-07-08 15:19:39 +0200 (Wed, 08 Jul 2009) $
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

#ifndef _CMDPARSER_H
#define	_CMDPARSER_H

#ifdef	__cplusplus
extern "C" {
#endif


//Startposition of command input
#define XCMDLINESTART   5
#define YCMDLINESTART   (getMaxY()-2)    //2 lines from bottom

//Lines of line-buffer when reading inputs
#define BUFFERLINES     10
//Buffersize
#define BUFLEN          256

void cmdParser(pthread_mutex_t *);
int parseCmd(char *cmd);



#ifdef	__cplusplus
}
#endif

#endif	/* _CMDPARSER_H */

