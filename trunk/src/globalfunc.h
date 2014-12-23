/** \file globalfunc.h
 *  \ingroup core
 *  \brief Globally used functions

 *  Global functions for serial/socket RX/TX and set serial etc..
 *
 *  \author Anders Billesø Beck
 *  $Rev: 878 $
 *  $Date: 2010-03-04 15:37:31 +0100 (Thu, 04 Mar 2010) $
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

#ifndef GLOBALFUNC_H
  #define GLOBALFUNC_H

  #ifdef __cplusplus
  extern "C" {
  #endif

  extern ssize_t secureWrite(int fd, const void *buf, ssize_t txLen);
  extern ssize_t secureRead(int fd, void *buf, ssize_t txLen);
  extern ssize_t secureSend(int fd, const void *buf, ssize_t txLen);
  extern ssize_t secureRecv(int fd, void *buf, ssize_t txLen);
  extern int set_serial(int fd, int baud);

  #ifdef __cplusplus
  }
  #endif

#endif

