/** \file sheduler.h
 *  \ingroup core
 *  \brief RT Sheduler
 *
 *  Real Time sheduler for RHD
 *
 *  The sheduler support a range of options for running a periodic task.
 *  Using the normal linux task sheduler, it supports sheduling by 
 *  itimer-interrupts and a calculated usleep. Using the basic linux
 *  sheduler, the wait period must be a multiple of the timer frequency
 *  of the linux kernel (default 250Hz = 4 ms)
 *  
 *  If complied with RTAI support, the third option is to use the LXRT
 *  userspace RT sheduler. Communication from the LXRT layer and normal
 *  userspace layer is performed by FIFO, ensuring that the main task
 *  does not need to move to the LXRT layer.
 *
 *  \author Anders Billesø Beck
 *  $Rev: 895 $
 *  $Date: 2010-03-19 11:45:18 +0100 (Fri, 19 Mar 2010) $
 */
 /**************************************************************************
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
#include <sys/time.h>

#ifndef SHEDULER_H
  #define SHEDULER_H

  #ifdef __cplusplus
  extern "C" {
  #endif

  int waitPeriodic(struct timeval *);
  int shedulerInitXML(char *);
  int schedulerRealtime(void);
  int shutdownSheduler(void);
  int getSchedulerPeriod(void);
  int setSchedulerPeriod(int period);

  #ifdef __cplusplus
  }
  #endif

#endif
