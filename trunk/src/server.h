/** \file server.h
 *  \ingroup core
 *  \brief TCP/IP socket server
 *
 *  This is the TCP socket server for the Robot Hardware Daemon
 *
 *  It maintains syncronisation between the variable databases in each end
 *  of the connection.
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1696 $
 *  $Date: 2011-09-18 13:43:12 +0200 (Sun, 18 Sep 2011) $
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


#include <pthread.h>
#include <rhd.h>

#ifndef SERVER_H
  #define SERVER_H

  #ifdef __cplusplus
  extern "C" {
  #endif

struct client {
  int fd;
  volatile int ready;     /*>0 to enable data to client */
  char mode;
  char *wupdated;          /*>  Pointer to updated write-variable flags array */
  char *rupdated;          /*>  Pointer to updated read-variable flags array */
};

/** Server configuration struct **/
struct config {
  int  port;              //Port
  int  clients;           //Number of clients to handle
  int  allwriters;           // number of allowed writers (default is 1, else all are allowed to write)
  char synchronized;      //Scheduler synchronized to master client
};

int serverInitXML(char *);
int startServer(void);
int syncClients(void);
int disconnectServer(void);

  #ifdef __cplusplus
  }
  #endif

#endif

