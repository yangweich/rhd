/** \file rhd.h
 *  \brief Robot Hardware Daemon Library
 *  Socket interface to the Robot Hardware Daemon HAL 
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

#ifndef RHD_H
  #define RHD_H

  #include <stdint.h>

  /** Definitions **/
  #define MAXNAMELEN 32
  #define DEFAULTHOST "127.0.0.1"
  #define DEFAULTPORT 24902

  /** \brief Holds data to define an element of the database
  * 
  * The struct contains the 32-bit data, a name string,
  * an boolean to variable update and a timestamp for 
  * last variable update.
  * \param data is an array of "length" data (32 bit integers) elements
  * \param length is the length of the integer array
  * \param name is the name of the variable, it should
  * start with a character and may contain characters and numbers plus underscore.
  * please do not use any other characters. The name is case sensitive.
  * \param updated this flag is set by the data source (rhd-plugin or a data writer client)
  * and is an indication that the RHD should distribute the value across to data users.
  * The data user can use this flag to see that a "read" variable data producer is alive and kicking.
  * A write variable change is spread to other data clients, but only the "changed" flag
  * and timestamp is set. The update flag is not set, as this is used to trigger
  * data transfer back to the RHD. At the RHD side the update flag is always set and should
  * trigger transfer of data to other clients.
  * \param changed flag is set when the received package has actually changed
  * a value in "data". the data user can use this flag to see that some value in "data"
  * is actually changed.
  * \param timestamp may be set by the data producer, and is transferred to the data user.
  * \param inputVar is a pointer used by the MRC.
  */
  typedef struct  {
      int32_t *data;            //Pointer to the dataarea
      int32_t length;
      char    name[MAXNAMELEN+1]; //Name describing the variable
      uint8_t updated;
      uint8_t changed;
      struct timeval time;  //timestamp - regardless if 32 or 64 bit integers
      struct timeval *timestamp;  //Pointer to the timestamp
      double  *inputVar;          //Pointer to the MRC variable
    } symTableElement;

  /** Function Prototypes **/
  char              rhdDisconnect(void);
  char              rhdConnect(char, char *, int);
  char              rhdSync(void);
  symTableElement*  getSymbolTable(char);
  int               getSymbolTableSize(char);
  int               readValue(int, int);
  int               readValueNamed(char *, int);
  int *             readArray(int);
  int *             readArrayNamed(char *);
  int               writeValue(int, int, int);
  int               writeValueNamed(char *, int, int);
  int               writeArray(int, int, int*);
  int               writeArrayNamed(char *,int, int*);
  int               isRead(int);
  int               isReadNamed(char *);


#endif

