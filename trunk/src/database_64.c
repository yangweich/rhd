/** \file database.c
 *  \ingroup core
 *  \brief Variable database
 *
 *  This database is the intermidiate data storage between the hardware 
 *  layer and the real time client layer
 *
 *  It is thread and real-time safe, as all data access is mutex-protected
 *  and no stack-allocation is performed after initialization.
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1945 $
 *  $Date: 2012-07-03 21:04:17 +0200 (Tue, 03 Jul 2012) $
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>

#include "database.h"

/******** Variables ***********/
symTableElement *rSymTable, *wSymTable;
int32_t *rData, *wData;
char poolLocked = 0;
int  rTableLen = 0,  wTableLen = 0;
int  rDataLen = 0,   wDataLen = 0;
int     iterator;
pthread_mutex_t databaseMutex = PTHREAD_MUTEX_INITIALIZER;;

struct timeval sysTime;
int isMasterAlive = 0;

/**
 * Updated for 64 bit compatible / chr feb 2013
 * - data structure in rData and wData
 * 8 bytes (2x int32_t) is timestamp in sec and usec
 * newlen times 4 bytes (int32_t), then next variable - all in 4-byte units
 * - rSymTable and wSymTable uses a struct symTableElement as unit
 * 
 * This function reallocates space on tables to hold new data
 */
int createVariable(char dir, char newLen, const char *name) {

  if (poolLocked) return -1; //Abort if pool is locked for RT operation
  
	symTableElement *tempTable;
	int32_t *tempData; // chr 64 bit
	int *tempTableLen;
	int *tempDataLen;
  int i;
  int32_t *tmpPtr; // chr 64 bit

	//Select the read or write pool
  if (dir == 'r') {
    tempTable = rSymTable;
		tempData = rData;
    tempTableLen = &rTableLen;
		tempDataLen = &rDataLen;
  } else if (dir == 'w') {
    tempTable = wSymTable;
		tempData = wData;
    tempTableLen = &wTableLen;
		tempDataLen = &wDataLen;
  } else {
    printf("Database: Invalid direction: %c for variable %s \n",dir,name);
    return -1;
  }

  //Allocate memory for the variable in the Symbol table
  tempTable = realloc(tempTable,((*tempTableLen)+1) * sizeof(symTableElement));
  if (tempTable == NULL) {
    printf("Database: Error allocating memory for %s in the symbol table\n",name);
  }
	
	//Allocate memory for the variable in the data field
	tempData = realloc(tempData,(*tempDataLen) * sizeof(int32_t) + /*sizeof(struct timeval)*/ + (newLen + 2) * sizeof(int32_t));
  if (tempData == NULL) {
    printf("Database: Error allocating memory for %s in the data field\n",name);
  }
  //Setup data (clear all data in new element)
  memset(&tempTable[*tempTableLen],0, sizeof(symTableElement));
  // set length of new element
  tempTable[*tempTableLen].length = newLen;
  // set time pointer
  tempTable[*tempTableLen].timestamp = &tempTable[*tempTableLen].time; // (struct timeval *)(tempData + (*tempDataLen) + 1);
  // set now as time
  gettimeofday(tempTable[*tempTableLen].timestamp,NULL);
// tempTable[*tempTableLen].data = (int *)(tempData + (*tempDataLen) + (sizeof(struct timeval)/sizeof(int)) + 1);
  // not needed - done below
// tempTable[*tempTableLen].data = (int32_t *)(tempData + (*tempDataLen) + 2);// + 1);
  strncpy(tempTable[*tempTableLen].name,name,MAXNAMELEN);
  (*tempTableLen)++;
//	(*tempDataLen)+= (sizeof(struct timeval)/sizeof(int)) + newLen;
  (*tempDataLen)+= 2 + newLen;
  memset(tempData,0,*tempDataLen * sizeof(int32_t));

  //Recreate datapointers efter realloc
  for(i = 0, tmpPtr = tempData; (i < *tempTableLen); i++) {
    tempTable[i].timestamp = &tempTable[i].time; //(struct timeval *) tmpPtr;
    tmpPtr += 2; //sizeof(struct timeval)/sizeof(int);
    tempTable[i].data = tmpPtr;
    tmpPtr += tempTable[i].length;
  }

  //Reassign the database pointers after realloc
  if (dir == 'r') {
		rSymTable = tempTable;
		rData = tempData;
	} else {
		wSymTable = tempTable;
		wData = tempData;
	}

  //printf("Database: Created %c variable: %s[%d] as Id %d - Database size: %d\n",dir,tempTable[*tempTableLen-1].name,newLen,(*tempTableLen),(*tempDataLen));

  //Return the variable id
  return (*tempTableLen) -1;
}


int databaseSoftRealtime(void) {

  poolLocked = 1;
  printf("   Database: Database is locked for real-time operation\n");
  return poolLocked;
	

}

int getDatabaseSize(char dir) {

  if (dir == 'r') {
    return rDataLen;
  } else if (dir == 'w') {
    return wDataLen;
  } else return -1;
}

int getSymtableSize(char dir) {
  
  if (dir == 'r') {
    return rTableLen;
  } else if (dir == 'w') {
    return wTableLen;
  } else return -1;
}

 symTableElement* getSymtable(char dir) {

  if (dir == 'r') {
    return (symTableElement *) rSymTable;
  } else if (dir == 'w') {
    return (symTableElement *) wSymTable;
  } else return NULL;
}

int setVariable(int id, int index, int value) {
  
  //Check bounds of update
  if ((id < 0) || (id >= rTableLen) || (index < 0) || 
      (index > rSymTable[id].length)) {
    return -1;
  }
  lockDatabaseAccess(); //Lock mutex
      rSymTable[id].data[index] = value;
      rSymTable[id].updated = 1;
      gettimeofday(rSymTable[id].timestamp,NULL);
  openDatabaseAccess(); //Reopen mutex;
  return 1;

}

int setArray(int id, int length, int* array) {
  
//Check bounds of update
  if ((id < 0) || (id >= rTableLen) || (length < 0) || 
      (length > rSymTable[id].length)) {
    return -1;
  }
  lockDatabaseAccess(); //Lock mutex
    memcpy(rSymTable[id].data, array, length * sizeof(int32_t));
    rSymTable[id].updated = 1;
    gettimeofday(rSymTable[id].timestamp,NULL);
  openDatabaseAccess(); //Reopen mutex;

  return 1;
}


  int getWriteVariable(int id, int index) {
  //Check bounds of update
  int value;
  if ((id < 0) || (id >= wTableLen) || (index < 0) || 
      (index > wSymTable[id].length)) {
    return -1;
  }
  lockDatabaseAccess(); //Lock mutex
      value = wSymTable[id].data[index];
  openDatabaseAccess(); //Reopen mutex;
  return value;

}

//int getWriteArray(int id, int index, int* array) {/* TODO */ return -1;}

  int getReadVariable(int id, int index) {
  //Check bounds of update
  int value;
  if ((id < 0) || (id >= rTableLen) || (index < 0) || 
      (index > rSymTable[id].length)) {
    return -1;
  }
  lockDatabaseAccess(); //Lock mutex
      value = rSymTable[id].data[index];
  openDatabaseAccess(); //Reopen mutex;
  return value;

}

//  int getReadArray(int id, int index, int* array) {/* TODO */ return -1;}

int resetUpdate(char dir) {

  int returnValue;

  if (dir == 'w') {
    for(iterator = 0; iterator < wTableLen; iterator++) {
      wSymTable[iterator].updated = 0;
    }
    returnValue = 1;
  } else if (dir == 'r') {
    for(iterator = 0; iterator < rTableLen; iterator++) {
      rSymTable[iterator].updated = 0;
    }
    returnValue = 1;
  } else returnValue = -1;

 return returnValue;
}

  int isUpdated(char dir, int id) {
  //Check bounds of update
  int value;
  if ((id < 0) || ((dir == 'r') && (id >= rTableLen)) || 
      ((dir == 'w') && (id >= wTableLen))) {
    return -1;
  }
  lockDatabaseAccess(); //Lock mutex
    if (dir == 'r') {
      value = rSymTable[id].updated;
    } else {
      value = wSymTable[id].updated;
    }
  openDatabaseAccess(); //Reopen mutex;
  return value;

}


void lockDatabaseAccess(void) {
 pthread_mutex_lock(&databaseMutex);

}


void openDatabaseAccess(void) {
 pthread_mutex_unlock(&databaseMutex);

}

void printDatabase(char type)
{ // get index of a variable in the database
  symTableElement * syms;
  int symsCnt;
  int i, j;
  //
  syms = getSymtable(type);
  symsCnt = getSymtableSize(type);
  for (i = 0; i < symsCnt; i++)
  {
    printf("%c %17s : upd %d :", type, syms->name, syms->updated);
    for (j = 0; j < syms->length; j++)
      printf(" %d", syms->data[j]);
    printf("\n");
    syms++;
  }
}

/** \brief Get a pointer to the symbol table
 * 
 * \param[in] char dir
 * Direction of symbol table ('r' or 'w')
 *
 * \returns int size
 * Returns a pointer of the database in the given direction
 * Null is returned in case of error
 */
symTableElement*  getSymbolTable(char dir) {

  if (dir == 'r') {
    return (symTableElement *) rSymTable;
  } else if (dir == 'w') {
    return (symTableElement *) wSymTable;
  } else return NULL;
}

/** \brief Get the number of elements in the symbol table
 * 
 * \param[in] char dir
 * Direction of symbol table ('r' or 'w')
 *
 * \returns int size
 * Returns the number of elements in the symboltable in the given direction
 * Negative is returned in case of error
 */
int getSymbolTableSize(char dir) {

  if (dir == 'r') {
    return rTableLen;
  } else if (dir == 'w') {
    return wTableLen;
  } else return -1;

}
