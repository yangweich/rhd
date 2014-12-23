/** \file database.h
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
 *  $Rev: 1948 $
 *  $Date: 2012-07-04 12:07:15 +0200 (Wed, 04 Jul 2012) $
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
#include <sys/time.h>

#include <rhd.h> //Robot Hardware Daemon definitions

#ifndef DATABASE_H
  #define DATABASE_H 1

  #ifdef __cplusplus
  extern "C" {
  #endif

  //Definitions
  #define MAXNAMELEN 32

  //Publically oriented functions
/**
 * \brief Add a variable array to the variable pool
 *
 * Adds one variable array to the read or write variable pool
 *
 * \param[in] char dir
 * Direction of the variable beeing added to the pool
 *
 * \param[in] char length
 * Number of variables in the array
 *
 * \param[in] *char name
 * Name of the variable beeing added to the pool
 *
 * \returns int id
 * The Id number of the variable that has been created. Negative on error.
 */
  int createVariable(char dir, char length, const char* name);

/**
 * \brief Lock the variable pool for RT operation
 *
 * \returns int status
 * Returns the boolean status if the pool is locked or not
 */
  int databaseSoftRealtime(void);

/**
 * \brief Get the size of the database data area
 *
 * \param[in] char dir
 * Direction of variable pool
 *
 * \returns int size
 * Returns the size of the database in the given direction in 32-bit segments
 */
  int getDatabaseSize(char dir);

/**
 * \brief Get the size of the database symbol table
 *
 * \param[in] char dir
 * Direction of variable symbol table
 *
 * \returns int size
 * Returns the size of the symbol table in number of elements
 */
  int getSymtableSize(char dir);

/** \brief Get a pointer to the symbol table
 *
 * \param[in] char dir
 * Direction of variable pool
 *
 * \returns int size
 * Returns the size of the database in the given direction
 * Null is returned in case of error
 */
  symTableElement* getSymtable(char dir);

/** \brief Update a variable in the data pool
 *
 * \param[in] int id
 * Variable Id
 *
 * \param[in] int index
 * Variable index in the array
 *
 * \param[in] int value
 * Value of the variable
 *
 * \returns int success
 * Returns the if the variable is successfully updated
 */
  int setVariable(int id, int index, int value);

/** \brief Update a array in the data pool
 *
 * \param[in] int id
 * Variable Id
 *
 * \param[in] int length
 * Length of the input array
 *
 * \param[in] int* array
 * Pointer to the array
 *
 * \returns int success
 * Returns the if the variable is successfully updated
 */
  int setArray(int id, int length, int* array);

/** \brief Get a write variable from the data pool
 *
 * \param[in] int id
 * Variable Id
 *
 * \param[in] int index
 * Variable index in the array
 *
 * \returns int value
 * Returns the if the variable is successfully read
 */
  int getWriteVariable(int id, int index);

/** \brief Get a read variable from the data pool
 * @todo not implemented
 * \param[in] int id
 * Variable Id
 *
 * \param[in] int length
 * Length of the input array
 *
 * \param[in] int* array
 * Pointer to the array
 *
 * \returns int success
 * Returns the if the variable is successfully updated
 */
  int getWriteArray(int id, int length, int* array);

/** \brief Get a read variable from the data pool
 *
 * \param[in] int id
 * Variable Id
 *
 * \param[in] int index
 * Variable index in the array
 *
 * \returns int value
 * Returns the if the variable is successfully updated
 */
  int getReadVariable(int id, int index);

/**
 * \brief Get a read array from the data pool
 * @todo not implemented
 * \param[in] int id
 * Variable Id
 *
 * \param[in] int length
 * Length of the input array
 *
 * \param[in] int* array
 * Pointer to the array
 *
 * \returns int success
 * Returns the if the variable is successfully updated
 */
  int getReadArray(int id, int length, int* array);

/**
 * \brief Resets the update flag on the symbol tables
 *
 * \param[in] char dir
 * Direction of the databuffer to clear
 *
 * \returns int success
 * Returns the if the variable is successfully updated
 */
 int resetUpdate(char dir);

/**
 * \brief Check if a variable is updated
 *
 * \param[in] char dir
 * Direction of the variable
 *
 * \param[in] int id
 * Variable Id
 *
 * \returns int value
 * Returns the if the variable is successfully updated
 */
 int isUpdated(char dir, int id);

/** \brief Lock database access to avoid multiple threads read/writing
 *
 */
  void lockDatabaseAccess(void);

/** \brief Reopen database access to avoid multiple threads read/writing
 *
 */
  void openDatabaseAccess(void);

/**
 * print database variables to console.
 * \param type is symbol table 'w' is write variables and 'r' is read
 */
  void printDatabase(char type);
  /** read flag for plug-ins - counts number of masters (with write access) */
  extern int isMasterAlive;

  #ifdef __cplusplus
  }
  #endif

#endif

