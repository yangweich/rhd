/** 
 *  
 *
 *   RHD Link
 *   Author: Peter Juhl Savnik, S113556
 *   DTU Automation
 *   2014 dec
 * 
 * 	This plugin allows this instance of RHD to connect to another  RHD instance via the Scocket.
 * 
 *******************************************************************/


#ifndef RHDLINK_H
#define RHDLINK_H

// Connect to RHD instance
//int connectRHD();

// Test connection
//int isConnectedRHD();

// Sync both read/write variables
//int syncRHD();

// Stop
//int stopRHD();


// RHD control functions
  //Plugin function definition
  extern int initXML(char*);
  extern int periodic(int );
  extern int terminate(void);
  extern int getDatabaseVariableLink(char type, const char * name);

#endif

