/** 
 *  
 *
 *   Tether Control of UAV - Flight Control System
 *   Author: Peter Savnik
 *
 *******************************************************************/


#ifndef FCS_H
#define FCS_H

extern int initXML(char *filename);
int terminate(void);
extern int periodic(int rhdTick);

#endif

