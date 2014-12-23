/**
 * Interface for phidgetbridge force sensor board
 *
 * $Rev: 215 $
 * $Id: phidgetbridge.h 215 2013-07-17 09:03:55Z jcan $
 *
 *******************************************************************/

#ifndef PHIDGETSBRIDGE_2_H
#define PHIDGETSBRIDGE_2_H

#include <phidget21.h>

/** RHD calls initXML once just after the plugin is loaded
    used to read configuration file and set initial state
    and open and configure connection to device */
extern int initXML(char *);
/**
 * RHD calls this function every time write variables may have beed updated
 * normally every 10ms (RHD sample time)
 * \param RHDtick is RHD sample count. */
extern int periodic(int RHDtick);
/**
 * Called by RHD when RHD is shutting down.
 * Should be used to close files and connections */
extern int terminate (void) ; //No shutdown function

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

/// structures used by this plugin

/// communication variables
struct
{ /** buffer for communication */
  #define MxBL 128
  char rxBuf[MxBL];
  /// start of unused message, if no unused message, then p2 == NULL
  char * p2;
  /// if connection is lost - restart?
  int lostConnection;
  /// requested update time in range 8..1000 (in millisec
  int updateTms;
  /// requested gain of bridge (1..128)
  int gain[4];
  /// enable channels
  int enableCh[4];
  /// Offset
  int offset[4];
  // K gain
  int k[4];
  
  ///
  CPhidgetBridgeHandle bridge;
} busif;


struct controlStateStruct
{ // steering and drive
  /// detected force in micro-volt input to bridge
  int varForce;
  int varData;
  int varDataFiltered;
  /// overload - out of range - detection
  int varRangeErr;
  /// index to x, y
  int idxX, idxY;
  /// values
  double x,y;
  /// detected update rate in Hz
  int varUpdateRate;

  /// filter - fraction of measured value used
  /// 0,1 = no filter
  /// 2: y=y*1/2+m*1/2
  /// 3: y=y*2/3+m*1/3
  /// 4: y=y*3/4+m*1/4
  int varFilterR;
  int varFilterW;
  int filter;
  int varX; // normalized force 
  int varY; // normalized value
  /// last rate update time
  double rateTime;
  /// updateCnt since laste rate time
  int dataCnt;
  double values[4];
  int varEnableCh;


} data;

#endif

