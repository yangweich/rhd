 /** \file sableandmagenc.h
 *  \ingroup hwmodule
 *
 * Interface for sabertooth motor controller and AS4050 magnetic encoders
 * through an arduino nano velocity controller
 *
 * $Rev: 476 $
 * $Id: frontenc.h 476 2014-04-15 09:29:57Z jcan $
 *
 *******************************************************************/

#ifndef FRONTENC_H
#define FRONTENC_H

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
typedef struct
{ /** buffer for communication */
  #define MxBL 128
  char rxBuf[MxBL];
  /// start of unused message, if no unused message, then p2 == NULL
  char * p2;
  /** device file descriptor */
  int ttyDev;  //File descriptors
  #define MxDL 64
  char serialDev[MxDL];
  /// if connection is lost - restart?
  int lostConnection;
  /// baudrate
  int baudrate;
} Busif;

Busif busif1;


struct controlStateStruct
{ /** encoder tics per revolution  */
  int encTicsPerRev;
  /// index to encoder left, right, tilt
  int varEncFront;
  /// index to encoder error count left, right, tilt
  int varEncFrontErr;
  /// state of magnet detector 0 = OK, 1=moving away, 2=moving closer, 3=bad
  int varEncMagnetState;
  /// index emergency stop pushed
  int varEmergStopPushed;
  /// battery voltage (24V)
  int varBattery;
  /// index to safety stop flag - no write master or switch pushed.
  int varSafetyStop;
  /// is speed set to 0,0 for safety reasons (switch or no write master)
  int safetyStop;
  /// offset to be added to motor value to make zero speed
  int zeroSpeedOffset[2];
  /// enable communication with front wheel encoders
  int enableFrontEncoder;
} sast;

#endif

