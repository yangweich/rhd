 /** \file buspirate.h
 *  \ingroup hwmodule
 *
 *   interface to bus pirate - using its i2c bus sniffer mode
 *
 * $Rev: 59 $
 * ¤Id$
 *
 *******************************************************************/

#ifndef BUSPIRATE_H
#define BUSPIRATE_H

extern int initXML(char *);
extern int periodic(int);
extern int terminate (void) ;

/// communication variables
struct
{ /** buffer for communication */
  #define MxBL 4000
  char txBuf[MxBL];
  char rxBuf[MxBL];
  /** device file descriptor */
  int ttyDev;  //File descriptors
  #define MxDL 64
  char devName[MxDL];
  /// serial speed
  int baudrate;
  /// is connection OK
  int lostConnection;
} devif;

#define MBL 50
/// variables for MD25 controller
struct
{ /** RHD variables for motor controller */
  /// sample rate in Hz - max is 3-4 Hz
  double sampleRate;
  /// base name for variables in RHD
  char basename[MBL];
  /// control value to ruddre
  int varToRudder;
  /// string from instrument
  int varCompas;
  /// compas update count
  int varCCnt;
  /// value to rudder cnt update count
  int varRCnt;
  /// settings string from device
  char setting[MBL];
} bupi;



#endif

