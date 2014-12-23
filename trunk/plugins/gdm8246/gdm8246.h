 /** \file gdm8246.h
 *  \ingroup hwmodule
 *
 *   Interface to digital multimeter  type GDM-8246 from GW INSTEK
 *
 * $Rev: 59 $
 * ¤Id$
 *
 *******************************************************************/

#ifndef GDM8246_H
#define GDM8246_H

extern int initXML(char *);
extern int periodic(int);
extern int terminate (void) ;

/// communication variables
struct
{ /** buffer for communication */
  #define MxBL 32
  char txBuf[MxBL];
  char rxBuf[MxBL];
  /** device file descriptor */
  int ttyDev;  //File descriptors
  #define MxDL 64
  char devName[MxDL];
  /// serial speed
  int baudrate;
  /// is connection loas
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
  /// measured value
  int varVal;
  /// string from instrument
  int varSetting;
  /// update count
  int varCnt;
  /// settings string from device
  char setting[MBL];
} gdm;



#endif

