#include <stdint.h>
 /** \file usbiss.h
 *  \ingroup hwmodule
 *
 *   Interface for two herkulex DRS-0201 servoes for steering of ackermann type robot
 *   
 *
 * $Rev: 1683 $
 * ¤Id$
 *
 *******************************************************************/

#ifndef MAESTRO12CH_H
#define MAESTRO12CH_H

extern int initXML(char *);
extern int periodic(int);
extern int terminate (void) ;

/// communication variables
struct
{ /** buffer for communication */
  #define MxBL 32
//  unsigned char txBuf[MxBL];
  unsigned char rxBuf[MxBL];
  /** device file descriptor */
  int ttyDev;  //File descriptors
  #define MxDL 64
  char serialDev[MxDL];
  /// stuck flag 0=not stuck, 1 = struk
  int stuck;
  /// is connection lost during read or write operation
  int lostConnection;
  /// debugIO value from config file
  int debugIoVal;
} busif;


typedef struct
{ /// servo index
  int servo;
  /// interface mode 0=servo, 1,2 = in 3=out (digital)
  int mode;
  /// is position updated
  int updated;
  /// minimum value
  int vmin;
  /// maximum servo value
  int vmax;
  /// center position
  int center;
  /// current position target in servo units
  int positionRef;
  /// current position of servo in servo units
  int position;
  /// name of servo channel
  char name[32];
} servo;


#define MAX_SERVO_CNT 12
/// variables herkulex interface
struct status
{ /** RHD variables for controller */
  /// servo position (if in mode 2)
  int servoRef; 
  /// servo setup configuration
  servo servos[MAX_SERVO_CNT];
  /// steeringangle variable
//  int camAngleRef;
  /// actual angle ref (pan tilt)
//  double camAngRefNew[2];
  /// status variable - error bytes and running-in position flag
//  int servoStatus;
  /// servo variable to report hard error
//  int servoError;
  /// servo position (10 bit) variable
  int servoPos;
  /// servo position limit (10 bit) variable
  int servoPosMin;
  /// servo position limit (10 bit) variable
  int servoPosMax;
  /// servo reset variable
//  int servoReset;
  /// ad input analog or digital
  int inIr;
  /// data rate for IR (and digital in) measurements
  int inIrRate;
  /// ad input analog or digital
  int inBat;
  /// index to cycle time (in us)
  int cycleTime;
  /// number of out channels, servo or digtal out
  int cntOut;
  /// number of in channels, analog or digital
  int cntIn;
  /// pololu error state variable
  int poError;
  //
} state;



#endif

