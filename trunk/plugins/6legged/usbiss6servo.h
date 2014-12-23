
//extern params p; 
/** \file usbiss.h
 *  \ingroup hwmodule
 *
 *   Interface for i2c bus from USBi2c converter
 *   and connected to MD25 motor controller unit
 *
 * $Rev: 59 $
 * ¤Id$
 *
 *******************************************************************/

#include <semaphore.h>
 
#ifndef USBISS_H
#define USBISS_H

extern int initXML(char *);
extern int periodic(int);
extern int terminate (void) ;

#define MAX_SERVO_CNT 24
#define MAX_NAME_CNT  16

/// communication variables
struct
{ /** buffer for communication */
  #define MxBL 128
  char txBuf[MxBL];
  unsigned char rxBuf[MxBL];
  /** device file descriptor */
  int ttyDev;  //File descriptors
  #define MxDL 64
  char serialDev[MxDL];
  /// stuck flag 0=not stuck, 1 = struk
  int stuck;
  /// version number 0: ID=7, 1: firmware, 2: current mode
  int version[3];
  /// serial number
  int serial;
  /// is connection lost diurinr read or write operation
  int lostConnection;
} busif;

/// variables related to bus control
struct
{ /** variables for usb to i2c bus */
  /// int varI2sSerial;
  /// version info
  int varI2sVersion;
  /// i2c bus id for bus controller
  int busid;
} usbiss;

typedef struct 
{ // status for each servo
  /// servo ID on i2c bus, 0 is no servo
  unsigned char id;
  /// servo name
  char name[MAX_NAME_CNT];
  /// alive
  int alive;
  /// pwm enabled
  // int pwmEnabled;
  /// position ref 
  int posRef;
  /// velocity ref
  int velRef;
  /// PD controller P value
  int p;
  /// PD controller D value
  int d;  
  /// current position
  int pos;
  /// current velocity
  int16_t vel;
  /// current current (force)
  int current;
  /// current PWM for direction a
  int pwma;
  /// current PWM for direction a
  int pwmb;
  /// minimum allowed position
  int posMin;
  /// maximum allowed position
  int posMax;
  /// scale factor from RHD units to servo units (position)
  float scale;
  /// offsert in servo units from rhd to servo, i.e. servoValue =  rhdValue * scale + offset
  int offset;
} UServo;
/// variables for MD25 controller
struct
{ /** RHD variables for motor controller */
  int busid;
  // servo read or write variables
  /// servo ID adress on i2c bus
  int varServoID;
  // is communication with servo alive
  int varAlive;
  /// is servo enabled (else idle)
  int varCmd;
  /// target position seen by robot
  int varPosRef;
  /// target velocity seen by robot
  int varVelRef;
  /// proportional gain
  int varCtrlP;
  /// differential gain
  int varCtrlD;
  /// servo current position
  int varPos;
  /// servo current velocity
  int varVel;
  /// servo current (force)
  int varCurrent;
  /// servo PWM to give direction
  /// should probably be converted to signed value
  int varPWMa;
  int varPWMb;
  /// debug servo banka read
  int varBank[3];
  // servo version bytes
  int varServoVersion;
  // pthread_mutex_t mLock; // pthread_mutex_lock
  sem_t mLock;
  //
  UServo servos[MAX_SERVO_CNT];
  //
  int servosCnt;

} oservo;



#endif

