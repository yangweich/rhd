 /** \file usbiss.h
 *  \ingroup hwmodule
 *
 *   Interface for i2c bus from USBi2c converter
 *   and connected to MD25 motor controller unit
 *
 * $Rev: 338 $
 * $Id: rpii2c.h 338 2013-12-15 17:22:18Z jcan $
 *
 *******************************************************************/

#ifndef USBISS_H
#define USBISS_H

extern int initXML(char *);
extern int periodic(int);
extern int terminate (void) ;

/// communication variables
struct
{ /** buffer for communication */
  #define MxBL 32
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
// struct
// { /** variables for i2c bus */
//   int varI2sSerial;
//   /// version info
//   int varI2sVersion;
//   /// i2c bus id for bus controller
//   int busid;
// } usbiss;

/// variables for MD25 controller
struct
{ /** RHD variables for motor controller */
  int busid;
  /// encoder left
  int varEnc1;
  /// encoder right
  int varEnc2;
  /// speed write for motor 1
//   int varM1SpeedR;
//   /// speed read for motor 1
//   int varM2SpeedR;
//   /// speed write for motor 2
//   int varM1SpeedW;
//   /// speed read for motor 2
//   int varM2SpeedW;
  /// battry voltage in deci-Volts  124 is 12.4 V
  int varBatt;
  /// motor current for motor 1
  int varM1Curr;
  /// motor current for motor 2
  int varM2Curr;
  /// Reset all values to zero (speed and encoders)
  int varReset;
  /** actual acceleration setting 10 = 400 tics a sec, 1 = 40 values a sec
    * value range 1..10 */
  int varAccR;
  /** actual acceleration setting 10 = 400 tics a sec, 1 = 40 values a sec
    * value range 1..10 */
  int varAccW;
  /// speed set point
  int speedref;
  /// steering angle - created by steering controller plug-in
  int varSteeringAngleRef;
  /** robot wheel base - distance between rear wheels in meters  */
  float wheelBase;
  /** robot steer base - distance between rear axle and steering wheel axle in meters */
  float steerBase;
  /** front wheel base - distance between front wheels */
  float frontBase;
  /**
   * acceleration value set last time */
  int accOld;
  /// tick time lock
  pthread_mutex_t mLock; // pthread_mutex_lock

} md25;

struct
{ // adress on i2c bus (IMU9150 has 0x68 as default
  int busid;
  /// accelerometer values (in m/s2) [x,y,z]
  int varAcc;
  /// gyro values
  int varGyro;
  /// compas values
//  int varMag;
  /// int temperatur value
  int varTemp;
} imu;


#endif

