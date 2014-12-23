#include <stdint.h>
 /** \file usbiss.h
 *  \ingroup hwmodule
 *
 *   Interface for two herkulex DRS-0201 servoes for steering of ackermann type robot
 *   
 *
 * $Rev: 59 $
 * ¤Id$
 *
 *******************************************************************/

#ifndef HERKULEX_H
#define HERKULEX_H

extern int initXML(char *);
extern int periodic(int);
extern int terminate (void) ;

/// communication variables
struct
{ /** buffer for communication */
  #define MxBL 32
  unsigned char txBuf[MxBL];
  unsigned char rxBuf[MxBL];
  /** device file descriptor */
  int ttyDev;  //File descriptors
  #define MxDL 64
  char serialDev[MxDL];
  /// serial line baudrate
  int baudrate;
  /// stuck flag 0=not stuck, 1 = struk
  int stuck;
  /// is connection lost diurinr read or write operation
  int lostConnection;
  /// debugIO value from config file
  int debugIoVal;
} busif;


typedef struct
{ /// servo index
  int servo;
  /// model number and version
  int version;
  /// measured voltage by servo
  float volt;
  /// minimum value
  int vmin;
  /// maximum servo value
  int vmax;
  /// center position
  int center;
  /// servo control scale angle in radians per tick
  /// 0.325 degree per tick   0.325 * pi / 180
  float scale;
  /// servo nickname
  const char * name;
  /// servo speed (implemented on sd84 board)
  int speed;
  /// current position target (in degrees)
  float degreeRef;
  /// current position target in servo units
  int positionRef;
  /// current position of servo in servo units
  int position;
  /// servo temperature 
  float temp;
  /// error status bytes
  int errorStatus;
  /// error status bytes
  int hardError;
  /// int communication failure count
  int comFailCnt;
} servo;


#define MAX_SERVO_CNT 2
/// variables herkulex interface
struct
{ /** RHD variables for controller */
  /// emergency stop value index
  int emergencyswitch;
  /// emergency stop value 0=OK, 1=stop (free steering)
  int emergencystop;
  /// servo mode 2 = servo direct (servoRef), 3 = steering from turnAngleRef
  int servoMode;
  /// servo position (if in mode 2)
  int servoRef; 
  /// servo setup configuration
  servo servos[MAX_SERVO_CNT];
  /// command string for servo positioning
  uint8_t s_jog[MAX_SERVO_CNT * 4 + 1];
  /// steeringangle variable
  int steeringAngleRef;
  /// steering angle value
  float steerRef;
  /// should  status of full RAM be updated - generated
  int getRam;
  /// servo RAM status - if enabled
  int servoFullRAM[2];
  /// actual steering angle variable
  int servoTorque;
  /// actual steering angle variable
  int servoLed;
  /// actual steering angle variable
  int steeringAngle;
  /// status variable - error bytes and running-in position flag
  int servoStatus;
  /// servo variable to report hard error
  int servoError;
  /// servo input voltage variable
  int servoVolt;
  /// servo tick variable
  int servoTick;
  /// servo temp variable
  int servoTemp;
  /// servo position (10 bit) variable
  int servoPos;
  /// servo position change in 11.2ms
  int servoPosDif;
  /// servo PWM variable
  int servoPWM;
  /// position or velocity servo mode (should be 0 here)
  int servoPosVel;
  /// servo reset variable
  int servoReset;
  /// play time variable
  int servoPlayTime90degVal;
  /// dead zone (with no movement around target position)
  int servoDeadZoneVal;
  /// saturator control - offset outside dead zone
  int servoSaturatorOffsetVal;
  /// saturator control (force P control) off if 0, else PWM value for each position error tick / 256,
  /// i.e. value of 256 will add 1 PWM value for each position error + offset outside dead-zone
  int servoSaturatorSlopeVal;
  /// accRatio
  int servoAccRatioVal;
  /// maxAcc
  int servoMaxAccVal;
  /// max PWM
  int servoMaxPWMVal;
  /// min PWM 
  int servoMinPWMVal;
  //
  /// XML file setup constants
  /** robot wheel base - distance between rear wheels in meters  */
  float wheelBase;
  /** robot steer base - distance between rear axle and steering wheel axle in meters */
  float steerBase;
  /** front wheel base - distance between front wheels */
  float frontBase;
  //
} state;



#endif

