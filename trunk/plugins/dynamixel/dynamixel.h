 /** \file sableandmagenc.h
 *  \ingroup hwmodule
 *
 * Interface for dynamixel servo controller - as in dxlctrl subdirectory
 * servo ID and communication speed is hard coded in that controller
 * i.e. must be set to match dynamixel setup wizard or changed in controller firmware
 *
 * $Rev: 281 $
 * $Id: dynamixel.h 281 2013-10-20 15:33:20Z jcan $
 *
 *******************************************************************/

#ifndef SAME_H
#define SAME_H

/** RHD calls initXML once just after the plugin is loaded
    used to read configuration file and set initial state
    and open and configure connection to device */
extern int initXML(char *);
/**
 * RHD calls this function every time write variables may have been updated
 * normally every 10ms (RHD sample time)
 * \param RHDtick is RHD sample count. */
extern int periodic(int RHDtick);
/**
 * Called by RHD when RHD is shutting down.
 * Should be used to close files and connections */
extern int terminate (void) ; //No shutdown function

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

/// structures used by this plug-in

/// communication variables
struct
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
} busif;

typedef struct
{/// servo ID
  int servo;
  /// minimum value
  int vmin;
  /// maximum servo value
  int vmax;
  /// center position (straight ahead)
  int center;
  /// invert direction (0 (false) or 1 (true))
  //int invert;
  /// deadzone - when servo goes to idle - if implemented
  int deadzone;
  /// servo control scale - from degrees to servo units
  float scale;
  /// servo nickname
  const char * name;
  /// servo speed (if implemented)
  //int speed;
  /// current position (if maintained)
  int value;
  /// commanded position in degrees
  float degreeRef;
  /// position ref in servo units
  int positionRef;
  /// position in rhd units
  int positionRefRhd;
  /// number of status updates (from servo)
  int updCnt;
} servo;

#define SERVO_COUNT_MAX 10

/// servo state variables
struct controlStateStruct
{ 
  /// number of servos configured
  int servoCnt;
  /// servo info array
  servo servos[SERVO_COUNT_MAX];
  /// joystick override is in force
  int joyOverride;
  /// index to joy override info
  int varJoyOverride;
  /// index to steering angle ref varable
  int varServoRef;
  /// index to speed set point
  int varSpeedRef;
  /// index to present servo position
  int varServoPos;
  /// index to present servo speed
  int varServoVel;
  /// index to present servo load (force)
  int varServoLoad;
  /// index to battery voltage
  int varServoVolt;
  /// index to present servo temperature
  int varServoTemp;
  /// servo communication error count
  int varServoComErrCnt;
  int varServoIntPos;
  int varServoIntRef;
  int varServoUpdCnt;
} sast;

#endif

