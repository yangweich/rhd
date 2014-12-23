 /** \file sableandmagenc.h
 *  \ingroup hwmodule
 *
 * Interface for sabertooth motor controller and AS4050 magnetic encoders
 * through an arduino nano velocity controller
 *
 * $Rev: 453 $
 * $Id: fieldsteer.h 453 2014-03-21 13:40:00Z jcan $
 *
 *******************************************************************/

#ifndef SAME_H
#define SAME_H

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

Busif busif1, busif2;

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
  /// current position (in degrees * 10)
  //int value;
  /// actual position in radians
  float angleRad;
  /// commanded position in degrees
  float degreeRef;
  /// position ref in servo units
  int positionRef;
} servo;

#define SERVO_COUNT 2

struct controlStateStruct
{ // steering and drive
  servo servos[SERVO_COUNT];
  /// steering angle set point in radians
  float steeringangleref;
  /** steering base distance (between front and rear wheels) in meters */
  float steerBase;
  /** distance between front wheels - in meter */
  float frontBase;
  /** robot wheel base - distance between rear wheels in meters  */
  float wheelBase;
  /** encoder tics per revolution  */
  int encTicsPerRev;
  /** gear from encoder to wheel */
  float gearRatio;
  /** wheel diameter for driving wheel - meter */
  float wheelDiameter;
  /// joystick override is in force
  int joyOverride;
  /// index to joy override info
  int varJoyOverride;
  /// index to steering angle ref varable
  int varSteeringAngleRef;
  /// index to steering angle read varable
  int varSteeringAngle;
  /// index to encoder left, right, tilt
  int varEncFront;
  /// index to encoder back
  int varEncLeft;
  /// index to encoder back
  int varEncRight;
  /// index to encoder error count left, right, tilt
  int varEncFrontErr;
  /// state of magnet detector 0 = OK, 1=moving away, 2=moving closer, 3=bad
  int varEncMagnetState;
  /// index to speed set point
  int varSpeedRef;
  /// index to speed set point
  int varSpeedRefLR;
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
  /// debug variable for dynamixel communication:
  /// dxlErrSuccess, dxlErrTimeout, dxlErrCurrupt, dxlRxTimeoutCnt,
  /// dxlRxTimeoutCntMax, dxlBusUseCnt, dxlBusAlreadyInUseCnt
  int varDynamixelErr;

  /// commanded wheel speed
  float speedRefLeft;
  /// commanded wheel speed
  float speedRefRight;
  /// last commanded velocity send to motor controller - left motor [tics/10ms]
  int velLeftRef;
  /// last commanded velocity send to motor controller - right motor [tics/10ms]
  int velRightRef;
  /// actual commanded wheel speed in m/s
  //float speedLeft;
  /// actual commanded wheel speed in m/s
  //float speedRight;

  /// index emergency stop pushed
  int varEmergStopPushed;
  /// index to safety stop flag - no write master or switch pushed.
  int varSafetyStop;
  /// is speed set to 0,0 for safety reasons (switch or no write master)
  int safetyStop;
  /// offset to be added to motor value to make zero speed
  int zeroSpeedOffset[2];
  /// enable communication with front wheel encoders
  int enableFrontEncoder;
  /// enable communication with dynamixel steering
  int enableDynamixel;
} sast;

#endif

