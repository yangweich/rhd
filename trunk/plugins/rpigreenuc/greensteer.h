 /** \file sableandmagenc.h
 *  \ingroup hwmodule
 *
 * Interface for sabertooth motor controller and AS4050 magnetic encoders
 * through an arduino nano velocity controller
 *
 * $Rev: 375 $
 * $Id: greensteer.h 375 2013-12-29 14:05:12Z jcan $
 *
 *******************************************************************/

#ifndef GREENSTEER_H
#define GREENSTEER_H

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
  /// servo nickname loaded value
#define SERVO_NAME_MAX 32
  char nameStr[SERVO_NAME_MAX];
  /// servo speed (if implemented)
  //int speed;
  /// current position (if maintained)
  int value;
  /// commanded position in degrees
  float degreeRef;
  /// position ref in servo units
  int positionRef;
} Servo;

#define SERVO_COUNT 4

struct controlStateStruct
{ // steering and drive
  Servo servos[SERVO_COUNT];
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
  /// index to steering angle varable
  int varSteeringAngle;
  /// index to steering angle ref varable
//  int varSteeringAngle;
  /// index to speed set point
//  int varCamPanTiltRef;
  /// actual pan-tilt
//  int varCamPanTilt;
  /// current servo (commanded) position
  int varServoIntRef;
  /// index to speed set point
  int varPortC;
  /// index to IR sensor values
  int varIRVal;
  /// index to bumper values
  int varBumper;
  /// index to safety stop flag - no write master or switch pushed.
//  int varSafetyStop;
  /// is speed set to 0,0 for safety reasons (switch or no write master)
//  int safetyStop;

} sast;

#endif

