 /** \file sableandmagenc.h
 *  \ingroup hwmodule
 *
 * Interface for sabertooth motor controller and AS4050 magnetic encoders
 * through an arduino nano velocity controller
 *
 * $Rev: 354 $
 * $Id: greensteer.h 354 2013-12-21 16:41:50Z jcan $
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
  /// is servo enables
  int enabled;
  /// idle count
  int idleCnt;
} Servo;

#define SERVO_COUNT 8

struct controlStateStruct
{ // steering and drive
  Servo servos[SERVO_COUNT];
  /// steering angle set point in radians
  /// current servo (commanded) position
  int varServoAngRef;
  int varServoIntRef;
  int varServoAng;
  int varServoEnable;
  /// index to IR sensor values
  int varIRVal;
} sast;

#endif

