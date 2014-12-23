 /** \file sableandmagenc.h
 *  \ingroup hwmodule
 *
 * Interface for sabertooth motor controller and AS4050 magnetic encoders
 * through an arduino nano velocity controller
 *
 * $Rev: 59 $
 * $Id: saberandmagenc.h 59 2012-10-21 06:25:02Z jcan $
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
{ /// joystick axis number
  int axis;
  /// sd84 servo index
  int servo;
  /// minimum value
  int vmin;
  /// maximum servo value
  int vmax;
  /// center position
  int center;
  /// invert direction (0 (false) or 1 (true))
  int invert;
  /// servo control scale
  int scale;
  /// servo nickname
  const char * name;
  /// servo speed (implemented on sd84 board)
  int speed;
  /// current position (if maintained)
  int value;
} servo;

struct controlStateStruct
{ // steering and drive
  servo servos[2];
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
  /// index to encoder left
  int varEncLeft;
  /// encoder right
  int varEncRight;
  /// index to speed status for left motor mm/s [left, right]
  int varSpeed;
  /// turnrate [deg*1000/sec]
  //int varTurnrate;
  /// Reset all values to zero (speed and encoders)
  //int varReset;
  /// index to speed set point
  int varSpeedRef;
  /// index emergency stop pushed
  int varEmergStopPushed;
  /// index to start enabled
  int varStartEnabled;
  /// index to velocity mode
  int varVelMode, varVelModeIs;
  /// index to PWM variables
  int varVelPWM;
  int velPWMisL, velPWMisR;
  /// requested vel mode
  int velMode;
  int velModeIs;
  /// feed velocity forward gain (0..128) -- see firmware
  int velCtrlFF;
  int velCtrlFFis;
  /// feed velocity proportional gain -- see firmware
  int velCtrlP;
  int velCtrlPis;
  /// index to battery voltage - not implemented
  //int varBatt;
  /// state of magnet detector 0 = OK, 1=moving away, 2=moving closer, 3=bad
  int varMagnetState;
  /// commanded wheel speed
  float speedRefLeft;
  /// commanded wheel speed
  float speedRefRight;
  /// last commanded velocity send to motor controller - left motor [tics/10ms]
  int velLeftRef;
  /// last commanded velocity send to motor controller - right motor [tics/10ms]
  int velRightRef;
  /// actual commanded wheel speed in m/s
  float speedLeft;
  /// actual commanded wheel speed in m/s
  float speedRight;
  /// wheel speed in m/s measured by encoder
  float speedLeftEnc;
  /// wheel speed in m/s measured by encoder
  float speedRightEnc;
  /// encoder velocity per 10ms for a robot velocity in m/s
  float robVel2EncVel;
  /// pwm control
  float robVel2PWMVel;
  /// datarate from controller - unit is 10 ms
  int datarate;
  int datarateIs;
  /// PWM offset to get zero velocity -50..50 us
  int zeroPWMOffsetR, zeroPWMOffsetL;
  int zeroPWMOffsetRis, zeroPWMOffsetLis;
  /// are we stopped for safety?
  int safetyStop;
} sast;

#endif

