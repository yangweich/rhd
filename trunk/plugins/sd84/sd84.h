 /** \file sd84.h
 *  \ingroup hwmodule
 *
 *   Interface for sd84 multi IO board
 *
 * $Rev: 59 $
 * $Id: sd84.h 59 2012-10-21 06:25:02Z jcan $
 *
 *******************************************************************/

#ifndef SD84_H
#define SD84_H

extern int initXML(char *);
extern int periodic(int);
extern int terminate (void) ; //No shutdown function

/// communication variables
struct
{ /** buffer for communication */
  #define MxBL 128
  unsigned char rxBuf[MxBL];
  /** device file descriptor */
  int ttyDev;  //File descriptors
  #define MxDL 64
  char serialDev[MxDL];
  /// stuck flag 0=not stuck, 1 = struk
  int stuck;
  /// if connection is lost - restart?
  int lostConnection;
  /// cpu version
  int cpuVersion[4];
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

typedef struct
{ /// joystick button index
  int butIdx;
  /// out value index
  int outIdx;
  /// button pressed time
  struct timeval onTime;
  /// last button value
  int lastVal;
  /// is it on
  int val;
  /// debug name
  const char * name;
} button;

/// variables for MD25 controller
struct
{ /** RHD variables for SD84 controller */
  /// version info - all CPUs
  int varCpuVer;
  /// AD values
  int varAD;
  /// IN values
  int varIn;
  /// out values
  int varOut;
  /// out values - in a rhd-read copy
  int varOutr;
  /// servo values
  //int varServo;
  /// servo values read values
  //int varServor;
  /// joystick control of steering
  int varJoySteer;
  /// joystick control of arm
  int varJoyArm;
  /// last servo value
  int servoOld[24];
  /// servo speed - 0 is fast, 1 is slow (1 step per 20 ms) 100 is 100 step/20ms
  int servoSpeed[24];
  /// joystick axes
  int joyaxes;
  /// joystick buttons
  int joybuttons;
  /// joystick override button
  int joyOverrideBut;
  /// tick time lock
  pthread_mutex_t mLock; // pthread_mutex_lock
} sd84;

struct
{ // joystick control of arm
#define ARM_AXIS_CNT 6
  /// rhd variables to control arm servos
  int varAxis;
  /// rhd with current servo value
  int varAxisr;
  /**
   * current servo position
   * 0 = turn at arm base -
   * 1 = lift (shoulder up-down)
   * 2 = elbow (up-down)
   * 3 = hand rotate
   * 4 = hand up-down
   * 5 = gripper open-close
   */
  servo servos[ARM_AXIS_CNT];
  /// on button number on joystick
  button butOn;
  /// laser button
  button butLaser;
  /// arm power button
  button butPower;
} arm;

struct
{ // steering
  servo servos[2];
  /// steering angle "measured"
  int steeringangle;
  /// steering angle set point
  int steeringangleref;
  /// return steering angle for motor plugin.
  int steeringChangedTo;
  /// actual steering angle
  int steeringAngle;
  /** scale from angle in radians to servo pulse width in us,
   * i.e. a change of 100 us pulse width may correspond to 0.1 radians, then
   * the steer scale should be 1000.0 */
  double steerScale;
  /** the pulse width offset (in signed us) resulting in zero steering angle - should be close to 0 */
  int steerLeftZero;
  /** the pulse width offset (in signed us) resulting in zero steering angle - should be close to 0 */
  int steerRightZero;
  /// current steering angle in radians
  float steerRef;
  /** steering base distance in meters */
  double steerBase;
  /** distance between front wheels - in meter */
  double frontBase;
  /// position (scaled) of front wheels
  int varSteerr;
} steer;

#endif

