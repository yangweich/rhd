/**
 * Interface for Mikrokopter flight controller board
 * for monitoring and control og platform
 *
 * $Rev: 108 $
 * $Id: kopterctrl.h 108 2013-01-20 16:34:12Z jcan $
 *
 *******************************************************************/
#ifndef RS232HEXA_H
#define RS232HEXA_H
#include <sys/types.h>

/// RHD functions
extern int initXML(char *);
extern int periodic(int);
extern int terminate (void);

/// main flight controller
#define FC_ADDRESS 1
/// navigation controller
#define NC_ADDRESS 2
/// magnetometer controller
#define MK3MAG_ADDRESS 3
/// brushless motor controller
#define BL_CTRL_ADDRESS 5

struct
{ /** device file descriptor */
  char txBuffer[100];
  int ttyDev;  //File descriptors
  #define MxDL 64
  char devName[MxDL];
  /// serial speed
  int baudrate;
  /// is connection OK
  int lostConnection;
} devif;

//#define MBL 50



struct
{ // RHD variables
  // measurements
  int kopterPose;
  int tanRoll, tanNick;
  // control values
  int nickRef; /// kopter nick (tilt) reference angle (+/- 128)
  int rollRef; /// kopter roll angle (+/- 128)
  int yawRef;  /// kopter yaw angle speed reference (+/- 128)
  int gasref;  /// thrust reference (0..256)
  int inCtrl;  /// nick,roll,yaw,trust,height,control,frame
  // controller parameters
  float pd[2];       /// PD controller parameters
  struct timeval ts; /// sample time
  int pdR;          /// rhd value read of PD parameters
  int pdW;          /// rhd value write of PD parameters
  float eroll[2], enick[2], croll[2], cnick[2];
  int wrollctrl; /// index to RHD variable - debug control value (degrees * 10)
  int wnickctrl; /// index to RHD variable - debug control value (degrees * 10)
  // initialization
  int initForce;
  int initCtrl;
} var;

#endif
