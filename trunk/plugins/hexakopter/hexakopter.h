/**
 * Interface for Mikrokopter flight controller board
 * for monitoring and control og platform
 *
 * $Rev: 94 $
 * $Id: hexakopter.h 94 2013-01-05 15:29:29Z jcan $
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

struct str_VersionInfo
{
  unsigned char SWMajor;
  unsigned char SWMinor;
  unsigned char ProtoMajor;
  unsigned char ProtoMinor;
  unsigned char SWPatch;
  unsigned char HardwareError[5];
};

struct str_ExternControl
{
    unsigned char Digital[2]; // not used in FlightCtrl
    unsigned char RemoteTasten; // not used in FlightCtrl
    signed char Nick; // offset to RC value
    signed char Roll; // offset to RC value
    signed char Gier; // YAW - offset to RC value
    unsigned char Gas; // 0 will stop motors
    signed char Hight; // height controller reference - if active
    unsigned char free2; // not used in FlightCtrl?
    unsigned char Frame; // this number is send back in confirmation to a control message (lag queue?)
    unsigned char Config; // set to 1 for external control (else 0)
};

/// structure for analog readings from flightcontroller
struct str_DebugOut
{
  unsigned char status[2];
  int16_t AngleNick; // 0
  int16_t AngleRoll; // 1
  int16_t AccNick;   // 2
  int16_t AccRoll;   // 3
  int16_t YawGyro;   // 4 operating radius
  int16_t HeightValue; // 5 FC-flags
  int16_t AccZ;        // 6 NC-flags
  int16_t Gas;         // 7 nick servo
  int16_t CompassValue; // 8 roll servo
  int16_t batVoltage;   // 9 GPS
  int16_t ReceiverLevel;// 10 Compas heading
  int16_t GyroCompass;  // 11 Gyro heading
  int16_t Motor1;       // 12 SPI error
  int16_t Motor2;       // 13 SPI OK
  int16_t Motor3;       // 14 I2C error
  int16_t Motor4;       // 15 I2C OK
  int16_t Motor5;       // 16 -
  int16_t Motor6;       // 17 -
  int16_t Motor7;       // 18 -
  int16_t Motor8;       // 19 -
  int16_t Servo;        // 20 earth magnet %
  int16_t Hovergas;     // 21 Z-speed
  int16_t Current;      // 22 N_speed
  int16_t Capacity;     // 23 E_speed
  int16_t HeightSetpoint;// 24 M-X
  int16_t analog25;      // 25 M-Y
  int16_t CPUOverLoad;   // 26 M-Z
  int16_t CompassSetpoint;// 27
  int16_t I2C_Error;      // 28
  int16_t BL_Limit;       // 29
  int16_t GPS_Nick;       // 30
  int16_t GPS_Roll;       // 31
  int16_t buffer[10];
};

struct str_Data3D
{
   int16_t  Winkel[3]; // nick, roll, compass in 0.1°
   signed char Centroid[3];
   signed char reserve[5];
   double rateTime;
   int dataCnt;
   double statDataTime;
   int statDataCnt;
};


struct
{ // RHD variables
  // write
  int   nick;
  int   roll;
  int   yaw;
  int   gasref;
  int   height;
  // status of external ctrl;
  int externCtrlGot;
  // read
  // version of flight controller
  int versionFC;
  // status
  int   Config; // extern control
  // debug status
  int AngleNick;
  int AngleRoll;
  int AccNick;
  int AccRoll;
  int YawGyro;
  int HeightValue;
  int AccZ;
  int gas;
  int CompassValue;
  int batVoltage;
  int ReceiverLevel;
  int GyroCompass;
  int Motor;
  int Servo;
  int Hovergas;
  int Current;
  int Capacity;
  int HeightSetpoint;
  int CPUOverLoad;
  int CompassSetpoint;
  int I2C_Error;
  int BL_Limit;
  int GPS_Nick;
  int GPS_Roll;
  int statRate;
  // 3D pose
  int pose;
  int poseRate;  
} var;

#endif
