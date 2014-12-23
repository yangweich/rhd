 /** \file fieldsteer.c
 *  \ingroup hwmodule
 *
 * Interface for field robot steering and motor control
 *
 *
 *
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 453 $:"
 #define DATE             "$Date: 2014-03-21 14:40:00 +0100 (Fri, 21 Mar 2014) $:"
 #define ID               "$Id: fieldsteer.c 453 2014-03-21 13:40:00Z jcan $"
 #define PLUGINNAME        "FSteer"
/***************************************************************************/

#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <linux/serial.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <expat.h>
#include <poll.h>
#include <math.h>
#include <stdint.h>

//RHD Core headers
#include <rhd.h>
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "fieldsteer.h"

#define LS_READ_BYTES 20
/// number of AD channals
#define AD_COUNT  13

///Struct for shared parse data
typedef struct  {
    int depth; // current XML tag level
    int skip;  // skip from this level up
    char enable;
    char found;
  } parseInfo;

/* prototypes */
void createVariables();
int init(void);
/** poll to see if a read would not block.
 * \param fd is the device number
 * \param msTimeout is the maximum wait time
 * \returns 0 is no data is available and 1 if at least 1 byte is available */
int pollDeviceRx(int fd, int msTimeout);
/** run thread for rx task */
void * controlio_task_1(void *);
void * controlio_task_2(void *);
/**
 * get index to a named variable - from another plugin
 * \param type is either 'r' or 'w'
 * \param name is the name of the variable
 * \returns -1 if the variable is not found, else the index (positive or zero) of the variable. */
int getDatabaseVariable(char type, const char * name);
/// Parsing function for start tag
void XMLCALL lsStartTag(void *, const char *, const char **);
/// parsing function for end tag
void XMLCALL lsEndTag(void *, const char *);
/// round a float to an integer
int roundi(const float v)
{ // round to closest integer
  if (v > 0)
    return (int)(v + 0.5);
  else
    return (int)(v - 0.5);
}
/** catch writes to a non existing file - allowed in debug mode
 * \param dev is the device
 * \param buf is the byte buffer with data to send
 * \param bufCnt is the number of bytes to send
 * \returns the number of bytes send, 0 if failed to send. */
ssize_t secure2Write(int dev, const void *buf, ssize_t txLen);
/**
 * calls secure2Write and supplies txlen from length of string */
inline ssize_t secure3Write(int dev, const void *buf)
{
  return secure2Write(dev, buf, strlen(buf));
}

/**
 * limit integer to this range.
 * \param value is the value to be tested
 * \param min is the minimum value returned
 * \param max is the maximum value returned
 * \returns value except if it exceeds minimum or maximum, if so it returns the maximum or minimum value */
int limitInt(int value, int min, int max);

/******** Global variables *************/

/// variables related to thread execution
typedef struct
{ /** is receive thread running */
  int running;
  /** Trigger polling of data from units - when set to 1 */
  int startNewRxCycle;
  /** name of serial device */
  /** thread handle and attributes */
  pthread_t controlio_thread;
} Rxtask;

Rxtask rxtask1, rxtask2;

int tick = 0;
/// old steering angle value - in radians
double sacOld = -4.0;
int debugFlag = 0;
/** flags for updating of write variables */
int settingUpd = 1, steerUpd = 1;
struct timeval tickTime;
/**
 * left and right motor variables found?
 * */
int leftRightSpeedRefVarOK;
/**
 * Set servo position if changed, and send to servo
 * \param angleLeft is desired wheel angle in radians - zero is straight ahead
 * \param angleRight is desired wheel angle in radians
 */
int setServos(float angleLeft, float angleRight);


// debug
FILE * freLog = NULL;
struct timeval logtime;
// debug end

////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int rhdTick)
{
  int returnValue = rxtask1.running && rxtask2.running;
  int speedSet = 0;
#define STR_LEN 30
  char s[STR_LEN];
// #define MAX_SPEED_STR_CNT 32
//   char cmdStr[MAX_SPEED_STR_CNT];
  //gettimeofday(&tickTime, NULL);
  rxtask1.startNewRxCycle += 1;
  rxtask2.startNewRxCycle += 1;
  tick = rhdTick;
  if (tick == 1)
  { // connect to joystick - if any
    sast.varJoyOverride = getDatabaseVariable('r', "joyoverride");
    printf(PLUGINNAME " joystick override  (r) variable index=%d\n", sast.varJoyOverride);
    // get write variable for speed compensation
  }
  if (sast.varJoyOverride >= 0)
  { // remote control is available
    sast.joyOverride = getReadVariable(sast.varJoyOverride, 0);
  }
  if (isUpdated('w', sast.varSteeringAngleRef))
  { // steering changed - calculate new angle for left and right wheel
    float tanSac, ofs, sal, sar;
    float steerRef = getWriteVariable(sast.varSteeringAngleRef, 0) * M_PI / 1800.0;
    // reset update flag
    steerUpd = 0;
    // get tan to steering angle, and approximate, if >= 90 deg
    if (fabs(steerRef) < (M_PI/2.0 - 1e-5))
      tanSac = tan(steerRef);
    else // use +-90 degrees
      tanSac = steerRef * 1e10;
    // sal = atan2(steerBase, turnRadius - frontBase/2)
    // sar = atan2(steerBase, turnRadius + frontBase/2)
    // turnRadius = steerBase / tanSac
    // multiply with tanSac and divide by steerBase gives
    // sal = atan2(tanSac, 1 - frontBase/2 * tanSac / steerBase)
    // sar = atan2(tanSac, 1 + frontBase/2 * tanSac / steerBase)
    ofs = sast.frontBase * tanSac / 2.0 / sast.steerBase;
    // find angle in radians for each wheel
    sal = atan2(tanSac, 1.0 - ofs);
    sar = atan2(tanSac, 1.0 + ofs);
    // implement new servo positions
    //if (tick % 2 == 1)
    setServos(sal, sar);
    sacOld = steerRef;
  }
  //
  if (isUpdated('w', sast.varSteeringAngleRef) || isUpdated('w', sast.varSpeedRef))
  { // convert steering angle to wheel speed, as steering angle or speed is changed
    int32_t speedInt = getWriteVariable(sast.varSpeedRef, 0);
    float speed = speedInt / 1000.0; // in m/s
    // get steering angle (for a cycle model steering) in deci degrees - convert to radians
    double steerAngle = -getWriteVariable(sast.varSteeringAngleRef, 0) * M_PI / 180.0 / 10.0;
    //symTableElement * symw = getSymtable('w');
    // reset write variables
    //printf(PLUGINNAME ": got speed %gm/s, %gdeg\n", speed, steerAngle * 180.0 / M_PI);
    //    symw[state.varSpeedRef].updated = 0;
    //    symw[state.varSteeringAngleRef].updated = 0;
    if (speedInt != 0)
    { // calculate wheel speed (left and right) at this steering angle and speedref
      // ensure steering angle is less than 90 deg, i.e. tan(steeringAngle) < infinity
      if (steerAngle > M_PI / 2.0 - 0.001)
        // close to 90 deg steering angle
        steerAngle = M_PI / 2.0 - 0.001;
      else if (steerAngle < -M_PI / 2.0 + 0.001)
        // close to 90 deg steering angle
        steerAngle = -M_PI / 2.0 + 0.001;
      // find matching driving wheel speed
      // valid tangent value
      float ct = fabs(tan(steerAngle) / sast.steerBase);
      float ofs = ct * sast.wheelBase;
      // find speed for outher wheel
      float speedo = speed * (2.0 + ofs) / (2.0 / cos(steerAngle) + sast.frontBase * ct);
      if (steerAngle >= 0)  // turning left
      {
        sast.speedRefRight = speedo;
        sast.speedRefLeft = speedo*(2.0 - ofs)/(2.0 + ofs);
      }
      else // turning right - reduce speed of right wheel
      {
        sast.speedRefLeft = speedo;
        sast.speedRefRight = speedo*(2.0 - ofs)/(2.0 + ofs);
      }
    }
    else
    { // stop
      sast.speedRefLeft = 0.0;
      sast.speedRefRight = 0.0;
    }
    speedSet = 1;
  }
  if ((!isMasterAlive && !sast.joyOverride) ||
       getReadVariable(sast.varEmergStopPushed, 0))
  { // no write allowed client - set speed to 0
    sast.speedRefLeft = 0.0;
    sast.speedRefRight = 0.0;
    if (!sast.safetyStop)
    {
      usleep(4000);
      secure3Write(busif2.ttyDev, "E 0 0\r\n");
      usleep(4000);
      printf(PLUGINNAME ":: safety stop - no rhd clients and no joystick control\n");
      printf(PLUGINNAME ":: safety stop - sending E 0 0 to disable motors\n");
      sast.safetyStop = 1;
    }
    speedSet = 1;
  }
  else if (sast.safetyStop)
  { // reenable motors
    printf(PLUGINNAME ":: safety stop over - sending E 1 1 to motors\n");
    secure3Write(busif2.ttyDev, "E 1 1\r\n");
    usleep(2000);
    sast.safetyStop = 0;
  }
  if (speedSet)
  {
    setVariable(sast.varSpeedRefLR, 0, roundi(sast.speedRefLeft * 1000));
    setVariable(sast.varSpeedRefLR, 1, roundi(sast.speedRefRight * 1000));
  }
  setVariable(sast.varSafetyStop, 0, sast.safetyStop);
  //
  // write motor control string and send to XMega on device 2
  // debug - half spped for motor 0 - and negative sign, until fixed in interface board?
  snprintf(s, STR_LEN, "M %d %d\r\n",
           - roundi(sast.speedRefLeft * 1000) + sast.zeroSpeedOffset[0], // 430), //1000),
           - roundi(sast.speedRefRight * 1000) + sast.zeroSpeedOffset[1]);
  secure3Write(busif2.ttyDev, s);
  // debug
  gettimeofday(&logtime, NULL);
  if (freLog != NULL)
     fprintf(freLog, "%lu.%06lu if2 send %s", logtime.tv_sec, logtime.tv_usec, s);
  // end debug
  // request steer status
  if (tick % 2 == 0)
  {
    secure3Write(busif1.ttyDev, "s\n");
    // secureWrite(busif1.ttyDev, "s\n", 2);
    //ssize_t m;
    //m = write(busif1.ttyDev, "s\n", 2);
    // debug
    gettimeofday(&logtime, NULL);
    if (freLog != NULL)
      fprintf(freLog, "%lu.%06lu if1 send s\n", logtime.tv_sec, logtime.tv_usec);
    // end debug
  }
  return returnValue;
}

//////////////////////////////////////////////////////
/**
 * Create variables for the usb to controlio converter itself */

void createVariables()
{ // create variables in RHD
  sast.varEncLeft = createVariable('r',1,"encl"); // encoder value back wheels
  sast.varEncRight = createVariable('r',1,"encr");
  sast.varEncFront = createVariable('r',3,"encfront"); // encoder value front wheels (left, right,tilt)
  sast.varEncMagnetState = createVariable('r', 3, "encfrontmagnet"); // state of encoder magnet (left, right, tilt), val: 0=OK, 1=moving away, 2=moving closer, 3= bad
  sast.varEncFrontErr = createVariable('r', 3, "encfronterr"); // error counter (parity error)
//  createVariable('w', 1, "dummy");
  /// in degrees * 10
  sast.varSteeringAngleRef  = createVariable('w', 1, "steeringangleref");
  sast.varSteeringAngle  = createVariable('r', 1, "steeringangle");
  sast.varSpeedRef = createVariable('w', 1, "speedref"); // actual speed in mm/s [left right)
  sast.varSpeedRefLR = createVariable('r', 2, "speedrefLR"); // actual speed in mm/s [left right)
  sast.varSafetyStop = createVariable('r', 1, "safetystop"); // 1 if stopped for safety - no master or switch pushed
  sast.varEmergStopPushed = createVariable('r', 1, "emergencyswitch"); // 1 if emergenct stop pushed
  /// index to present servo position - in degrees * 10 relative to center
  sast.varServoPos = createVariable('r', SERVO_COUNT, "steerPos");
  /// index to present servo speed
  sast.varServoVel = createVariable('r', SERVO_COUNT, "steerVel");
  /// index to present servo load (force)
  sast.varServoLoad = createVariable('r', SERVO_COUNT, "steerLoad");
  /// index to battery voltage
  sast.varServoVolt = createVariable('r', SERVO_COUNT, "steerVolt");
  /// index to present servo temperature
  sast.varServoTemp = createVariable('r', SERVO_COUNT, "steerTemp");
  /// servo communication error count
  sast.varServoComErrCnt = createVariable('r', SERVO_COUNT, "steerServoComErr");
  sast.varServoIntRef = createVariable('r', SERVO_COUNT, "steerServoIntRef");
  sast.varServoIntPos = createVariable('r', SERVO_COUNT, "steerServoIntPos");
  sast.varDynamixelErr = createVariable('r', 7, "dynamixelComErr");
  //
  // initialize other variables
  sast.speedRefLeft = 0.0;
  sast.speedRefRight = 0.0;
  sast.joyOverride = 0;
  printf(PLUGINNAME ": has created its read and write variables\n");
}

////////////////////////////////////////////////////////////////

int terminate(void)
{
  rxtask1.running = 0;
  rxtask2.running = 0;
  printf(PLUGINNAME ": stopping ... \n");
  // stop motors
  setServos(0, 0);
  usleep(10000);
  secure3Write(busif2.ttyDev, "E 0 0\r\n");
  printf(PLUGINNAME ": send E 0 0 to motor\n");
  usleep(20000);
  secure3Write(busif2.ttyDev, "M 0 0\r\n");
  printf(PLUGINNAME ": send M 0 0 to motor\n");
  fflush(stdout);
  // stop listening
  pthread_join(rxtask1.controlio_thread, NULL);
  printf(PLUGINNAME "steering thread stopped [OK]\n");
  pthread_join(rxtask2.controlio_thread, NULL);
  printf(PLUGINNAME "motor thread stopped [OK]\n");
  return 0;
  if (freLog != NULL)
    fclose(freLog);
}

/************************** XML Initialization **************************/


/** \brief Initialize from XML file
 *
 * Reads the XML file and initializes plugin
 * after successfully reading the XML file.
 *
 * \param[in] *char filename
 * Filename of the XML file
 *
 * \returns int status 1 is OK and -1 is error
 * Status of the initialization process. Negative on error.
 */
extern int initXML(char *filename)
{
  int result;
  parseInfo xmlParse;
  char *xmlBuf = NULL;
  int xmlFilelength;
  int done = 0;
  int len, i;
  FILE *fp;
  //Print initialization message
  //Find revision number from SVN Revision
  char * p1;
  char versionString[20] = REVISION;
  char tempString[10];
  p1 = strrchr(versionString, '$');
  strncpy(tempString, p1, 10);
  p1 = strrchr(tempString, '$');
  if (p1 != NULL)
    *p1 = '\0';
  printf(PLUGINNAME ": Initializing " PLUGINNAME " version 3.%s\n", tempString);
  // initialize default values
    // this is variable in another plugin,
  // and can not be set just now.
  // initial values
  for (i = 0; i < SERVO_COUNT; i++)
  {
    sast.servos[i].servo = i;
    sast.servos[i].vmin = 0;
    sast.servos[i].center = 2048;
    sast.servos[i].vmax = 4095;
    sast.servos[i].name = "servo";
    sast.servos[i].scale = 4096.0/360.0;
  }
  sast.varJoyOverride = -1;
  sast.varSteeringAngleRef = -1;
  sast.varSteeringAngle = -1;
  sast.steerBase = 0.5; // distance between front and driving wheels
  sast.wheelBase = 0.3; // distance between driving wheels
  sast.frontBase = 0.3; // distance between front wheels
  sast.gearRatio = 1.0;   // encoder revolutions per wheel revolution
  sast.wheelDiameter = 0.14; // of (encoder) wheels
  sast.zeroSpeedOffset[0] = 0;
  sast.zeroSpeedOffset[1] = 0;
  sast.enableFrontEncoder = 1;
  sast.enableDynamixel = 1;
  busif1.baudrate=115200;
  busif2.baudrate=115200;
  /* Initialize Expat parser*/
  XML_Parser parser = XML_ParserCreate(NULL);
  result = parser != 0;
  if (!result)
    fprintf(stderr, PLUGINNAME ": Couldn't allocate memory for XML parser\n");
  if (result)
  {  //Setup element handlers
    XML_SetElementHandler(parser, lsStartTag, lsEndTag);
    //Setup shared data
    memset(&xmlParse, 0, sizeof(parseInfo));
    //
    XML_SetUserData(parser, &xmlParse);

    //Open and read the XML file
    fp = fopen(filename,"r");
    result = fp != NULL;
    if(!result)
      printf(PLUGINNAME ": Error reading: %s\n",filename);
  }
  if (result)
  { //Get the length of the file
    fseek(fp,0,SEEK_END);
    xmlFilelength = ftell(fp); //Get position
    fseek(fp,0,SEEK_SET); //Return to start of file
    //Allocate text buffer for full file length
    xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
    result = (xmlBuf != NULL);
    if (!result)
        fprintf(stderr, PLUGINNAME ": Couldn't allocate memory for XML File buffer\n");
  }
  if (result)
  { // clear buffer
    memset(xmlBuf,0,xmlFilelength);
    // read full file
    len = fread(xmlBuf, 1, xmlFilelength, fp);
    fclose(fp);
    //Start parsing the XML file
    result = (XML_Parse(parser, xmlBuf, len, done) != XML_STATUS_ERROR);
    if (!result)
      fprintf(stderr, PLUGINNAME ": XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
  }
  if (parser != NULL)
    XML_ParserFree(parser);
  if (xmlBuf != NULL)
    free(xmlBuf);
  if (result)
  { // all is fine - start plugin
    result = init();
  }
  if (result)
    return 1;
  else
    return -1;
}

//////////////////////////////////////////////////

/**
 * A start tag is detected by the XML parser
 \param data is a user defined context pointer.
 \param el is the tag name
 \param attr is the list of attributes in the start tag. */
void XMLCALL lsStartTag(void *data, const char *el, const char **attr)
{ // a start tag is detected
  int i;
  parseInfo *info = (parseInfo *) data;
  // detect context
  info->depth++;
  if (info->depth < info->skip || info->skip == 0)
  {
    switch (info->depth)
    {
      case 1:
        if (strcmp("rhd",el) == 0)
          ;
        else
          info->skip = info->depth;
        break;
      case 2:
        if (strcmp("plugins",el) == 0)
          ; // no attributes here - but at next level
        else if (strcmp("robot",el) == 0)
        { // robot configuration is needed
          for(i = 0; attr[i]; i+=2)
          {
            if (strcasecmp("frontbase",attr[i]) == 0)
              // distance to steering wheels (ackermann)
              sast.frontBase = strtod(attr[i+1], NULL);
            else if (strcasecmp("steerbase",attr[i]) == 0)
              // distance between steering wheels (ackermann)
              sast.steerBase = strtod(attr[i+1], NULL);
            else if (strcasecmp("wheelbase",attr[i]) == 0)
              // distance between driving wheels (ackermann)
              sast.wheelBase = strtod(attr[i+1], NULL);
            else if (strcasecmp("tickPerRev",attr[i]) == 0)
              // encoder tics per axis revolution
              sast.encTicsPerRev = strtod(attr[i+1], NULL);
            else if (strcasecmp("gearRatio",attr[i]) == 0)
              // gear ratio from encoder to wheel
              sast.gearRatio = strtod(attr[i+1], NULL);
            else if (strcasecmp("wheelDiameter",attr[i]) == 0)
              // wheel diameter in m
              sast.wheelDiameter = strtod(attr[i+1], NULL);
          }
        }
        else
          // skip this group
          info->skip = info->depth;
        break;
      case 3:
        // this one handles interface to FSteer (Sabertooth and magnetic encoder)
        // through arduino interface - the only option is debug
        if (strcmp("dxlSteer",el) == 0)
        { // get enable bit and device name
          const char * att;
          const char * val;
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i];
            val = attr[i + 1];
            if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
              info->enable = 1;
            else if (strcmp("dev1", att) == 0 || strcmp("dev", att) == 0)
              // interface device for steering
              strncpy(busif1.serialDev, val, MxDL);
            else if (strcmp("dev2", att) == 0)
              // interface device for motor control
              strncpy(busif2.serialDev, val, MxDL);
            else if (strcmp("baudrate2",attr[i]) == 0)
              // motor interface speed
              busif2.baudrate = strtol(attr[i+1], NULL, 0);
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
            }
            else if (strcmp("deadzone", att) == 0)
            {
              sast.servos[0].deadzone = strtol(val, NULL, 0);
            }
            else if (strcmp("frontencoder", att) == 0)
            {
              sast.enableFrontEncoder = strtol(val, NULL, 0);
            }
            else if (strcmp("usedynamixel", att) == 0)
            {
              sast.enableDynamixel = strtol(val, NULL, 0);
            }
            else if (strcmp("servoLeft", att) == 0)
            {
              char * p1 = (char*)val;
              if (SERVO_COUNT > 0)
              {
                sast.servos[0].servo = strtol(p1, &p1, 0);
                sast.servos[0].vmin = strtol(p1, &p1, 0);
                sast.servos[0].center = strtol(p1, &p1, 0);
                sast.servos[0].vmax = strtol(p1, &p1, 0);
                sast.servos[0].scale = strtof(p1, &p1);
                sast.servos[0].name = "left";
//                sast.servos[0].scale = 4096.0/360.0;
              }
            }
            else if (strcmp("servoRight", att) == 0)
            {
              char * p1 = (char*)val;
              if (SERVO_COUNT > 1)
              {
                sast.servos[1].servo = strtol(p1, &p1, 0);
                sast.servos[1].vmin = strtol(p1, &p1, 0);
                sast.servos[1].center = strtol(p1, &p1, 0);
                sast.servos[1].vmax = strtol(p1, &p1, 0);
                sast.servos[1].scale = strtof(p1, &p1);
                sast.servos[1].name = "right";
//                sast.servos[1].scale = 4096.0/360.0;
              }
            }
            else if (strcmp("zerospeed", att) == 0)
            {
              char * p1 = (char*)val;
              sast.zeroSpeedOffset[0] = strtol(p1, &p1, 0);
              sast.zeroSpeedOffset[1] = strtol(p1, &p1, 0);
            }
          }
          if (!info->enable)
            printf("   FSteer: Use is disabled in configuration\n");
        }
        else
          info->skip = info->depth;
        break;
      default: // unknown tag series
        break;
    }
  }
}

///////////////////////////////////////////////////////

void XMLCALL lsEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
//  printf("endtag %s changed depth from %d", el, info->depth);
  info->depth--;
  if (info->depth < info->skip)
    // back to normal search for valid tags
    info->skip = 0;
//  printf(" to %d\n", info->depth);
}

////////////////////////////////////////////////////

int openAndSetModeDev1()
{// steering and front encoders
  int result = 0;
  //
  busif1.ttyDev = open(busif1.serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = busif1.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,"   FSteer: Can't open device 1 (steer): %s\n", busif1.serialDev);
//   else
//   { // set baudrate
//     result = (set_serial(busif1.ttyDev, busif1.baudrate) != -1);
//     if (result == 0)
//       fprintf(stderr,"   FSteer: Can't set first serial port parameters\n");
//   }
  if (result)
  { // set desired mode
    printf( PLUGINNAME ": opened %s successfully\n", busif1.serialDev);
    // reset interface - detect servos
    secure3Write(busif1.ttyDev, "reset\n");
    // debug
    //secure3Write(busif1.ttyDev, "i=1\n");
    if (freLog != NULL)
    {
      gettimeofday(&logtime, NULL);
      if (freLog != NULL)
        fprintf(freLog, "%lu.%06lu if1 send servo 'reset'", logtime.tv_sec, logtime.tv_usec);
      fflush(freLog);
    }
    // debug end
    // get status every 5ms
    // secure2Write(busif1.ttyDev, "s=1\n", 5);
  }
  if (result)
  {
    if (sast.enableFrontEncoder)
    {
      secure3Write(busif1.ttyDev, "m=1\n");
      printf( PLUGINNAME ": front encoder enabled - may be reason for instability??\n");
    }
    else
    {
      secure3Write(busif1.ttyDev, "m=0\n");
      printf( PLUGINNAME ": front encoder disabled\n");
    }
    if (sast.enableDynamixel)
    {
      secure3Write(busif1.ttyDev, "x=1\n");
      printf( PLUGINNAME ": dynamixel steering is enabled\n");
    }
    else
    {
      secure3Write(busif1.ttyDev, "x=0\n");
      printf( PLUGINNAME ": dynamixel steering is disabled!!!\n");
    }
  }
  if (debugFlag)
  { // allow no device in debug mode
    result = 1;
    printf( PLUGINNAME ": debug flag set - some errors will be ignored\n");
  }
  busif1.lostConnection = ! result;
  // initialize rx buffer to empty
  busif1.p2 = busif1.rxBuf;
  *busif1.p2 = '\0';
  //
  return result || debugFlag;
}



int openAndSetModeDev2()
{
  int result = 0;
  //
  busif2.ttyDev = open(busif2.serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = busif2.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,"   FSteer: Can't open device 2 (motor): %s\n", busif2.serialDev);
  else
  { // set baudrate
    printf(PLUGINNAME " setting if2 %s to %d baud\n",
           busif2.serialDev, busif2.baudrate);
    result = (set_serial(busif2.ttyDev, busif2.baudrate) != -1);
    if (result == 0)
      fprintf(stderr,"   FSteer: Can't set first serial port parameters for dev 2 (motor)\n");
  }
  if (result)
  { // set desired mode
    printf( PLUGINNAME ": opened %s successfully\n", busif2.serialDev);
    //
    // enable motors
    secure3Write(busif2.ttyDev, "M 0 0\r\n");
    usleep(50000);
    secure3Write(busif2.ttyDev, "E 1 1\r\n");
    if (freLog != NULL)
    {
      gettimeofday(&logtime, NULL);
      if (freLog != NULL)
        fprintf(freLog, "%lu.%06lu if2 send 'E 1 1'\n", logtime.tv_sec, logtime.tv_usec);
      fflush(freLog);
    }
  }
  busif2.lostConnection = ! result;
  // initialize rx buffer to empty
  busif2.p2 = busif2.rxBuf;
  *busif2.p2 = '\0';
  //
  return result || debugFlag;
}

//////////////////////////////////////////////

/**
 * Initialize the communication and start rx thread
 * \returns 1 on success (else 0) */
int init(void)
{ //Open first serial port
  int result;
  //
  rxtask1.running = 0;
  rxtask1.startNewRxCycle = 0;
  rxtask2.running = 0;
  rxtask2.startNewRxCycle = 0;
  sast.safetyStop = 0;
  // debug
  /**
  freLog = fopen("fre.log", "w");
  if (freLog == NULL)
  {
    printf(PLUGINNAME "Failed to open logfile 'fre.log' - no rights? - continues without logging\n");
  }
  else
    fprintf(freLog, "- %s\n", REVISION);
  */
  // debug end
  // open device
  result = openAndSetModeDev1();
  if (result)
  { // start thread to handle bus
    // debug
    gettimeofday(&logtime, NULL);
    if (freLog != NULL)
      fprintf(freLog, "%lu.%06lu opened device 1\n", logtime.tv_sec, logtime.tv_usec);
    // end debug
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rxtask2.controlio_thread, &attr, controlio_task_1, 0))
    {
      perror(PLUGINNAME ": Can't start controlio 1 receive thread (steering)");
      result = 0;
    }
  }
  result &= openAndSetModeDev2();
  if (result)
  { // start thread to handle bus
    // debug
    gettimeofday(&logtime, NULL);
    if (freLog != NULL)
      fprintf(freLog, "%lu.%06lu opened device 2\n", logtime.tv_sec, logtime.tv_usec);
    // end debug
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rxtask2.controlio_thread, &attr, controlio_task_2, 0))
    {
      perror(PLUGINNAME ": Can't start controlio 2 receive thread (motor)");
      result = 0;
    }
  }
  //if (result == 1 || debugFlag)
  { /****** Create database variables if all is ok **************/
    int waitCount = 0;
    // create RHD database variables
    createVariables();
    while (!(rxtask1.running || rxtask2.running))
    { // wait a bit for thread to start
      usleep(20000); //Don't return before threads are running
      if (waitCount >= 50)
      {
        result = 0;
        break;
      }
      waitCount++;
    }
    if (!rxtask1.running)
      perror(PLUGINNAME ": Read thread (steering) is not running");
    if (!rxtask2.running)
      perror(PLUGINNAME ": Read thread (motor) is not running");
  }
  // debug
  gettimeofday(&logtime, NULL);
  if (freLog != NULL)
    fprintf(freLog, "%lu.%06lu init finished\n", logtime.tv_sec, logtime.tv_usec);
  // end debug
  return result;
}

////////////////////////////////////////////////////

char limitSignedChar(int value, int min, int max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}

////////////////////////////////////////////////////

int limitInt(int value, int min, int max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}

////////////////////////////////////////////////////

unsigned char limitUnsignedChar(int value, int min, int max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}

////////////////////////////////////////////////////

int getNewLine(Busif * bus, int timeoutms, int maxWaitCnt)
{
  int l = 0;
  int n = 0;
  char * p1 = bus->rxBuf;
  char * p3;
  // wait for reply - up to 500ms
  // there may be stuff already,
  // remove used part, and append
  n = strlen(bus->p2);
  if (n > 0)
  { // partial message received
    if (n > 0 && n < MxBL - 1)
      memmove(p1, bus->p2, n);
    else
      n = 0;
    p1 = &bus->rxBuf[n];
  }
  // zero terminate - for debug
  *p1 = '\0';
  bus->p2 = bus->rxBuf;
  // see if we already has the next line
  p3 = strchr(bus->p2, '\n');
  // get some data if no new line end
  while ((l < maxWaitCnt) && (p3 == NULL))
  { // more data needed
    int dataOK;
    int m;
    l++; // loop counter
    // wait for data
    dataOK = pollDeviceRx(bus->ttyDev, timeoutms);
    if (dataOK)
    { // there is data
      m = read(bus->ttyDev, p1, MxBL - n - 1);
      if (m > 0)
      { // zero terminate and look for new-line
        p1[m] = '\0';
        p3 = strchr((char*)p1, '\n');
        if (p3 == NULL)
          p3 = strchr((char*)p1, '\r');
        n += m;
        p1 = &bus->rxBuf[n];
      }
      else if (m < 0)
      { // poll error - device is removed?
        printf(PLUGINNAME ": Read error from device - device is lost\n");
        bus->lostConnection = 0;
        break;
      }
    }
  }
  if (p3 != NULL)
  { // terminate and advance p2 to first byte in next message
    *p3++ = '\0';
    while (*p3 < ' ' && *p3 > '\0')
      p3++;
    bus->p2 = p3;
    //printf("# l=%d i=%d (%x) %s\n", strlen(bus->rxBuf), l, (unsigned int)p3, bus->rxBuf);
  }
  else
  {
  }
  //if (l > 1)
  //  printf("getData: got %d bytes in %d pools (timeout=%dms maxLoop=%d\n",
  //          n, l, timeoutms, maxWaitCnt);
  return strlen(bus->rxBuf);
}


///RS232  control thread - for steering
void * controlio_task_1(void * not_used)
{ // run in thread (steer)
  int n;
//   int m = 0; // loop counter
//   int mWait = 12; // wait this many lines before testing status
//   const int MSL = 50;
//   char s[MSL];
  //
  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, "   FSteer: signal: can't ignore SIGPIPE.\n");

  rxtask1.running = 1;
  fprintf(stderr, "   FSteer: rx_task 1 running\n");
  // wait for RHD to init variables
  //usleep(300000);
  // set initial values of variables
  //Mark thread as running
  while (rxtask1.running)
  { // maintain interface
    if (busif1.lostConnection)
    { // connection is lost - during read or write
      if (busif1.ttyDev >= 0)
      {
        close(busif1.ttyDev);
        busif1.ttyDev = -1;
        printf("**** controlio: lost connection - trying to reconnect\n");
        sleep(1);
      }
      // wait a while - for udev to detect device is back
      sleep(3);
      // try to open
      openAndSetModeDev1();
      if (busif1.lostConnection)
        // connection is still lost, so try again
        continue;
    }
    // connection is open get next status message
//    printf("if1 >\n");
    n = getNewLine(&busif1, 20, 22);
//    printf("if1 %d chars %s\n", n, busif2);
    if (n > 2)
    { // new data available)
      char * p1, *p2;
      int n1, n2, n3;
      int servoIdx;
      //float speed;
      // debug
      //printf(PLUGINNAME ": %d get from device: %s\n", m, busif1.rxBuf);
      // debug end
      p1 = busif1.rxBuf;
      while (*p1 < ' ')
        p1++;
      // most likely a usable status message
      // debug
      gettimeofday(&logtime, NULL);
      if (freLog != NULL)
      {
        fprintf(freLog, "%lu.%06lu if1 got %s\n", logtime.tv_sec, logtime.tv_usec, p1);
        fflush(freLog);
      }
      // end debug

      switch (*p1)
      { // status messages
        case 'E': // fast status message
          //printf("%s\n", p1);
          while (*p1 != '\0')
          {
            p2 = p1;
            switch (*p1++)
            {
              case 'E': // encoder values
                n1 = strtol(p1, &p2, 16);
                n2 = strtol(++p2, &p2, 16);
                n3 = strtol(++p2, &p2, 16);
                setVariable(sast.varEncFront, 0, n1);
                setVariable(sast.varEncFront, 1, n2);
                setVariable(sast.varEncFront, 2, n3);
                break;
              case 'M': // magnet status
                n1 = strtol(p1, &p2, 16);
                n2 = strtol(++p2, &p2, 16);
                n3 = strtol(++p2, &p2, 16);
                setVariable(sast.varEncMagnetState, 0, n1);
                setVariable(sast.varEncMagnetState, 1, n2);
                setVariable(sast.varEncMagnetState, 2, n3);
                break;
              case 'P': // start and stop switch
                n1 = strtol(p1, &p2, 0);
                n2 = strtol(++p2, &p2, 0);
                n3 = strtol(++p2, &p2, 0);
                setVariable(sast.varEncFrontErr, 0, n1);
                setVariable(sast.varEncFrontErr, 1, n2);
                setVariable(sast.varEncFrontErr, 2, n3);
                break;
              case 'N': // start and stop switch
                n1 = strtol(p1, &p2, 0);
                setVariable(sast.varEmergStopPushed, 0, n1);
                break;
              default:
                p2++;
                break;
            }
            if (p2 < p1)
              p1++;
            else
              p1 = p2;
          }
          break;
        case 'Y': // fast status message
          //printf("%s\n", p1);
          while (*p1 != '\0')
          {
            servo * dxl;
            switch (*p1++)
            {
              case 'Y': // encoder values
                servoIdx = strtol(p1, &p2, 0);
                if (servoIdx >= 0 && servoIdx <= SERVO_COUNT)
                  dxl = &sast.servos[servoIdx];
                else
                  break;
                n1 = strtol(++p2, &p2, 16);
                setVariable(sast.varServoIntPos, servoIdx, n1);
                dxl->angleRad = (n1 - dxl->center) / dxl->scale * M_PI / 180;
                setVariable(sast.varServoPos, servoIdx, (n1 - dxl->center) * 10 / dxl->scale);
                n1 = strtol(++p2, &p2, 16);
                 if (n1 > 0x3ff)
                   n1 = 0x400 - n1;
                setVariable(sast.varServoVel, servoIdx, n1);
                n1 = strtol(++p2, &p2, 16);
                if (n1 > 0x3ff)
                  n1 = 0x400 - n1;
                setVariable(sast.varServoLoad, servoIdx, n1);
                n1 = strtol(++p2, &p2, 16);
                setVariable(sast.varServoVolt, servoIdx, n1);
                n1 = strtol(++p2, &p2, 16);
                setVariable(sast.varServoTemp, servoIdx, n1);
                n1 = strtol(++p2, &p2, 16);
                setVariable(sast.varServoComErrCnt, servoIdx, n1);
                break;
              case 'F':
                for (n1 = 0; n1 < 7; n1++)
                { // 7 debug variables
                  // dxlErrSuccess, dxlErrTimeout, dxlErrCurrupt, dxlRxTimeoutCnt,
                  // dxlRxTimeoutCntMax, dxlBusUseCnt, dxlBusAlreadyInUseCnt
                  n2 = strtol(++p2, &p2, 16);
                  setVariable(sast.varDynamixelErr, n1, n2);
                }
                break;
              default:
                p2++;
                break;
            }
            if (p2 < p1)
              p1++;
            else
              p1 = p2;
          }
          break;
        default:
          break;
      }
      float sr, sl, saddeg;
      if (sast.servos[0].angleRad > 1e-10)
        // calculate actual steering angle in radians for each wheel
        sl = atan2(sast.frontBase, sast.frontBase / tan(sast.servos[0].angleRad) + sast.steerBase/2.0);
      else if (sast.servos[0].angleRad < - 1e-10)
        sl = -atan2(sast.frontBase, sast.frontBase / -tan(sast.servos[0].angleRad) - sast.steerBase/2.0);
      else
        // except for small angles to avoid div by zero
        sl = 0.0;
      if (sast.servos[1].angleRad > 1e-10)
        sr = atan2(sast.frontBase, sast.frontBase / tan(sast.servos[1].angleRad) - sast.steerBase/2.0);
      else if (sast.servos[1].angleRad < - 1e-10)
        sr = -atan2(sast.frontBase, sast.frontBase / -tan(sast.servos[1].angleRad) + sast.steerBase/2.0);
      else
        sr = 0.0;
      // convert to average steering angle in deci degrees
      saddeg = roundi((sl + sr) / 2.0 * 1800 / M_PI);
      // set RHD variable - used by MRC for (actual) heading calculation
      setVariable(sast.varSteeringAngle, 0, saddeg);
    }
    // rx variables updated
  }
  rxtask1.running = 0;
  fprintf(stderr,PLUGINNAME ": closing bus device\n");
  close(busif1.ttyDev);
  fprintf(stderr,PLUGINNAME ": Shutting down thread\n");
  pthread_exit(0);
  return NULL;
}



///RS232  control thread for motor interface
void * controlio_task_2(void * not_used)
{ // run in thread (motor)
  int n;
//   int m = 0; // loop counter
//   int mWait = 12; // wait this many lines before testing status
//   const int MSL = 50;
//   char s[MSL];
  //
  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, "   FSteer: signal: can't ignore SIGPIPE.\n");

  rxtask2.running = 1;
  fprintf(stderr, "   FSteer: rx_task 2 running\n");
  // wait for RHD to init variables
  //usleep(300000);
  // set initial values of variables
  //Mark thread as running
  while (rxtask2.running)
  { // maintain interface
    if (busif2.lostConnection)
    { // connection is lost - during read or write
      if (busif2.ttyDev >= 0)
      {
        close(busif2.ttyDev);
        busif2.ttyDev = -1;
        printf("**** controlio: lost connection (2) - trying to reconnect\n");
        sleep(1);
      }
      // wait a while - for udev to detect device is back
      sleep(3);
      // try to open
      openAndSetModeDev2();
      if (busif2.lostConnection)
        // connection is still lost, so try again
        continue;
    }
    // connection is open get next status message
    //printf(">");
//    printf("if2 >\n");
    n = getNewLine(&busif2, 20, 22);
//    printf("if2 %d chars %s\n", n, busif2.rxBuf);
    //printf("<");
    if (n > 2)
    { // new data available)
      char * p1, *p2;
      int n1, n2;
      // printf("if2 %d chars %s\n", n, busif2.rxBuf);
      p1 = busif2.rxBuf;
      while (*p1 < ' ')
        p1++;
      // debug
      gettimeofday(&logtime, NULL);
      if (freLog != NULL)
        fprintf(freLog, "%lu.%06lu if2 got %s\n", logtime.tv_sec, logtime.tv_usec, p1);
      // end debug
      // most likely a usable status message
      switch (*p1)
      { // status messages
        case 'E': // encoder message
//          printf("%s\n", p1);
          n1 = strtol(++p1, &p2, 0);
          n2 = strtol(++p2, &p2, 0);
          setVariable(sast.varEncLeft, 0, -n1);
          setVariable(sast.varEncRight, 0, -n2);
          break;
        default:
          break;
      }
    }
    // rx variables updated
  }
  rxtask2.running = 0;
  fprintf(stderr,PLUGINNAME ": closing bus device\n");
  close(busif2.ttyDev);
  fprintf(stderr,PLUGINNAME ": Shutting down thread\n");
  pthread_exit(0);
  return NULL;
}

///////////////////////////////////////////////////////////

int pollDeviceRx(int fd, int msTimeout)
{
  int dataAvailable = 0;
  //
  if (fd == -1)
  { // no device - debugging mode
    usleep(50000);
  }
  else
  { // test for data
    int err = 0;
    struct pollfd pollStatus;
    int revents;
    //
    pollStatus.fd = fd;
    pollStatus.revents = 0;
    pollStatus.events = POLLIN  |  /*  0x0001  There is data to read */
                        POLLPRI;   /*  0x0002  There is urgent data to read */
                                  /* POLLOUT 0x0004  Writing now will not block */
    //
    err = poll(&pollStatus, 1, msTimeout);
    if (err < 0)
    { // not a valid call (may be debugger interrrupted)
      perror(PLUGINNAME ":pollDeviceRx (poll)");
    }
    else if (err > 0)
    { // at least one connection has data (or status change)
      revents = pollStatus.revents;
      if (((revents & POLLIN) != 0) ||
          ((revents & POLLPRI) != 0))
        dataAvailable = 1;
    }
  }
  return dataAvailable;
}

/////////////////////////////////////////////////

int getDatabaseVariable(char type, const char * name)
{ // get index of a variable in the database
  symTableElement * syms;
  int symsCnt;
  int result = -1;
  int i;
  //
  syms = getSymbolTable(type);
  symsCnt = getSymtableSize(type);
  for (i = 0; i < symsCnt; i++)
  {
    if (strcmp(syms->name, name) == 0)
    {
      result = i;
      break;
    }
    syms++;
  }
  return result;
}

/////////////////////////////////////////////////////////

ssize_t secure2Write(int dev, const void *buf, ssize_t txLen)
{
  if (dev >= 0)
    return secureWrite(dev, buf, txLen);
  else if (debugFlag)
    // perform as if all is written - debug mode
    return txLen;
  else
    return 0;
}

///////////////////////////////////////////////

int limitSigned(int value, int min, int max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}

////////////////////////////////////////////////////

unsigned int limitUnsigned(int value, int min, int max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}


/**
 * Set servo position if changed, and send to servo */
int setServos(float angleLeft, float angleRight)
{
#define ALL_SERVOS 0xfe
  int isOK = 0;
  int r0, r1, p0, p1;
//   float ddeg;
//   // average movement for movement time
//   ddeg = (sast.servos[0].degreeRef - angleLeft +
//           sast.servos[1].degreeRef - angleRight)/2.0;
  // save angle ref for each servo (debug)
  angleLeft  *= 180.0 / M_PI;
  angleRight *= 180.0 / M_PI;
  sast.servos[0].degreeRef = angleLeft;
  sast.servos[1].degreeRef = angleRight;
  // convert angle to servo units
  r0 = roundi(angleLeft  * sast.servos[0].scale + sast.servos[0].center);
  r1 = roundi(angleRight * sast.servos[1].scale + sast.servos[1].center);
  // limit manoeuvre space
  p0 = limitSigned(r0, sast.servos[0].vmin, sast.servos[0].vmax);
  p1 = limitSigned(r1, sast.servos[1].vmin, sast.servos[1].vmax);
  // implement
  if ((p0 != sast.servos[0].positionRef) ||
      (p1 != sast.servos[1].positionRef))
  { // servo position needs to be updated
    char s[32];
    sast.servos[0].positionRef = p0;
    sast.servos[1].positionRef = p1;
    snprintf(s, 32, "K=%d,%d\n", p0, p1);
    secure3Write(busif1.ttyDev, s);
    setVariable(sast.varServoIntRef, 0, p0);
    setVariable(sast.varServoIntRef, 1, p1);
    // debug
    gettimeofday(&logtime, NULL);
    if (freLog != NULL)
      fprintf(freLog, "%lu.%06lu if1 send %s", logtime.tv_sec, logtime.tv_usec, s);
    // end debug
  }
  return isOK;
}
