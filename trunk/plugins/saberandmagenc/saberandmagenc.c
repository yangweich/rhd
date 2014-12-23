 /** \file saberandmagenc.c
 *  \ingroup hwmodule
 *
 * Interface for sabertooth motor controller and AS4050 magnetic encoders
 * through an arduino nano velocity controller
 *
 *
 *       NBNB! not finished (18/6 2012) Christian
 * 
 * 
 *******************************************************************/
/***************************** Plug-in version  *****************************/
#define SD84_VERSION    "0.1501"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 63 $:"
 #define DATE             "$Date: 2011-09-18 13:43:12 +0200 (Sun, 18 Sep 2011) $:"
 #define ID               "$Id: saberandmagenc.c 63 2012-10-21 16:15:41Z jcan $"
 #define PLUGINNAME        "SaMe"
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

#include "saberandmagenc.h"

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
void * controlio_task(void *);
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
 * \param buf is the byte buffer with data to send
 * \param bufCnt is the number of bytes to send
 * \returns the number of bytes send, 0 if failed to send. */
ssize_t secure2Write(const void *buf, ssize_t txLen);
/**
 * calls secure2Write and supplyes txlen from length of string */
inline ssize_t secure3Write(const void *buf)
{
  return secure2Write(buf, strlen(buf));
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
struct
{ /** is receive thread running */
  int running;
  /** Trigger polling of data from units - when set to 1 */
  int startNewRxCycle;
  /** name of serial device */
  /** thread handle and attributes */
  pthread_t controlio_thread;
} rxtask;

int tick = 0;
/// old steering angle value - in radians
double sacOld = -4.0;
int debugFlag = 0;
/** flags for updating of write variables */
int settingUpd = 1, steerUpd = 1;
struct timeval tickTime;


////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int rhdTick)
{
  int returnValue = rxtask.running;
#define MAX_SPEED_STR_CNT 32
  char cmdStr[MAX_SPEED_STR_CNT];
  int n;
  //gettimeofday(&tickTime, NULL);
  rxtask.startNewRxCycle += 1;
  tick = rhdTick;
  if (tick == 1)
  { // connect to joystick - if any
    sast.varJoyOverride = getDatabaseVariable('r', "joyoverride");
    printf(  PLUGINNAME " joystick override  (r) variable index=%d\n", sast.varJoyOverride);
    sast.varSteeringAngleRef = getDatabaseVariable('w', "steeringangleref");
    if (sast.varSteeringAngleRef < 0)
      printf(PLUGINNAME ": ERROR steering angle ref variable is unavailable, but is needed for ackerman steering!!!\n");
    else
      printf(PLUGINNAME " steering angle ref (w) variable index=%d\n", sast.varSteeringAngleRef);
    // set desired speed to 0
    secure2Write("w 0 0\n\r", 7);
    secure2Write("v 0 0\n\r", 7);
    // enable velocity control
    //secure2Write("p 1\n\r", 5);
  }
  if (isUpdated('w', sast.varVelMode))
  {
    sast.velMode = getWriteVariable(sast.varVelMode, 0);
  }
  if (sast.varJoyOverride >= 0)
  { // remote control is available
    sast.joyOverride = getReadVariable(sast.varJoyOverride, 0);
  }
  if (sast.varSteeringAngleRef >= 0)
  { // steering is possible
    sast.steeringangleref = getWriteVariable(sast.varSteeringAngleRef, 0) * M_PI / 180000.0;
  }
  if (isUpdated('w', sast.varSteeringAngleRef) || isUpdated('w', sast.varSpeedRef))
  { // convert steering angle to wheel speed, as steering angle or speed is changed
    float speed = getWriteVariable(sast.varSpeedRef, 0) / 1000.0; // in m/s
    // get steering angle (for a cycle model steering) in deci degrees - convert to radians
    double steerAngle = getWriteVariable(sast.varSteeringAngleRef, 0) * M_PI / 180.0 / 10.0;
    //symTableElement * symw = getSymtable('w');
    // reset write variables
    //printf(PLUGINNAME ": got speed %gm/s, %gdeg\n", speed, steerAngle * 180.0 / M_PI);
//    symw[state.varSpeedRef].updated = 0;
//    symw[state.varSteeringAngleRef].updated = 0;
    if (speed != 0)
    { // calculate wheel speed (left and right) at this steering angle and speedref
      // ensure steering angle is less than 90 deg, i.e. tan(steeringAngle) < infinity
      if (steerAngle > M_PI / 2.0 - 0.001)
        // close to 90 deg steering angle
        steerAngle = M_PI / 2.0 - 0.001;
      else if (steerAngle < -M_PI / 2.0 + 0.001)
        // close to 90 deg steering angle
        steerAngle = -M_PI / 2.0 + 0.001;
      // find matching driving wheel speed
      { // normal steering angle (valid tangent value)
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
        // write values back to symbol stack
      }
    }
    else
    { // stop
      sast.speedRefLeft = 0.0;
      sast.speedRefRight = 0.0;
    }
  }
  if ((!isMasterAlive && !sast.joyOverride) || 
       (sast.varSteeringAngleRef < 0) ||
       (getReadVariable(sast.varEmergStopPushed, 0)))
  { // no write allowed client - set speed to 0
    sast.speedRefLeft = 0.0;
    sast.speedRefRight = 0.0;
    if (!sast.safetyStop)
    {
      printf(PLUGINNAME ":: safety stop - no rhd clients and no joystick control\n");
      sast.safetyStop = 1;
    }
  }
  else
    sast.safetyStop = 0;
  // implement desired speed
  if (tick % 4 == 0)
  { // intended speed converted to encoder velocity in 10 ms
    int velLeft; //= roundi(sast.speedRefLeft * sast.robVel2EncVel);
    int velRight; // = roundi(sast.speedRefRight * sast.robVel2EncVel);
    int same;
    if (sast.velMode == 0)
    { // PWM mode
      velLeft = roundi((sast.speedRefLeft /* + sast.zeroVelOffsetL*/)
                   * sast.robVel2PWMVel);
      velRight = roundi((sast.speedRefRight /*+ sast.zeroVelOffsetR*/) 
                   * sast.robVel2PWMVel);
    }
    else
    { // velocity control mode
      velLeft = roundi(sast.speedRefLeft * sast.robVel2EncVel);
      velRight = roundi(sast.speedRefRight * sast.robVel2EncVel);
    }
    same = (velRight == sast.velRightRef) && (velLeft == sast.velLeftRef);
    if (! same)
    { // set new command to motor controller
      if (sast.velMode == 0)
        // PWM mode
        snprintf(cmdStr, MAX_SPEED_STR_CNT, "w %d %d\n", velLeft, velRight);
      else
        // velocity mode (not OK yet)
        snprintf(cmdStr, MAX_SPEED_STR_CNT, "v %d %d\n", velLeft, velRight);
      // send data to motor controller
      n = secure2Write(cmdStr, strlen(cmdStr));
      if (n != strlen(cmdStr))
        printf(PLUGINNAME "::periodic: velocity command: 'v %d %d' [send failed]\n", velLeft, velRight);
      else
      {
        // debug
  //         printf(PLUGINNAME "::periodic: velocity command: 'v %d %d' [send OK]\n", velLeft, velRight);
        // debug end
        sast.velLeftRef = velLeft;
        sast.velRightRef = velRight;
      }
    }
  }
    //
  return returnValue;
}

//////////////////////////////////////////////////////
/**
 * Create variables for the usb to controlio converter itself */

void createVariables()
{ // create variables in RHD
  //state.varBatt = createVariable('r',1,"batteryvolt"); // battery voltage in V * 100
  sast.varEncLeft = createVariable('r',1,"encl"); // encoder value
  sast.varEncRight = createVariable('r',1,"encr");
  sast.varSpeedRef  = createVariable('w', 1, "speedref"); // in mm/sec 
  sast.varSpeed = createVariable('r', 2, "encspeed"); // actual speed in mm/s [left right)
  //state.varReset = createVariable('w', 1, "reset"); // 1 to reset (no function)
  //sast.varTurnrate = createVariable('r', 1, "encturnrate"); // turnrate based on diferential motor speed
  sast.varStartEnabled = createVariable('r', 1, "startenabled"); // 1 if start is enabled (wire pulled)
  sast.varEmergStopPushed = createVariable('r', 1, "emergencyswitch"); // 1 if emergenct stop pushed
  sast.varMagnetState = createVariable('r', 2, "encmagnet"); // state of encoder magnet 0=left, 1=right, val: 0=OK, 1=moving away, 2=moving closer, 3= bad
  sast.varVelMode = createVariable('w',1,"velmode");
  sast.varVelModeIs = createVariable('r',1,"velmodeis");
  sast.varVelPWM = createVariable('r',2,"velPWM");
  //
  // initialize other variables
  sast.speedRefLeft = 0.0;
  sast.speedRefRight = 0.0;
  sast.joyOverride = 0;
  sast.speedLeft = 0.0;
  sast.speedRight = 0.0;
  sast.speedLeftEnc = 0.0;
  sast.speedRightEnc = 0.0;
  sast.robVel2PWMVel = sast.encTicsPerRev * sast.gearRatio / (100.0 * sast.wheelDiameter * M_PI * 4.0);
  sast.robVel2EncVel = sast.encTicsPerRev * sast.gearRatio / (100.0 * sast.wheelDiameter * M_PI);
  printf(PLUGINNAME ": has created its read and write variables\n");
}

////////////////////////////////////////////////////////////////

int terminate(void)
{
  rxtask.running = 0;
  printf(PLUGINNAME ": stopping ... ");
  // stop motors - in either mode
  secure2Write("v 0 0\n\r", 7);
  secure2Write("w 0 0\n\r", 7);
  fflush(stdout);
  pthread_join(rxtask.controlio_thread, NULL);
  printf("[OK]\n");
  return 0;
}

/************************** XML Initialization **************************/


/** \brief Initialize the SD84 board from XML file
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
  int i, len;
  FILE *fp;
  //Print initialization message
  //Find revision number from SVN Revision
  char * p1;
  char versionString[20] = REVISION;
  char tempString[10];
  p1 = strrchr(versionString, '$');
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  printf(PLUGINNAME ": Initializing plug-in version %s.%s\n",SD84_VERSION, tempString);
  printf(PLUGINNAME ": waiting for udev to detect device .");
  for (i = 0; i < 3; i++)
  {
    fflush(NULL);
    sleep(1);
    printf(".");
  }
  printf(" [OK]\n");
  // initialize default values
    // this is variable in another plugin,
  // and can not be set just now.
  sast.varJoyOverride = -1;
  sast.varSteeringAngleRef = -1;
  sast.steerBase = 0.5; // distance between front and driving wheels
  sast.wheelBase = 0.3; // distance between driving wheels
  sast.frontBase = 0.3; // distance between front wheels
  sast.gearRatio = 5.5; // encoder revolutions per wheel revolution
  sast.wheelDiameter = 0.14; // of (encoder) wheels
  sast.zeroPWMOffsetR = 0;
  sast.zeroPWMOffsetL = 0; // PWM offset in us for zero speed
  sast.zeroPWMOffsetRis = -1; // from controller
  sast.zeroPWMOffsetLis = -1; // PWM offset in us for zero speed
  sast.velPWMisL = -1;
  sast.velPWMisR = -1;
  sast.velMode = 1;
  sast.velModeIs = -1;
  sast.velCtrlFF = 16;
  sast.velCtrlFFis = -1;
  sast.velCtrlP = 1;
  sast.velCtrlPis = -1;
  sast.safetyStop = 1;
  sast.datarateIs = -1;
  busif.baudrate=115200;

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
        fprintf(stderr, "   SaMe: Couldn't allocate memory for XML File buffer\n");
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
  if (result && xmlParse.enable)
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
        // this one handles interface to SaMe (Sabertooth and magnetic encoder)
        // through arduino interface - the only option is debug
        if (strcmp("saberandmagenc",el) == 0)
        { // get enable bit and device name
          const char * att;
          const char * val;
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i]; 
            val = attr[i + 1];
            if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
              info->enable = 1;
            else if (strcmp("dev", att) == 0)
              // interface device
              strncpy(busif.serialDev, val, MxDL);
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
              if (debugFlag)
                printf("   SaMe started in DEBUG mode!\n");
            }
            else if (strcasecmp("baudrate",att) == 0)
              // velocity offset in m/s to get zero velocity
              busif.baudrate = strtol(val, NULL, 0);
            else if (strcasecmp("zeroPWMOffsetL",att) == 0)
              // velocity offset in m/s to get zero velocity
              sast.zeroPWMOffsetL = strtod(val, NULL);
            else if (strcasecmp("zeroPWMOffsetR",att) == 0)
              // velocity offset in m/s to get zero velocity
              sast.zeroPWMOffsetR = strtod(val, NULL);
            else if (strcasecmp("velmode",att) == 0)
              // velocity mode 0=PWM, 1=velocity P-FF ctrl
              sast.velMode = strtol(val, NULL, 10);
            else if (strcasecmp("velctrlFF",att) == 0)
              // velocity mode fead forward gain
              sast.velCtrlFF = strtol(val, NULL, 10);
            else if (strcasecmp("velctrlP",att) == 0)
              // velocity mode proportional gain
              sast.velCtrlP = strtol(val, NULL, 10);
            else if (strcasecmp("datarate",att) == 0)
              // velocity mode proportional gain
              sast.datarate = strtol(val, NULL, 10);
          }
          if (!info->enable)
            printf("   SaMe: Use is disabled in configuration\n");
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

int openAndSetMode()
{
  int result = 0;
  //
  busif.ttyDev = open(busif.serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = busif.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,"   SaMe: Can't open device: %s\n", busif.serialDev);
  else
  { // set baudrate
    result = (set_serial(busif.ttyDev, busif.baudrate) != -1);
    if (result == 0)
      fprintf(stderr,"   SaMe: Can't set first serial port parameters\n");
  }
  if (result)
  { // set desired mode
//     const int MSL = 20;
//     char s[MSL];
//     int n;
//     // set vel ctrl parameters
//     usleep(300000);
//     n = secure2Write("g100\n\r", 6);
//     result &= (n == 6);
//     printf(PLUGINNAME ":set %d chars for g100\n", n);
//     usleep(30000);
//     snprintf(s, MSL, "kp%d\n\r", sast.velCtrlK);
//     n = secure2Write(s, strlen(s));
//     result &= (n == strlen(s));
//     printf(PLUGINNAME ":set %d chars for %s", n, s);
//     usleep(30000);
//     snprintf(s, MSL, "kf%d\n\r", sast.velCtrlFF);
//     n = secure2Write(s, strlen(s));
//     result &= (n == strlen(s));
//     printf(PLUGINNAME ":set %d chars for %s", n, s);
//     usleep(30000);
//     // set velocity mode (default is PWM)
//     snprintf(s, MSL, "p%d\n\r", sast.velMode);
//     n = secure2Write(s, strlen(s));
//     result &= (n == strlen(s));
//     printf(PLUGINNAME ":set %d chars for %s", n, s);
//     usleep(30000);
//     // get status every 20ms
//     n = secure2Write("g2\n\r", 4);
//     result &= (n == 4);
//     printf(PLUGINNAME ":set %d chars for g2\n", n);
  }
  if (debugFlag)
  { // allow no device in debug mode
    result = 1;
  }
  busif.lostConnection = ! result;
  // initialize rx buffer to empty
  busif.p2 = busif.rxBuf;
  *busif.p2 = '\0';
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
  rxtask.running = 0;
  rxtask.startNewRxCycle = 0;
  // open device
  result = openAndSetMode();
  if (result == 1)
  { // start thread to handle bus
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rxtask.controlio_thread, &attr, controlio_task, 0))
    {
      perror("   usbiss: Can't start controlio receive thread");
      result = 0;
    }
  }
  //if (result == 1 || debugFlag)
  { /****** Create database variables if all is ok **************/
    int waitCount = 0;
    // create RHD database variables
    createVariables();
    while (!rxtask.running)
    { // wait a bit for thread to start
      usleep(20000); //Don't return before threads are running
      if (waitCount >= 50)
      {
        result = 0;
        break;
      }
      waitCount++;
    }
  }
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

int getNewLine(int timeoutms, int maxWaitCnt)
{
  int l = 0;
  int n = 0;
  char * p1 = busif.rxBuf;
  char * p3;
  // wait for reply - up to 500ms
  // there may be stuff already,
  // remove used part, and append
  n = strlen(busif.p2);
  if (n > 0)
  { // partial message received
    if (n > 0 && n < MxBL - 1)
      memmove(p1, busif.p2, n);
    else
      n = 0;
    p1 = &busif.rxBuf[n];
  }
  // zero terminate - for debug
  *p1 = '\0';
  busif.p2 = busif.rxBuf;
  // see if we already has the next line
  p3 = strchr(busif.p2, '\n');
  // get some data if no new line end
  while ((l < maxWaitCnt) && (p3 == NULL))
  { // more data needed
    int dataOK;
    int m;
    l++; // loop counter
    // wait for data
    dataOK = pollDeviceRx(busif.ttyDev, timeoutms);
    if (dataOK)
    { // there is data
      m = read(busif.ttyDev, p1, MxBL - n - 1);
      if (m > 0)
      { // zero terminate and look for new-line
        p1[m] = '\0';
        p3 = strchr((char*)p1, '\n');
        n += m;
        p1 = &busif.rxBuf[n];
      }
      else if (m < 0)
      { // poll error - device is removed?
        printf(PLUGINNAME ": Read error from device - device is lost\n");
        busif.lostConnection = 0;
        break;
      }
    }
  }
  if (p3 != NULL)
  { // terminate and advance p2 to first byte in next message
    *p3++ = '\0';
    while (*p3 < ' ' && *p3 > '\0')
      p3++;
    busif.p2 = p3;
    //printf("# l=%d i=%d (%x) %s\n", strlen(busif.rxBuf), l, (unsigned int)p3, busif.rxBuf);
  }
  else
  {
  }
  //if (l > 1)
  //  printf("getData: got %d bytes in %d pools (timeout=%dms maxLoop=%d\n",
  //          n, l, timeoutms, maxWaitCnt);
  return strlen(busif.rxBuf);
}


///RS232  control thread
void * controlio_task(void * not_used)
{ // run in thread
  int n;
  int m = 0; // loop counter
  int mWait = 12; // wait this many lines before testing status
  const int MSL = 50;
  char s[MSL];
  //
  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, "   SaMe: signal: can't ignore SIGPIPE.\n");

  rxtask.running = 1;
  fprintf(stderr, "   SaMe: rx_task running\n");
  // wait for RHD to init variables
  usleep(300000);
  // set initial values of variables
  //Mark thread as running
  while (rxtask.running)
  { // maintain interface
    if (busif.lostConnection)
    { // connection is lost - during read or write
      if (busif.ttyDev >= 0)
      {
        close(busif.ttyDev);
        busif.ttyDev = -1;
        printf("**** controlio: lost connection - trying to reconnect\n");
        sleep(1); 
      }
      // wait a while - for udev to detect device is back
      sleep(1);
      // try to open
      openAndSetMode();
      if (busif.lostConnection)
        // connection is still lost, so try again
        continue;
    }
    // connection is open get next status message
    //printf(">");
    n = getNewLine(20, 22);
    //printf("<");
    if (n > 10)
    { // new data available)
      char * p1, *p2;
      int n1, n2;
      //float speed;
      // debug
      //printf(PLUGINNAME ": %d get from device: %s\n", m, busif.rxBuf);
      // debug end
      p1 = busif.rxBuf;
      while (*p1 < ' ')
        p1++;
      // most likely a usable status message
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
                setVariable(sast.varEncLeft, 0, n1);
                setVariable(sast.varEncRight, 0, n2);
                break;
              case 'V': // velocity
                n1 = strtol(p1, &p2, 0);
                n2 = strtol(++p2, &p2, 0);
                sast.speedLeftEnc = (float)n1 / sast.robVel2EncVel;
                setVariable(sast.varSpeed, 0, roundi(sast.speedLeftEnc*1000));
                sast.speedRightEnc = (float)n2 / sast.robVel2EncVel;
                setVariable(sast.varSpeed, 1, roundi(sast.speedRightEnc*1000));
                break;
              case 'M': // magnet status
                n1 = strtol(p1, &p2, 0);
                n2 = strtol(++p2, &p2, 0);
                setVariable(sast.varMagnetState, 0, n1);
                setVariable(sast.varMagnetState, 1, n2);
                break;
              case 'S': // start and stop switch
                n1 = strtol(p1, &p2, 0);
                n2 = strtol(++p2, &p2, 0);
                setVariable(sast.varStartEnabled, 0, n1);
                setVariable(sast.varEmergStopPushed, 0, n2);
                break;
              case 'W': // PWM values
                sast.velPWMisL = strtol(p1, &p2, 0);
                sast.velPWMisR = strtol(++p2, &p2, 0);
                setVariable(sast.varVelPWM, 0, sast.velPWMisL);
                setVariable(sast.varVelPWM, 1, sast.velPWMisR);
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
        case 'g': // update rate
          p1 += 2;
          sast.datarateIs = strtol(p1, NULL, 10);
          break;
        case 'o': // PWM offset
          p1 += 2;
          sast.zeroPWMOffsetLis = strtol(p1, &p1, 10);
          sast.zeroPWMOffsetRis = strtol(++p1, &p1, 10);
          break;
//         case 'w': // PWM used
//           p1 += 2;
//           sast.velPWMisL = 1500 - strtol(p1, &p1, 10);
//           sast.velPWMisR = 1500 - strtol(++p1, &p1, 10);
//           break;
        case 'p': // control mode 0=PWM 1=vel
          p1 += 2;
          sast.velModeIs = strtol(p1, NULL, 10);
          setVariable(sast.varVelModeIs, 0, sast.velModeIs);
          break;
        case 'k': // vel controll params
          p1++;
          switch (*p1++)
          {
            case 'f':
              sast.velCtrlFFis = strtol(++p1, NULL, 10);
              break;
            case 'p':
              sast.velCtrlPis = strtol(++p1, NULL, 10);
              break;
            default: break;
          }
          break;
        default:
          break;
      }
    }
    if (m > mWait)
    { // ensure general setting is OK
      int flushAndWrite = 0;
      if (sast.datarateIs < 0)
      {
        snprintf(s, MSL, "h\n\r");
        flushAndWrite = 1;
      }
      else
      { // test if there is changes
        if (sast.velMode != sast.velModeIs)
        {
          snprintf(s, MSL, "p%d\n\r", sast.velMode);
          flushAndWrite = 1;
        }
        else if (sast.datarate != sast.datarateIs)
        {
          snprintf(s, MSL, "g%d\n\r", sast.datarate);
          flushAndWrite = 1;
        }
        else if ((sast.zeroPWMOffsetL != sast.zeroPWMOffsetLis) ||
                 (sast.zeroPWMOffsetR != sast.zeroPWMOffsetRis))
        {
          snprintf(s, MSL, "o%d %d\n\r", sast.zeroPWMOffsetL, 
                                         sast.zeroPWMOffsetR);
          flushAndWrite = 1;
        }
        else if (sast.velCtrlFF != sast.velCtrlFFis)
        {
          snprintf(s, MSL, "kf%d\n\r", sast.velCtrlFF);
          flushAndWrite = 1;
        }
        else if (sast.velCtrlP != sast.velCtrlPis)
        {
          snprintf(s, MSL, "kp%d\n\r", sast.velCtrlP);
          flushAndWrite = 1;
        }
      }
      if (flushAndWrite)
      { // flush any waiting data
        int f = 0;
        while (getNewLine(1, 1) > 0)
          printf(".%d.", f++); // discard line
        secure3Write(s);
      }
      if (sast.datarate == 0)
        mWait = m + 10;
      else
        mWait = m + 10 + 100/sast.datarate;
    }
    m++;
    // rx variables updated
  }
  rxtask.running = 0;
  fprintf(stderr,PLUGINNAME ": closing bus device\n");
  close(busif.ttyDev);
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

ssize_t secure2Write(const void *buf, ssize_t txLen)
{
  if (busif.ttyDev >= 0)
    return secureWrite(busif.ttyDev, buf, txLen);
  else if (debugFlag)
    // perform as if all is written - debug mode
    return txLen;
  else
    return 0;
}

