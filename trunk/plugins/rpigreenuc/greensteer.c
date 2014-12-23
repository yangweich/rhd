 /** \file greensteer.c
 *  \ingroup hwmodule
 *
 * Interface for green robot steering and some sensors
 *
 *
 *
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 381 $:"
 #define DATE             "$Date: 2013-12-29 16:43:50 +0100 (Sun, 29 Dec 2013) $:"
 #define ID               "$Id: greensteer.c 381 2013-12-29 15:43:50Z jcan $"
 #define PLUGINNAME        "GrSteer"
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

#include "greensteer.h"

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

Rxtask rxtask1;

int tick = 0;
/// old steering angle value - in radians
//double sacOld = -4.0;
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
/**
 * Set pan-tilt servo position (if changed)
 * \param anglePan is desired wheel angle in degrees - zero is straight ahead
 * \param angleTilt is desired angle in degrees - zero is straight ahead
 */
//int setPanTilt(float anglePan, float angleTilt);


// debug
FILE * gsLog = NULL;
struct timeval logtime;
// debug end

////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int rhdTick)
{
  //gettimeofday(&tickTime, NULL);
  rxtask1.startNewRxCycle += 1;
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
  if (tick % 2 == 1) // isUpdated('w', sast.varSteeringAngleRef))
  { // steering changed - calculate new angle for left and right wheel
    float tanSac, ofs, sal, sar, aal, aar;
    float steerRef = -getWriteVariable(sast.varSteeringAngleRef, 0) * M_PI / 1800.0;
    // reset update flag
    steerUpd = 0;
    // get tan to steering angle, and approximate, if >= 90 deg
    if (fabs(steerRef) < (M_PI/2.0 - 1e-5))
      tanSac = tan(steerRef);
    else // use +-90 degrees
      tanSac = steerRef * 1e10;
    ofs = sast.frontBase * tanSac / 2.0 / sast.steerBase;
    // find angle in radians for each wheel
    sal = atan2(tanSac, 1.0 - ofs);
    sar = atan2(tanSac, 1.0 + ofs);
    // implement new servo positions
//    if (tick % 2 == 1)
      setServos(sar, sal);
    // calculate actual steering reference
    aal = (sast.servos[0].positionRef - sast.servos[0].center)/sast.servos[0].scale;
    aar = (sast.servos[1].positionRef - sast.servos[1].center)/sast.servos[1].scale;
    setVariable(sast.varSteeringAngle, 0, roundi((aal + aar) / 2.0 * 10.0));
//    sacOld = steerRef;
  }
//   if (isUpdated('w', sast.varCamPanTiltRef))
//   { // set pan-tilt if needed
//     setPanTilt(getWriteVariable(sast.varCamPanTiltRef, 0) / 10.0, getWriteVariable(sast.varCamPanTiltRef, 1) / 10.0);
//   }
  //
  // request steer status
//  if (tick % 2 == 0)
  {
    secure3Write(busif1.ttyDev, "s\n");
    // debug
    if (gsLog != NULL)
    {
      gettimeofday(&logtime, NULL);
      fprintf(gsLog, "%lu.%06lu if1 send s\n", logtime.tv_sec, logtime.tv_usec);
    }
    // end debug
  }
  return rxtask1.running;;
}

////////////////////////////////////////////////////// 
/**
 * Create variables for the usb to controlio converter itself */

void createVariables()
{ // create variables in RHD
  /// in degrees * 10
  sast.varSteeringAngleRef  = createVariable('w', 1, "steeringangleref");
  sast.varSteeringAngle  = createVariable('r', 1, "steeringangle");
  sast.varServoIntRef  = createVariable('r', 2, "servoref");
  sast.varIRVal  = createVariable('r', 6, "irval");
  sast.varBumper  = createVariable('r', 2, "bumper");
//  sast.varSteeringAngle  = createVariable('r', 1, "steeringangle");
//  sast.varCamPanTilt = createVariable('r', 2, "campantilt"); // actual speed in mm/s [left right)
//  sast.varCamPanTiltRef = createVariable('w', 2, "campantiltref"); // actual speed in mm/s [left right)
//  sast.varSafetyStop = createVariable('r', 1, "safetystop"); // 1 if stopped for safety - no master or switch pushed
  /// index to present servo position - in degrees * 10 relative to center
  // initialize other variables
  sast.varPortC = createVariable('r', 1, "portc"); // 1 if stopped for safety - no master or switch pushed
  sast.joyOverride = 0;
  printf(PLUGINNAME ": has created its read and write variables\n");
}

////////////////////////////////////////////////////////////////

int terminate(void)
{
  rxtask1.running = 0;
  printf(PLUGINNAME ": stopping ... \n");
  // stop motors
  setServos(0, 0);
  usleep(10000);
  // stop listening
  pthread_join(rxtask1.controlio_thread, NULL);
  printf(PLUGINNAME "steering thread stopped [OK]\n");
  if (gsLog != NULL)
    fclose(gsLog);
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
    sast.servos[i].center = 1; // < 100 is disabled (pulsed servo)
    sast.servos[i].vmax = 40950;
    sast.servos[i].name = "servo";
    sast.servos[i].scale = 24000.0/1800.0;
  }
  sast.varJoyOverride = -1;
  sast.varSteeringAngleRef = -1;
  sast.steerBase = 0.5; // distance between front and driving wheels
  sast.wheelBase = 0.3; // distance between driving wheels
  sast.frontBase = 0.3; // distance between front wheels
  sast.gearRatio = 1.0;   // encoder revolutions per wheel revolution
  sast.wheelDiameter = 0.14; // of (encoder) wheels
  busif1.baudrate=115200;
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
        if (strcmp("greensteer",el) == 0)
        { // get enable bit and device name
          const char * att;
          const char * val;
          printf(PLUGINNAME ": initializeing from XML file\n");
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i];
            val = attr[i + 1];
            // printf(PLUGINNAME ": initializeing from XML file, now %s=%s\n", att, val);
            if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
              info->enable = 1;
            else if (strcmp("dev1", att) == 0 || strcmp("dev", att) == 0)
              // interface device for steering
              strncpy(busif1.serialDev, val, MxDL);
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
            }
            else if (strcmp("deadzone", att) == 0)
            {
              sast.servos[0].deadzone = strtol(val, NULL, 0);
            }
            else if (strncmp("servo", att, 5) == 0)
            {
              char * p1 = (char *)val;
              int servoIdx = strtol(p1, &p1, 0);
              if (servoIdx >= 0 && servoIdx < SERVO_COUNT)
              {
                Servo * svo = &sast.servos[servoIdx];
                svo->servo = servoIdx;
                svo->vmin = strtol(p1, &p1, 0);
                svo->center = strtol(p1, &p1, 0);
                svo->vmax = strtol(p1, &p1, 0);
                svo->scale = strtod(p1, &p1);
                if (strlen(p1) > 0)
                {
                  strncpy(svo->nameStr, p1, SERVO_NAME_MAX);
                  svo->name = svo->nameStr;
                }
                printf(PLUGINNAME " servo %d from %.1f (%d) to %.1f (%d) center %d (0deg) %s\n", 
                       servoIdx, 
                       (svo->vmin-svo->center)/svo->scale, svo->vmin, 
                       (svo->vmax-svo->center)/svo->scale, svo->vmax,
                       svo->center, svo->name);
              }
              else
                printf(PLUGINNAME " servo index %d is not valid\n", servoIdx);
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
    fprintf(stderr,PLUGINNAME ": Can't open device 1 (steer): %s\n", busif1.serialDev);
//   else
//   { // set baudrate
//     result = (set_serial(busif1.ttyDev, busif1.baudrate) != -1);
//     if (result == 0)
//       fprintf(stderr,"   FSteer: Can't set first serial port parameters\n");
//   }
  if (result)
  { // set desired mode
    printf( PLUGINNAME ": opened %s successfully\n", busif1.serialDev);
    // ask for fast status from sensors
//     secure3Write(busif1.ttyDev, "s=1\n");
//     // debug
//     //secure3Write(busif1.ttyDev, "i=1\n");
//     if (gsLog != NULL)
//     {
//       gettimeofday(&logtime, NULL);
//       if (gsLog != NULL)
//         fprintf(gsLog, "%lu.%06lu if1 send servo 's=1' (push mode)", logtime.tv_sec, logtime.tv_usec);
//       fflush(gsLog);
//     }

    // debug end
    // get status every 5ms
    // secure2Write(busif1.ttyDev, "s=1\n", 5);
  }
  if (debugFlag)
  { // allow no device in debug mode
    result = 1;
    printf( PLUGINNAME ": debug flag set - some errors will be ignored, and more printout\n");
  }
  busif1.lostConnection = ! result;
  // initialize rx buffer to empty
  busif1.p2 = busif1.rxBuf;
  *busif1.p2 = '\0';
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
//  sast.safetyStop = 0;
  // debug
  gsLog = fopen("greensteer.log", "w");
  if (gsLog == NULL)
  {
    printf(PLUGINNAME "Failed to open logfile 'greensteer.log' - no rights? - continues without logging\n");
  }
  else
    fprintf(gsLog, "- version 300.1\n");
  // debug end
  // open device
  result = openAndSetModeDev1();
  if (result)
  { // start thread to handle bus
    // debug
    gettimeofday(&logtime, NULL);
    if (gsLog != NULL)
      fprintf(gsLog, "%lu.%06lu opened device 1\n", logtime.tv_sec, logtime.tv_usec);
    // end debug
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rxtask1.controlio_thread, &attr, controlio_task_1, 0))
    {
      perror(PLUGINNAME ": Can't start controlio 1 receive thread (steering)");
      result = 0;
    }
  }
  //if (result == 1 || debugFlag)
  { /****** Create database variables if all is ok **************/
    int waitCount = 0;
    // create RHD database variables
    createVariables();
    while (!rxtask1.running)
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
  }
  // debug
  gettimeofday(&logtime, NULL);
  if (gsLog != NULL)
    fprintf(gsLog, "%lu.%06lu init finished\n", logtime.tv_sec, logtime.tv_usec);
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
  int n, m;
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
      char * p1;
      p1 = busif1.rxBuf;
      while (*p1 < ' ')
        p1++;
      // most likely a usable status message
      // debug
      gettimeofday(&logtime, NULL);
      if (gsLog != NULL)
      {
        fprintf(gsLog, "%lu.%06lu if1 got %s\n", logtime.tv_sec, logtime.tv_usec, p1);
        fflush(gsLog);
      }
      // end debug
      switch (*p1)
      { // status messages
        case 'G': // fast status message
          p1++;
          m = 0;
          while (*p1 >= ' ' && m < 9)
          {
            int v;
            v = strtol(p1, &p1, 0);
            if (m < 2)
              setVariable(sast.varBumper, m, v);
            else if (m < 8)
              setVariable(sast.varIRVal, m - 2, v);
            else if (m < 9)
              setVariable(sast.varPortC, 0, v);
            if (*p1 == ',')
              p1++;
            m++;
          }
          break;
        default:
          break;
      }
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
  // tell RHD actual value
  setVariable(sast.varServoIntRef, 0, p0);
  setVariable(sast.varServoIntRef, 1, p1);
  // implement
  if ((p0 != sast.servos[0].positionRef) ||
      (p1 != sast.servos[1].positionRef))
  { // servo position needs to be updated
    char s[32];
    snprintf(s, 32, "K=%d,%d\n", p0, p1);
    secure3Write(busif1.ttyDev, s);
    sast.servos[0].positionRef = p0;
    sast.servos[1].positionRef = p1;
      // debug
    gettimeofday(&logtime, NULL);
    if (gsLog != NULL)
      fprintf(gsLog, "%lu.%06lu if1 send %s", logtime.tv_sec, logtime.tv_usec, s);
  // end debug

  }
  return isOK;
}


/**
 * Set servo position if changed, and send to servo - pan and tilt in degrees */
// int setPanTilt(float anglePan, float angleTilt)
// {
// #define ALL_SERVOS 0xfe
//   int isOK = 0;
//   int r0, r1, p0, p1;
// //   float ddeg;
// //   // average movement for movement time
// //   ddeg = (sast.servos[0].degreeRef - angleLeft +
// //           sast.servos[1].degreeRef - angleRight)/2.0;
//   // save angle ref for each servo (debug)
// //   anglePan  *= 180.0 / M_PI;
// //   angleTilt *= 180.0 / M_PI;
//   sast.servos[2].degreeRef = anglePan;
//   sast.servos[3].degreeRef = angleTilt;
//   // convert angle to servo units
//   r0 = roundi(anglePan  * sast.servos[2].scale + sast.servos[2].center);
//   r1 = roundi(angleTilt * sast.servos[3].scale + sast.servos[3].center);
//   // limit manoeuvre space
//   p0 = limitSigned(r0, sast.servos[2].vmin, sast.servos[2].vmax);
//   p1 = limitSigned(r1, sast.servos[3].vmin, sast.servos[3].vmax);
//   setVariable(sast.varServoIntRef, 0, p0);
//   setVariable(sast.varServoIntRef, 1, p1);
//   // implement
//   if ((p0 != sast.servos[2].positionRef) ||
//       (p1 != sast.servos[3].positionRef))
//   { // servo position needs to be updated
//     char s[32];
//     snprintf(s, 32, "P=%d,%d\n", p0, p1);
//     secure3Write(busif1.ttyDev, s);
// //     setVariable(sast.varCamPanTilt, 0, roundi(((p0 - sast.servos[2].center)/sast.servos[2].scale) * 10));
// //     setVariable(sast.varCamPanTilt, 1, roundi(((p1 - sast.servos[3].center)/sast.servos[3].scale) * 10));
//     sast.servos[2].positionRef = p0;
//     sast.servos[3].positionRef = p1;
//       // debug
//     gettimeofday(&logtime, NULL);
//     if (gsLog != NULL)
//       fprintf(gsLog, "%lu.%06lu if1 send %s", logtime.tv_sec, logtime.tv_usec, s);
//   // end debug
// 
//   }
//   return isOK;
// }
