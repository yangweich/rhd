 /** \file greensteer.c
 *  \ingroup hwmodule
 *
 * Interface for green robot steering and some sensors
 *
 *
 *
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 359 $:"
 #define DATE             "$Date: 2013-12-22 09:36:24 +0100 (Sun, 22 Dec 2013) $:"
 #define ID               "$Id: greensteer.c 359 2013-12-22 08:36:24Z jcan $"
 #define PLUGINNAME        "Gr2nd"
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

#include "green2nd.h"

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
 * \param dev is the device integer
 * \param buf is the byte buffer with data to send
 * \param bufCnt is the number of bytes to send
 * \returns the number of bytes send, 0 if failed to send. */
ssize_t secure2Write(int dev, const void *buf, ssize_t txLen);
/**
 * calls secure2Write and supplies txlen from length of string 
 * \param dev is the device integer
 * \param buf is the byte buffer with data to send
 */
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
int debugFlag = 0;
/** flags for updating of write variables */
int settingUpd = 1;
struct timeval tickTime;
/**
 * left and right motor variables found?
 * */
int newServoPos;


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
  int i, v, vi;
  char s[120];
  int upd = 0;
  Servo * srv;
  //gettimeofday(&tickTime, NULL);
  rxtask1.startNewRxCycle += 1;
  tick = rhdTick;
  // increase idle clunt
  if (isUpdated('w', sast.varServoEnable))
  { // set pan-tilt if needed
    for (i = 0; i < SERVO_COUNT; i++)
    { // update servo position
      srv = &sast.servos[i];
      v = getWriteVariable(sast.varServoEnable, i);
      if (v != srv->enabled)
      {
        srv->enabled = v;
        if (!v)
          // set value to disable servo
          srv->positionRef = i + 1;
        else
        { // servo is enabled, so set reference value
          vi = srv->degreeRef * srv->scale + srv->center;
          vi = limitInt(vi, srv->vmin, srv->vmax);
          srv->positionRef = vi;
        }
        srv->idleCnt = 0;
        upd = 1;
      }
    }
  }
  if (isUpdated('w', sast.varServoAngRef))
  { // set new servo position as needed
    int vi;
    for (i = 0; i < SERVO_COUNT; i++)
    { // update servo position
      srv = &sast.servos[i];
      v = getWriteVariable(sast.varServoAngRef, i);
      if (srv->degreeRef != (float)v / 10.0)
      { // new value
        srv->degreeRef = v / 10.0;
        vi = roundi(srv->degreeRef * srv->scale) + srv->center;
        vi = limitInt(vi, srv->vmin, srv->vmax);
        if (srv->enabled)
        { // move servo timing value
          srv->positionRef = vi;
          upd = 1;
          srv->idleCnt = 0;
        }
      }
    }
  }
  // test for gone idle
  for (i = 0; i < SERVO_COUNT; i++)
  {
    srv = &sast.servos[i];
    if (srv->idleCnt == 100 && srv->enabled == 2)
    { // gone idle - disable
      upd = 1;
      srv->positionRef = i + 1;
    }
    srv->idleCnt++;
  }  
  // send new values to controller
  if (upd)
  { // at least one is changed
    snprintf(s, 120, "T%u,%u,%u,%u,%u,%u,%u,%u\n\r",
            sast.servos[0].positionRef,
            sast.servos[1].positionRef,
            sast.servos[2].positionRef,
            sast.servos[3].positionRef,
            sast.servos[4].positionRef,
            sast.servos[5].positionRef,
            sast.servos[6].positionRef,
            sast.servos[7].positionRef);
    secure3Write(busif1.ttyDev, s);
  }
  // set 
  for (i = 0; i < SERVO_COUNT; i++)
  { // update read variables
    Servo * srv = &sast.servos[i];
    setVariable(sast.varServoAng, i, srv->degreeRef * 10.0);
    setVariable(sast.varServoIntRef, i, srv->positionRef);
  }  
  if (1)
  { // request new IR status
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
  sast.varServoAngRef  = createVariable('w', SERVO_COUNT, "servo2angref");
  sast.varServoAng     = createVariable('r', SERVO_COUNT, "servo2ang");
  sast.varServoEnable  = createVariable('w', SERVO_COUNT, "servo2enable");
  sast.varIRVal  = createVariable('r', 4, "ad2val");
  sast.varServoIntRef  = createVariable('r', SERVO_COUNT, "servo2Intref");
  /// index to present servo position - in degrees * 10 relative to center
  // initialize other variables
  printf(PLUGINNAME ": has created its read and write variables\n");
}

////////////////////////////////////////////////////////////////

int terminate(void)
{
  rxtask1.running = 0;
  printf(PLUGINNAME ": stopping ... \n");
  // stop motors
  //setServos(0, 0);
  usleep(10000);
  // stop listening
  pthread_join(rxtask1.controlio_thread, NULL);
  printf(PLUGINNAME " stopped [OK]\n");
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
    Servo * srv = &sast.servos[i];
    srv->servo = i;
    srv->vmin = 0;
    srv->center = 24000;
    srv->vmax = 50000;
    srv->name = "servo";
    srv->scale = 16000.0/180.0;
    srv->nameStr[0] = '\0';
    srv->enabled = 0;
    srv->idleCnt = 0;
  }
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
        else
          // skip this group
          info->skip = info->depth;
        break;
      case 3:
        // this one handles interface to FSteer (Sabertooth and magnetic encoder)
        // through arduino interface - the only option is debug
        if (strcmp("green2nd",el) == 0)
        { // get enable bit and device name
          const char * att;
          const char * val;
          printf(PLUGINNAME ": initializeing from XML file\n");
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i];
            val = attr[i + 1];
            printf(PLUGINNAME ": initializeing from XML file, now %s=%s\n", att, val);
            if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
              info->enable = 1;
            else if (strcmp("dev1", att) == 0 || strcmp("dev", att) == 0)
              // interface device for steering
              strncpy(busif1.serialDev, val, MxDL);
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
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
                svo->enabled = strtod(p1, &p1);
                svo->idleCnt = 10000;
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
    fprintf(stderr,PLUGINNAME " Can't open device: %s\n", busif1.serialDev);
  else
  { // set baudrate
    result = (set_serial(busif1.ttyDev, busif1.baudrate) != -1);
    if (result == 0)
      fprintf(stderr,PLUGINNAME " FSteer: Can't set first serial port parameters\n");
  }
  if (result)
  { // set desired mode
    printf( PLUGINNAME " opened %s successfully\n", busif1.serialDev);
    // ask for fast status from sensors
  }
  if (debugFlag)
  { // allow no device in debug mode
    result = 1;
    printf( PLUGINNAME " debug flag set - some errors will be ignored, and more printout\n");
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
  // debug
//  gsLog = fopen("green2nd.log", "w");
  if (gsLog == NULL)
  {
    printf(PLUGINNAME "Failed to open logfile 'green2nd.log' - no rights? - continues without logging\n");
  }
  else
    fprintf(gsLog, "- plugin version %s\n", ID);
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
    fprintf(stderr, PLUGINNAME " signal: can't ignore SIGPIPE.\n");

  rxtask1.running = 1;
  fprintf(stderr, PLUGINNAME " rx_task 1 running\n");
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
        printf(PLUGINNAME " **** lost connection - trying to reconnect\n");
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
          while (p1 != NULL && m < 8)
          {
            int v;
            v = strtol(p1, &p1, 0);
            if (m < 4)
              setVariable(sast.varIRVal, m, v);
            if (*p1 == ',' && *p1 != 0)
              p1++;
            else
              p1 = NULL;
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




