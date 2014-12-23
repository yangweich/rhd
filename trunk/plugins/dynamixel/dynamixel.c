 /** \file saberandmagenc.c
 *  \ingroup hwmodule
 *
 * Interface for dynamixel servo series using the teensy controller in subdirectory
 *
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 281 $:"
 #define DATE             "$Date: 2013-10-20 17:33:20 +0200 (Sun, 20 Oct 2013) $:"
 #define ID               "$Id: dynamixel.c 281 2013-10-20 15:33:20Z jcan $"
 #define PLUGINNAME        "Dynamixel"
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

#include "dynamixel.h"

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

int tick = 0, updTick[SERVO_COUNT_MAX];
int debugFlag = 0;
struct timeval tickTime;
/// commands send to device at startup
#define INIT_CMDS_MAX 100
/// command array
char * initCmds[INIT_CMDS_MAX];
/// number of actual startup commands
int initCmdsCnt = 0;
/**
 * Set servo position if changed, and send to servo
 * \param servo is servo to set
 * \param value is desired wheel position in scaled units (rhd units)
 */
int setServos(int servo, int value);

////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int rhdTick)
{
  int returnValue = rxtask.running;
// #define MAX_SPEED_STR_CNT 32
//   char cmdStr[MAX_SPEED_STR_CNT];
  //gettimeofday(&tickTime, NULL);
  rxtask.startNewRxCycle += 1;
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
  if (isUpdated('w', sast.varServoRef) || tick == 1)
  { // ref position changed - calculate new angle for left and right wheel
    int i;
    for (i = 0; i < sast.servoCnt; i++)
    {
      int newRef = getWriteVariable(sast.varServoRef, i);
      // implement new servo positions
      if (tick > 1 && sast.servos[i].positionRefRhd != newRef)
        setServos(i, newRef);
      sast.servos[i].positionRefRhd = newRef;
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
  int servoMin, servoMax, i;
  sast.varServoRef  = createVariable('w', sast.servoCnt, "servoref");
  /// index to present servo position - in degrees * 10 relative to center
  sast.varServoPos = createVariable('r', sast.servoCnt, "servoPos");
  servoMin = createVariable('r', sast.servoCnt, "servoMin");
  servoMax = createVariable('r', sast.servoCnt, "servoMax");
  /// index to present servo speed
  sast.varServoVel = createVariable('r', sast.servoCnt, "servoVel");
  /// index to present servo load (force)
  sast.varServoLoad = createVariable('r', sast.servoCnt, "servoLoad");
  /// index to battery voltage
  sast.varServoVolt = createVariable('r', sast.servoCnt, "servoVolt");
  /// index to present servo temperature
  sast.varServoTemp = createVariable('r', sast.servoCnt, "servoTemp");
  /// servo communication error count
  sast.varServoComErrCnt = createVariable('r', sast.servoCnt, "servoComErr");
  sast.varServoIntRef = createVariable('r', sast.servoCnt, "servoIntRef");
  sast.varServoIntPos = createVariable('r', sast.servoCnt, "servoIntPos");
  sast.varServoUpdCnt = createVariable('r', sast.servoCnt, "servoUpdCnt");
  //
  sast.joyOverride = 0;
  printf(PLUGINNAME ": has created its read and write variables\n");
  //
  for ( i=0; i < sast.servoCnt; i++)
  { // set these values once
    servo * dxl = &sast.servos[i];
    setVariable(servoMin, i, (dxl->vmin - dxl->center) / dxl->scale);
    setVariable(servoMax, i, (dxl->vmax - dxl->center) / dxl->scale);
  }
}

////////////////////////////////////////////////////////////////

int terminate(void)
{
  int i;
  rxtask.running = 0;
  printf(PLUGINNAME ": stopping ... ");
  // stop motors - in either mode
  fflush(stdout);
  pthread_join(rxtask.controlio_thread, NULL);
  // release initial commands
  for (i = 0; i < initCmdsCnt; i++)
    free(initCmds[i]);
  printf("[OK]\n");
  return 0;
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
  for (i = 0; i < SERVO_COUNT_MAX; i++)
  {
    sast.servos[i].servo = i;
    sast.servos[i].vmin = 0;
    sast.servos[i].center = 2048;
    sast.servos[i].vmax = 4095;
    sast.servos[i].name = "servo";
    sast.servos[i].scale = 4096.0/360.0/10;
    sast.servos[i].updCnt = 0;
  }
  sast.servoCnt = 0;
  sast.varJoyOverride = -1;
  busif.baudrate=115200;
  for (i = 0; i < INIT_CMDS_MAX; i++)
    initCmds[i] = NULL;
  initCmdsCnt = 0;
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
        if (strcmp("dynamixel",el) == 0)
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
            }
            else if (strcmp("deadzone", att) == 0)
            {
              sast.servos[0].deadzone = strtol(val, NULL, 0);
            }
            else if (strncmp("servo", att, 5) == 0)
            {
              char * p1 = (char*)val;
              int id = strtol(&att[5], NULL, 0);
              if (id >= 0 && id < SERVO_COUNT_MAX)
              {
                if (id >= sast.servoCnt)
                  sast.servoCnt = id + 1;
                sast.servos[id].vmin = strtol(p1, &p1, 0);
                sast.servos[id].center = strtol(p1, &p1, 0);
                sast.servos[id].vmax = strtol(p1, &p1, 0);
                sast.servos[id].scale = strtof(p1, &p1);
              }
            }
          }
          if (!info->enable)
            printf(PLUGINNAME ": Use is disabled in configuration\n");
        }
        else
          info->skip = info->depth;
        break;
      case 4:
        if (strcmp("dxlinit",el) == 0)
        { // get enable bit and device name
          const char * att;
          const char * val;
          int n;
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i];
            val = attr[i + 1];
            if (strcmp("msg",att) == 0 && initCmdsCnt < INIT_CMDS_MAX)
            {
              n = strlen(val) + 1;
              initCmds[initCmdsCnt] = malloc(n);
              strncpy(initCmds[initCmdsCnt], val, n);
              initCmdsCnt++;
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

int openAndSetMode()
{
  int result = 0;
  int i, n;
#define CMD_LENGTH_MAX 200
  char s4[CMD_LENGTH_MAX];
  //
  busif.ttyDev = open(busif.serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = busif.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,PLUGINNAME ": Can't open device: %s\n", busif.serialDev);
  if (result)
  { // set desired mode
    printf( PLUGINNAME ": opened %s successfully\n", busif.serialDev);
    // get status every 5ms
    for (i = 0; i < initCmdsCnt; i++)
    {
      n = strlen(initCmds[i]);
      if (n < CMD_LENGTH_MAX - 3)
      { // make command string
        snprintf(s4, CMD_LENGTH_MAX, "%s\n", initCmds[i]);
        secure2Write(s4, n + 1);
        if (debugFlag)
          printf("%s init: %s", PLUGINNAME, s4);
      }
    }
  }
  if (debugFlag)
  { // allow no device in debug mode
    result = 1;
    printf( PLUGINNAME ": debug flag set - some errors will be ignored\n");
  }
  busif.lostConnection = ! result;
  // initialize rx buffer to empty
  busif.p2 = busif.rxBuf;
  *busif.p2 = '\0';
  //
  
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
      perror(PLUGINNAME ": Can't start controlio receive thread");
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
    if (!rxtask.running)
      perror(PLUGINNAME ": Read thread is not running");
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
    if (--*p3 > ' ')
      p3++;
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
//   int m = 0; // loop counter
//   int mWait = 12; // wait this many lines before testing status
//   const int MSL = 50;
//   char s[MSL];
  //
  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, PLUGINNAME ": signal: can't ignore SIGPIPE.\n");

  rxtask.running = 1;
  fprintf(stderr, PLUGINNAME ": rx_task running\n");
  // wait for RHD to init variables
  //usleep(300000);
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
        printf(PLUGINNAME ": **** lost connection - trying to reconnect\n");
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
    if (n > 3)
    { // new data available)
      char * p1, *p2;
      int n1;
      int servoIdx;

      //float speed;
      // debug
      //printf(PLUGINNAME ": %d get from device: %s\n", m, busif.rxBuf);
      // debug end
      p1 = busif.rxBuf;
      while (*p1 < ' ')
        p1++;
      //debug
      //printf("%s\n", p1);
      //debug end
      // most likely a usable status message
      switch (*p1)
      { // status messages
        case 'Y': // fast status message
          //printf("%s\n", p1);
          while (*p1 != '\0')
          {
            servo * dxl;
            switch (*p1++)
            {
              case 'Y': // encoder values
                servoIdx = strtol(p1, &p2, 0);
                if ((p1 != p2) && (servoIdx >= 0) && (servoIdx < sast.servoCnt)) // && tick != updTick[servoIdx])
                { // servo number is OK - read status
                  updTick[servoIdx] = tick;
                  //debug
                  n1 = ++sast.servos[servoIdx].updCnt;
                  setVariable(sast.varServoUpdCnt, servoIdx, n1);
                  // debug end
                  dxl = &sast.servos[servoIdx];
                  n1 = strtol(++p2, &p2, 16);
                  setVariable(sast.varServoIntPos, servoIdx, n1);
                  setVariable(sast.varServoPos, servoIdx, (n1 - dxl->center) / dxl->scale);
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
          printf("%s %s\n", PLUGINNAME, busif.rxBuf);
          break;
      }
    }
//     if (m > mWait)
//     { // ensure general setting is OK
//       int flushAndWrite = 0;
//       if (sast.datarateIs < 0)
//       {
//         snprintf(s, MSL, "h\n\r");
//         flushAndWrite = 1;
//       }
//       else
//       { // test if there is changes
//       }
//       if (flushAndWrite)
//       { // flush any waiting data
//         int f = 0;
//         while (getNewLine(1, 1) > 0)
//           printf(".%d.", f++); // discard line
//         secure3Write(s);
//       }
//       if (sast.datarate == 0)
//         mWait = m + 10;
//       else
//         mWait = m + 10 + 100/sast.datarate;
//     }
//     m++;
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
  {
    return secureWrite(busif.ttyDev, buf, txLen);
  }
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
int setServos(int servo, int value)
{
#define ALL_SERVOS 0xfe
  int isOK = 0;
  int r0, p0;
  // convert angle to servo units
  r0 = roundi(value * sast.servos[servo].scale + sast.servos[servo].center);
  // limit manoeuvre space
  p0 = limitSigned(r0, sast.servos[servo].vmin, sast.servos[servo].vmax);
  // implement
  if (p0 != sast.servos[servo].positionRef)
  { // servo position needs to be updated
    char s[32];
    snprintf(s, 32, "W=%d,30,%d\n", servo, p0);
    secure2Write(s, strlen(s));
    setVariable(sast.varServoIntRef, servo, p0);
//    if (debugFlag)
      printf("%s %s", PLUGINNAME, s);
  }
  return isOK;
}
