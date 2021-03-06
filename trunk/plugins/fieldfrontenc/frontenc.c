 /** \file fieldsteer.c
 *  \ingroup hwmodule
 *
 * Interface for field robot steering and motor control
 *
 *
 *
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 478 $:"
 #define DATE             "$Date: 2014-04-15 12:21:35 +0200 (Tue, 15 Apr 2014) $:"
 #define ID               "$Id: frontenc.c 478 2014-04-15 10:21:35Z jcan $"
 #define PLUGINNAME        "FrontEnc"
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

#include "frontenc.h"

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
// #define MAX_SPEED_STR_CNT 32
//   char cmdStr[MAX_SPEED_STR_CNT];
  //gettimeofday(&tickTime, NULL);
  rxtask1.startNewRxCycle += 1;
  rxtask2.startNewRxCycle += 1;
  tick = rhdTick;
  if (sast.safetyStop)
  { // reenable motors
    printf(PLUGINNAME ":: safety stop over - sending E 1 1 to motors\n");
    sast.safetyStop = 0;
  }
  setVariable(sast.varSafetyStop, 0, sast.safetyStop);
  //
  // request steer status
  if (tick % 2 == 0)
  {
    secure3Write(busif1.ttyDev, "s\n");
    if (freLog != NULL)
    {
      gettimeofday(&logtime, NULL);
      fprintf(freLog, "%lu.%06lu if1 send s\n", logtime.tv_sec, logtime.tv_usec);
    }
    // end debug
  }
  return returnValue;
}

//////////////////////////////////////////////////////
/**
 * Create variables for the usb to controlio converter itself */

void createVariables()
{ // create variables in RHD
  sast.varEncFront = createVariable('r',3,"encfront"); // encoder value front wheels (left, right,tilt)
  sast.varEncMagnetState = createVariable('r', 3, "encfrontmagnet"); // state of encoder magnet (left, right, tilt), val: 0=OK, 1=moving away, 2=moving closer, 3= bad
  sast.varEncFrontErr = createVariable('r', 3, "encfronterr"); // error counter (parity error)
  sast.varEmergStopPushed = createVariable('r', 1, "emergencyswitch"); // 1 if emergenct stop pushed
  sast.varBattery = createVariable('r', 1, "battery"); // battery voltage
  // initialize other variables
  printf(PLUGINNAME ": has created its read and write variables\n");
}

////////////////////////////////////////////////////////////////

int terminate(void)
{
  rxtask1.running = 0;
  printf(PLUGINNAME ": stopping ... \n");
  // stop listening
  pthread_join(rxtask1.controlio_thread, NULL);
  printf(PLUGINNAME "steering thread stopped [OK]\n");
  if (freLog != NULL)
    fclose(freLog);
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
  int len;
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
  sast.enableFrontEncoder = 1;
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
        if (strcmp("frontenc",el) == 0)
        { // get enable bit and device name
          const char * att;
          const char * val;
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i];
            val = attr[i + 1];
            if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
              info->enable = 1;
            else if (strcmp("dev", att) == 0 || strcmp("dev", att) == 0)
              // interface device for steering
              strncpy(busif1.serialDev, val, MxDL);
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
            }
            else if (strcmp("frontencoder", att) == 0)
            {
              sast.enableFrontEncoder = strtol(val, NULL, 0);
            }
          }
          if (!info->enable)
            printf(PLUGINNAME ": Use is disabled in configuration\n");
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
    // reset interface - detect servos
    secure3Write(busif1.ttyDev, "reset\n");
    // debug
    //secure3Write(busif1.ttyDev, "i=1\n");
    if (freLog != NULL)
    {
      gettimeofday(&logtime, NULL);
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
    secure3Write(busif1.ttyDev, "x=0\n");
    printf( PLUGINNAME ": dynamixel not me anymore!\n");
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
  
  freLog = fopen("fre.log", "w");
  if (freLog == NULL)
  {
    printf(PLUGINNAME "Failed to open logfile 'fre.log' - no rights? - continues without logging\n");
  }
  else
  {
    fprintf(freLog, "- %s\n", REVISION);
    printf(PLUGINNAME ": opend logfile fre.log\n");
  }
  
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
  { /****** Create database variables if all is ok **************/
    int waitCount = 0;
    // create RHD database variables
    createVariables();
    while (!(rxtask1.running))
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
  if (freLog != NULL)
  {
    gettimeofday(&logtime, NULL);
    fprintf(freLog, "%lu.%06lu init finished\n", logtime.tv_sec, logtime.tv_usec);
  }
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
              case 'P': // encoder error
                n1 = strtol(p1, &p2, 0);
                n2 = strtol(++p2, &p2, 0);
                n3 = strtol(++p2, &p2, 0);
                setVariable(sast.varEncFrontErr, 0, n1);
                setVariable(sast.varEncFrontErr, 1, n2);
                setVariable(sast.varEncFrontErr, 2, n3);
                break;
              case 'N': // emergency stop switch and battery voltage
                n1 = strtol(p1, &p2, 16);
                setVariable(sast.varEmergStopPushed, 0, n1);
                n1 = strtol(++p2, &p2, 16);
                // AD converter has 36.5 count per volt (max 28.0V)
                n1=roundi(n1 / 3.65);
                setVariable(sast.varBattery, 0, n1);
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


