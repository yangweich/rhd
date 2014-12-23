 /** \file buspirate.c
 *  \ingroup hwmodule
 *
 *   interface to bus pirate - using its 10 bit AD converter
 *   http://dangerousprototypes.com/docs/Bitbang
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2011-07-02 08:12:03 +0200 (Sat, 02 Jul 2011) $:"
 #define ID               "$Id: bupi_ad.c 59 2012-10-21 06:25:02Z jcan $"
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

//RHD Core headers
#include <rhd.h>
//#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "bupi_ad.h"

#define LS_READ_BYTES 20
#define SETT_LEN 5
///Struct for shared parse data
typedef struct  {
    int depth; // current XML tag level
    int skip;  // skip from this level up
    char enable;
    char found;
  } parseInfo;

/* prototypes */
/**
 * Create exchange variables */
void createRHDvariables();
/**
 * initialize plugin */
int initPlugin(void);
/** poll to see if a read would not block.
 * \param fd is the device number
 * \param msTimeout is the maximum wait time
 * \returns 0 is no data is available and 1 if at least 1 byte is available */
int pollDeviceRx(int fd, int msTimeout);
/** run thread for rx task */
void * read_task(void *);
/**
 * Parsing function for start tag */
void XMLCALL lsStartTag(void *, const char *, const char **);
/**
 * Parsing function for end tag */
void XMLCALL lsEndTag(void *, const char *);
/**
 * get data from device
 * \param minCnt is the minimum number of bytes to fetch
 * \param timeoutms is the timeout used when waiting for data
 * \param maxWaitCnt is the max number of poll cycles to wait for required amount of data
 * \returns number of data received */
int getData(int minCnt, int timeoutms, int maxWaitCnt);
/**
 * Get data from line until timeout.
 * \param dest is where data should be stored
 * \param destCnt is length of destination buffer
 * \param timeout is the max time to wait for data
 * \returns when 2 timeoutperiods has expired with no data or
 * when data is received (within two timeout periods) and there have been no new data within one timeout period or
 * when there is no more space or just one character left in destination buffer. */
//int getDataToTimeout(char * dest, int destCnt, int timeoutms);
/**
 * get index to a named variable - from another plugin
 * \param type is either 'r' or 'w'
 * \param name is the name of the variable
 * \returns -1 if the variable is not found, else the index (positive or zero) of the variable. */
int getDatabaseVariable(char type, const char * name);
/**
 * get floating point value from two integer values */
double getVariableDouble(char type, int index);
/**
 * Set double value in 2 integers */
int setVariableDouble(char type, int index, double value);
/**
 * Round to an integer value - also for negative values. */
int roundi(const float v)
{ // round to closest integer
  if (v > 0)
    return (int)(v + 0.5);
  else
    return (int)(v - 0.5);
}
/**
 * return minimum integer of these two */
int mini(int a, int b)
{
  if (a < b)
    return a;
  else
    return b;
}
/**
 * Reset update flag for one read variable */
void resetUpdateRead(int id);
/** catch writes to a non existing file - allowed in debug mode
 * \param buf is the string to write
 * \param bufCnt is number of bytes to write
 * \returns the number of characters written and sets the 'lostConnection flag if
 * no data is written, allows write if no device - used in debug mode */
ssize_t secure2Write(const void *buf, ssize_t bufCnt)
{
  int nw = bufCnt; // number of characters written
  if (devif.ttyDev >= 0)
    nw = secureWrite(devif.ttyDev, buf, bufCnt);
  if (nw < 0 || nw < bufCnt)
    devif.lostConnection = 1;
  return nw;
}
/**
 * write all bytes to device using a zero terminated string
 * \param str is the string to write
 * \returns the number of characters written and sets the 'lostConnection flag if
 * no data is written, allows write if no device - used in debug mode */
ssize_t secure3Write(const void * str)
{
  int len = strlen(str);
  int nw = len; // number of characters written
  if (devif.ttyDev >= 0)
    nw = secureWrite(devif.ttyDev, str, len);
  if (nw < 0 || nw < len)
    devif.lostConnection = 1;
  return nw;
}
/** absolute value of long integers */
//int64_t llabs(int64_t val)
//{
//  if (val < 0)
//    return -val;
//  else
//    return val;
//}
/**
 * Get time since passed in us since this reference time
 * \param refTime reference time
 * \returns time passed in microseconds. */
int32_t getTimePassed(struct timeval refTime)
{
  struct timeval tv;
  //
  gettimeofday(&tv, NULL);
  int ds = tv.tv_sec - refTime.tv_sec;
  if (ds > INT32_MAX / 1000000)
    return INT32_MAX;
  else
  {
    ds = ds * 1000000 + tv.tv_usec - refTime.tv_usec;
    return ds;
  }
}

/******** Global variables *************/

/// variables related to thread execution
struct
{ /** is receive thread running */
  int running;
  /** Trigger polling of data from units - when set to 1 */
  int startNewRxCycle;
  /** name of serial device */
  /** thread handle and attributes */
  pthread_t read_thread;
} rxtask;

int keepAlive = 0;
int debugFlag = 0;
FILE * logState = NULL;

/////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int tick)
{
  int returnValue = 1;
  rxtask.startNewRxCycle = 1;
  // debug
  // printf("tick %d - write = %d\n", tick, keepAlive);
  // debug end
  return returnValue;
}

//////////////////////////////////////////////////////

int terminate(void)
{
  rxtask.running = 0;
  printf("BUPI::%s: stopping ... ", bupi.basename);
  fflush(stdout);
  pthread_join(rxtask.read_thread, NULL);
  close(devif.ttyDev);
  printf("[OK]\n");
  return 0;
}

/************************** XML Initialization **************************/


/** \brief Initialize plug-in from XML configuration file
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
  //Find revision number from SVN Revision
  char * p1;
  char versionString[20] = REVISION;
  char tempString[10];
  p1 = strrchr(versionString, '$');
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  printf("BUPI: Initializing plug-in version %s.%s\n",REVISION, tempString);
  /* Initialize Expat parser*/
  XML_Parser parser = XML_ParserCreate(NULL);
  result = parser != 0;
  strncpy(bupi.basename, "gdm0", MBL);
  devif.baudrate = 9600;
  strncpy(devif.devName, "/dev/ttyUSB0", MxDL);
  if (!result)
    fprintf(stderr, "BUPI: Couldn't allocate memory for XML parser\n");
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
      printf("BUPI: Error reading: %s\n",filename);
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
        fprintf(stderr, "   BUPI: Couldn't allocate memory for XML File buffer\n");
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
      fprintf(stderr, "BUPI: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
  }
  if (parser != NULL)
    XML_ParserFree(parser);
  if (xmlBuf != NULL)
    free(xmlBuf);
  if (result && xmlParse.enable)
  { // all is fine - start plugin
    result = initPlugin();
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
        if (strcmp("rhd",el) != 0)
          info->skip = info->depth;
        break;
      case 2:
        if (strcmp("plugins",el) == 0)
          ;
        else
          info->skip = info->depth;
        break;
      case 3:
        // this one handles bs pirate stuff only
        if (strcmp("bupiad",el) == 0)
        { // is it enabled, the only info needed
          for(i = 0; attr[i]; i+=2)
          {
            const char * att, * val;
            att = attr[i];
            val = attr[i + 1];
            if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
              info->enable = 1;
            else if (strcasecmp("dev", att) == 0)
            {
              strncpy(devif.devName, val, MxDL);
            }
            else if (strcasecmp("baudRate", att) == 0)
            {
              devif.baudrate = strtol(val, NULL, 10);
            }
            else if (strcasecmp("valName", att) == 0)
            {
              strncpy(bupi.basename, val, MBL);
            }
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
              if (debugFlag)
                printf("   BUPI-AD %s started in DEBUG mode!\n", bupi.basename);
            }
          }
          if (!info->enable)
            printf("   BUPI-AD:%s Use is disabled in configuration\n", bupi.basename);
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

int openDeviceAndSetMode()
{
  int result = 0;
  int state = 0;
  int loop = 0;
  //
  devif.ttyDev = open(devif.devName, O_RDWR /*| O_NONBLOCK*/);
  result = devif.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,"   BUPI-AD::%s Can't open device: %s\n", bupi.basename, devif.devName);
  else
  { // set baudrate
    result = (set_serial(devif.ttyDev, devif.baudrate) != -1);
    if (result == 0)
      fprintf(stderr,"   BUPI-AD::%s, Can't set serial port parameters\n", bupi.basename);
  }
  if (debugFlag)
    // allow no device in debug mode
    result = 1;
  //
  if (result)
    devif.lostConnection = 0;
  // set 20 zeros to send to bus-pirate - should ensure reset state
  while (result == 1)
  { // get name and version
    int n;
    const char * p1;
    switch (state)
    {
      case 0: // searching for reset or menu state
      case 1: // in reset state
      case 2: // in reset state
        // reset device and wait for first line
        if (state == 0)
          n = secure3Write("#\n");
        else
          n = secure3Write("m\n");
        if (n > 0)
        { // data written search result
          n = getData(MxBL-1, 50, 5);
          // p2 = devif.rxBuf;
          //printf("BUPI:\n%s\n", p2);
        }
        if (n > 0)
        { // analyze data
          p1 = strstr(devif.rxBuf, "Bus Pirate");
          if (p1 != NULL)
          { // found reset message
            printf("BUPI:: got reset menu\n");
            state = 1;
          }
          p1 = strstr(devif.rxBuf, "Protocol interaction");
          if (p1 != NULL)
          { // general help page
            printf("BUPI:: got short cut key list\n");
            state = 2;
          }
          p1 = strstr(devif.rxBuf, "4. I2C");
          if (p1 != NULL)
          { // found menu list with i2c possibility
            printf("BUPI: got menu: with i2c point\n");
            state = 3;
          }
        }
        loop++;
        if (loop > 50)
          devif.lostConnection = 1;
        break;
      case 3: // got menu entry 4 (i2c)
        // switch to mode 4
        printf("BUPI: switching to i2c");
        n = secure3Write("4\n");
        if (n > 0)
        {
          n = getData(MxBL - 1, 20, 5);
        }
        if (n > 0)
        { // analyze data
          p1 = strstr(devif.rxBuf, "speed");
          if (p1 != NULL)
          {
            state = 4;
            printf(" [OK]\n");
          }
          else
          {
            state = 0;
            printf(" [fail]\n");
          }
        }
        break;
      case 4:
        // select speed
        printf("BUPI: switching to 50kHz mode");
        n = secure3Write("2\n");
        if (n > 0)
          n = getData(MxBL - 1, 50, 5);
        if (n > 0)
        { // analyze data
          p1 = strstr(devif.rxBuf, "I2C>");
          if (p1 != NULL)
          {
            state = 6;
            printf(" [OK]\n");
          }
          else
          {
            state = 0;
            printf(" [fail]\n");
          }
        }
        break;
      case 5:
        // go to sniffer mode
        printf("BUPI: sniffer mode");;
        n = secure3Write("(2)\n");
        if (n > 0)
          n = getData(MxBL-1, 50, 5);
        if (n > 0)
        { // analyze data
          p1 = strstr(devif.rxBuf, "Sniffer");
          if (p1 != NULL)
          {
            state = 9;
            printf(" [OK]\n");
          }
          else
          {
            state = 0;
            printf(" [fail]\n");
          }
        }
        break;
      case 6:
        // turn on 5V
        printf("BUPI-AD: power on (5V)");;
        n = secure3Write("W\n");
        if (n > 0)
          n = getData(MxBL - 1, 50, 5);
        if (n > 0)
        { // analyze data
          p1 = strstr(devif.rxBuf, "Power");
          if (p1 != NULL)
          {
            state = 7;
            printf(" [OK]\n");
          }
          else
          {
            state = 0;
            printf(" [fail]\n");
          }
        }
        break;
      case 7:
        // turn on 5V
        printf("BUPI-AD: AD");;
        n = secure3Write("D\n");
        if (n > 0)
          n = getData(MxBL-1, 50, 5);
        if (n > 0)
        { // analyze data
          p1 = strstr(devif.rxBuf, "V");
          if (p1 != NULL)
          {
            state = 9;
            printf(" '%s' [OK]\n", devif.rxBuf);
          }
          else
          {
            state = 0;
            printf(" [fail]\n");
          }
        }
        break;
      default:
        state = 0;
        break;
    }
    if (state == 9 || devif.lostConnection)
      break;
  }
  return result;
}

//////////////////////////////////////////////

/**
 * Initialize the communication and start rx thread
 * \returns 1 on success (else 0) */
int initPlugin(void)
{ //Open first serial port
  int result;
  //
  rxtask.running = 0;
  rxtask.startNewRxCycle = 0;
  // open device
  result = openDeviceAndSetMode();
  if (result)
  {
    createRHDvariables();
  }
  if (result == 1)
  { // start thread to handle bus
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rxtask.read_thread, &attr, read_task, 0))
    {
      perror("   BUPI-AD: Can't start receive thread");
      result = 0;
    }
  }
  if (result == 1)
  { /****** Create database variables if all is ok **************/
    int waitCount = 0;
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

unsigned char limitUnsignedChar(int value, int min, int max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}

/////////////////////////////////////////////////////

int maxi(int a, int b)
{
  if (a > b)
    return a;
  else
    return b;
}

/////////////////////////////////////////////////////


// int getDataToTimeout(char * dest, int destCnt, int timeoutms)
// {
//   int n = 0;
//   char * p1 = dest;
//   int toMult = 2; // timeout multiplier on first timeout
//   //
//   *p1 = '\0'; // clear result (to improve debug);
//   while (1)
//   { // wait for data
//     int dataOK;
//     int m;
//     //
//     if (destCnt - n <= 1)
//       // no more space
//       break;
//     // wait for data
//     dataOK = pollDeviceRx(devif.ttyDev, timeoutms * toMult);
//     if (dataOK)
//     { // there is data, so get it
//       m = read(devif.ttyDev, p1, destCnt - n - 1);
//       if (m > 0)
//       { // got some data - search for new-line
//         n += m;
//         p1 = &devif.rxBuf[n];
//       }
//       else if (m < 0)
//       { // poll error - device is removed?
//         printf("BUPI::%s Read error from device - connection lost\n", bupi.basename);
//         devif.lostConnection = 1;
//       }
//     }
//     else
//     { // we have a timeout - no more data
//       // zero terminate
//       *p1 = '\0';
//       break;
//     }
//     toMult = 1;
//   }
//   return n;
// }

////////////////////////////////////////////////////

///RS232  Recieve thread
void * read_task(void * not_used)
{ // run in thread
  int cn, loopCnt = 0;
  struct timeval tm;
  struct timeval startTime;
  int adCnt = 0; // control updates
  char * p1;
  int idleCnt = 0;
  //
  if (0)
  {
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
    {
      perror("mlockall");
      exit(-1);
    }

    { /* use real-time (fixed priority) scheduler
      * set priority to one less than the maximum
      */
      struct sched_param param;

      param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
      if (sched_setscheduler(0, SCHED_RR, &param)) {
          perror("setscheduler");
          pthread_exit(0);
      }

    }
  }

  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, "   BUPI-AD: signal: can't ignore SIGPIPE.\n");

  fprintf(stderr, "   BUPI-AD: read task running\n");
  if (debugFlag)
  {
    char fn[100];
    snprintf(fn, 100, "%s.log", bupi.basename);
    logState = fopen(fn, "w");
  }
  if (logState != NULL)
  {
    gettimeofday(&tm, NULL);
    fprintf(logState, "bus pirate logging\n");
  }
  //Mark thread as running
  rxtask.running = 1;
  gettimeofday(&startTime, NULL);
  p1 = devif.rxBuf;
  cn = 0;
  while (rxtask.running)
  { // maintain interface
    char * p2 = devif.rxBuf, *p3;
    loopCnt++;
    // request new data
    cn = getData(10, 30, 3);
    if (cn <= 0)
    {
      idleCnt++;
      if (idleCnt > 100)
      { // the connection is lost
        devif.lostConnection = 1;
      }
    }
    if (cn >= 9)
    { // data to analyze
      double v;
      p1 = strchr(p2, '\b');
      if (p1 != NULL)
      {
        while (*p1 < ' ')
          p1++;
        v = strtod(p1, &p3);
        if (*p3 == 'V')
        {
          //gettimeofday(&tm, NULL);
          // set update counts
          setVariable(bupi.varAd, 0, round(v/6.6*1024));
          setVariableDouble('r', bupi.varAdV, v);
          setVariable(bupi.varCnt, 0, adCnt++);
        }
      }
    }
    if (adCnt %100 == 0)
    {
      int32_t us;
      us = getTimePassed(startTime);
      setVariable(bupi.varAdHz, 0, 100000000/us);
      gettimeofday(&startTime, NULL);
    }
    if (devif.lostConnection)
    {
      printf("BUPI-AD: lost connection - tries to reopen the connection to %s\n", devif.devName);
      close(devif.ttyDev);
      sleep(1);
      openDeviceAndSetMode();
      cn = 0;
      p1 = devif.rxBuf;
    }
  }
  rxtask.running = 0;
  fprintf(stderr,"BUPI-AD: closing bus device\n");
  close(devif.ttyDev);
  fprintf(stderr,"BUPI-AD: Shutting down thread\n");
  if (logState != NULL)
    fclose(logState);
  logState = NULL;
  pthread_exit(0);
  return NULL;
}

///////////////////////////////////////////////////////////

/**
 * Create variables for the usb to i2c converter itself */
void createRHDvariables()
{
  char vn[100];
  snprintf(vn, 100, "%s", bupi.basename);
  bupi.varAd = createVariable('r', 1, vn);
  snprintf(vn, 100, "%sV", bupi.basename);
  bupi.varAdV = createVariable('r', 2, vn);
  snprintf(vn, 100, "%sN", bupi.basename);
  bupi.varCnt = createVariable('r', 1, vn);
  snprintf(vn, 100, "%sHz", bupi.basename);
  bupi.varAdHz = createVariable('r', 1, vn);
}

////////////////////////////////////////////////////////////////

int pollDeviceRx(int fd, int msTimeout)
{
  int err = 0;
  int dataAvailable = 0;
  struct pollfd pollStatus;
  int revents;
  //
  if (fd == -1)
  { // no device (debugging mode)
    usleep(50000);
  }
  else
  { // test if there is available data
    pollStatus.fd = fd;
    pollStatus.revents = 0;
    pollStatus.events = POLLIN  |  /*  0x0001  There is data to read */
                        POLLPRI;   /*  0x0002  There is urgent data to read */
                                  /* POLLOUT 0x0004  Writing now will not block */
    //
    err = poll(&pollStatus, 1, msTimeout);
    if (err < 0)
    { // not a valid call (may be debugger interrrupted)
      perror("UServerPort::serviceClients (poll)");
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

//////////////////////////////////////////////////

int setVariableDouble(char type, int index, double value)
{ // get type double value from 2 integer values - assuming integer part and micro decimal value.
  double result;
  if (type == 'r')
  { //
    int v = (int) value;
    setVariable(index, 0, v);
    v = (int)((value - v) * 1e6);
    result = setVariable(index, 1, v);
  }
  else
  {
    int v = (int) value;
    writeValue(index, 0, v);
    v = (int)((value - v) * 1e6);
    result = writeValue(index, 1, v);
  }
  return result;
}

/////////////////////////////////////////////////

double getVariableDouble(char type, int index)
{ // get type double value from 2 integer values - assuming integer part and micro decimal value.
  int v, uv;
  double result;
  if (type == 'r')
  {
    v = getReadVariable(index, 0);
    uv =   getReadVariable(index, 1);
  }
  else
  {
    v = getWriteVariable(index, 0);
    uv = getWriteVariable(index, 1);
  }
  result = v + uv * 1e-6;
  return result;
}

/////////////////////////////////////

void resetUpdateRead(int id)
{
  symTableElement * syms;
  int symsCnt;
  //
  syms = getSymbolTable('r');
  symsCnt = getSymtableSize('r');
  //
  if (id >= 0 && id < symsCnt)
    syms[id].updated = 0;
}

int getData(int minCnt, int timeoutms, int maxWaitCnt)
{
  int l = 0;
  int n = 0;
  char * p1 = devif.rxBuf;
  // wait for reply - up to 500ms
  while (l < maxWaitCnt)
  {
    int dataOK;
    int m;
    l++; // loop counter
    // wait for data
    dataOK = pollDeviceRx(devif.ttyDev, timeoutms);
    if (dataOK)
    { // there is data
      m = read(devif.ttyDev, p1, MxBL - n - 1);
      if (m > 0)
      {
        n += m;
        p1 = &devif.rxBuf[n];
        *p1 = '\0';
      }
      else if (m < 0)
      { // poll error - device is removed?
        printf("BUPI-AD:: Read error from device - device is lost\n");
        devif.lostConnection = 1;
        break;
      }
    }
    if (n >= minCnt)
      break;
  }
  //if (l > 1)
  //  printf("getData: got %d bytes in %d pools (timeout=%dms maxLoop=%d\n",
  //          n, l, timeoutms, maxWaitCnt);
  return n;
}
