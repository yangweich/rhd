 /** \file gdm8246.c
 *  \ingroup hwmodule
 *
 *   Interface to digital multimeter  type GDM-8246 from GW INSTEK
 *
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2011-07-02 08:12:03 +0200 (Sat, 02 Jul 2011) $:"
 #define ID               "$Id: gdm8246.c 59 2012-10-21 06:25:02Z jcan $"
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

#include "gdm8246.h"

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
void createI2Cvariables();
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
 * Get data from instrument from this command
 * \param cmd is command to send to instrument.
 * \param timeoutms total timeout tome if no reply (with a linefeed) - must be > 100
 * \returns true (1) if data is available in devif.rxBuf.
 * returned data is zero terminated string with newline removed.  */
int getData(const char * cmd, int timeoutms);
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
/** catch writes to a non existing file - allowed in debug mode */
ssize_t secure2Write(int fd, const void *buf, ssize_t txLen)
{
  if (fd >= 0)
    return secureWrite(fd, buf, txLen);
  else
    return 0;
}
/** absolute value of long integers */
// int64_t i64abs(int64_t val)
// {
//   if (val < 0)
//     return -val;
//   else
//     return val;
// }

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
FILE * logfile = NULL;
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
  printf("GDM8246::%s: stopping ... ", gdm.basename);
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
  printf("GDM8246: Initializing plug-in version %s.%s\n",REVISION, tempString);
  /* Initialize Expat parser*/
  XML_Parser parser = XML_ParserCreate(NULL);
  result = parser != 0;
  strncpy(gdm.basename, "gdm0", MBL);
  devif.baudrate = 9600;
  strncpy(devif.devName, "/dev/ttyUSB0", MxDL);
  if (!result)
    fprintf(stderr, "GDM8246: Couldn't allocate memory for XML parser\n");
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
      printf("GDM8246: Error reading: %s\n",filename);
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
        fprintf(stderr, "   GDM8246: Couldn't allocate memory for XML File buffer\n");
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
      fprintf(stderr, "GDM8246: XML Parse error at line %d: %s\n",
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
        // this one handles gdm8246 only
        if (strcmp("gdm8246",el) == 0)
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
            else if (strcasecmp("sampleRate", att) == 0)
            {
              gdm.sampleRate = strtod(val, NULL);
            }
            else if (strcasecmp("valName", att) == 0)
            {
              strncpy(gdm.basename, val, MBL);
            }
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
              if (debugFlag)
                printf("   GDM8246 %s started in DEBUG mode!\n", gdm.basename);
            }
          }
          if (!info->enable)
            printf("   GDM8246:%s Use is disabled in configuration\n", gdm.basename);
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
  devif.lostConnection = 1;
  // open device
  devif.ttyDev = open(devif.devName, O_RDWR /*| O_NONBLOCK*/);
  result = devif.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,"   GDM8246::%s Can't open device: %s\n", gdm.basename, devif.devName);
  else
  { // set baudrate
    result = (set_serial(devif.ttyDev, devif.baudrate) != -1);
    if (result == 0)
      fprintf(stderr,"   GDM8246::%s, Can't set serial port parameters\n", gdm.basename);
  }
  if (debugFlag)
    // allow no device in debug mode
    result = 1;
  //
  if (result == 1)
  { // get name and version
    devif.lostConnection = 0;
    printf("GDM8246::%s: starting \n", gdm.basename);
    result = getData("*idn?", 2000);
    if (result)
      fprintf(stdout, "# *idn? gave OK reply '%s'\n", devif.rxBuf);
    else
      fprintf(stdout, "# *idn? failed '%s'\n", devif.rxBuf);
    // get actual setting (DCV or otherwise)
    result = getData(":configure:function?", 2000);
    if (result)
    {
      fprintf(stdout, "# :configure:function? gave OK reply '%s'\n", devif.rxBuf);
      strncpy(gdm.setting, devif.rxBuf, MBL);
    }
    else
      fprintf(stdout, "# :configure:function? failed '%s'\n", devif.rxBuf);
  }
  if (result)
  {
    int i;
    createI2Cvariables();
    for (i = 0; i < mini(strlen(gdm.setting), SETT_LEN); i++)
      setVariable(gdm.varSetting, i, gdm.setting[i]);
    for (; i < SETT_LEN; i++)
      setVariable(gdm.varSetting, i, 0);
  }
  if (result == 1)
  { // start thread to handle bus
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rxtask.read_thread, &attr, read_task, 0))
    {
      perror("   gdm8246: Can't start i2c receive thread");
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

int getData(const char * cmd, int timeoutms)
{
  int gotData = 0;
  int l = 0;
  int n;
  char * p1 = devif.rxBuf;
  // write command to device
  n = secure2Write(devif.ttyDev, cmd, strlen(cmd));
  n += secure2Write(devif.ttyDev, "\n", 1);
  if (n <= 0)
  {
    devif.lostConnection = 1;
    return 0;
  }
  n = 0;
  // wait for reply - up to 500ms
  while (l < maxi(1, timeoutms / 100))
  {
    int dataOK;
    int m;
    // wait for data
    dataOK = pollDeviceRx(devif.ttyDev, 100);
    if (dataOK)
    { // there is data
      m = read(devif.ttyDev, p1, MxBL - n - 1);
      if (m > 0)
      { // got some data - search for new-line
        char * p2;
        p1[m] = '\0';
        p2 = strchr(p1, '\n');
        if (p2 != NULL)
        { // strip newline from data
          while (*p2 < ' ' && p2 > devif.rxBuf)
            *p2-- = '\0';
          gotData = 1;
          break;
        }
        n += strlen(p1);
        p1 = &devif.rxBuf[n];
      }
      else if (m < 0)
      { // poll error - device is removed?
        printf("GDM8246::%s Read error from device - device is closed - RHD needs restart\n", gdm.basename);
        rxtask.running = 0;
      }
    }
    l++; // loop counter
  }
  return gotData;
}


////////////////////////////////////////////////////

///RS232  Recieve thread
void * read_task(void * not_used)
{ // run in thread
  int loopCnt = 0;
  struct timeval tm;
  int sampleInterval; // in ms
  struct timeval startTime;
  int updCnt = 0;
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
    fprintf(stderr, "   GDM8246: signal: can't ignore SIGPIPE.\n");

  fprintf(stderr, "   GDM8246: read task running\n");
  if (debugFlag)
  {
    char fn[100];
    snprintf(fn, 100, "%s.log", gdm.basename);
    logfile = fopen(fn, "w");
    snprintf(fn, 100, "%s-raw.log", gdm.basename);
    logState = fopen(fn, "w");
  }
  if (logfile != NULL)
  {
    gettimeofday(&tm, NULL);
    fprintf(logfile, "time value\n");
  }
  if (logState != NULL)
  {
    gettimeofday(&tm, NULL);
    fprintf(logState, "time state logging\n");
  }
  //Mark thread as running
  rxtask.running = 1;
  devif.txBuf[0] = '\0';
  if (gdm.sampleRate <= 10)
    sampleInterval = roundi(1000.0/gdm.sampleRate);
  else if (gdm.sampleRate > 1e-3)
    sampleInterval = roundi(1000.0/gdm.sampleRate);
  else
    sampleInterval = 1000;
  gettimeofday(&startTime, NULL);
  while (rxtask.running)
  { // maintain interface
    int upd; /*, deb;*/
    struct timeval time;
    long toTime, nowTime, m; /* time in miliseconds */
    struct timespec waitTime;
    const int s2msec = 1000;
    const int m2nsec = 1000000;
    //char * p1;
    double v;
    //
    loopCnt++;
    gettimeofday(&time, NULL);
    //p1 = devif.rxBuf;
    // request new data
    upd = getData(":val?", 500);
    if (upd)
    {
      if (logState != NULL)
        fprintf(logState, "%d.%06d %s\n", (int)time.tv_sec, (int)time.tv_usec, devif.rxBuf);
      printf("%s at %d.%06d :':val?' -> '%s'\n", gdm.basename, (int)time.tv_sec, (int)time.tv_usec,  devif.rxBuf);
      if (devif.rxBuf[0] == '+' || devif.rxBuf[0] == '-')
      {
        v = strtod(devif.rxBuf, NULL);
        setVariableDouble('r', gdm.varVal, v);
        updCnt++;
        setVariable(gdm.varCnt, 0, updCnt);
      }
      gettimeofday(&time, NULL);
    }
    // wait for next sample time
    // toTime in ms
    toTime = loopCnt * sampleInterval;
    nowTime = (time.tv_sec - startTime.tv_sec) * s2msec + (time.tv_usec - startTime.tv_usec)/1000;
    waitTime.tv_sec = (toTime - nowTime) / s2msec;
    m = (toTime - nowTime) - waitTime.tv_sec * s2msec;
    if (m > 0)
    { // too early for next sample, so wait
      waitTime.tv_nsec = (m) * m2nsec;
      nanosleep(&waitTime, NULL);
    }
  }
  rxtask.running = 0;
  fprintf(stderr,"GDM8246: closing bus device\n");
  close(devif.ttyDev);
  fprintf(stderr,"GDM8246: Shutting down thread\n");
  if (logfile != NULL)
    fclose(logfile);
  if (logState != NULL)
    fclose(logState);
  pthread_exit(0);
  return NULL;
}

///////////////////////////////////////////////////////////

/**
 * Create variables for the usb to i2c converter itself */
void createI2Cvariables()
{
  char vn[100];
  snprintf(vn, 100, "%ssetting", gdm.basename);
  gdm.varSetting = createVariable('r', SETT_LEN, vn);
  gdm.varVal = createVariable('r', 2, gdm.basename);
  snprintf(vn, 100, "%sN", gdm.basename);
  gdm.varCnt = createVariable('r', 1, vn);
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
