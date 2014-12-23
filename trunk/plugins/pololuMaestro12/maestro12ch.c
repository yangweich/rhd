 /** \file usbiss.c
 *  \ingroup hwmodule
 *
 * interface to usb-to-i2c module and a MD25 mototcontroller (2xdc motor control with gearing and encoder)
 *
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 1683 $:"
 #define DATE             "$Date: 2011-09-04 16:07:32 +0200 (Sun, 04 Sep 2011) $:"
 #define ID               "$Id: usbiss.c 1683 2011-09-04 14:07:32Z jca $"
 #define PLUGINNAME        "pololu12ch"
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
#include <semaphore.h>

//RHD Core headers
#include <rhd.h>
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "protocol.h"
#include "maestro12ch.h"

#define LS_READ_BYTES 20

#define POLOLU_MODE_SERVO        0
#define POLOLU_MODE_IN_ANALOG    1
#define POLOLU_MODE_IN_DIGITAL   2
#define POLOLU_MODE_OUT_DIGITAL  3

///Struct for shared parse data
typedef struct  {
    int depth; // current XML tag level
    int skip;  // skip from this level up
    char enable;
    char found;
    int myTag;
  } parseInfo;

/* prototypes */
/**
 * Create exchange variables */
void createVariables();
/**
 * initialize plugin device */
int initDevice(void);
/** poll to see if a read would not block.
 * \param fd is the device number
 * \param msTimeout is the maximum wait time
 * \returns 0 is no data is available and 1 if at least 1 byte is available */
int pollDeviceRx(int fd, int msTimeout);
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
int getDataToTimeout(unsigned char * dest, int destCnt, int timeoutms);
/** run thread for rx task */
void * run(void *);
/**
 * Parsing function for start tag */
void XMLCALL lsStartTag(void *, const char *, const char **);
/**
 * Parsing function for end tag */
void XMLCALL lsEndTag(void *, const char *);
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
 * Round to an integer value - also for negative values. */
int roundi(const float v)
{ // round to closest integer
  if (v > 0)
    return (int)(v + 0.5);
  else
    return (int)(v - 0.5);
}
/**
 * Reset update flag for one read variable */
void resetUpdateRead(int id);
/**
 * Set servo initial value from string */
void setControlAxis(servo * axis, const char * val);

/**
 * Send servo position */
void sendOutputValues(void);

/**
 * get input positions */
void getPositionValues(void);
/**
 * get input values
 * \returns error number - 0 is no error
 * error (bit value): 
 * 0 1 = serial error (no stop-bit)
 * 1 2 = serial buffer overflow hw
 * 2 4 = serial buffer overflow sw
 * 3 8 = crc error
 * 4 16 = command error or not finished with last command
 * 5 32 = serial timeout - serial host dead
 * 6 64 = user script error
 * 7 128 = user script stack overflow
 * 8 256 = user script program counter overflow
 * 9-15 not used */
int getErrorState(void);
////////////////////////////////////////////////////
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
////////////////////////////////////////////////
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
  /** should we shut down */
  int shutDown;
  /** Trigger polling of data from units - when set to 1 */
  int startNewRxCycle;
  /** name of serial device */
  /** thread handle and attributes */
  pthread_t i2c_thread;
} rxtask;

int tick = 0;
struct timeval tickTime;
//int camPosUpd = 0;
//int keepAlive = 0;
int debugFlag = 0;
int debugIO = 0;
FILE * logfile = NULL;
FILE * logState = NULL;
sem_t tickSem;

/** catch writes to a non existing file - allowed in debug mode */
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

/////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int rhdTick)
{
  int returnValue = 1;
  int i, sn = 0;
  //
  tick = rhdTick;
  rxtask.startNewRxCycle = 1;
  //
//   if (isUpdated('w', state.camAngleRef))
//   {
//     state.camAngRefNew[0] = getWriteVariable(state.camAngleRef, 0) * M_PI / 1800.0;
//     state.camAngRefNew[1] = getWriteVariable(state.camAngleRef, 1) * M_PI / 1800.0;
//     camPosUpd = 1;
//   }
  if (isUpdated('w', state.servoRef))
  {
    for (i = 0; i < MAX_SERVO_CNT; i++)
    {
      servo * ps;
      ps = &state.servos[i];
      if (ps->mode == POLOLU_MODE_SERVO)
      {
        int pr = getWriteVariable(state.servoRef, sn);
        pr = limitSigned(pr, ps->vmin, ps->vmax);
        if (pr != ps->positionRef)
        {
          ps->positionRef = pr;
          ps->updated++;
        }
        sn++;
      }
    }
  }
//  pthread_mutex_unlock(&state.mLock);
  sem_trywait(&tickSem);
  // then post a flag, that there is something to do for main thread
  sem_post(&tickSem);
  //
  return returnValue;
}

//////////////////////////////////////////////////////

int terminate(void)
{
  rxtask.shutDown = 1;
  printf(PLUGINNAME ": stopping control ... ");
  if (logfile != NULL)
    fclose(logfile);
  if (logState != NULL)
    fclose(logState);
  fflush(stdout);
  pthread_join(rxtask.i2c_thread, NULL);
  close(busif.ttyDev);
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
  int len;
  FILE *fp;
  rxtask.shutDown = 0;
  busif.debugIoVal = 0;
  strncpy(busif.serialDev, "/dev/ttyACM0", MxDL);
  state.cntIn = 0;
  state.cntOut = 0;
  //Print initialization message
  //Find revision number from SVN Revision
  char * p1;
  char versionString[20] = REVISION;
  char tempString[10];
  p1 = strrchr(versionString, '$');
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  printf("maestro12ch: plug-in version %s\n", tempString);
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
      printf( PLUGINNAME ": Error reading: %s\n",filename);
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
  if (result && xmlParse.enable)
  { // all is fine - start plugin
    createVariables();
    result = initDevice();
  }
  // this is variable in another plugin,
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
        // this one handles usbiss only
        if (strcmp("pololu12ch",el) == 0)
        { // is it enabled, the only info needed
          info->myTag = info->depth;
          for(i = 0; attr[i]; i+=2)
          {
            if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0))
              info->enable = 1;
            if (strcmp("dev",attr[i]) == 0)
              strncpy(busif.serialDev,attr[i+1], MxDL);
            if (strcmp("debugIO",attr[i]) == 0)
              busif.debugIoVal = strtol(attr[i+1], NULL, 0);
            else if (strcmp("debug", attr[i]) == 0)
            {
              debugFlag = strtol(attr[i+1], NULL, 0);
              if (debugFlag)
                printf(PLUGINNAME ": started in DEBUG mode!\n");
            }
          }
          if (!info->enable)
            printf(PLUGINNAME ": Use is disabled in configuration\n");
        }
        else
          info->skip = info->depth;
        break;
      case 4:
        if (info->myTag && (strcmp("channel",el) == 0))
        {
          int id = -1;
          const char * par = NULL;
          int mode = -1;
          const char * name = NULL;
          for(i = 0; attr[i]; i+=2)
          {
            if (strcmp("id",attr[i]) == 0)
              id = strtol(attr[i+1], NULL, 0);
            if (strcmp("mode",attr[i]) == 0)
            {
              if (strcasecmp("servo",attr[i+1]) == 0)
                mode = POLOLU_MODE_SERVO;
              else if (strcasecmp("in_analog",attr[i+1]) == 0)
                mode = POLOLU_MODE_IN_ANALOG;
              else if (strcasecmp("in_digital",attr[i+1]) == 0)
                mode = POLOLU_MODE_IN_DIGITAL;
              else if (strcasecmp("out_digital",attr[i+1]) == 0)
                mode = POLOLU_MODE_OUT_DIGITAL;
            }
            // servo limit parameters
            else if (strcmp("param", attr[i]) == 0)
              par = attr[i+1];
            // name
            else if (strcmp("name", attr[i]) == 0)
              name = attr[i+1];
          }
          if (id >= 0 && id < 12 && mode != -1)
          {
            state.servos[id].mode = mode;
            state.servos[id].servo = id;
            if (mode == POLOLU_MODE_SERVO || mode == POLOLU_MODE_OUT_DIGITAL)
            {
              if (par != NULL)
                setControlAxis(&state.servos[id], par);
              else
                printf(PLUGINNAME ":read config: failed servo setting "
                " id=%d mode=%d param=%s\n", id, mode, par);
              state.cntOut++;
            }
            else
              state.cntIn++;
            if (name != NULL)
              strncpy(state.servos[id].name, name, 32);
          }
          else
          {
            printf(PLUGINNAME ":read config: failed channel setting "
            " id=%d mode=%d\n", id, mode);
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
  if (info->myTag == info->depth)
    info->myTag = 0;
  info->depth--;
  if (info->depth < info->skip)
    // back to normal search for valid tags
    info->skip = 0;
//  printf(" to %d\n", info->depth);
}

////////////////////////////////////////////////////


/**
 * transmit package to maestro12ch, maxPWMand check reply.
 * \param data is payload and is command dependent 
 * \param dataCnt is number of additional bytes in data (the command parameters - if any)
 * \param rxDataCnt is expected number of bytes to receive:
 * 0 is no reply expected; 
 * <7 is wait for timeout (or buffer is full);
 * >= 7 wait for exactly this number of bytes.
 * \returns nuber of bytes received if reply is OK, the reply is in rxBuf - if any */
int sendToDevice(uint8_t data[], int dataCnt, int rxDataCnt)
{
  int n = 0, i;
  // write to device
  secure2Write(data, dataCnt);
  // debug
  if (debugIO)
  {
    printf(PLUGINNAME ": send ");
    for (i = 0; i < dataCnt; i++)
      printf(" %2x", data[i]);
    printf("\n");
  }
  // debug end
  if (rxDataCnt > 0)
  { // reply is expected
    if (rxDataCnt < 7)
      n = getDataToTimeout(busif.rxBuf, MxBL, 15);
    else
      // return when rxDataCnt is reached (and swallow up to one more byte)
      n = getDataToTimeout(busif.rxBuf, rxDataCnt + 1, 30);
  }
  // debug
  if (debugIO && (n > 0) && (rxDataCnt != 0))
  {
    printf(PLUGINNAME ":  got %d bytes :", n);
    for (i = 0; i < n; i++)
      printf(" %2x", busif.rxBuf[i]);
  }
  // debug end
  return n;
}


/**
 * Open device, initialize device in right mode, and get first status */
int openAndSetMode()
{
  int result = 0;
  // open device
  busif.ttyDev = open(busif.serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = busif.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,PLUGINNAME ": Can't open device: %s\n", busif.serialDev);
  if (debugFlag)
    // allow no device in debug mode
    result = 1;
  busif.lostConnection = ! result;
  if (result)
    printf(PLUGINNAME ": opened device %s successfully\n", busif.serialDev);
  if (result)
  { // empty data from device
    int n = 1;
    while (n > 0)
      n = getDataToTimeout(busif.rxBuf, MxBL, 100);
  }
  if (result)
  {
    int i;
    int err;
    for (i = 0; i < 5; i++)
    {
      err = getErrorState();
      if (err == 0)
        break;
      printf(PLUGINNAME ": detected error 0x%x - reset and continued\n", err);
    }
    result = (err == 0);
  }
  debugIO = busif.debugIoVal;
  result |= debugFlag;
  if (result)
    printf(PLUGINNAME ": connected successfully\n");
  return result;
}
//////////////////////////////////////////////

/**
 * Initialize the communication and start rx thread
 * \returns 1 on success (else 0) */
int initDevice(void) 
{ //Open first serial port
  int result;
  //
  rxtask.running = 0;
  rxtask.startNewRxCycle = 0;
  // open and configure device
  result = openAndSetMode();
  //
  if (result == 1)
  { // start thread to handle bus
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rxtask.i2c_thread, &attr, run, 0))
    {
      perror("   usbiss: Can't start i2c receive thread");
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

/**
 * utility to limit an integer 
 * \param min is the minimum value
 * \param max is the maximum value
 * \param value is the value to limit
 * \returns value except if it exceeds one of the limits, if so the limit is returned. */
int limit(int min, int max, int value)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}

/// communication thread
void * run(void * not_used)
{ // run in thread
  int seq = 0;
  int loopCnt = 0;
  int loopCntOld = 0;
  int errorLoop = 0;
  struct timeval loopTimeOld;
  struct timeval tm;
  int32_t dt1 = 0, dt2 = 0;
  int ch; // channel
  int servoId; // servo number
  int inIr;  // analog input IR-distance sensor
  int inBat; // analog battrey sensor (volts,amps)
  int error;
  float battAvg[2] = {1e6,1e6};
  //
  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, PLUGINNAME ": signal: can't ignore SIGPIPE.\n");

  //fprintf(stderr, "   USBISS: rx_task running\n");
  if (debugFlag)
  {
    logfile = fopen(PLUGINNAME ".log", "w");
    logState = fopen(PLUGINNAME "-state.log", "w");
  }
  if (logfile != NULL)
  {
    gettimeofday(&tm, NULL);
    fprintf(logfile, "time tick - not a complete list\n");
  }
  if (logState != NULL)
  {
    gettimeofday(&tm, NULL);
    fprintf(logState, "time state logging\n");
  }
  //
  //Mark thread as running
  rxtask.running = 1;
  while (!rxtask.shutDown)
  { // maintain interface
    loopCnt++;
    if (logState != NULL)
    {
      if (seq == 0)
        fprintf(logState,"\n");
      gettimeofday(&tm, NULL);
      fprintf(logState, "%lu.%06lu %d %2d set %dus get %dus\n ", tm.tv_sec, tm.tv_usec, loopCnt, seq, dt1, dt2);
    }
    sem_wait(&tickSem);
    gettimeofday(&tickTime, NULL);
    //printf("tick unlocked new=%d\n", rxtask.startNewRxCycle);
    rxtask.startNewRxCycle = 0;
    // update output
    sendOutputValues();
    gettimeofday(&tm, NULL);
    dt1 = tm.tv_sec - tickTime.tv_sec;
    dt1 = dt1 * 1000000;
    dt1 += tm.tv_usec - tickTime.tv_usec;
    getPositionValues();
    error = getErrorState();
    if (error > 0)
    { // set (now cleared error)
      printf(PLUGINNAME "pololu error %x\n", error);
      errorLoop = loopCnt;
      setVariable(state.poError, 0, error);
    }
    else if (loopCnt - errorLoop == 500)
    { // reset error state after a while (ca. 25 hz data rate)
      setVariable(state.poError, 0, error);
    }
    gettimeofday(&tm, NULL);
    dt2 = tm.tv_sec - tickTime.tv_sec;
    dt2 = dt2 * 1000000;
    dt2 += tm.tv_usec - tickTime.tv_usec;
    setVariable(state.cycleTime, 0, dt1);
    setVariable(state.cycleTime, 1, dt2);
    // set RHD variables
    inIr = 0;
    inBat = 0;
    servoId = 0;
    for (ch = 0; ch < MAX_SERVO_CNT; ch++)
    { // get channel data
      servo * ps;
      ps = &state.servos[ch];
      switch (ps->mode)
      {
        case POLOLU_MODE_SERVO:
          if (servoId < state.cntOut)
          {
            setVariable(state.servoPos, servoId, ps->position);
            setVariable(state.servoPosMin, servoId, ps->vmin);
            setVariable(state.servoPosMax, servoId, ps->vmax);
            servoId++;
          }
          break;
        case POLOLU_MODE_IN_ANALOG:
        if (inIr < state.cntIn)
        { // set all input values
          if (ps->updated)
          {
            ps->updated = 0;
            setVariable(state.inIr, inIr, ps->position);
            if (inBat < 2 && strncasecmp(ps->name, "batt", 4) == 0)
            { // this is battery measurement voltage and current
              int amps, volt;
              // average over 50 samples and multiply result with 10
              if (battAvg[inBat] > 1e5)
                battAvg[inBat] = ps->position * 50.0;
              else
                battAvg[inBat] = battAvg[inBat] * 0.98 + ps->position;
              switch (inBat)
              {
                case 0: // current 6750 == 0A, 6.4mA pr cnt
                  amps = roundi(((float)(battAvg[inBat]/5) - 6750.0) * 6.7);
                  setVariable(state.inBat, inBat, amps);
                  break;
                case 1: // battery voltage 675 = 2.45V
                  volt = roundi((float)(battAvg[inBat]/5) / 6750.0 * 245);
                  setVariable(state.inBat, inBat, volt);
                  break;
                default:
                  break;
              }
              inBat++;
            }
          inIr++;
          }
        }
        break;
        default:
          break;
      }
    }
    if ((loopCnt == loopCntOld + 10) || (loopCnt < loopCntOld))
    {
      struct timeval tm;
      double dt;
      double fr;
      gettimeofday(&tm, NULL);
      dt  = tm.tv_sec - loopTimeOld.tv_sec;
      dt += (tm.tv_usec - loopTimeOld.tv_usec) / 1e6;
      fr  = 10.0/dt;
      setVariable(state.inIrRate, 0, roundi(fr));
      loopCntOld = loopCnt;
      loopTimeOld = tm;
    }
  }
  //
  //
  rxtask.running = 0;
  // set servos to free
  fprintf(stderr,PLUGINNAME ": thread shut down\n");
  pthread_exit(0);
  return NULL;
}

///////////////////////////////////////////////////////////

/**
 * Create variables for the usb to i2c converter itself */
void createVariables() 
{
  // - if using ackerman steering or just servo control (to find limits)
  int numServos = state.cntOut;
  int numIn = state.cntIn;
  state.servoRef = createVariable('w',numServos,"poservoref");
//   state.servoRef = createVariable('w',numServos,"poservospeed");
//   state.servoReset = createVariable('w',numServos,"poservoreset");
  // steeringangle in radians and uRadians
  //state.camAngleRef  = createVariable('w', 2, "pocamangleref");
  state.servoPos = createVariable('r', numServos, "popos");
  state.servoPosMin = createVariable('r', numServos, "poposmin");
  state.servoPosMax = createVariable('r', numServos, "poposmax");
  //state.servoStatus = createVariable('r', numServos, "postatus");
  state.inIr = createVariable('r', numIn, "poirdist");
  state.inIrRate = createVariable('r', 1, "poirrate");
  state.inBat = createVariable('r', 2, "pobattery");
  state.cycleTime = createVariable('r', 2, "pocycletime");
  state.poError = createVariable('r', 1, "poerror");
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
      perror(PLUGINNAME ":deviceRx (poll)");
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

////////////////////////////////////////////////

int getData(int minCnt, int timeoutms, int maxWaitCnt)
{
  int l = 0;
  int n = 0;
  unsigned char * p1 = busif.rxBuf;
  // wait for reply - up to 500ms
  while (l < maxWaitCnt && !rxtask.shutDown)
  {
    int dataOK;
    int m;
    l++; // loop counter
    // wait for data
    dataOK = pollDeviceRx(busif.ttyDev, timeoutms);
    if (dataOK)
    { // there is data
      m = read(busif.ttyDev, p1, MxBL - n - 1);
      if (m > 0)
      {
        n += m;
        p1 = &busif.rxBuf[n];
      }
      else if (m < 0)
      { // poll error - device is removed?
        printf("usbiss md25: Read error from device - device is lost\n");
        busif.lostConnection = 0;
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

/////////////////////////////////////////////////

int getDataToTimeout(unsigned char * dest, int destCnt, int timeoutms)
{
  int n = 0;
  unsigned char * p1 = dest;
  int toMult = 2; // timeout multiplier on first timeout
  int timout = timeoutms / 3 + 1;
  //
  *p1 = '\0'; // clear result (to improve debug);
  while ((toMult > 0) && !rxtask.shutDown)
  { // wait for data
    int dataOK;
    int m;
    //
    if (destCnt - n <= 1)
      // no more space
      break;
    // wait for data
    dataOK = pollDeviceRx(busif.ttyDev, timout * toMult);
    if (dataOK)
    { // there is data, so get it
      m = read(busif.ttyDev, p1, destCnt - n - 1);
      if (m > 0)
      { // got some data - search for new-line
        n += m;
        p1 = &busif.rxBuf[n];
      }
      else if (m < 0)
      { // poll error - device is removed?
        printf("usbiss md25: Read error from device - connection lost\n");
        busif.lostConnection = 1;
      }
    }
    else
    { // we have a timeout - no more data
      // zero terminate
      *p1 = '\0';
      break;
    }
    toMult--;
  }
  return n;
}

///////////////////////////////////////////////

void setControlAxis(servo * axis, const char * val)
{
  const char * p1;
  int v, i;
  //
  p1 = val;
  // set default values
  axis->vmin =   100;
  axis->vmax = 11900;
  axis->center = 6000;
  axis->updated = 0;
  // radians per tick - from servo spec
  axis->position = 6000;
  axis->positionRef = 6000;
  for (i = 1; i < 4; i++)
  {
    if (p1 == NULL)
      break;
    v = strtol(p1, (char **) &p1, 0);
    // control axis attribute should be "servo-index min max invert"
    switch (i)
    {
      case 1: 
        axis->center = v; 
        break; // center (initial) position
      case 2: axis->vmin = v; break; // lower limit
      case 3: axis->vmax = v; break; // upper limit
      default: break;
    }
  }
  axis->positionRef = axis->center; // desired start position
  axis->updated = 1;
  printf(PLUGINNAME " servo %d from '%s' set to\n"
         "min=%d, ref=%d, max=%d [t] [r/t]\n",
          axis->servo, val, axis->vmin,
	 axis->positionRef, axis->vmax);
}



/**
 * Send servo position */
void sendOutputValues(void)
{
  int i;
  for (i = 0; i < MAX_SERVO_CNT; i++)
  {
    servo * ps;
    uint8_t tx[5];
    ps = &state.servos[i];
    if (ps->updated && 
        (ps->mode == POLOLU_MODE_SERVO || ps->mode == POLOLU_MODE_OUT_DIGITAL))
    {
      tx[0] = 0x9f;
      tx[1] = 1;
      tx[2] = i;
      tx[3] = ps->positionRef & 0x7f;
      tx[4] = (ps->positionRef >> 7) & 0x7f;
      sendToDevice(tx, 5, 0);
      ps->position = ps->positionRef;
      ps->updated = 0;
    }
  }
}

/**
 * get input values */
void getPositionValues(void)
{
  int i, n;
  for (i = 0; i < MAX_SERVO_CNT; i++)
  {
    servo * ps;
    uint8_t tx[5];
    ps = &state.servos[i];
    if (ps->mode == POLOLU_MODE_IN_ANALOG || ps->mode == POLOLU_MODE_IN_DIGITAL)
    {
      tx[0] = 0x90;
      tx[1] = i;
      n = sendToDevice(tx, 2, 2);
      if (n == 2)
      { // convert to integer
        int v = (busif.rxBuf[1] << 8) + busif.rxBuf[0];
        if (v != ps->position)
        {
          ps->position = v;
          ps->updated++;
        }
      }
    }
  }
}
///////////////////////////////////

/**
 * get input values */
int getErrorState(void)
{
  int v, n;
  uint8_t tx[5];
  // only one byte needed
  tx[0] = 0xa1;
  n = sendToDevice(tx, 1, 2);
  if (n == 2)
  { // convert to integer
    v = (busif.rxBuf[1] << 8) + busif.rxBuf[0];
//    setVariable(state.poError, 0, v);
  }
  return v;
}
