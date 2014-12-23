 /** \file usbiss.c
 *  \ingroup hwmodule
 *
 * interface to usb-to-i2c module and a MD25 mototcontroller (2xdc motor control with gearing and encoder)
 *
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2011-09-04 16:07:32 +0200 (Sun, 04 Sep 2011) $:"
 #define ID               "$Id: usbiss.c 59 2012-10-21 06:25:02Z jcan $"
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
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "usbiss.h"

#define LS_READ_BYTES 20

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
int initUsbIss(void);
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
void * i2c_task(void *);
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
/** absolute value of long integers */
int64_t i64abs(int64_t val)
{
  if (val < 0)
    return -val;
  else
    return val;
}
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
  pthread_t i2c_thread;
} rxtask;

int tick = 0;
struct timeval tickTime;
int keepAlive = 0;
int debugFlag = 0;
FILE * logfile = NULL;
FILE * logState = NULL;

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
  if (tick == 0)
  { // connect to joystick - if any
    md25.joyaxes = getDatabaseVariable('r', "joyaxes");
    md25.joybuttons = getDatabaseVariable('r', "joybuttons");
    md25.joySteer = getDatabaseVariable('r', "joySteer");
    //
    printf("md25 joystick database index axes=%d, buttons=%d (-1=none)\n", md25.joyaxes, md25.joybuttons);
    //
  }
  tick = rhdTick;
  if (md25.joybuttons >= 0 && md25.joySteer >= 0)
  { // use remote control to control speed
    // axis 3 is speed min is -32000 max is +32000
    int joySteer = getReadVariable(md25.joySteer, 0);
    if (joySteer)
    { // use remote control speed
      int jval = getReadVariable(md25.joyaxes, md25.joySpeedAxis);
      int maxSpeed = md25.joyMaxSpeed; // no speed override
      float vel;
      if (getReadVariable(md25.joybuttons, md25.joyFastBut) > 0)
        maxSpeed = 127; // speed override
      vel = (float)-jval / 32767.0 * maxSpeed;
  /*    if (jval != 0)
        printf("md25 vel %d => %d\n", jval, roundi(vel));*/
      writeValue(md25.speedref, 0, roundi(vel));
    }
  }
  rxtask.startNewRxCycle = 1;
  pthread_mutex_unlock(&md25.mLock);

  return returnValue;
}

//////////////////////////////////////////////////////

int terminate(void)
{
  rxtask.running = 0;
  printf("stopping USBISS ... ");
  fflush(stdout);
  pthread_join(rxtask.i2c_thread, NULL);
  close(busif.ttyDev);
  printf("[OK]\n");
  return 0;
}

/************************** XML Initialization **************************/


/** \brief Initialize the USB to i2c bus and its units
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
  md25.wheelBase = 0.203;
  md25.steerBase = 0.255;
  //Print initialization message
  //Find revision number from SVN Revision
  char * p1;
  char versionString[20] = REVISION;
  char tempString[10];
  p1 = strrchr(versionString, '$');
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  printf("usbiss md25: plug-in version %s\n", tempString);
  /* Initialize Expat parser*/
  XML_Parser parser = XML_ParserCreate(NULL);
  result = parser != 0;
  if (!result)
    fprintf(stderr, "USBISS: Couldn't allocate memory for XML parser\n");
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
      printf("USBISS: Error reading: %s\n",filename);
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
        fprintf(stderr, "   USBISS: Couldn't allocate memory for XML File buffer\n");
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
      fprintf(stderr, "USBISS: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
  }
  if (parser != NULL)
    XML_ParserFree(parser);
  if (xmlBuf != NULL)
    free(xmlBuf);
  if (result && xmlParse.enable)
  { // all is fine - start plugin
    result = initUsbIss();
  }
  // this is variable in another plugin,
  // and can not be set just now.
  md25.joybuttons = -1;
  md25.joyaxes = -1;
  md25.joySteer = -1;
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
        else if (strcmp("robot",el) == 0)
        { // we need something from robot configuration
          for(i = 0; attr[i]; i+=2)
          {
            if (strcasecmp("wheelbase",attr[i]) == 0)
              md25.wheelBase = strtod(attr[i+1], NULL);
            else if (strcasecmp("steerbase",attr[i]) == 0)
              md25.steerBase = strtod(attr[i+1], NULL);
            else if (strcasecmp("frontbase",attr[i]) == 0)
              md25.frontBase = strtod(attr[i+1], NULL);
          }
        }
        else
          info->skip = info->depth;
        break;
      case 3:
        // this one handles usbiss only
        if (strcmp("usbiss",el) == 0)
        { // is it enabled, the only info needed
          for(i = 0; attr[i]; i+=2)
            if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0))
              info->enable = 1;
          if (!info->enable)
            printf("   USBISS: Use is disabled in configuration\n");
        }
        else
          info->skip = info->depth;
        break;
      case 4:
        // devices on i2c bus
        if (strcmp("bus",el) == 0)
        {
          const char * att, * val;
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i];
            val = attr[i + 1];
            if (strcmp("dev",att) == 0)
              strncpy(busif.serialDev, val, MxDL);
            else if (strcmp("id", att) == 0)
              usbiss.busid = strtol(val, NULL, 0);
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
              if (debugFlag)
                printf("   USBISS started in DEBUG mode!\n");
            }
          }
          info->found = 1;
          printf("   USBISS: serial device to %s, busID 0x%x\n", busif.serialDev, usbiss.busid);
        }
        else if (strcmp("md25drive",el) == 0)
        {
          const char * att, * val;
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i];
            val = attr[i + 1];
            if (strcmp("id", att) == 0)
              md25.busid = strtol(val, NULL, 0);
            else if (strcmp("joySpeedAxis", att) == 0)
              md25.joySpeedAxis = strtol(val, NULL, 0);
            else if (strcmp("joyFastBut", att) == 0)
              md25.joyFastBut = strtol(val, NULL, 0);
            else if (strcmp("joyMaxSpeed", att) == 0)
              md25.joyMaxSpeed = strtol(val, NULL, 0);
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
  int i, n = 1, v;
  // open device
  busif.ttyDev = open(busif.serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = busif.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,"   usbiss: Can't open device: %s\n", busif.serialDev);
  else
  { // set baudrate
    result = (set_serial(busif.ttyDev, 9600) != -1);
    if (result == 0)
      fprintf(stderr,"   usbiss: Can't set serial port parameters\n");
  }
  if (debugFlag)
    // allow no device in debug mode
    result = 1;
  busif.lostConnection = ! result;
  if (result)
  { // empty data from device
    while (n > 0)
      n = getDataToTimeout(busif.rxBuf, MxBL, 100);
  }
  if (result)
  { // format 0x5A is internal USBISS command
    //        0x02 is subcommand
    //        0x60 is i2c at 100 kHz using i2c hardware
    //        0x04 sets 
    unsigned char setI2cMode[4] = { 0x5a, 0x02, 0x60, 0x04};
    printf("usbiss md25: setting i2c mode ...");
    for (i = 0; i < 5; i++)
    { // signed mode
      secure2Write(setI2cMode, sizeof(setI2cMode));
      n = getDataToTimeout(busif.rxBuf, MxBL, 150);
      result = (n > 0 && busif.rxBuf[0] != 0);
      if (result)
        break;
      printf("[fail (n=%d v=%d)]", n, busif.rxBuf[0]);
    }
    if (result)
      printf("[OK]\n");
    else
      printf("\n");
  }
  if (result)
  { // mode 1 sets to signed speed of both motors
    unsigned char setMd25Mode[5] = { 0x55, 0xb0, 0x0f, 0x01, 0x01};
    // set modes
    printf("usbiss md25: setting motor mode - signed ...");
    setMd25Mode[4] = 0x01;
    for (i = 0; i < 5; i++)
    { // signed mode
      secure2Write(setMd25Mode, sizeof(setMd25Mode));
      n = getDataToTimeout(busif.rxBuf, MxBL, 150);
      result = (n > 0 && busif.rxBuf[0] != 0);
      if (result)
        break;
      printf("[fail]");
    }
    if (result)
      printf("[OK (%d bytes)]\n", n);
    else
      printf("\n");
  }
  if (result)
  {
    unsigned char getI2cVersion[2]={ 0x5a, 0x01};
    // set modes
    printf("usbiss md25: getting version number ...");
    for (i = 0; i < 5; i++)
    { // signed mode
      secure2Write(getI2cVersion, sizeof(getI2cVersion));
      n = getDataToTimeout(busif.rxBuf, MxBL, 150);
      result = (n >= 3);
      if (result)
      {
        printf("USBISS: model %x, version %x, mode %x ", busif.rxBuf[0], busif.rxBuf[1], busif.rxBuf[2]);
        busif.version[0] = busif.rxBuf[0];
        busif.version[1] = busif.rxBuf[1];
        busif.version[2] = busif.rxBuf[2];
    /*          setVariable(usbiss.varI2sVersion, 0, busif.version[0]);
            setVariable(usbiss.varI2sVersion, 1, busif.version[1]);
            setVariable(usbiss.varI2sVersion, 2, busif.version[2]);*/
        break;
      }
      printf("[fail]");
    }
    if (result)
      printf("[OK (%d bytes)]\n", n);
    else
      printf("\n");
  }
  if (result)
  { // serial number of device
    unsigned char getI2cSerial[2] ={ 0x5a, 0x03};
    // request version info
    printf("usbiss md25: getting serial number ...");
    for (i = 0; i < 5; i++)
    { // signed mode
      secure2Write(getI2cSerial, sizeof(getI2cSerial));
      n = getDataToTimeout(busif.rxBuf, MxBL, 150);
      result = n >= 8;
      if (result)
      { // convert from ascii to integer
        v = 0;
        for (i = 0; i < 8; i++)
          v = v * 10 + (busif.rxBuf[i] - '0');
        busif.serial = v;
        break;
      }
      printf("[fail]");
    }
    if (result)
      printf("%d [OK (%d bytes)]\n", busif.serial, n);
    else
      printf("\n");
  }
  md25.accOld = -1;
  return result || debugFlag;
}
//////////////////////////////////////////////

/**
 * Initialize the communication and start rx thread
 * \returns 1 on success (else 0) */
int initUsbIss(void)
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
    if (pthread_create(&rxtask.i2c_thread, &attr, i2c_task, 0))
    {
      perror("   usbiss: Can't start i2c receive thread");
      result = 0;
    }
  }
  if (result == 1)
  { /****** Create database variables if all is ok **************/
    int waitCount = 0;
    createI2Cvariables();
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

////////////////////////////////////////////////////

///RS232  Recieve thread
void * i2c_task(void * not_used)
{ // run in thread
  // MD25 read requests
  // format 0x55 send i2c message using register byte
  //        0xb1 device adress (read mode)
  //        0x0a port number to read
  //        0x01 number of bytes ro read (1 is one register, 2 is this and the next register)
  unsigned char getMd25BatVolt[4] ={ 0x55, 0xb1, 0x0a, 0x01};
  unsigned char getMd25Enc[4]     ={ 0x55, 0xb1, 0x02, 0x08};
  unsigned char getMd25Current[4] ={ 0x55, 0xb1, 0x0b, 0x02};
  unsigned char getMd25MotSpeed[4]={ 0x55, 0xb1, 0x00, 0x02};
//  unsigned char getMd25Acc[4]     ={ 0x55, 0xb1, 0x0d, 0x01};
  // MD25 write requests
  // format 0x55 send i2c message using register byte
  //        0xb0 device adress (write mode)
  //        0x0a port number to write to
  //        0x02 number of bytes ro write (1 is one register, 2 is this and the next register)
  // speed is signed char -128 (0x80) is full reverse, 127 (0x7f) is full fwd and 0 is stop
  unsigned char setMd25MotSpeed[6]={ 0x55, 0xb0, 0x00, 0x02, 0x00, 0x00};
  // mode 1 sets to signed speed of both motors
//  unsigned char setMd25Mode[5]    ={ 0x55, 0xb0, 0x0f, 0x01, 0x01};
  // acceleration
  unsigned char setMd25Acc[5]     ={ 0x55, 0xb0, 0x0e, 0x01, 0x0a};


  const int MaxWaitCycles = 2; // max number of timeout periods
  const int PollTimeoutMs = 4; // timeout period
  int state = 0;
  int loopCnt = 0;
  struct timeval tm;
  unsigned int lastEnc1, lastEnc2;
  uint32_t e1, e2;
  int64_t eDiff1, eDiff2;
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
    fprintf(stderr, "   USBISS: signal: can't ignore SIGPIPE.\n");

  //fprintf(stderr, "   USBISS: rx_task running\n");
  if (debugFlag)
  {
    logfile = fopen("usbiss.log", "w");
    //logState = fopen("usbiss-state.log", "w");
  }
  if (logfile != NULL)
  {
    gettimeofday(&tm, NULL);
    fprintf(logfile, "time tick looptime-ms speedref motor1 motor2 battery enc1 enc2 curr1 curr2 speed1 speed2 acc\n");
  }
  if (logState != NULL)
  {
    gettimeofday(&tm, NULL);
    fprintf(logState, "time state logging\n");
  }
  //Mark thread as running
  rxtask.running = 1;
  busif.txBuf[0] = '\0';
  while (rxtask.running)
  { // maintain interface
    int i, v, n, isOK;
    int upd; /*, deb;*/
    //
    loopCnt++;
    if (logState != NULL)
    {
      if (state == 0)
        fprintf(logState,"\n");
      gettimeofday(&tm, NULL);
      fprintf(logState, "%lu.%06lu %d %2d\n ", tm.tv_sec, tm.tv_usec, loopCnt, state);
    }
    if (busif.lostConnection)
    { // connection is lost - durinr read or write
      if (busif.ttyDev >= 0)
      {
        close(busif.ttyDev);
        busif.ttyDev = -1;
        printf("**** usbiss: lost connection - trying to reconnect\n");
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
    switch (state)
    { // sync
      case 1:
        // wait for next rhd time tick
        v = getTimePassed(tickTime);
        pthread_mutex_lock(&md25.mLock);
        gettimeofday(&tickTime, NULL);
        //printf("tick unlocked new=%d\n", rxtask.startNewRxCycle);
        rxtask.startNewRxCycle = 0;
        // check for updated values
        if (md25.steeringChangedTo < 0)
          md25.steeringChangedTo = getDatabaseVariable('r', "steeringChangedTo");
        upd  = isUpdated('w', md25.varAccW);
        upd |= isUpdated('w', md25.varM1SpeedW);
        upd |= isUpdated('w', md25.varM2SpeedW);
        if (md25.steeringChangedTo >= 0)
          upd |= isUpdated('r', md25.steeringChangedTo);
        upd |= isUpdated('w', md25.speedref);
        if (logfile != NULL)
        {
          gettimeofday(&tm, NULL);
          fprintf(logfile, "%lu.%06lu %d %.1f %3d %3d %3d %d %d %d %3d %3d %3d %3d %d\n", tm.tv_sec, tm.tv_usec, tick, v/1000.0,
                  getWriteVariable(md25.speedref, 0),
                  getWriteVariable(md25.varM1SpeedW, 0),
                  getWriteVariable(md25.varM2SpeedW, 0),
                  getReadVariable(md25.varBatt,0),
                  getReadVariable(md25.varEnc1,0),
                  getReadVariable(md25.varEnc2,0),
                  getReadVariable(md25.varM1Curr,0),
                  getReadVariable(md25.varM2Curr,0),
                  getReadVariable(md25.varM1SpeedR,0),
                  getReadVariable(md25.varM2SpeedR,0),
                  getReadVariable(md25.varAccR,0)
                  );
        }
        //printf("tick %d\n", tick);
        break;
      // send new settings and request for data    
      case 2: // speed of motor 1 and 2
        if ( ! upd)
          // no need to update motor speed
          break;
        // if ackermann steering, then steering angle is needed
        if (md25.steeringChangedTo >= 0 &&
            (isUpdated('r', md25.steeringChangedTo) || isUpdated('w', md25.speedref)))
        { // convert steering angle to wheel speed, as steering angle or speed is changed
          int speed = getWriteVariable(md25.speedref, 0);
          // get steering angle (for a cycle model steering)
          double steerAngle = getVariableDouble('r', md25.steeringChangedTo);
          // reset the update flag
          resetUpdateRead(md25.steeringChangedTo);
          if (speed != 0)
          { // calculate wheel speed (left and right) at this steering angle and speedref
            if (steerAngle > M_PI / 2.0 - 0.001)
              // close to 90 deg steering angle
              steerAngle = M_PI / 2.0 - 0.001;
            else if (steerAngle < -M_PI / 2.0 + 0.001)
              // close to 90 deg steering angle
              steerAngle = -M_PI / 2.0 + 0.001;
            { // normal steering angle (valid tangent value)
              float ct = fabs(tan(steerAngle) / md25.steerBase);
              float ofs = ct * md25.wheelBase;
              float speedo = speed * (2.0 + ofs) / (2.0 / cos(steerAngle) + md25.frontBase * ct);
              float spl, spr;
              if (steerAngle >= 0)  // turning left
              {
                spr = speedo;
                spl = speedo*(2.0 - ofs)/(2.0 + ofs);
              }
              else // turning right - reduce speed of right wheel
              {
                spl = speedo;
                spr = speedo*(2.0 - ofs)/(2.0 + ofs);
              }
              // write values back to symbol stack
              writeValue(md25.varM1SpeedW, 0, roundi(spl));
              writeValue(md25.varM2SpeedW, 0, roundi(spr));
            }
          }
          else
          { // stop
            writeValue(md25.varM1SpeedW, 0, 0);
            writeValue(md25.varM2SpeedW, 0, 0);
          }
        }
        if (!isMasterAlive && md25.joybuttons < 0)
        { // no write allowed client - set speed to 0
          if (getWriteVariable(md25.varM1SpeedW, 0) != 0 || getWriteVariable(md25.varM2SpeedW, 0) != 0)
          {
            writeValue(md25.varM1SpeedW, 0, 0);
            writeValue(md25.varM2SpeedW, 0, 0);
            printf("Emergency stop! - no rhd clients\n");
          }
        }
        // get speed values
        setMd25MotSpeed[4] = limitSignedChar(getWriteVariable(md25.varM1SpeedW, 0), -128, 127);
        setMd25MotSpeed[5] = limitSignedChar(getWriteVariable(md25.varM2SpeedW, 0), -128, 127);
        //printf("md25: speed %d %d\n", limitSignedChar(getWriteVariable(md25.varM1SpeedW, 0), -128, 127),
        //       limitSignedChar(getWriteVariable(md25.varM2SpeedW, 0), -128, 127));
        if (0) //(deb)
          printf("send motor speed as %x %x %x %x %x %x\n",
                  setMd25MotSpeed[0], setMd25MotSpeed[1], setMd25MotSpeed[2],
                  setMd25MotSpeed[3], setMd25MotSpeed[4], setMd25MotSpeed[5]);
        // send data to motor controller
        secure2Write(setMd25MotSpeed, sizeof(setMd25MotSpeed));
        n = getData(1, PollTimeoutMs, MaxWaitCycles);
        isOK = n >= 1 && busif.rxBuf[0] != 0;
        v = getTimePassed(tickTime);
        if ( ! isOK)
          printf("usbiss md25: failed to set motor speed (n=%d v=%d)\n", n, busif.rxBuf[0]);
        //printf("Time after set motor speed %.1fms\n", v / 1000.0);
        break;
      case 3: // acceleration
        v = getWriteVariable(md25.varAccW, 0);
        if (v != md25.accOld)
        { // acc value changed
          if (v == 0)
            setMd25Acc[4] = 10;
          else
            setMd25Acc[4] = limitUnsignedChar(v, 1, 10);
          secure2Write(setMd25Acc, sizeof(setMd25Acc));
          n = getData(1, PollTimeoutMs, MaxWaitCycles);
          isOK = n >= 1 && busif.rxBuf[0] != 0;
          if (isOK)
            md25.accOld = v;
          else
            printf("usbiss md25: failed to set acc (n=%d v=%d)\n", n, busif.rxBuf[0]);
          if (logState != NULL)
          {
            fprintf(logState, "\"set acc to\" %4d (1..10 slow..fast) ", setMd25Acc[4]);
          }
          v = getTimePassed(tickTime);
          //printf("Time after set acc %.1fms\n", v / 1000.0);
        }
        break;
      case 4:
        if (tick % 3 == 0)
        {
          secure2Write(getMd25BatVolt, sizeof(getMd25BatVolt));
          n = getData(1, PollTimeoutMs, MaxWaitCycles);
          isOK = n >= 1;
          if (isOK)
          {
            v = busif.rxBuf[0];
            //printf("usbiss md25: got battery voltage in %d byte, value %d\n", n, v);
            setVariable(md25.varBatt, 0, v);
            if (logState != NULL)
            {
              fprintf(logState, "battery voltage %d\n", v);
            }
          }
          else
            printf("usbiss md25: failed to get battery voltage\n");
          v = getTimePassed(tickTime);
          //printf("Time after get battery voltage %.1fms\n", v / 1000.0);
        }
        break;
      case 5:
        secure2Write(getMd25Enc, sizeof(getMd25Enc));
        n = getData(8, PollTimeoutMs, MaxWaitCycles);
        isOK = n >= 8;
        if (isOK)
        {
          e1 = busif.rxBuf[0];
          for (i = 1; i < 4; i++)
            e1 = (e1 << 8) + busif.rxBuf[i];
          e2 = busif.rxBuf[4];
          for (i = 5; i < 8; i++)
            e2 = (e2 << 8) + busif.rxBuf[i];
          eDiff1 = i64abs((int64_t)e1 - (int64_t)lastEnc1);
          eDiff2 = i64abs((int64_t)e2 - (int64_t)lastEnc2);
          if (tick < 100 || eDiff1 < 100 || eDiff2 < 100 || (UINT32_MAX - eDiff1) < 100 || (UINT32_MAX - eDiff2) < 100)
          {
            setVariable(md25.varEnc1, 0, e1);
            setVariable(md25.varEnc2, 0, e2);
          }
          else
          { // encoder read error - too big a jump
            // state--;
            if (logState != NULL)
              fprintf(logState, "encoder read error ");
            printf("usbiss md25: tick=%d: encoder read error\n", tick);
          }
          lastEnc1 = e1;
          lastEnc2 = e2;
          if (logState != NULL)
          {
            fprintf(logState, "%dchars %x %x %x %x  %x %x %x %x enc %d %d diff %ld %ld\n", n,
                    busif.rxBuf[0], busif.rxBuf[1], busif.rxBuf[2], busif.rxBuf[3],
                    busif.rxBuf[4], busif.rxBuf[5], busif.rxBuf[6], busif.rxBuf[7],
                    e1, e2, eDiff1, eDiff2);
          }
        }
        v = getTimePassed(tickTime);
        //printf("Time after get encoder value %.1fms\n", v / 1000.0);
        break;
      case 6:
        secure2Write(getMd25Current, sizeof(getMd25Current));
        n = getData(2, PollTimeoutMs, MaxWaitCycles);
        isOK = n >= 2;
        if (isOK)
        {
          setVariable(md25.varM1Curr, 0, (char)busif.rxBuf[0]);
          setVariable(md25.varM2Curr, 0, (char)busif.rxBuf[1]);
          if (logState != NULL)
          {
            fprintf(logState, "motor current: %dchars %d %d\n", n, busif.rxBuf[0], busif.rxBuf[1]);
          }
        }
        v = getTimePassed(tickTime);
        //printf("Time after get motor current %.1fms\n", v / 1000.0);
        break;
      case 7:
        secure2Write(getMd25MotSpeed, sizeof(getMd25MotSpeed));
        n = getData(2, PollTimeoutMs, MaxWaitCycles);
        isOK = n >= 2;
        if (isOK)
        {
          setVariable(md25.varM1SpeedR, 0, (char)busif.rxBuf[0]);
          setVariable(md25.varM2SpeedR, 0, (char)busif.rxBuf[1]);
          if (logState != NULL)
          {
            fprintf(logState, "motor speed: %dchars %d %d\n", n, busif.rxBuf[0], busif.rxBuf[1]);
          }
        }
        v = getTimePassed(tickTime);
        //printf("Time after get motor speed %.1fms\n", v / 1000.0);
        break;
      default:
        state = 0;
        break;
    }
    state++;
  }
  rxtask.running = 0;
  fprintf(stderr,"usbiss md25: closing bus device\n");
  close(busif.ttyDev);
  fprintf(stderr,"usbiss md25: Shutting down thread\n");
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
  usbiss.varI2sVersion = createVariable('r',3,"usbissVersion");
  usbiss.varI2sSerial = createVariable('r',1,"usbissSerial");
  // md25
  md25.varBatt = createVariable('r',1,"batteryvolt");
  md25.varEnc1 = createVariable('r',1,"encl");
  md25.varEnc2 = createVariable('r',1,"encr");
  md25.varM1Curr = createVariable('r',1,"currentL");
  md25.varM2Curr = createVariable('r',1,"currentR");
  md25.varM1SpeedW = createVariable('w',1,"speedl");
  md25.varM2SpeedW = createVariable('w',1,"speedr");
  md25.varM1SpeedR = createVariable('r',1,"speedLR");
  md25.varM2SpeedR = createVariable('r',1,"speedRR");
  md25.varReset = createVariable('w',1,"reset");
  md25.varAccR = createVariable('r',1,"accR");
  md25.varAccW = createVariable('w',1,"accW");
  // - if using ackerman steering
  md25.speedref = createVariable('w',1,"speedref");
  md25.steeringChangedTo = getDatabaseVariable('r', "steeringChangedTo");
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

////////////////////////////////////////////////

int getData(int minCnt, int timeoutms, int maxWaitCnt)
{
  int l = 0;
  int n = 0;
  unsigned char * p1 = busif.rxBuf;
  // wait for reply - up to 500ms
  while (l < maxWaitCnt)
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
  //
  *p1 = '\0'; // clear result (to improve debug);
  while (1)
  { // wait for data
    int dataOK;
    int m;
    //
    if (destCnt - n <= 1)
      // no more space
      break;
    // wait for data
    dataOK = pollDeviceRx(busif.ttyDev, timeoutms * toMult);
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
    toMult = 1;
  }
  return n;
}
