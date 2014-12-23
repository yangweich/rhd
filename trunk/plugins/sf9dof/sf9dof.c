 /** \file sf9dof.c
 *  \ingroup hwmodule
 *
 *   Interface for sparkfun 9dof with default software (version 18) *
 *
 *******************************************************************/
/***************************** Plugin version  *****************************/
#define SF9DOF_VERSION    "0.1526"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2011-07-23 18:16:56 +0200 (Sat, 23 Jul 2011) $:"
 #define ID               "$Id: sf9dof.c 59 2012-10-21 06:25:02Z jcan $"
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

#include "sf9dof.h"

#define LS_READ_BYTES 20

///Struct for shared parse data
typedef struct  {
    int depth; // current XML tag level
    int skip;  // skip from this level up
    char enable;
    char found;
  } parseInfo;

/* prototypes */
void createSf9dofvariables();
int initSf9dof(void);
/** poll to see if a read would not block.
 * \param fd is the device number
 * \param msTimeout is the maximum wait time
 * \returns 0 is no data is available and 1 if at least 1 byte is available */
int pollDeviceRx(int fd, int msTimeout);
/** run thread for rx task */
void * sf9dof_task(void *);
/** get variable value from 2 32 bit integer elements 
 * \param type is either 'r' or 'w'.
 * \param index is index to variable.
 * \returns value that is first element signed + decimal part (signed) times 1e-6 */
double getVariableDouble(char type, int index);
/**
 * get index to a named variable - from another plugin
 * \param type is either 'r' or 'w'
 * \param name is the name of the variable
 * \returns -1 if the variable is not found, else the index (positive or zero) of the variable. */
int getDatabaseVariable(char type, const char * name);
/** set a double value in the read or write database.
 * \param type is either 'r' or 'w'.
 * \param index is index to variable.
 * \param value is the value to set - is set as integer value in first element
 *  and remaining part times 1e6 in next element. NB! must fit in this format.
 * \returns true if set (index is valid) */
int setVariableDouble(char type, int index, double value);
//Parsing functions
void XMLCALL lsStartTag(void *, const char *, const char **);
void XMLCALL lsEndTag(void *, const char *);
int roundi(const float v)
{ // round to closest integer
  if (v > 0)
    return (int)(v + 0.5);
  else
    return (int)(v - 0.5);
}
/** catch writes to a non existing file - allowed in debug mode */
ssize_t secure2Write(int fd, const void *buf, ssize_t txLen)
{
  if (fd >= 0)
    return secureWrite(fd, buf, txLen);
  else
    return 1;
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
  pthread_t sf9dof_thread;
} rxtask;

int keepAlive = 0;
/// old steering angle value - in radians
double sacOld = -4.0;
int timeoutCnt = 0;
//int debugFlag = 0;
/** flags for updating of write variables */
int settingUpd = 1, steerUpd = 1;
int waitTicks = 0;
FILE * logImu;
FILE * logImuTick;

/////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int tick)
{
  int returnValue = rxtask.running;
  if (logImuTick != NULL)
  {
    struct timeval tt;
    gettimeofday(&tt, NULL);
    fprintf(logImuTick, "%lu.%06lu %d  %d %d %d  %d %d %d  %d %d %d\n",
            tt.tv_sec, tt.tv_usec, tick,
            getReadVariable(sf9dof.varAcc, 0),
            getReadVariable(sf9dof.varAcc, 1),
            getReadVariable(sf9dof.varAcc, 2),
            getReadVariable(sf9dof.varGyro, 0),
            getReadVariable(sf9dof.varGyro, 1),
            getReadVariable(sf9dof.varGyro, 2),
            getReadVariable(sf9dof.varMag, 0),
            getReadVariable(sf9dof.varMag, 1),
            getReadVariable(sf9dof.varMag, 2));
  }
  return returnValue;
}

//////////////////////////////////////////////////////

int terminate(void)
{
  rxtask.running = 0;
  printf("stopping SF9DOF ... ");
  fflush(stdout);
  if (logImuTick != NULL)
    fclose(logImuTick);
  pthread_join(rxtask.sf9dof_thread, NULL);
  printf("[OK]\n");
  return 0;
}

/************************** XML Initialization **************************/


/** \brief Initialize the SF9DOF board from XML file
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
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  printf("SF9DOF: Initializing plug-in version %s.%s\n",SF9DOF_VERSION, tempString);
  /* Initialize Expat parser*/
  XML_Parser parser = XML_ParserCreate(NULL);
  result = parser != 0;
  if (!result)
    fprintf(stderr, "SF9DOF: Couldn't allocate memory for XML parser\n");
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
      printf("SF9DOF: Error reading: %s\n",filename);
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
        fprintf(stderr, "   SF9DOF: Couldn't allocate memory for XML File buffer\n");
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
      fprintf(stderr, "SF9DOF: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
  }
  if (parser != NULL)
    XML_ParserFree(parser);
  if (xmlBuf != NULL)
    free(xmlBuf);
  if (result && xmlParse.enable)
  { // all is fine - start plugin
    result = initSf9dof();
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
        // this one handles sf9dof only
        if (strcmp("sf9dof",el) == 0)
        { // get enable bit and device name
          const char * att, * val;
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i];
            val = attr[i + 1];
            if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
              info->enable = 1;
            else if (strcmp("dev", att) == 0)
            {
              char * s = serif.serialDev;
              strncpy(serif.serialDev, val, MxDL);
              printf("%s\n", s);
            }
            else if (strcmp("baudrate", att) == 0)
            {
              serif.baudrate = strtol(val, NULL, 0);
            }
            else if (strcmp("debug", att) == 0)
            {
              serif.debugFlag = strtol(val, NULL, 0);
              if (serif.debugFlag)
                printf("   SF9DOF started in DEBUG mode!\n");
            }
          }
          if (!info->enable)
            printf("   SF9DOF: Use is disabled in configuration\n");
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
int initSf9dof(void)
{ //Open first serial port
  int result;
  //
  rxtask.running = 0;
  rxtask.startNewRxCycle = 0;
  // open device
  serif.ttyDev = open(serif.serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = serif.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,"   SF9DOF: Can't open device: %s\n", serif.serialDev);
  else
  { // set baudrate
    result = (set_serial(serif.ttyDev, serif.baudrate) != -1);
    if (result == 0)
      fprintf(stderr,"   SF9DOF: failed to set baudrate to %d\n", serif.baudrate);
    else
      fprintf(stderr,"   SF9DOF: set baudrate to %d bit/s\n", serif.baudrate);
  }
  if (serif.debugFlag == 1)
  { // allow no device in debug mode
    result = 1;
  }
  if (result == 1)
  { // start thread to handle bus
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rxtask.sf9dof_thread, &attr, sf9dof_task, 0))
    {
      perror("   SF9DOF: Can't start sf9dof receive thread");
      result = 0;
    }
  }
  if (result == 1)
  { /****** Create database variables if all is ok **************/
    int waitCount = 0;
    createSf9dofvariables();
    while (!rxtask.running)
    { // wait a bit for thread to start
      usleep(20000); //Don't return before thread is running
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

int limitSignedInt(int value, int min, int max)
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
void * sf9dof_task(void * not_used)
{ // run in thread
  int idleCnt = 0;
  int wait = 0;
  const int MaxWaitCycles = 15; // max number of timeout periods
  const int PollTimeoutMs = 100; // timeout period in ms
  int devErr = 0;
  int cCnt; // character count
  char * gotNL;
  int updCnt = 0;
  struct timeval tv, tvt;
  //
  if (serif.debugFlag)
  {
    logImu = fopen("sf9dof.log", "w");
    logImuTick = fopen("sf9dofTick.log", "w");
  }
  if (0)
  {
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
    {
      perror("SF9DOF: mlockall");
      exit(-1);
    }

    { /* use real-time (fixed priority) scheduler
      * set priority to one less than the maximum
      */
      struct sched_param param;

      param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
      if (sched_setscheduler(0, SCHED_RR, &param)) {
          perror("SF9DOF: setscheduler");
          pthread_exit(0);
      }

    }
  }

  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, "   SF9DOF: signal: can't ignore SIGPIPE.\n");

  fprintf(stderr, "   SF9DOF: rx_task running\n");

    //Mark thread as running
  rxtask.running = 1;
  gettimeofday(&tv, NULL);
  setVariable(sf9dof.varUpdRate, 0, -1);
  while (rxtask.running)
  { // maintain interface
    struct timeval tImu;
    if (serif.ttyDev < 0)
    { // debug mode with no device - act like timeout
      cCnt = 0;
      idleCnt ++;
    }
    else if (pollDeviceRx(serif.ttyDev, PollTimeoutMs))
    { // expect reply from device
      int c = read(serif.ttyDev, &serif.rxBuf[cCnt], MxBL - cCnt - 1);
      gettimeofday(&tImu, NULL);
      if (c <= 0 && ! serif.debugFlag)
      {
        fprintf(stderr, "   SF9DOF: error (%d) reading ttyDev at %d bit/s\n", devErr, serif.baudrate);
        devErr++;
        if (devErr > 10)
        {
          printf("SF9DOF shut down\n");
          break;
        }
      }
      else
      {
        devErr = 0;
        cCnt += c;
        // debug - terminate like a string for debug display
        serif.rxBuf[cCnt] = '\0';
      }
      idleCnt = 0;
      wait = 0;
    }
    else
    { // poll timed out - no data
      cCnt = 0;
      idleCnt++;
    }
    if (cCnt > 0)
    { // line oriented, so look for new-line
      gotNL = strchr(serif.rxBuf, '\n');
      if (gotNL == NULL)
        gotNL = strchr(serif.rxBuf, '\r');
      if (gotNL < serif.rxBuf || (gotNL - serif.rxBuf) >= cCnt)
      {
        gotNL = NULL;
        if (cCnt >= MxBL - 1)
          cCnt = 0;
      }
      else
        while (gotNL[1] > '\0' && gotNL[1] < ' ')
          gotNL++;
    }
    else
      gotNL = NULL;
    // debug
    if (0 && cCnt > 0 && serif.debugFlag == 1)
    { // there is data - print to console
      char * cr = strchr(serif.rxBuf, '\r');
      char cr0[3] = "xxx";
      if (cr != NULL)
      {
        cr0[0] = cr[-1];
        cr0[1] = cr[0];
        cr0[2] = cr[1];
        *cr = ' ';
        printf("SF9DOF: (idle %d) got %d bytes: '%s' (%02x+%02x+%02x)\n", idleCnt, cCnt, serif.rxBuf, cr0[0], cr0[1], cr0[2]);
      }
      else
        printf("SF9DOF: (idle %d) got %d bytes: '%s' - not a full line\n", idleCnt, cCnt, serif.rxBuf);
    }
    // debug end
    if (gotNL != NULL)
    { // receive reply
      int ll = gotNL - serif.rxBuf;
      *gotNL = '\0'; // terminate string
      if (serif.rxBuf[0] == '$')
      { // raw data
        // format $,aaaX,aaaY,aaaZ,gggX,gggY,gggZ,mmmX,mmmY,mmmZ,#\n
        char *p1 = serif.rxBuf;
        int a[3], g[3],m[3];
        int dc;
        p1 += 2;
        dc = sscanf(p1, "%d,%d,%d,%d,%d,%d,%d,%d,%d", &a[0], &a[1], &a[2],
               &g[0], &g[1], &g[2], &m[0], &m[1], &m[2]);
        if (dc == 9)
        { // data is fine
          setVariable(sf9dof.varAcc, 0, a[0]);
          setVariable(sf9dof.varAcc, 1, a[1]);
          setVariable(sf9dof.varAcc, 2, a[2]);
          setVariable(sf9dof.varGyro, 0, g[0]);
          setVariable(sf9dof.varGyro, 1, g[1]);
          setVariable(sf9dof.varGyro, 2, g[2]);
          setVariable(sf9dof.varMag, 0, m[0]);
          setVariable(sf9dof.varMag, 1, m[1]);
          setVariable(sf9dof.varMag, 2, m[2]);
          setVariableDouble('r', sf9dof.varCompas, atan2(m[0], m[1])*180.0 / M_PI); // compass orientation
          setVariableDouble('r', sf9dof.varRoll, atan2(a[0], a[2])*180.0 / M_PI);
          setVariableDouble('r', sf9dof.varPitch, atan2(a[1], a[2])*180.0 / M_PI);
          updCnt++;
          wait = 0;
        }
        else
        {
          printf("SF9DOF: got %d not 9 values as expected\n", dc);
          wait++;
        }
        if (logImu != NULL)
        { // remove from trailing komma
          char *p2 = strstr(p1, ",#");
          if (p2 != NULL)
            *p2 = '\0';
          fprintf(logImu, "%lu.%06lu %d %s\n", tImu.tv_sec, tImu.tv_usec, updCnt, p1);
        }
      }
      else if (serif.rxBuf[0] == '[')
      { // help text (menu items)
        printf("SF9DOF: version %s\n", gotNL);
      }
      else if (strncmp(serif.rxBuf, "9DOF", 4) == 0)
      { // version string
        printf("SF9DOF: version %s\n", gotNL);
      }
      else
        // no usefull data
        wait++;
      // skip the used line
      ll++; // advance index to first unused
      if (cCnt > ll)
      {
        memmove(serif.rxBuf, &serif.rxBuf[ll], cCnt - ll);
        cCnt -= ll;
      }
      else
        cCnt = 0;
    }
    else
      // no data
      wait++;
    //
    gettimeofday(&tvt, NULL);
    if ((tvt.tv_sec - tv.tv_sec)*1000000 + (tvt.tv_usec - tv.tv_usec) > 5000000)
    { // 5 seconds has passed - calculate update rate
      setVariable(sf9dof.varUpdRate, 0, updCnt/5);
      updCnt = 0;
      tv = tvt;
    }
    if (wait > MaxWaitCycles)
    { // no data stream
      const int MSL = 4;
      char s[MSL];
      int n;
      timeoutCnt++;
      if (timeoutCnt % 10 == 1)
        printf("SF9DOF: timed out %d times each %dms, no (usable) data - sends ctrl-z\n",
              timeoutCnt, PollTimeoutMs);
      snprintf(s, MSL, "%c\n", 0x1a);
      n = secure2Write(serif.ttyDev, s, 2); 
      if (n <= 0)
      { // data could not be written to device
        printf("SF9DOF: failed to write to %s, trying to reopen ...", serif.serialDev);
        fflush(stdout);
        close(serif.ttyDev);
        serif.ttyDev = open(serif.serialDev, O_RDWR /*| O_NONBLOCK*/);
        if (serif.ttyDev != -1)
        {
          set_serial(serif.ttyDev, serif.baudrate);
          printf(" [reopened OK]\n");
        }
        else
          printf(" [failed]\n");
      }
      wait = 0;
      cCnt = 0;
    }
  }
  rxtask.running = 0;
  fprintf(stderr,"SF9DOF: closing bus device\n");
  close(serif.ttyDev);
  if (logImu != NULL)
    fclose(logImu);
  fprintf(stderr,"SF9DOF: Shutting down thread\n");
  pthread_exit(0);
  return NULL;
}

///////////////////////////////////////////////////////////

/**
 * Create variables for the usb to sf9dof converter itself */
void createSf9dofvariables()
{ // version
  sf9dof.varVersion = createVariable('r', 1, "version");
  // accelerometer values
  sf9dof.varAcc = createVariable('r', 3, "sf9acc");
  // gyro values
  sf9dof.varGyro = createVariable('r', 3, "sf9gyro");
  // magnetometer values
  sf9dof.varMag = createVariable('r', 3, "sf9mag");
  // calculated rotation from acc
  sf9dof.varRoll = createVariable('r', 2, "sf9roll");
  // calculated rotation from acc
  sf9dof.varPitch = createVariable('r', 2, "sf9pitch");
  // calculated heading from compas - based on magnetic north
  sf9dof.varCompas = createVariable('r', 2, "sf9compas");
  /// update rate
  sf9dof.varUpdRate = createVariable('r', 1, "sf9updateRate");
}

////////////////////////////////////////////////////////////////

int pollDeviceRx(int fd, int msTimeout)
{
  int err = 0;
  int dataAvailable = 0;
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
    perror("UServerPort::serviceClients (poll)");
  }
  else if (err > 0)
  { // at least one connection has data (or status change)
    revents = pollStatus.revents;
    if (((revents & POLLIN) != 0) ||
        ((revents & POLLPRI) != 0))
      dataAvailable = 1;
  }
  return dataAvailable;
}

////////////////////////////////////////////////////////////

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
