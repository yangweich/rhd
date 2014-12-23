 /** \file usbiss.c
 *  \ingroup hwmodule
 *
 * interface to usb-to-i2c module and a MD25 mototcontroller (2xdc motor control with gearing and encoder)
 *
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 129 $:"
 #define DATE             "$Date: 2011-09-04 16:07:32 +0200 (Sun, 04 Sep 2011) $:"
 #define ID               "$Id: herkulex.c 129 2013-03-03 08:13:17Z jcan $"
 #define PLUGINNAME        "HerkuleX"
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

#include "herkulex.h"

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
/// set control axis from ini-string
void setControlAxis(servo * axis, const char * val, const char * name);
/**
 * Set servo position if changed, and send to servo */
int setServos(float angleLeft, float angleRight);
/**
 * Convert code to temp for herkulex - based on simplified table in manual */
float codeToTemp(int code);
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
 * Set torque on for servo movement 
 * \param serviIdx is servo index number - 0 or 1
 * \param mode 0=free, 1=on, 2=break
 * \returns 1 if setting is acknowledged */
int setTorqueMode(int servoIdx, int mode);

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
int setRamReg1(int servoIdx, uint8_t addr, uint8_t value);
int setRamReg2(int servoIdx, uint8_t addr, uint16_t value);
/**
 * print error flags to console.
 * \param servo is index to servo (0 or 1)
 * \param ss is 16bit state flag MSByte=reg48 LSByte=reg49  */
void printErrorToConsole(int servo, int ss);

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
//int keepAlive = 0;
int debugFlag = 0;
int debugIO = 0;
int steerUpd = 0; // steering is updated
float sacOld = 0.0; // old steeringAngleRef
FILE * logfile = NULL;
FILE * logState = NULL;
int steerMode = 0; // 0 = servo control direct; 1 = steering control
int servoReset[2] = {0,0};
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
  //
  tick = rhdTick;
  rxtask.startNewRxCycle = 1;
  if (rhdTick == 0)
  { // look for emergency switch
    int i = getDatabaseVariable('r', "emergencyswitch");
    if (i >= 0)
    {
      state.emergencyswitch = i; 
      printf(PLUGINNAME ": emergency switch index found %d\n", state.emergencyswitch);
    }
    else
    {
      printf(PLUGINNAME ": emergency switch not found - running without\n");
      state.emergencystop = 0;
    }
  }
  if (state.emergencyswitch >= 0)
    state.emergencystop = getReadVariable(state.emergencyswitch, 0);
  //
  if (isUpdated('w', state.steeringAngleRef))
  {
    state.steerRef = getWriteVariable(state.steeringAngleRef, 0) * M_PI / 1800.0;
    steerUpd = 1;
  }
  if (isUpdated('w', state.servoMode))
    steerMode = getWriteVariable(state.servoMode, 0);
  if (isUpdated('w', state.servoReset))
  {
    servoReset[0] = getWriteVariable(state.servoReset, 0);
    servoReset[1] = getWriteVariable(state.servoReset, 1);
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
  setTorqueMode(0, 0);
  setTorqueMode(1, 0);
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
  state.wheelBase = 0.203;
  state.steerBase = 0.255;
  state.servoDeadZoneVal = 5;
  state.servoSaturatorOffsetVal = 0;
  state.servoSaturatorSlopeVal = 0;
  state.servoAccRatioVal = 19;
  state.servoMaxAccVal = 5;
  state.servoMaxPWMVal = 1023;
  state.servoMinPWMVal = 3;
  state.servoPlayTime90degVal = 60;
  state.getRam = 0;
  busif.baudrate = 115200;
  busif.debugIoVal = 0;
  state.emergencystop = 1;
  state.emergencyswitch = -1;
  strncpy(busif.serialDev, "/dev/ttyUSB0", MxDL);
  //Print initialization message
  //Find revision number from SVN Revision
  char * p1;
  char versionString[20] = REVISION;
  char tempString[10];
  p1 = strrchr(versionString, '$');
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  printf("herkulex: plug-in version %s\n", tempString);
  setControlAxis(&state.servos[0], "0 12 -830 0 900", "left"); // left
  setControlAxis(&state.servos[1], "1 13 -750 0 750", "right"); // right
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
        else if (strcmp("robot",el) == 0)
        { // we need something from robot configuration
          for(i = 0; attr[i]; i+=2)
          {
            if (strcasecmp("wheelbase",attr[i]) == 0)
              state.wheelBase = strtod(attr[i+1], NULL);
            else if (strcasecmp("steerbase",attr[i]) == 0)
              state.steerBase = strtod(attr[i+1], NULL);
            else if (strcasecmp("frontbase",attr[i]) == 0)
              state.frontBase = strtod(attr[i+1], NULL);
          }
        }
        else
          info->skip = info->depth;
        break;
      case 3:
        // this one handles usbiss only
        if (strcmp("herkulex",el) == 0)
        { // is it enabled, the only info needed
          for(i = 0; attr[i]; i+=2)
          {
            if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0))
              info->enable = 1;
            // joystick override button number
            else if (strcmp("servoLeft", attr[i]) == 0)
              setControlAxis(&state.servos[0], attr[i+1], NULL);
            else if (strcmp("servoRight", attr[i]) == 0)
              setControlAxis(&state.servos[1], attr[i+1], NULL);
            if (strcmp("dev",attr[i]) == 0)
              strncpy(busif.serialDev,attr[i+1], MxDL);
            if (strcmp("baudrate",attr[i]) == 0)
              busif.baudrate = strtol(attr[i+1], NULL, 0);
/*deadzone="3"
       saturatorOffset="0"
       saturatorSlope="0"
       minPWM="5"
       maxPWM="1023"
       playtime90deg="50"*/
            if (strcmp("deadzone",attr[i]) == 0)
              state.servoDeadZoneVal = strtol(attr[i+1], NULL, 0);
            if (strcmp("saturatorOffset",attr[i]) == 0)
              state.servoSaturatorOffsetVal = strtol(attr[i+1], NULL, 0);
            if (strcmp("saturatorSlope",attr[i]) == 0)
              state.servoSaturatorSlopeVal = strtol(attr[i+1], NULL, 0);
            if (strcmp("minPWM",attr[i]) == 0)
              state.servoMinPWMVal = strtol(attr[i+1], NULL, 0);
            if (strcmp("maxPWM",attr[i]) == 0)
              state.servoMaxPWMVal = strtol(attr[i+1], NULL, 0);
            if (strcmp("maxAcc",attr[i]) == 0)
              state.servoMaxAccVal = strtol(attr[i+1], NULL, 0);
            if (strcmp("accRatio",attr[i]) == 0)
              state.servoAccRatioVal = strtol(attr[i+1], NULL, 0);
            if (strcmp("playtime90deg",attr[i]) == 0)
              state.servoPlayTime90degVal = strtol(attr[i+1], NULL, 0);
            if (strcmp("getRAM",attr[i]) == 0)
              state.getRam = strtol(attr[i+1], NULL, 0);
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

/**
 * Packes transmit package into transmit buffer txBuf.
 * \param pID is the servo to adress - here 0 or 1
 * \param CMD is the command type - 0 to 9 is valid
 * \param data is payload and is command dependent 
 * \param dataCnt is number of bytes in data
 * \returns number of bytes in tx buffer */
int packServoMsg(uint8_t pID, uint8_t CMD, uint8_t data[], int dataCnt)
{
  uint8_t cs1 = 0;
  int i;
  busif.txBuf[0] = 0xff;  
  busif.txBuf[1] = 0xff;
  busif.txBuf[2] = 7 + dataCnt;  
  busif.txBuf[3] = pID;
  busif.txBuf[4] = CMD;
  cs1 = (7 + dataCnt) ^ pID ^ CMD;
  for (i = 0; i < dataCnt; i++)
  {
    busif.txBuf[7 + i] = data[i];
    cs1 = cs1 ^ data[i];
  }
  busif.txBuf[5] = cs1 & 0xfe;  
  busif.txBuf[6] = (~cs1) & 0xfe;
  return 7 + dataCnt;
}

/**
 * Check for ACK reply
 * \param CMD is the CMD the reply is expected to match 
 * \param rxCnt is number of bytes in reply 
 * \returns 1 if ack, else 0 */
int isAckOK(uint8_t CMD, int RxCnt)
{
  int cs1;
  int result = 0;
  int i;
  if ((RxCnt >= 7) && (busif.rxBuf[4] == (CMD | 0x40)))
  { // length is OK, and CMD type is OK too
    // check sum 1
    cs1 = busif.rxBuf[2] ^ busif.rxBuf[3] ^ busif.rxBuf[4];
    for (i = 7; i < RxCnt; i++)
      cs1 = cs1 ^ busif.rxBuf[i];
    if ((busif.rxBuf[5] == (cs1 & 0xfe)) &&
        (busif.rxBuf[6] == (~cs1 & 0xfe)))
      result = 1;
  }
  return result;
}

/**
 * transmit package to herkulex, maxPWMand check reply.
 * \param pID is the servo to adress - here 0xfc or 0xfd
 * \param CMD is the command type - 0 to 9 is valid
 * \param data is payload and is command dependent 
 * \param dataCnt is number of additional bytes in data (the command parameters - if any)
 * \param rxDataCnt is expected number of bytes to receive:
 * 0 is no reply expected; 
 * <7 is wait for timeout (or buffer is full);
 * >= 7 wait for exactly this number of bytes.
 * \returns 1 if reply is OK, the reply is in rxBuf - if any */
int sendToHerkulex(uint8_t pID, uint8_t CMD, uint8_t data[], int dataCnt, int rxDataCnt)
{
  int ns, n, i, isOK;
  ns = packServoMsg(pID, CMD, data, dataCnt);
  // write to device
  secure2Write(busif.txBuf, ns);
  // debug
  if (debugIO)
  {
    printf(PLUGINNAME ": send %2d: %2x %2x %2x %2x %2x %2x %2x :", ns, busif.txBuf[0], busif.txBuf[1], busif.txBuf[2], busif.txBuf[3], busif.txBuf[4], busif.txBuf[5], busif.txBuf[6]);
    for (i = 0; i < dataCnt; i++)
      printf(" %2x", busif.txBuf[7 + i]);
    printf("\n");
  }
  // debug end
  // get ack
  if (rxDataCnt == 0)
    isOK = 1;
  else
  { // reply is expected
    if (rxDataCnt < 7)
      n = getDataToTimeout(busif.rxBuf, MxBL, 15);
    else
      // return when rxDataCnt is reached (and swallow up to one more byte)
      n = getDataToTimeout(busif.rxBuf, rxDataCnt + 1, 30);
//      n = secureRead(busif.ttyDev, busif.rxBuf, rxDataCnt);
    isOK = isAckOK(CMD, n);
  }
  // debug
  if (debugIO && (n > 0) && (rxDataCnt != 0))
  {
    printf(PLUGINNAME ":  got %2d: %2x %2x %2x %2x %2x %2x %2x :", n, busif.rxBuf[0], busif.rxBuf[1], busif.rxBuf[2], busif.rxBuf[3], busif.rxBuf[4], busif.rxBuf[5], busif.rxBuf[6]);
    for (i = 7; i < n; i++)
      printf(" %2x", busif.rxBuf[i]);
    if (isOK)
      printf(" (crc OK)\n");
    else
      printf(" (crc failed)\n");
  }
  // debug end
  // check result
  return isOK;
}

#define EEP_WRITE 0x01
#define EEP_READ  0x02
#define RAM_WRITE 0x03
#define RAM_READ  0x04
#define I_JOG     0x05
#define S_JOG     0x06
#define STAT      0x07
#define ROLLBACK  0x08
#define REBOOT    0x09


/**
 * Get selected status from servos */
int getServoStatusFull(int servoIdx)
{ // get servo status for servo i
  int i, n;
  uint8_t getCalPos[2] = {0, 10};
  int res[100];
  int isOK = 1;
//  int j = debugIO;
//   debugIO = 1;
  for (i = 0; i < 7; i++)
  {
    getCalPos[0] = i * 10;
    getCalPos[1] = 10;
    // send request for data and receive result buffer
    isOK &= sendToHerkulex(state.servos[servoIdx].servo, 
                  RAM_READ, getCalPos, 2, 7 + 2 + 10 + 2);
    if (state.getRam)
      for (n = 0; n < 10; n++) 
        res[i*10 + n] = busif.rxBuf[9 + n];
  }
  getCalPos[0] = 70;
  getCalPos[1] = 3;
  // send request for data and receive result buffer
  isOK &= sendToHerkulex(state.servos[servoIdx].servo, 
                RAM_READ, getCalPos, 2, 7 + 2 + 3 + 2);
  if (state.getRam)
  {
    for (n = 0; n < 3; n++) 
      res[7*10 + n] = busif.rxBuf[9 + n];
    setArray(state.servoFullRAM[servoIdx], 73, res);
  }
//  debugIO = j;
  return isOK;
}

  
  /**
 * Get selected status from servos */
int getServoStatusOne(int servoIdx)
{ // get servo status for servo i
#define READ_BYTES 14
#define READ_START_ADR 52
  uint8_t getCalPos[2] = {READ_START_ADR, READ_BYTES};
  int volt, temp, mode, tick, pos, dpos, pwm, torque, led;
  int ss; // servo state
  int isOK = 1;
  // send request for data and receive result buffer
  isOK &= sendToHerkulex(state.servos[servoIdx].servo, 
                RAM_READ, getCalPos, 2, 7 + 2 + READ_BYTES + 2);
  if (isOK)
  { // torque control
    torque = busif.rxBuf[9];
    // LED setting
    led = busif.rxBuf[10];
    // servo voltage
    volt = roundi(busif.rxBuf[11] * 7.4);
    state.servos[servoIdx].volt = volt/100.0;
    // temperature
    temp = busif.rxBuf[12];
    state.servos[servoIdx].temp = codeToTemp(temp);
    // mode - should be 0 all the time
    mode = busif.rxBuf[13];
    // tick counter (each 11.2 ms)
    tick = busif.rxBuf[14];
    // position
    pos = busif.rxBuf[17] + ((int)busif.rxBuf[18] << 8);
    state.servos[servoIdx].position = pos;
    // position chage
    dpos = busif.rxBuf[19] + ((int)busif.rxBuf[20] << 8);
    // position chage
    pwm = busif.rxBuf[21] + ((int)busif.rxBuf[22] << 8);
    // get also servo state from status bytes
    ss = ((int)busif.rxBuf[23] << 8) + busif.rxBuf[24];
    if ((ss & 0x0800) == 0)
      // remove old detail bits, if no com error
      ss &= 0xffc3;
    state.servos[servoIdx].errorStatus = ss;
    if (debugIO && (ss & ~0x8282) != 0)
      printErrorToConsole(servoIdx, ss & ~0x8282);
    // update RHD variables
    setVariable(state.servoTorque, servoIdx, torque);
    setVariable(state.servoLed, servoIdx, led);
    setVariable(state.servoVolt, servoIdx, volt);
    setVariable(state.servoPos, servoIdx, pos);
    setVariable(state.servoTick, servoIdx, tick);
    setVariable(state.servoPosDif, servoIdx, dpos);
    setVariable(state.servoPosVel, servoIdx, mode);
    // temperature in degrees * 10
    setVariable(state.servoTemp, servoIdx, roundi(state.servos[servoIdx].temp * 10.0));
    // set PWM (torque)
    setVariable(state.servoPWM, servoIdx, pwm);
    // set status for both servos
    setVariable(state.servoStatus, servoIdx, ss);
    state.servos[servoIdx].comFailCnt = 0;
  }
  else
    state.servos[servoIdx].comFailCnt++;
  return isOK;
}


/**
 * Get selected status from servos */
int getServoStatus()
{ // get servo status for servo i
  int isOK;
  isOK  = getServoStatusOne(0);
  isOK &= getServoStatusOne(1);
  if (isOK)
  { // Calculate actual steering angle
    float a, a0, a1; //, tb; // wheel angle in radians
    // left
    a0 = (state.servos[0].position - state.servos[0].center) * 
          state.servos[0].scale;
    // right
    a1 = (state.servos[1].position - state.servos[1].center) * 
          state.servos[1].scale;
    // convert to robot steering angle (based on each wheel)
//     tb = tan(a0);
//     a0 = atan2(tb, 1 - state.frontBase / 2.0 * tb / state.steerBase);
//     tb = tan(a1);
//     a1 = atan2(tb, 1 - state.frontBase / 2.0 * tb / state.steerBase);
    // they may not totally agree, so use average
    // they are mounted upside down, so change sign
    a = (a0 + a1) / -2.0;
    // set actual steering angle in degrees * 10
    setVariable(state.steeringAngle, 0, roundi(a * 1800.0 / M_PI));
  }
  if (debugIO > 0)
    debugIO--;
  else
    debugIO = 0;
  return isOK;
}

//////////////////////////////////////////////////////////////////

/**
 * Set servo ID once only - this sets the (one) servo on line to
 * ID=0xfc (from default ID as is 0xfd) */
void setServoID(void)
{ // Use this command only once to change pID for a servo
  uint8_t SET_PID[3] = {6, 1, 0xFC};
  int result;
  // get stat from all servos
  printf(PLUGINNAME ": set servo ID to FC from default FD, need power cycle to implement\n");
  result  = sendToHerkulex(0xfd, EEP_WRITE, SET_PID, 3, 0);
  if (result == 0)
    printf("failed to set servo ID\n");
  result |= debugFlag;
}

/////////////////////////////////////////////////////////////

/**
 * Setup one servo 
 * returns trus if set (reply acknowledged) */
int setupServo(int idx)
{
  int result = 1;
  uint8_t SET_ACK_POLICY[3] = {1, 1, 2};
  int pID = state.servos[idx].servo;
  printf(PLUGINNAME ": servo %d, ID %d setup:\n", idx, pID);
  // ACK policy = 0 - no reply
  // ACK policy = 1 - only on read
  // ACK policy = 2 - reply on all
  // set ACK policy to reply on all requests
  //result  = sendToHerkulex(1, RAM_WRITE, SET_ACK_POLICY, 1);
  printf(PLUGINNAME " * set ack policy to %d\n", SET_ACK_POLICY[2]);
  result  = sendToHerkulex(pID, RAM_WRITE, SET_ACK_POLICY, 3, -1);
  if (result == 0)
    printf(PLUGINNAME " --- failed to set ACK policy\n");
  result |= debugFlag;
  //
  if (result)
  { 
    uint8_t W_MAX_VOLTAGE_12V[3] = {7, 1, 0xa8};
    // send command to allow 12V
    // is 12.2V = 0.074V * 0xa8 
    printf(PLUGINNAME " * set max 12V\n");
    result  = sendToHerkulex(pID, RAM_WRITE, W_MAX_VOLTAGE_12V, 3, -1);
    if (result == 0)
      printf(PLUGINNAME " --- failed to set maximum allowed voltage\n");
    result |= debugFlag;
    // get version number
    if (result)
    { // model and version - reads 3 bytes from adr 1
      // should be 0x02,0x00,0x90
      uint8_t R_MODEL_VER[2] = {1, 3};
      printf(PLUGINNAME ": * get VERSION\n");
      result = sendToHerkulex(pID, EEP_READ, R_MODEL_VER, 2, -1);
      if (result)
        state.servos[idx].version = (busif.rxBuf[9] << 16) + 
                                  (busif.rxBuf[10] << 18) + 
                                  (busif.rxBuf[11] << 16);
      if (result == 0)
        printf(PLUGINNAME " --- failed to get version info\n");
      result |= debugFlag;
    }
    /// get also general status from each servo
    printf(PLUGINNAME ": * get servo status and position\n");
    result &= getServoStatusOne(idx);
    if (result == 0)
      printf("failed to get full servo status\n");
    result |= debugFlag;
  }
  if (result)
  { // set deadzone (ticks around target position with no move)
    setRamReg1(idx, 8, state.servoDeadZoneVal);
    // set saturator offset start pwm outside dead zone (0 = off)
    setRamReg1(idx, 8, state.servoSaturatorOffsetVal);
    // set saturator slope PWM value increase for position error (plus offset) outside deadzone
    setRamReg1(idx, 8, state.servoSaturatorSlopeVal);
    // set acceleration ratio
    setRamReg1(idx, 8, state.servoAccRatioVal);
    // max acc setting
    setRamReg1(idx, 9, state.servoMaxAccVal);
    // max PWM setting
    setRamReg2(idx, 16, state.servoMaxPWMVal);
    // min PWM setting
    setRamReg1(idx, 15, state.servoMinPWMVal);
  }
  return result;
}

/**
 * reset servo to boot condition and clear error flags
 * \returns 1 if successful */
int resetServo(int servoIdx)
{
  int result = 0;
  int k;
  servoReset[servoIdx] = 0;
  for (k = 0; k < 5; k++)   
  { // try 5 times to set servo to get error free status at end
    printf(PLUGINNAME " Resetting servo %d (attempt %d):\n", servoIdx, k);
    if (setupServo(servoIdx))
    { 
      int e = state.servos[servoIdx].errorStatus;
      if ((e & ~0x82c3) == 0)
      { // all is fine
        result = 1;
        break;
      }
      else
      {
        printErrorToConsole(servoIdx, e);
        // reboot
        if (0)
        { // reset servo
          printf(PLUGINNAME " * reboot servo\n");
          sendToHerkulex(state.servos[servoIdx].servo, REBOOT, 0, 0, -1);
        }
        // clear error register
        setRamReg2(servoIdx, 48, 0);
      }
    }
  }
  return result;
}


/**
 * Open device, initialize device in right mode, and get first status */
int openAndSetMode()
{
  int result = 0;
  int s;
  // open device
  busif.ttyDev = open(busif.serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = busif.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,PLUGINNAME ": Can't open device: %s\n", busif.serialDev);
  else
  { // set baudrate
    result = (set_serial(busif.ttyDev, busif.baudrate) != -1);
    if (result == 0)
      fprintf(stderr, PLUGINNAME ": Can't set serial port parameters\n");
  }
  if (debugFlag)
    // allow no device in debug mode
    result = 1;
  busif.lostConnection = ! result;
  if (result)
    printf(PLUGINNAME ": opened device %s at %d bit/sec sucessfully\n", busif.serialDev, busif.baudrate);
  if (result)
  { // empty data from device
    int n = 1;
    while (n > 0)
      n = getDataToTimeout(busif.rxBuf, MxBL, 100);
  }
  debugIO = busif.debugIoVal;
  for (s = 0; s < 2; s++)
  { // set both servos
    if (resetServo(s) == 0)
    {
      result = 0;
      printf(PLUGINNAME ": servo %d reset failed\n", s);
    }
  }
  result |= debugFlag;
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

////////////////////////////////////////////////////

/**
 * Set torque on for servo movement 
 * \param serviIdx is servo index number - 0 or 1
 * \param addr is ram address for the byte value
 * \param value is the 8-bit value to write
 * \returns 1 if setting is acknowledged */
int setRamReg1(int servoIdx, uint8_t addr, uint8_t value)
{
    int result = 1;
  uint8_t cmd[3] = {addr, 1, value};
  int pID = state.servos[servoIdx].servo;
  //
  printf(PLUGINNAME ": setting servo %d (0x%x) 8bit register %d to %d:\n", servoIdx, pID, addr, value);
  result  = sendToHerkulex(pID, RAM_WRITE, cmd, 3, -1);
  if (result == 0)
    printf(PLUGINNAME " --- failed to set %d (0x%x) at address %d for servo %d (0x%x)\n",
      value, value, addr, servoIdx, pID);
  result |= debugFlag;
  return result;
}

/**
 * Set torque on for servo movement 
 * \param serviIdx is servo index number - 0 or 1
 * \param addr is ram address at start of 2-byte value
 * \param value is the 16-bit value to write
 * \returns 1 if setting is acknowledged */
int setRamReg2(int servoIdx, uint8_t addr, uint16_t value)
{
    int result = 1;
  uint8_t cmd[4] = {addr, 2, value & 0xff, value >> 8};
  int pID = state.servos[servoIdx].servo;
  //
  printf(PLUGINNAME ": setting servo %d (0x%x) 16bit register %d to %d:\n", 
         servoIdx, pID, addr, value);
  result  = sendToHerkulex(pID, RAM_WRITE, cmd, 4, -1);
  if (result == 0)
    printf(PLUGINNAME " --- failed to set %d (0x%4x) at address %d for servo %d (0x%x)\n",
      value, value, addr, servoIdx, pID);
  result |= debugFlag;
  return result;
}

//////////////////////////////////////

int setTorqueMode(int servoIdx, int mode)
{
    int result = 1;
  uint8_t SET_TORQUE_ON[3] = {52, 1, 0x60};
  uint8_t SET_TORQUE_BREAK[3] = {52, 1, 0x40};
  uint8_t SET_TORQUE_FREE[3] = {52, 1, 0x00};
  int pID = state.servos[servoIdx].servo;
  uint8_t * modeCmd = SET_TORQUE_FREE;
  if (mode == 1)
    modeCmd = SET_TORQUE_ON;
  else if (mode == 2)
    modeCmd = SET_TORQUE_BREAK;
  printf(PLUGINNAME ": setting servo %d torque to %d:\n", servoIdx, mode);
  result  = sendToHerkulex(pID, RAM_WRITE, modeCmd, 3, -1);
  if (result == 0)
    printf(PLUGINNAME " --- failed to set torque\n");
  result |= debugFlag;
  return result;
}

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

///RS232  Recieve thread
void * run(void * not_used)
{ // run in thread
  int seq = 0;
  int loopCnt = 0;
  struct timeval tm;
  int torqueOK = 0;
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
    fprintf(stderr, PLUGINNAME ": signal: can't ignore SIGPIPE.\n");

  //fprintf(stderr, "   USBISS: rx_task running\n");
  if (debugFlag)
  {
    logfile = fopen(PLUGINNAME ".log", "w");
    //logState = fopen("usbiss-state.log", "w");
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
  //Mark thread as running
  rxtask.running = 1;
  busif.txBuf[0] = '\0';
  while (!rxtask.shutDown)
  { // maintain interface
    loopCnt++;
    if (logState != NULL)
    {
      if (seq == 0)
        fprintf(logState,"\n");
      gettimeofday(&tm, NULL);
      fprintf(logState, "%lu.%06lu %d %2d\n ", tm.tv_sec, tm.tv_usec, loopCnt, seq);
    }
//     if (busif.lostConnection)
//     { // connection is lost - during read or write
//       if (busif.ttyDev >= 0)
//       {
//         close(busif.ttyDev);
//         busif.ttyDev = -1;
//         printf(PLUGINNAME ":**** lost connection - trying to reconnect\n");
//         sleep(1);
//       }
//       // wait a while - for udev to detect device is back
//       sleep(1);
//       // try to open
//       openAndSetMode();
//       if (busif.lostConnection)
//         // connection is still lost, so try again
//         continue;
//       torqueOK = 0;
//     }
    if (state.emergencystop)
    {
      if (torqueOK)
      { // running with full torque, release for emergency
        setTorqueMode(0, 0); 
        setTorqueMode(1, 0);
        torqueOK = 0;
        printf(PLUGINNAME ":torque off (emergency switch)\n");
      }
    }
    else if (torqueOK == 0)
    { // activate servo
      setTorqueMode(0, 1);
      setTorqueMode(1, 1);
      printf(PLUGINNAME ":torque on\n");
      torqueOK = 1;
    }
    switch (seq)
    { // sync
      case 1:
        // wait for next rhd time tick
        sem_wait(&tickSem);
        gettimeofday(&tickTime, NULL);
        //printf("tick unlocked new=%d\n", rxtask.startNewRxCycle);
        rxtask.startNewRxCycle = 0;
        // check for reset
        if (servoReset[0] || servoReset[1])
        {
          debugIO = busif.debugIoVal;
          if (servoReset[0])
          {
            resetServo(0);
            setTorqueMode(0, 1);
          }
          if (servoReset[1])
          {
            resetServo(1);
            setTorqueMode(1, 1);
          }
          // debug
          // dump full servo RAM
          getServoStatusFull(0);
          getServoStatusFull(1);
          // debug end
        }
//         if (isUpdated('w', state.servoPlayTime))
//         { /// play time variable
//           state.servoPlayTimeVal = 
//             limit(0, 255, getWriteVariable(state.servoPlayTime, 0));
//           /// @todo mangler implementering
//         }
//         if (isUpdated('w', state.servoAccRatio))
//         { /// acceleration ratio
//           state.servoAccRatioVal = getWriteVariable(state.servoAccRatio, 0);
//           setRamReg1(0, 8, state.servoAccRatioVal);
//           setRamReg1(1, 8, state.servoAccRatioVal);
//         }
//         if (isUpdated('w', state.servoMaxAcc))
//         { /// max acc setting
//           state.servoMaxAccVal = getWriteVariable(state.servoMaxAcc, 0);
//           setRamReg1(0, 9, state.servoMaxAccVal);
//           setRamReg1(1, 9, state.servoMaxAccVal);
//         }
//         if (isUpdated('w', state.servoMaxPWM))
//         { /// max PWM setting
//           state.servoMaxPWMVal = getWriteVariable(state.servoMaxPWM, 0);
//           setRamReg2(0, 16, state.servoMaxPWMVal);
//           setRamReg2(1, 16, state.servoMaxPWMVal);
//         }
//         if (isUpdated('w', state.servoMinPWM))
//         { /// min PWM setting
//           state.servoMinPWMVal = getWriteVariable(state.servoMinPWM, 0);
//           setRamReg1(0, 15, state.servoMinPWMVal);
//           setRamReg1(1, 15, state.servoMinPWMVal);
//         }
        break;
      // send new settings and request for data    
      case 2: // set servo raw
        if (getWriteVariable(state.servoMode, 0) == 2)
        { // control servo position directly 
        }
        break;
      case 3: // set steering servo positions
        // get steering angle in radians
//         state.steerRef = getWriteVariable(state.steeringAngleRef, 0) * M_PI / 1800.0;
        if (0) // tick %10 == 0)
          printf(PLUGINNAME ": set servo, ref %g radians\n", state.steerRef);
        if (steerUpd && fabs(state.steerRef - sacOld) > 1e-6)
        { // steering changed - calculate new angle for left and right wheel
          float tanSac, ofs, sal, sar;
          // reset update flag
          steerUpd = 0;
          // get tan to steering angle, and approximate, if >= 90 deg
          if (fabs(state.steerRef) < (M_PI/2.0 - 1e-5))
            tanSac = tan(state.steerRef);
          else // use +-90 degrees
            tanSac = state.steerRef * 1e10;
          // sal = atan2(steerBase, turnRadius - frontBase/2)
          // sar = atan2(steerBase, turnRadius + frontBase/2)
          // turnRadius = steerBase / tanSac
          // multiply with tanSac and divide by steerBase gives
          // sal = atan2(tanSac, 1 - frontBase/2 * tanSac / steerBase)
          // sar = atan2(tanSac, 1 + frontBase/2 * tanSac / steerBase)
          ofs = state.frontBase * tanSac / 2.0 / state.steerBase;
          // find angle in radians for each wheel
          sal = atan2(tanSac, 1.0 - ofs);
          sar = atan2(tanSac, 1.0 + ofs);
          // implement new servo positions
          // debug
          debugIO = busif.debugIoVal;
          // debug end
          setServos(-sar, -sal);
          sacOld = state.steerRef;
        }
        break;
      case 4: 
        // get actual servo status (every 4th time)
        if (tick % 4 == 0)
        { // get subset of status values
          int i;
          /* rx CMD (RAM ADDRESS)
              9 54 Voltage 1 byte (8 bit  0.074 V/LSB
             10 55 Temperature 1 byte (8-bit non-linear)
             11 56 Current Control Mode 1 byte (0: position, 1=velocity))
             12 57 Tick 1 byte (11.2ms per tick)
             13 58 Calibrated Position 2 bytes (LSB,MSB 10 bit)
             15 60 Absolute Position 2 bytes (LSB,MSB)
             17 62 Differential Position 2 bytes (LSB,MSB) position change each 11.2ms
             19 64 PWM 2 bytes (LSB,MSB) PWM (torque) to motor
             21 status bye 1 (error)
             22 status detailed
          */
          getServoStatus();
          for (i = 0; i < 2; i++)
          {
            if ((state.servos[i].errorStatus & 0x1400) > 0)
            { // print servo hard error - overtemp
              setTorqueMode(i, 0);
              printf(PLUGINNAME ": servo %d has hard error - set to no torque! - power cycle needed\n", i);
              printErrorToConsole(state.servos[i].errorStatus, 0);
              state.servos[i].hardError = 1;
            }
            else if ((state.servos[i].errorStatus & 0x7b3c) > 0)
            {
              printf(PLUGINNAME ": servo %d has resetable error - reset\n", i);
              // reset flags
              setRamReg2(i, 48, 0);
              state.servos[i].hardError = 0;
            }
            else
            { // no hard error
              int old = getReadVariable(state.servoError, i);
              if (old == 1)
              { // recovered
                printf(PLUGINNAME ": servo %d has recovered - reset\n", i);
                resetServo(i);
              }
              state.servos[i].hardError = 0;
              setVariable(state.servoError, i, 0);
            }
            if (state.servos[i].hardError | (state.servos[i].comFailCnt > 10))
            { // set hard error status 
              int old = getReadVariable(state.servoError, i);
              if (old == 0)
              {
                printf(PLUGINNAME ": servo %d has hard temp=%.1f comErrCnt=%d\n",
                    i, state.servos[i].temp, state.servos[i].comFailCnt);
                printf(PLUGINNAME ": servo %d setting to free mode\n", i);
                setRamReg1(i, 52, 0);
              }
              setVariable(state.servoError, i, 1);
            }
          }
        }
        break;
      case 5:
        if (state.getRam && tick %16 == 2)
          getServoStatusFull(0);
        else if (state.getRam && tick %16 == 6)
          getServoStatusFull(1);
        break;  
      default:
        seq = 0;
        break;
    }
    seq++;
  }
  rxtask.running = 0;
  // set servos to free
//   setTorqueMode(0, 0);
//   setTorqueMode(1, 0);
//   fprintf(stderr,PLUGINNAME ": closing bus device\n");
//   close(busif.ttyDev);
  fprintf(stderr,PLUGINNAME ": thread shut down\n");
//   if (logfile != NULL)
//     fclose(logfile);
//   if (logState != NULL)
//     fclose(logState);
  pthread_exit(0);
  return NULL;
}

///////////////////////////////////////////////////////////

/**
 * Create variables for the usb to i2c converter itself */
void createVariables() 
{
  // - if using ackerman steering or just servo control (to find limits)
  state.servoMode = createVariable('w',1,"hxServoMode");
  state.servoRef = createVariable('w',2,"hxServoRef");
  state.servoReset = createVariable('w',2,"hxServoReset");
  // steeringangle in radians and uRadians
  state.steeringAngleRef  = createVariable('w', 1, "steeringangleref");
  //icurvatureref  = createVariable('w',1,"curvatureref");
  // state.steeringChangedTo = createVariable('r', 2, "steeringChangedTo");
  state.steeringAngle = createVariable('r', 1, "steeringAngle");
  // status variables
  state.servoTorque = createVariable('r', 2, "hxtorquemode");
  state.servoLed = createVariable('r', 2, "hxled");
  state.servoVolt = createVariable('r', 2, "hxvoltage");
  state.servoTick = createVariable('r', 2, "hxtick");
  state.servoPosVel = createVariable('r', 2, "hxvelmode");
  state.servoTemp = createVariable('r', 2, "hxtemp");
  state.servoPos = createVariable('r', 2, "hxpos");
  state.servoPosDif = createVariable('r', 2, "hxposdif");
  state.servoPWM = createVariable('r', 2, "hxpwm");
  state.servoStatus = createVariable('r', 2, "hxstatus");
  state.servoError = createVariable('r', 2, "hxError");
  if (state.getRam)
  {
    state.servoFullRAM[0] = createVariable('r', 73, "hxRam0");
    state.servoFullRAM[1] = createVariable('r', 73, "hxRam1");
  }
  
//    /// play time variable
//   state.servoPlayTime90deg = createVariable('w', 1, "hxplaytime");
//   /// accRatio
//   state.servoAccRatio = createVariable('w', 1, "hxaccration");
//   /// maxAcc
//   state.servoMaxAcc = createVariable('w', 1, "hxaccmax");
//   /// minPWM
//   state.servoMinPWM = createVariable('w', 1, "hxpwmmin");
//   /// maxAcc
//   state.servoMaxPWM = createVariable('w', 1, "hxpwmmax");


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
  //
  *p1 = '\0'; // clear result (to improve debug);
  while (!rxtask.shutDown)
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

///////////////////////////////////////////////

void setControlAxis(servo * axis, const char * val, const char * name)
{
  const char * p1;
  int v, i;
  //
  p1 = val;
  // set default values
  axis->name = "(none)";
  axis->vmin = 21;
  axis->vmax = 1002;
  axis->center = 512;
  axis->speed = 60;
  // radians per tick - from servo spec
  axis->scale = 0.325 * M_PI / 180.0;
  axis->degreeRef = 0;
  axis->position = 512;
  axis->positionRef = 512;
  axis->hardError = 0;
  axis->comFailCnt = 0;
  axis->volt = 0;
  for (i = 1; i < 8; i++)
  {
    if (p1 == NULL)
      break;
    v = strtol(p1, (char **) &p1, 0);
    // control axis attribute should be "joy-axis servo-index min max invert"
    switch (i)
    {
      case 1: axis->servo = v; break;
      case 2: axis->vmin = v; break;
      case 3: axis->center = v; break;
      case 4: axis->vmax = v; break;
//      case 6: axis->scale = v; break;
//      case 5: axis->speed = v; break;
      default: break;
    }
  }
  if (name != NULL)
    axis->name = name;
  printf(PLUGINNAME " servo %s: from '%s' set to\n"
         "servo=%d, min=%d, cen=%d, max=%d [t], sc=%g [r/t]\n",
          axis->name, val, axis->servo, axis->vmin,
          axis->center, axis->vmax, axis->scale);
}

///////////////////////////////////

/**
 * Set servo position if changed, and send to servo */
int setServos(float angleLeft, float angleRight)
{
#define ALL_SERVOS 0xfe
  int isOK = 0;
  int r0, r1, p0, p1;
  float ddeg;
  // save angle ref for each servo (debug)
  state.servos[0].degreeRef = angleLeft;
  state.servos[1].degreeRef = angleRight;
  // average movement for movement time
  ddeg = (state.servos[0].degreeRef - angleLeft + 
          state.servos[1].degreeRef - angleRight)/2.0;
  // convert angle to servo units
  r0 = roundi(angleLeft  / state.servos[0].scale + state.servos[0].center);
  r1 = roundi(angleRight / state.servos[1].scale + state.servos[1].center);
  // limit manoeuvre space
  p0 = limitSigned(r0, state.servos[0].vmin, state.servos[0].vmax);
  p1 = limitSigned(r1, state.servos[1].vmin, state.servos[1].vmax);
  // implement
  if ((p0 != state.servos[0].positionRef) ||
      (p1 != state.servos[1].positionRef))
  { // servo position needs to be updated
    uint8_t * s;
    // set playtime
    state.s_jog[0] = limit(10, 255, roundi((state.servoPlayTime90degVal + 30) * fabs(ddeg) / 90.0));
    // servo 0
    s = &state.s_jog[1];
    s[0] = p1 & 0xff;
    s[1] = (p1 >> 8) & 0x3f;
    if (p1 != r1)
      s[2] = 0x0c; // blue and green - outside 
    else
      s[2] = 0x04; // just green
    s[3] = state.servos[0].servo;
    // servo 1
    s[4] = p0 & 0xff;
    s[5] = (p0 >> 8) & 0x3f;
    if (p0 != r0)
      s[6] = 0x0c; // blue and green
    else
      s[6] = 0x04; // just green
    s[7] = state.servos[1].servo;
    // send as S_JOG message
    isOK = sendToHerkulex(ALL_SERVOS, S_JOG, state.s_jog, 2 * 4 + 1, 0);
    if (isOK)
    { // save commanded position
      state.servos[0].positionRef = p0;
      state.servos[1].positionRef = p1;
    } 
  }
  return isOK;
}

////////////////////////////////////////

/**
 * Convert non-linear temperature measurement to approximate temp
 * \param code is code from servo
 * \returns temperature in degrees C */
float codeToTemp(int code)
{
#define TEMPS_CNT 12
  int codes[TEMPS_CNT] =   {    9,    21,    40,   64, 90  , 116  , 141 , 162 ,  181 , 197 , 209 , 223};
  float temps[TEMPS_CNT] = {-41.5,-26.59,-12.98, 0.84, 10.1, 20.39, 30.6, 40.9, 50.09, 60.4, 70.1, 85.0};
  float temp = -45.0;
  int i;
  for (i = TEMPS_CNT - 2; i >= 0; i--)
  {
    if (code > codes[i])
    {
      float dt = (temps[i + 1] - temps[i])/((float)(codes[i + 1] - codes[i]));
      temp = temps[i] + (code - codes[i]) * dt;
      break;
    }
  }
  return temp;
}

///////////////////////////////////////

void printErrorToConsole(int servo, int ss)
{
      if ((ss & 0x100) != 0)
        printf(PLUGINNAME ": --- servo %d input voltage limit exceeded - is %.1f V\n", 
               servo, state.servos[servo].volt);
      if ((ss & 0x400) != 0)
        printf(PLUGINNAME ": --- servo %d temperature limit exceeded - is %.1f C\n",
               servo, state.servos[servo].temp);
      if ((ss & 0x800) != 0)
      {
        printf(PLUGINNAME ": --- servo %d invalid package received by servo\n", servo);
        if ((ss & 0x4) != 0)
          printf(PLUGINNAME ": --- servo %d checksum error\n", servo);
        if ((ss & 0x8) != 0)
          printf(PLUGINNAME ": --- servo %d unknown command\n", servo);
        if ((ss & 010) != 0)
          printf(PLUGINNAME ": --- servo %d exceed register range\n", servo);
        if ((ss & 0x20) != 0)
          printf(PLUGINNAME ": --- servo %d other garbage detected\n", servo);
      }
      if ((ss & 0x1000) != 0)
        printf(PLUGINNAME ": --- servo %d overload\n", servo);
      if ((ss & 0x2000) != 0)
        printf(PLUGINNAME ": --- servo %d driver fault\n", servo);
      if ((ss & 0x4000) != 0)
        printf(PLUGINNAME ": --- servo %d EEP register distorted\n", servo);
      if ((ss & 0x1) != 0)
        printf(PLUGINNAME ": --- servo %d is moving\n", servo);
      if ((ss & 0x2) != 0)
        printf(PLUGINNAME ": --- servo %d is in position (not an error)\n", servo);
      if ((ss & 0x40) != 0)
        printf(PLUGINNAME ": --- servo %d motor on\n", servo);
}