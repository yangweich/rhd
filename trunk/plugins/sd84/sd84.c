 /** \file sd84.c
 *  \ingroup hwmodule
 *
 *   Interface for sd84 board - 84 in/out with mixed analog, digital and servo control
 *
 *
 *******************************************************************/
/***************************** Plug-in version  *****************************/
#define SD84_VERSION    "0.1501"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2011-09-18 13:43:12 +0200 (Sun, 18 Sep 2011) $:"
 #define ID               "$Id: sd84.c 59 2012-10-21 06:25:02Z jcan $"
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

#include "sd84.h"

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
void createSd84variables();
int initSd84(void);
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
void * sd84_task(void *);
/** get variable value from 2 32 bit integer elements 
 * \param type is either 'r' or 'w'.
 * \param index is index to variable.
 * \returns value that is first element signed + decimal part (signed) times 1e-6 */
float getVariableFloat(char type, int index);
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
/// absolute value of an integer
int absi(int a)
{
  if (a >= 0)
    return a;
  else
    return -a;
}
/** catch writes to a non existing file - allowed in debug mode
 * \param buf is the byte buffer with data to send
 * \param bufCnt is the number of bytes to send
 * \returns the number of bytes send, 0 if failed to send. */
ssize_t secure2Write(const void *buf, ssize_t txLen);
/// set joystick button initial value
void initButton(button * but, const char * val, const char * name);
/// got new joystick value for button
int checkButton(button * but, int val);
/// update servo with new joystick position
void checkAxis(servo * s, int val);
/// set control axis from ini-string
void setControlAxis(servo * axis, const char * val, const char * name);

/******** Global variables *************/

/// variables related to thread execution
struct
{ /** is receive thread running */
  int running;
  /** Trigger polling of data from units - when set to 1 */
  int startNewRxCycle;
  /** name of serial device */
  /** thread handle and attributes */
  pthread_t sd84_thread;
} rxtask;

int tick = 0;
/// old steering angle value - in radians
double sacOld = -4.0;
int debugFlag = 0;
/** flags for updating of write variables */
int settingUpd = 1, steerUpd = 1;
struct timeval tickTime;
/** sets 24 channels as output all low (21), 22 is high) */
unsigned char setModeOut24[30] ={ 0xaa, 0xa0, 0x55, 0x04, 37, 24,
                                21, 21, 21, 21, 21, 21, 21, 21,
                                21, 21, 21, 21, 21, 21, 21, 21,
                                21, 21, 21, 21, 21, 21, 21, 21 };

/////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////

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

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int rhdTick)
{
  int returnValue = rxtask.running;
  //gettimeofday(&tickTime, NULL);
  rxtask.startNewRxCycle += 1;
  tick = rhdTick;
  if (tick == 1)
  { // connect to joystick - if any
    sd84.joyaxes = getDatabaseVariable('r', "joyaxes");
    sd84.joybuttons = getDatabaseVariable('r', "joybuttons");
    //
    printf("SD84 joystick database index axes=%d, buttons=%d (-1=none)\n", sd84.joyaxes, sd84.joybuttons);
    //
  }
  if (sd84.joybuttons >= 0)
  { // use remote control
    // axis 2 i steering min is -32000 max is +32000
    int jval = getReadVariable(sd84.joyaxes, steer.servos[0].axis);
    float ang = (float)-jval / 32767.0 * M_PI/2.0; // in radians
    int v;
    int joySteer;
    // is right button pressed
    int override = getReadVariable(sd84.joybuttons, sd84.joyOverrideBut);
    if (override)
    { // remove arm control, when we want to control steering
      if (getReadVariable(sd84.varJoyArm, 0))
      { // tell rhd and on-button that it is now off
        setVariable(sd84.varJoyArm, 0, 0);
        arm.butOn.val = 0;
      }
    }
    joySteer = (!isMasterAlive && !getReadVariable(sd84.varJoyArm, 0)) || override;
    if (joySteer != getReadVariable(sd84.varJoySteer, 0))
    { // tell rhd  that we are now on joystick steering
      setVariable(sd84.varJoySteer, 0, joySteer);
      if (debugFlag)
        printf("setting joystick steering to %d\n", joySteer);
    }
    if (joySteer)
    { // set steeringangle from remote control
      // if (jval != 0 && debugFlag)
      //   printf("SD84 steer %d => %g radians\n", jval, ang);
//      setVariableDouble('w', sd84.steeringangleref, ang );
      // steering angle is in degrees times 10
      writeValue(steer.steeringangleref, 0, roundi(ang*1800.0/M_PI) );
      steerUpd = 1;
      // debug
      if (0) //tick % 10 == 0)
        printf("SD84: steering angle %g is %d==%d deci degrees\n", ang, roundi(ang*1800.0/M_PI), getWriteVariable(steer.steeringangleref, 0));
      // debug end
    }
    v = checkButton(&arm.butOn, getReadVariable(sd84.joybuttons, arm.butOn.butIdx));
    if (v && !arm.butOn.val)
    { // arm is off but button is pressed, get initial values of power and laser
      arm.butPower.val = getReadVariable(sd84.joybuttons, arm.butPower.outIdx);
      arm.butLaser.val = getReadVariable(sd84.joybuttons, arm.butLaser.outIdx);
    }
    if (arm.butOn.val)
    { // arm control is on
      int i;
      // check other buttons
      checkButton(&arm.butPower, getReadVariable(sd84.joybuttons, arm.butPower.butIdx));
      checkButton(&arm.butLaser, getReadVariable(sd84.joybuttons, arm.butLaser.butIdx));
      for (i = 0; i < ARM_AXIS_CNT; i++)
      {
        checkAxis(&arm.servos[i], getReadVariable(sd84.joyaxes, arm.servos[i].axis));
      }
    }
    else
    {
      int i;
      for (i = 0; i < ARM_AXIS_CNT; i++)
      {
        arm.servos[i].value = getWriteVariable(arm.varAxis, i);
      }      
    }
  }
  // update needs to be checked in every cycle ??? why
  if (isUpdated('w', sd84.varOut) ||
      isUpdated('w', arm.varAxis))
  {
    settingUpd = 1;
    //printf("sd84: settings updated - out or servo\n");
  }
  if (isUpdated('w', steer.steeringangleref))
  {
    steerUpd = 1;
    settingUpd = 1;
    //printf("sd84: settings updated - steering\n");
  }
  // let data capture cycle start
  pthread_mutex_unlock(&sd84.mLock);
  //
  return returnValue;
}

//////////////////////////////////////////////////////

int terminate(void)
{
  rxtask.running = 0;
  printf("stopping SD84 ... ");
  fflush(stdout);
  pthread_join(rxtask.sd84_thread, NULL);
  printf("[OK]\n");
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
  int i, len;
  FILE *fp;
  //Print initialization message
  //Find revision number from SVN Revision
  char * p1;
  char versionString[20] = REVISION;
  char tempString[10];
  p1 = strrchr(versionString, '$');
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  printf("SD84: Initializing plug-in version %s.%s\n",SD84_VERSION, tempString);
  printf("SD84: waiting for udev to detect device .");
  for (i = 0; i < 3; i++)
  {
    fflush(NULL);
    sleep(1);
    printf(".");
  }
  printf(" [OK]\n");
  for (i = 0; i < 24; i++)
    sd84.servoSpeed[i] = 0;
  setControlAxis(&arm.servos[0], "0 12 -830 0 900 0  6", "turn"); // turn shoulder base +=left
  setControlAxis(&arm.servos[1], "1 13 -750 0 750 0  3", "lift"); // lift shoulder +=lift
  setControlAxis(&arm.servos[2], "2 18 -800 0 900 1  3", "elbow"); // elbow +=up
  setControlAxis(&arm.servos[3], "3 16 -850 0 850 0 20", "hand-rot"); // hand rotate (+=left)
  setControlAxis(&arm.servos[4], "4 19 -850 0 900 0  8", "hand"); // hand (+=up)
  setControlAxis(&arm.servos[5], "5 17 -340 0 380 0 16", "gripper"); // gripper (+=close)
  initButton(&arm.butOn, "0 -1", "arm-takeCtrl");
  initButton(&arm.butPower, "1 11", "arm-power");
  initButton(&arm.butLaser, "2 8", "arm-laser");
  // steering
  setControlAxis(&steer.servos[0], "2 6 -950 0 950 0", "frontleft");
  setControlAxis(&steer.servos[0], "2 7 -950 0 950 0", "frontright");
  /* Initialize Expat parser*/
  XML_Parser parser = XML_ParserCreate(NULL);
  result = parser != 0;
  if (!result)
    fprintf(stderr, "SD84: Couldn't allocate memory for XML parser\n");
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
      printf("SD84: Error reading: %s\n",filename);
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
        fprintf(stderr, "   SD84: Couldn't allocate memory for XML File buffer\n");
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
      fprintf(stderr, "SD84: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
  }
  if (parser != NULL)
    XML_ParserFree(parser);
  if (xmlBuf != NULL)
    free(xmlBuf);
  if (result && xmlParse.enable)
  { // all is fine - start plugin
    result = initSd84();
  }
  // this is variable in another plugin,
  // and can not be set just now.
  sd84.joybuttons = -1;
  sd84.joyaxes = -1;
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
              steer.frontBase = strtod(attr[i+1], NULL);
            else if (strcasecmp("steerbase",attr[i]) == 0)
              steer.steerBase = strtod(attr[i+1], NULL);
          }
        }
        else
          // skip this group
          info->skip = info->depth;
        break;
      case 3:
        // this one handles sd84 only
        if (strcmp("sd84",el) == 0)
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
            {
              char * s = busif.serialDev;
              strncpy(busif.serialDev, val, MxDL);
              printf("%s\n", s);
            }
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
              if (debugFlag)
                printf("   SD84 started in DEBUG mode!\n");
            }
            else if (strcmp("servospeed", att) == 0)
            {
              printf("sd84 servo speed %s\n", val);
              if (strlen(val) > 0)
              {
                const int MVL = 200;
                char v[MVL];
                char * p1 = v;
                int i = 0;
                strncpy(v, val, MVL);
                while (p1 != NULL)
                {
                  sd84.servoSpeed[i] = strtol(p1, &p1, 0);
                  if (debugFlag)
                    printf("sd84 servo speed index %d = %d\n", i, sd84.servoSpeed[i]);
                  if (i >= 24)
                    break;
                  i++;
                }
              }
            }
            // joystick override button number
            else if (strcmp("jssOverrideBut", attr[i]) == 0)
              sd84.joyOverrideBut = strtol(attr[i+1], NULL, 0);
            else if (strcmp("jssFrontLeft", attr[i]) == 0)
              setControlAxis(&steer.servos[0], attr[i+1], NULL);
            else if (strcmp("jssFrontRight", attr[i]) == 0)
              setControlAxis(&steer.servos[1], attr[i+1], NULL);
            else if (strcmp("steerScale", attr[i]) == 0)
              steer.steerScale = strtod(attr[i+1], NULL);
            // arm control values
            else if (strcmp("jaturn", attr[i]) == 0)
              setControlAxis(&arm.servos[0], attr[i+1], NULL);
            else if (strcmp("jalift", attr[i]) == 0)
              setControlAxis(&arm.servos[1], attr[i+1], NULL);
            else if (strcmp("jaelbow", attr[i]) == 0)
              setControlAxis(&arm.servos[2], attr[i+1], NULL);
            else if (strcmp("jahandrot", attr[i]) == 0)
              setControlAxis(&arm.servos[3], attr[i+1], NULL);
            else if (strcmp("jahand", attr[i]) == 0)
              setControlAxis(&arm.servos[4], attr[i+1], NULL);
            else if (strcmp("jagripper", attr[i]) == 0)
              setControlAxis(&arm.servos[5], attr[i+1], NULL);
            // number of control buttons
            else if (strcmp("jatakectrl", attr[i]) == 0)
              initButton(&arm.butOn, attr[i+1], NULL);
            else if (strcmp("japower", attr[i]) == 0)
              initButton(&arm.butPower, attr[i+1], NULL);
            else if (strcmp("jalaser", attr[i]) == 0)
              initButton(&arm.butLaser, attr[i+1], NULL);
          }
          if (!info->enable)
            printf("   SD84: Use is disabled in configuration\n");
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
  int n = 1;
  int i;
  //
  busif.ttyDev = open(busif.serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = busif.ttyDev != -1;
  if (result == 0)
    fprintf(stderr,"   usbiss: Can't open device: %s\n", busif.serialDev);
  else
  { // set baudrate
    result = (set_serial(busif.ttyDev, 115200) != -1);
    if (result == 0)
      fprintf(stderr,"   usbiss: Can't set first serial port parameters\n");
  }
  if (debugFlag)
  { // allow no device in debug mode
    result = 1;
  }
  busif.lostConnection = ! result;
  if (result)
  { /** set number of analog channels - here 15 */
    unsigned char setAdCnt[7] ={ 0xaa, 0xa0, 0x55, 0x06, 0x00, 0x01, 8};
    // empty data from device, it is always is in poll mode
    while (n > 0)
      n = getDataToTimeout(busif.rxBuf, MxBL, 100);
    // set modes
    printf("sd84: setting AD count to %d (ad1..%d) ...", AD_COUNT, AD_COUNT);
    setAdCnt[6] = AD_COUNT;
    for (i = 0; i < 5; i++)
    {
      secure2Write(setAdCnt, sizeof(setAdCnt));
      n = getDataToTimeout(busif.rxBuf, MxBL, 200);
      result = (n > 0 && busif.rxBuf[0] <= 1);
      if (result)
        break;
      printf("[fail]");
    }
    if (result)
      printf("[OK]\n");
    else
      printf("\n");
  }
  if (result)
  { /** set channel 1..36 to ind, except the A/D channels */
    unsigned char setModeIn36[42] ={ 0xaa, 0xa0, 0x55, 0x04, 1, 36,
                                  23, 23, 23, 23, 23, 23, 23, 23,
                                  23, 23, 23, 23, 23, 23, 23, 23,
                                  23, 24, 23, 24, 24, 24, 23, 23,
                                  23, 24, 24, 24, 24, 23, 23, 23, 23, 23, 23, 23};
    // digital input pins - port 1..36 - except AD pins - see above
    printf("sd84: set input mode channel 1..36 (except AD) ...");
    for (i = 0; i < 5; i++)
    { // try up to 5 times to set input channels
      secure2Write(setModeIn36, sizeof(setModeIn36));
      n = getDataToTimeout(busif.rxBuf, MxBL, 150);
      result = n > 0 && busif.rxBuf[0] <= 1;
      if (result)
        break;
      printf("[fail]");
    }
    if (result)
      printf("[OK]\n");
    else
      printf("\n");
  }
  if (result)
  {  /** sets 24 channels as output channel 37..60 8 low (21), 8 high (22), 8 low (21) */
    printf("sd84: set output channel 37..60 ...");
    for (i = 0; i < 5; i++)
    { // try up to 5 times to set input channels
      secure2Write(setModeOut24, sizeof(setModeOut24));
      n = getDataToTimeout(busif.rxBuf, MxBL, 150);
      result = n > 0 && busif.rxBuf[0] <= 1;
      if (result)
        break;
      printf("[fail]");
    }
    if (result)
      printf("[OK]\n");
    else
      printf("\n");
  }
  if (result)
  { // set servo channls from 61 to and including 84 to mode 25 (servo)
    unsigned char setModeServo24[30] ={ 0xaa, 0xa0, 0x55, 0x04, 61, 24,
                                  25, 25, 25, 25, 25, 25, 25, 25,
                                  25, 25, 25, 25, 25, 25, 25, 25,
                                  25, 25, 25, 25, 25, 25, 25, 25 };

    // servo output - port 61..84
    printf("sd84: set servo mode channel 61..84 ... ");
    for (i = 0; i < 5; i++)
    {
      secure2Write(setModeServo24, sizeof(setModeServo24));
      n = getDataToTimeout(busif.rxBuf, MxBL, 150);
      result = n > 0 && busif.rxBuf[0] <= 1;
      if (result)
        break;
      printf("[fail]");
    }
    if (result)
      printf("[OK]\n");
    else
      printf("\n");
  }
  if (result)
  { // set servo speed
    unsigned char setServoSpeed24[30] ={ 0xaa, 0xa0, 0x55, 0x03, 61, 24,
                                  0, 0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0, 0};
    // servo output - port 61..84
    printf("sd84: set servo speed 61..84 ... ");
    for (i = 0; i < 24; i++)
      setServoSpeed24[i + 6] = sd84.servoSpeed[i];
    for (i = 0; i < 5; i++)
    {
      secure2Write(setServoSpeed24, sizeof(setServoSpeed24));
      n = getDataToTimeout(busif.rxBuf, MxBL, 150);
      result = n > 0 && busif.rxBuf[0] <= 1;
      if (result)
        break;
      printf("[fail]");
    }
    if (result)
      printf("[OK]\n");
    else
      printf("\n");
  }
  if (result)
  { /** get version number for cpu (2) */
    unsigned char getVersion[6] ={ 0xaa, 0xa0, 0x55, 0x0a, 0x02, 0x00};
    int cpu;
    for (cpu = 1; cpu <= 4; cpu++)
    {
      printf("sd84: getVersion for cpu %d ", cpu);
      getVersion[4] = cpu;
      for (i = 0; i < 5; i++)
      {
        secure2Write(getVersion, sizeof(getVersion));
        n = getDataToTimeout(busif.rxBuf, MxBL, 150);
        result = n >= 2;
        if (result)
          break;
        printf("[fail]");
      }
      if (result)
      {
        busif.cpuVersion[cpu - 1] = busif.rxBuf[1];
        printf(" %d [OK]\n", busif.rxBuf[1]);
      }
      else
        printf("\n");
    }
  }
  return result || debugFlag;
}


//////////////////////////////////////////////

/**
 * Initialize the communication and start rx thread
 * \returns 1 on success (else 0) */
int initSd84(void)
{ //Open first serial port
  int result;
  //
  rxtask.running = 0;
  rxtask.startNewRxCycle = 0;
  pthread_mutex_init(&sd84.mLock, NULL);
  pthread_mutex_lock(&sd84.mLock);
  // open device
  result = openAndSetMode();
  if (result == 1)
  { // start thread to handle bus
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rxtask.sd84_thread, &attr, sd84_task, 0))
    {
      perror("   usbiss: Can't start sd84 receive thread");
      result = 0;
    }
  }
  //if (result == 1 || debugFlag)
  { /****** Create database variables if all is ok **************/
    int waitCount = 0;
    // create RHD database variables
    createSd84variables();
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

///RS232  control thread
void * sd84_task(void * not_used)
{ // run in thread
  // format 0xaa 0xa0 0x55 byte sync sequence
  //        0x01 is command
  //          48 number of bytes to follow
  //        .... value for command
  unsigned char setServo24Pos[54] ={ 0xaa, 0xa0, 0x55, 0x01, 61, 48,
                                   0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 
                                   0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05,
                                   0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05,
                                   0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05,
                                   0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05,
                                   0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05  };
  /** get one servo value - valid are from 61..84 */
  //unsigned char getServo[6]={ 0xaa, 0xa0, 0x55, 0x02, 61, 0x00};
  /** get number of analog channels */
  //unsigned char getAdCnt[6] ={ 0xaa, 0xa0, 0x55, 0x07, 0x00, 0x00};
  /** get input */
  //unsigned char getInput[6] ={ 0xaa, 0xa0, 0x55, 0x08, 0x01, 0x00};
  /** get input CPU 1 (bulk) */
  unsigned char getInputCpu1[6] ={ 0xaa, 0xa0, 0x55, 0x15, 1, 0x00};
  /** get input CPU 2 (bulk) */
  unsigned char getInputCpu2[6] ={ 0xaa, 0xa0, 0x55, 0x15, 2, 0x00};
  /** get analog input input */
  unsigned char getAd[6] ={ 0xaa, 0xa0, 0x55, 0x09, 0x01, 0x00};
  // channel number for each ad converter channel
  // index 0 is not used
  int getAdCh[14] ={ -1, 21, 22, 20, 18, 28, 27, 29, 26, 30, 11, 10, 9, 7};
  //
  const int MaxWaitCycles = 3; // max number of timeout periods
  const int PollTimeoutMs = 3; // timeout period
  //const char * active;
  int state = 0;
  int adCh = 1;
  int i;
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
    fprintf(stderr, "   SD84: signal: can't ignore SIGPIPE.\n");

  rxtask.running = 1;
  fprintf(stderr, "   SD84: rx_task running\n");
  // wait for RHD to init variables
  usleep(300000);
  // set initial values of variables
  for (i = 0; i < 4; i++)
    setVariable(sd84.varCpuVer, i, busif.cpuVersion[i]);
  for (i = 0; i < 14; i++)
    setVariable(sd84.varAD, i, -1);
  for (i = 0; i < 36; i++)
    setVariable(sd84.varIn, i, -1);

    //Mark thread as running
  while (rxtask.running)
  { // maintain interface
    int i, v, n;
    int isOK = 1;
    //
    if (busif.lostConnection)
    { // connection is lost - during read or write
      if (busif.ttyDev >= 0)
      {
        close(busif.ttyDev);
        busif.ttyDev = -1;
        printf("**** sd84: lost connection - trying to reconnect\n");
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
    //
    switch (state)
    { // send new settings and request for data
      case 1:
        // wait for next rhd time tick
        pthread_mutex_lock(&sd84.mLock);
        gettimeofday(&tickTime, NULL);
        //printf("tick unlocked new=%d\n", rxtask.startNewRxCycle);
        rxtask.startNewRxCycle = 0;
        break;
      case 2: // 24 outout pins - port 37..60
        //active = "setModeOut24";
        isOK = 1;
        for (i = 0; i < 24; i++)
        { // is output to be high or low
          v = getWriteVariable(sd84.varOut, i) == 0;
          if (v != (setModeOut24[6 + i] & 0x01))
          {
            isOK = 0;
            if (v)
              setModeOut24[6 + i] = 21; // low
            else
              setModeOut24[6 + i] = 22; // high
          }
        }
        if ( ! isOK)
        { // data has changed set all 24 output values
          v = getTimePassed(tickTime);
          secure2Write(setModeOut24, sizeof(setModeOut24));
          n = getData(1, PollTimeoutMs, MaxWaitCycles);
          isOK = n >=1;
          if (isOK)
          { // make a copy in read mode
            for (i = 0; i < 24; i++)
              setVariable(sd84.varOutr, i, setModeOut24[i] & 0x01);
          }
          if (!isOK || busif.rxBuf[0] != 0x00)
            printf("SD84: %lu.%06lu (%dus) set out 37-60 failed - returned %d bytes rxBuf[0]=%2x (0 or 1 is OK)\n",
                  tickTime.tv_sec, tickTime.tv_usec, getTimePassed(tickTime), n, busif.rxBuf[0]);
        }
        break;
      case 3: // set servo positions
        //active = "setSteering";
        // double sac = getVariableDouble('w', sd84.steeringangleref);
        // get steering angle in radians
        steer.steerRef = getWriteVariable(steer.steeringangleref, 0) * M_PI / 1800.0;
        if (0) // tick %10 == 0)
          printf("SD84 set servo, ref %g radians\n", steer.steerRef);
        if (steerUpd && fabs(steer.steerRef - sacOld) > 1e-6)
        { // steering changed - calculate new angle for left and right wheel
          int vl, vr;
          float tanSac, ofs, sal, sar;
          // reset update flag
          steerUpd = 0;
          if (fabs(steer.steerRef) < (M_PI/2.0 - 1e-5))
            tanSac = tan(steer.steerRef);
          else // use +-90 degrees
            tanSac = steer.steerRef * 1e10;
          ofs = steer.frontBase * tanSac / 2.0 / steer.steerBase;
          sal = atan2(tanSac, 1.0 - ofs);
          sar = atan2(tanSac, 1.0 + ofs);
          // reset update flag
          // servo values for steering has changed
          vl = roundi(sal * steer.steerScale);
          vr = roundi(sar * steer.steerScale);
          // inform motor controller
          setVariableDouble('r', steer.steeringChangedTo, steer.steerRef);
          // put values into list of servo positions
/*          writeValue(sd84.varServo, steer.servos[0].servo, vl); // left is servo 6 or port number 61+6 = 67
          writeValue(sd84.varServo, steer.servos[1].servo, vr); // right is servo 7 or port number 61+7 = 68*/
          steer.servos[0].value = vl; // left servo
          steer.servos[1].value = vr; // right servo
          // debug
          //if (debugFlag)
          //  printf("sd84: steering setting heading=%.4f rad"
          //      " (old=%.4f), scale=%g, left=%d, right=%d\n",
          //      steer.steerRef, sacOld, steer.steerScale, vl, vr);
          // debug end
          sacOld = steer.steerRef;
          /// actual steering angle should be calculated taking
          /// the servo speed into consideration
          /// - but set to reference value for now
          setVariable(steer.steeringAngle, 0, roundi(steer.steerRef * 1800.0 / M_PI));
        }
        // set servos
        //active = "setServo24Pos";
        isOK = tick > 5;
        for (i = 0; i < 6; i++)
        { // get requested servo positions
          // get index to servo control
          int sx = arm.servos[i].servo;
          // get axis value
          //v = getWriteVariable(arm.varAxis, i);
          v = arm.servos[i].value;
          if (v != sd84.servoOld[sx] || tick < 5)
          { // write servo the first 5 times and on change
            isOK = 0;
            sd84.servoOld[sx] = v;
            arm.servos[i].value = v;
            v = 1300 + limitSignedInt(v, -920, 920);
            setServo24Pos[6 + sx * 2] = v & 0xff;
            setServo24Pos[7 + sx * 2] = v >> 8;
          }
        }
        for (i = 0; i < 2; i++)
        { // steering servos
          // get index to servo control
          int sx = steer.servos[i].servo;
          // get axis value
          v = steer.servos[i].value + steer.servos[i].center;
          //if (debugFlag)
          //  printf("servo raw=%d offset=%d v=%d\n", 
          //         steer.servos[i].value, steer.servos[i].center, v);
          if (v != sd84.servoOld[sx] || tick < 5)
          { // write servo the first 5 times and on change
            isOK = 0;
            sd84.servoOld[sx] = v;
            v = 1300 + limitSignedInt(v, -920, 920);
            setServo24Pos[6 + sx * 2] = v & 0xff;
            setServo24Pos[7 + sx * 2] = v >> 8;
          }
        }
        if ( ! isOK)
        { // all servos are not as they should, so set new values
          v = getTimePassed(tickTime);
          secure2Write(setServo24Pos, sizeof(setServo24Pos));
          // wait for reply
          n = getData(1, PollTimeoutMs, MaxWaitCycles);
          isOK = n >=1;
          if (isOK)
          { // tell new state as rhd read variable
            for (i = 0; i < 6; i++)
              setVariable(arm.varAxisr, i, arm.servos[i].value);
            for (i = 0; i < 6; i++)
              setVariable(steer.varSteerr, i, steer.servos[i].value);
          }
          if (!isOK || busif.rxBuf[0] != 0x00)
            printf("SD84: %lu.%06lu (%dus) set servo failed - returned %d bytes: rxBuf[0]=%2x (0 or 1 is OK)\n",
                  tickTime.tv_sec, tickTime.tv_usec, getTimePassed(tickTime), n, busif.rxBuf[0]);
        }
        break;
      case 4: /* send request for digital input   */
        //if (tick % 2 == 0)
        //  break;
        //active = "getInputcpu1";
        v = getTimePassed(tickTime);
        secure2Write(getInputCpu1, sizeof(getInputCpu1));
        n = getData(3, PollTimeoutMs, MaxWaitCycles);
        isOK = n >=3;
        if (isOK)
        { // set variables
          setVariable(sd84.varIn, 17, (busif.rxBuf[0] & 0x80) > 0); //
          setVariable(sd84.varIn, 16, (busif.rxBuf[0] & 0x40) > 0); //
          setVariable(sd84.varIn, 19, (busif.rxBuf[0] & 0x10) > 0); //
          setVariable(sd84.varIn, 23, (busif.rxBuf[1] & 0x80) > 0); //
          setVariable(sd84.varIn, 24, (busif.rxBuf[1] & 0x40) > 0); //
          setVariable(sd84.varIn, 25, (busif.rxBuf[1] & 0x20) > 0); //
          setVariable(sd84.varIn, 31, (busif.rxBuf[2] & 0x20) > 0); //
          setVariable(sd84.varIn, 32, (busif.rxBuf[2] & 0x10) > 0); //
          setVariable(sd84.varIn, 12, (busif.rxBuf[2] & 0x08) > 0); //
          setVariable(sd84.varIn, 13, (busif.rxBuf[2] & 0x04) > 0); //
          setVariable(sd84.varIn, 14, (busif.rxBuf[2] & 0x02) > 0); //
          setVariable(sd84.varIn, 15, (busif.rxBuf[2] & 0x01) > 0); //
        }
        else if (debugFlag)
          printf("SD84: %lu.%06lu (%dus) input cpu 1 - returned %d bytes: 0x%02x 0x%02x\n",
            tickTime.tv_sec, tickTime.tv_usec, getTimePassed(tickTime),
             n, busif.rxBuf[0], busif.rxBuf[1]);
        setVariable(sd84.varIn, 0, tick); //
        break;
      case 5:
        if (tick % 2 == 1)
          break;
        //active = "getInputcpu2";
        v = getTimePassed(tickTime);
        secure2Write(getInputCpu2, sizeof(getInputCpu2));
        n = getData(3, PollTimeoutMs, MaxWaitCycles);
        isOK = n >=3;
        if (isOK)
        { // set variables
          setVariable(sd84.varIn,  6, (busif.rxBuf[0] & 0x80) > 0); //
          setVariable(sd84.varIn,  5, (busif.rxBuf[0] & 0x40) > 0); //
          setVariable(sd84.varIn,  8, (busif.rxBuf[0] & 0x10) > 0); //
          setVariable(sd84.varIn, 33, (busif.rxBuf[1] & 0x80) > 0); //
          setVariable(sd84.varIn, 34, (busif.rxBuf[1] & 0x40) > 0); //
          setVariable(sd84.varIn, 35, (busif.rxBuf[1] & 0x20) > 0); //
          setVariable(sd84.varIn, 36, (busif.rxBuf[1] & 0x10) > 0); //
          setVariable(sd84.varIn, 1, (busif.rxBuf[2] & 0x08) > 0); //
          setVariable(sd84.varIn, 2, (busif.rxBuf[2] & 0x04) > 0); //
          setVariable(sd84.varIn, 3, (busif.rxBuf[2] & 0x02) > 0); //
          setVariable(sd84.varIn, 4, (busif.rxBuf[2] & 0x01) > 0); //
        }
        else if (debugFlag)
          printf("SD84: %lu.%06lu (%dus) input cpu 2 - returned %d bytes: 0x%02x 0x%02x\n",
            tickTime.tv_sec, tickTime.tv_usec, getTimePassed(tickTime),
             n, busif.rxBuf[0], busif.rxBuf[1]);
        setVariable(sd84.varIn, 0, tick); //
        break;
      case 6:
      case 7: /* send request for AD data */
      //case 8:
        getAd[4] = getAdCh[adCh];
        secure2Write(getAd, sizeof(getAd));
        v = getTimePassed(tickTime);
        n = getData(2, PollTimeoutMs * 3, MaxWaitCycles);
        isOK = n >= 2;
        if (isOK)
        { // save AD value
          v = busif.rxBuf[0] + (busif.rxBuf[1] << 8);
          setVariable(sd84.varAD, adCh, v);
        }
        else if (debugFlag)
        {
          unsigned int u = getTimePassed(tickTime);
          printf("SD84: %lu.%06lu (%.1fms) get AD %d "
                "failed - returned %d bytes: 0x%02x 0x%02x\n",
                tickTime.tv_sec, tickTime.tv_usec, (u - v)/1000.0,
                adCh - 1, n, busif.rxBuf[0], busif.rxBuf[1]);
        }
        setVariable(sd84.varAD, 0, adCh + 100);
        if (adCh >= AD_COUNT)
          adCh = 1;
        else
          adCh++;
        break;
      default:
        {
          unsigned int cycleTime;
          cycleTime = getTimePassed(tickTime);
          if (cycleTime > 3000000)
            printf("sd84: tick %d cycle time %.1fms\n", tick, cycleTime/1000.0);
          state = 0;
        }
        break;
    }
    state++;
  }
  rxtask.running = 0;
  fprintf(stderr,"SD84: closing bus device\n");
  close(busif.ttyDev);
  fprintf(stderr,"SD84: Shutting down thread\n");
  pthread_exit(0);
  return NULL;
}

///////////////////////////////////////////////////////////

/**
 * Create variables for the usb to sd84 converter itself */
void createSd84variables()
{ // create variables in RHD
  // version of cpu-firmware
  sd84.varCpuVer = createVariable('r', 4, "sd84cpu");
  // is joystick controlling steering
  sd84.varJoySteer = createVariable('r', 1, "joySteer");
  // is joystick controlling arm
  sd84.varJoyArm = createVariable('r', 1, "joyArm");
  // AD values
  sd84.varAD = createVariable('r', 14, "sd84ad");
  // IN values
  sd84.varIn = createVariable('r', 37, "sd84in");
  // out values
  sd84.varOut = createVariable('w', 24, "sd84out");
  // out values
  sd84.varOutr = createVariable('r', 24, "sd84outr");
  // servo values
  //sd84.varServo = createVariable('w', 24, "sd84servo");
  // servo values
  //sd84.varServor = createVariable('r', 24, "sd84servor");
  // control of arm axis
  arm.varAxis = createVariable('w', 6, "armAxis");
  // current position of servos for arm axis
  arm.varAxisr = createVariable('r', 6, "armAxisr");
  // current position of servos for arm axis
  steer.varSteerr = createVariable('r', 2, "steerr");
  // steeringangle in radians and uRadians
  steer.steeringangleref  = createVariable('w', 1, "steeringangleref");
  //icurvatureref  = createVariable('w',1,"curvatureref");
  steer.steeringChangedTo = createVariable('r', 2, "steeringChangedTo");
  steer.steeringAngle = createVariable('r', 1, "steeringAngle");
  //
}

////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////

float getVariableFloat(char type, int index)
{ // get type double value from 2 integer values - assuming integer part and micro decimal value.
  int v, uv;
  float result;
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
        printf("sd84:: Read error from device - connection lost\n");
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
        printf("sd84:: Read error from device - device is lost\n");
        busif.lostConnection = 1;
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

////////////////////////////////////////////////

int checkButton(button * but, int val)
{
  if (val != but->lastVal)
  {
    if (val == 1)
      gettimeofday(&but->onTime, NULL);
    else if (getTimePassed(but->onTime) > 1)
    { // time to switch
      but->val = ! but->val;
      if (debugFlag)
      {
        if (but->val)
          printf("Turned on %s\n", but->name);
        else
          printf("Turned off %s\n", but->name);
      }
      if (but->outIdx >= 0)
        writeValue(sd84.varOut, but->outIdx, but->val);
      else if (but->outIdx == -1)
        setVariable(sd84.varJoyArm, 0, but->val);
    }
    but->lastVal = val;
  }
  return val;
}

///////////////////////////////////////////

void initButton(button * but, const char * val, const char * name)
{
  const char * p1;
  p1 = val;
  if (*p1 != '\0')
    but->butIdx = strtol(p1, (char **)&p1, 0);
  if (p1 != NULL)
    but->outIdx = strtol(p1, (char **)&p1, 0);
  but->val = 0;
  but->lastVal = 0;
  if (name != NULL)
    but->name = name;
}

//////////////////////////////////////////////

void checkAxis(servo * ss, int val)
{
  if (absi(val) > 100)
  {
    int vnew, vadd;
    // get current value
    //vold = getWriteVariable(sd84.varServo, ss->servo);
    if (val > 0)
    { // add 1 in addition to scaled proportional value 
      vadd = 1;
      val -= 101;
    }
    else
    { // subtract one in addition to proportional value
      vadd = -1;
      val += 101;
    }
    // calculate new value
    if (ss->scale == 0)
    {
      printf("scale=0! of %s - set to 1\n", ss->name);
      ss->scale=1;
    }
    if (ss->invert)
      vnew = ss->value - val / (INT16_MAX / ss->scale) - vadd;
    else
      vnew = ss->value + val / (INT16_MAX / ss->scale) + vadd;
    // check limits
    if (vnew > ss->vmax)
      vnew = ss->vmax;
    else if (vnew < ss->vmin)
      vnew = ss->vmin;
    // write to variable database
    if (vnew != ss->value)
    {
      ss->value = vnew;
      // writeValue(sd84.varServo, ss->servo, vnew);
      // debug
      //if (debugFlag)
      //  printf("set axis %s (%d->%d) to %d\n", ss->name, ss->axis, ss->servo, vnew);
      // debug
    }
  }
}

//////////////////////////////////////////////////

void setControlAxis(servo * axis, const char * val, const char * name)
{
  const char * p1;
  int v, i;
  //
  p1 = val;
  for (i = 0; i < 8; i++)
  {
    if (p1 == NULL)
      break;
    v = strtol(p1, (char **) &p1, 0);
    // control axis attribute should be "joy-axis servo-index min max invert"
    switch (i)
    {
      case 0: axis->axis = v; break;
      case 1: axis->servo = v; break;
      case 2: axis->vmin = v; break;
      case 3: axis->center = v; break;
      case 4: axis->vmax = v; break;
      case 5: axis->invert = v; break;
      case 6: axis->scale = v; break;
      case 7: axis->speed = v; break;
      default: break;
    }
  }
  if (name != NULL)
    axis->name = name;
  printf("from '%s' set\n"
         "axis=%d, servo=%d, min=%d, cen=%d, max=%d, inv=%d, sc=%d, sp=%d\n",
          val, axis->axis, axis->servo, axis->vmin,
          axis->center, axis->vmax, axis->invert, axis->scale, axis->speed);
}

///////////////////////////////////

