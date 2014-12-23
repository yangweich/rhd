 /** \file usbiss.c
 *  \ingroup hwmodule
 *
 * interface to usb-to-i2c module and a MD25 mototcontroller (2xdc motor control with gearing and encoder)
 *
 *******************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 388 $:"
 #define DATE             "$Date: 2011-09-04 16:07:32 +0200 (Sun, 04 Sep 2011) $:"
 #define ID               "$Id: rpii2c.c 388 2013-12-30 14:30:09Z jcan $"
/***************************************************************************/
#define PLUGINNAME "rpii2c"


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
#include <linux/i2c-dev.h>

//RHD Core headers
#include <rhd.h>
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

// #include "glue/linux_glue.h"
// #include "mpu9150/mpu9150.h"
#include "rpii2c.h"

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
int initDevice(void);
/* * poll to see if a read would not block.
 * \param fd is the device number
 * \param msTimeout is the maximum wait time
 * \returns 0 is no data is available and 1 if at least 1 byte is available */
//int pollDeviceRx(int fd, int msTimeout);
/* *
 * get data from device
 * \param minCnt is the minimum number of bytes to fetch
 * \param timeoutms is the timeout used when waiting for data
 * \param maxWaitCnt is the max number of poll cycles to wait for required amount of data
 * \returns number of data received */
//int getData(int minCnt, int timeoutms, int maxWaitCnt);
/* *
 * Get data from line until timeout.
 * \param dest is where data should be stored
 * \param destCnt is length of destination buffer
 * \param timeout is the max time to wait for data
 * \returns when 2 timeoutperiods has expired with no data or
 * when data is received (within two timeout periods) and there have been no new data within one timeout period or
 * when there is no more space or just one character left in destination buffer. */
// int getDataToTimeout(unsigned char * dest, int destCnt, int timeoutms);
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
int32_t roundi(const float v)
{ // round to closest integer
  if (v > 0)
    return (int32_t)(v + 0.5);
  else
    return (int32_t)(v - 0.5);
}
/**
 * Reset encoders in MD25 to 0 */
void resetEncoders(void);
/**
 * set motor acceleration */
void setAcc(int acc);
/**
 * Read encoder values into RHD variables (32 bit) */
int readEncoderValues (void);
/**
 * drive  motors to a given speed 128 is zero speed 0 is full (forward) and 255 is full (reverse) */
int driveMotors(int left, int right);
/**
 * set both motors to 128 */
void stopMotors(void);
/**
 * get IMU data and set RHD variables  */
void getIMUdata();
/**
 * Set calibration values for acceleration (0) and magnetometer (1) */
int set_cal(int mag, char *cal_file);
/**
 * read from 9150 */
//void read_loop(unsigned int sample_rate);
/** debug */
//void print_fused_euler_angles(mpudata_t *mpu);
/** debug */
//void print_fused_quaternion(mpudata_t *mpu);
/** debug */
//void print_calibrated_accel(mpudata_t *mpu);
/** debug */
//void print_calibrated_mag(mpudata_t *mpu);
/** absolute value of long integers */
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
FILE * logState = NULL;

/** catch writes to a non existing file - allowed in debug mode */
// ssize_t secure2Write(const void *buf, ssize_t txLen)
// {
//   if (busif.ttyDev >= 0)
//     return secureWrite(busif.ttyDev, buf, txLen);
//   else if (debugFlag)
//     // perform as if all is written - debug mode
//     return txLen;
//   else
//     return 0;
// }

/////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int rhdTick)
{
  tick = rhdTick;
  rxtask.startNewRxCycle = 1;
  if ((rhdTick <= 1) && (md25.varSteeringAngleRef < 0))
  {
    md25.varSteeringAngleRef = getDatabaseVariable('w', "steeringangleref");
    printf(PLUGINNAME " got index to steeringangle for MD25, is %d\n", md25.varSteeringAngleRef);
  }
  pthread_mutex_unlock(&md25.mLock);
  return 1;
}

//////////////////////////////////////////////////////

int terminate(void)
{
  rxtask.running = 0;
  printf(PLUGINNAME " stopping ... \n");
  fflush(stdout);
  pthread_join(rxtask.i2c_thread, NULL);
  close(busif.ttyDev);
  printf(PLUGINNAME " stopped\n");
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
  imu.busid = 0x68;
  //Print initialization message
  //Find revision number from SVN Revision
  char * p1;
  char versionString[20] = REVISION;
  char tempString[10];
  p1 = strrchr(versionString, '$');
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  printf(PLUGINNAME " md25: plug-in version %s\n", tempString);
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
        fprintf(stderr, PLUGINNAME " Couldn't allocate memory for XML File buffer\n");
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
    result = initDevice();
  }
  // this is variable in another plugin,
  // and can not be set just now.
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
        if (strcmp(PLUGINNAME,el) == 0)
        { // is it enabled, the only info needed
          for(i = 0; attr[i]; i+=2)
            if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0))
              info->enable = 1;
          if (!info->enable)
            printf(PLUGINNAME " Use is disabled in configuration\n");
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
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
              if (debugFlag)
                printf(PLUGINNAME " started in DEBUG mode!\n");
            }
          }
          info->found = 1;
          //printf("   USBISS: serial device to %s, busID 0x%x\n", busif.serialDev, usbiss.busid);
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
          }
        }
        else if (strcmp("IMU9150",el) == 0)
        {
          const char * att, * val;
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i];
            val = attr[i + 1];
            if (strcmp("id", att) == 0)
              imu.busid = strtol(val, NULL, 0);
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
  const unsigned char md25_version[1] = {13};
  // open device
//  int i;
//  printf(PLUGINNAME " waiting 1s for i2c-bus\n");
//  for (i = 0; i < 10; i++)
//    usleep(100000);
  busif.ttyDev = open(busif.serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = busif.ttyDev >= 0;
  if (!result)
    printf(PLUGINNAME " Failed to open i2c port on '%s' got %d\n", busif.serialDev, busif.ttyDev);
  if (result)
  { // Set the port options and set the address of the device we wish to speak to
    result = ioctl(busif.ttyDev, I2C_SLAVE, md25.busid) >= 0;
    if (!result)
      printf(PLUGINNAME " Unable to get bus access to talk to slave\n");
  }
  //
  if (result)
  {  // This is the register we wish to read software version from
    // Send regiter to read software from from
    result = write(busif.ttyDev, md25_version, 1) == 1;
    if (!result)
      printf(PLUGINNAME " Error writing to i2c slave\n");
    else
    { // Read back data into buffer
      result = read(busif.ttyDev, busif.rxBuf, 1) == 1;
      if (!result)
        printf(PLUGINNAME " Unable to read from slave\n");
      else
        printf(PLUGINNAME " Software version: %u\n", busif.rxBuf[0]);
    }
  }
  //
  if (result)
    resetEncoders();                      // Reset the encoder values to 0
  if (result)
  { // disable motor speed control
    busif.txBuf[0] = 48; // motor speed controller disable register
    busif.txBuf[1] = 1;  // set to disable
    result = ((write(busif.ttyDev, busif.txBuf, 2)) == 2);
  }

  if (debugFlag)
    // allow no device in debug mode
    result = 1;
  busif.lostConnection = ! result;
  md25.accOld = -1;
  //
  // and make IMU active
  if (result)
  { // set device for IPU use
//     i2c_fd = busif.ttyDev;
//     if (mpu9150_init(1 /* not used */, 300 /* sample_rate 4..1000 */, 4 /* yaw_mix_factor 0..10 */))
//       result = 0;
//     
//     set_cal(0, acc);
//     set_cal(1, mag_cal_file);
// 
//     
    result = ioctl(busif.ttyDev, I2C_SLAVE, imu.busid) >= 0;
    if (!result)
      printf(PLUGINNAME " Unable to get bus access to talk to slave\n");
    busif.txBuf[0] = 107;
    busif.txBuf[1] = 0;
    result = write(busif.ttyDev, busif.txBuf, 2) == 2;
    if (! result)
      printf(PLUGINNAME " Error activating (or write to) IMU\n");
  }
  return result || debugFlag;
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
    if (pthread_create(&rxtask.i2c_thread, &attr, i2c_task, 0))
    {
      perror(PLUGINNAME "   usbiss: Can't start i2c receive thread");
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
//   const int MaxWaitCycles = 2; // max number of timeout periods
//   const int PollTimeoutMs = 4; // timeout period
  int state = 0;
  int loopCnt = 0;
  struct timeval tm;
  float spl, spr;
  //
  if (debugFlag)
    logState = fopen("rpii2c.log", "w");
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
    int v;
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
      if (logState != NULL)
      {
        gettimeofday(&tm, NULL);
        fprintf(logState, "%lu.%06lu lost i2c connection\n ", tm.tv_sec, tm.tv_usec);
      }
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
        upd = isUpdated('w', md25.speedref);
      // send new settings and request for data    
      case 2: // speed of motor 1 and 2
        if ( ! upd)
          // no need to update motor speed
          break;
        // if ackermann steering, then steering angle is needed
        if (isUpdated('w', md25.speedref))
        { // convert steering angle to wheel speed, as steering angle or speed is changed
          int speed = getWriteVariable(md25.speedref, 0);
          int sa = 0;
          // get steering angle (for a cycle model steering)
          if (md25.varSteeringAngleRef >= 0)
            sa = getWriteVariable(md25.varSteeringAngleRef, 0);
          float steerAngle = sa / 1800.0 * M_PI;
          spl = 0.0;
          spr = 0.0;
          if (speed != 0)
          { // calculate wheel speed (left and right) at this steering angle and speedref
            if (steerAngle > M_PI / 2.0 - 0.001)
              // close to 90 deg steering angle
              steerAngle = M_PI / 2.0 - 0.001;
            else if (steerAngle < -M_PI / 2.0 + 0.001)
              // close to 90 deg steering angle
              steerAngle = -M_PI / 2.0 + 0.001;
            if (1)
            { // normal steering angle (valid tangent value)
              float ct = fabs(tan(steerAngle) / md25.steerBase);
              float ofs = ct * md25.wheelBase;
              float speedo = speed * (2.0 + ofs) / (2.0 / cos(steerAngle) + md25.frontBase * ct);
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
              if (debugFlag)
                printf(PLUGINNAME " md25 ctrl angle %dddeg %.3frad, speed %dcm/s, left %.1fcm/s, right %.1fcm/s\n", 
                       sa, steerAngle, speed, spl, spr);
            }
          }
        }
        // get speed values
        driveMotors(spl, spr);
        v = getTimePassed(tickTime);
        break;
      case 3: // acceleration
        v = getWriteVariable(md25.varAccW, 0);
        if (v != md25.accOld)
        { // acc value changed
          setAcc(v);
          md25.accOld = v;
          v = getTimePassed(tickTime);
          //printf("Time after set acc %.1fms\n", v / 1000.0);
        }
        break;
      case 4:
        // read encoder, battery and motor current values
        readEncoderValues();
        v = getTimePassed(tickTime);
        break;
      case 5:
        getIMUdata();
        break;
      default:
        state = 0;
        break;
    }
    state++;
  }
  rxtask.running = 0;
  fprintf(stderr,PLUGINNAME " md25: closing bus device\n");
  close(busif.ttyDev);
  fprintf(stderr,PLUGINNAME " md25: Shutting down thread\n");
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
  // md25
  md25.varBatt = createVariable('r',1,"batteryvolt");
  md25.varEnc1 = createVariable('r',1,"encl");
  md25.varEnc2 = createVariable('r',1,"encr");
  md25.varM1Curr = createVariable('r',1,"currentL");
  md25.varM2Curr = createVariable('r',1,"currentR");
//   md25.varM1SpeedW = createVariable('w',1,"speedl");
//   md25.varM2SpeedW = createVariable('w',1,"speedr");
//   md25.varM1SpeedR = createVariable('r',1,"speedLR");
//   md25.varM2SpeedR = createVariable('r',1,"speedRR");
  md25.varReset = createVariable('w',1,"reset");
  md25.varAccR = createVariable('r',1,"accR");
  md25.varAccW = createVariable('w',1,"accW");
  // - if using ackerman steering
  md25.speedref = createVariable('w',1,"speedref");
  md25.varSteeringAngleRef = getDatabaseVariable('w', "steeringangleref");
  //
  //
  /// IMU9150
  imu.varAcc = createVariable('r', 3, "imuAcc");
  imu.varGyro = createVariable('r', 3, "imuGyro");
 // imu.varMag = createVariable('r', 3, "imuMag");
  imu.varTemp = createVariable('r', 1, "imuTemp");
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

////////////////////////////////////////////////////

void resetEncoders(void)
{
  int isOK = ioctl(busif.ttyDev, I2C_SLAVE, md25.busid) >= 0;
  if (! isOK)
  { // Set the port options and set the address of the device we wish to sp$
    printf(PLUGINNAME " Unable to get bus access to talk to md25\n");
  }
  else
  {
    busif.txBuf[0] = 16; // Command register
    busif.txBuf[1] = 32; // command to set decoders back to zero

    if ((write(busif.ttyDev, busif.txBuf, 2)) != 2)
      printf(PLUGINNAME " Error writing to i2c slave (reset encoder)\n");
    else
      printf(PLUGINNAME " encoders reset\n");
  }
}

////////////////////////////////////////////////////

void setAcc(int acc) {
  // set acceleration 1 slowest, 10 fastest
  int isOK = ioctl(busif.ttyDev, I2C_SLAVE, md25.busid) >= 0;
  if (! isOK)
  { // Set the port options and set the address of the device we wish to sp$
    printf(PLUGINNAME " Unable to get bus access to talk to md25\n");
  }
  else
  {
    busif.txBuf[0] = 15; // Command register
    busif.txBuf[1] = limitUnsignedChar(acc, 1, 10);
    if ((write(busif.ttyDev, busif.txBuf, 2)) != 2)
      printf(PLUGINNAME " Error writing to i2c slave (acc)\n");
    if (debugFlag)
      printf(PLUGINNAME " acc set to %d\n", busif.txBuf[1]);
  }
}

///////////////////////////////////////

int readEncoderValues (void)
{
  uint32_t encoder1, encoder2;
  unsigned char * buf = busif.rxBuf;
  int isOK = 0;
  //
  isOK = ioctl(busif.ttyDev, I2C_SLAVE, md25.busid) >= 0;
  if (! isOK)
  { // Set the port options and set the address of the device we wish to sp$
    printf(PLUGINNAME " Unable to get bus access to talk to md25\n");
  }
  if (isOK)
  {
    busif.txBuf[0] = 2;  // register for start of encoder values
    isOK = ((write(busif.ttyDev, busif.txBuf, 1)) == 1);
  }
  if (!isOK)
    printf(PLUGINNAME " failed to request read from addr %d\n", 2);
  else
  {
    int n = read(busif.ttyDev, buf, 11);
    isOK = (n == 11);
    if (!isOK)
      printf(PLUGINNAME " failed to read status, requested %d bytes, got %d\n", 11, n);
  }
  if (isOK)
  {
    encoder1 = (buf[0] <<24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]; // Put encoder values together
    encoder2 = (buf[4] <<24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
    setVariable(md25.varEnc1, 0, encoder1);
    setVariable(md25.varEnc2, 0, encoder2);
    setVariable(md25.varBatt, 0, buf[8]);
    setVariable(md25.varM1Curr, 0, buf[9]);
    setVariable(md25.varM2Curr, 0, buf[10]);
//     if (debugFlag)
//       printf(PLUGINNAME "Encoder 1: %08X   Encoder 2: %08X, batt %d, curr1: %d, curr2: %d:\n",encoder1, encoder2, buf[8], buf[9], buf[10]);
  }
  if (!isOK)
  { // Read back 8 bytes for the encoder values and battery voltage and motor1 and 2 current (11 bytes in total)
    printf(PLUGINNAME " Failed to read from slave (read encoder)\n");
  }
  return isOK;
}


///////////////////////////////////

int driveMotors(int left, int right)
{
  int isOK, e = 0;
  struct timeval tm;
  isOK = ioctl(busif.ttyDev, I2C_SLAVE, md25.busid) >= 0;
  if (! isOK)
  { // Set the port options and set the address of the device we wish to sp$
    printf(PLUGINNAME " Unable to get bus access to talk to md25\n");
    e = 1;
  }
  busif.txBuf[0] = 0;     // Register to set speed of motor 1
  busif.txBuf[1] = left;  // speed to be set
  if (isOK)
  {
    isOK = ((write(busif.ttyDev, busif.txBuf, 2)) == 2);
    if (!isOK)
      e=2;
  }
  if (isOK)
  {
    busif.txBuf[0] = 1;                         // motor 2 speed
    busif.txBuf[1] = right;
    isOK = ((write(busif.ttyDev, busif.txBuf, 2)) == 2);
    if (!isOK)
      e=3;
  }
  if (!isOK)
    printf(PLUGINNAME " Error writing to i2c slave\n");
  if (logState != NULL)
  {
    gettimeofday(&tm, NULL);
    fprintf(logState, "%lu.%06lu motor left %d, right %d (err=%d)\n ", tm.tv_sec, tm.tv_usec, left, right, e);
  }
  return isOK;
}

///////////////////////////////////////

void stopMotors(void)
{
  int isOK = ioctl(busif.ttyDev, I2C_SLAVE, md25.busid) >= 0;
  if (! isOK)
  { // Set the port options and set the address of the device we wish to sp$
    printf(PLUGINNAME " Unable to get bus access to talk to md25\n");
  }
  else
  {
    busif.txBuf[0] = 0;
    busif.txBuf[1] = 128;                       // A speed of 128 stops the motor

    if ((write(busif.ttyDev, busif.txBuf, 2)) != 2) {
      printf(PLUGINNAME "Error writing to i2c slave\n");
    }

    busif.txBuf[0] = 1;
    busif.txBuf[1] = 128;

    if ((write(busif.ttyDev, busif.txBuf, 2)) != 2) {
      printf(PLUGINNAME "Error writing to i2c slave\n");
    }
  }
}

///////////////////////////////////////////

void getIMUdata()
{
  int isOK;
  isOK = ioctl(busif.ttyDev, I2C_SLAVE, imu.busid) >= 0;
  if (! isOK)
  { // Set the port options and set the address of the device we wish to sp$
    printf(PLUGINNAME " Unable to get bus access to talk to IMU\n");
  }
  if (isOK)
  { // accelerometer
    int m;
    busif.txBuf[0] = 59;
    m = write(busif.ttyDev, busif.txBuf, 1);
    isOK = (m == 1);
  }
  if (isOK)
  {
    int m = read(busif.ttyDev, busif.rxBuf, 14);
    isOK = (m == 14);
    if (isOK)
    {
      const float g = 9.8;
      const float ALSB = 16384.0 / 1000.0; // result in mm/s2
      const float GLSB = 131.0 / 1000.0; // result in m degrees/s
      int16_t ax, ay, az, t, gx, gy, gz;
      ay = -(busif.rxBuf[0] << 8 | busif.rxBuf[1]);
      ax =   busif.rxBuf[2] << 8 | busif.rxBuf[3];
      az =   busif.rxBuf[4] << 8 | busif.rxBuf[5];
      setVariable(imu.varAcc, 0, roundi(ax / ALSB * g));
      setVariable(imu.varAcc, 1, roundi(ay / ALSB * g));
      setVariable(imu.varAcc, 2, roundi(az / ALSB * g));
      t = busif.rxBuf[6] << 8 | busif.rxBuf[7];
      setVariable(imu.varTemp, 0, roundi(t / 3.40 + 35));
      gy = -(busif.rxBuf[8] << 8 | busif.rxBuf[9]);
      gx =   busif.rxBuf[10] << 8 | busif.rxBuf[11];
      gz =   busif.rxBuf[12] << 8 | busif.rxBuf[13];
      setVariable(imu.varGyro, 0, roundi(gx / GLSB));
      setVariable(imu.varGyro, 1, roundi(gy / GLSB));
      setVariable(imu.varGyro, 2, roundi(gz / GLSB));
      if (0)
      {
        float accX, accY, accZ, tmp, gyX, gyY, gyZ;
        accX = ax / 16384.0 * g;
        accY = ay / 16384.0 * g;
        accZ = az / 16384.0 * g;
        tmp = t / 340.0 + 35.0;
        gyX= gx/131.0;
        gyY= gy/131.0;
        gyZ= gz/131.0;
        if (debugFlag)
          printf(PLUGINNAME " IMU accX %.3f, accY %.3f, accZ %.3f, temp %.3f, gyX %.2f, gyY %.2f, gyZ %.2f\n", accX, accY, accZ, tmp, gyX, gyY, gyZ);
      }
    }
  }
//   if (isOK)
//   { // magnetometer
//     int m;
//     busif.txBuf[0] = 0x02;
//     m = write(busif.ttyDev, busif.txBuf, 1);
//     isOK = (m == 1);
//   }
//   if (isOK)
//   { // failed to get decent data from magnetometer
//     int m = read(busif.ttyDev, busif.rxBuf, 8);
//     isOK = (m == 8);
//     if (isOK)
//     { // scale is in uTesla * 10.
//       const float LSB = 1229.0 / 4096.0 * 10.0;
//       int16_t ax, ay, az;
//       float mx, my, mz;
//       ax =   busif.rxBuf[2] << 8 | busif.rxBuf[1];
//       ay = -(busif.rxBuf[4] << 8 | busif.rxBuf[3]);
//       az = -(busif.rxBuf[6] << 8 | busif.rxBuf[5]);
//       setVariable(imu.varMag, 0, roundi(ax * LSB));
//       setVariable(imu.varMag, 1, roundi(ay * LSB));
//       setVariable(imu.varMag, 2, roundi(az * LSB));
//       mx = ax  / 4096.0 * 1229.0;
//       my = ay  / 4096.0 * 1229.0;
//       mz = az  / 4096.0 * 1229.0;
//       printf(PLUGINNAME " IMU status1 %x magX %.3f, magY %.3f, magZ %.3f status2 %x\n", busif.rxBuf[0], mx , my, mz, busif.rxBuf[7]);
//     }
//     if (isOK)
//     { // magnetometer set single measurement
//       int m;
//       busif.txBuf[0] = 0x0a;
//       busif.txBuf[1] = 1;
//       m = write(busif.ttyDev, busif.txBuf, 2);
//       isOK = (m == 2);
//       if (!isOK)
//         printf(PLUGINNAME "failed to set magnetometer to single measurement mode\n");
//     }
//   }
}

////////////////////////////////////////////////

// void print_fused_euler_angles(mpudata_t *mpu)
// {
//   printf("\rX: %0.0f Y: %0.0f Z: %0.0f        ",
//       mpu->fusedEuler[VEC3_X] * RAD_TO_DEGREE,
//       mpu->fusedEuler[VEC3_Y] * RAD_TO_DEGREE,
//       mpu->fusedEuler[VEC3_Z] * RAD_TO_DEGREE);
// 
//   fflush(stdout);
// }
// 
// ////////////////////////////////////////////////
// 
// void print_fused_quaternions(mpudata_t *mpu)
// {
//   printf("\rW: %0.2f X: %0.2f Y: %0.2f Z: %0.2f        ",
//       mpu->fusedQuat[QUAT_W],
//       mpu->fusedQuat[QUAT_X],
//       mpu->fusedQuat[QUAT_Y],
//       mpu->fusedQuat[QUAT_Z]);
// 
//   fflush(stdout);
// }
// 
// ////////////////////////////////////////////////
// 
// void print_calibrated_accel(mpudata_t *mpu)
// {
//   printf("\rX: %05d Y: %05d Z: %05d        ",
//       mpu->calibratedAccel[VEC3_X],
//       mpu->calibratedAccel[VEC3_Y],
//       mpu->calibratedAccel[VEC3_Z]);
// 
//   fflush(stdout);
// }
// 
// ////////////////////////////////////////////////
// 
// void print_calibrated_mag(mpudata_t *mpu)
// {
//   printf("\rX: %03d Y: %03d Z: %03d        ",
//       mpu->calibratedMag[VEC3_X],
//       mpu->calibratedMag[VEC3_Y],
//       mpu->calibratedMag[VEC3_Z]);
// 
//   fflush(stdout);
// }
// 
// ////////////////////////////////////////////////
// 
// int set_cal(int mag, char *cal_file)
// {
//   int i;
//   FILE *f;
//   char buff[32];
//   long val[6];
//   caldata_t cal;
// 
//   if (cal_file) {
//     f = fopen(cal_file, "r");
// 
//     if (!f) {
//       perror("open(<cal-file>)");
//       return -1;
//     }
//   }
//   else {
//     if (mag) {
//       f = fopen("./magcal.txt", "r");
// 
//       if (!f) {
//         printf("Default magcal.txt not found\n");
//         return 0;
//       }
//     }
//     else {
//       f = fopen("./accelcal.txt", "r");
// 
//       if (!f) {
//         printf("Default accelcal.txt not found\n");
//         return 0;
//       }
//     }
//   }
// 
//   memset(buff, 0, sizeof(buff));
// 
//   for (i = 0; i < 6; i++) {
//     if (!fgets(buff, 20, f)) {
//       printf("Not enough lines in calibration file\n");
//       break;
//     }
// 
//     val[i] = atoi(buff);
// 
//     if (val[i] == 0) {
//       printf("Invalid cal value: %s\n", buff);
//       break;
//     }
//   }
// 
//   fclose(f);
// 
//   if (i != 6)
//     return -1;
// 
//   cal.offset[0] = (short)((val[0] + val[1]) / 2);
//   cal.offset[1] = (short)((val[2] + val[3]) / 2);
//   cal.offset[2] = (short)((val[4] + val[5]) / 2);
// 
//   cal.range[0] = (short)(val[1] - cal.offset[0]);
//   cal.range[1] = (short)(val[3] - cal.offset[1]);
//   cal.range[2] = (short)(val[5] - cal.offset[2]);
// 
//   if (mag)
//     mpu9150_set_mag_cal(&cal);
//   else
//     mpu9150_set_accel_cal(&cal);
// 
//   return 0;
// }
