 /** \file saberandmagenc.c
 *  \ingroup hwmodule
 *
 * Interface for sabertooth motor controller and AS4050 magnetic encoders
 * through an arduino nano velocity controller
 *
 *
 *       NBNB! not finished (18/6 2012) Christian
 * 
 * 
 *******************************************************************/
/***************************** Plug-in version  *****************************/
#define PHBR_VERSION    "0.1501"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 544 $:"
 #define DATE             "$Date: 2014-08-27 18:50:14 +0200 (Wed, 27 Aug 2014) $:"
 #define SVNID            "$Id: phidgetbridge.c 544 2014-08-27 16:50:14Z jcan $"
 #define PLUGINNAME       "PhBr"
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
#include <sys/time.h>

//RHD Core headers
#include <rhd.h>
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "phidgetbridge.h"


#define LS_READ_BYTES 20
/// number of AD channals
#define AD_COUNT  4

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

/**
 * limit integer to this range.
 * \param value is the value to be tested
 * \param min is the minimum value returned
 * \param max is the maximum value returned
 * \returns value except if it exceeds minimum or maximum, if so it returns the maximum or minimum value */
int limitInt(int value, int min, int max);

/******** Global variables *************/


int tick = 0;
/// old steering angle value - in radians
double sacOld = -4.0;
int debugFlag = 0;
/** flags for updating of write variables */
int settingUpd = 1, steerUpd = 1;
struct timeval tickTime;


////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int rhdTick)
{
  tick = rhdTick;
  if (isUpdated('w', data.varFilterW))
  { // is filter value changed
    data.filter = getWriteVariable(data.varFilterW, 0);
    setVariable(data.varFilterR, 0, data.filter);
  }
    //
  return 0;
}

//////////////////////////////////////////////////////
/**
 * Create RHD variables */

void createVariables()
{ // create variables in RHD
  data.varForce = createVariable('r',4,"force"); // measured value
  data.varRangeErr = createVariable('r', 4, "forceerr");
  data.varUpdateRate = createVariable('r',1,"forcerate");
  data.varFilterR = createVariable('r',1,"forcefilter");
  data.varFilterW = createVariable('w',1,"forcefilterref");
  data.varRoll = createVariable('r',1,"tanroll");
  data.varNick = createVariable('r',1,"tannick");
  printf(PLUGINNAME ": has created read and write variables\n");
}

////////////////////////////////////////////////////////////////

int terminate(void)
{
  CPhidgetHandle bridge = (CPhidgetHandle)busif.bridge;
  printf(PLUGINNAME ": stopping ... ");
  
  CPhidget_close(bridge);
  CPhidget_delete(bridge);
  printf("[OK]\n");
  return 0;
}

/************************** XML Initialization **************************/


/** \brief Initialize the plug-in from XML file
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
  // make version string
  p1 = strrchr(versionString, '$');
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  //
  printf(PLUGINNAME ": Initializing plug-in (version %s.%s)\n",PHBR_VERSION, tempString);
//   printf(PLUGINNAME ": waiting for udev to detect device .");
//   for (i = 0; i < 3; i++)
//   {
//     fflush(NULL);
//     sleep(1);
//     printf(".");
//   }
//   printf(" [OK]\n");
  // initialize default values
    // this is variable in another plugin,
  // and can not be set just now.
  data.varForce = -1;
  data.varUpdateRate = -1;
  data.varRangeErr = -1;
  data.varNick = -1;
  data.varRoll = -1;
  data.down = 1.0;
  data.idxDown = 0;
  data.idxNick = 1;
  data.idxDown = 2;
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
        fprintf(stderr, "   PhBr: Couldn't allocate memory for XML File buffer\n");
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
void XMLCALL lsStartTag(void *datainfo, const char *el, const char **attr)
{ // a start tag is detected
  int i;
  parseInfo *info = (parseInfo *) datainfo;
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
        // this one handles interface to PhBr (Sabertooth and magnetic encoder)
        // through arduino interface - the only option is debug
        if (strcmp("phidgetbridge",el) == 0)
        { // get enable bit and device name
          const char * att;
          const char * val;
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i]; 
            val = attr[i + 1];
            if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
              info->enable = 1;
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
              if (debugFlag)
                printf("   PhBr started in DEBUG mode!\n");
            }
            else if (strcasecmp("updateTimeMs",att) == 0)
              // update period in ms allowed is 8..1000
              busif.updateTms = strtol(val, NULL, 0);
            else if (strcasecmp("gain",att) == 0)
            { // sensor gain  1..128
              const char * p1 = val;
              int i = 0;
              while ((p1 != NULL) && (*p1 > '\0') && (i < 4))
              { // get up to 4 gain values (does not change scaled value)
                busif.gain[i++] = strtol(p1, (char**)&p1, 0);
              }
            }
//             else if (strcasecmp("scale",att) == 0)
//             { // sensor scale in mN/V, from bridge value in volt (microvolt) to integer value in mN.
//               const char * p1 = val;
//               int i = 0;
//               while ((p1 != NULL) && (*p1 > '\0') && (i < 4))
//               { // get up to 4 gain values
//                 busif.scale[i++] = strtod(p1, (char**)&p1);
//               }
//             }
            else if (strcasecmp("filter",att) == 0)
            { // sensor scale in mN/V, from bridge value in volt (microvolt) to integer value in mN.
              data.filter = strtol(val, NULL, 0);
            }
            else if (strcasecmp("idxroll", att) == 0)
            {
                data.idxRoll = limitInt(strtol(val, NULL, 10),0,3);
                if (data.idxRoll != strtol(val, NULL, 10))
                  printf(PLUGINNAME ": roll index out if range\n");
            }
            else if (strcasecmp("idxnick", att) == 0)
            {
                data.idxNick = limitInt(strtol(val, NULL, 10),0,3);
                if (data.idxNick != strtol(val, NULL, 10))
                  printf(PLUGINNAME ": nick (tilt) index out if range\n");
            }
            else if (strcasecmp("idxdown", att) == 0)
            {
                data.idxDown = limitInt(strtol(val, NULL, 10),0,3);
                if (data.idxDown != strtol(val, NULL, 10))
                  printf(PLUGINNAME ": down (cable weight) index out if range\n");
            }
            else if (strcasecmp("scaleroll", att) == 0)
            {
                data.scaleRoll = strtod(val, NULL);
            }
            else if (strcasecmp("scalenick", att) == 0)
            {
                data.scaleNick = strtod(val, NULL);
            }
            else if (strcasecmp("scaledown", att) == 0)
            {
                data.scaleDown = strtod(val, NULL);
            }
          }
          if (!info->enable)
            printf("   PhBr: Use is disabled in configuration\n");
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

int attachHandler(CPhidgetHandle phid, void *userptr)
{
  int i;
  CPhidgetBridgeHandle bridge = (CPhidgetBridgeHandle)phid;
  CPhidgetBridge_setEnabled(bridge, 0, PTRUE);
  CPhidgetBridge_setEnabled(bridge, 1, PTRUE);
  CPhidgetBridge_setEnabled(bridge, 2, PTRUE);
  CPhidgetBridge_setEnabled(bridge, 3, PTRUE);
  for (i = 0; i < 4; i++)
  {
    CPhidgetBridge_Gain gain;
    if (busif.gain[i] < 4)
      gain = PHIDGET_BRIDGE_GAIN_1;
    else if (busif.gain[i] < 12)
      gain = PHIDGET_BRIDGE_GAIN_8;
    else if (busif.gain[i] < 24)
      gain = PHIDGET_BRIDGE_GAIN_16;
    else if (busif.gain[i] < 49)
      gain = PHIDGET_BRIDGE_GAIN_32;
    else if (busif.gain[i] < 96)
      gain = PHIDGET_BRIDGE_GAIN_64;
    else
      gain = PHIDGET_BRIDGE_GAIN_128;
    //
    CPhidgetBridge_setGain(bridge, 0, gain);
    CPhidgetBridge_setGain(bridge, 1, gain);
    CPhidgetBridge_setGain(bridge, 2, gain);
    CPhidgetBridge_setGain(bridge, 3, gain);
    CPhidgetBridge_setDataRate(bridge, busif.updateTms);
  }

  printf("   PhBr: Attach handler ran!\n");
  return 0;
}

int detachHandler(CPhidgetHandle phid, void *userptr)
{
        printf("   PhBr: Detach handler ran!\n");
        return 0;
}

int errorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *errorStr)
{
  const char * pe, *pi;
  int i;
  pe = strstr(errorStr, "Overrange");
  pi = strstr(errorStr, "nput");
  if (pi != NULL)
  {
    pi += 5;
    i = strtol(pi, NULL, 0);
    setVariable(data.varRangeErr, i, pe != NULL);
  }
  printf("   PhBr: Error event: %s\n",errorStr);
  return 0;
}

int gotdata(CPhidgetBridgeHandle phid, void *userPtr, int index, double val)
{
  double mn;
  struct timeval t;
  double now;
  //CPhidgetBridgeHandle bridge = (CPhidgetBridgeHandle)phid;
  if (index < 4)
  {
    if (tick > 100 && data.filter > 1)
      // low pass filter value
      mn = data.values[index] * (data.filter - 1) / data.filter + val / data.filter;
    else
      mn = val;
    setVariable(data.varForce, index, roundi(mn * 10000.0));
    data.values[index] = mn;
    if (index == data.idxRoll)
    {
      data.froll = mn * data.scaleRoll / data.fdown;
      setVariable(data.varRoll, 0, roundi(data.froll * 1000));
    }
    if (index == data.idxNick)
    {
      data.fnick = mn * data.scaleNick / data.fdown;
      setVariable(data.varNick, 0, roundi(data.fnick * 1000));
    }
    if (index == data.idxDown)
    { // avoid division by zero
      data.fdown = mn * data.scaleDown;
      if (fabs(data.fdown) < 1e-3)
        data.fdown = 1e-4;
    }
  }
  gettimeofday(&t, NULL);
  now = t.tv_sec + t.tv_usec * 1e-6;
  data.dataCnt++;
  if (now - data.rateTime > 1.0)
  {
    setVariable(data.varUpdateRate, 0, data.dataCnt/4);
    data.dataCnt = 0;
    data.rateTime = now;
  }
    
  //printf("   PhBr: %lu.%06lu Data Event (%d) %10.6lfV, %8dmN\n", t.tv_sec, t.tv_usec, index, val, mn);
  return 0;
}

void display_generic_properties(CPhidgetHandle phid)
{
  int sernum, version;
//  int i;
//  double mn, mx;
  const char *deviceptr;
  CPhidget_getDeviceType(phid, &deviceptr);
  CPhidget_getSerialNumber(phid, &sernum);
  CPhidget_getDeviceVersion(phid, &version);
  //
//   for (i = 0; i < 4; i++)
//   {
//     CPhidgetBridge_getBridgeMax((CPhidgetBridgeHandle) phid, i, &mx);
//     CPhidgetBridge_getBridgeMin((CPhidgetBridgeHandle) phid, i, &mn);
//   }

  printf("   PhBr: %s\n", deviceptr);
  printf("   PhBr: Version: %8d SerialNumber: %10d\n", version, sernum);
}

////////////////////////////////////////////////////////////

int openAndSetMode()
{
  int result = 0;
  const char *err;
  CPhidgetHandle * bridge = (CPhidgetHandle *)&busif.bridge;
  //
  busif.p2 = busif.rxBuf;
  *busif.p2 = '\0';
  //
//   CPhidgetBridgeHandle bridge;
  CPhidget_enableLogging(PHIDGET_LOG_VERBOSE, NULL);

  CPhidgetBridge_create(&busif.bridge);

  CPhidget_set_OnAttach_Handler(*bridge, attachHandler, NULL);
  CPhidget_set_OnDetach_Handler(*bridge, detachHandler, NULL);
  CPhidget_set_OnError_Handler(*bridge, errorHandler, NULL);

  CPhidgetBridge_set_OnBridgeData_Handler(busif.bridge, gotdata, NULL);

  CPhidget_open(*bridge, -1);

  //Wait for 5 seconds, otherwise fail
  result = CPhidget_waitForAttachment((CPhidgetHandle)bridge, 5000);
  if(!result)
  {
    CPhidget_getErrorDescription(result, &err);
    printf("   PhBr: Problem waiting for bridge attachment: %s\n", err);
  }
  else
    display_generic_properties((CPhidgetHandle)bridge);
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
  // open device
  result = openAndSetMode();
  data.rateTime = 0.0;
  if (result == 1)
  { // start thread to handle bus
    printf(" attach bridge\n");
  }
  //if (result == 1 || debugFlag)
  { /****** Create database variables if all is ok **************/
    //int waitCount = 0;
    // create RHD database variables
    createVariables();
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

