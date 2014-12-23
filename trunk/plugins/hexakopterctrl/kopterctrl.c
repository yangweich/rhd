#define REVISION        "$Rev: 108 $:"
#define DATE            "$Date: 2013-01-20 17:34:12 +0100 (Sun, 20 Jan 2013) $"
#define ID              "$Id: kopterctrl.c 108 2013-01-20 16:34:12Z jcan $"
#define PLUGINNAME        "HexCtrl"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> 
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <semaphore.h>
#include <expat.h>
#include <string.h>
#include <math.h>

#include <smr.h>
#include <database.h>
#include <rhd.h>
#include <globalfunc.h>
#include "kopterctrl.h"

//
// XML config file decode structure
int debugFlag=0;
typedef struct {
    int depth;
    int skip;
    char enable;
    char found;
}   parseInfo;
//
/// create variables
void createRHDvariables();
/// init the device interface
int initPlugin(void);
void initPluginDefault(void);
//
/// local functions for configuration read
void XMLCALL lsStartTag(void *, const char *, const char **);
void XMLCALL lsEndTag(void *, const char *);

/**
 * get index to a named variable - from another plugin
 * \param type is either 'r' or 'w'
 * \param name is the name of the variable
 * \returns -1 if the variable is not found, else the index (positive or zero) of the variable. */
int getDatabaseVariable(char type, const char * name);
/**
 * limit integer to this range.
 * \param value is the value to be tested
 * \param min is the minimum value returned
 * \param max is the maximum value returned
 * \returns value except if it exceeds minimum or maximum, if so it returns the maximum or minimum value */
int limitInt(int value, int min, int max);
unsigned char limitUnsignedChar(int value, int min, int max);
char limitSignedChar(int value, int min, int max);
/// round a float to an integer
int roundi(const float v)
{ // round to closest integer
  if (v > 0)
    return (int)(v + 0.5);
  else
    return (int)(v - 0.5);
}

/** Periodic function is called by RHD.
 * It checks if any of the writeable in variables in RHD are 
 * updated, if so it makes sure they are within the boundries
 * and sends them to the Hexacopter */ 
int periodic(int tick)
{
  if (isUpdated('w', var.pdW))
  {
    var.pd[0] = getWriteVariable(var.pdW, 0)/1000.0;
    var.pd[1] = getWriteVariable(var.pdW, 1)/1000.0;
    setVariable(var.pdR, 0, roundi(var.pd[0] * 1000.0));
    setVariable(var.pdR, 1, roundi(var.pd[1] * 1000.0));
  }
  if (isUpdated('w', var.wrollctrl))
  {
    /// @todo hertil
  }
  if (isUpdated('w', var.wnickctrl))
  {  /// @todo hertil
  
  }
  if ( var.initCtrl == 0 || var.initForce == 0)
  {
    // force sensor values
    var.tanRoll = getDatabaseVariable('r', "tanroll");
    var.tanNick = getDatabaseVariable('r', "tannick");
    var.initForce = var.tanNick >= 0 && var.tanRoll >= 0;
    // control variables

    var.kopterPose = getDatabaseVariable('r', "pose3d");
    var.nickRef = getDatabaseVariable('w', "nickref");
    var.rollRef = getDatabaseVariable('w', "rollref");
    var.yawRef = getDatabaseVariable('w', "yawref");
    var.gasref = getDatabaseVariable('w', "thrustref");
    // is external control enabled
    var.inCtrl = getDatabaseVariable('r', "externalctrl"); /// nick,roll,yaw,trust,height,control,frame
    // are all controll values in place
    var.initCtrl = var.gasref >= 0 && var.nickRef >= 0 &&
                   var.rollRef >= 0 && var.yawRef >= 0 &&
                   var.inCtrl >= 0 && var.kopterPose >= 0;
    if (tick %100 == 0)
      if (var.initCtrl == 0 || var.initForce == 0)
        printf(PLUGINNAME "failed to init ctrl %d, force %d\n", var.initCtrl, var.initForce);
    if (var.initCtrl)
    {
      double dt;
      struct timeval t; /// time now
      // get time since last sample
      gettimeofday(&t, NULL);
      dt = t.tv_sec - var.ts.tv_sec + (t.tv_usec - var.ts.tv_usec)/1e6;
      var.ts = t;
      //
      if (dt < 1.0 && dt > 1e-4)
      {
        var.cnick[0] = 0.0;
        var.croll[0] = 0.0;
        var.eroll[0] = 0.0;
        var.enick[0] = 0.0;
        // calculate control factors
        /// @todo hertil
      }
    }
  }
  else
  { // ready to control
    int inCtrl = getReadVariable(var.inCtrl, 5);
    if (inCtrl || debugFlag)
    { // we should do something
      // get tangent to wire angle in radians relative to kopter
      float troll = getReadVariable(var.tanRoll, 0) / 1000.0;
      float tnick = getReadVariable(var.tanNick, 0) / 1000.0;
      // get kopter roll and nick (tilt) absolute
      // 3D pose is (x,y,z ???),nick,roll,yaw
      float kroll = getReadVariable(var.kopterPose, 4) * M_PI / 1800;
      float knick = getReadVariable(var.kopterPose, 3) * M_PI / 1800;
      //
      float wroll = atan(troll);
      float wnick = atan(tnick);
      //
      if (debugFlag || tick % 100 == 0)
      {
        printf(PLUGINNAME ": roll wire %.1f kopter %.1f; nick wire %.1f kopter %.1f [deg]\n",
              wroll*180.0/M_PI, kroll*180.0/M_PI, wnick*180.0/M_PI, knick*180.0/M_PI);
      }
      // get new error value
      var.eroll[1] = var.eroll[0];
      var.enick[1] = var.enick[0];
      var.eroll[0] = getWriteVariable(var.rollRef, 0) - wroll + kroll;
      var.enick[0] = getWriteVariable(var.nickRef, 0) - wnick + knick;
      // controller
      var.croll[1] = var.croll[0];
      var.croll[1] = var.croll[0];
      /// @todo var.croll[0] = var.pd[0] * (var.eroll[0] - var.eroll[1] * 100.0 * var.pd[1] / (1 + 100.0 * var.pd[1]) + 999;
    }
    else if (tick % 100 == 0)
      printf(PLUGINNAME ": controlling\n");
  }
  //
  return 0;
}

/**
 * Initialize plugin - default values. */
void initPluginDefault()
{
  var.initForce = 0;
  var.initCtrl = 0;
}

/**
 * Initialize plugin - after read of configuration file. */
int initPlugin()
{
  createRHDvariables();
  printf(PLUGINNAME ": created variables\n");
  return 1;
}

/**
 * Create RHD variables as needed for this plugin. */
void createRHDvariables()
{ // RHD variables
  var.pdR       = createVariable('r',3,"wctrlpd");
  var.pdW       = createVariable('w',3,"wctrlpd");
  var.wrollctrl = createVariable('r',3,"wrollref");
  var.wnickctrl = createVariable('w',3,"wnickref");
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
  initPluginDefault();
  //Find revision number from SVN Revision
  char * p1;
  char versionString[20] = REVISION;
  char tempString[10];
  p1 = strrchr(versionString, '$');
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  printf(PLUGINNAME " Initializing plug-in version %s.%s\n",REVISION, tempString);
  /* Initialize Expat parser*/
  XML_Parser parser = XML_ParserCreate(NULL);
  result = parser != 0;
  devif.baudrate = 57600;
  strncpy(devif.devName, "/dev/ttyUSB0", MxDL);
  if (!result)
    fprintf(stderr, PLUGINNAME " Couldn't allocate memory for XML parser\n");
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
  if (result && xmlParse.enable)
  { // all is fine - start plugin
    result = initPlugin();
  }
  if (result)
    return 1;
  else
    return -1;
}
//////////////////////////////////////////////////////

int terminate(void)
{
    printf(PLUGINNAME ": [OK]\n");
    return 0;
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
            // this one handles hexakopter stuff only
            if (strcmp("kopterctrl",el) == 0)
            { // is it enabled, the only info needed
                for(i = 0; attr[i]; i+=2)
                {
                    const char * att, * val;
                    att = attr[i];
                    val = attr[i + 1];
                    if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
                        info->enable = 1;
                    else if (strcasecmp("pid", att) == 0)
                    {
                      char * p1 = (char *)val;
                      int i;
                      for (i = 0; i < 2 && p1 != NULL; i++)
                        var.pd[i] = strtod(p1, &p1);
                      printf(PLUGINNAME ": PD parameters (proportional=%f, differential=%f)\n", var.pd[0], var.pd[1]);
                      if (i != 2)
                        printf(PLUGINNAME ": expected 2 PID parameters values - found just %d!\n", i);
                    }
                    else if (strcmp("debug", att) == 0)
                    {
                        debugFlag = strtol(val, NULL, 0);
                        if (debugFlag)
                            printf(PLUGINNAME ": started in DEBUG mode!\n");
                    }
                }
                if (!info->enable)
                    printf(PLUGINNAME ": hexakopter: Use is disabled in configuration\n");
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

