 /**
 *  FLight Control System for TCUAV
 *
 * Author: Peter Savnik
 *
 *******************************************************************/
/***************************** Plug-in version  *****************************/
#define PHBR_VERSION    "0.1"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 0 $:"
 #define PLUGINNAME       "fcs"
/***************************************************************************/


//#include <sched.h> 
//#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
//#include <signal.h>
//#include <linux/serial.h>
#include <sys/time.h>
//#include <sys/mman.h>
#include <expat.h>
//#include <poll.h>
#include <math.h>
#include <stdint.h>
#include <sys/time.h>

//RHD Core headers
#include <rhd.h>
#include "../rhdlink/rhdlink.h"
#include "../rhdlink/librhdlink.h"
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "fcs.h"


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

// Magic definitions 
//int getDatabaseVariableLink(char type, const char * name);
//symTableElement*  getSymbolTableLink(char);
//int getSymbolTableSizeLink(char);

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
int debugFlag = 0;
struct timeval tickTime;

int x,y,z; 
int RHDLinkConnected = 0;
int varRHDLinkConnected;
int varPhiGCS,varRGCS,varCableLengthGCS; // vars from rhd link (Ground Control Station)
int varR,varPhi,varForce, varTheta; // Database variable
double phiGCS,rGCS,cableLength; // vars from RHDlink (Ground Control Station)
double phi,r,theta;


////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int rhdTick)
{
  tick = rhdTick;
  
  // 1. Get Phidgets values, Get RHD link variables
  // 2. Calculate
  // 3. Update database 
  
  // 1. Get Phidgets values and RHD link
  varForce = getDatabaseVariable('r', "force"); // scaled by 10000
  symTableElement *st = getSymtable('r');
  x = st[varForce].data[1];
  y = st[varForce].data[0];
  z = st[varForce].data[2];
  
  // RHD LINK
  varRGCS = getDatabaseVariableLink('r',"r");
  varPhiGCS = getDatabaseVariableLink('r',"r");
  varCableLengthGCS = getDatabaseVariableLink('r',"r");
  
  symTableElement *stLink = getSymbolTableLink('r');
  rGCS = stLink[varRGCS].data[0];
  phiGCS = stLink[varPhiGCS].data[0];
  cableLength = stLink[varCableLengthGCS].data[0];
  
  // 2. Calculate translate to angle
  r = sqrt(pow(x/10000,2)+pow(y/10000,2)+pow(z/10000,2));

  theta = acos(z/r);

  phi = atan2(y,x);
  
  // 3. Update database
  setVariable(varPhi, 0, roundi(phi));    
  setVariable(varR, 0, roundi(r));  
  setVariable(varTheta, 0, roundi(theta));    
  
  
  return 0;
}

//////////////////////////////////////////////////////
/**
 * Create RHD variables */

void createVariables()
{ // create variables in RHD
  varPhi = createVariable('r',1,"phi"); // Angle
  varR = createVariable('r',1,"r"); // combined force
  varTheta = createVariable('r',1,"theta"); // Angle
  
  // RHD link vars
  varRHDLinkConnected = createVariable('r',1,"RHDLinkConnected");
  varPhiGCS = createVariable('w',1,"phiLink");
  varRGCS = createVariable('w',1,"RLink");
  varCableLengthGCS = createVariable('w',1,"cableLengthLink");
  
  printf(PLUGINNAME ": has created read and write variables\n");
}

////////////////////////////////////////////////////////////////

int terminate(void)
{
 
  printf(PLUGINNAME ": stopping ... ");
  
  
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

  // initialize default values
  // this is variable in another plugin,
  // and can not be set just now.
  //data.varForce = -1;
  //data.varUpdateRate = -1;
  //data.varRangeErr = -1;
  //data.varEnableCh = -1;
  
  
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
        fprintf(stderr, "   FCS: Couldn't allocate memory for XML File buffer\n");
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
  else if(!xmlParse.enable) printf("ERROR: xmlParse.enable is False!");
  else if(!result) printf("ERROR: result is False!");
  
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
        if (strcmp("fcs",el) == 0)
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
                printf("   FCS started in DEBUG mode!\n");
            }
            
            //else if (strcasecmp("updateTimeMs",att) == 0)
              // update period in ms allowed is 8..1000
              //busif.updateTms = strtol(val, NULL, 0);
	  
          }
          if (!info->enable)
            printf("   FCS: Use is disabled in configuration\n");
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



//////////////////////////////////////////////

/**
 * Initialize the communication and start rx thread
 * \returns 1 on success (else 0) */
int init(void)
{ 
    createVariables();
  
  return 1;
}



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











