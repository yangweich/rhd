 /**
 *  GROUND CONTROL STATION for TCUAV
 *
 * Author: Peter Savnik
 *
 *******************************************************************/
/***************************** Plug-in version  *****************************/
#define PHBR_VERSION    "0.1"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 0 $:"
 #define PLUGINNAME       "tcuav"
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
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "tcuav.h"
#include "pwm.h"
#include "gpio.h"


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
int debugFlag = 0;
struct timeval tickTime;

int x,y; 
int varPhi, varForce,varR, varSpeedZ, varSpeedLR, varSpeedFwd, varSpeedSpin, varCableLength; // Database variable
double phi,r,speedZ;

unsigned int M1EN;	// Enable
unsigned int M1NA;	// Direction
unsigned int M1NB;	// Direction
unsigned int M1CS;	// Current sense


////////////////////////////////////////////////////////

/**
 * init new requests to bus (called periodically)
 * */
extern int periodic(int rhdTick)
{
  tick = rhdTick;
  
  // 1. Get Phidgets values
  // 2. Calculate
  // 3. Update database 
  
  // 1. Get Phidgets values
  varForce = getDatabaseVariable('r', "force"); // scaled by 10000
  symTableElement *st = getSymtable('r');
  x = st[varForce].data[0];
  y = st[varForce].data[1];
  speedZ = st[varSpeedZ].data[0];
  
  // 2. Calculate translate to angle
  phi = atan2(y/10000.0,x/10000.0); // Range [-pi,pi]
  phi = phi * (180.0/M_PI); // translate to degree
  if(phi < 0) phi = 360+phi; // from range [-180; 180] to [0,360]
  
  r = sqrt(pow(x,2)+pow(y,2));
  
  // Speed Z set direction
  if(speedZ > 0){ // Clockwise
    gpio_set_value(M1NA, HIGH);
    gpio_set_value(M1NB, LOW);
  }
  else if(speedZ < 0){ // Counter Clockwise
    gpio_set_value(M1NA, LOW);
    gpio_set_value(M1NB, HIGH);
  }
  else{ // disabled
    gpio_set_value(M1NA, LOW);
    gpio_set_value(M1NB, LOW);
  }
   
  pwm_set_duty(speedZ*(5000000/5000)); // speed*(DYTY/MAX_JOYSTICK)
  
  
  
  // 3. Update database
  setVariable(varPhi, 0, roundi(phi));    
  setVariable(varR, 0, roundi(r));  
      
  
  
  return 0;
}

//////////////////////////////////////////////////////
/**
 * Create RHD variables */

void createVariables()
{ // create variables in RHD
  varPhi = createVariable('r',1,"phi"); // Angle
  varR = createVariable('r',1,"r"); // combined force
  varCableLength = createVariable('r',1,"cableLength");
  varSpeedZ = createVariable('w',1,"speedZ"); // Joystick speed up/down
  varSpeedFwd = createVariable('w',1,"speedFwd"); // Joystick speed Fwd/Back
  varSpeedSpin = createVariable('w',1,"speedSpin"); // Joystick speed spin
  varSpeedLR = createVariable('w',1,"speedLR"); // Joystick speed Left/right
  
  printf(PLUGINNAME ": has created read and write variables\n");
}

////////////////////////////////////////////////////////////////

int terminate(void)
{
 
  printf(PLUGINNAME ": stopping ... ");
  
  // echo 1/0 > value
  gpio_set_value(M1EN, LOW);
  gpio_set_value(M1NA, LOW);
  gpio_set_value(M1NB, LOW);
  gpio_set_value(M1CS, LOW);
  printf("Values set to zero... ");
	
  // Setup PWM to zero output
  pwm_set_enable(0);
  pwm_set_duty(0);
  pwm_set_polarity(1);
  printf("PWM disabled...");
  
  // release GPIO
  gpio_unexport(M1EN);
  gpio_unexport(M1NA);
  gpio_unexport(M1NB);
  gpio_unexport(M1CS);
  printf("GPIO unexported... ");
  
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
        fprintf(stderr, "   TCUAV: Couldn't allocate memory for XML File buffer\n");
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
        if (strcmp("tcuav",el) == 0)
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
                printf("   TCUAV started in DEBUG mode!\n");
            }
            
            //else if (strcasecmp("updateTimeMs",att) == 0)
              // update period in ms allowed is 8..1000
              //busif.updateTms = strtol(val, NULL, 0);
	  
          }
          if (!info->enable)
            printf("   TCUAVz: Use is disabled in configuration\n");
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
{ //Open first serial port
  int result;
  //
  // open device
  //result = openAndSetMode();
  result = 1;
  //data.rateTime = 0.0;
  if (result == 1)
  { // start thread to handle bus
    printf(" attach bridge\n");
  }
  if (result == 1 || debugFlag)
  { /****** Create database variables if all is ok **************/
    //int waitCount = 0;
    // create RHD database variables
    createVariables();
  }
  
  printf("Setting up Motor pin configuration\n");

  M1EN = gpio_no(0,30);	// P9-11
  M1NA = gpio_no(1,28);	// P9-12
  M1NB = gpio_no(0,31);	// P9-13
  M1CS = gpio_no(1,19);	// P9-15
	
  // echo n > export
  gpio_export(M1EN);
  gpio_export(M1NA);
  gpio_export(M1NB);
  gpio_export(M1CS);
	
  // Set Direction, echo in/out > direction
  gpio_set_dir(M1EN, OUTPUT_PIN);
  gpio_set_dir(M1NA, OUTPUT_PIN);
  gpio_set_dir(M1NB, OUTPUT_PIN);
  gpio_set_dir(M1CS, INPUT_PIN);
	
  // echo 1/0 > value
  gpio_set_value(M1EN, LOW);
  gpio_set_value(M1NA, LOW);
  gpio_set_value(M1NB, LOW);
  gpio_set_value(M1CS, LOW);
	
  // Setup PWM to zero output
  pwm_set_enable(0);
  pwm_set_period(5000000);
  pwm_set_duty(0);
  pwm_set_polarity(1);
  
  // Ready?? Enable motor and output
  pwm_set_enable(1);
  gpio_set_value(M1EN, HIGH);
  
  return result;
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











