/** \file joystickcontrol.c
 *  \ingroup hwmodule
 *  \brief Remotecontrol plugin using joystick
 *
 *  This plug-in provides remote control possibilities using a
 *  standard HID joystick.
 *
 *  The joystick driver should be initialized in safety-mode
 *  to be able to override the automatic computerized control.
 *
 *  \author Anders Billso Beck
 *  $Rev: 282 $
 *  $Date: 2012-07-29 18:20:37 +0200 (Sun, 29 Jul 2012) $
 *  Updated by: Peter Savnik 2014
 *  Added mode for UAV
 */
/***************************************************************************
 *                  Copyright 2011 Anders Billesø Beck, DTU                *
 *                       anbb@teknologisk.dk                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
/************************** Library version  ***************************/
#define JOYCONTROLVERSION "1.2"
/************************** Version control information ***************************/
 #define REVISION         "$Rev: 282 $:"
 #define DATE             "$Date: 2012-07-29 18:20:37 +0200 (Sun, 29 Jul 2012) $:"
/**********************************************************************************/

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
#include <arpa/inet.h>
#include <expat.h>
#include <math.h>
#include <linux/joystick.h>

//RHD Core headers
#include <rhd.h>
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "joycontrol.h"

/******** Global variables *************/
int iAxes, iButtons, iOverride, iEstop;
/*
symTableElement * seSpeedr = NULL, *seSpeedl = NULL;
symTableElement * seVel = NULL, * seOmega = NULL;
symTableElement * seSpeedRef = NULL, * seAngleRef = NULL;
*/
int seSpeedr = -1, seSpeedl = -1;
int seVel = -1, seOmega = -1;
int seSpeedRef = -1, seAngleRef = -1;
int seLR = -1, seFwd = -1, seSpin = -1, seUpDown = -1; // UAV


int jDev = 0;  //File descriptors
char joyDev[64];
volatile int joyRunning = -1;
pthread_t rxThread;
pthread_attr_t attr;
char toggleState = 0, manOverride = 0;


//Structure to hold joystick values
struct jVal {
	int button[16];
	int axes[16];
};
struct jVal joyValues;
char number_of_axes, number_of_buttons;

//Structure to hold control configuration
struct controlConfig {
	int maxfwd;
	int maxturn;
	int maxlr; //Max Left right
	int maxspin;
	int maxupdown;
	int deadmin;
	int deadmax;
        int fastBut;
	char control;
	double scaleleft;  // for differential drive
	double scaleright;
        double slowFactor;
};
struct controlConfig ctrlConfig;

/// redirection of joypad axis and buttons
typedef struct routeConfig {
  int axis;
  int button;
  float scale;
  int offset;
  int writeVar;
  int writeIdx;
  char varName[32];
} RouteConfig;
#define ROUTES_MAX 50
RouteConfig routes[ROUTES_MAX];
int routesCnt = 0;

/******** Function prototypes *************/
int initJoyCtrl(void);
void *rxtask(void *);

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
 * get index to a named variable - from another plugin
 * \param type is either 'r' or 'w'
 * \param name is the name of the variable
 * \returns -1 if the variable is not found, else the index (positive or zero) of the variable. */
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

/** \brief Initialize communications and variables
 *
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
int initJoyCtrl(void) {


	//Open first serial port
  if ((jDev = open (joyDev, O_RDWR /*| O_NONBLOCK*/)) == -1) {
    fprintf(stderr,"   Can't open joystick port: %s\n",joyDev);
    joyRunning = -1;
    return -1;
  }  else {
      printf("   Opened joystick on port: %s\n",joyDev);
      joyRunning = 2;
  }

  //Query and print the joystick name
  char name[128];
  if (ioctl(jDev, JSIOCGNAME(sizeof(name)), name) < 0)
        strncpy(name, "Unknown", sizeof(name));
  printf("   Joystick model: %s\n", name);

  //Query and print number of axes and buttons
  ioctl (jDev, JSIOCGAXES, &number_of_axes);
  ioctl (jDev, JSIOCGBUTTONS, &number_of_buttons);
  printf("   Registrered %d axes and %d buttons on joystick\n",number_of_axes,number_of_buttons);

   //Extract the symtableelement for speedl and speedr
  symTableElement *wtab = getSymtable('w');
  int wtSize = getSymtableSize('w');
  int i;
  // debug
  printf("joycontrol found %d write symbol:\n", wtSize);
  
  /****** Create database variables if all is ok **************/
  iAxes		= createVariable('r',number_of_axes,"joyaxes");
  iButtons      = createVariable('r',number_of_buttons,"joybuttons");
  iOverride     = createVariable('r',1,"joyoverride");
  iEstop        = createVariable('w',10,"joy");
  
  // debug end
  for(i = 0; i < wtSize; i++) {
    // debug
    printf("   symbol %d is (len %d) '%s'\n", i, (int)strlen(wtab[i].name), wtab[i].name);
    // debug end
    if (strcmp("speedl",wtab[i].name) == 0) {
        seSpeedl = i;
    } else if (strcmp("speedr",wtab[i].name) == 0) {
        seSpeedr = i;
    }
    // maybe vel-omega control
    else if (strcmp("cmdtransvel",wtab[i].name) == 0)
      seVel = i;
    else if (strcmp("cmdrotvel",wtab[i].name) == 0)
      seOmega = i;
    // or maybe ackerman control
    else if (strcmp("steeringangleref",wtab[i].name) == 0)
      seAngleRef = i;
    else if (strcmp("speedref",wtab[i].name) == 0)
      seSpeedRef = i;
    else if (strcmp("speedLR",wtab[i].name) == 0)
      seLR = i; //UAV
    else if (strcmp("speedfwd",wtab[i].name) == 0)
      seFwd = i; //UAV
    else if (strcmp("speedSpin",wtab[i].name) == 0)
      seSpin = i; //UAV
    else if (strcmp("speedZ",wtab[i].name) == 0){
      seUpDown = i; //UAV
      printf("speedZ Found!\n");}
  } // steeringangleref speedref

	//Printout control status
    if (ctrlConfig.control) 
      printf("   Joystick override robot control: ENABLED!\n");
    else	
      printf("   Joystick override robot control: disabled\n");

  //Exit if no speedl+speedr was found
  if (ctrlConfig.control == 1)
  { // there should be something to control
    if ((seSpeedr != -1) && (seSpeedl != -1))
      printf("Differential control enabled\n");
    else if ((seVel != -1) && (seOmega != -1))
      printf("Vel-omega control enabled\n");
    else if ((seSpeedRef != -1) && (seAngleRef != -1))
      printf("Ackerman control enabled\n");
    else if (seUpDown != -1)
      printf("UAV control enabled\n");
    else
    { // no control set of variables - exit
      printf("   ERROR: Looked for Ackerman, differential, Vel-omega and UAV control variables!\n");
      printf("   ERROR: but nothing found, control not possible - proceding however!\n");
      
    }
  }


  

  //Start RX thread
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  if (pthread_create(&rxThread, &attr, rxtask, 0))
    {
      perror("   Can't start Joystick receive thread");
      joyRunning = -1;
      return -1;
    }

  while (joyRunning > 1) usleep(10000); //Don't proceed before threads are running

  return 1;
}


/** \brief Periodic joystick control function
 *
 *  * \param[in] int tick
 * Tick counter from main program
 *
 * \returns int status
 * Status of the transmission process. Negative on error.
 */
extern int periodic(int tick)
{

  double speedbase = 0, spdr = 0, spdl = 0;
  double spdFwd = 0, spdLR = 1, spdSpin = 0, spdUpDown = 0; // UAV
  double joyOmega = 0.0;
  double fastScale=ctrlConfig.slowFactor;
  int i;
  //Detect manual override toggling
  if (((joyValues.button[7] == 1) || (joyValues.button[8] == 1)) && (!toggleState)) 
  {
    if (manOverride) 
      manOverride = 0;
    else
      manOverride = 1;
    toggleState = 1; //Set state for toggleing
  } else if (!(joyValues.button[7] || joyValues.button[8])) {
    toggleState = 0;
  }
  if (joyValues.button[ctrlConfig.fastBut])
    fastScale = 1.0;
  //Goto manual control, if manual override or buttons are pressed
  if ((manOverride) || (joyValues.button[0])|| (joyValues.button[1])|| (joyValues.button[2])||
  (joyValues.button[3])|| (joyValues.button[4])) {
    // (seSpin == -1) Not for UAV

    //Control by left stick and forward - left or right "speed" button on gamepad-F710
    if (((joyValues.axes[2] > -32767) || (joyValues.axes[5] > -32767)) && (seUpDown == -1)) { //Left stick and going backwards
      // (seSpin == -1) Not for UAV
      
      if(joyValues.axes[2] > -32767) {
              speedbase = - ((32767.0 + joyValues.axes[2])/65534.0)*ctrlConfig.maxfwd;
      } else {
              speedbase =   ((32767.0 + joyValues.axes[5])/65534.0)*ctrlConfig.maxfwd;
      }

      //Turning?
      if (joyValues.axes[0] < ctrlConfig.deadmin) {
                spdl = speedbase - fabs((double)joyValues.axes[0]/32000.0)* speedbase* 2;
                spdr = speedbase;
      } else if (joyValues.axes[0] > ctrlConfig.deadmax) {
                spdl = speedbase;
                spdr = speedbase - fabs((double)joyValues.axes[0]/32000.0)* speedbase* 2;
      } else {
              spdr = speedbase;
              spdl = speedbase;
      }
      if (abs(joyValues.axes[0]) > ctrlConfig.deadmax)
        // left stick
        joyOmega = (double)joyValues.axes[0] / 32000.0 * ctrlConfig.maxturn;


    } else if (((fabs(joyValues.axes[4]) > ctrlConfig.deadmax) || (fabs(joyValues.axes[3]) > ctrlConfig.deadmax)) && (seUpDown == -1)) { //Right stick control
      // (seSpin == -1) Not for UAV
      
      if(joyValues.axes[4] < 0) { //Forward or backwards (axis are negative on forward)
              speedbase = fabs((joyValues.axes[4]+ctrlConfig.deadmax)/(32767.0 + ctrlConfig.deadmin)) * ctrlConfig.maxfwd;
      } else {
              speedbase = - fabs((joyValues.axes[4]+ctrlConfig.deadmax)/(32767.0 + ctrlConfig.deadmin)) * ctrlConfig.maxfwd;
      }

      //Turning?
      if (joyValues.axes[3] < ctrlConfig.deadmin) { // left turn
              spdl = speedbase - fabs((double)joyValues.axes[3]/32000.0)* ctrlConfig.maxturn;
              spdr = speedbase + fabs((double)joyValues.axes[3]/32000.0)* ctrlConfig.maxturn;
      } else if (joyValues.axes[3] > ctrlConfig.deadmax) { // right turn
              spdl = speedbase + fabs((double)joyValues.axes[3]/32000.0)* ctrlConfig.maxturn;
              spdr = speedbase - fabs((double)joyValues.axes[3]/32000.0)* ctrlConfig.maxturn;
      } else {
                spdr = speedbase;
                spdl = speedbase;
              }

    } else if ((manOverride) && (seUpDown != -1) ) {          
              // For UAV control
      // Axis: 0 Left/Right
      // Axis: 1 Front/back
      // Axis: 2 LT 
      // Axis: 3 Spin
      // Axis: 4 Up/Down
      // Axis: 5 RT
      // Axis: 6 Not used
      // Axis: 7 Not used
      // Buttom: 0A
      // Buttom: 1B
      // Buttom: 2X
      // Buttom: 3Y
      // Buttom: 4LB
      // Buttom: 5RB
      // Butotm: 6Back
      // Buttom: 7Start
      
      
      
      // Left/Right
      if ((joyValues.axes[0] < ctrlConfig.deadmin) || (joyValues.axes[0] > ctrlConfig.deadmax)){ // If value is out of deadband
	spdLR = (joyValues.axes[0]/32767.0)*ctrlConfig.maxlr;
      }
      
      // Fwd / Back
      if ((joyValues.axes[1] < ctrlConfig.deadmin) || (joyValues.axes[1] > ctrlConfig.deadmax)){ // If value is out of deadband
	spdFwd = (joyValues.axes[1]/32767.0)*ctrlConfig.maxfwd;
      }
      
      // Spin
      if ((joyValues.axes[3] < ctrlConfig.deadmin) || (joyValues.axes[3] > ctrlConfig.deadmax)){ // If value is out of deadband
	spdSpin = (joyValues.axes[3]/32767.0)*ctrlConfig.maxspin;
      }
      
      // Up/Down
      if ((joyValues.axes[4] < ctrlConfig.deadmin) || (joyValues.axes[4] > ctrlConfig.deadmax)){ // If value is out of deadband
	spdUpDown = (joyValues.axes[4]/32767.0)*ctrlConfig.maxupdown;
      }
              
    } else { //Stop
        spdl = 0;
        spdr = 0;
	spdLR = 1;
	spdFwd = 0;
	spdSpin = 0;
	spdUpDown = 0;
    }
    if (roundi(joyOmega) == 0 && abs(joyValues.axes[3]) > ctrlConfig.deadmax)
      // right stick (if left stick is 0)
      joyOmega = (double)joyValues.axes[3] / 32000.0 * ctrlConfig.maxturn;

    //Has control override been enabled
    if (ctrlConfig.control == 1) {
      
      symTableElement *st = getSymtable('w');
      if (seSpeedr != -1)
      { // differential drive variables available
        //Scale the control variables
        spdr *= ctrlConfig.scaleright;
        spdl *= ctrlConfig.scaleleft;
        //Write the values back to robot speed values
        /*
        seSpeedr->data[0] = (int32_t)roundi(spdr * fastScale);
        seSpeedr->updated = 1;
        seSpeedl->data[0] = (int32_t)roundi(spdl * fastScale);
        seSpeedl->updated = 1;
        */
        
        st[seSpeedl].data[0] = (int32_t)roundi(spdl * fastScale);
        st[seSpeedl].updated = 1;
        st[seSpeedr].data[0] = (int32_t)roundi(spdr * fastScale);
        st[seSpeedr].updated = 1;
                
        /*
        printf("  joyctrl: sespeedr = %d", seSpeedl->data[0]);
        printf("  joyctrl: pointer = %x\n", seSpeedl);
        symTableElement *st = getSymtable('w');
			
	printf("  joyctrl: pointer = %x  sespeed pointer = %x\n", st->data, seSpeedl->data);
	*/
        
      }
      if (seVel != -1)
      { // vel-omega variables available
        /*seVel->data[0] = (int32_t)roundi(speedbase * fastScale);
        seVel->updated = 1;
        seOmega->data[0] = (int32_t)roundi(-joyOmega);
        seOmega->updated = 1;
        */
        
        st[seVel].data[0] = (int32_t)roundi(speedbase * fastScale);
        st[seVel].updated = 1;
        st[seOmega].data[0] = (int32_t)roundi(-joyOmega);
        st[seOmega].updated = 1;
        
        // debug
        //if (tick % 30 == 0)
        //  printf("vel-omega-RC: vel=%d omega=%d\n", (int32_t)roundi(speedbase), (int32_t)roundi(joyOmega));
        // debug end
      }
      if (seSpeedRef != -1)
      { // Ackerman  variables available
        /*
        seSpeedRef->data[0] = (int32_t)roundi(speedbase * fastScale);
        seSpeedRef->updated = 1;
        seAngleRef->data[0] = (int32_t)roundi(-joyOmega);
        seAngleRef->updated = 1;
        */
        
        st[seSpeedRef].data[0] = (int32_t)roundi(speedbase * fastScale);
        st[seSpeedRef].updated = 1;
        st[seAngleRef].data[0] = (int32_t)roundi(-joyOmega);
        st[seAngleRef].updated = 1;
      }
      
      // UAV
      if(seUpDown != -1){
	//printf("UAV data set\n");
	//st[seLR].data[0] = (int32_t) spdLR;
	//st[seLR].updated = 1;
	
	//st[seFwd].data[0] = (int32_t) spdFwd;
	//st[seFwd].updated = 1;
	
	//st[seSpin].data[0] = (int32_t) spdSpin;
	//st[seSpin].updated = 1;
	
	//st[seUpDown].data[0] = (int32_t) roundi(spdUpDown);
	st[seUpDown].data[0] = (int32_t) 1000;
	st[seUpDown].updated = 1;
	
      }
      
      //Using manual control
      setVariable(iOverride,0,1);
    }

  } else {
          //Not using manual control
          setVariable(iOverride,0,0);
  }
  
  if (tick <= 1)
  {
    for (i = 0; i < routesCnt; i++)
      routes[i].writeVar = getDatabaseVariable('w', routes[i].varName);
  }
  
  else
  {
    symTableElement *st = getSymtable('w');
    for (i = 0; i < routesCnt; i++)
    {
      RouteConfig * ro = &routes[i];
      int vn = ro->writeVar;
      if (vn >= 0)
      {
        float v;
        int isOK = 1;
        if (ro->axis >= 0 && ro->axis < number_of_axes)
          v = (joyValues.axes[ro->axis] - ro->offset) * ro->scale;
        else if (ro->button >= 0 && ro->button < number_of_buttons)
          v = (joyValues.axes[ro->axis] - ro->offset) * ro->scale;
        else
          isOK = 0;
        if (isOK && ro->writeIdx > 0 && ro->writeIdx < st[vn].length)
        { // write to write variable
          st[vn].data[ro->writeIdx] = roundi(v);
          st[vn].updated = 1;
        }
      }
    }
  }

  return 1;
}


/** \brief Recieve thread for rFLEX communication
 *
 * \returns * void
 * Unused
 */
void *rxtask(void *something) {

    	int bytes;
        struct js_event jse;

        joyRunning = 1; //Thread has now started

        while (joyRunning > 0) {
            //Read a joystick package from dev
            bytes = read(jDev, &jse, sizeof(jse));

            if (bytes == -1) {
                joyRunning = -1;
                printf("   JoyControl: Failed reading from joystick - Exiting!\n");
                break;
            } else if (bytes != sizeof(jse)) {
                printf("   JoyControl: Unexpected bytes from joystick:%d\n", bytes);

            } else { //Proper joystick package has been recieved

                //Joystick package parser
                jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
                switch(jse.type) {
                    case JS_EVENT_AXIS:
                        if (jse.number < 16) {
                            joyValues.axes[jse.number] = jse.value;
                            setVariable(iAxes,jse.number,jse.value);
                        }
                        break;

                    case JS_EVENT_BUTTON:
                        if (jse.number < 16) {
                            joyValues.button[jse.number] = jse.value;
                            setVariable(iButtons,jse.number,jse.value);
                        }
                        break;
                    default:
                        break;
                }
            }
        }
	return something;
}

/** \brief Shut down rFLEX driver
 *
 * \returns int success
 *  Checksum value
 */
extern int terminate(void) {

	//rFLEX commands to shut down robot!
	if (joyRunning > 0) {
            joyRunning = -1;
	    if (jDev) close(jDev);
            //Wait for thread to close
            if (joyRunning > -1)	usleep(200000); //Wait 500 ms
	}

	return 1;
}


/************************** XML Initialization **************************/
///Struct for shared parse data
typedef struct  {
    int depth;
    char skip;
    char enable;
		char found;
  }parseInfo;

//Parsing functions
void XMLCALL startTag(void *, const char *, const char **);
void XMLCALL endTag(void *, const char *);

/** \brief Initialize the SMRD HAL
 *
 * Reads the XML file and sets up the SMRD settings
 *
 * Finally the rx threads is started and the driver
 * is ready to read data
 *
 * \param[in] *char filename
 * Filename of the XML file
 *
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
extern int initXML(char *filename) {

  parseInfo xmlParse;
  char *xmlBuf = NULL;
	int xmlFilelength;
  int done = 0;
  int len;
  FILE *fp;

  //Print initialization message
  //Find revision number from SVN Revision
  char *i,versionString[20] = REVISION, tempString[10];
  i = strrchr(versionString,'$');
  strncpy(tempString,versionString+6,(i-versionString-6));
  tempString[(i-versionString-6)] = 0;
  printf("JoyControl: Initializing Joystick Control Plug-in %s.%s\n",JOYCONTROLVERSION,tempString);
  ctrlConfig.control = 0;

   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "   Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, startTag, endTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("   Error reading: %s\n",filename);
    return -1;
  }
  //Get the length of the file
	fseek(fp,0,SEEK_END);
	xmlFilelength = ftell(fp); //Get position
	fseek(fp,0,SEEK_SET); //Return to start of file

	//Allocate text buffer
	xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
	if (xmlBuf == NULL) {
		fprintf(stderr, "   Couldn't allocate memory for XML File buffer\n");
		return -1;
	}
	memset(xmlBuf,0,xmlFilelength);
  len = fread(xmlBuf, 1, xmlFilelength, fp);
  fclose(fp);

	//Clear the configuration struct
	memset(&ctrlConfig,0,sizeof(ctrlConfig));
	ctrlConfig.scaleleft = 1;
	ctrlConfig.scaleright = 1;
        ctrlConfig.fastBut = 5;
        ctrlConfig.slowFactor = 1.0;

  //Start parsing the XML file
  if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) {
    fprintf(stderr, "   XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <joycontrol> XML tag found in plugins section\n");
		return -1;
	}

  //Initialize JoyControl, if XML parsed properly
  if (xmlParse.enable) done = initJoyCtrl();



 return done;
}

void XMLCALL
startTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("joycontrol",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("joycontrol",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1;
    }
    if (!info->enable) {
      printf("   Use of JoyControl disabled in configuration\n");
      info->skip = info->depth;
    }
  } else if (strcmp("joystick",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(joyDev,attr[i+1],63);
    printf("   Using Joystick port: %s\n",joyDev);
  } else if (strcmp("speed",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("maxfwd",attr[i]) == 0)  ctrlConfig.maxfwd = atoi(attr[i+1]);
    for(i = 0; attr[i]; i+=2) if (strcmp("maxturn",attr[i]) == 0) ctrlConfig.maxturn = atoi(attr[i+1]);
    for(i = 0; attr[i]; i+=2) if (strcmp("maxlr",attr[i]) == 0) ctrlConfig.maxlr = atoi(attr[i+1]);
    for(i = 0; attr[i]; i+=2) if (strcmp("maxupdown",attr[i]) == 0) ctrlConfig.maxupdown = atoi(attr[i+1]);
    for(i = 0; attr[i]; i+=2) if (strcmp("maxspin",attr[i]) == 0) ctrlConfig.maxspin = atoi(attr[i+1]);
    for(i = 0; attr[i]; i+=2) if (strcmp("fastBut",attr[i]) == 0) ctrlConfig.fastBut = atoi(attr[i+1]);
    for(i = 0; attr[i]; i+=2) if (strcmp("slowFactor",attr[i]) == 0) ctrlConfig.slowFactor = strtod(attr[i+1], NULL);

    printf("   Control speed MaxFwd: %d MaxTurn: %d\n",ctrlConfig.maxfwd,ctrlConfig.maxturn);
  } else if (strcmp("deadband",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("min",attr[i]) == 0) ctrlConfig.deadmin = atoi(attr[i+1]);
    for(i = 0; attr[i]; i+=2) if (strcmp("max",attr[i]) == 0) ctrlConfig.deadmax = atoi(attr[i+1]);

    printf("   Deadband max: %d min: %d\n",ctrlConfig.deadmin, ctrlConfig.deadmax);
  } else if (strcmp("control",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i] != NULL; i+=2) {
      if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0))  {
          ctrlConfig.control = 1;
          break;
      }
    }
  } else if (strcmp("scale",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("left",attr[i]) == 0) ctrlConfig.scaleleft  = strtod(attr[i+1],NULL);
    for(i = 0; attr[i]; i+=2) if (strcmp("right",attr[i]) == 0) ctrlConfig.scaleright = strtod(attr[i+1],NULL);
    printf("   Joystick correction scale set: right = %4.3f left = %4.3f\n",
           ctrlConfig.scaleright,ctrlConfig.scaleleft);
  } else if (strcmp("joyroute",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    else if (routesCnt < ROUTES_MAX)
    {
      routes[routesCnt].axis = -1;
      routes[routesCnt].button = -1;
      routes[routesCnt].offset = 0;
      routes[routesCnt].scale = 1.0;
      routes[routesCnt].varName[0] = '\0';
      routes[routesCnt].writeIdx = 0;
      routes[routesCnt].writeVar = -1;
      for(i = 0; attr[i]; i += 2)
      {
        if (strcmp("axis",attr[i]) == 0)
          routes[routesCnt].axis  = strtol(attr[i+1],NULL, 10);
        else if (strcmp("button",attr[i]) == 0)
          routes[routesCnt].button  = strtol(attr[i+1],NULL, 10);
        else if (strcmp("offset",attr[i]) == 0)
          routes[routesCnt].offset  = strtol(attr[i+1],NULL, 10);
        else if (strcmp("scale",attr[i]) == 0)
          routes[routesCnt].scale  = strtod(attr[i+1],NULL);
        else if (strcmp("dest",attr[i]) == 0)
          strncpy(routes[routesCnt].varName, attr[i+1], 32);
        else if (strcasecmp("destIdx",attr[i]) == 0)
          routes[routesCnt].writeIdx  = strtol(attr[i+1],NULL, 10);
      }
      routesCnt++;
    }
  }
}

void XMLCALL
endTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
