/** \file fogyro.c
 *  \ingroup hwmodule
 *  \brief Hardware abstraction layer for FOG IMU
 *
 *  This liberary implements the hardware abstraction layer
 *  for the nice and expensive Fibre Optic Gyro
 *  
 *  Based on code from hakod, written by 
 *  Anders Reeske Nilsen and Asbjørn Mejnertsen in 2006
 *
 *  \author Anders Billesø Beck
 *  $Rev: 59 $
 *  $Date: 2013-10-30 13:58:13 +0100 (Wed, 30 Oct 2013) $
 *
 */
 /**************************************************************************
 *                  Copyright 2008 Anders Billesø Beck DTU                 *
 *                       anders.beck@get2net.dk                            *
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
/***************************** Plugin version  *****************************/
#define FOGYROVERSION 	  "1.0"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2013-10-30 13:58:13 +0100 (Wed, 30 Oct 2013) $:"
 #define ID               "$Id: fogyro.c 59 2012-10-21 06:25:02Z jcan $"
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
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <linux/serial.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <expat.h>
#include <poll.h>

#include <database.h>
#include <globalfunc.h>
#include <time.h>
#include "fogyro.h"


// Threads are being defined
pthread_t fog_thread;
pthread_attr_t attr;

///Number of bytes received from the FUG IMU in every communication
#define FOGSTRINGSIZE 32

///FOG RS422 data port pointer
int fogDataDev;
///FOG RS422  data port string identifier (loaded by XML)
char fogDataDevString[64];
///Baudrate of RS422 serial DATA connection
int dataBaudrate;
///FOG RS422 config port pointer
int fogConfigDev;
///FOG RS422  config port string identifier (loaded by XML)
char fogConfigDevString[64];
///Baudrate of RS422 serial CONFIG connection
int configBaudrate;
///Sampletime of the FOG
int sampletime;

//Function prototypes
int fog_rx(void);
int fog_init(void);
void *fog_task(void *);
int fog_set_serial(int,int);
int startFOG(void);


//Variable indexes in variable database
int iPhX, iPhY, iPhdZ, iTempB, iTempE, iFailStatus, iGyroStatus;

///Initialize FOG and spawn RX thread
int startFog(void) 
{
  
	if(fog_init()<0) 
	{
		printf("   FOG: Not able to initialize FOG IMU\n");
		return -1;
	}

	//If initialization is done, create variables
	iPhX         = createVariable('r',1,"fogPhX");
	iPhY         = createVariable('r',1,"fogPhY");
	iPhdZ        = createVariable('r',1,"fogPhdZ");
	iTempB       = createVariable('r',1,"fogTempB");
	iTempE       = createVariable('r',1,"fogTempE");
	iFailStatus  = createVariable('r',1,"fogFailStatus");
	iGyroStatus  = createVariable('r',1,"fogGyroStatus");

	// Initialization and starting of threads
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	// Start XBOW thread
	// The XBOW thread receives all incomming data from the XBOW
	if (pthread_create(&fog_thread, &attr, fog_task, 0)) 
	{
		perror("FOG: can't start FOG tx thread");
		return -1;
	}

	return 1;  
}

///Initialization of the RS422 FOGIMU port and rx buffer
int fog_init(void)
{
  char confBuf[32];

	//Configure the FOG
	// Open configuration port
	fogConfigDev = open(fogConfigDevString, O_RDWR);
	if (fogConfigDev<0) 
	{
		fprintf(stderr,"   FOG: Error opening data serial %s\n",fogConfigDevString);
		return -1;
	}
	// Set baudrate configuration port
	if (set_serial(fogConfigDev,configBaudrate) == -1) 
	{
		fprintf(stderr,"   FOG: Can't set FOG data serial port parameters\n");
		return -1;
	}
  
  //Limit sampletime
  if (sampletime > 0xFFFF) sampletime = 0xFFFF;
  else if  (sampletime <= 0) sampletime = 1;

  //Create sampletime configstring
  sprintf(confBuf,"JF=%04X\r",sampletime);

  //Write configuration (twice just to be sure)
	int i;
	for(i = 0; i < 2; i++) 
	{
		if (secureWrite(fogConfigDev, confBuf, 8) <= 0) 
		{
			fprintf(stderr,"   FOG: Error writing configuration\n");
			return -1;
		}
		usleep(10000); //8,3ms to send 8byte@9600, we wait 10ms
	}
	close(fogConfigDev);

//Launch data rx
// Open data port
	fogDataDev = open(fogDataDevString, O_RDWR);
	if (fogDataDev<0) 
	{
		fprintf(stderr,"   FOG: Error opening data serial %s\n",fogDataDevString);
		return -1;
	}
// Set baudrate for data port
	if (set_serial(fogDataDev,dataBaudrate) == -1)
	{
		fprintf(stderr,"   FOG: Can't set FOG data serial port parameters\n");
		return -1;
	}  
	return 1;
}

/// The fog thread receives all incomming data from the FOG.
void *fog_task(void *not_used)
{
	//Lock memory - No more allocations
	if (mlockall(MCL_CURRENT | MCL_FUTURE))
	{
		perror("mlockall");
		exit(-1);
	}
	
	printf("   FOG: Rx thread started\n");
	unsigned char buf[FOGSTRINGSIZE];
	unsigned char tmp = 0;
	int n;
	
	while(1) 
	{
		n=0;
		memset(buf,0,FOGSTRINGSIZE);
		while(n < FOGSTRINGSIZE)
		{
			if(secureRead(fogDataDev,&tmp,1)>0) 
			{		
				buf[n]=tmp;
				n++;
			} 
			else 
			{
				printf("Error read");
			}
			//break; //Error reading
			// Check for sync header
			if(n == 2) 
			{
				if ((buf[1] != 0xBA) && (buf[0] != 0xD9)) 
				{
					buf[0] = buf[1];
					n = 1;
				} 
			}
		}	
		setVariable(iPhX	, 0, (buf[5] << 24) + (buf[4] << 16) + (buf[3] << 8) + (buf[2]));
		setVariable(iPhY	, 0, (buf[9] << 24) + (buf[8] << 16) + (buf[7] << 8) + (buf[6]));
		setVariable(iPhdZ  	, 0, (buf[13] << 24) + (buf[12] << 16) + (buf[11] << 8) +(buf[10]));
		setVariable(iTempB  , 0, ((buf[15] << 8) + (buf[14])));
		setVariable(iTempE  , 0, ((buf[17] << 8) + (buf[16])));
		setVariable(iFailStatus , 0, ((int)buf[21] << 8) + buf[20]);
		setVariable(iGyroStatus , 0, ((int)buf[23] << 8) + buf[22]);
	}
	
	fprintf(stderr,"FOG: Rx task terminated\n");
	pthread_exit(0);
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
void XMLCALL fogStartTag(void *, const char *, const char **);
void XMLCALL fogEndTag(void *, const char *);

/** \brief Initialize the Crossbow HAL
 *
 * Reads the XML file and sets up the Crossbow settings
 * 
 * Finally the rx thread is started and the server 
 * is ready to accept connections
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
  printf("FOG: Initializing Fibre Optic Gyro HAL %s.%s\n",FOGYROVERSION,tempString);

   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "FOG: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, fogStartTag, fogEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("FOG: Error reading: %s\n",filename);
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

  //Start parsing the XML file
  if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) {
    fprintf(stderr, "FOG: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <fogyro> XML tag found in plugins section\n");
		return -1;
	}

  //Start crossbow thread after init
  if (xmlParse.enable) done = startFog();


 return done;
}

///Handle XML Start tags
void XMLCALL
fogStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("fogyro",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("fogyro",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   FOG: Use of FOG disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("dataserial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(fogDataDevString,attr[i+1],63); 
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) dataBaudrate = atoi(attr[i+1]); 
    printf("   FOG: Data RS422 port %s at %d baud\n",fogDataDevString,dataBaudrate);
  } else if (strcmp("configserial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(fogConfigDevString,attr[i+1],63); 
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) configBaudrate = atoi(attr[i+1]); 
    printf("   FOG: Config RS422 port %s at %d baud\n",fogConfigDevString,configBaudrate);
  } else if (strcmp("sampletime",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("value",attr[i]) == 0) sampletime = atoi(attr[i+1]); 
    printf("   FOG: Using sampletime of %d ms\n",sampletime);
  }

}

///Handle XML End tags
void XMLCALL
fogEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
