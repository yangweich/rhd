/** \file cruizcore.h
 *  \ingroup hwmodule
 *  \brief Driver for the Microinfinity CruizCore Gyro module
 *
 *  This driver interfaces the Microinfinity CruizCore XG1010
 *
 *  Interface to the hardware is rather simplistic, so data is directly
 *  forwarded from the gyro and only calibration reset is avaliable in
 *  the opposite direction
 *
 *  \author Anders Billesø Beck
 *  $Rev: 481 $
 *  $Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $
 *
 */
/***************************************************************************
 *                  Copyright 2010 Anders Billesø Beck, DTU / DTI          *
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
#define VERSION   	      "1.0"
/************************** Version control information ***************************/
 #define REVISION         "$Rev: 481 $:"
 #define DATE             "$Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $:"
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

//RHD Core headers
#include <rhd.h>
#include <database.h>
#include <globalfunc.h>

#include "cruizcore.h"

/******** Global variables *************/
int iReset, iRate, iPos;
int sDev = 0;  //File descriptors
char serialDev[64];
int baudrate;
volatile int cruizRunning = -1;
pthread_t rxThread;
pthread_attr_t attr;

const char debug = 0;



/******** Function prototypes *************/
int initCruiz(void);
void *rxtask(void *);



/** \brief Initialize communications and variables
 * 
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
int initCruiz(void) {


	//Open first serial port
  if ((sDev = open (serialDev, O_RDWR /*| O_NONBLOCK*/)) == -1) {
		fprintf(stderr,"   Can't open serial port: %s\n",serialDev);
		cruizRunning = -1;
		return -1;
  } else if (set_serial(sDev,baudrate) == -1) {
		fprintf(stderr,"   Can't set serial port parameters\n");
		fprintf(stderr,"   CruizCore is NOT running!\n");
		cruizRunning = -1;
		return -1;
  } else cruizRunning = 2;
 
	/****** Create database variables if all is ok **************/
   iRate 		= createVariable('r',1,"cruizrate");
	iPos 			= createVariable('r',1,"cruizpos");
	iReset      = createVariable('w',1,"cruizreset");

	//Start RX thread
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED); 
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  if (pthread_create(&rxThread, &attr, rxtask, 0))
    {
      perror("   Can't start CruizCore receive thread");
      cruizRunning = -1;
      return -1;
    }

	while (cruizRunning > 1) usleep(1000); //Don't proceed before threads are running

  return 1;
}


/** \brief Transmit data to rflex serial bus
 * 
 *  * \param[in] int tick
 * Tick counter from main program
 * 
 * \returns int status
 * Status of the transmission process. Negative on error.
 */
extern int periodic(int tick)
{

		//Reset module if requested
		if (isUpdated('w',iReset)) {
			//Send software reset to CruizCore
			if (secureWrite(sDev,"$MIB,RESET*87",strlen("$MIB,RESET*87")) < 0) {
				cruizRunning = -1;
			}
		}

  return 1;
}


/** \brief Recieve thread for CruizCore communication
 * 
 * \returns * void
 * Unused
 */
void *rxtask(void *something) {

	//Set running back to 1 to indicate thread start
	enum {GET_FULL_PACKAGE, GET_SINGLE_BYTE} rxState = GET_FULL_PACKAGE;
	unsigned char buf[8], tmp, i = 0;
	int16_t header, rate, pos, checksum, calcCheck;
	
	cruizRunning = 1;
	printf("  CruizCore: RX Thread started\n");

	//Send software reset to CruizCore
	if (secureWrite(sDev,"$MIB,RESET*87",strlen("$MIB,RESET*87")) < 0) {
		cruizRunning = -1;
	}

	//Everloop that recieves 
	while (cruizRunning > 0) {
		//Recieve state machine
		//Makes it possible to recie a full package or one byte at a time
		//to regain sync if it has been lost
		switch (rxState) {
			//Recieve all 8 bytes when sync is correct
			case GET_FULL_PACKAGE:
				if(secureRead(sDev,buf,8) < 0) {
					cruizRunning = -1; //Shutdown for RX error
				}
				break;
			//If reception of a full package fails, then get single bytes until synchronization
			case GET_SINGLE_BYTE:
				if(secureRead(sDev,&tmp,1) < 0) {
					cruizRunning = -1; //Shutdown for RX error
				}
				//Move all buffer data one byte forward
				for (i = 0; i < 7; i++) buf[i] = buf[i+1];
				buf[7] = tmp; //Place new data in the end
				break;
			//Default behavior
			default:
				rxState = GET_SINGLE_BYTE;
				break;
		}
		
		//Now parse the package and check if it is valid
		header =		((buf[1] << 8) + buf[0]);
		rate =		((buf[3] << 8) + buf[2]);
		pos  =		((buf[5] << 8) + buf[4]);
		checksum =	((buf[7] << 8) + buf[6]);
		calcCheck = (int16_t)(0xFFFF + rate + pos);
		
		if (debug) printf("Data package 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n",(uint8_t)buf[0],(uint8_t)buf[1],(uint8_t)buf[2],(uint8_t)buf[3],(uint8_t)buf[4],(uint8_t)buf[5],(uint8_t)buf[6],(uint8_t)buf[7]);

		//Check header and checksum and save valid data
		if ((buf[0] == (uint8_t)0xFF) && (buf[1] == (uint8_t)0xFF) && (calcCheck == checksum)) {
			setVariable(iRate , 0, rate);
			setVariable(iPos , 0, pos);
			rxState = GET_FULL_PACKAGE;
			if (debug) printf("Check OK r=%d,p=%d  (%d == %d)\n",rate,pos,checksum,calcCheck);
		} else {
			if (debug) printf("Check FAILED!\nRate %d - Pos %d\n",rate,pos);
			if (debug) printf("Checksum %d != %d\n",checksum,calcCheck);
			//Go to single byte mode to regain sync
			rxState = GET_SINGLE_BYTE;
		}

		if (debug) printf("-------------------------------\n");
           usleep(2000);
	}//End of rxLoop

	//Shut down CruizCore, if recieve fails
	cruizRunning = -2;
	fprintf(stderr,"CruizCore: RX thread terminated\n");

	return something;
}

/** \brief Shut down rFLEX driver
 * 
 * \returns int success
 *  Checksum value
 */
extern int terminate(void) {

	//rFLEX commands to shut down robot!
	if (cruizRunning > 0) {
		cruizRunning = -1;
		if (sDev) close(sDev);
		//Wait for thread to close
		while (cruizRunning > -1);	usleep(100000); //Wait 100 ms
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
  printf("CruizCore: Initializing CruizCore Gyro HAL %s.%s\n",VERSION,tempString);


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
		printf("   Error: No <cruizcore> XML tag found in plugins section\n");
		return -1;
	}

  //Initialize if XML parsed properly
  if (xmlParse.enable) done = initCruiz();



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
 				((info->depth == 3) && (strcmp("cruizcore",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("cruizcore",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   Use of CruizCore gyro disabled in configuration\n");
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(serialDev,attr[i+1],63);
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate = atoi(attr[i+1]);  
    printf("   Serial port %s at %d baud\n",serialDev,baudrate);
  }

}

void XMLCALL
endTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
