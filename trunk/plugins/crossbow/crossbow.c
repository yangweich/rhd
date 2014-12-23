/** \file crossbow.c
 *  \ingroup hwmodule
 *  \brief Hardware abstraction layer for Crossbow IMU
 *
 *  This liberary implements the hardware abstraction layer
 *  for the nice and expensive Crossbow IMU
 *  
 *  Based on code from hakod, written by 
 *  Anders Reeske Nilsen and Asbjørn Mejnertsen in 2006
 *
 *  \author Anders Billesø Beck
 *  $Rev: 59 $
 *  $Date: 2012-01-14 09:49:21 +0100 (Sat, 14 Jan 2012) $
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
#define CROSSBOWVERSION 	"1.0"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2012-01-14 09:49:21 +0100 (Sat, 14 Jan 2012) $:"
 #define ID               "$Id: crossbow.c 59 2012-10-21 06:25:02Z jcan $"
/***************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <expat.h>
#include <poll.h>

#include "database.h"
#include "crossbow.h"
#include "globalfunc.h"

// Threads are being defined
pthread_t xbow_thread;
pthread_attr_t attr;

///Number of bytes received from the XBOW in every communication
#define XBOWSTRINGSIZE 18
///Number of times to PING XBOW before giving up and setting xbow_online flag to zero.
#define XBOW_PING 20

///Receive buffer for XBOW communication
unsigned char xbowrxbuf[XBOWSTRINGSIZE];

///Flag indicating that the xbowtxbuf is in use and should not be accessed by other threads
int xbowrx_flag; 
///Flag indicating that the XBOW is connected and online
int xbow_online;

///XBOW RS232 port pointer
int xbow_dev;
///XBOW RS232 port string identifier (loaded by XML)
char xbowDevString[64];

int xbow_rx(void);
int xbow_init(void);
void *xbow_task(void *);
int startCrossbow(void);


//Variable indexes in variable database
int iRoll, iPitch, iYaw, iaX, iaY, iaZ, iTemp, iTime;

int startCrossbow(void) {
  
  //initialization of flags
  xbowrx_flag = 0;

  if((xbow_online = xbow_init())<0) {
    printf("   Crossbow: Not able to initialize crossbow IMU\n");
    return -1;
  }

  //If initialization is done, create variables
  iRoll  = createVariable('r',1,"XbowRoll");
  iPitch = createVariable('r',1,"XbowPitch");
  iYaw   = createVariable('r',1,"XbowYaw");
  iaX    = createVariable('r',1,"XbowX");
  iaY    = createVariable('r',1,"XbowY");
  iaZ    = createVariable('r',1,"XbowZ");
  iTemp  = createVariable('r',1,"XbowTemp");
  iTime  = createVariable('r',1,"XbowTime");


// Initialization and starting of threads
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

// Start XBOW thread
// The XBOW thread receives all incomming data from the XBOW
  if(xbow_online)
  {
    if (pthread_create(&xbow_thread, &attr, xbow_task, 0))
    {
      perror("Crossbow: can't start XBOW thread");
      return -1;
    }
  }

  return 1;  
}

///Initialization of the RS232 XBOW port and rx buffer
int xbow_init(void)
{
  char cmd,rsp = 0;
  int i/*,ret*/;
  struct pollfd pollData;



// Open port NONBLOCKING for initializing without freeze
  xbow_dev = open(xbowDevString, O_RDWR| O_NONBLOCK);
  if (xbow_dev<0) 
  {
    fprintf(stderr,"   Crossbow: Error opening %s\n",xbowDevString);
    return -1;
  }
// Set baudrate for XBOW
  if (set_serial(xbow_dev,38400) == -1)
  {
    fprintf(stderr,"   Crossbow: Can't XBOW serial port parameters\n");
      return -1;
  }  

  pollData.events = POLLIN; //Check for data in rx buffer
  pollData.fd = xbow_dev;
  
// Check if XBOW is connected and responding 
  for(i=0; rsp != 'H' && i < XBOW_PING; i++)
  {
// Send command to enter polled mode (in case XBOW is in continues mode)
    cmd = 'P';
    secureWrite(xbow_dev, &cmd, 1);
    //Empty rx buffer before pinging crossbow
	 //char c;
    while(poll(&pollData,1,0))
      /*c =*/ read(xbow_dev, &rsp, 1);
// Send ping command
    cmd = 'R';
    secureWrite(xbow_dev, &cmd, 1);

		//Wait for response, kill if no response is there
		if (poll(&pollData,1,100)) { //Poll for 100 ms
		// Get response. If response is 'H' everything is OK
			/*ret=*/read(xbow_dev, &rsp, 1);
		}
  }
// If unable to get valid response assume XBOW not connected
  if (i > XBOW_PING-1) {
    printf("   Crossbow: No Crossbow IMU4000 detected on %s\n",xbowDevString);
    return -1;
  } else {
    //Empty read buffer first
    while(poll(&pollData,1,0)) secureRead(xbow_dev, &rsp, 1);
    // XBOW connected and enter scaled and continues mode
    xbow_online = 1;
    cmd='c';
    secureWrite(xbow_dev, &cmd, 1);
    secureRead(xbow_dev, &rsp, 1);
    if(rsp != 'C')
    {
      printf("   Crossbow: Error setting Scaled mode\n");
      xbow_online = 0;
      return -1;
    }
    cmd='C';
    secureWrite(xbow_dev, &cmd, 1); //Setting Continues mode
  }

	//Close serial port and re-open in blocking mode
  //This is a sad hack, i know...
	close(xbow_dev);
  xbow_dev = open(xbowDevString, O_RDWR);
  if (xbow_dev<0) 
  {
    fprintf(stderr,"   Crossbow: Error opening %s\n",xbowDevString);
    return -1;
  }
// Set baudrate for XBOW
  if (set_serial(xbow_dev,38400) == -1)
  {
    fprintf(stderr,"   Crossbow: Can't XBOW serial port parameters\n");
      return -1;
  }  

  printf("   Crossbow: IMU400 Initialized and running Continous mode\n");
  return 1;
}

/// The XBOW thread receives all incomming data from the XBOW.
/// The xbow_rx function is called in an "infinate" loop.
/// One XBOW datagram is received in every loop.
void *xbow_task(void *not_used)
{

  printf("   Crossbow: Rx thread started\n");
  unsigned char buf[XBOWSTRINGSIZE];
  unsigned char tmp = 0;
  int n,i;
  
  while(1) {
  n=0;
  memset(buf,0,XBOWSTRINGSIZE);
  while(n < XBOWSTRINGSIZE)
  {
    if(secureRead(xbow_dev,&tmp,1)>0) {
        buf[n]=tmp;
        n++;
    } //else usleep(10);
  // Check for communication header
    if(buf[0] != 0xFF)  n=0;
  }
    tmp = 0;
  // Verify checksum of communication
    for(i=1; i< XBOWSTRINGSIZE-1; i++)
      tmp += buf[i];
    if(tmp == buf[XBOWSTRINGSIZE-1]) {
  
      //Move data to variable database
      setVariable(iRoll , 0, (int16_t)(buf[1] << 8) + buf[2]);
      setVariable(iPitch, 0, (int16_t)(buf[3] << 8) + buf[4]);
      setVariable(iYaw  , 0, (int16_t)(buf[5] << 8) + buf[6]);
      setVariable(iaX   , 0, (int16_t)(buf[7] << 8) + buf[8]);
      setVariable(iaY   , 0, (int16_t)(buf[9] << 8) + buf[10]);
      setVariable(iaZ   , 0, (int16_t)(buf[11] << 8) + buf[12]);
      setVariable(iTemp , 0, (int16_t)(buf[13] << 8) + buf[14]);
      setVariable(iTime , 0, (int16_t)(buf[15] << 8) + buf[16]);
    }
    else
        fprintf(stderr,"Checksum error in Xbow communication\n");
  }
  fprintf(stderr,"Crossbow: Rx task terminated\n");
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
void XMLCALL crossbowStartTag(void *, const char *, const char **);
void XMLCALL crossbowEndTag(void *, const char *);


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
  printf("Crossbow: Initializing Crossbow IMU HAL %s.%s\n",CROSSBOWVERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "Crossbow: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, crossbowStartTag, crossbowEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("Crossbow: Error reading: %s\n",filename);
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
    fprintf(stderr, "Crossbow: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);
	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <crossbow> XML tag found in plugins section\n");
		return -1;
	}

  //Start crossbow thread after init
  if (xmlParse.enable) done = startCrossbow();


 return done;
}

///Handle XML Start tags
void XMLCALL
crossbowStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("crossbow",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("crossbow",el)) {
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   Crossbow: Use of Crossbow disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(xbowDevString,attr[i+1],63); 
    printf("   Crossbow: Using serialport %s\n",xbowDevString);
  } 

}

///Handle XML End tags
void XMLCALL
crossbowEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
