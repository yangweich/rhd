/** \file imu.c
 *  \ingroup hwmodule
 *  \brief Serial IMU Module
 *
 * IMU Module for RHD. 
 * 
 * Based on gps by Anders Billesø Beck
 * 
 * Modules supported: sparkfun Razor 9DOF IMU+magnetometer
 * 
  *
 *  \author Dan Hermann
  *  $Rev: 01 $
 *  $Date: 2014-03-03 09:49:21 +0100 (Mon, 03 Mar 2014) $
 */
 /***************************************************************************
 *                  Copyright 2014 Dan Hermann                             *
 *                       danh@elektro.dtu.dk                               *
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
#define IMUVERSION 	      "1.0"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 01 $:"
 #define DATE             "$Date: 2014-03-03 09:49:21 +0100 (Mon, 03 Mar 2014) $:"
 #define ID               "$Id: imu.c 01 2014-03-03 06:25:02Z jcan $"
/***************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <netdb.h>
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
#include <math.h>


#include <database.h>

#include "imu.h"

//Definitions

int  insDevN = 0;              ///Number of devices defined
char insDevStringDecl[64];   ///String to hold INS device(s)
int  baudrate = 0;
static volatile char running = 0;
struct imuData imu[IMU_DEV_MAX];

///INS database indexes



// Threads are being defined
pthread_t ins_thread_read;

//pthread_mutex_t decodeFlag; // pthread_mutex_lock


//Function prototypes
int initINS(void);
int set_serial(int fd, int baud);
void *ins_task_read(void *);

/** \brief Initialize IMU
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
int initINS(void) {
  int r,  d, ns, np, n;
  char *p;
  char str[DEF_STR_LEN] = {0};
  
  //Detect no of IMU's
  p = insDevStringDecl;
  insDevN = 0;
  ns = strlen(insDevStringDecl);
  np=0;
  for(d=0; d<IMU_DEV_MAX && np<ns; d++)
  {
    //Get device name
    r = sscanf(p, "%s", imu[insDevN].insDevStr);
    n = strlen(imu[insDevN].insDevStr);
    p += n;
    np+= n;
    if(r==1 && n>strlen("/dev/tty"))  //Chech that one parameter found && check path is longer than strlen("/dev/tty")
    {
      printf("   IMU: ID=%d, dev=%s\r\n", d, imu[d].insDevStr);
      insDevN++;
    } 
  }
  printf("   IMU: %d devices (max=%d)\r\n", insDevN, IMU_DEV_MAX);
  
  for(d=0; d<insDevN; d++)
  {    
    // Open RS232 port
    imu[d].insDev = open(imu[d].insDevStr, O_RDWR | O_NONBLOCK);
    if (imu[d].insDev<0) 
    {
      fprintf(stderr,"   IMU%d: Error opening %s\n", d, imu[d].insDevStr);
      return -1;
    }
    //Set baudrate for IMU
    if (set_serial(imu[d].insDev,baudrate) == -1)  
    {
      fprintf(stderr,"   IMU%d: Can't set IMU serial port parameters\n", d);
        return -1;
    }
    running = 1;    
    
    //iAttitude     = createVariable('r', 3, "IMUattitude");
    snprintf(str,DEF_STR_LEN, "IMUaccRaw%d", d);
    imu[d].iAccRaw   = createVariable('r', 3, str);
    snprintf(str,DEF_STR_LEN, "IMUgyroRaw%d", d);
    imu[d].iGyroRaw  = createVariable('r', 3, str);
    snprintf(str,DEF_STR_LEN, "IMUmagRaw%d", d);
    imu[d].iMagRaw   = createVariable('r', 3, str);
    snprintf(str,DEF_STR_LEN, "IMUconnected%d", d);
    imu[d].iConnected= createVariable('r', 1, str);
  }
  
  // Initialization and starting of threads
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  if (pthread_create(&ins_thread_read, &attr, ins_task_read, 0))
  {
    perror("   IMU: Can't start IMU read thread");
    return -1;
  }
  
  return 1;
}

/** \brief Initialize Shut down IMU rx thread
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
int shutdownINS(void) {

  running = 0;
  pthread_join(ins_thread_read,NULL);

  return 1;
}


/** \brief IMU RX thread.
 */
void *ins_task_read(void *not_used) {
  int i[IMU_DEV_MAX] = {0};
  int r, n, d;
  int dataRec;
  //int timeOut[IMU_DEV_MAX] = {0};
  //float attitude[3];  
  int accRaw[3];
  int gyroRaw[3];
  int magRaw[3];  
  static struct timeval tv; // Image timestamp
  int timeLast[IMU_DEV_MAX]={0};
  //Recieve from IMU
  printf("   IMU: Receive thread started\n");

  for(d=0; d<insDevN; d++) setVariable(imu[d].iConnected, 0, 0);
  
  while(running)
  {
    dataRec = 0;
    for(d=0; d<insDevN; d++)
    {
      n = read(imu[d].insDev, &imu[d].buf[i[d]], 1);
      if(n == 1)
      {
        dataRec = 1;
        //printf("   IMU: Received %d bytes\r\n", n);
        if(imu[d].buf[i[d]] == '#' && i>0)
        {
          imu[d].buf[++i[d]] = 0;
          r = sscanf(imu[d].buf, "$%d,%d,%d,%d,%d,%d,%d,%d,%d#", &accRaw[0], &accRaw[1], &accRaw[2], &gyroRaw[0], &gyroRaw[1], &gyroRaw[2], &magRaw[0], &magRaw[1], &magRaw[2]);
          //printf("IMU: %s\r\n", imu[d].buf);
          if(r==9)
          {
            /*
            setVariable(iAttitude, 0, 1000*attitude[0]);
            setVariable(iAttitude, 1, 1000*attitude[1]);
            setVariable(iAttitude, 2, 1000*attitude[2]);
            */
            setArray(imu[d].iGyroRaw, 3, gyroRaw);
            setArray(imu[d].iAccRaw, 3, accRaw);
            setArray(imu[d].iMagRaw, 3, magRaw);
            timeLast[d] = tv.tv_sec;          
          }
          else
          {
            imu[d].buf[++i[d]] = 0;            
            printf("   IMU%d: Error in data string (len=%d) : %s\r\n", d, i[d], imu[d].buf);
            i[d]=0;
          }        
          i[d]=0;
        }
        else
        {        
          if(i[d]==0) // New start char received
          {
            if(imu[d].buf[i[d]] == '$') i[d]++;           
          }
          else if(i[d]>=128) // String exceed buffer size
          {            
            imu[d].buf[127] = 0;
            printf("   IMU%d: Data string is too long (len=%d) : %s\r\n", d, i[d], imu[d].buf);
            i[d]=0;
          }
          else
          {
            i[d]++;
          }
        }        
      }     
      if(!dataRec) usleep(1000);     
      gettimeofday(&tv, NULL);    
      if(timeLast[d]>0 && (tv.tv_sec-timeLast[d])>2) setVariable(imu[d].iConnected, 0, 0);
      else                                           setVariable(imu[d].iConnected, 0, 1);
      //setVariable(imu[d].iConnected, 0, 1);
      //printf("%ld %ld\r\n", timeLast[d], (tv.tv_sec-timeLast[d]));
    }
  } //Ending IMU loop
  for(d=0; d<insDevN; d++) 
  {
    close(imu[d].insDev);
    setVariable(imu[d].iConnected, 0, 0);
  }
  fprintf(stderr,"   IMU: Shutdown IMU read task\n");
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
void XMLCALL insStartTag(void *, const char *, const char **);
void XMLCALL insEndTag(void *, const char *);

/** \brief Initialize the IMU HAL
 *
 * Reads the XML file and sets up the INS settings
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
int initXML(char *filename) {

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
  printf("   IMU: Initializing Serial IMU %s\n",IMUVERSION);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "   IMU: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, insStartTag, insEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("   IMU: Error reading: %s\n",filename);
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
    fprintf(stderr, "smrdSerial: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <imu> XML tag found in plugins section\n");
		return -1;
	}
  printf("   IMU: XML enable: %i\r\n", xmlParse.enable);
  //Start crossbow thread after init
  if (xmlParse.enable) done = initINS();
  printf("   IMU: XML done: %i\r\n", done);

 return done;
}

void XMLCALL
insStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("imu",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("imu",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   IMU: Use of IMU disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(insDevStringDecl,attr[i+1],63); 
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate = atoi(attr[i+1]); 
    printf("   IMU: Serial port(s) '%s' at %d baud\n", insDevStringDecl, baudrate);
  }
}

void XMLCALL
insEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}

