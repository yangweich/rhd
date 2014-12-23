/** \file esm.c
 *  \ingroup hwmodule
 *  \brief TCM2 Electronic Sensor Module (atrv-jr)
 *
 * ESM for RHD. 
 * 
 * 
 *
 *  \author Claes Jæger-Hansen & Jørgen Eriksen
 *  $Rev: 59 $
 *  $Date: 2010-08-02 11:07:57 +0200 (Mon, 02 Aug 2010) $
 */

/***************************************************************************
 *                  Copyright 2010 Claes Jæger-Hansen                      *
 *                                 claeslund@gmail.com                     *
 *                                 Jørgen Eriksen                          *
 *                                 mail@erixen.com                         *
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
 #define ESMVERSION 	      "1.0"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2010-08-02 11:07:57 +0200 (Mon, 02 Aug 2010) $:"
 #define ID               "$Id: esm.c 59 2012-10-21 06:25:02Z jcan $"
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
#include <globalfunc.h>

#include "esm.h"

///Maximum size of esm String
#define ESMSTRINGSIZE 128

FILE *esmLog;

//Definitions
int  esmDev;           ///esm Port file pointer
char esmDevString[64]; ///String to hold esm device
int  baudrate = 0;
static volatile char esmRunning = 0;

//Datavars for esm
int  iCompass, iPitch, iRoll, iX, iY, iZ, iTemperature;
char iError[4];

// Threads are being defined
pthread_t esm_thread;
pthread_attr_t attr;

//Function prototypes
int initesm(void);

//int set_serial(int fd, int baud);
void *esm_task(void *);
int parseesmData(char* inBuf);

/** \brief Initialize ESM
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
 
int initesm(void){
	
	// Open RS232 port
	esmDev = open(esmDevString, O_RDWR);
	if (esmDev<0) 
	{
		fprintf(stderr,"ESM: Error opening %s\n",esmDevString);
		return -1;
	}
	//Set baudrate for ESM unit
	if (set_serial(esmDev,baudrate) == -1)  {
		fprintf(stderr," ESM: Can't set esm serial port parameters\n");
		return -1;
	}
	
	esmRunning = 1;
	//setup baudrate
	//Create variables	
	iCompass     = createVariable('r', 1, "esmcompassheading");
	iPitch       = createVariable('r', 1, "esmpitch");
	iRoll        = createVariable('r', 1, "esmroll");
	iX           = createVariable('r', 1, "esmBxmagnetic");
	iY           = createVariable('r', 1, "esmBymagnetic");
	iZ           = createVariable('r', 1, "esmBzmagnetic");
	iTemperature = createVariable('r', 1, "esmtemperature");
//	iError       = createVariable('r', 1, "esmerror");
	
	// Initialization and starting of threads
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	if (pthread_create(&esm_thread, &attr, esm_task, 0))
	{
		perror("ESM: Can't start ESM thread");
		return -1;
	}

	return 1;

}

/** \brief Initialize Shut down esm rx thread
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
int shutdownEsm(void) {
  esmRunning = 0;
  pthread_join(esm_thread,NULL);

  return 1;
}
/** \brief ESM RX thread.
 */
void *esm_task(void *not_used) {
	printf("ESM: Rx thread started\n");
	char buf[ESMSTRINGSIZE];
	char tmp;
	int n;
	
	//Recieve from esm
	while(esmRunning) {
	memset(buf,0,ESMSTRINGSIZE);
	tmp = 0;
	for(n = 0; tmp != '\n'; n++) {
		if(secureRead(esmDev,&tmp,1) > 0) { //Debug: fread(&tmp,1,1,esmLog)
			buf[n]=tmp;
		}
		else {
			esmRunning = 0; //Shutdown if read-error
			fprintf(stderr,"ESM: Error reading from Serial port, shutting down\n");
		}
		//Check for $ tag at start of ESM String
		if(buf[0] != '$')  n=-1;
		if (!esmRunning) break; //Abort if shutdown signal is recieved
	}

	//Parse esm String to right variables
		parseesmData(buf);
	} //Ending ESM loop

  close(esmDev);
  fprintf(stderr,"ESM: Shutdown ESM task\n");
  pthread_exit(0);
  
}
 
int parseesmData(char* inBuf) {	
	int  inputVar[2];
	double dTemp;
	//int len;
	char *p1, *p2;
	//char **p;


	p2 = inBuf;
	p1 = strchr(p2, '*');

	if (p1 != NULL)
		// terminate before checksum
		*p1 = '\0';
	
	if(strncmp(p2,"$",1)!=0) {
		return -1;
	}
	
	//p = p1;

	//parse compass heading
	dTemp = strtod(p2+2, &p1);
        inputVar[0] = (int) (dTemp * 1000);
//        inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
        setArray(iCompass,1,inputVar);
	p2=p1;
	
	//parse pitch
	dTemp = strtod(p2+1, &p1);
	inputVar[0] = (int) (dTemp * 1000); // + 90) * 100); // Offset to 90 deg and remove decimals
//        inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
        setArray(iPitch,1,inputVar);
	p2=p1;

	//parse Roll
	dTemp = strtod(p2+1, &p1);
	inputVar[0] = (int) (dTemp * 1000); // + 90) * 100); // Offset to 90 deg and remove decimals
//        inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
        setArray(iRoll,1,inputVar);
	p2=p1;

	//parse Bx
	dTemp = strtod(p2+1, &p1);
	inputVar[0] = (int) (dTemp * 1000); // + 90) * 100); // Offset to 90 deg and remove decimals
//        inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
        setArray(iX,1,inputVar);
	p2=p1;

	//parse By
	dTemp = strtod(p2+1, &p1);
	inputVar[0] = (int) (dTemp * 1000); // + 90) * 100); // Offset to 90 deg and remove decimals
//        inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
        setArray(iY,1,inputVar);
	p2=p1;

	//parse Bz
	dTemp = strtod(p2+1, &p1);
	inputVar[0] = (int) (dTemp * 1000); // + 90) * 100); // Offset to 90 deg and remove decimals
//        inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
        setArray(iZ,1,inputVar);
	p2=p1;

	//parse Temp
	dTemp = strtod(p2+1, &p1);
	inputVar[0] = (int) (dTemp * 1000); // + 90) * 100); // Offset to 90 deg and remove decimals
//        inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
        setArray(iTemperature,1,inputVar);

/*

	p1 = strsep(&p2, "P");
	if(strncmp(p1,"$C",2)==0)
	{
		p1 += 2;
		len = sscanf(p1,"%lf",&dTemp);
		printf("Compass = %s", p1);
        if(len == 1) {
          inputVar[0] = (int)dTemp;
          inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
          setArray(iCompass,2,inputVar);
        }
	}
	//parse Pitch value
	p1 = strsep(&p2, "R");
	//printf("Pitch = %s", p1);
	if(strncmp(p1,"P",1))
	{
		//p1 += 1;
		len = sscanf(p1,"%lf",&dTemp);
        if(len == 1) {
          inputVar[0] = (int)((dTemp));// + 90) * 100); // Offset to 90 deg and remove decimals
          inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
          setArray(iPitch,1,inputVar);
        }
	}
	//parse Roll value
	p1 = strsep(&p2, "X");
	if(strncmp(p1,"R",1))
	{
		//p1 += 1;
		len = sscanf(p1,"%lf",&dTemp);
        if(len == 1) {
          inputVar[0] = (int)((dTemp));// + 90) * 100); // Offset to 90 deg and remove decimals
          inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
          setArray(iRoll,2,inputVar);
        }
	}
	//parse Magnetometer Bx
	p1 = strsep(&p2, "Y");
	if(strncmp(p1,"X",1))
	{
		//p1 += 1;
		len = sscanf(p1,"%lf",&dTemp);
        if(len == 1) {
          inputVar[0] = (int)dTemp;
          inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
          setArray(iX,2,inputVar);
        }
	}
	//parse Magnetometer By
	p1 = strsep(&p2, "Z");
	if(strncmp(p1,"Y",1))
	{
		//p1 += 1;
		len = sscanf(p1,"%lf",&dTemp);
        if(len == 1) {
          inputVar[0] = (int)dTemp;
          inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
          setArray(iY,2,inputVar);
        }
	}
	//parse Magnetometer Bz
	p1 = strsep(&p2, "T");
	if(strncmp(p1,"Z",1))
	{
		//p1 += 1;
		len = sscanf(p1,"%lf",&dTemp);
        if(len == 1) {
          inputVar[0] = (int)dTemp;
          inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
          setArray(iZ,2,inputVar);
        }
	}
	
 	//parse temperature
	p1 = strsep(&p2, "\0");
	if(strncmp(p1,"T",1))
	{
		//p1 += 1;
		len = sscanf(p1,"%lf",&dTemp);
	        if(len == 1) {
        	  inputVar[0] = (int)dTemp;
	          inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
        	  setArray(iTemperature,2,inputVar);
	        }
	}*/
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
void XMLCALL esmStartTag(void *, const char *, const char **);
void XMLCALL esmEndTag(void *, const char *);

/** \brief Initialize the ESM HAL
 *
 * Reads the XML file and sets up the esm settings
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
  printf("ESM: Initializing Serial ESM HAL %s.%s\n",ESMVERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "ESM: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, esmStartTag, esmEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("ESM: Error reading: %s\n",filename);
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
		printf("   Error: No <esm> XML tag found in plugins section\n");
		return -1;
	}

  //Start crossbow thread after init
  if (xmlParse.enable) done = initesm();



 return done;
}

void XMLCALL
esmStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("esm",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("esm",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   esm: Use of esm disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(esmDevString,attr[i+1],63); 
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate = atoi(attr[i+1]); 
    printf("   esm: Serial port %s at %d baud\n",esmDevString,baudrate);
  } 

}

void XMLCALL
esmEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}



