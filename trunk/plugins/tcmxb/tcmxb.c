/** \file tcmxb.c
 *  \ingroup hwmodule
 *  \brief PNI ForceField TCM XB sensor (Armadillo Scout - University of Hohenheim)
 *
 *
 * Tilt-compensated compass modules provide reliable, pinpoint-accurate pitch,
 * roll and compass heading. The TCMs use advanced algorithms to counter the effects
 * of hard and soft iron interference, providing highly accurate heading information
 *  in most any environment and any orientation.  PNI's patented magneto-inductive
 *  sensors and pioneering measurement technology combine to provide all this
 *  performance under a low power budget that extends mission duration.
 *
 * It is designed for the Armadillo Scout, which SDU build for UniHo. It is the first robot that can be controlled with both MobotWare and FroboMind
 *
 * Armadillo Scout official webpage: https://mpt.uni-hohenheim.de/90593?&L=1
 * Armadillo Scout Documentation webpage: http://mpt-internal.uni-hohenheim.de/doku.php?id=robots:armadillo:welcome
 *
 *  \author Kim Kofoed Nielsen
 *  $Rev: 284 $
 *  $Date: 2013-10-29 09:41:31 +0100 (Tue, 29 Oct 2013) $
 */

/***************************************************************************
 *                  Copyright 2012 Claes Jæger-Hansen                      *
 *                                 cjh@uni-hohenheim.de                    *
 *                                 claeslund@gmail.com                     *
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
 #define TCMXBVERSION 	      "1.0"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 284 $"
 #define DATE             "$Date: 2013-10-29 09:41:31 +0100 (Tue, 29 Oct 2013) $"
 #define ID               "$Id: tcmxb.c 284 2013-10-28 13:14:56Z cjag $"
/***************************************************************************/

/*********************** Included Libraries ***********************/
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
#include <math.h>

//RHD Core headers
#include <rhd.h>
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "tcmxb.h"

/*********************** Global Variables ***********************/
int 	irs232,
		baudrate,
		rrsRunning,
		ret, 
		startup=1,
		num, 
		num_ids;
char 	FFsensorDataDevString[64]; 
short 	data_length;
unsigned char 	buff[50], //! Buffer array for sending messages
				buffrx[50]; //! Buffer array for reciving messages.
unsigned char   *pointer; //! Pointer, that points to buffrx.

// Variables for the database
float fhead, ftemp, fPA, fPR, fIZ, froll, fpitch, fPX, fPY, fPZ, num_f;

// Threads are being defined
pthread_t tcmxb_thread;
pthread_attr_t attr;

/*********************** Function initializations ***********************/
int tcmxb_init(void);
void *tcmxb_task(void *);
unsigned short CRC(void);
float ieee754(int);

/*********************** Start of Program ***********************/
/**
 * @brief This function initializes the sensor, the threads and the data 
 * base variables.
 * @returns 
 * 
 * 
 */
int tcmxb_init(void)
{
	if ((irs232 = open (FFsensorDataDevString, O_RDWR /*| O_NONBLOCK*/)) == -1) 
	{
		fprintf(stderr,"   FFSensor: Can't open first serial port: %s\n",FFsensorDataDevString);
		rrsRunning = -1;
		return -1;
	} 
	if (set_serial(irs232,baudrate) == -1) 
	{
		fprintf(stderr,"   FFSensor: Can't set first serial port parameters\n");
		fprintf(stderr,"   FFSensor: RHD is NOT running!\n");
		rrsRunning = -1;
		return -1;
	} 
	else 
	{	
		rrsRunning = 1;
	}

	//If initialization is done, create variables
	fhead         	= createVariable('r',1,"TCMhead"); // heading (degrees)
	ftemp        	= createVariable('r',1,"TCMtemp"); // internal temp of TCM
	
	fPA        		= createVariable('r',1,"TCMPA");
	fPR       		= createVariable('r',1,"TCMPR");
	fIZ       		= createVariable('r',1,"TCMIZ");
	
	froll  			= createVariable('r',1,"TCMroll");
	fpitch  		= createVariable('r',1,"TCMpitch");
	
	fPX  			= createVariable('r',1,"TCMxVector");
	fPY  			= createVariable('r',1,"TCMyVector");
	fPZ  			= createVariable('r',1,"TCMzVector");
	
	/* Create threads */
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED); 
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	if (pthread_create(&tcmxb_thread, &attr, tcmxb_task, 0)) 
	{
		perror("FFSensor: can't start FFSensor tx thread");
		return -1;
	}
	
	return 1;  
}

/**
 * @brief This function is for reading, parsing and saving the data of 
 * the sensor.
 * @param not_used 
 * @returns 
 * 
 * 
 */
void *tcmxb_task(void *not_used)
{
	
	#ifdef DEBUG
		printf("FFS-Plugin Running\n");
	#endif
	if (mlockall(MCL_CURRENT | MCL_FUTURE))
	{
		perror("mlockall");
		exit(-1);
	}

	/* use real-time (fixed priority) scheduler
	* set priority to one less than the maximum
	*/
	struct sched_param param;

	param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
	if (sched_setscheduler(0, SCHED_RR, &param)) 
	{
		perror("setscheduler");
		pthread_exit(0);
	}

	if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
		fprintf(stderr, "signal: can't ignore SIGPIPE.\n");

	fprintf(stderr, "   FFSensor: RS232 FFS rx_task running\n");
	
	usleep(150000);

	while(rrsRunning == 1) 
	{
		//printf("running");
		pointer = &buffrx;
		int i = 0;
		buffrx[1] = 5;
		while(i != buffrx[1])
		{
			ret = secureRead(irs232, pointer,1);
			if (ret <= 0) 
			{
				fprintf(stderr, "   RS232: error reading rs232");
				rrsRunning = -1;
				pthread_exit(0);
			}		
			*pointer++;
			i++;
		}
		
		unsigned int index = 1;
		switch ( buffrx[2] ) 
		{
			case kModInfoResp:
			//	printf("kModInfoResp");
			break;
			
			case kDataResp:

				if(buffrx[index++] != 0)
				{
					data_length = (buffrx[0] << 8) | buffrx[1];
				//	printf("Datalen: %d\n",data_length);  
				}

				if(buffrx[index++] == kDataResp)
					;//printf("kDataResp\n");
				else break;
				
				num_ids = buffrx[index++];
				// printf("num ids: %d\n",num_ids);
				
				if(buffrx[index++] == kHeading)
				{
					num =  (buffrx[index++]<<24);
					num =  (buffrx[index++]<<16) 	| num;
					num =  (buffrx[index++]<<8)		| num;
					num =  (buffrx[index++])		| num;
					num_f = ieee754(num);
					setVariable(fhead	, 0, num_f*100);
				//	printf("kHeading is: %f\n",num_f);
				}
				if(buffrx[index++] == kTemperature)
				{
					num =  (buffrx[index++]<<24);
					num =  (buffrx[index++]<<16) 	| num;
					num =  (buffrx[index++]<<8)		| num;
					num =  (buffrx[index++])		| num;
					num_f = ieee754(num);
					setVariable(ftemp	, 0, num_f*100);
				//	printf("kTemperature is: %f\n",num_f);
				}
				if(buffrx[index++] == kPAligned)
				{
					num =  (buffrx[index++]<<24);
					num =  (buffrx[index++]<<16) 	| num;
					num =  (buffrx[index++]<<8)		| num;
					num =  (buffrx[index++])		| num;
					num_f = ieee754(num);
					setVariable(fPA	, 0, num_f*100);
				//	printf("kPAligned is: %f\n",num_f);
				}
				if(buffrx[index++] == kRAligned)
				{
					num =  (buffrx[index++]<<24);
					num =  (buffrx[index++]<<16) 	| num;
					num =  (buffrx[index++]<<8)		| num;
					num =  (buffrx[index++])		| num;
					num_f = ieee754(num);
					setVariable(fPR	, 0, num_f*100);
				//	printf("kRAligned is: %f\n",num_f);
				}
				if(buffrx[index++] == kIZAligned)
				{
					num =  (buffrx[index++]<<24);
					num =  (buffrx[index++]<<16) 	| num;
					num =  (buffrx[index++]<<8)		| num;
					num =  (buffrx[index++])		| num;
					num_f = ieee754(num);
					setVariable(fIZ	, 0, num_f*100);
				//	printf("kIZAligned is: %f\n",num_f);
				}
				if(buffrx[index++] == kPAngle)
				{
					num =  (buffrx[index++]<<24);
					num =  (buffrx[index++]<<16) 	| num;
					num =  (buffrx[index++]<<8)		| num;
					num =  (buffrx[index++])		| num;
					num_f = ieee754(num);
					setVariable(fpitch	, 0, num_f*100);
				//	printf("kPAngle is: %f\n",num_f);
				}
				if(buffrx[index++] == kRAngle)
				{
					num =  (buffrx[index++]<<24);
					num =  (buffrx[index++]<<16) 	| num;
					num =  (buffrx[index++]<<8)		| num;
					num =  (buffrx[index++])		| num;
					num_f = ieee754(num);
					setVariable(froll	, 0, num_f*100);
				//	printf("kRAngle is: %f\n",num_f);
				}
				if(buffrx[index++] == kXAligned)
				{
					num =  (buffrx[index++]<<24);
					num =  (buffrx[index++]<<16) 	| num;
					num =  (buffrx[index++]<<8)		| num;
					num =  (buffrx[index++])		| num;
					num_f = ieee754(num);
					setVariable(fPX	, 0, num_f*100);
				//	printf("kXAligned is: %f\n",num_f);
				}
				if(buffrx[index++] == kYAligned)
				{
					num =  (buffrx[index++]<<24);
					num =  (buffrx[index++]<<16) 	| num;
					num =  (buffrx[index++]<<8)		| num;
					num =  (buffrx[index++])		| num;
					num_f = ieee754(num);
					setVariable(fPY	, 0, num_f*100);
				//	printf("kYAligned is: %f\n",num_f);
				}
				if(buffrx[index++] == kZAligned)
				{
					num =  (buffrx[index++]<<24);
					num =  (buffrx[index++]<<16) 	| num;
					num =  (buffrx[index++]<<8)		| num;
					num =  (buffrx[index++])		| num;
					num_f = ieee754(num);
					setVariable(fPZ	, 0, num_f*100);
				//	printf("kZAligned is: %f\n",num_f);
				}
			break;
			
			default:
				printf("Unknown response\n");
			break;
		}
		

	}

	if(rrsRunning != 1)
	{		
		close(irs232);
		fprintf(stderr,"FFS: Shutdown FFS task\n");
		pthread_exit(0);
	}
	return 1;
}

/**
 * @brief This periodic function runs every 1 sec, as defined in the config
 * @param tick specified in: rhdconfig.tcmxb.xml.
 * @returns 
 * 
 * 
 */
extern int periodic(int (tick))
{
	if(rrsRunning == 1)
	{
		
		// Check if if the plugin has just started
		if(startup)
		{
			/* This message is required to be sent the first time
			 * Because the TCMXB do not know which variables you want
			 * with the kgetdata command.
			 * Message:
			 * **Length
			 * **command id
			 * **number of variable id's
			 * **list of id's
			 * ** Checksum
			 * */
			buff[0] 	= 	0x00;
			buff[1]		= 	16;
			buff[2]		= 	kSetDataComponents; //kSetDataComponents
			buff[3]		= 	10;
			buff[4]		= 	kHeading;
			buff[5]		= 	kTemperature;
			buff[6]		= 	kPAligned;
			buff[7] 	= 	kRAligned;
			buff[8]		= 	kIZAligned;
			buff[9]		= 	kPAngle;
			buff[10]	= 	kRAngle;
			buff[11]	= 	kXAligned;
			buff[12]	= 	kYAligned;
			buff[13]	= 	kZAligned;
			
			unsigned int crc_c = CRC(); // calculate checksum

			buff[14]	=	(unsigned char) (crc_c >> 8);
			buff[15]	=	(unsigned char) (crc_c & 0xff);
			ret = secureWrite(irs232, &buff, 16);
			
		}
	
		//printf("\n");
		// if the initial command has been sent, keep requesting data.
		if(startup == 0)
		{
			buff[0] = 0x00;
			buff[1] = 0x05;
			buff[2]	= kGetData;
			buff[3]	= 0xBF;
			buff[4]	= 0x71;
			
			ret = secureWrite(irs232, &buff, 5);
			if (ret<0) 
			{
				fprintf(stderr,"Error sending message with id:0x%4x\n",2);
				rrsRunning = -1;
				return -1;
			}			
		}
		startup = 0;
	}
	
	return 0;
}

/**
 * @brief This function converts the integer from the sensor to floating point
 * @param num This is the integer from the sensor
 * @returns The function returns the floating point value.
 * 
 * 
 */
float ieee754(int num)
{
	int man_int, exponent,s; 
	float converted, mantissa,exp;
	
	s = (num & 0x80000000) >> 31;
	exponent = (num & 0x7F800000) >> 23; // Isolate the exponent bitvalue
	man_int = num & 0x007FFFFF; 		// Isolate the mantissa bitvalue 
	
	exp = pow(2, exponent-127);			// Calculate exponent
	mantissa = (float) (man_int) / 0x800000 + 1;  // Divide the number with 2^23 (2^22 to 2^-1)		

	converted = exp * mantissa; // calvulate the Floating point.
	if(s == 1)
		converted = -(converted);
	return converted;
}

/**
 * @brief This function calculates the checksum of the message in question
 * @returns it returns the checksum value.
 * 
 * 
 */
unsigned short CRC(void)
{
	int len = (buff[0] << 2) + (buff[1]) - 2; 
	unsigned int index = 0;
	// Update the CRC for transmitted and received data using
	// the CCITT 16bit algorithm (X^16 + X^12 + X^5 + 1).
	unsigned short crc = 0;
	while(len--)
	{
		crc = (unsigned char)(crc >> 8) | (crc << 8);
		crc ^= buff[index++];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= (crc << 8) << 4;
		crc ^= ((crc & 0xff) << 4) << 1;
	}
	return crc;
}

/************************** XML Initialization **************************/
///Struct for shared parse data

typedef struct  
{
	int 	depth; /**<  */
	char 	skip; /**<  */
	char 	enable; /**<  */
	char 	found; /**<  */

}parseInfo;

//Parsing functions
void XMLCALL tcmxbStartTag(void *, const char *, const char **);
void XMLCALL tcmxbEndTag(void *, const char *);

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

extern int initXML(char *filename) 
{
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
	printf("FFsensor: Initializing Force Field Sensor HAL %s.%s\n",TCMXBVERSION,tempString);

	/* Initialize Expat parser*/
	XML_Parser parser = XML_ParserCreate(NULL);
	if (! parser) 
	{
		fprintf(stderr, "FFsensor: Couldn't allocate memory for XML parser\n");
		return -1;
	}

	//Setup element handlers
	XML_SetElementHandler(parser, tcmxbStartTag, tcmxbEndTag);
	//Setup shared data
	memset(&xmlParse,0,sizeof(parseInfo));
	XML_SetUserData(parser,&xmlParse);

	//Open and read the XML file
	fp = fopen(filename,"r");
	if(fp == NULL)
	{
		printf("FFsensor: Error reading: %s\n",filename);
		return -1;
	}
	//Get the length of the file
	fseek(fp,0,SEEK_END);
	xmlFilelength = ftell(fp); //Get position
	fseek(fp,0,SEEK_SET); //Return to start of file

	//Allocate text buffer
	xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
	if (xmlBuf == NULL) 
	{
		fprintf(stderr, "   Couldn't allocate memory for XML File buffer\n");
		return -1;
	}
	memset(xmlBuf,0,xmlFilelength);
	len = fread(xmlBuf, 1, xmlFilelength, fp);
	fclose(fp);

	//Start parsing the XML file
	if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) 
	{
		fprintf(stderr, "FFsensor: XML Parse error at line %d: %s\n",
		(int)XML_GetCurrentLineNumber(parser),
		XML_ErrorString(XML_GetErrorCode(parser)));
		
		return -1;
	}
	XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) 
	{
		printf("   Error: No <FFsensor> XML tag found in plugins section\n");
		return -1;
	}

	//Start crossbow thread after init
	if (xmlParse.enable) done = tcmxb_init();

	return done;
}

///Handle XML Start tags

void XMLCALL
tcmxbStartTag(void *data, const char *el, const char **attr)
{
	int i;
	parseInfo *info = (parseInfo *) data;
	info->depth++;

	//Check for the right 1., 2. and 3. level tags
	if (!info->skip) 
	{
		if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
		((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
		((info->depth == 3) && (strcmp("FFsensor",el) != 0))) 
		{
			info->skip = info->depth;
			return;
		} 
		else if (info->depth == 3) 
		{
			info->found = 1;
		}
	} else return;

	//Branch to parse the elements of the XML file.
	if (!strcmp("FFsensor",el)) 
	{
		//Check for the correct depth for this tag
		for(i = 0; attr[i]; i+=2) 
			if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) 
			{
				info->enable = 1; 
			}
		if (!info->enable) 
		{
			printf("   FFsensor: Use of FFsensor disabled in configuration\n"); 
			info->skip = info->depth;
		}
	} 
	else if (strcmp("dataserial",el) == 0) 
	{
		//Check for the correct depth for this tag
		if(info->depth != 4) 
		{
			printf("Error: Wrong depth for the %s tag\n",el);
		}
	
		for(i = 0; attr[i]; i+=2) 
			if (strcmp("port",attr[i]) == 0) strncpy(FFsensorDataDevString,attr[i+1],63); 
		for(i = 0; attr[i]; i+=2) 
			if (strcmp("baudrate",attr[i]) == 0) baudrate = atoi(attr[i+1]); 
	
		printf("   FFsensor: Data RS232 port %s at %d baud\n",FFsensorDataDevString,baudrate);
	} 
}

///Handle XML End tags

void XMLCALL
tcmxbEndTag(void *data, const char *el)
{
	parseInfo *info = (parseInfo *) data;
	info->depth--;

	if (info->depth < info->skip) info->skip = 0;
}

