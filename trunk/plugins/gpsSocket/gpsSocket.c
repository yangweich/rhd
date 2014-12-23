/** \file gpsSocket.c
 *  \ingroup hwmodule
 *  \brief GPS Module serial and socket
 *
 * GPS Module for RHD. 
 * 
 * Based on AuGps by Lars Mogensen and Christian Andersen and 
 * HAKOD by Asbjørn Mejnertsen and Anders Reeske Nielsen
 * 
 * The module supports both standard NMEA GPS and RTK GPS through
 * serial port.
 * 
 * So far, conversion from lat/long <> UTM is not working so
 * data from a UTM GPS is only in UTM and data from a lat/long
 * GPS is only in lat/long
 *
 *  \author Anders Billesø Beck
 *  modified to add Leika GPS messages GPLLQ and GPLLK message types 
 *         by ex26 (inserted by christian 27/8/2010)
 *  modified by Claes Dühring Jaeger and Emir Memic to include socket interface.
 *  $Rev: 320 $
 *  $Date: 2013-12-10 14:59:15 +0100 (Tue, 10 Dec 2013) $
 *  $Id: gpsSocket.c 176 2013-04-03 15:03:53Z cjag $
 * 
 * \todo check why buffer is mised in line 360 when using socket
 */
 /***************************************************************************
 *                  Copyright 2008 Anders Billesø Beck                     *
 *                       anders.beck@get2net.dk                            *
 *                  Copyright 2013 Claes Jaeger                            *
 *                       cjh@uni-hohenheim.de                              *
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
#define GPSVERSION 	      "1.0"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 320 $"
 #define DATE             "$Date: 2013-12-10 14:59:15 +0100 (Tue, 10 Dec 2013) $"
 #define ID               "$Id: gpsSocket.c 176 2013-04-03 15:03:53Z cjag $"
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
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <expat.h>
#include <math.h>

#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr

#include <database.h>
//#include <globalfunc.h>

#include "gpsSocket.h"

///Maximum size of GPS String
#define GPSSTRINGSIZE 128
///Conversion factor from knot to m/s
#define KNOT2MS 0.51444444

FILE *gpsLog;

//Definitions
int  gpsDev;      ///GPS Port file pointer
int  gpsDesSoc;		///GPS socket file pointer
char gpsDevString[64]; ///String to hold GPS device
char connection[64]; ///String for the connection type
char ip[64];
int  portIP = 0;
int  baudrate = 0;
int  utmZone;
static volatile char gpsRunning = 0;
struct sockaddr_in server;

///Database indexes
int iTime, iDate, iTod, iUTMn, iUTMe, iLat, iLong, iQuality, iFixvalid, iDop;
int iSatuse,iUtmzone, iHeading, iSpeed, iEgnos, iLlfixes, iUtmfixes;
int iAlt, iHeigth, iWUTMZone, iUtmQuality;

// Threads are being defined
pthread_t gps_thread_read;
pthread_t gps_thread_decode;
pthread_attr_t attr;
pthread_mutex_t decodeFlag; // pthread_mutex_lock
/**
10 buffers for NMEA messages, each 512 bytes long */
#define MaxBufSize 512
#define MaxBufCnt 10
char buf[MaxBufCnt][MaxBufSize];
int bufIdx = -1; /* index to completed buffer */


//Function prototypes
int initGps(void);
int set_serial(int fd, int baud);
void *gps_task_read(void *);
void *gps_task_decode(void *);
int validateNMEA(char* inBuf);
int parseNMEA(char* inBuf);
int parsePTNL(char * inBuf);
int parseGPLLK(char * inBuf);
int parseGPLLQ(char * inBuf);
int parseGPGGA(char * inBuf);
int parseGPRMC(char * inBuf);
int latlon2UTM(void);
int utm2latlon(void);
void decodeWait(void);
void decodePost(void);


/** \brief Initialize GPS                              {}
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
int initGps(void) {
  // Open RS232 port
	if (strcmp("serial",connection) == 0)									/************/
	{
		gpsDev = open(gpsDevString, O_RDWR);
		if (gpsDev<0) 
		{
			fprintf(stderr,"   GPS: Error opening %s\n",gpsDevString);
			return -1;
		}
		//Set baudrate for GPS receiver
		if (set_serial(gpsDev,baudrate) == -1) 
		{
			fprintf(stderr,"   GPS: Can't set GPS serial port parameters\n");
			return -1;
		}
	}																				/**********************/
	// open socket
	else if(strcmp("socket",connection) == 0)
	{
		gpsDesSoc = socket(AF_INET, SOCK_STREAM, 0);
			
		if (gpsDesSoc == -1)
		{
			printf(" ERROR : could not create socket\n");
		}
				
		server.sin_addr.s_addr = inet_addr(ip);
		server.sin_family = AF_INET;
		server.sin_port = htons(portIP);
				
		//Connect to GNSS
		if (connect(gpsDesSoc , (struct sockaddr *)&server , sizeof(server)) < 0)
		{
			printf("ERROR : connect error\n");
			return 1;
		}
	
			printf("Connected\n");
			//return 0;
	}																			/*********************/
	gpsRunning = 1;
	//gpsLog = fopen("../test/gpslog.txt","r"); //Test RTK GPS

	//Create variables
	iTime         = createVariable('r', 4, "GPStime");
	iDate         = createVariable('r', 3, "GPSdate");
	iTod          = createVariable('r', 2, "GPStimeofday");
	iUTMn         = createVariable('r', 2, "GPSnorthing");
	iUTMe         = createVariable('r', 2, "GPSeasting");
	iLat          = createVariable('r', 2, "GPSlattitude");
	iLong         = createVariable('r', 2, "GPSlongitude");
	iQuality      = createVariable('r', 1, "GPSquality");
	iFixvalid     = createVariable('r', 1, "GPSfixvalid");
	iSatuse       = createVariable('r', 1, "GPSsatused");
	iDop          = createVariable('r', 2, "GPSdop");
	iAlt          = createVariable('r', 2, "GPSaltitude");
	iHeigth       = createVariable('r', 2, "GPSheigth");
	iUtmzone      = createVariable('r', 1, "GPSutmzone");
	iHeading      = createVariable('r', 2, "GPSheading");
	iSpeed        = createVariable('r', 2, "GPSspeed");
	iEgnos        = createVariable('r', 1, "GPSegnos");
	iLlfixes      = createVariable('r', 1, "GPSllfixes");
	iUtmfixes     = createVariable('r', 1, "GPSutmfixes");
	iWUTMZone     = createVariable('w', 1, "GPSsetutmzone");
	iUtmQuality   = createVariable('r', 2, "GPSutmquality");


	// Initialization and starting of threads
	pthread_mutex_init(&decodeFlag, NULL);
	decodeWait();
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	if (pthread_create(&gps_thread_read, &attr, gps_task_read, 0))
	{
		perror("   GPS: Can't start GPS read thread");
		return -1;
	}
	if (pthread_create(&gps_thread_decode, &attr, gps_task_decode, 0))
	{
		perror("   GPS: Can't start GPS decode thread");
		return -1;
	}
	return 1;
}

/** \brief Initialize Shut down GPS rx thread
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
int shutdownGps(void)
{
	if (strcmp("serial",connection) == 0) 
	{
		close(gpsDev);
	}
	else if(strcmp("socket",connection) == 0) 
	{
		close(gpsDesSoc);
	}
	gpsRunning = 0;
	pthread_join(gps_thread_read,NULL);
	pthread_join(gps_thread_decode,NULL);

	return 1;
}

/**
Wait for decode data ready flag */
void decodeWait()
{
	pthread_mutex_lock(&decodeFlag);
}

/**
Flag that decode data is ready */
void decodePost()
{
	pthread_mutex_unlock(&decodeFlag);
}

/** \brief GPS RX thread.
 */
void *gps_task_read(void *not_used) {
  int i = -1;
  char *bufRx, *bufNext;
  //Recieve from GPS
  printf("   GPS: Receive thread started\n");
  bufRx = buf[0];
  bufRx[0] = '\0';
  int misCnt = 0;
 while(gpsRunning)
  {
    bufNext = bufRx;
    *bufNext = '\0';
    while (bufNext - bufRx < MaxBufSize - 1 && gpsRunning)
    { /* space for more data */
      if(strcmp("serial",connection) == 0){
	      i = read(gpsDev, bufNext, 1);
	}
	else if(strcmp("socket",connection) == 0){
	      i = read(gpsDesSoc, bufNext, 1);
	}
      if (i <= 0)
      { /* read error */
        gpsRunning = 0;
        fprintf(stderr,"GPS: Error reading from Serial port, shutting down\n");
        break;
      }
      if (*bufNext == '\n')
        // newline marks end of message
        break;
      if (*bufNext == '$' && bufNext != bufRx)
      { // $ must be at start of message and is not, so adjust
        bufNext = bufRx;
        *bufNext++ = '$';
        printf("GPS: (read) must have missed a message (%d), restarts at $\n", ++misCnt);
      }
      else if (*bufNext >= ' ')
        bufNext++;
      // other control characters (like \r) are just skipped
    }
    if (gpsRunning && i > 0)
    { /* new data buffer is ready for decoding */
      *bufNext = '\0'; // terminate NMEA string - at '\n' char
      // set buffer index of ready buffer
      bufIdx = (bufIdx + 1) % MaxBufCnt;
      // start decode thread
      decodePost();
      /* get next buffer to fill */
      bufRx = buf[(bufIdx + 1) % MaxBufCnt];
    }
  } //Ending GPS loop
  decodePost();
  close(gpsDev);
  fprintf(stderr,"GPS: Shutdown GPS read task\n");
  pthread_exit(0);
}

/** \brief GPS Decode thread.
 */
void *gps_task_decode(void *not_used)
{
	char * nmea;
	int last = -1;
	int decodeCnt = 0;
	printf("   GPS: Decode thread started\n");
	//Recieve from GPS
	while(gpsRunning)
	{ //Update UTM Zone if changed
		if (isUpdated('w',iWUTMZone) && (getWriteVariable(iWUTMZone,0) != 0))
		{ // UTM zone is updated
			utmZone = getWriteVariable(iWUTMZone,0);
			setVariable(iUtmzone,0,utmZone);
		}
		else if (getReadVariable(iUtmzone,0) == 0)
		{ 
			//Write config UTM to database (can't be done until database is locked)
			setVariable(iUtmzone,0,utmZone); 
		}
		// wait for decode flag
		decodeWait();
		// decode is we should finish
		if (bufIdx >= 0 && gpsRunning)
		{ 
			// get most recent completed buffer
			nmea = buf[bufIdx];
			//Parse NMEA GPS String
			parseNMEA(nmea);
			if (last >= 0 && (last + 1) % MaxBufCnt != bufIdx)
				printf("GPS: (decode) missed a buffer (%d)\n", ++decodeCnt);
		
			last = bufIdx;
		}
	} //Ending GPS loop
	fprintf(stderr,"GPS: Shutdown GPS decode task\n");
	pthread_exit(0);
}

/** \brief Parse input NMEA data
 *
 * \param[in] char* inbuf;
 * NMEA input buffer
 * 
 * \return int status
 * Return success or not
 */
int parseNMEA(char* inBuf) 
{
	char isOK;

	//Find the string format
	isOK = validateNMEA(inBuf);
	if (!isOK) 
	{
		printf("Failed to validate the GPS message: '%s'\n", inBuf);
		return -1;
	} 
	else if (isOK) 
	{ 
		//Recommended Minimum Specific GNSS information relayed by all satellites
		if(strncmp(inBuf,"$GPRMC", 6) == 0) 
		{ 
			// $GPRMC,180432,A,4027.027912,N,08704.857070,W, 000.04,181.9,131000,1.8,W,D*25
			//        hhmmss   ddmm.mmmmmm   dddmm,mmmmmm    knots   deg  ddmmyy, mag  A/D/N
			if (parseGPRMC(inBuf)) 
			{
				latlon2UTM(); //Convert to UTM - Not working!
			} 
			else 
			{
				printf("   GPS: $GPRMC Parse error\n");
			}
		}
		//GPS Fix data
		else if(strncmp(inBuf,"$GPGGA", 6) == 0) 
		{ 
			// $GPGGA,180432.00,4027.027912,N,08704.857070, W,2,07,1.0,212.15,M,-33.81,M,4.2,0555*73
			//        hhmmss.ss ddmm.mmmmmm   dddmm.mmmmmm     sat hdop hgt[m]   (DGPS base)
			if (parseGPGGA(inBuf)) 
			{
				//Update LL counter
				setVariable(iLlfixes,0,getReadVariable(iLlfixes,0)+1);
				latlon2UTM(); //Convert to UTM - Not working!
			} 
			else 
			{
				printf("   GPS: $GPGGA Parse error\n");
			}
		}
		//Geografic position Lat/Lon
		//Redundant information  - Not implemented
		else if(strncmp(inBuf,"$GPGLL", 6) == 0) {    }
		//GNSS Dillution Of Presition and Active Satellites - Not implemented
		else if(strncmp(inBuf,"$GPGSA", 6) == 0) {    }
		//GNSS Satellites in view - Not implemented
		else if(strncmp(inBuf,"$GPGSV", 6) == 0) {    }
		//Course over Ground and Ground Speed
		//Redundant information - Not implemented
		else if(strncmp(inBuf,"$GPVTG", 6) == 0) {    }  
		//Proparitary SiRF message - Not implemented
		else if(strncmp(inBuf,"$PSRF151", 8) == 0) {	} 
		else if (strncmp(inBuf, "$PTNL", 5) == 0) 
		{
			if (parsePTNL(inBuf)) 
			{
				//Update UTM counter
				setVariable(iUtmfixes,0,getReadVariable(iUtmfixes,0)+1);
				// should not be done here?
				// utm2latlon(); //Convert to lat/long
			} 
			else 
			{
				printf("   GPS: $PTNL Parse error\n");
			}
			//Proparitary Leica Local Position and GDOP
		}
		else if(strncmp(inBuf,"$GPLLK", 6) == 0) 
		{
			if (parseGPLLK(inBuf)) 
			{
				//Update UTM counter
				setVariable(iUtmfixes,0,getReadVariable(iUtmfixes,0)+1);
				//utm2latlon(); //Convert to lat/long
			} 
			else 
			{
				printf("   GPS: $GPLLK Parse error\n");
			}
		} 
			//Proparitary Leica Local Position and Quality
		else if(strncmp(inBuf,"$GPLLQ", 6) == 0) 
		{
			if (parseGPLLQ(inBuf)) 
			{
				//Update UTM counter
				setVariable(iUtmfixes,0,getReadVariable(iUtmfixes,0)+1);
				//utm2latlon(); //Convert to lat/long
			} 
			else 
			{
				printf("   GPS: $GPLLQ Parse error\n");
			}
		} 
	}
	return isOK;
}

/** \brief Validate NMEA Checksum
 *
 * \param[in] char* inbuf;
 * NMEA input buffer
 * 
 * \return int status
 * Return success or not
 */
int validateNMEA(char* inBuf)
{
	char tmp = 0;
	char tmpStr[3] = {0,0,0}; 
	int i = 2;
	char result;
	tmp = inBuf[1];
	if(inBuf[0] == '$')
	{
		do
		{
			tmp = tmp ^ inBuf[i];
			i++;
		} while((inBuf[i] != '*') && (inBuf[i] >= ' '));
	}
	//Validate to GPS checksum
	snprintf(tmpStr,3,"%02X",tmp);
	if (strncmp(tmpStr,inBuf+i+1,2) == 0) 
	{
		result = 1;
	} 
	else result = 0;

	return result;
}

/** \brief Parse PTNL NMEA String 
 *
 * \param[in] char* inbuf;
 * NMEA input buffer
 * 
 * \return int status
 * Return success or not
 */
int parsePTNL(char * inBuf)
{ // decode proparitary message from HAKO RTK-GPS - in UTM
// $PTNL,PJK,123148.30,091807,+6174357.366,N,+707854.368,E,1,07,3.1,EHT+64.037,M*74

	char isOK = 1;
	char *p1, *p2;
	int param = 0, len;
	char parse[3][20]={{0},{0}};
	int inputVar[4];
	double dTemp;
	double val;
	struct tm tod;
	p2 = inBuf;
	p1 = strchr(p2, '*');
	if (p1 != NULL)
	// terminate before checksum
	*p1 = '\0';

	// make a copy of the sentance - for debug - monitoring
	//strncpy(sentance, inBuf, GPS_SENTANCE_MAX_LENGTH);
	while (*p2 >= ' ')
	{ // separate into substring
		p1 = strsep(&p2, ",");
		switch (param)
		{
			case 0: // type
				if (strcmp(p1, "$PTNL") != 0)
				{ // not my message
					printf("Wrong message I know about $PTNL, not '%s'\n", p1);
					isOK = 0;
					return -1;
				}
				break;
			case 1:  // PJK - meening???
				break;
			case 2: // time of day
				len = sscanf(p1,"%2s%2s%lf",parse[0],parse[1],&dTemp);
				isOK = len == 3;
				if (isOK)
				{ 
					// reconstruct date and time (from time only)
					inputVar[0] = atoi(parse[0]);
					inputVar[1] = atoi(parse[1]);
					inputVar[2] = (int) dTemp;
					inputVar[3] = (int)(((dTemp - (int)dTemp) * 1e3) + 0.5); //Convert to ms
					//Set values for epoc tod
					tod.tm_hour = inputVar[0];
					tod.tm_min  = inputVar[1];
					tod.tm_sec  = (int) dTemp;
					setArray(iTime,4,inputVar);
				}
				break;
			case 3: // date
				len = sscanf(p1,"%2s%2s%2s",parse[0],parse[1],parse[2]);
				if(len == 3)
				{
					inputVar[0] = atoi(parse[0]);
					inputVar[1] = atoi(parse[1]);
					inputVar[2] = atoi(parse[2]) + 2000;
					//Set values for epoc tod
					tod.tm_year = inputVar[2] - 1900; //year from 1900
					tod.tm_mon  = inputVar[1] - 1; //Month 0-11
					tod.tm_mday = inputVar[0];
					setArray(iDate,3,inputVar);
					inputVar[0] = mktime(&tod);
					inputVar[1] = (int)(((dTemp - (int)dTemp) * 1e6) + 0.5); //Convert to us
					if (inputVar[0] != -1) setArray(iTod,2,inputVar);
				}
				break;
			case 4: // northing (assume this is signed)
				len = sscanf(p1,"%lf",&dTemp);
				if(len == 1) 
				{
					inputVar[0] = (int)dTemp;
					inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
					setArray(iUTMn,2,inputVar);
				}
				break;
			case 5: // N or S
				break;
			case 6:
				len = sscanf(p1,"%lf",&dTemp);
				if(len == 1) 
				{
					inputVar[0] = (int)dTemp;
					inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
					setArray(iUTMe,2,inputVar);
				}
				break;
			case 7: // E or W
				break;
			case 8: // fix mode - assume 1=no fix, 2=floating (or DGPS), 3=FIX
				setVariable(iQuality,0,atoi(p1));
				break;
			case 9: // number of satellites
				setVariable(iSatuse,0,atoi(p1));
				break;
			case 10: // DOP
				# if(0)
					p3 = strchr(p1, '.');
					if (p3 != NULL) *p3 = ' ';
					
					len = sscanf(p1,"%s%s",parse[0],parse[1]);
					if(len == 2) 
					{
						inputVar[0] = atoi(parse[0]);
						inputVar[1] = atoi(parse[1]);
						setArray(iDop,2,inputVar);
					}
				# endif
				len = sscanf(p1,"%lf",&val);
				if(len == 1) 
				{
					inputVar[0] = val;
					inputVar[1] = (val - inputVar[0]) * 1e6;
				}
				setArray(iDop,2,inputVar);
				break;
			default:
				break;
		}	
		if (p2 == NULL)
			break;
			
		param++;
	}

	return isOK;
}

/** \brief Parse GPLLK NMEA String 
 *
 * \param[in] char* inbuf;
 * NMEA input buffer
 * 
 * \return int status
 * Return success or not
 */
int parseGPLLK(char * inBuf)
{ // decode proparitary message from Leica RTK-GPS - in UTM
// $GPLLK,113616.00,041006,764413.024,M,252946.774,M,3,08,0.010,1171.279,M*12

	char isOK = 1;
	char *p1, *p2;
	int param = 0, len;
	char parse[3][20]={{0},{0}};
	int inputVar[4];
	double dTemp;
	double val;
	struct tm tod;
	p2 = inBuf;
	p1 = strchr(p2, '*');
	if (p1 != NULL)
	// terminate before checksum
	*p1 = '\0';

	// make a copy of the sentance - for debug - monitoring
	//strncpy(sentance, inBuf, GPS_SENTANCE_MAX_LENGTH);
	while (*p2 >= ' ')
	{ // separate into substring
		p1 = strsep(&p2, ",");
		switch (param)
		{
			case 0: // Header type, incl. talker ID
				if (strcmp(p1, "$GPLLK") != 0)
				{ // not my message
					printf("Wrong message I know about $GPLLK, not '%s'\n", p1);
					isOK = 0;
					return -1;
				}
				break;
			case 1:  // UTC time of position
				len = sscanf(p1,"%2s%2s%lf",parse[0],parse[1],&dTemp);
				isOK = len == 3;
				if (isOK)
				{ // reconstruct date and time (from time only)
					inputVar[0] = atoi(parse[0]);
					inputVar[1] = atoi(parse[1]);
					inputVar[2] = (int) dTemp;
					inputVar[3] = (int)(((dTemp - (int)dTemp) * 1e3) + 0.5); //Convert to ms
					//Set values for epoc tod
					tod.tm_hour = inputVar[0];
					tod.tm_min  = inputVar[1];
					tod.tm_sec  = (int) dTemp;
					setArray(iTime,4,inputVar);
				}
				break;
			case 2:  // UTC date
				len = sscanf(p1,"%2s%2s%2s",parse[0],parse[1],parse[2]);
				if(len == 3)
				{
					inputVar[0] = atoi(parse[0]);
					inputVar[1] = atoi(parse[1]);
					inputVar[2] = atoi(parse[2]) + 2000;
					//Set values for epoc tod
					tod.tm_year = inputVar[2] - 1900; //year from 1900
					tod.tm_mon  = inputVar[1] - 1; //Month 0-11
					tod.tm_mday = inputVar[0];
					setArray(iDate,3,inputVar);
					inputVar[0] = mktime(&tod);
					inputVar[1] = (int)(((dTemp - (int)dTemp) * 1e6) + 0.5); //Convert to us
					if (inputVar[0] != -1) setArray(iTod,2,inputVar);
				}
				break;
			case 3: // Grid Easting in metres (assume this is signed)
				len = sscanf(p1,"%lf",&dTemp);
				if(len == 1) 
				{
					inputVar[0] = (int)dTemp;
					inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
					setArray(iUTMe,2,inputVar);
				}
				break;
			case 4: // Units of grid Easting as fixed text M
				break;
			case 5: // Grid Northing in metres (assume this is signed)
				len = sscanf(p1,"%lf",&dTemp);
				if(len == 1) 
				{
					inputVar[0] = (int)dTemp;
					inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
					setArray(iUTMn,2,inputVar);
				}
				break;
			case 6: // Units of grid Easting as fixed text M
				break;
			case 7: // Position Quality:
					// 0 = Fix not available or invalid
					// 1 = No real-time position, navigation fix
					// 2 = Real-time position, ambiguities not fixed
					// 3 = Real-time position, ambiguities fixed
				if (atoi(p1) > 2)
					setVariable(iQuality,0,4);
				else
					setVariable(iQuality,0,atoi(p1));
				break;
			case 8: // number of satellites
				setVariable(iSatuse,0,atoi(p1));
				break;
			case 9: // GDOP
				#if (0)
					p3 = strchr(p1, '.');
					if (p3 != NULL) *p3 = ' ';
					len = sscanf(p1,"%s%s",parse[0],parse[1]);
					if(len == 2) 
					{
						inputVar[0] = atoi(parse[0]);
						inputVar[1] = atoi(parse[1]);
						setArray(iDop,2,inputVar);
					}
				#endif
				len = sscanf(p1,"%lf",&val);
				if(len == 1) 
				{
					inputVar[0] = val;
					inputVar[1] = (val - inputVar[0]) * 1e6;
				}
				setArray(iDop,2,inputVar);
				break;
			case 10: // Altitude of position marker above/below mean sea level in metres
				len = sscanf(p1,"%lf",&val);
				if(len == 1) 
				{
					inputVar[0] = val;
					inputVar[1] = (val - inputVar[0]) * 1e6;
				}
				setArray(iAlt,2,inputVar);
				break;
			case 11: // Units of altitude as fixed text M
				break;
			default:
				break;
		}

		if (p2 == NULL)
		break;

		param++;
	}
	return isOK;
}


/** \brief Parse GPLLQ NMEA String 
 *
 * \param[in] char* inbuf;
 * NMEA input buffer
 * 
 * \return int status
 * Return success or not
 */
int parseGPLLQ(char * inBuf)
{ // decode proparitary message from Leica RTK-GPS - in UTM
// $GPLLQ,113616.00,041006,764413.024,M,252946.774,M,3,08,0.010,1171.279,M*12

	char isOK = 1;
	char *p1, *p2;
	int param = 0, len;
	char parse[3][20]={{0},{0}};
	int inputVar[4];
	double dTemp;
	double val;
	struct tm tod;
	p2 = inBuf;
	p1 = strchr(p2, '*');
	if (p1 != NULL)
	// terminate before checksum
	*p1 = '\0';

	// make a copy of the sentance - for debug - monitoring
	//strncpy(sentance, inBuf, GPS_SENTANCE_MAX_LENGTH);
	while (*p2 >= ' ')
	{ 
		// separate into substring
		p1 = strsep(&p2, ",");
		switch (param)
		{
			case 0: // Header type, incl. talker ID
				if (strcmp(p1, "$GPLLQ") != 0)
				{ 
					// not my message
					printf("Wrong message I know about $GPLLQ, not '%s'\n", p1);
					isOK = 0;
					return -1;
				}
				break;
			case 1:  // UTC time of position
				len = sscanf(p1,"%2s%2s%lf",parse[0],parse[1],&dTemp);
				isOK = len == 3;
				if (isOK)
				{ 
					// reconstruct date and time (from time only)
					inputVar[0] = atoi(parse[0]);
					inputVar[1] = atoi(parse[1]);
					inputVar[2] = (int) dTemp;
					inputVar[3] = (int)(((dTemp - (int)dTemp) * 1e3) + 0.5); //Convert to ms
					//Set values for epoc tod
					tod.tm_hour = inputVar[0];
					tod.tm_min  = inputVar[1];
					tod.tm_sec  = (int) dTemp;
					setArray(iTime,4,inputVar);
				}
				break;
			case 2:  // UTC date
				len = sscanf(p1,"%2s%2s%2s",parse[0],parse[1],parse[2]);
				if(len == 3)
				{
					inputVar[0] = atoi(parse[0]);
					inputVar[1] = atoi(parse[1]);
					inputVar[2] = atoi(parse[2]) + 2000;
					//Set values for epoc tod
					tod.tm_year = inputVar[2] - 1900; //year from 1900
					tod.tm_mon  = inputVar[1] - 1; //Month 0-11
					tod.tm_mday = inputVar[0];
					setArray(iDate,3,inputVar);
					inputVar[0] = mktime(&tod);
					inputVar[1] = (int)(((dTemp - (int)dTemp) * 1e6) + 0.5); //Convert to us
					if (inputVar[0] != -1) setArray(iTod,2,inputVar);
				}
				break;
			case 3: // Grid Easting in metres (assume this is signed)
				len = sscanf(p1,"%lf",&dTemp);
				if(len == 1) 
				{
					inputVar[0] = (int)dTemp;
					inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
					setArray(iUTMe,2,inputVar);
				}
				break;
			case 4: // Units of grid Easting as fixed text M
				break;
			case 5: // Grid Northing in metres (assume this is signed)
				len = sscanf(p1,"%lf",&dTemp);
				if(len == 1) 
				{
					inputVar[0] = (int)dTemp;
					inputVar[1] = (int)(((dTemp - inputVar[0]) * 1e6) + 0.5); //Convert to µDeg
					setArray(iUTMn,2,inputVar);
				}
				break;
			case 6: // Units of grid Easting as fixed text M
				break;
			case 7: // Position Quality:
				  // 0 = Fix not available or invalid
				  // 1 = No real-time position, navigation fix
				  // 2 = Real-time position, ambiguities not fixed
				  // 3 = Real-time position, ambiguities fixed
				if (atoi(p1) > 2)
					setVariable(iQuality,0,4);
				else
					setVariable(iQuality,0,atoi(p1));
				break;
			case 8: // number of satellites
				setVariable(iSatuse,0,atoi(p1));
				break;
			case 9: // Coordinate quality in metres
				#if (0)
					p3 = strchr(p1, '.');
					if (p3 != NULL) *p3 = ' ';
					len = sscanf(p1,"%s%s",parse[0],parse[1]);
					if(len == 2) 
					{
						inputVar[0] = atoi(parse[0]);
						inputVar[1] = atoi(parse[1]);
						setArray(iUtmQuality,2,inputVar);
					}
				#endif
				len = sscanf(p1,"%lf",&val);
				if(len == 1) 
				{
					inputVar[0] = val;
					inputVar[1] = (val - inputVar[0]) * 1e6;
				}
				setArray(iUtmQuality,2,inputVar);
				break;
			case 10: // Altitude of position marker above/below mean sea level in metres
				len = sscanf(p1,"%lf",&val);
				if(len == 1) 
				{
					inputVar[0] = val;
					inputVar[1] = (val - inputVar[0]) * 1e6;
				}
				setArray(iAlt,2,inputVar);
				break;
			case 11: // Units of altitude as fixed text M
				break;
			default:
				break;
		}
		
		if (p2 == NULL)
			break;
		  
		param++;
	}
	return isOK;
}



/** \brief Parse GPGGA NMEA String 
 *
 * \param[in] char* inbuf;
 * NMEA input buffer
 * 
 * \return int status
 * Return success or not
 */
int parseGPGGA(char * inBuf)
{ 
	// $GPGGA,180432.00,4027.027912,N,08704.857070, W,2,07,1.0,212.15,M,-33.81,M,4.2,0555*73
	//        hhmmss.ss ddmm.mmmmmm   dddmm.mmmmmm     sat hdop hgt[m]   (DGPS base)
	char isOK = 0;
	char parse[3][20]={{0},{0}};
	int  inputVar[4];
	double dTemp, mm;
	int len;
	char *p1, *p2, *p3;
	int param = 0;

	p2 = inBuf;
	p1 = strchr(p2, '*');
	if (p1 != NULL)
	// terminate before checksum
    *p1 = '\0';

	while (*p2 >= ' ')
	{
		p1 = strsep(&p2, ",");
		switch(param)
		{ // decode parameters
			case 0: // just $GPGGA
				if (strcmp(p1, "$GPGGA") != 0)
				{
					printf("this is not the right message type (expected $GPGGA, got %s\n", p1);
					isOK = 0;
				}
				break;
			case 1: // time of day hhmmss.sss
				len = sscanf(p1,"%2s%2s%lf",parse[0],parse[1],&dTemp);
				isOK = len == 3;
				if (isOK)
				{ // reconstruct time (from time only)
					inputVar[0] = atoi(parse[0]);
					inputVar[1] = atoi(parse[1]);
					inputVar[2] = (int) dTemp;
					inputVar[3] = (int)(((dTemp - (int)dTemp) * 1e3) +0.5); //Convert to ms
					setArray(iTime,4,inputVar);
				}
				break;
			case 2: // Parse latitude
				sscanf(p1, "%2s%lf", parse[0], &mm);
				dTemp = mm / 60.0;
				inputVar[0] = atoi(parse[0]);
				inputVar[1] = (int)(((dTemp - (int)dTemp) * 1e6) +0.5); //Convert to *micro* deg
				setArray(iLat,2,inputVar);
				break;
			case 3: // N or S
				if (*p1 == 'S') setVariable(iLat,0,-getReadVariable(iLat,0)); //Invert lattitude
				break;
			case 4: // Parse lontitude (3 digits for degrees)
				sscanf(p1, "%3s%lf", parse[0], &mm);
				dTemp =  mm / 60.0;
				inputVar[0] = atoi(parse[0]);;
				inputVar[1] = (int)(((dTemp - (int)dTemp) * 1e6) +0.5); //Convert to *micro* deg
				setArray(iLong,2,inputVar);
				break;
			case 5: // E or W
				if (*p1 == 'W') setVariable(iLong,0,-getReadVariable(iLong,0)); //Invert longitude
				break;
			case 6: // Parse quality
				setVariable(iQuality,0,atoi(p1));
				break;
			case 7: // Parse number of satellites
				setVariable(iSatuse,0,atoi(p1));
				break;
			case 8: // Parse HDOP
				p3 = strchr(p1, '.');
				if (p1 != NULL) *p3 = ' ';
				len = sscanf(p1,"%s%s",parse[0],parse[1]);
				if(len == 2) 
				{
					inputVar[0] = atoi(parse[0]);
					inputVar[1] = atoi(parse[1]);
					setArray(iDop,2,inputVar);
				}
				break;
			case 9: // Antenna height
				setVariable(iHeigth,0,atoi(p1));
				break;
			case 10: //Antenna height difference
				setVariable(iAlt,0,atoi(p1));
				break;
			default:
				break;
		}
		if (p2 == NULL)
			break;
		param++;
	}
	return isOK;
}

/** \brief Parse GPRMC NMEA String 
 *
 * \param[in] char* inbuf;
 * NMEA input buffer
 * 
 * \return int status
 * Return success or not
 */
int parseGPRMC(char * inBuf)
{
// $GPRMC,180432,A,4027.027912,N,08704.857070,W, 000.04,181.9,131000,1.8,W,D*25
	//        hhmmss   ddmm.mmmmmm   dddmm,mmmmmm    knots   deg  ddmmyy, mag  A/D/N
	char isOK = 1;
	//char tmp[20][20]={{0},{0}};
	char parse[3][20]={{0},{0}};
	int inputVar[4];
	int len = 0;
	double dTemp = 0.0, mm;
	char *p1, *p2;
	int param = 0;
	struct tm tod;

	//Prepare parsing
	p2 = inBuf;
	p1 = strchr(p2, '*');
	if (p1 != NULL)
	// terminate before checksum
	*p1 = '\0';

	while (*p2 >= ' ')
	{ 
			// get string to next comma
		p1 = strsep(&p2, ",");
		switch (param)
		{ // decode message parameters
			case 0: // just the name
				if (strcmp(p1, "$GPRMC") != 0)
				{
					printf("this is not the right message type (expected $GPRMC, got %s\n", p1);
					isOK = 0;
				}
				break;
			case 1: // time of day
				len = sscanf(p1,"%2s%2s%lf",parse[0],parse[1],&dTemp);
				isOK = len == 3;
				if (isOK)
				{ 
					// reconstruct date and time (from time only)
					inputVar[0] = atoi(parse[0]);
					inputVar[1] = atoi(parse[1]);
					inputVar[2] = (int) dTemp;
					inputVar[3] = (int)(((dTemp - (int)dTemp) * 1e3) + 0.5); //Convert to ms
					//Set values for epoc tod
					tod.tm_hour = inputVar[0];
					tod.tm_min  = inputVar[1];
					tod.tm_sec  = (int) dTemp;
					setArray(iTime,4,inputVar);
				}
				break;
			case 2:  //Convert the validity of the measurement
				if(*p1 == 'A') // valid
					setVariable(iFixvalid,0,1);
				else //(V = warning, or other value)
					setVariable(iFixvalid,0,0);
				break;
			case 3: // Parse latitude
				sscanf(p1, "%2s%lf", parse[0], &mm);
				dTemp = mm /60.0;
				inputVar[0] = atoi(parse[0]);
				inputVar[1] = (int)(((dTemp - (int)dTemp) * 1e6) +0.5); //Convert to *micro* deg
				setArray(iLat,2,inputVar);
				break;
			case 4: // N or S
				if (*p1 == 'S') setVariable(iLat,0,-getReadVariable(iLat,0)); //Invert lat
				break;
			case 5: // Parse lontitude (3 digits degrees)
				sscanf(p1, "%3s%lf", parse[0], &mm);
				dTemp =  mm / 60.0;
				inputVar[0] = atoi(parse[0]);
				inputVar[1] = (int)(((dTemp - (int)dTemp) * 1e6) +0.5); //Convert to *micro* deg
				setArray(iLong,2,inputVar);
				break;
			case 6: // E or W
				if (*p1 == 'W') setVariable(iLong,0,-getReadVariable(iLong,0)); //Invert lat
				break;
			case 7: // Convert speed
				dTemp = atof(p1) * KNOT2MS;
				inputVar[0] = (int)dTemp;
				inputVar[1] = (int)(((dTemp - (int)dTemp) * 1e3) +0.5); //Convert to mm/s
				setArray(iSpeed,2,inputVar);
				break;
			case 8: // Convert heading
				dTemp = atof(p1);
				inputVar[0] = (int)dTemp;
				inputVar[1] = (int)(((dTemp - (int)dTemp) * 1e3) +0.5); //Convert to m deg
				setArray(iHeading,2,inputVar);
				break;
			case 9: //Convert date
				len = sscanf(p1,"%2s%2s%2s",parse[0],parse[1],parse[2]);
				if(len == 3)
				{
					inputVar[0] = atoi(parse[0]);
					inputVar[1] = atoi(parse[1]);
					inputVar[2] = atoi(parse[2]) + 2000;
					//Set values for epoc tod
					tod.tm_year = inputVar[2] - 1900; //year from 1900
					tod.tm_mon  = inputVar[1] - 1; //Month 0-11
					tod.tm_mday = inputVar[0];
					setArray(iDate,3,inputVar);
					inputVar[0] = mktime(&tod);
					inputVar[1] = (int)(((dTemp - (int)dTemp) * 1e6) + 0.5); //Convert to us
					if (inputVar[0] != -1) setArray(iTod,2,inputVar);
				}
				break;
			default: // ignore the rest
				break;
		}
		if (p2 == NULL)
			break;
		param++;
	}
	return isOK;
}

/** \brief Convert lat/long to UTM coordinates
 * 
 * This is purely adapted from AUGps, and the algorithm is NOT
 * understood, please check for correctness of the data
 * 
 * \return int status
 * Return success or not
 */
int latlon2UTM(void)
{
	//Internal variables
	double lat = 0, /*lon = 0,*/  zone_CM = 0, delta_lon = 0;
	double longDeg, latDeg;
	int inputVar[2];

	//Datum constants
	double a = 6378137.0;
	double b = 6356752.314;

	double k0 = 0.9996;    //Scale factor
	double e = sqrt(1-pow((b/a),2)); //Eccentricity
	double e_2 = e*e/(1-e*e);
	double n = (a-b)/(a+b);
	double nu = 0.0;

	//Calcualte Meridional Arc Length
	double A0 = a*(1-n+ (5*n*n/4)*(1-n) + (81*pow(n,4)/64)*(1-n));
	double b0 = (3.0 * a * n / 2.0) * (1 - n - (7.0 * n * n / 8.0) * (1.0 - n) + 55.0 * pow(n,4.0) / 64.0);
	double C0 = (15*a*n*n/16)*(1 - n +(3*n*n/4)*(1-n));
	double D0 = (35*a*pow(n,3)/48)*(1 - n + 11*n*n/16);
	double E0 = (315*a*pow(n,4)/51)*(1-n);
	double S = 0;

	//Calculate constants
	double p = 0, sin_1=0;

	//Coefficients for UTM coordinates
	double Ki = 0, Kii = 0, Kiv = 0, Kv = 0;
	double Kiii = 0;

	//Get lat from variable database
	latDeg  = ((double)getReadVariable(iLat,0) + ((double)getReadVariable(iLat,1)/1e6));
	longDeg = ((double)getReadVariable(iLong,0) + ((double)getReadVariable(iLong,1)/1e6));
	lat = latDeg * M_PI / 180.0;
	//lon = longDeg * M_PI / 180.0;
	zone_CM = 6.0 * utmZone - 183.0;
	delta_lon = longDeg - zone_CM;
	nu = a/sqrt(1.0 - pow((e * sin(lat)),2));

	//Calcualte Meridional Arc Length
	S=A0*lat - b0*sin(2*lat) + C0*sin(4*lat) - D0*sin(6*lat) + E0*sin(8*lat);

	//Calculate constants
	p = delta_lon * 3600.0/10000.0;
	sin_1=M_PI/(180.0 * 3600.0);

	//Coefficients for UTM coordinates
	Ki=S*k0;
	Kii=nu*sin(lat)*cos(lat)*pow(sin_1,2)*k0*1e8/2.0;
	Kiii=((pow(sin_1,4)*nu*sin(lat)*pow(cos(lat),3))/24.0)*(5.0-pow(tan(lat),2)+9.0*e_2*
	pow(cos(lat),2)+4.0*e_2*e_2*pow(cos(lat),4))*k0*1e16;
	Kiv=nu*cos(lat)*sin_1*k0*1e4;
	Kv=pow(sin_1 * cos(lat),3) * (nu / 6.0) * (1-pow(tan(lat),2)+e_2*pow(cos(lat),2))*k0*1e12;
	//Transfer the data to database UTM Variables
	//Northing
	inputVar[0] = (int)(Ki + Kii * p * p + Kiii * pow(p, 4));
	inputVar[1] = (int)(((Ki + Kii * p * p + Kiii * pow(p, 4)) - inputVar[0]) * 1e6 + 0.5); //In µdeg
	setArray(iUTMn,2,inputVar);
	//Easting
	inputVar[0] = (int)(500000.0 + (Kiv * p + Kv * pow(p,3)));
	inputVar[1] = (int)(((500000.0 + (Kiv * p + Kv * pow(p,3)))- inputVar[0]) *1e6 + 0.5); //In µdeg
	setArray(iUTMe,2,inputVar);

	return 1;
}

/** \brief Convert UTM to lat/long coordinates
 * 
 * This is purely adapted from AUGps, and the algorithm is NOT
 * understood, please check for correctness of the data
 * 
 * \return int status
 * Return success or not
 */
int utm2latlon(void)
{
	//Internal variables
	double latitude, longitude;
	int inputVar[2];

	// signed in meters from central meridian
	double x = ((double)getReadVariable(iUTMe,0) + ((double)getReadVariable(iUTMe,1)/1e6)) - 500000.0; 
	double y = ((double)getReadVariable(iUTMn,0) + ((double)getReadVariable(iUTMn,1)/1e6));
	// center meridian in radians
	double zone_CM = (6.0 * utmZone - 183.0) * M_PI / 180.0;
	//Datum constants
	double a = 6378137.0;  // WGS84 equatorial radius
	double b = 6356752.314;// polar radius
	double k0 = 0.9996;    //Scale factor - along longitude 0
	double e = sqrt(1.0 - sqrt(b) / sqrt(a)); // excentricity ~ 0.08
	// meditorial arc
	double M = y/k0;
	// footprint latitude
	double mu = M/(a*(1.0 - sqrt(e)/4.0 - 3.0 * pow(e,4) / 64.0 -
		5.0 * pow(e,6) / 256.0));
	double e1 = (1.0 - sqrt(1.0 - sqrt(e)))/(1.0 + sqrt(1.0 - sqrt(e)));
	double j1 = (3.0 * e1 / 2.0 - 27.0 * pow(e1, 3) / 32.0);
	double j2 = (21.0 * sqrt(e1) / 16.0 - 55.0 * pow(e, 4) / 32.0);
	double j3 = (151.0 * pow(e1, 3) / 96.0);
	double j4 = (1097.0 * pow(e1, 4) / 512.0);
	double fp = mu + j1 * sin(2.0 * mu) + j2 * sin(4.0 * mu) +
		j3 * sin(6.0 * mu) + j4 * sin(8.0 * mu);
	// and now lat-long
	double e2 = sqrt(e * a / b);
	double c1 = e2 * sqrt(cos(fp));
	double t1 = sqrt(tan(fp));
	double r1 = a * (1.0 - sqrt(e))/pow(1.0 - sqrt(e) * sqrt(sin(fp)), 3.0/2.0);
	double n1 = a / sqrt(1.0 - sqrt(e) * sqrt(sin(fp)));
	double d  = x / (n1 * k0);
	double q1 = n1 * tan(fp) / r1;
	double q2 = sqrt(d)/2.0;
	double q3 = (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * sqrt(c1) - 9.0 * e2) *
		pow(d, 4) / 24.0;
	double q4 = (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * sqrt(t1) -
			   3.0 * sqrt(c1) - 252.0 * e2) * pow(d, 6) / 720.0;
	double q5 = d;
	double q6 = (1.0 + 2.0 * t1 + c1) * pow(d, 3) / 6.0;
	double q7 = (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * sqrt(c1) +
			   8.0 * e2 + 24.0 * sqrt(t1)) * pow(d, 5) / 120.0;
	double lat = fp - q1*(q2 - q3 + q4);
	double lon = zone_CM + (q5 - q6 + q7) / cos(fp);
	//
	latitude = lat * 180.0 / M_PI;
	longitude = lon * 180.0 / M_PI;

	//Write to database
	inputVar[0] = (int)latitude;
	inputVar[1] = (int)((latitude - inputVar[0]) * 1e6 + 0.5);
	setArray(iLat,2,inputVar);
	inputVar[0] = (int)longitude;
	inputVar[1] = (int)((longitude - inputVar[0]) * 1e6 + 0.5);
	setArray(iLong,2,inputVar);

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
void XMLCALL gpsStartTag(void *, const char *, const char **);
void XMLCALL gpsEndTag(void *, const char *);

/** \brief Initialize the GPS HAL
 *
 * Reads the XML file and sets up the GPS settings
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
  printf("GPS: Initializing Serial GPS HAL %s.%s\n",GPSVERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "GPS: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, gpsStartTag, gpsEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("GPS: Error reading: %s\n",filename);
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
		printf("   Error: No <gps> XML tag found in plugins section\n");
		return -1;
	}

  //Start crossbow thread after init
  if (xmlParse.enable) done = initGps();



 return done;
}

void XMLCALL
gpsStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("gps",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("gps",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   GPS: Use of GPS disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(gpsDevString,attr[i+1],63); 
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate = atoi(attr[i+1]); 
    printf("   GPS: Serial port %s at %d baud\n",gpsDevString,baudrate);
  }
   else if (strcmp("connection",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("con",attr[i]) == 0) strncpy(connection,attr[i+1],63); 
    
    printf("   GPS: Connection  %s\n",connection);
  } 
  else if (strcmp("socket",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("portIP",attr[i]) == 0) portIP = atoi(attr[i+1]); 
    for(i = 0; attr[i]; i+=2) if (strcmp("ip",attr[i]) == 0) strncpy(ip,attr[i+1],63); 
    printf("   GPS: TCP/IP %s at %d port\n",ip,portIP);
  }
  
  

  else if (strcmp("utmzone",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("value",attr[i]) == 0) utmZone = atoi(attr[i+1]); 
    printf("   GPS: Default UTM Zone: %d\n",utmZone);
  }
  //else if(socket)
  //else if(connection)

}

void XMLCALL
gpsEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
