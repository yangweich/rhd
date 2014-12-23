/** \file vectornav.c
 *  \ingroup hwmodule
 *  \brief VectorNav VM100-Rugged IMU
 *
 *
 *  \todo Add documentation
 *  \todo Create vars and parsers for all registers. At the moment only the default output can be parsed.
 *  \todo Set the delimiter in the conf-file and read it instead of hard coded.
 *
 *  \author Claes Dühring Jæger
 *  $Rev: 195 $
 *  $Date: 2013-05-17 15:33:42 +0200 (Fri, 17 May 2013) $
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
 #define VNVERSION 	      "1.0"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 195 $"
 #define DATE             "$Date: 2013-05-17 15:33:42 +0200 (Fri, 17 May 2013) $"
 #define ID               "$Id: vn100RHD.c 195 2013-05-17 13:33:42Z cjag $"
/***************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
//#include <conio.h>
#include <linux/serial.h>
#include <expat.h>
#include <unistd.h>
//#include <math.h>
//#include <time.h>
//#include <sys/time.h>
//#include <unistd.h>
//#include <errno.h>
//#include <termios.h>
//#include <sys/ioctl.h>
//#include <signal.h>
//#include <netdb.h>
#include <stdint.h>

//RHD Core headers
#include <rhd.h>
#include <database.h>
#include <globalfunc.h>

//Plugin Header
#include "vectornav.h"
#include "vn100RHD.h"

//Debugging flag - Enables printout
#define DEBUG 1				//!<Set to 1 to enable debug printout.


/************* Definitions ************/
//#define BAUDRATE 115200 		//!< Controller demands this baudrate. It cannot be changed.
#define VNSTRINGSIZE 128		//!< Maximum stringsize from the sensor
#define PI 3.141592 //!<Pi

/******** Global Variables *************/
static volatile char vnRunning = 0;	//!< used to check if the thread is running or not


/******** RHD Variables *************/
//Default
int32_t iYaw, iPitch, iRoll, iMagX, iMagY, iMagZ, iAccX, iAccY, iAccZ, iGyroX, iGyroY, iGyroZ;
//double iYaw, iPitch,iRoll;


/******** RS232 Variables *************/
int  vnDev;  				//!< VN100 Port file pointer
char vnDevString[64]; 		//!< String to hold vn device
int baudrate = 0;			//!< Baudrate for VN100

Vn100 vn100;				//!< the vn-100 device
VnYpr ypr;					//!< Yaw, pitch, roll struct

char vn100DefaultValues[VNSTRINGSIZE];


/******** Variables *************/
int readRegister = 50;						//!< The register to be read. See VN-100 Man. page 39. Range is 0-48, therefore initialisered to 50.
vn100Default storeDefault;					//!< Struct for storing the default values from the VN-100
//vn100Default *storeDefaultPointer = &sthoreDefault;

static char *ptr;

char delimiter = ','; 						//!< Delimiter in strings from VN-100
const char *pointDelimiter = &delimiter;	//!< Pointer to the delimiter

/*********** Threads are being defined **********/
pthread_t vn_thread;			//!<Main thread for
pthread_attr_t attr;			//!<??


/** \brief Initialize RTQ plugin
 *	Initialize settings and communication
 *
 *
 * \returns int status
 * Status of the server thread - negative on error.
 */
int initvn100(void){

	if(DEBUG){
		printf("VN-100: Init Start\n");
	}

	if(readRegister == 50){
		//Default State, open normal com-port
		vnDev = open(vnDevString, O_RDWR); // Open RS232 port - see http://linux.die.net/man/3/open
		if (vnDev<0) {
			fprintf(stderr,"VN-100: Error opening %s - IMU\n",vnDevString);
			return -1;
		}
		//Set baudrate for RTQ unit Right
		if (set_serial(vnDev,baudrate) == -1) {
			fprintf(stderr," RTQ: Can't set rtq-right serial port parameters\n");
			return -1;
		}
	}
	else {
		//not default, use VectorNav API to connect
		vn100_connect(&vn100, vnDevString, baudrate);
	}


	/****** Create database variables if all is ok **************/
	switch(readRegister){
				case 0 : //add code
						break;
				case 1 : //add code
						break;
				case 2 : //add code
						break;
				case 3 : //add code
						break;
				case 4 : //add code
						break;
				case 5 : //add code
						break;
				case 6 : //add code
						break;
				case 7 : //add code
						break;
				case 8 : //add code
						break;
				case 9 : //add code
						break;
				case 10 : //add code
						break;
				case 11 : //add code
						break;
				case 12 : //add code
						break;
				case 13 : //add code
						break;
				case 14 : //add code
						break;
				case 15 : //add code
						break;
				case 16 : //add code
						break;
				case 17 : //add code
						break;
				case 18 : //add code
						break;
				case 19 : //add code
						break;
				case 20 : //add code
						break;
				case 21 : //add code
						break;
				case 22 : //add code
						break;
				case 23 : //add code
						break;
				case 24 : //add code
						break;
				case 25 : //add code
						break;
				case 26 : //add code
						break;
				case 27 : //add code
						break;
				case 28 : //add code
						break;
				case 29 : //add code
						break;
				case 30 : //add code
						break;
				case 31 : //add code
						break;
				case 32 : //add code
						break;
				case 33 : //add code
						break;
				case 34 : //add code
						break;
				case 35 : //add code
						break;
				case 36 : //add code
						break;
				case 37 : //add code
						break;
				case 38 : //add code
						break;
				case 39 : //add code
						break;
				case 40 : //add code
						break;
				case 41 : //add code
						break;
				case 42 : //add code
						break;
				case 43 : //add code
						break;
				case 44 : //add code
						break;
				case 45 : //add code
						break;
				case 46 : //add code
						break;
				case 47 : //add code
						break;
				case 48 : //add code
						break;
				default:
					iYaw = createVariable('r',1,"vnYaw");
					iPitch = createVariable('r',1,"vnPitch");
					iRoll = createVariable('r',1,"vnRoll");
					iMagX = createVariable('r',1,"vnMagX");
					iMagY = createVariable('r',1,"vnMagY");
					iMagZ = createVariable('r',1,"vnMagZ");
					iAccX = createVariable('r',1,"vnAccX");
					iAccY = createVariable('r',1,"vnAccY");
					iAccZ = createVariable('r',1,"vnAccZ");
					iGyroX = createVariable('r',1,"vnGyroX");
					iGyroY = createVariable('r',1,"vnGyroY");
					iGyroZ = createVariable('r',1,"vnGyroZ");
						break;
			}

	// Initialization and starting of threads
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	if (pthread_create(&vn_thread, &attr, vn_task, 0))
	{
		perror("VN-100: Can't start VN-100 thread");
		return -1;
	}
	vnRunning = 1;
	return 1;

}

/** \brief Initialize Shut down rtq thread
 *
 * \returns int status
 * Status of the server thread - negative on error.
 */
int shutdownVN(void) {
  vnRunning = 0;
  pthread_join(vn_thread,NULL);

  return 1;
}

/** \brief RTQ  thread.
 */
void *vn_task(void *not_used) {
	#ifdef DEBUG
	printf("VN-100 Plugin Running\n");
	#endif

	int sleepTime = 500;
	int counter =0;

	while(vnRunning){

		switch(readRegister){
			case 0 : //add code
					break;
			case 1 : //add code
					break;
			case 2 : //add code
					break;
			case 3 : //add code
					break;
			case 4 : //add code
					break;
			case 5 : //add code
					break;
			case 6 : //add code
					break;
			case 7 : //add code
					break;
			case 8 : //add code
					break;
			case 9 : //add code
					break;
			case 10 : //add code
					break;
			case 11 : //add code
					break;
			case 12 : //add code
					break;
			case 13 : //add code
					break;
			case 14 : //add code
					break;
			case 15 : //add code
					break;
			case 16 : //add code
					break;
			case 17 : //add code
					break;
			case 18 : //add code
					break;
			case 19 : //add code
					break;
			case 20 : //add code
					break;
			case 21 : //add code
					break;
			case 22 : //add code
					break;
			case 23 : //add code
					break;
			case 24 : //add code
					break;
			case 25 : //add code
					break;
			case 26 : //add code
					break;
			case 27 : //add code
					break;
			case 28 : //add code
					break;
			case 29 : //add code
					break;
			case 30 : //add code
					break;
			case 31 : //add code
					break;
			case 32 : //add code
					break;
			case 33 : //add code
					break;
			case 34 : //add code
					break;
			case 35 : //add code
					break;
			case 36 : //add code
					break;
			case 37 : //add code
					break;
			case 38 : //add code
					break;
			case 39 : //add code
					break;
			case 40 : //add code
					break;
			case 41 : //add code
					break;
			case 42 : //add code
					break;
			case 43 : //add code
					break;
			case 44 : //add code
					break;
			case 45 : //add code
					break;
			case 46 : //add code
					break;
			case 47 : //add code
					break;
			case 48 : //add code
					break;
			default:
				readDefault();
					break;

		}

		//vn100_getYawPitchRoll(&vn100, &ypr);
		//printf("YPR: %+#7.2f %+#7.2f %+#7.2f\n", ypr.yaw, ypr.pitch, ypr.roll);
		//counter++;

		//if(counter>10) break;



		/*
		if(counter>100000){
			printf("RTQ-Plugin Running\n");
			printf("RTQ: Speed Left: %d \n",speedLeft);
			printf("RTQ: Speed Right: %d \n",speedRight);
				counter=0;
			}
			counter++;
		*/

		/*

		if(speedLeftUpdated){
			setSpeed(rtqDevLeft,speedLeft);
			//printf("RTQ: Speed Left: %d \n",speedLeft);
			speedLeftUpdated = 0;
		}
		if(speedRightUpdated){
			setSpeed(rtqDevRight,speedRight);
			//printf("RTQ: Speed Right: %d \n",speedRight);
			speedRightUpdated = 0;
		}
		if(resetMotorLeft>0){
			resetMotor(rtqDevLeft);
			resetMotorLeft = 0;
		}
		if(resetMotorRight>0){
			resetMotor(rtqDevRight);
			resetMotorRight=0;
		}

		readEncoder(rtqDevLeft, readType);
		readEncoder(rtqDevRight, readType);

		*/
		//sleep so the loop is not using all the calculation power

		usleep(sleepTime);
	}

	if(readRegister == 50) close(vnDev); //if default close the normal com-port
	else vn100_disconnect(&vn100);// otherwise close the port using the VectorNac API
	fprintf(stderr,"VN-100: Shutdown VN-100 task\n");
	pthread_exit(0);
}


///Transmit all data to serial bus (called periodically)
extern int periodic(int tick)
{
	//static int counter = 0;
	//check that we enter the function
	/*
	if(counter>100){
		printf("In periodic\n");
		counter=0;
	}
	counter++;
	*/

	/*
	// Check if variables are updated
	if (isUpdated('w',iSpl)){
		speedLeft = getWriteVariable(iSpl,0);
		speedLeftUpdated = 1;
	}
	if (isUpdated('w',iSpr)){
		speedRight = - getWriteVariable(iSpr,0);
		speedRightUpdated = 1;
	}
	if (isUpdated('w',iRstl)){
		resetMotorLeft = 1;
	}
	if (isUpdated('w',iRstr)){
		resetMotorRight = 1;
	}
	if (isUpdated('w',iSteeringangleref)){
		//??
	}*/

	return 1;
}

/** \breif Read the values from the encoders/hall sensor counters
 *
 * \param rtqDev The controller to send the command to, left or right.
 * \param readType Read absolute or relative value from the controller
 */

/*
int readEncoder(int rtqDev, int readType){
	char buf[RTQSTRINGSIZE];
	int reply=0;
	char sendCB[]="?CB\r"; 			//!< Command to read absolute value of Hallsensor counter
	char sendCBR[]="?CBR\r";		//!< Command to read the Hallsensor coubnter relative to last read
	if(readType == READENCODERRELATIVE){
		reply = secureWrite(rtqDev,sendCBR,strlen(sendCBR));
	}
	else if (readType == READENCODERABSOLUTE){
		reply = secureWrite(rtqDev,sendCB,strlen(sendCB));
	}
	else {
		reply =-1;
		rtqRunning = 0; //Shutdown if read-error
		fprintf(stderr,"RTQ: Error in reading type, shutting down\n");
	}
	memset(buf,0,RTQSTRINGSIZE);
	readSerial(rtqDev,buf);

	//printf("RTQ: Value: %s \n",buf);
	//printf("Controller: n value: %d \n",n);
*/
	/************Parse input data******************************/
/*
	double dTemp = 0.0;
	static double dTempLeft=0.0;
	static double dTempRight=0.0;
	int dTempConvert = 0;
	char *p1, *p2, *pend;

	if(strlen(buf)>=2){
		p1 = buf;
		p2 = strchr(p1, '=');
		pend = strchr(p1, '\r');
		dTemp = strtod(p2+1,&pend);
		dTempConvert = (int)dTemp;
		//dTempConvert = 500;
		//printf("RTQ: Parsed value: %d \n", dTempConvert);

		//Send value to the db so the MRC can read them
		if(rtqDev == rtqDevLeft){
			if(dTemp != dTempLeft){
				setVariable(iEncl,0,dTempConvert);
				//printf("RTQ: Set left encoder: %d \n",dTempConvert);
				dTempLeft=dTemp;
			}
		}
		else if(rtqDev == rtqDevRight){
			if(dTemp != dTempRight){
				setVariable(iEncr,0,-dTempConvert);
				//printf("RTQ: Set right encoder: %d \n",dTempConvert);
				dTempRight=dTemp;
			}
		}
	}

	return reply;
}
*/
/** \breif Read serial data
 *
 * Read data from the serial buffer until '\r'. The rtq always ends its data with carriage return.
 */
/*
int readSerial(int rtqDev, char* bufPoint){

	char tmp=0;
	int n=0;
	int reply=0;

	memset(bufPoint,0,RTQSTRINGSIZE);
	for(n = 0; tmp != '\r'; n++) {
		if(secureRead(rtqDev,&tmp,1) > 0) {
			bufPoint[n]=tmp;
			//bufPoint++;
			//printf("readSerial: %s \n",&tmp);
		}
		else {
			reply=-1;
			fprintf(stderr,"RTQ: Error reading from Serial port\n");
		}
	}

	bufPoint[n+1] = '\0';
	reply=n;
	return reply;
}
*/

int readDefault() {
	char tmp=0;
	int n=0;
	int reply = 0;

	memset(vn100DefaultValues,0,VNSTRINGSIZE);
	for(n = 0; tmp != '\n'; n++) {
		if(secureRead(vnDev,&tmp,1) > 0) {
			vn100DefaultValues[n]=tmp;
			//bufPoint++;
			//printf("readSerial: %s \n",&tmp);
		}
		else {
			reply=-1;
			fprintf(stderr,"VN-100: Error reading from Default Serial port\n");
		}
	}

	vn100DefaultValues[n+1] = '\0';
	reply = n;

	parseDefault();

	return reply;
}

int parseDefault() {
	int i;
	char *p_str, *token, *p1, *pend;

	// terminate before checksum
	p1 = strchr(vn100DefaultValues, '*');
	*p1 = '\0';

	//Tokenize the sting and save data in struct
	for (i = 1, p_str = vn100DefaultValues; ; i++, p_str = NULL) {
		token = mystrtok(p_str,pointDelimiter);
	    if (token == NULL) break;
	    //printf("parse: %d: %s \n",i,token);

	    switch(i){
	    case 1:
	    	storeDefault.command = token;
	    	//printf("Case 1: %s \n", storeDefault.command);
	    	break;
	    case 2:
	    	storeDefault.yaw = atof(token);
	    	//printf("Case 2: %f",storeDefault.yaw);
	    	break;
	    case 3:
	    	storeDefault.pitch = atof(token);
	    	break;
	    case 4:
	    	storeDefault.roll = atof(token);
	    	break;
	    case 5:
	    	storeDefault.magX = atof(token);
	    	break;
	    case 6:
	    	storeDefault.magY = atof(token);
	    	break;
	    case 7:
	    	storeDefault.magZ = atof(token);
	    	break;
	    case 8:
	    	storeDefault.accX = atof(token);
	    	break;
	    case 9:
	    	storeDefault.accY = atof(token);
	    	break;
	    case 10:
	    	storeDefault.accZ = atof(token);
	    	break;
	    case 11:
	    	storeDefault.gyroX = atof(token);
	    	break;
	    case 12:
	    	storeDefault.gyroY = atof(token);
	    	break;
	    case 13:
	    	storeDefault.gyroZ = atof(token);
	    	//printf("Case 13: %f",storeDefault.gyroZ);
	    	break;
	    default:
	    	break;
	    }
	}
	//printf("VN-100: %s",vn100DefaultValues);

	if(updateDefaultValues())return 1;
	else return -1;
}

int updateDefaultValues() {
	int tempVar[2];

	tempVar[0]  = (int32_t) 1000*storeDefault.yaw;
	setArray(iYaw,1,tempVar);
	tempVar[0] = (int32_t) 1000*storeDefault.pitch;
	setArray(iPitch,1,tempVar);
	tempVar[0]  = (int32_t) 1000*storeDefault.roll;
	setArray(iRoll,1,tempVar);
	tempVar[0]  = (int32_t) 10000*storeDefault.magX;
	setArray(iMagX,1,tempVar);
	tempVar[0]  = (int32_t) 10000*storeDefault.magY;
	setArray(iMagY,1,tempVar);	
	tempVar[0]  = (int32_t) 10000*storeDefault.magZ;
	setArray(iMagZ,1,tempVar);
	tempVar[0]  = (int32_t) 1000*storeDefault.accX;
	setArray(iAccX,1,tempVar);
	tempVar[0]  = (int32_t) 1000*storeDefault.accY;
	setArray(iAccY,1,tempVar);
	tempVar[0]  = (int32_t) 1000*storeDefault.accZ;
	setArray(iAccZ,1,tempVar);
	tempVar[0] = (int32_t) 1000000*storeDefault.gyroX;
	setArray(iGyroX,1,tempVar);
	tempVar[0] = (int32_t) 1000000*storeDefault.gyroY;
	setArray(iGyroY,1,tempVar);
	tempVar[0] = (int32_t) 1000000*storeDefault.gyroZ;
	setArray(iGyroZ,1,tempVar);

	//printf("Test typecast: %d \n",iPitch);

	return 1;
}

/*
 * \breif String Tokenizer
 *
 * Based on the code from http://simplestcodings.blogspot.de/2010/08/custom-string-tokenizer-in-c.html
 *
 * You have to use a loop that runs through the string to tokenize it. See parseDefault() for example.
 *
 */
char *mystrtok(char* string,const char *delim) {
    char *p;
    if(string != NULL) {
        ptr=string;
        p=string;
    }
    else {
    	if(*ptr == '\0') return NULL;
        p=ptr;
    }

    while(*ptr != '\0') {
            if(*ptr == *delim) {
                if(ptr == p) {
                    p++;
                    ptr++;
                }
                else
                {
                    *ptr='\0';
                    ptr++;

                    return p;
                }
            }
        ptr++;
    }
    return p;
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
void XMLCALL vnStartTag(void *, const char *, const char **);
void XMLCALL vnEndTag(void *, const char *);

/** \brief Initialize the RTQ with settings from configuration file.
 *
 * Reads the XML file and sets up the RTQ settings
 *
 * Finally the initialization of the RTQ plugin is started.
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
  printf("VN-100: Initializing VectorNav VN-100 IMU %s.%s\n",VNVERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "VN-100: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, vnStartTag, vnEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("VN-100: Error reading: %s\n",filename);
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
    fprintf(stderr, "VN-100: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <vn100> XML tag found in plugins section\n");
		return -1;
	}

  //Start vn thread after init

  if (xmlParse.enable) done = initvn100();

 return done;
}

void XMLCALL
vnStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("vn100",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("vn100",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1;
    }
    if (!info->enable) {
      printf("   VN-100: Use of vn100 disabled in configuration\n");
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0){strncpy(vnDevString,attr[i+1],63);printf("VN-100: Serial port %s \n",vnDevString);}
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0){baudrate = atoi(attr[i+1]); printf("VN-100: Baudrate: %d \n",baudrate);}
  }
  else if (strcmp("output",el) == 0) {
	  //Check for the correct depth for this tag
	  if(info->depth != 4) {
		  printf("Error: Wrong depth for the %s tag\n",el);
	  }
	  for(i = 0; attr[i]; i+=2) if (strcmp("regid",attr[i]) == 0){readRegister = atoi(attr[i+1]);printf("VN-100: Output register: %d \n",readRegister);}
  }

}

void XMLCALL
vnEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
