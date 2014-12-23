/** \file roboteq.c
 *  \ingroup hwmodule
 *  \brief RoboTeQ Motor Controller Interface (Phoenix - University of Hohenheim)
 *
 * RoboTeQ Motor Controller H1650 Interface for RHD.
 * RobotTeQ will be referred to as rtq.
 *
 * This plugin is designed to be used with a computer with two serial ports. So each motorcontroller has its own connection.
 * If the RoboTeq controllers are used with another computer configuration this plugin needs to be changed.
 *
 * It is designed for the Armadillo Scout, which SDU build for UniHo, and UniHo changed to the Phoenix. It is the first robot that can be controlled with both MobotWare and FroboMind
 *
 * Armadillo Scout Official webpage: https://mpt.uni-hohenheim.de/90593?&L=1
 * Armadillo Scout Documentation webpage: http://mpt-internal.uni-hohenheim.de/doku.php?id=robots:armadillo:welcome
 * 
 * Phoenix Official webpage:
 * Phoenix Documentation webpage:
 *
 *	\todo Implementation of debug printout using DEBUG define. Specify what should be printed.
 *  \todo Move gear-ratio, drivewheelradius and MAXRPM to conf-file instead of sourcecode.
 *  \todo Handle firmware string if rtq is reset when the rhd is running. At startup the rtq sends a string with firmware version, this is handled by starting the rhd after the rtq. If the rtq for some reason is restarted while the rhd is running the plugin will crash. This needs to be handled.
 *  \todo add option so the speed command is open or closed loop.
 *
 *  \author Claes Dühring Jæger
 *  $Rev: 436 $
 *  $Date: 2012-03-20 21:54:25 +0100 (Tue, 20 Mar 2012) $
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
 #define RTQVERSION 	      "1.1"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 436 $"
 #define DATE             "$Date: 2012-03-20 21:54:25 +0100 (Tue, 20 Mar 2012) $"
 #define ID               "$Id: roboteq.c 436 2014-03-06 12:11:04Z cjag $"
/***************************************************************************/

#include <stdlib.h>
#include <stdio.h>

//serial com
#include <fcntl.h>
#include <termios.h> //#include <asm/termbits.h> is included in termios
#include <sys/ioctl.h>

//#include <sys/types.h>
//#include <sys/stat.h>

#include <pthread.h>
#include <string.h>
#include <linux/serial.h>
#include <expat.h>
#include <unistd.h>
//#include <math.h>
//#include <time.h>
//#include <sys/time.h>
//#include <unistd.h>
//#include <errno.h>
//#include <signal.h>
//#include <netdb.h>


//RHD Core headers
#include <rhd.h>
#include <database.h>
#include <globalfunc.h>

//Plugin Header
#include "roboteq.h"

//Debugging flag - Enables printout
#define DEBUG 0					//!<Set to 1 to enable debug printout.


//Definitions
#define BAUDRATE 115200 		//!< Controller demands this baudrate. It cannot be changed.
#define RTQSTRINGSIZE 128		//!< Maximum stringsize from controller
#define READENCODERRELATIVE 1	//!< Read the value relative to the last read command
#define READENCODERABSOLUTE 2	//!< Read the absolute value of the encoder
#define MAXRPM 6000				//!< Max RPM configured for the motorcontrollers
#define PI 3.141592 //!<Pi

/******** Global Variables *************/
int speedLeft = 0;				//!< Speed received from the MRC to the left rtq
int speedLeftUpdated = 0;		//!< Set when the speed has been updated in periodic()
int speedRight = 0;				//!< Speed received from the MRC to the right rtq
int speedRightUpdated = 0;		//!< Set when the speed has been updated in periodic()
int resetMotorLeft = 0;			//!< Set when the MRC reset the hall sensor counters
int resetMotorRight = 0;		//!< Set when the MRC reset the hall sensor counters
int motorLeftInverted = 1;		//!< Set the direction of the speed and encoder. If 1 -> not inverted; if -1 inverted
int motorRightInverted = 1;		//!< Set the direction of the speed and encoder. If 1 -> not inverted; if -1 inverted

static volatile char rtqRunning = 0;	//!< used to check if the thread is running or not

/******** RHD Variables *************/
int iEncl; 						//!< Hall counter left
int iEncr; 						//!< Hall counter right
int iSpl; 						//!< Speed left
int iSpr; 						//!< Speed Right
int iSteeringangleref; 			//!< ??
int iRstl; 						//!< Reset RTQ left
int iRstr; 						//!< Reset RTQ right

/******** RS232 Variables *************/
int  rtqDevLeft;  				//!<rtq Port file pointer mc-left
int  rtqDevRight;           	//!<rtq Port file pointer mc-right
char rtqDevStringLeft[64]; 		//!<String to hold rtq device mc-left
char rtqDevStringRight[64]; 	//!<String to hold rtq device mc-right
struct termios oldtio,newtio;	//!<Struct with the serial settings

/******** Variables *************/
int readType = 0;	//!< Depending on the control parameter in <odometry> in robot.conf file. If 0 readType = READENCODERABSOLUTE; else readType = READENCODERABSOLUTE

//Function Prototype
int initrtq(void);
int shutdownRTQ(void);

void *rtq_task(void *);

int readEncoder(int,int);
int setSpeed(int,int);

void readControllerStart(int);
int stopControllerEcho(int,char*);
int resetMotor(int);

double gearRatio(void);

int readSerial(int,char*);

// Threads are being defined
pthread_t rtq_thread;			//!<Main thread for rtq
pthread_attr_t attr;			//!<??


/** \brief Initialize RTQ plugin, settings and communication
 *
 * The com-ports are opened, the echo from the rtq stopped, variables in rhd is initialized and the thread is started.
 *
 * \returns int status
 * Status of the server thread - negative on error.
 */
int initrtq(void){
	//*************Left controller************************//
	
	rtqDevLeft = open(rtqDevStringLeft, O_RDWR | O_NOCTTY); // Open RS232 port - see http://linux.die.net/man/3/open
	if (rtqDevLeft<0)
	{
		fprintf(stderr,"RTQ: Error opening %s - Left Motor Controller\n",rtqDevStringLeft);
		return -1;
	}
	
	//Set baudrate for RTQ unit Left
	if (set_serial(rtqDevLeft,BAUDRATE) == -1)  {
		fprintf(stderr," RTQ: Can't set rtq-left serial port parameters\n");
		return -1;
	}
	//*************Left controller************************//
	//*************Right controller************************//
	
	rtqDevRight = open(rtqDevStringRight, O_RDWR | O_NOCTTY); // Open RS232 port - see http://linux.die.net/man/3/open
	if (rtqDevRight<0)
	{
		fprintf(stderr,"RTQ: Error opening %s - Right Motor Controller\n",rtqDevStringRight);
		return -1;
	}
	
	//Set baudrate for RTQ unit Right
	if (set_serial(rtqDevRight,BAUDRATE) == -1)  {
		fprintf(stderr," RTQ: Can't set rtq-right serial port parameters\n");
		return -1;
	}
	//*************Right controller************************//

	//************* Init commands to controllers***********//
	//Uncomment if the controllers are turned on after the RHD is started
	//readControllerStart(rtqDevLeft);
	//readControllerStart(rtqDevRight);

	stopControllerEcho(rtqDevLeft,rtqDevStringLeft);
	stopControllerEcho(rtqDevRight,rtqDevStringRight);
	

	/****** Create database variables if all is ok **************/
	  iEncl = createVariable('r',1,"encl");
	  iEncr = createVariable('r',1,"encr");
	  iSpl  = createVariable('w',1,"speedl");
	  iSpr  = createVariable('w',1,"speedr");
	  iRstl  = createVariable('w',1,"resetmotorl");
	  iRstr  = createVariable('w',1,"resetmotorr");
	  iSteeringangleref  = createVariable('w',1,"steeringangleref");

	// Initialization and starting of threads
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	if (pthread_create(&rtq_thread, &attr, rtq_task, 0))
	{
		perror("RTQ: Can't start RTQ thread");
		return -1;
	}
	rtqRunning = 1;
	return 1;

}

/** \brief Initialize Shut down rtq thread
 *
 * \returns int status
 * Status of the server thread - negative on error.
 */
int shutdownRTQ(void) {
  rtqRunning = 0;
  pthread_join(rtq_thread,NULL);

  return 1;
}

/** \brief RTQ  thread.
 * 
 * The rtq only runs very shortly for each speed command therefor the speed is sent to the rtq in this thread every time the loop is running.
 * 
 * The resetmotors sets the encoder values to 0.
 */
void *rtq_task(void *not_used) {
	#ifdef DEBUG
	printf("RTQ-Plugin Running\n");
	#endif

	int sleepTime = 500;

	while(rtqRunning){

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

		//sleep so the loop is not using all the calculation power
		usleep(sleepTime);
	}

	close(rtqDevLeft);
	close(rtqDevRight);
	fprintf(stderr,"RTQ: Shutdown RTQ task\n");
	pthread_exit(0);
}


/** \brief is called form the rhd every sample
 * 
 *  Reads the values that is written to the plugin variables.
 * 
 * 
 * */
extern int periodic(int tick)
{
	// Check if variables are updated
	if (isUpdated('w',iSpl)){
		speedLeft = motorLeftInverted * getWriteVariable(iSpl,0);
		speedLeftUpdated = 1;
	}
	if (isUpdated('w',iSpr)){
		//speedRight = - getWriteVariable(iSpr,0);
		speedRight = motorRightInverted * getWriteVariable(iSpr,0);
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
	}

	return 1;
}

/** \brief Read the first 2 lines from the rtq
 *
 *The RTQ send two lins of data during upstart. Read them to make sure they are not stuck in the buffer.
 */
 /*
void readControllerStart(int rtqDev){
	char buf[RTQSTRINGSIZE];

	memset(buf,0,RTQSTRINGSIZE);
	readSerial(rtqDev,buf);
	printf("RTQ: Start Status: %s\n",buf);

	memset(buf,0,RTQSTRINGSIZE);
	readSerial(rtqDev,buf);
	printf("RTQ: Start Status: %s\n",buf);

}*/

/** \breif Sends commands to the Motor Controllers to stop echo
 *
 * By default the controllers echo all commands sent to them over the serial interface.
 * To avoid this send "^echof 1\r". The controllers reply with "+\r" if the command is acknowledged.
 * 
 * This function makes sure the echo is off by lookinf at the following commands.
 * 
 * If the echo is on the controller will send:
 * ^echof 1\r
 * +\r
 * 
 * If the echo is off it will send:
 * +\r
 *	
 * \param rtqDev The identifier for the serial port
 * \param rtqDevName The port name in linux, used for output to tell the user if it is the left or right controller.
 */
int stopControllerEcho(int rtqDev, char* rtqDevName){
	char stopEcho[] = "^echof 1\r";
	char buf[RTQSTRINGSIZE];
	
	secureWrite(rtqDev,stopEcho,strlen(stopEcho)); 				// Send stop echo command
	
	memset(buf,0,RTQSTRINGSIZE);								//Reset buffer so it is ready to receive
	readSerial(rtqDev,buf);										//Read response from controller
	
	if(strncmp(buf,stopEcho,8)==0){								//Check response to see if the controller echoed the command
		memset(buf,0,RTQSTRINGSIZE);							//Reset buffer so it is ready to receive
		readSerial(rtqDev,buf);									//If the echo is on read again
	}
	
	if(strncmp(buf,"+",1)==0){									//Check if the controller acknowlegded the command
		printf("RTQ: Echo status %s: Echo off\n",rtqDevName);
		return 1;
	}
	else {
		fprintf(stderr,"RTQ: Error turning off echo on %s\n",rtqDevName);
		return 0;
	}

}

/** \breif Reset the counters on the rtq
 *
 */
int resetMotor(int rtqDev){
	char resetMotor[] = "!CB 0\r";
	char buf[RTQSTRINGSIZE];

	secureWrite(rtqDev,resetMotor,strlen(resetMotor));			//Send command


	memset(buf,0,RTQSTRINGSIZE);
	readSerial(rtqDev,buf);
	if(strncmp(buf,"+",1)==0){									//Check if the controller acknowlegded the command
		printf("RTQ: Motor %d reset: %1s \n",rtqDev,buf);
		return 1;
	}
	else {
		fprintf(stderr,"RTQ Error: Motor %d not\n",rtqDev);
		return 0;
	}
}

/** \breif Sends the speed commends to the motors
 *
 * v: Speed
 * r: Radius
 * O: circumference
 * R: Gear Ratio
 * Formula: rpm=v/O*R*60sec
 *
 * \todo Read the max RPM from the controller and set it during init
 */

int setSpeed(int rtqDev, int speed){
	char motorSpeedCom[]="!G ";
	char intSpeed[RTQSTRINGSIZE];
	char motorSpeedCommand[RTQSTRINGSIZE];
	char buf[RTQSTRINGSIZE];
	double driveWheelRadius = 0.165; 		//!< Radius of drivewheel + belt in meters.
	double driveWheelCircumvence = 0.0; 	//!< Circumvence of the drivewheel in meters.
	double recalculatedSpeed = 0.0;			//!< Placeholder to change speed to from cm/s to m/s.
	double motorRPM = 0.0;					//!< The rmp the motor should be running with in order to maintain current speed
	int motorRPMrecal = 0;					//!< Recalculte the rmp to a value in the range [0-1000]. The rtq expects this range as speed input.
	double gearRatio = 30.0;				//!< Gear-ratio from motor to drivewheel.


	motorSpeedCommand[0]='\0';

	recalculatedSpeed = speed / 100.0;	//change speed to from cm/s to m/s
	driveWheelCircumvence = 2*PI*driveWheelRadius;
	
	motorRPM = recalculatedSpeed/driveWheelCircumvence*gearRatio*60; //calculate the motorspeed in rpm
	motorRPMrecal = (motorRPM*1000.0)/MAXRPM; //6000 is max rpm set in the controllers
	//motorRPMrecal = motorRPMrecal*gearRatioFactor;//multiplyed by gearRatioFactor to get correct actual speed. The value is based on measurements and calculation made with roboteq utility. This needs to be checked, maybe it is the gearing that is the problem
	//printf("mrpm %f \n",motorRPM);

	if(motorRPMrecal>0 && motorRPMrecal<985){
		motorRPMrecal=motorRPMrecal+35;
	}else if(motorRPMrecal<0 && motorRPMrecal>-985){
		motorRPMrecal=motorRPMrecal-35;
	}

	sprintf(intSpeed,"%d\r",motorRPMrecal); 		//convert int to char and add \r for motorcontroller
	strcat(motorSpeedCommand,motorSpeedCom); 	//add motorcontroller command to command
	strcat(motorSpeedCommand,intSpeed); 			//add speed to motorcontroller command

	//printf("RTQ: Motor command: %s \n",motorSpeedCommand);

	secureWrite(rtqDev,motorSpeedCommand,strlen(motorSpeedCommand));
	//int tcflush(rtqDev, TCOFLUSH);

	memset(buf,0,RTQSTRINGSIZE);
	readSerial(rtqDev,buf);
	//printf("RTQ: Speed: %s \n", buf);
	//int tcflush(rtqDev, TCIFLUSH);	

	return 1;

}


/** \breif Read the values from the encoders/hall sensor counters
 *
 * \param rtqDev The controller to send the command to, left or right.
 * \param readType Read absolute or relative value from the controller
 */
int readEncoder(int rtqDev, int readType){
	char buf[RTQSTRINGSIZE];
	int reply=0;
	char sendCB[]="?CB\r"; 			//!< Command to read absolute value of Hallsensor counter
	char sendCBR[]="?CBR\r";		//!< Command to read the Hallsensor coubnter relative to last read
	if(readType == READENCODERRELATIVE){
		reply = secureWrite(rtqDev,sendCBR,strlen(sendCBR));
		//int tcflush(rtqDev, TCOFLUSH);
	}
	else if (readType == READENCODERABSOLUTE){
		reply = secureWrite(rtqDev,sendCB,strlen(sendCB));
		//int tcflush(rtqDev, TCOFLUSH);
	}
	else {
		reply =-1;
		rtqRunning = 0; //Shutdown if read-error
		fprintf(stderr,"RTQ: Error in reading type, shutting down\n");
	}
	memset(buf,0,RTQSTRINGSIZE);
	readSerial(rtqDev,buf);
	//int tcflush(rtqDev, TCIFLUSH);	

	//printf("RTQ: Value: %s \n",buf);
	//printf("Controller: n value: %d \n",n);

	/************Parse input data******************************/
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
				setVariable(iEncl,0, motorLeftInverted*dTempConvert);
				//printf("RTQ: Set left encoder: %d \n",dTempConvert);
				dTempLeft=dTemp;
			}
		}
		else if(rtqDev == rtqDevRight){
			if(dTemp != dTempRight){
				setVariable(iEncr,0,motorRightInverted*dTempConvert);
				//printf("RTQ: Set right encoder: %d \n",dTempConvert);
				dTempRight=dTemp;
			}
		}
	}

	return reply;
}

/** \breif Read serial data
 *
 * Read data from the serial buffer until '\r'. The rtq always ends its data with carriage return.
 */
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
	
	//insert flush here
	return reply;
}

/** \breif Write serial data
 * 
 * Write data to the serial port, end with a flush.
 * 
 * \todo think a bit more about this so it can work correct.
 */
 /*
 int writeSerial(int rtqDev, char* bufPoint) {
	 
	 //secureWrite(rtqDev,resetMotor,strlen(resetMotor));			//Send command
	//int tcflush(rtqDev, TCOFLUSH);
 }*/

/************************** XML Initialization **************************/
///Struct for shared parse data
typedef struct  {
    int depth;
    char skip;
    char enable;
		char found;
  }parseInfo;

//Parsing functions
void XMLCALL rtqStartTag(void *, const char *, const char **);
void XMLCALL rtqEndTag(void *, const char *);

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
  printf("RTQ: Initializing Serial RTQ Motor Controller %s.%s\n",RTQVERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "RTQ: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, rtqStartTag, rtqEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("RTQ: Error reading: %s\n",filename);
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
    fprintf(stderr, "RTQ: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <rtq> XML tag found in plugins section\n");
		return -1;
	}

  //Start rtq thread after init
  if (xmlParse.enable) done = initrtq();



 return done;
}

void XMLCALL
rtqStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("rtq",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("rtq",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1;
    }
    if (!info->enable) {
      printf("   rtq: Use of rtq disabled in configuration\n");
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("portleft",attr[i]) == 0){strncpy(rtqDevStringLeft,attr[i+1],63);printf("RTQ: Serial port-left %s \n",rtqDevStringLeft);}
    for(i = 0; attr[i]; i+=2) if (strcmp("portright",attr[i]) == 0){strncpy(rtqDevStringRight,attr[i+1],63);printf("RTQ: Serial port-right %s \n",rtqDevStringRight);}
    //for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate = atoi(attr[i+1]);
    //printf("   rtq: Serial port %s at %d baud\n",rtqDevString,baudrate);
  } else if (strcmp("var",el) == 0) {
	//Check for the correct depth for this tag
		if(info->depth != 4) {
			printf("Error: Wrong depth for the %s tag\n",el);
		}
		for(i = 0; attr[i]; i+=2) if (strcmp("leftinvert",attr[i]) == 0){ if(strcmp("true",attr[i+1])==0){motorLeftInverted = -1;}printf("RTQ: Left Motor Invert Status (-1:inverted;1:not inverted): %d \n", motorLeftInverted);}
		for(i = 0; attr[i]; i+=2) if (strcmp("rightinvert",attr[i]) == 0){ if(strcmp("true",attr[i+1])==0){motorRightInverted = -1;}printf("RTQ: Right Motor Invert Status (-1:inverted;1:not inverted): %d \n",motorRightInverted);}
  } else if (strcmp("encoder",el) == 0) {
	  if(info->depth != 4) {
			printf("Error: Wrong depth for the %s tag\n",el);
		}
		for(i = 0; attr[i]; i+=2) if (strcmp("readtype",attr[i]) == 0){
			if(strcmp("abs",attr[i+1])==0){readType = READENCODERABSOLUTE;printf("RTQ: Encoder ReadStatus: Absolute\n");}
			else if(strcmp("rel",attr[i+1])==0){readType = READENCODERRELATIVE;printf("RTQ: Encoder ReadStatus: Relative\n");}			
			}	
  }
  

}

void XMLCALL
rtqEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
