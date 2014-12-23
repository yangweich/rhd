/** \file rflex.c
 *  \ingroup hwmodule
 *  \brief Driver for iRobot rFlex research robots
 *
 *  This plug-in provides connection to the iRobot rFLEX robot-
 *  control system. It should basically support all rFLEX robots
 *  but is only tested on the iRobot ATRV-Jr
 * 
 *  Most code is ported from Carnegie Mellon Robot
 *  Navigation Toolkit (CARMEN) rflex-lib by Michael Montemerlo, 
 *  Nicholas Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 *  and Jared Glover (thank you guys!!)
 * 
 *  \author Anders Billesø Beck
 *  $Rev: 240 $
 *  $Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $
 *  
 */
/***************************************************************************
 *                  Copyright 2008 Anders Billesø Beck, DTU                *
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
/************************** Library version  ***************************/
#define RFLEXVERSION   		"3.238"
/************************** Version control information ***************************/
 #define REVISION         "$Rev: 240 $:"
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
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "rflex.h"

/******** Global variables *************/
int iTv, iTa, iTt, iTp, iRv, iRa, iRt, iRp, iSonar, iUseSonar, iCmdBrake, iBatt;
int iCmdTv, iCmdTa, iCmdTt, iCmdRv, iCmdRa, iCmdRt, iBump, iBrake, iBumpbrake;
int sDev = 0;  //File descriptors
char serialDev[64];
int baudrate;
volatile int rflexRunning = -1;
int odoPeriod = 0;
uint8_t txId = 0;
pthread_t rxThread;
pthread_attr_t attr;
///Default values for accelleration and torque
uint32_t defRotTorque, defRotAccl, defTransTorque, defTransAccl;
uint32_t	bumpBrake;


/******** Function prototypes *************/
int initRflex(void);
void *rxtask(void *);
//rFLEX functions
int computeCRC( unsigned char *buf, int nChars );
uint16_t convertBytes2UInt16( unsigned char *bytes );
uint32_t convertBytes2UInt32( unsigned char *bytes );
void convertUInt8( unsigned int i, unsigned char *bytes );
void convertUInt32(uint32_t l, unsigned char *bytes );
void cmdSend( int port, int id, int opcode, int len, unsigned char *data );
int parseBuffer( unsigned char *buffer, unsigned int len );
void parseMotReport( unsigned char *buffer );
void parseSonarReport( unsigned char *buffer );
void parseSystemReport( unsigned char *buffer );
void parseDioReport( unsigned char *buffer );
int  sonarMap(int sonarId);
void digitalIoOn(int period);
void digitalIoOff(void);
void brakeOn(void);
void brakeOff(void);
void motionSetDefaults(void);
void odometryOn (long period);
void odometryOff (void);
int sonarOn(void);
int sonarOff(void);
void getSystemReport(void);
int setVelocity(int tVel, int rVel);
int errcnt = 0;
//int flexPosRepCnt = 0;
//int flexRotRepCnt = 0;
//int iFlexPosRepCnt;
//int iFlexRotRepCnt;
int ierrcnt;
FILE * fil = NULL;

/** \brief Initialize communications and variables
 * 
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
int initRflex(void) {


	//Open first serial port
  if ((sDev = open (serialDev, O_RDWR /*| O_NONBLOCK*/)) == -1) {
  	fprintf(stderr,"   Can't open serial port: %s\n",serialDev);
    rflexRunning = -1;
    return -1;
  } else if (set_serial(sDev,baudrate) == -1) {
		fprintf(stderr,"   Can't set serial port parameters\n");
    fprintf(stderr,"   rFLEX is NOT running!\n");
    rflexRunning = -1;
    return -1;
  } else rflexRunning = 2;
 
	/****** Create database variables if all is ok **************/
  iTv 			= createVariable('r',1,"transvel");
	iTa 			= createVariable('r',1,"transaccl");
	iTt 			= createVariable('r',1,"transtorque");
	iTp 			= createVariable('r',1,"transpos");
  iRv 			= createVariable('r',1,"rotvel");
	iRa 			= createVariable('r',1,"rotaccl");
	iRt 			= createVariable('r',1,"rottorque");
	iRp 			= createVariable('r',1,"rotpos");
	iSonar 		= createVariable('r',17,"sonar");
	iBump     = createVariable('r',2,"bumper");
  iBrake		= createVariable('r',1,"brake");
	iBatt			= createVariable('r',1,"battvoltage");
  ierrcnt		= createVariable('r',1,"errcnt");
//   iFlexPosRepCnt	= createVariable('r',1,"posrepcnt");
//   iFlexRotRepCnt	= createVariable('r',1,"rotrepcnt");
  iUseSonar = createVariable('w',1,"usesonar");
  iCmdBrake	= createVariable('w',1,"cmdbrake");
	iCmdTv		= createVariable('w',1,"cmdtransvel");
	iCmdTa		= createVariable('w',1,"cmdtransaccl");
	iCmdTt		= createVariable('w',1,"cmdtranstorque");
	iCmdRv		= createVariable('w',1,"cmdrotvel");
	iCmdRa		= createVariable('w',1,"cmdrotaccl");
	iCmdRt		= createVariable('w',1,"cmdrottorque");	
	iBumpbrake	= createVariable('w',1,"bumperbrake");

	//Start RX thread
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED); 
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  if (pthread_create(&rxThread, &attr, rxtask, 0))
    {
      perror("   Can't start rFLEX receive thread");
      rflexRunning = -1;
      return -1;
    }

	while (rflexRunning > 1) usleep(1000); //Don't proceed before threads are running
  
	//Setup default parameters for robot
	odometryOn(odoPeriod);
	digitalIoOn(100000);
  brakeOff();
	getSystemReport();

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
		
		//Send velocity to robot
		if ((isUpdated('w',iCmdTv)) || (isUpdated('w',iCmdRv))) {
			setVelocity(getWriteVariable(iCmdTv,0),getWriteVariable(iCmdRv,0));
		}

		//Activate / deactivate sonar
		if (isUpdated('w',iUseSonar)) {
			if (getWriteVariable(iUseSonar,0)) {
				sonarOn();
			} else {
				sonarOff();
			}
		}

		//Activate / deactivate brake
		if (isUpdated('w',iCmdBrake)) {
			if (getWriteVariable(iCmdBrake,0)) {
				brakeOn();
			} else {
				brakeOff();
			}
		}
	
		//Activate / deactivate bumper brake
		if (isUpdated('w',iBumpbrake)) {
			bumpBrake = getWriteVariable(iBumpbrake,0);
		}
		
		//Request system report every 100 ticks
		if (!(tick %100)) getSystemReport();
               setVariable(ierrcnt, 0, errcnt);
//                setVariable(iFlexPosRepCnt, 0, flexPosRepCnt);
//                setVariable(iFlexRotRepCnt, 0, flexRotRepCnt);
  return 1;
}


/** \brief Recieve thread for rFLEX communication
 * 
 * \returns * void
 * Unused
 */
void *rxtask(void *something) 
{  //Set running back to 1 to indicate thread start
  rflexRunning = 1;
  unsigned char buf[MAX_COMMAND_LENGTH];
  unsigned char tmp;
  unsigned int n = 0;
  int escaped = 0;
//  fprintf(stdout, "trying to open rflex communication (receive) log 'rflex_rx.log'\n");
//  fil = fopen("rfles_rx.log", "w");
  if (fil != NULL)
    fprintf(fil, "received position and/or rotation data from rflex with 'crc calc and expected\n");
//   else
//     fprintf(stderr, "failed to open rflex communication log 'rflex_rx.log'\n");
  //Clear buffer
  memset(buf,0,MAX_COMMAND_LENGTH);
  
  //Everloop that recieves 
  while (rflexRunning > 0) {
    //Read one byte from serial bus
    if(secureRead(sDev,&tmp,1)>0) {
      buf[n]=tmp;
      n++;
    } else { //Error reading, exit thread!
      rflexRunning = -1;
      break;
    }
    //printf("0x%02X (%d) ",tmp,n);
    //Wait for start of package
    if (n == 2) {
      if ((buf[0] != B_ESC) && (buf[1] != B_STX)) {
        buf[0] = buf[1];
        n = 1; 
        errcnt++;
      }
    } else if (n > 2) 
    {
      if (escaped)
        escaped = 0;
      else if ((buf[n-2] == B_ESC) && (buf[n-1] == B_NUL))
      { // remove the B_NULL, but keep the B_ESC as data
        // and next char should not be tested for escape sequence
        escaped = 1;
        n--;
      }
      else if ((buf[n-2] == B_ESC) && (buf[n-1] == B_SOH))
      { // and B_ESC B_SOH should just be removed - but why?
        escaped = 1;
        n -= 2;
      }
      else if ((buf[n-2] == B_ESC) && (buf[n-1] == B_ETX)) 
      { //true end of package - Wait for end of package (and parse the package)
        parseBuffer(buf,n);			
        n = 0;
        memset(buf,0,MAX_COMMAND_LENGTH); 
        //printf("\n");
      }
      if (n >= MAX_COMMAND_LENGTH) { //Package overflow! (reset rx)
        n = 0;
        memset(buf,0,MAX_COMMAND_LENGTH);
      }
    }
  }//End of rxLoop
  //Shut down rFLEX, if recieve fails
  rflexRunning = -2;
  fprintf(stderr,"rFLEX: RX thread terminated\n");
  if (fil != NULL)
    fclose(fil);
  return something;
}

/** \brief Shut down rFLEX driver
 * 
 * \returns int success
 *  Checksum value
 */
extern int terminate(void) {

	//rFLEX commands to shut down robot!
	if (rflexRunning > 0) {
		rflexRunning = -1;
		setVelocity(0,0);
		brakeOn();
		sonarOff();
		odometryOff();
		digitalIoOff();
		if (sDev) close(sDev);
		//Wait for thread to close
		while (rflexRunning > -1);	usleep(100000); //Wait 100 ms
	}

	return 1;
}

/********** RFLEX Functions, ported from CARMEN and edited shamelessly ***********/

/** \brief Compute checksum (not really CRC) from/to rFLEX messages
 * 
 *  * \param[in] unsigned char *buf
 * Data buffer to calculate checksum from
 * 
 * \param[in] int nChars
 * Number of bytes in the checksum calculation
 * 
 * \returns int crc
 *  Checksum value
 */
int computeCRC( unsigned char *buf, int nChars )
{ 
  int i, crc;
  if (nChars==0) {
    crc = 0;
  } else {
    crc = buf[0];
    for (i=1; i<nChars; i++) {
      crc ^= buf[i];
    }
  }
  return(crc);
}


/* CONVERSION BYTES -> NUM */
/** \brief Convert data-buffer into unsigned 16-bit integer
 * 
 * \param[in] unsigned char *bytes
 * Databuffer place that holds the 16-bit variable
 * 
 * \returns int i
 * Data in host byte order
 */

uint16_t convertBytes2UInt16( unsigned char *bytes ) {
  unsigned int i;
  memcpy( &i, bytes, 2 );
  return(ntohs(i));
}

/** \brief Convert data-buffer into unsigned 32-bit integer
 * 
 * \param[in] unsigned char *bytes
 * Databuffer place that holds the 32-bit variable
 * 
 * \returns int i
 * Data in host byte order
 */
uint32_t convertBytes2UInt32( unsigned char *bytes ) {
  unsigned long i;
  memcpy( &i, bytes, 4 );
  return(ntohl(i));
}

/* CONVERSION NUM -> BYTES */

/** \brief Convert 8-bit integer to data-buffer
 * 
 * \param[in]  int i
 * Variable to be transferred into data-buffer
 *
 * \param[in] unsigned char *bytes
 * Databuffer place to hold the data
 */
void convertUInt8( unsigned int i, unsigned char *bytes ) {
  memcpy( bytes, &i, 1 );
}

/** \brief Convert 32-bit integer to data-buffer
 * 
 * \param[in]  int i
 * Variable to be transferred into data-buffer
 *
 * \param[in] unsigned char *bytes
 * Databuffer place to hold the data
 */
void convertUInt32(uint32_t l, unsigned char *bytes ) {
  uint32_t conv;
  conv = htonl( l );
  memcpy( bytes, &conv, 4 );
}

/** \brief Send prepared command to rFLEX controller
 * 
 * \param[in]  int port
 * What port in the rFLEX controller to process the command 
 * (what module)
 * 
 * \param[in]  int id
 * Sequential ID of command (not really used)
 * 
 * \param[in]  int opcode
 * Opcode of the command
 *
 * \param[in]  int len
 * Length of data buffer
 * 

 * \param[in] unsigned char *data
 * Pointer to data buffer
 *
 */
void cmdSend( int port, int id, int opcode, int len, unsigned char *data ) {
  int i;
  unsigned char cmd[MAX_COMMAND_LENGTH];

  /* START CODE */
  cmd[0] = 0x1b;
  cmd[1] = 0x02;
  /* PORT */
  cmd[2] = (unsigned char) port;
  /* ID */
  cmd[3] = (unsigned char) id;
  /* OPCODE */
  cmd[4] = (unsigned char) opcode;
  /* LENGTH */
  cmd[5] = (unsigned char) len;
  /* DATA */
  for (i=0; i<len; i++) {
    cmd[6+i] = data[i];
  }
  /* CRC */
  cmd[6+len] = computeCRC( &(cmd[2]), len+4 );    /* END CODE */
  cmd[6+len+1] = 0x1b;
  cmd[6+len+2] = 0x03;

  secureWrite( sDev, cmd, 9+len ); //Transmit data
}


/** \brief Parse data-buffer recieved from rFLEX controller
 * 
 * \param[in] char *buffer
 * Databuffer containing the data
 *
 * \param[in] unsigned int len
 * Length of data-buffer
 *
 */
int parseBuffer( unsigned char *buffer, unsigned int len )
{
  unsigned int port, dlen, crc;
  int isOK = 0;
  struct timeval t;
  int i;
  //
  port   = buffer[2];
  dlen   = buffer[5];
  //
  if (dlen+8>len) {
//     errcnt--;
    return(0);
  } else {
    crc    = computeCRC( &(buffer[2]), dlen+4 );
    isOK = crc == buffer[len-3];
    if (!isOK)
    {
      errcnt++;
      return(0);
    }
    switch(port) {
    case SYS_PORT:
      parseSystemReport(buffer);
      break;
    case MOT_PORT:
      gettimeofday(&t, NULL);
      if (fil != NULL)
      {
        fprintf(fil, "%ld.%06ld %d %d %d %02x %02x  ", t.tv_sec, t.tv_usec, isOK, len, dlen, crc, buffer[len-3]);
        for (i = 0; i < len; i++)
          fprintf(fil, " %02x", buffer[i]);
        fprintf(fil, "\n");
      }
      parseMotReport( buffer );
      break;
    case JSTK_PORT:
      break;
    case SONAR_PORT:
      parseSonarReport( buffer );
      break;
    case DIO_PORT:
			parseDioReport(buffer);
      break;
    case IR_PORT:
      fprintf( stderr, "(ir)" );
      break;
    default:
      break;
    }
  }
  return(1);
}

/** \brief Parse returned motion control report
 * 
 * \param[in]  char *buffer
 * Databuffer containing the motion control report
 *
 */
void parseMotReport( unsigned char *buffer )
{
//  int rv, timeStamp;
  unsigned char axis, opcode;
	uint32_t tmp;

	memcpy( &tmp, &(buffer[15]), 4 );

  opcode = buffer[4];
  switch(opcode) {
  case MOT_SYSTEM_REPORT:
//     rv        = convertBytes2UInt32(&(buffer[6]));
//     timeStamp = convertBytes2UInt32(&(buffer[10]));
    axis      = buffer[14];
		
    if (axis == 0) { //Translational axis
			/*printf("0x%02X 0x%02X 0x%02X 0x%02X convert: %u (test %u)\n",
				buffer[15],buffer[16],buffer[17],buffer[18],convertBytes2UInt32(&(buffer[15])),ntohl(tmp));*/
      setVariable(iTp,0,convertBytes2UInt32(&(buffer[15])));  //Distance
      setVariable(iTv,0,convertBytes2UInt32(&(buffer[19])));  //Velocity
      setVariable(iTa,0,convertBytes2UInt32(&(buffer[23])));  //Accelleration
      setVariable(iTt,0,convertBytes2UInt32(&(buffer[27])));  //Torque
//      flexPosRepCnt++;
    } else if (axis == 1) { //Rotational axis
      setVariable(iRp,0,convertBytes2UInt32(&(buffer[15])));  //Bearing
      setVariable(iRv,0,convertBytes2UInt32(&(buffer[19])));  //Angular velocity
      setVariable(iRa,0,convertBytes2UInt32(&(buffer[23])));  //Accelleration
      setVariable(iRt,0,convertBytes2UInt32(&(buffer[27])));  //Torque
//      flexRotRepCnt++;
    }
    break;
  default:
    break;
  }
}


/** \brief Parse returned sonar report
 * 
 * \param[in]  char *buffer
 * Databuffer containing the sonar report
 *
 */
void parseSonarReport( unsigned char *buffer ) {
  unsigned int sid;
 
  int count;
//  int retval, timeStamp;
  unsigned char opcode, dlen;
    
  opcode = buffer[4];
  dlen   = buffer[5];
  //status.num_sonars=MAX_NUM_SONARS;
  switch(opcode) {
  case SONAR_REPORT:
//     retval    = convertBytes2UInt32(&(buffer[6]));
//     timeStamp = convertBytes2UInt32(&(buffer[10]));
    count = 0;
    while ((8+count*3<dlen) && (count<256)) {
      sid   = buffer[14+count*3];
			sid		= sonarMap(sid); //get mapped index
			if (sid >= 0) {
				//printf("s[%d]= %d ",sid,convertBytes2UInt16( &(buffer[14+count*3+1])));
				setVariable(iSonar,sid,convertBytes2UInt16( &(buffer[14+count*3+1])));
			}
			count++;
    }
		//if (buffer[14] == 5) printf("\n");
    //	    fprintf( stderr, "(s:%d)", count );
    break;
  default:
    break;
  }
	
}

/** \brief Parse returned system report
 * 
 * \param[in]  char *buffer
 * Databuffer containing the sonar report
 *
 */
void parseSystemReport( unsigned char *buffer ) {
//   unsigned long timeStamp;
   unsigned char opcode, length;


   opcode = buffer[4];
   length = buffer[5];

  switch (opcode)
  {
    case SYS_STATUS:
      if (length < 9)
      {
        fprintf(stderr, "Got bad Sys packet (status)\n");
        break;
      }
//      timeStamp=convertBytes2UInt32(&(buffer[6]));
      // raw voltage measurement...needs calibration offset added
      setVariable(iBatt,0,convertBytes2UInt32(&(buffer[10])));
      setVariable(iBrake,0,buffer[14]);
      break;
    default:
      fprintf(stderr,"Unknown sys opcode recieved\n");
  }
}


/** \brief Parse returned digital I/O report
 * 
 * \param[in]  char *buffer
 * Databuffer containing the DIO report
 *
 */
void parseDioReport( unsigned char *buffer ) {
//  unsigned long timeStamp;
  unsigned char opcode, length, address;
  unsigned short data;

  opcode = buffer[4];
  length = buffer[5];

  switch(opcode) 
  {
    case DIO_REPORT:
      if (length < 6)
      {
      fprintf(stderr, "DIO Data Packet too small\n");
      break;
      }
//      timeStamp = convertBytes2UInt32(&(buffer[6]));
      address = buffer[10];
      data = convertBytes2UInt16(&(buffer[11]));
      //Parse bumperpackage
      if(address == BUMPER_ADDR){
        //0x0001 is front bumper
        if (data & 0x0001) {
          setVariable(iBump,0,1);
          //Pull emergency brake, if enabled
          if (bumpBrake) brakeOn();
        } else {
          setVariable(iBump,0,0);
        }
        //0x0002 is rear bumper
        if (data & 0x0002) {
          setVariable(iBump,1,1);
          //Pull emergency brake, if enabled
          if (bumpBrake) brakeOn(); 
        } else {
          setVariable(iBump,1,0);
        }
      }
    break;
    default:
    break;
  }
}

/** \brief Map sonars accoding to robot configuration
 * 
 * This function make it possible to map the sonar values to
 * a given robot configuration
 *
 * So far, it is only supported by ATRV-Jr
 * 
 * \param[in]  char *buffer
 * Databuffer containing the sonar report
 *
 */
int  sonarMap(int sonarId) {

	int dbIndex;

	//map according to ATRV-Jr
	switch (sonarId) {
		case 0: //map to 0
    case 1: //map to 1
		case 2: //map to 2
		case 3: //map to 3
		case 4: //map to 4
		case 5: //map to 5
			dbIndex = sonarId;
			break;
		case 16: //map to 6
		case 17: //map to 7
		case 18: //map to 8
		case 19: //map to 9
		case 20: //map to 10
			dbIndex = sonarId - 10;
			break;
		case 32: //map to 16
			dbIndex = 16;
			break;
    case 33: //map to 15
			dbIndex = 15;
			break;
		case 34: //map to 14
			dbIndex = 14;
			break;
		case 35: //map to 13
			dbIndex = 13;
			break;
		case 36: //map to 12
			dbIndex = 12;
			break;
		case 37: //map to 11
			dbIndex = 11;
			break;
		default: //Unmapped sensor - ignore
			dbIndex = -1;
			break;
	}

return dbIndex;
}

/** \brief Activate digital IO report
 * 
 * \param[in]  int period
 * Period of the returned report (in ms?)
 * Actually, I think the period is the holdoff between DIO interrupts
 * before a new DIO report is emitted
 * 
 *
 */
void digitalIoOn(int period) {
  unsigned char data[MAX_COMMAND_LENGTH];
  convertUInt32( (long) period, &(data[0]) );
  cmdSend(DIO_PORT, 0, DIO_REPORTS_REQ, 4, data);
}

/** \brief Deactivate digital IO report
 *
 */
void digitalIoOff(void)
{
  unsigned char data[MAX_COMMAND_LENGTH];
  convertUInt32( (long) 0, &(data[0]) );
  cmdSend(SONAR_PORT, 4, DIO_REPORTS_REQ, 4, data );
}

/** \brief Activate brake on robot
 */
void brakeOn(void) {
  cmdSend(MOT_PORT, 0, MOT_BRAKE_SET, 0, NULL);
}

/** \brief Release brake on robot
 */
void brakeOff (void) {
  cmdSend(MOT_PORT, 0, MOT_BRAKE_RELEASE, 0, NULL);
}

/** \brief Reset motion control values
 */
void motionSetDefaults(void) {
  cmdSend(MOT_PORT, 0, MOT_SET_DEFAULTS, 0, NULL);
}

/** \brief Activate odometry reports
 * 
 * \param[in]  int period
 * Period of the returned report (in ms?)
 *
 */
void odometryOn (long period) { 
  unsigned char data[MAX_COMMAND_LENGTH];
  //int num_read;
  
  convertUInt32( period, &(data[0]) );         /* period in ms */
  convertUInt32( (long) 3, &(data[4]) );       /* mask */
  cmdSend(MOT_PORT, 0, MOT_SYSTEM_REPORT_REQ, 8, data);

  //num_read = read(dev_fd, data, MAX_COMMAND_LENGTH);   
}

/** \brief Deactivate odometry
 */
void odometryOff(void) { 
  unsigned char data[MAX_COMMAND_LENGTH];
  convertUInt32( (long) 0, &(data[0]) );       /* period in ms */
  convertUInt32( (long) 0, &(data[4]) );       /* mask */
  cmdSend(MOT_PORT, 0, MOT_SYSTEM_REPORT_REQ, 8, data);
}

/** \brief Activate sonar
 */
int sonarOn(void) {
  unsigned char data[MAX_COMMAND_LENGTH];
  convertUInt32( (long) 30000, &(data[0]) );
  convertUInt32( (long) 0, &(data[4]) );
  convertUInt32( (long) 0, &(data[8]) );
  convertUInt8(  (int) 2, &(data[12]) );
  cmdSend(SONAR_PORT, 4, SONAR_RUN, 13, data);


  return 0;
}

/** \brief Deactivate sonar
 */
int sonarOff(void){
  unsigned char data[MAX_COMMAND_LENGTH];
  convertUInt32( (long) 0, &(data[0]) );
  convertUInt32( (long) 0, &(data[4]) );
  convertUInt32( (long) 0, &(data[8]) );
  convertUInt8(  (int) 0, &(data[12]) );
  cmdSend(SONAR_PORT, 4, SONAR_RUN, 13, data);

  return 0;
}

/** \brief Get system report
 */
void getSystemReport(void) {

	cmdSend(SYS_PORT, 0, SYS_STATUS, 0, NULL );

}

/** \brief Set robot velocity
 * 
 * \param[in]  int tVel
 * Velocity on translatorital axis
 *
 * \param[in]  int rVel
 * Velocity on rotational axis
 *
 */
int setVelocity(int32_t tVel, int32_t rVel) {
  unsigned char data[MAX_COMMAND_LENGTH];

  long utvel;long urvel;

	// *** Borrowed from the player project - Thanks! 
  // ** workaround for stupid hardware bug, cause unknown, but this works
  // ** with minimal detriment to control
  // ** avoids all values with 1b in highest or 3'rd highest order byte
  
  // 0x1b is part of the packet terminating string
  // which is most likely what causes the bug

  // ** if 2'nd order byte is 1b, round to nearest 1c, or 1a
  utvel=labs(tVel);
  urvel=labs(rVel);
  if((urvel&0xff00)==0x1b00){
    // ** if lowest order byte is>127 round up, otherwise round down 
    urvel=(urvel&0xffff0000)|((urvel&0xff)>127?0x1c00:0x1aff);
  }

  // ** if highest order byte is 1b, round to 1c, otherwise round to 1a
  if((urvel&0xff000000)==0x1b000000){
    // ** if 3'rd order byte is>127 round to 1c, otherwise round to 1a
    urvel=(urvel&0x00ffffff)|((urvel&0xff0000)>127?0x1c000000:0x1aff0000);
  }
  
	//Setup translatorical axis
  convertUInt8( (long) 0, &(data[0]));               /* forward motion */
  convertUInt32( (long) labs(utvel), &(data[1]));        /* abs trans velocity */
	//Select default or cmd acceleration
	if (getWriteVariable(iCmdTa,0)) convertUInt32( labs(getWriteVariable(iCmdTa,0)), &(data[5]));
  else convertUInt32( (long) defTransAccl, &(data[5]));
	//Select default or cmd torque
	if (getWriteVariable(iCmdTt,0)) convertUInt32(labs(getWriteVariable(iCmdTt,0)), &(data[9]));
  else convertUInt32( (long) defTransTorque, &(data[9]));

	//Set translatory direction
	if (tVel < 0) convertUInt8((long) 0, &(data[13]));  /* negative trans direction */	
  else 					convertUInt8((long) 1, &(data[13]));  /* positive trans direction */
  cmdSend(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data); //Send command

	//Setup rotational axis
  convertUInt8((long) 1, &(data[0]));                  /* rotational motion */
  convertUInt32((long) labs(urvel), &(data[1]));         /* abs rot velocity  */
  /* 0.275 rad/sec * 10000 */ 
	//Select default or cmd acceleration
	if (getWriteVariable(iCmdRa,0)) convertUInt32( labs(getWriteVariable(iCmdRa,0)), &(data[5]));
  else convertUInt32( (long) defRotAccl, &(data[5]));
	//Select default or cmd torque
	if (getWriteVariable(iCmdRt,0)) convertUInt32(labs(getWriteVariable(iCmdRt,0)), &(data[9]));
  else convertUInt32( (long) defRotTorque, &(data[9]));

	//Set rotational direction
	if (rVel < 0) convertUInt8((long) 0, &(data[13]));  /* negative rot direction */	
  else 					convertUInt8((long) 1, &(data[13]));  /* positive rot direction */
  cmdSend(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data);

  return 0;
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
  printf("rFLEX: Initializing iRobot rFLEX HAL %s.%s\n",RFLEXVERSION,tempString);


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
		printf("   Error: No <rflex> XML tag found in plugins section\n");
		return -1;
	}

  //Initialize rFLEX, if XML parsed properly
  if (xmlParse.enable) done = initRflex();



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
 				((info->depth == 3) && (strcmp("rflex",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("rflex",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   Use of rFLEX disabled in configuration\n"); 
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
  } else if (strcmp("odometry",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("period",attr[i]) == 0) odoPeriod = atoi(attr[i+1]);  
    printf("   Odometry period: %d.%03d ms\n",odoPeriod/1000,odoPeriod%1000);
	} else if (strcmp("translation",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("torque",attr[i]) == 0) defTransTorque = atoi(attr[i+1]);
		for(i = 0; attr[i]; i+=2) if (strcmp("accel",attr[i]) == 0) defTransAccl = atoi(attr[i+1]);  
    printf("   Translation default: torque: %u accel: %u\n",defTransAccl,defTransTorque);
  } else if (strcmp("rotation",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("torque",attr[i]) == 0) defRotTorque = atoi(attr[i+1]);
		for(i = 0; attr[i]; i+=2) if (strcmp("accel",attr[i]) == 0) defRotAccl = atoi(attr[i+1]);  
    printf("   Rotation default: torque: %u accel: %u\n",defRotTorque,defRotAccl);
  } else if (strcmp("bumperbrake",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if ((strcmp("default",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
			bumpBrake = 1;
		} else {
			bumpBrake = 0;
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
