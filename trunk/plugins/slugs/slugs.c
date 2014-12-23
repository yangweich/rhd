/** \file slugs.c
 *  \ingroup slugs
 *  \brief Serial SLUGS Module
 *
 * SLUGS Module for RHD. 
 * 
 * Based on AuGps by Lars Mogensen and Christian Andersen and 
 * HAKOD by Asbjørn Mejnertsen and Anders Reeske Nielsen
 * GPS module by Anders Billesø Beck
 * .
 * 
 *  \author Jonathan Løwert
 *  $Rev: 59 $
 *  $Date: 2009-12-21 15:46:28 +0200 (Tue, 21 Apr 2009) $
 */
 /***************************************************************************
 *                  Copyright 2009 Jonathan Løwert                         *
 *                       jonathan@lowert.dk                                *
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
#define SLUGSVERSION 	      "1.0"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2009-12-21 15:46:28 +0200 (Tue, 21 Dec 2009) $:"
 #define ID               "$Id: slugs.c 59 2012-10-21 06:25:02Z jcan $"
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

#include "slugs.h"
//#include "circBuffer.h"

///Maximum size of SLUGS String
#define SLUGSSTRINGSIZE 250
///Conversion factor from knot to m/s
#define KNOT2MS 0.51444444

//struct CircBuffer protParseBuffer;
//CBRef ppBuffer;

FILE *slugsLog;

//Definitions
int  slugsDev;      ///SLUGS Port file pointer
char slugsDevString[64]; ///String to hold SLUGS device
int  baudrate = 0;
static volatile char slugsRunning = 0;

//Database indexes
int  igyroX, igyroY,igyroZ, iaX, iaY, iaZ, iRoll, iPitch, iYaw;
int  iP, iQ, iR, iTime;
int  iBaX, iBaY, iBaZ, iBgX, iBgY, iBgZ;

// Threads are being defined
pthread_t slugs_thread;
pthread_attr_t attr;

//Function prototypes
void protParserInit(void);
void protParseDecode (char unsigned* fromSPI);
int initSlugs(void);
int initXML(char *filename);
int shutdownSlug(void);
void initVars(void);
void updateStates(unsigned char * completeSentence);
void *slugs_task(void *not_used);
unsigned char getChecksum(unsigned char* sentence, unsigned char size);
float bytesToFloat (unsigned char* inBytes);
void floatToBytes (float inFloat, unsigned char* inBytes);
unsigned short bytesToUShort (unsigned char* inBytes);
void uShortToBytes (unsigned short inUShort, unsigned char* inBytes);
short bytesToShort (unsigned char* inBytes);
void shortToBytes (short inShort, unsigned char* inBytes);
int bytesToInt (unsigned char* inBytes);

struct CircBuffer protParseBuffer;
CBRef ppBuffer;

void protParserInit(void){
	// initialize the circular buffer
	ppBuffer = (struct CircBuffer* )&protParseBuffer;
	newCircBuffer(ppBuffer);
	
	
	// Control MCU boot procedures
//	#ifdef _IN_PC_
		//
//	#else
//		aknControlData.reboot = 1;		
//	#endif
		
	// Manual mode
//	apsControlData.controlType = CTRL_TYPE_MANUAL;
	
	
	
}




/** \brief Initialize SLUGS
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
int initSlugs(void) {

  // Open RS232 port
  slugsDev = open(slugsDevString, O_RDWR);
  if (slugsDev<0) 
  {
    fprintf(stderr,"   SLUGS: Error opening %s\n",slugsDevString);
    return -1;
  }
//Set baudrate for SLUGS receiver
  if (set_serial(slugsDev,baudrate) == -1)  {
    fprintf(stderr,"   SLUGS: Can't set SLUGS serial port parameters\n");
      return -1;
  }
  slugsRunning = 1;

  protParserInit();
  initVars();
  //slugsLog = fopen("../test/slugslog.txt","r"); //Test RTK SLUGS

 //Create variables 
 //iTime         = createVariable('r', 4, "SLUGStime");
 
  // Initialization and starting of threads
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  usleep(50000);
  if (pthread_create(&slugs_thread, &attr, slugs_task, 0))
  {
    perror("   SLUGS: Can't start SLUGS thread");
    return -1;
  }
 
	
  return 1;

}

void initVars(void){

  //If initialization is done, create variables
  igyroX = createVariable('r',1,"slugsGyroX");
  igyroY = createVariable('r',1,"slugsGyroY");
  igyroZ = createVariable('r',1,"slugsGyroZ");
  iaX    = createVariable('r',1,"slugsAccX");
  iaY    = createVariable('r',1,"slugsAccY");
  iaZ    = createVariable('r',1,"slugsAccZ");
  iRoll  = createVariable('r',1,"slugsRoll");
  iPitch = createVariable('r',1,"slugsPitch");
  iYaw   = createVariable('r',1,"slugsYaw");
  iP     = createVariable('r',1,"slugsP");
  iQ     = createVariable('r',1,"slugsQ");
  iR     = createVariable('r',1,"slugsR");
  iTime  = createVariable('r',1,"slugsTime");
  iBaX   = createVariable('r',1,"slugsBiasAccX");
  iBaY   = createVariable('r',1,"slugsBiasAccY");
  iBaZ   = createVariable('r',1,"slugsBiasAccZ");
  iBgX   = createVariable('r',1,"slugsBiasGyroX");
  iBgY   = createVariable('r',1,"slugsBiasGyroY");
  iBgZ   = createVariable('r',1,"slugsBiasGyroZ");
}

/** \brief Initialize Shut down GPS rx thread
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
int shutdownSlug(void) {
  printf("shutdownSlugs has been runned\n");

  slugsRunning = 0;
  pthread_join(slugs_thread,NULL);

  return 1;
}

/** \brief SLUGS RX thread.
 */
void *slugs_task(void *not_used) {

  printf("   SLUGS: Rx thread started\n");
  unsigned char buf[SLUGSSTRINGSIZE];
  unsigned char tmp;
  int n;

  //Recieve from SLUGS
  while (slugsRunning < 0) usleep(100000);
  while(slugsRunning) {
//printf("Slugs running\n");
    memset(buf,0,SLUGSSTRINGSIZE);
	buf[0]=SLUGSSTRINGSIZE;
    tmp = 0;
//	printf("out for loop, %d, %d\n",tmp,slugsRunning);

    //for(n = 0; tmp != '\n'; n++) {
      for(n = 0; n+1<SLUGSSTRINGSIZE; n++) {
	//printf("in for loop, %d, %d\n",n,slugsRunning);
      if(secureRead(slugsDev,&tmp,1) > 0) { //Debug: fread(&tmp,1,1,gpsLog)
		buf[n+1]=tmp;
		if(n-1>=SLUGSSTRINGSIZE){
			//printf("Safety - too long %d %d \n",n,slugsRunning);
			protParseDecode(buf);
			//printf("Back from Decode %d \n", slugsRunning);
			break;
		}
		//printf("Temp: %d\n",tmp);	
	}
	  else {
	fprintf(stderr,"SLUGS: Error reading from Serial port, shutting down\n");        
	slugsRunning = 0; //Shutdown if read-error
        
      }
      if (!slugsRunning) break; //Abort if shutdown signal is recieved
}
	//Sending to decoding
	protParseDecode(buf);
  } //Ending while loop


  close(slugsDev);
  fprintf(stderr,"   SLUGS: ...Shutdown SLGUS task %d\n",slugsRunning);
  pthread_exit(0);
}

void protParseDecode (char unsigned* fromSPI){

	// Static variables CAREFUL
	static unsigned char prevBuffer[2*MAXLOGLEN];
	static unsigned char previousComplete =1;
	static unsigned char indexLast = 0;
    #ifdef _IN_PC_
         static long long checkSumFail = 0;
         static long long totalPackets = 0;
         static float test = 0.0;
         float alpha = 0.3;
    #endif


	// local variables
	unsigned char i;
	unsigned char tmpChksum = 0, headerFound=0, noMoreBytes = 1;
	unsigned char trailerFound = 0;

	//unsigned char logSize = 0;

	// Add the received bytes to the protocol parsing circular buffer
    for(i = 1; i <= fromSPI[0]; i += 1 )
    //for(i = 0; i <= 95; i += 1 )
	{
		writeBack(ppBuffer, fromSPI[i]);
	}

	// update the noMoreBytes flag accordingly
   noMoreBytes = (fromSPI[0]>0)?0:1;
   // noMoreBytes = 0;

	while (!noMoreBytes){
		// if the previous message was complete then read from the circular buffer
		// and make sure that you are into the begining of a message
		if(previousComplete){
			while (!headerFound && !noMoreBytes) {
				// move along until you find a dollar sign or run out of bytes
				while (getLength(ppBuffer)>1 && peak(ppBuffer)!=DOLLAR){
					readFront(ppBuffer);
				}
				// if you found a dollar then make sure the next one is an AT
				if(getLength(ppBuffer)>1 && peak(ppBuffer) == DOLLAR){
					// read it
					prevBuffer[indexLast++] = readFront(ppBuffer);
                    // if next is a at sign
					if (peak(ppBuffer) == AT){
						// read it
						prevBuffer[indexLast++] = readFront(ppBuffer);
						// and signal you found a header
						headerFound = 1;
                         // and set as  incomplete the sentece
                         previousComplete = 0;
					}
				} else {
					noMoreBytes = 1;
				} // else no dollar
			} // while we found header && no more bytes
		}// if previous complete

		// At this point either you found a header from a previous complete
		// or you are reading from a message that was incomplete the last time
		// in any of those two cases, keep reading until you run out of bytes
		// or find a STAR and an AT
		while (!trailerFound && !noMoreBytes){
			while (getLength(ppBuffer)>2 && peak(ppBuffer)!=STAR){
				prevBuffer[indexLast++] = readFront(ppBuffer);
			}
			// if you found a STAR (42) and stil have bytes
			if (getLength(ppBuffer)>2 && peak(ppBuffer)==STAR){
				// read it
				prevBuffer[indexLast++] = readFront(ppBuffer);
				// if you still have 2 bytes
				if (getLength(ppBuffer)>1){
					// and the next byte is an AT sign
					if (peak(ppBuffer)==AT){
						// then you found a trailer
						trailerFound =1;
					}
				} else {
					noMoreBytes =1;
				}
			} else {
				// no more bytes
				noMoreBytes =1;
			}
		}

		// if you found a trailer, then the message is done
		if(trailerFound){
			// read the AT and the checksum
			prevBuffer[indexLast++] = readFront(ppBuffer);
			prevBuffer[indexLast] = readFront(ppBuffer);

			// Compute the checksum
			tmpChksum= getChecksum(prevBuffer, indexLast-1);
            #ifdef _IN_PC_
               totalPackets++;
            #endif

			// if the checksum is valid
			if (tmpChksum ==prevBuffer[indexLast]){
				// update the states depending on the message
				updateStates(&prevBuffer[0]);
				// increment the log size
				//logSize += (indexLast+1);
                #ifdef _IN_PC_
					// if in PC and loggin is enabled
                    if ((outFile != NULL)){
                       printState(outFile, prevException);
                    }
                    //test = alpha*test;
                #endif
			}
            else{
                 #ifdef _IN_PC_
                    checkSumFail++;
                    //test = (1.0-alpha) + alpha*test;
                 #endif
            }
            // get everything ready to start all-over
			previousComplete =1;
			indexLast = 0;
            headerFound = 0;
            trailerFound = 0;
            memset(prevBuffer, 0, sizeof(prevBuffer));

		}else { // you ran out of bytes
			// No More Bytes
			noMoreBytes = 1;
		}// else no star
	} // big outer while (no more bytes)
    #ifdef _IN_PC_
       if (totalPackets>0){
          //test =  ((float)checkSumFail/(float)totalPackets);
          test = (float)checkSumFail;

       } else {
          test = 0.0;
       }
       return test;
    #endif
}

void updateStates(unsigned char * completeSentence){
	
	switch (completeSentence[2]){
		// Sensor MCU sentences
		// ====================
		case GPSMSG_ID:		// GPS Sentence

		break;
		case LOADMSG_ID:
		break;
		case RAWMSG_ID: // Sensor Raw data
			setVariable(igyroX , 0, (int16_t)bytesToShort(&completeSentence[4]));
			setVariable(igyroY , 0, (int16_t)bytesToShort(&completeSentence[6]));
			setVariable(igyroZ , 0, (int16_t)bytesToShort(&completeSentence[8]));

			setVariable(iaX , 0, (int16_t)bytesToShort(&completeSentence[10]));
			setVariable(iaY , 0, (int16_t)bytesToShort(&completeSentence[12]));
			setVariable(iaZ , 0, (int16_t)bytesToShort(&completeSentence[14]));
			
		break;
		case ATTMSG_ID:
			setVariable(iRoll , 0, (int32_t)bytesToInt(&completeSentence[4]));//is float
			setVariable(iPitch , 0, (int32_t)bytesToInt(&completeSentence[8]));//is float
			setVariable(iYaw , 0, (int32_t)bytesToInt(&completeSentence[12]));//is float
			
			setVariable(iP , 0, (int32_t)bytesToInt(&completeSentence[16]));//is float
			setVariable(iQ , 0, (int32_t)bytesToInt(&completeSentence[20]));//is float
			setVariable(iR , 0, (int32_t)bytesToInt(&completeSentence[24]));//is float
			
			setVariable(iTime , 0, (int16_t)bytesToShort(&completeSentence[28]));	
		break;
		case DYNMSG_ID:
		break;
		case BIAMSG_ID:
			setVariable(iBaX , 0, (int32_t)bytesToInt(&completeSentence[4]));//is float
			setVariable(iBaY , 0, (int32_t)bytesToInt(&completeSentence[8]));//is float
			setVariable(iBaZ , 0, (int32_t)bytesToInt(&completeSentence[12]));//is float

			setVariable(iBgX , 0, (int32_t)bytesToInt(&completeSentence[16]));//is float
			setVariable(iBgY , 0, (int32_t)bytesToInt(&completeSentence[20]));//is float
			setVariable(iBgZ , 0, (int32_t)bytesToInt(&completeSentence[24]));//is float
		break;		
		case DIAMSG_ID:
			
		break;
		case XYZMSG_ID:
		break;	

		case PILMSG_ID: // Pilot Console Commands data
		break;
		
		case AKNMSG_ID: // Aknowledge Messages
		break;
		
		case PWMMSG_ID: // PWM Control Surface Commands data
		break;
		
		case APSMSG_ID: // AP Status Report
	   	break;
		
		case CALMSG_ID: // report from AP to GS regarding Calib Values
		break;
				
		case NAVMSG_ID:
		break;
		
		case SENMSG_ID:
		break;	
		case LOGMSG_ID:
		break;	
						
		default:
		break;   
	}
}

float bytesToFloat (unsigned char* inBytes){
	tFloatToChar convert;
	convert.chData[0] = inBytes[0];
	convert.chData[1] = inBytes[1];
	convert.chData[2] = inBytes[2];
	convert.chData[3] = inBytes[3];
	return convert.flData;
}

void floatToBytes (float inFloat, unsigned char* inBytes){
	tFloatToChar convert;
	convert.flData = inFloat;
	
	inBytes[0] = convert.chData[0];
	inBytes[1] = convert.chData[1];
	inBytes[2] = convert.chData[2];
	inBytes[3] = convert.chData[3];	
}

unsigned short bytesToUShort (unsigned char* inBytes){
	tUnsignedShortToChar convert;
	convert.chData[0] = inBytes[0];
	convert.chData[1] = inBytes[1];
	return convert.usData;
}

short bytesToShort (unsigned char* inBytes){
	tShortToChar convert;
	convert.chData[0] = inBytes[0];
	convert.chData[1] = inBytes[1];
	return convert.shData;
}

int bytesToInt (unsigned char* inBytes){
	tIntToChar convert;
	convert.chData[0] = inBytes[0];
	convert.chData[1] = inBytes[1];
	convert.chData[2] = inBytes[2];
	convert.chData[3] = inBytes[3];
	return convert.inData;
}


// GPS checksum code based on 
// http://www.codeproject.com/KB/mobile/WritingGPSApplications2.aspx
// original code in C# written by Jon Person, author of "GPS.NET" (www.gpsdotnet.com)
unsigned char getChecksum(unsigned char* sentence, unsigned char size){

    // Loop through all chars to get a checksum
    unsigned char checkSum = 0;
	unsigned char i;
	for (i = 0; i< size; i++)
    {
      if (sentence[i] == DOLLAR)
      {
        // Ignore the dollar sign
      }
      else if (sentence[i] == STAR)
      {
        // Stop processing before the asterisk
        break;
      }
      else
      {
        // Is this the first value for the checksum?
        if (i == 0)
        {
          // Yes. Set the checksum to the value
          checkSum = sentence[i];
        }
        else
        {
          // No. XOR the checksum with this character's value
	       checkSum ^= sentence[i];
        }
      }
    }
    // Return the checksum 
    return checkSum;
}



//----------------CircBuffer------------------

// Constructors - Destructors
// ==========================
// this Function returns a pointer to a new Circular Buffer of size pm_size 

#if __IN_DSPIC__
	void newCircBuffer (CBRef cB){
		
		// initialize to zero
		int i;
		for (i=0; i<BSIZE; i++){
			cB->buffer[i] = 0;
		}
				
		// initialize the data members
		cB->head = 0;
		cB->tail = 0;
		cB->size = BSIZE;
		cB->overflowCount = 0;

	}

// this function frees the Circular Buffer CB Ref
	void freeCircBuffer (CBRef* cB){
		// if it is already null, nothing to free
		if (cB == NULL || *cB == NULL) {return;}
				
		// free and nil the pointer
		//free(*cB);
		*cB = NULL;
	}


#else
	CBRef newCircBuffer (int pm_size){
		// create the circular buffer pointer
		CBRef cB;
		
		// allocate memory for it
		cB = (CBRef) malloc(sizeof(CircBuffer));
		
		// allocate memory for the buffer
		cB->buffer = (unsigned char *)calloc(pm_size, sizeof(unsigned char));
		
		// initialize the data members
		cB->head = 0;
		cB->tail = 0;
		cB->size = pm_size;
		cB->overflowCount = 0;
		
		// return the buffer pointer
		return (cB);
		
	}

  

	// this function frees the Circular Buffer CB Ref
	void freeCircBuffer (CBRef* cB){
		// if it is already null, nothing to free
		if (cB == NULL || *cB == NULL) {return;}
		
		// free the buffer
		free((*cB)->buffer);
		
		// free and nil the pointer
		free(*cB);
		*cB = NULL;
	}

#endif

// Accesor Methods
// ===============

// returns the amount of unread bytes in the circular buffer
unsigned int getLength (CBRef cB){	
	// if the circular buffer is not null
	if (cB != NULL){
		if (cB->head <= cB->tail){
			return (cB->tail-cB->head);
		} else{
			return (cB->size + cB->tail - cB->head);
		}		
	}
	else{
		return 0;
	}
	

}

// returns the actual index of the head
int readHead (CBRef cB){
	// if the circular buffer is not null
	if (cB != NULL){
		return (cB->head);
	}
	else{
		return 0;
	}

}

// returns the actual index of the tail
int readTail (CBRef cB){
	// if the circular buffer is not null
	if (cB != NULL){
		return (cB->tail);
	}
	else{
		return 0;
	}

}

// returns the byte (actual value) that the head points to. this
// does not mark the byte as read, so succesive calls to peak will
// always return the same value
unsigned char peak(CBRef cB){
	// if the circular buffer is not null
	if (cB != NULL)
	{	
		// if there are bytes in the buffer
		if (getLength(cB) > 0){
			return cB->buffer[cB->head];
		}
	}
	return 0;	
}


// Manipulation Procedures
// ======================
// returns the front of the circular buffer and marks the byte as read
unsigned char readFront (CBRef cB){
	// if the circular buffer is not null
	if (cB != NULL)
	{	
		char retVal;
		// if there are bytes in the buffer
		if (getLength(cB) > 0){
			retVal = cB->buffer[cB->head];
			cB->head = cB->head < (cB->size -1)? cB->head+1: 0;
			return retVal;
		}
		return 128;
	}
	return 254;
}

// writes one byte at the end of the circular buffer, 
// increments overflow count if overflow occurs
unsigned char writeBack (CBRef cB, unsigned char data){
	// if the circular buffer is not null
	if (cB != NULL){			
		if (getLength (cB) == (cB->size -1)){
			cB->overflowCount ++;
			return 1;
		} else {		
			cB->buffer[cB->tail] = data;
			cB->tail = cB->tail < (cB->size -1)? cB->tail+1: 0;
			return 0;
		}
		return 0;
	}
	else{
		return -1;
	}
}

// empties the circular buffer. It does not change the size. use with caution!!
void makeEmpty(CBRef cB){
	if (cB != NULL){
		int i;
		for(i = 0; i < cB->size; ++i)
		{
			cB->buffer[i]= 0;
		}
		cB->head = 0;
		cB->tail = 0;
		cB->overflowCount = 0;
	}
}

// returns the amount of times the CB has overflown;
unsigned char getOverflow(CBRef cB){
	if (cB != NULL){
		return cB->overflowCount;
	}else{
		return -1;
	}
}

#if DEBUG
// Other Functions
// ===============
// prints the circular buffer, used for debug
void printCircBuf(CBRef cB){
	// if the circular buffer is not null
	if (cB != NULL){
		printf("[");
		int i;
		for(i = 0; i < cB->size; ++i)
		{
			printf("%d ", cB->buffer[i]);
		}
		printf("]\n");
		printf("Size of: %d\n", cB->size );
		printf("Head at: %d\n", cB->head );
		printf("Tail at: %d\n\n", cB->tail );

	}
	else{
		printf("Calling Print on an empty Circular Buffer");
	}
}
#endif

/************************** XML Initialization **************************/
///Struct for shared parse data
typedef struct  {
    int depth;
    char skip;
    char enable;
    char found;
  }parseInfo;

//Parsing functions
void XMLCALL slugsStartTag(void *, const char *, const char **);
void XMLCALL slugsEndTag(void *, const char *);


/** \brief Initialize the Slugs HAL
 *
 * Reads the XML file and sets up the Slugs settings
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
  printf("Slugs: Initializing Slugs IMU HAL %s.%s\n",SLUGSVERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "Slugs: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, slugsStartTag, slugsEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("Slugs: Error reading: %s\n",filename);
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
    fprintf(stderr, "Slugs: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);
	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <slugs> XML tag found in plugins section\n");
		return -1;
	}

  //Start slugs thread after init
  if (xmlParse.enable) done = initSlugs();


 return done;
}

///Handle XML Start tags
void XMLCALL
slugsStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("slugs",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("slugs",el)) {
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   Slugs: Use of Slugs disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(slugsDevString,attr[i+1],63); 
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate = atoi(attr[i+1]); 
    printf("   GPS: Serial port %s at %d baud\n",slugsDevString,baudrate);
  } 

}

///Handle XML End tags
void XMLCALL
slugsEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
