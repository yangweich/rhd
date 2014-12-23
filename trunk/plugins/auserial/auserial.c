/** \file auserial.c
 *  \ingroup hwmodule
 *  \brief Hardware abstraction layer for AU Serial bus
 *
 *  This liberary inplements the hardware abstraction layer
 *  for communicating using the serial bus protocol used on the 
 *  SMR platform in the Institute of Automation (AU) on the Technical
 *  University of Denmark
 * 
 *  Initialization is done by XML that creates the desired serial busses
 *  devices and defines the commands and variables associated by the devices
 *  
 *  Interface is done through the variable database, that ensures thread 
 *  safety and open access.
 *
 *  Recieving from the serial bus is done in seperate threads, and 
 *  automatically transferred to the variable pool.
 *
 *  \author Anders Billesø Beck
 *  $Rev: 59 $
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
/***************************** Plugin version  *****************************/
#define AUSERIALVERSION 	"1.1"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $:"
 #define ID               "$Id: auserial.c 59 2012-10-21 06:25:02Z jcan $"
/***************************************************************************/

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

//RHD Headers
#include <database.h>
#include <scheduler.h>
#include <globalfunc.h>

#include "auserial.h"

///Debugging flag - Enables printout
#define DEBUG 0

//Function prototypes
int initAuserial(void);
void *serialrx_task(void *);
int sendCommand(int bus, int cmd);
char txVariable2txBuf(int var, char *txBuf);
char rxBuf2rxVariable(char *txBuf,int var);
int parseBustag(void *data, const char **attr);
int parseDevicetag(void *data, const char **attr);
int parseCmdtag(void *data, const char **attr);
int parseVariablestart(void *data, const char **attr);
int parseVariabletag(void *data, const char **attr);
int parseVariableend(void *data);

//Global variables
pthread_attr_t attr;

/** Structures **/
///Length of structure arrays 
struct confLen {
  int bus;        /**< Number of busses in the system */
  int device;     /**< Number of devices in the system */
  int command;    /**< Number of commands in the system */
  int variable;   /**< Number of variables in the system */
} len;

///Information of the serial busses
struct busInfo {
  int  fd;                 /**< filepointer to the serial bus */
  char devString[64];      /**< device string  */
  char name[64];           /**< identification name */
  int  baudrate;           /**< bitrate of the serial bus */
  int  maxTxByte;          /**< Maximum number of bytes to TX */
  int  holdoff;            /**< Bytes to spare of the full bus capacity */
  int  enabled;   				/**< is the port opened and running */
  int resetRx;    				/**< flag to get rx to sync */
	int lastPad;						/**< How many bytes was padded to the bus last period */
  pthread_t threadId;      /**< thread identifier for the RX thread */
} *busses;

///Information of the defined commands
struct deviceInfo {
  int busId;       /**< id of the associated bus */
  int device;      /**< device Id on the bus */
  char name[64];
} *devices;

///Information of the defined commands
struct cmdInfo {
  int busId;       /**< id of the associated bus */
  int deviceId;    /**< index in database array */
  int cmd;         /**< value of the command on the device*/
  char name[64];   /**< Name of the command */
  char type;       /**< Type of command, poll, request or send */
  int period;      /**< Transmission period for poll variables (of sheduler periods) */
  int offset;      /**< Periodic offset, to avoid bus overflow at certain periods */
  int pad;         /**< Zeroes to pad while device is responding */
} *commands;


///Information of the defined variables
struct varInfo {
    int dbIndex;          /**< index in database array */
    int dbId;             /**< id in database*/
    int busId;            /**< id of the associated bus */
    int deviceId;         /**< id of the associated device*/
    int cmdId;            /**< id of the associated command*/
    int device;           /**< bus id of the associated device*/
    int cmd;              /**< command assicoated with the device */
    char dir;             /**< direction of the variable */
    char sign;            /**< is the variable signed */
    char invert;          /**< flag to indicate if the variable should be inverted */
    struct par {          /**< structure to parse AU-serial msg's into variable*/
      int8_t byte[4];     /**< bytewise parsing byte to byte tansfer*/
      int8_t bwBit[32];   /**< bitbise parsing: incomming bit */
      int8_t bwByte[32];  /**< bitbise parsing: in incomming byte */
      int8_t bwIndex[32]; /**< bitbise parsing: goes to bit*/
    } parse;
  } *varPool;

/** \brief Open serial ports and spawn rx threads
 * 
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
int initAuserial(void) {

 int i;
 int returnValue = 1;

  // Initialization of threads
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  //Open and spawn bus threads
  for (i = 0; i < len.bus; i++) {
    //Open serial
    busses[i].fd = open(busses[i].devString, O_RDWR);
    if (busses[i].fd < 0) 
    {
      fprintf(stderr,"   AuSerial: Error opening %s on %s\n",busses[i].name,busses[i].devString);
      returnValue = -1;
    } else {
			//Setup serial
			if (set_serial(busses[i].fd,busses[i].baudrate) == -1) {
				fprintf(stderr,"   AuSerial: Can't set serial %s on %s parameters\n",busses[i].name,busses[i].devString);
				returnValue = -1;
			} else {
		
				//Mark bus as enabled
				busses[i].enabled = 2;
		
				//Create RX Thread
				if (pthread_create(&busses[i].threadId, &attr, serialrx_task, &i)) {
						perror("   AuSerial: Can't start serial receive thread");
						busses[i].enabled = 0;
						returnValue = -1;
					} else {
					
						//Wait for thread to start
						while (busses[i].enabled > 1) usleep(100000);
				}
			}
		}
  }

  return returnValue;

}

/** \brief Transmit data to serial busses
 * 
 *  * \param[in] int tick
 * Tick counter from main program
 * 
 * \returns int status
 * Status of the transmission process. Negative on error.
 */
extern int periodic(int tick) {

  int busId, cmdId, varId;
  char sendFlag = 0;
  int bytesTx = 0; 
  
  //Loop through the busses
  for (busId = 0; ((busId < len.bus) && (busses[busId].enabled)); busId++) {
	
		//Reset RX, if more than 32 bytes was padded to the bus
		if (busses[busId].lastPad > 32) busses[busId].resetRx = 1;
	
    //Send the the request commands first
    for(cmdId = 0; cmdId < len.command; cmdId++) {
      if (commands[cmdId].type == 'r') {
        //Check if any write variables are updated for this command
        for(varId = 0; ((varId < len.variable) && (!sendFlag)); varId++) {
          if ((varPool[varId].cmdId == cmdId) && (varPool[varId].dir == 'w')) {
            if(isUpdated(varPool[varId].dir,varPool[varId].dbId)) {
              sendFlag = 1;
            }
          }
        }
        //Send the command if variables were updated
        if (sendFlag) {
          bytesTx += sendCommand(busId,cmdId);
          sendFlag = 0;
          //Control if too many bytes are transmitted
          if (bytesTx > busses[busId].maxTxByte) {
            fprintf(stderr,"AuSerial: TX Overflow of %d[%d] at tick %d @ command: %s!\n",
                  bytesTx,busses[busId].maxTxByte,tick,commands[cmdId].name);
            //Other handling here!
          }
        }
      }
    }
    //Then send the poll commands
    for(cmdId = 0; cmdId < len.command; cmdId++) {
      if (commands[cmdId].type == 'p') {
        //Check if period is set (0 means every time)
        if (commands[cmdId].period == 0) {
          bytesTx += sendCommand(busId,cmdId);
        //Check if period is matched
        } else if (((tick % commands[cmdId].period)- commands[cmdId].offset) == 0) {
          bytesTx += sendCommand(busId,cmdId);
        }
        //Control if too many bytes are transmitted
        if (bytesTx > busses[busId].maxTxByte) {
            fprintf(stderr,"AuSerial: TX Overflow of %d[%d] at tick %d @ command: %s!\n",
                  bytesTx,busses[busId].maxTxByte,tick,commands[cmdId].name);
          //Other handling here!
        }
      }
    }

    //Pad the bus with zeroes to sync potientially lost devices
    busses[busId].lastPad = busses[busId].maxTxByte - bytesTx;
    char ch = 0;
		int i;
    for (i = 0; i < busses[busId].lastPad; i++) secureWrite(busses[busId].fd,&ch,1);

  } //Bus loop ends
	return 1;
}



/** \brief Thread entry point for serial RX 
 * 
 * \param[in] void *busStruct
 * Index of the serial bus
 * 
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
void *serialrx_task(void *busId) {

	int id = *((int*) busId); //Get id value of bus
  //struct busInfo *busData = &busses[id];
  char rxBuf[32];
  int n;
	
	int sfd = busses[id].fd;
	
	//Copy data from busInfo to local struct
	//memcpy(&busData,busStruct, sizeof(struct busInfo));

  printf("   AuSerial: %s RX Thread is running on %s\n",busses[id].name,busses[id].devString);
	
	//Mark thread as running
	busses[id].enabled = 1;

  //Recieve data from the serial bus
  while(busses[id].enabled) {
      
			//Recieve length byte
      if (read(sfd,&rxBuf[0],1) < 0) {
        busses[id].enabled = 0;
        fprintf(stderr,"   AuSerial: Error reading %s on %s\n",busses[id].name,busses[id].devString);
        break;
      }
      busses[id].resetRx = 0;//Make sure reset flag is no longer set
      //Then revice the payload
      for (n = 1; ((n <= rxBuf[0]) && (n < 32)); n++) {
        //Recieve one byte
        if (read(sfd,&rxBuf[n],1) < 0) {
          busses[id].enabled = 0;
          fprintf(stderr,"   AuSerial: Error reading %s on %s\n",busses[id].name,busses[id].devString);
					if (errno == EAGAIN) fprintf(stderr,"   AuSerial: No data was ready for non-blocking I/O\n");
					else if (errno == EBADF) fprintf(stderr,"   AuSerial: Serial port is no longer open\n");
					else if (errno == EFAULT) fprintf(stderr,"   AuSerial: Buffer is outside address-space\n");
					else  fprintf(stderr,"   AuSerial: Other error %d\n",errno);
          break;
        }
        //Restart recive if reset flag is set
        //Recover sync
        if (busses[id].resetRx) {
          rxBuf[0] = rxBuf[n];
          n = 0; //Set n=1 (because of n++)
          busses[id].resetRx = 0;
        } 
      }

      //Parse rx buffer for matching variables
      for(n = 0; n < len.variable; n++) {
        //Check if the device is associated with the variable
        if (devices[varPool[n].deviceId].device == (rxBuf[1] & 0x0F)) {
          //Check if the command is accociated with the variable
          if (commands[varPool[n].cmdId].cmd == ((rxBuf[1] >> 4) & 0x0F)) {
            rxBuf2rxVariable(rxBuf,n); //Parse command
          }
        }
      }

  //Debug print
#if DEBUG
  if (rxBuf[0] != 0) {
    printf("RX: ");
    for(n = 0; n <= rxBuf[0]; n++)
      printf("0x%02X ",(unsigned char)rxBuf[n]);
    printf("\n");
    }
#endif
  }
  fprintf(stderr,"AuSerial: Out of RX Thread for %s on %s!\n",busses[id].name,busses[id].devString);

	return NULL;
}

/** \brief Send a command to the serial bus
 *
 * Parses variables and creates the transmission buffer
 * to send data to the serial bus
 * 
 * \param[in] int bus
 * Index of the serial bus
 * 
 * \param[in] int cmd
 * Index of the command to be transmitted
 * 
 * \returns int txLen
 * Number of bytes transmitted in the function
 */
int sendCommand(int busId, int cmdId) {

  char txBuf[64];
  char txLen = 0, newLen;
  int32_t varId;

  //Clear txBuffer
  memset(txBuf,0,64);
  
  //Loop through variables and create tx buffer
  for (varId = 0; varId < len.variable; varId++) {
    //Only send write variables
    if ((varPool[varId].cmdId == cmdId) && (varPool[varId].dir == 'w')) {
      //byte-parse the variable to txBuffer
      newLen = txVariable2txBuf(varId,&txBuf[2]); //Save 2 byte for len+cmd
      //Update tx length
      if (txLen < newLen) txLen = newLen;
    }
  }

  //Finalize command-buffer
  txLen += 1; //Add (cmd+id) to length
  txBuf[0]  = txLen; //Set command length
  txBuf[1]  = devices[commands[cmdId].deviceId].device & 0x0F; //Set device id
  txBuf[1] |= ((commands[cmdId].cmd << 4) & 0xF0); //Set command

  //Add length and padding to transmission length
  txLen += 1 + commands[cmdId].pad;

  //Write to serial bus
  if (secureWrite(busses[busId].fd,txBuf,txLen) <= 0) {
    busses[busId].enabled = 0;
    fprintf(stderr,"   AuSerial: Error writing to %s on %s\n",busses[busId].name,busses[busId].devString);
  }
  //Debug print
#if DEBUG
	int i;
  printf("TX: ");
  for(i = 0; i < txLen; i++)
    printf("0x%02X ",(unsigned char)txBuf[i]);
  printf("\n");
#endif

  return txLen;
  
}

/** \brief Byte-parse write variable to transmission buffer
 * 
 * \param[in] int var
 * Index of the source variable
 * 
 * \param[in] char *txBuf
 * Pointer to the transmission buffer's payload area
 * (must be allocated in advance)
 * 
 * \returns char txLen
 * Last byte used of tx-buffer
 */
char txVariable2txBuf(int varId, char *txBuf) {

  int32_t tmpData, i;
  int8_t tmpChar;
  char txLen = -1; //1 is added in the end

  //Get variable from database
  tmpData = getWriteVariable(varPool[varId].dbId,varPool[varId].dbIndex);

  //Invert the variable if invert flag is set
  if (varPool[varId].invert) tmpData = -tmpData;

  //Start by byte-to-byte parsing
  for(i = 0; i < 4; i++) {
    if (varPool[varId].parse.byte[i] != -1) {
      txBuf[varPool[varId].parse.byte[i]] = (char)(tmpData >> (8*i)); 
    }
    //Save length
    if (txLen < varPool[varId].parse.byte[i]) txLen = varPool[varId].parse.byte[i];
  }

  //Then bit-to-bit parsing
  //Loop through until index is -1
  for(i = 0; ((varPool[varId].parse.bwIndex[i] >= 0) && (i < 32));i++) {
    tmpChar = 0x01 & (tmpData >> varPool[varId].parse.bwIndex[i]);
    txBuf[varPool[varId].parse.bwByte[i]] |= (tmpChar << (varPool[varId].parse.bwBit[i]));
    //Save length
     if (txLen < varPool[varId].parse.byte[i]) txLen = varPool[varId].parse.byte[i];
  }

  return txLen + 1; //Go from index to length
}

/** \brief Byte-parse recieve buffer into variable
 *
 * If the recieve buffer is not long enough to hold
 * all bytes required to parse variable the 
 * variable is NOT updated
 * 
 * If signedness or inversion is set in the configuration
 * this is also handled in this function
 * 
 * \param[in] char *rxBuf
 * Pointer to the recieve buffer
 *
 * \param[in] int var
 * Index of the destination variable
 * 
 * \returns char status
 * If the variable is updated or not
 */
char rxBuf2rxVariable(char *rxBuf, int varId) {

int32_t i, tmpData = 0;
char tooShort = 0;  //Is the message too short to hold the variable
char tmpChar = 0;
char *payload = &rxBuf[2]; //Point to payload area
char signBit = 0; //What bit are the sign placed in?


  //Start byte-to-byte parsing
  for (i = 0; i < 4; i++) {
    //Only parse the indexes that are not -1
    if (varPool[varId].parse.byte[i] >= 0) {
      //Mark as overflow, if msg is to short for parsing
      if (rxBuf[0] < varPool[varId].parse.byte[i]+2) tooShort = 1;
      //transfer data unsigned
      tmpData |= (uint32_t)((uint8_t)payload[varPool[varId].parse.byte[i]] << (8*i));
      //Save signbit
      signBit = (8*(i+1))-1;
    } 
  }

  //Bit-by-bit parsing
  for(i = 0; ((varPool[varId].parse.bwIndex[i] >= 0) && (i < 32));i++) {
    //Mark as overflow, if msg is to short for parsing
    if (rxBuf[0] < varPool[varId].parse.bwByte[i]+2) tooShort = 1;
    //Save signbit
    if (signBit < varPool[varId].parse.bwIndex[i]) signBit = varPool[varId].parse.bwIndex[i];
    //Parse data
    tmpChar = 0x01 & (payload[varPool[varId].parse.bwByte[i]] >> varPool[varId].parse.bwBit[i]);
    tmpData |= tmpChar << varPool[varId].parse.bwIndex[i];
  }

  //Sign extend, if variable is signed
  if (varPool[varId].sign) {
    tmpChar = 0x01 & (tmpData >> signBit); //Extract signbit
    for(i = signBit+1; i < 32; i++) {
      tmpData |= (tmpChar << i);
    }
  }

  //Invert the variable if invert flag is set
  if (varPool[varId].invert) tmpData = -tmpData;

  //Update variable, if rxBuf was long enough
  if (tooShort) {
    i =  -1;
  } else {
    i = setVariable(varPool[varId].dbId,varPool[varId].dbIndex,tmpData);
  }

  return i;
}

/************************** XML Initialization **************************/

///Struct for shared parse data
typedef struct  {
    int depth;              /**< Depth of the XML tree */
    char skip;              /**< Skip all tags below this level */
    char enable;            /**< Enable use of this module */
		char found;
    struct serialParse {     /**< Bookkeeping for parsing serial bus parameters */
      int bus;
      int device;
      int cmd;
      int var;
      int oldCount[4];
      int index;
      char varDir;
      char deviceName[64];
      char cmdName[64];
      char varName[64];
    } auserial;
  }parseInfo;

//Parsing functions
void XMLCALL auserialStartTag(void *, const char *, const char **);
void XMLCALL auserialEndTag(void *, const char *);


/** \brief Initialize the AU Serial HAL
 *
 * Reads the XML file and creates the busses, devices and
 * variables. Using this function is NOT RT Safe.
 * 
 * Must only be used in initialization.
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

  //Zero all structures
  memset(&xmlParse,0,sizeof(parseInfo));
  memset(&len,0,sizeof(struct confLen));

  //Print initialization message
  //Find revision number from SVN Revision
	char *i,versionString[20] = REVISION, tempString[10];
	i = strrchr(versionString,'$');
	strncpy(tempString,versionString+6,(i-versionString-6));
	tempString[(i-versionString-6)] = 0;
  printf("AuSerial: Initializing AU Serial bus HAL %s.%s\n",AUSERIALVERSION,tempString);

   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "AuSerial: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, auserialStartTag, auserialEndTag);
   //Setup shared data
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("AuSerial: Error reading: %s\n",filename);
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
    fprintf(stderr, "AuSerial: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);	
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <auserial> XML tag found in plugins section\n");
		return -1;
	}

  //Initialize AuSerial after XML parsing
  if (xmlParse.enable) done = initAuserial();

 return done;
}

///Parse XML start tag
void XMLCALL
auserialStartTag(void *data, const char *el, const char **attr)
{
  int i = 0;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("auserial",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("auserial",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   AuSerial: Use of AuSerial disabled in configuration\n"); 
      info->skip = info->depth; //Skip the rest of auserial
    }
  } 
  //Bus definition
  else if (strcmp("bus",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
      info->skip = info->depth; //Skip following tags in this depth
    } else {
        parseBustag(data,attr);
    }
  }
  //Device definition
  else if (strcmp("device",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 5) {
      printf("Error: Wrong depth for the %s tag\n",el);
      info->skip = info->depth; //Skip following tags in this depth
    } else {
      parseDevicetag(data,attr);
    }
  } 
  //Command definition
  else if (strcmp("cmd",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 6) {
      printf("Error: Wrong depth for the %s tag\n",el);
      info->skip = info->depth; //Skip following tags in this depth
    } else {
      //Call parse command
      parseCmdtag(data,attr);
    }
  }
 //Array definition
 else if (strcmp("array",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 7) {
      printf("Error: Wrong depth for the %s tag\n",el);
      info->skip = info->depth; //Skip following tags in this depth
    } else {
      //Parse the start of the variable definition
      parseVariablestart(data,attr);
   }
  }
  //Variable definition
  else if (strcmp("variable",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 7) {
      printf("Error: Wrong depth for the %s tag\n",el);
      info->skip = info->depth; //Skip following tags in this depth
    } else {
      //Call the initial variable parser
      if (parseVariablestart(data,attr) >= 0) {
        //Call variable setup-parsing function
        parseVariabletag(data,attr);
      }
    }
  }
  //Parse each element in a array (joined to one database variable)
  else if (strcmp("element",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 8) {
      printf("Error: Wrong depth for the %s tag\n",el);
      info->skip = info->depth; //Skip following tags in this depth
    } else {
      //Parse the individual variable-configuration
      parseVariabletag(data,attr);
    }
  }
}

///Parse XML Endtag
void XMLCALL
auserialEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;
  int i;

  //Only process end tag if not skipping
  if (info->skip == 0) { 
    //Handle end of variable or array tags
    if ((strcmp("array",el) == 0) ||(strcmp("variable",el) == 0)) {
        parseVariableend(data);
    } 
    //Print BUS information when bus tag is finished
    else if (strcmp("bus",el) == 0) {
      printf("   AuSerial: BUS%d %s on %s at %d baud with %d byte capacity\n",
            info->auserial.bus, busses[info->auserial.bus].name, busses[info->auserial.bus].devString,
            busses[info->auserial.bus].baudrate, busses[info->auserial.bus].maxTxByte);
      printf("             Devices:");
      for (i = info->auserial.oldCount[1]; i < len.device; i++) 
        printf(" %s",devices[i].name);
      printf("\n");
      info->auserial.oldCount[1] = len.device;

    }
  }

  //Stop skipping, when back to level before skip
  if (info->depth < info->skip) {
    info->skip = 0;
  } 
}

/** \brief Parse <bus>-tag
 *
 * Parses the <bus> XML tag and creates the required data-space
 * for holding the data
 * 
 * \param[in] void* data
 * Shared user-data for parsing
 *
 *  * \param[in] const char **attr
 * XML tag arrtibutes
 * 
 * \returns int status
 * Status of parsing
 */
int parseBustag(void *data, const char **attr) {

  parseInfo *info = (parseInfo *) data;
  int i, safeCnt = 0;

      //Create new bus index and transfer configuration
      busses = realloc(busses,sizeof(struct busInfo) * (len.bus+1));
			if (busses == NULL) {
				fprintf(stderr,"   AuSerial: Failed allocating memory for busses\n");
				return -1;
			}
      memset(&busses[len.bus],0, sizeof(struct busInfo));
      for(i = 0; attr[i]; i+=2) if ((strcmp("name",attr[i]) == 0)) { 
        strncpy(busses[len.bus].name,attr[i+1],63); //Transfer name
        safeCnt++;
      } 
      for(i = 0; attr[i]; i+=2) if ((strcmp("dev",attr[i]) == 0)) { 
        strncpy(busses[len.bus].devString,attr[i+1],63); //Transfer device
        safeCnt++;
      }
      for(i = 0; attr[i]; i+=2) if ((strcmp("baudrate",attr[i]) == 0)) {
        busses[len.bus].baudrate = atoi(attr[i+1]); //Transfer baudrate
        safeCnt++;
      }
      //Holdoff - How many bytes should be left of the bus-capacity
      for(i = 0; attr[i]; i+=2) if ((strcmp("holdoff",attr[i]) == 0)) {
        busses[len.bus].holdoff = atoi(attr[i+1]); //Transfer holdoff
      }

      //Setup parsing variables
      info->auserial.bus = len.bus;

      //ONLY create bus if enough parameters are defined!
      if (safeCnt >= 3) {
        //Calculate bus capacity
        double txUsec;
        //Byte pr µSec (10 bit pr. byte in serial tx)
        txUsec = (double)(busses[len.bus].baudrate / 1e6) / 10.0; 
        txUsec *= getSchedulerPeriod(); //Multiply by sheduler period in µSec
        busses[len.bus].maxTxByte = (int)txUsec - busses[len.bus].holdoff;
        len.bus++; //Increment bus counter
        i = 1; 
      } else {
        fprintf(stderr,"   AuSerial: ERROR: Not enough parameters for bus%d\n",len.bus);
        info->skip = info->depth; //Skip the children
        i = -1;
      }

  return i;
}

/** \brief Parse <device>-tag
 *
 * Parses the <device> XML tag and creates the required data-space
 * for holding the data
 * 
 * \param[in] void* data
 * Shared user-data for parsing
 *
 *  * \param[in] const char **attr
 * XML tag arrtibutes
 * 
 * \returns int status
 * Status of parsing
 */
int parseDevicetag(void *data, const char **attr) {

  parseInfo *info = (parseInfo *) data;
  int i, safeCnt = 0;

  //Create new bus index and transfer configuration
  devices = realloc(devices,sizeof(struct deviceInfo) * (len.device+1));
	if (devices == NULL) {
		fprintf(stderr,"   AuSerial: Failed allocating memory for devices\n");
		return -1;
	}
  memset(&devices[len.device],0, sizeof(struct deviceInfo));

  //Save device information for further parsing only (no local database of devices)
  for(i = 0; attr[i]; i+=2) if ((strcmp("name",attr[i]) == 0)) {
    strncpy(devices[len.device].name,attr[i+1],63); //Transfer name
    safeCnt++;
  }
  for(i = 0; attr[i]; i+=2) if ((strcmp("id",attr[i]) == 0)) {
    devices[len.device].device = strtoul(attr[i+1],0,16); //Transfer idnumber from HEX
    safeCnt++;
  }

  devices[len.device].busId = info->auserial.bus;
  info->auserial.device = len.device; 

  //ONLY create device if enough parameters are set
  if (safeCnt >= 2) {
     len.device++;
     i = 1; 
   } else {
     fprintf(stderr,"   AuSerial: ERROR: Not enough parameters for device%d\n",len.device);
     info->skip = info->depth; //Skip the children
     i = -1;
   }

  return i;
}

/** \brief Parse <cmd>-tag
 *
 * Parses the <cmd> XML tag and creates the required data-space
 * for holding the data
 * 
 * \param[in] void* data
 * Shared user-data for parsing
 *
 *  * \param[in] const char **attr
 * XML tag arrtibutes
 * 
 * \returns int status
 * Status of parsing
 */
int parseCmdtag(void *data, const char **attr) {

  parseInfo *info = (parseInfo *) data;
  int i,j, safeCnt = 0;

  //Allocate memory for another command
  commands = realloc(commands,sizeof(struct cmdInfo) * (len.command +1));
  if (commands == NULL) {
    fprintf(stderr, "   AuSerial: Failed to allocate memory for command pool %d\n",(len.command +1));
    return -1;
  }
  memset(&commands[len.command],0,sizeof(struct cmdInfo));

  //Type (poll or request) (required)
  for(i = 0; attr[i]; i+=2) if ((strcmp("type",attr[i]) == 0)) {
    if ((strcmp("poll",attr[i+1]) == 0)) { //Poll type variable, setup all poll-stuff
      commands[len.command].type = 'p';
      for(j = 0; attr[j]; j+=2) if ((strcmp("period",attr[j]) == 0)) 
        commands[len.command].period = atoi(attr[j+1]);
      for(j = 0; attr[j]; j+=2) if ((strcmp("offset",attr[j]) == 0)) 
        commands[len.command].offset = atoi(attr[j+1]);
      safeCnt++;
    } else if ((strcmp("request",attr[i+1]) == 0)) {
      commands[len.command].type = 'r';
      safeCnt++;
    }
  }
  //Name (required)
  for(i = 0; attr[i]; i+=2) if ((strcmp("name",attr[i]) == 0)) { 
    strncpy(commands[len.command].name,attr[i+1],63); //Transfer name
    safeCnt++;
  }
  //Command from HEX (required)
  for(i = 0; attr[i]; i+=2) if ((strcmp("cmd",attr[i]) == 0)) {
    commands[len.command].cmd = strtoul(attr[i+1],0,16); 

    safeCnt++;
  }
  //Padding (not required)
  for(i = 0; attr[i]; i+=2) if ((strcmp("pad",attr[i]) == 0)) {
    commands[len.command].pad = atoi(attr[i+1]);
  }

  //Transfer last variables
  commands[len.command].busId     = info->auserial.bus;
  commands[len.command].deviceId  = info->auserial.device;
  info->auserial.cmd              = len.command;

  //ONLY create command if enough parameters are set
  if (safeCnt >= 3) {
    /*printf("       CMD %d: %s type %c, cmd %d, pad %d (period %d, offset %d)\n",
          len.command,info->auserial.cmdName,commands[len.command].type,commands[len.command].cmd,
           commands[len.command].pad,commands[len.command].period,commands[len.command].offset);*/
    len.command++;
    i = 1; 
   } else {
     printf("   AuSerial: Not enough parameters for cmd%d (Type:%c,Name:%s,Cmd:%d)\n",
            len.command,commands[len.command].type,info->auserial.cmdName,
            commands[len.command].cmd);
     info->skip = info->depth; //Skip the children
     i = -1;
   }
return i;
}

/** \brief Parse start of <variable> and <array> tag
 *
 * Must only be used in initialization.
 * 
 * \param[in] void* data
 * Shared user-data for parsing
 *
 *  * \param[in] const char **attr
 * XML tag arrtibutes
 * 
 * \returns int status
 * Status of parsing.
 */
int parseVariablestart(void *data,const char **attr) {
  parseInfo *info = (parseInfo *) data;
  int i, safeCnt = 0;

  for(i = 0; attr[i]; i+=2) if ((strcmp("name",attr[i]) == 0)) { 
    strncpy(info->auserial.varName,attr[i+1],63); //Transfer variable name
    safeCnt++;
  }
  for(i = 0; attr[i]; i+=2) if ((strcmp("dir",attr[i]) == 0)){
    if (*attr[i+1] == 'r' || *attr[i+1] == 'w') {
      info->auserial.varDir = *attr[i+1]; //Transfer variable direction
      safeCnt++;
    }
  }

  //ONLY create variable if enough parameters are set
  if (safeCnt >= 2) {
     info->auserial.index = 0;
     i = 1; 
   } else {
     fprintf(stderr,"   AuSerial: ERROR: Not enough parameters for variable/array%d\n",len.variable);
     info->skip = info->depth; //Skip the children
     i = -1;
   }
   return i;
}

/** \brief Parse <variable> and <element> tag
 *
 * Must only be used in initialization.
 * 
 * \param[in] void* data
 * Shared user-data for parsing
 *
 *  * \param[in] const char **attr
 * XML tag arrtibutes
 * 
 * \returns int status
 * Status of parsing.
 */
int parseVariabletag(void *data,const char **attr) {

  parseInfo *info = (parseInfo *) data;
  int i,j,k;
  int num;
  char p1[32], p2[32];
  char compString[32];

  //Allocate memory for variable
  varPool = realloc(varPool,(len.variable + 1) * sizeof(struct varInfo));
  if (varPool == NULL) {
    fprintf(stderr,"   AuSerial: Failed to allocate memory for variable %d\n",len.variable+1);
    return -1;
  }
  memset(&varPool[len.variable],0,sizeof(struct varInfo));

  //Setup bus,device and cmd for variable
  varPool[len.variable].dbIndex = info->auserial.index;
  varPool[len.variable].busId = info->auserial.bus;
  varPool[len.variable].deviceId = info->auserial.device;
  varPool[len.variable].cmdId = info->auserial.cmd;
  varPool[len.variable].dir = info->auserial.varDir;
  //Set command and device connected to the variable
  varPool[len.variable].cmd = commands[info->auserial.cmd].cmd;
  varPool[len.variable].device = devices[info->auserial.device].device;

  //Set all parsing variables to -1
  for(i = 0; i < 4; i++) varPool[len.variable].parse.byte[i] = -1;
  for(i = 0; i < 32; i++) varPool[len.variable].parse.bwIndex[i] = -1;

  //printf("         VAR%d : ",len.variable);

  //Find and setup parsing variables
  //Bytewise parsing
  //eks: byte2="3" (byte2 of variable comes/goes from/to bus byte3)
  for(i = 0; i < 4; i++) { 
    snprintf(compString,32,"byte%d",i); //Create compare string
    for(j = 0; attr[j]; j+=2) if ((strcmp(compString,attr[j]) == 0)) {
      varPool[len.variable].parse.byte[i] = atoi(attr[j+1]);
      //printf("by%d=%d ",i,atoi(attr[j+1]));
    }
  }
  //Bitwise parsing
  //eks: b3="3,2" (translates bit3 of variable comes/goes from/to bit3 of bus byte2)
  k = 0;
  for (i = 0; i < 32; i++) { //Loop through all possible 32 bX values
    snprintf(compString,32,"b%d",i); //Create compare string
    for(j = 0; attr[j]; j+=2) if ((strcmp(compString,attr[j]) == 0)) {
       strncpy(compString,attr[j+1],32); //attr is const
       //printf("Bit start %s = %s\n",attr[j],compString);
       num = sscanf(attr[j+1],"%2s%2s",p1,p2);//for
       if (num == 2) {
         varPool[len.variable].parse.bwIndex[k] = i;
         varPool[len.variable].parse.bwBit[k]   = (char)atoi(p1);
         varPool[len.variable].parse.bwByte[k]  = (char)atoi(p2);
         k++;
         //printf("%d:b%d=(%d,%d) ",k,i,atoi(p1),atoi(p2));
       }
    }
  }

  //Get the "signed tag"
  for(i = 0; attr[i]; i+=2) if ((strcmp("signed",attr[i]) == 0)){
    if ((strcmp("true",attr[i+1]) == 0)) {
      varPool[len.variable].sign = 1; //Set the unsigned flag
    }
  }

  //Get the "invert tag"
  for(i = 0; attr[i]; i+=2) if ((strcmp("invert",attr[i]) == 0)){
    if ((strcmp("true",attr[i+1]) == 0)) {
      varPool[len.variable].invert = 1; //Set the invert flag
    }
  }

  //printf("\n");

  //Increment variable couters
  len.variable++;
  info->auserial.index++;

  return 1;

}

/** \brief End variable parsing
 *
 * Ends variable parsing by creating database variables and
 * setting up the variable structure
 * 
 * \param[in] void* data
 * Shared user-data for parsing
 * 
 * \returns int status
 * Status of parsing
 */
int parseVariableend(void *data) {

  parseInfo *info = (parseInfo *) data;
  int tempIndex, i;

  //Create variable in the database
  tempIndex = createVariable(info->auserial.varDir,info->auserial.index,info->auserial.varName);

  //printf("         VARPOOL%d %s: dir %c len %d\n",
  //        tempIndex,info->auserial.varName,info->auserial.varDir,info->auserial.index);

  //Set database index in the 
  for (i = info->auserial.index; i > 0; i--) {
    varPool[len.variable - i].dbId = tempIndex;
  }
  return 1;
}
