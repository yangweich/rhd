 /** \file rs232linesensor.c
 *  \ingroup hwmodule
 *
 *   Interface for RS232 Linesensor module
 *
 * This plugin is based on SMRDSerial and is generally deprecated
 * as the linesensor should be placed on the RS485 bus instead.
 *
 * However, this plugin still provides the support for the module
 * for legacy purposes
 *
 *******************************************************************/
/***************************** Plugin version  *****************************/
#define RS232LSVERSION    "1.1"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $:"
 #define ID               "$Id: rs232linesensor.c 59 2012-10-21 06:25:02Z jcan $"
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

//RHD Core headers
#include <rhd.h>
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "rs232linesensor.h"


/******** Global variables *************/
int iLs;
uint8_t txBuf[32];
int iterator;
int32_t inputRS232[32];
int rs232;  //File descriptors
int lsRunning = -1;
char serialDev[64];
int baudrate;
int sensorSize = 0;

//SMRD Functions
int initLinesensor(void);
void *rs232_task(void *);
pthread_t rs232_thread;
pthread_attr_t attr;

#define BLOCK_MAX 200
#define XMIT_BYTES 104    /* about 10ms at 115.2 Kbaud */

/// Init SMRD
int initLinesensor(void) {

   //Open first serial port
   if ((rs232 = open (serialDev, O_RDWR /*| O_NONBLOCK*/)) == -1) {
     fprintf(stderr,"   232Linesensor: Can't open first serial port: %s\n",serialDev);
      lsRunning = -1;
      return -1;
   } else if (set_serial(rs232,baudrate) == -1) {
    fprintf(stderr,"   232Linesensor: Can't set first serial port parameters\n");
    fprintf(stderr,"   232Linesensor: SMRD is NOT running!\n");
    lsRunning = -1;
    return -1;
   } else lsRunning = 1;
 
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED); 
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	lsRunning = 2;
  if (rs232 != -1)
    if (pthread_create(&rs232_thread, &attr, rs232_task, 0)) {
      perror("   232Linesensor: Can't start RS232 linesensor receive thread");
      return -1;
    }

  /****** Create database variables if all is ok **************/
  iLs   = createVariable('r',sensorSize,"linesensor");

	int waitCount = 0;
	while (lsRunning > 1) {
		 usleep(100000); //Don't return before threads are running
		if (waitCount >= 10) return -1;
		waitCount++;
	}
  return 1;
}

#define LS_READ_BYTES 20

///RS232  Recieve thread
void *rs232_task(void *not_used)
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE))
    {
      perror("mlockall");
      exit(-1);
    }

  { /* use real-time (fixed priority) scheduler
     * set priority to one less than the maximum
     */
    struct sched_param param;

    param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
    if (sched_setscheduler(0, SCHED_RR, &param)) {
  		perror("setscheduler");
  		pthread_exit(0);
     }

  }

  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, "signal: can't ignore SIGPIPE.\n");

  fprintf(stderr, "   RS232Linesensor: RS232 LS rx_task running\n");

	//Mark thread as running
	lsRunning = 1;

  while (lsRunning)
    {
      unsigned char buf[ENET_BYTES_MAX];

      int c = read(rs232, buf + 2, LS_READ_BYTES);
      if (c <= 0) {
    		fprintf(stderr, "   RS232Linesensor: error reading rs232");
				lsRunning = 0;
    		pthread_exit(0);
  		}
      buf[0] = (c + 1) | ENET_TYPE_232;
      buf[1] = 's';
      if ((ENET_BYTES(buf) >= 2+sensorSize) && (buf[1] == 's')
      && !memchr(buf+2, 0, sensorSize)) /* filter out zero data */
      {
        int len,i;
        len=ENET_BYTES(buf);
        for (i = 0; ((i < len-1) && (i < 32)); i++)  inputRS232[i] = buf[2+i];
        setArray(iLs, sensorSize, inputRS232);
    }
    }
    
    fprintf(stderr,"RS232 Linesensor: Shutting down thread\n");

	return NULL;
}

///Transmit linesensor request to serial bus (called periodically)
extern int periodic(int tick)
{

	char req = 's';
	int returnValue = 1;

	if (!lsRunning) return -1;
  
  /*** Request all variables from the bus ***/
  if (rs232 != -1) {       /* get line sensor data */
    if (secureWrite(rs232, &req, 1) < 0) returnValue = -1;
  }

  return returnValue;
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
void XMLCALL lsStartTag(void *, const char *, const char **);
void XMLCALL lsEndTag(void *, const char *);

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
  printf("RS232Linesensor: Initializing RS232 Linesensor HAL %s.%s\n",RS232LSVERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "RS232Linesensor: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, lsStartTag, lsEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("RS232Linesensor: Error reading: %s\n",filename);
    return -1;
  }
  //Get the length of the file
	fseek(fp,0,SEEK_END);
	xmlFilelength = ftell(fp); //Get position
	fseek(fp,0,SEEK_SET); //Return to start of file

	//Allocate text buffer
	xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
	if (xmlBuf == NULL) {
		fprintf(stderr, "   RS232Linesensor: Couldn't allocate memory for XML File buffer\n");
		return -1;
	}
	memset(xmlBuf,0,xmlFilelength);
  len = fread(xmlBuf, 1, xmlFilelength, fp);
  fclose(fp);

  //Start parsing the XML file
  if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) {
    fprintf(stderr, "RS232Linesensor: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <rs232linesensor> XML tag found in plugins section\n");
		return -1;
	}

  //Start crossbow thread after init
  if (xmlParse.enable) done = initLinesensor();



 return done;
}

void XMLCALL
lsStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("rs232linesensor",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("rs232linesensor",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   RS232Linesensor: Use of SMRD disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(serialDev,attr[i+1],63);
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate = atoi(attr[i+1]);  
    printf("   RS232Linesensor: RS232 port %s at %d baud\n",serialDev,baudrate);
  
	} else if (strcmp("linesensor",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("size",attr[i]) == 0) sensorSize = atoi(attr[i+1]);  
    printf("   RS232Linesensor: Linesensor size %d\n",sensorSize);
  } 

}

void XMLCALL
lsEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}


