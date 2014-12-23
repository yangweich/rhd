/**  Very basic serial communication with modified Pixhawk setup.
 *   Reads gyro and accelerometer data.
 *   SH, 30/04-2014.
 */
 /***************************************************************************
 *                  Copyright 2008 Anders Billesø Beck                     *
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

# include <stdlib.h>
# include <stdio.h>
# include <unistd.h>
# include <netdb.h>
# include <fcntl.h>
# include <string.h>
# include <pthread.h>
# include <signal.h>
# include <string.h>
# include <time.h>
# include <sys/time.h>
# include <unistd.h>
# include <errno.h>
# include <termios.h>
# include <sys/ioctl.h>
# include <linux/serial.h>
# include <expat.h>
# include <math.h>
# include <database.h>

# include "pixhawk.h"

//Definitions
int  pixhawkdev;      ///Port file pointer
char pixhawkds[64]; ///String to hold device
int  baudrate = 0;
static volatile char pixhawkRunning = 0;
///Database indexes
int ipxgyrox, ipxgyroy, ipxgyroz, ipxaccx, ipxaccy, ipxaccz;

// Threads are being defined
pthread_t pixhawk_thread_read;
pthread_attr_t attr;

// Buffer for serial comm
# define MaxBufSize 200
char buf[MaxBufSize];

// Function prototypes
int init_pixhawk(void);
int set_serial(int fd, int baud);
void *pixhawk_task_read(void *);

/** \brief Initialize 
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
int init_pixhawk(void) 
{
  // Open RS232 port
  pixhawkdev = open(pixhawkds, O_RDWR);
  if(pixhawkdev < 0) 
  {
    fprintf(stderr,"   Pixhawk: Error opening %s\n", pixhawkds);
    return -1;
  }
  
  //Set baudrate 
  if(set_serial(pixhawkdev, baudrate) == -1)  
  {
    fprintf(stderr,"   Pixhawk: Can't set Pixhawk serial port parameters\n");
    return -1;
  }
  pixhawkRunning = 1;

  //Create variables
  ipxgyrox      = createVariable('r', 1, "pxgyrox");
  ipxgyroy      = createVariable('r', 1, "pxgyroy");
  ipxgyroz      = createVariable('r', 1, "pxgyroz");
  ipxaccx       = createVariable('r', 1, "pxaccx");
  ipxaccy       = createVariable('r', 1, "pxaccy");
  ipxaccz       = createVariable('r', 1, "pxaccz");

  // Initialization and starting of threads
  if(pthread_create(&pixhawk_thread_read, &attr, pixhawk_task_read, 0))
  {
    perror("   Pixhawk: Can't start read thread");
    return -1;
  }

  return 1;
}

/** \brief Initialize Shut down rx thread
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
int shutdownpixhawk(void) 
{
  pixhawkRunning = 0;
  pthread_join(pixhawk_thread_read, NULL);
  return 1;
}

void *pixhawk_task_read(void *not_used) 
{
  int i = -1;
  char bufNext;
  //Recieve from Pixhawk
  printf("   Pixhawk: Receive thread started\n");
  int buf_cnt = 0;
  float dgx, dgy, dgz, dax, day, daz;
  
  while(pixhawkRunning)
  {
    while(buf_cnt < MaxBufSize - 1 && pixhawkRunning)
    { /* space for more data */
      i = read(pixhawkdev, &bufNext, 1);
      if (i <= 0)
      { /* read error */
        pixhawkRunning = 0;
        fprintf(stderr,"Pixhawk: Error reading from Serial port, shutting down\n");
        break;
      }
      if (bufNext == '\n')
        // newline marks end of message
        break;
      
      else if(bufNext >= ' ')
      {
        buf[buf_cnt] = bufNext;
        buf_cnt++;
      // other control characters (like \r) are just skipped
      }
    }
    
    if (pixhawkRunning && i > 0)
    { /* new data buffer is ready for decoding */
      buf[buf_cnt] = '\0'; // terminate NMEA string - at '\n' char
      // set buffer index of ready buffer
      //bufIdx = (bufIdx + 1) % MaxBufCnt;
      // start decode thread
      //decodePost();
      
      //printf("%s\n", buf);
      
      if(sscanf(buf, "%f %f %f %f %f %f", &dax, &day, &daz, &dgx, &dgy, &dgz) == 6)
      {
        setVariable(ipxaccx, 0, dax * 100000);
        setVariable(ipxaccy, 0, day * 100000);
        setVariable(ipxaccz, 0, daz * 100000);
        setVariable(ipxgyrox, 0, dgx * 100000);
        setVariable(ipxgyroy, 0, dgy * 100000);
        setVariable(ipxgyroz, 0, dgz * 100000);        
      } 
      
      
      /* get next buffer to fill */
      buf_cnt = 0;
    }
  } //Ending Pixhawk loop

  close(pixhawkdev);
  fprintf(stderr,"Pixhawk: Shutdown Pixhawk read task\n");
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
void XMLCALL pixhawkStartTag(void *, const char *, const char **);
void XMLCALL pixhawkEndTag(void *, const char *);

/** \brief Initialize the Pixhawk HAL
 *
 * Reads the XML file and sets up the Pixhawk settings
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
  printf("Pixhawk: Initializing Serial Pixhawk HAL (TESTING)\n");


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "Pixhawk: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, pixhawkStartTag, pixhawkEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("Pixhawk: Error reading: %s\n",filename);
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
		printf("   Error: No <pixhawk> XML tag found in plugins section\n");
		return -1;
	}

  //Start crossbow thread after init
  if (xmlParse.enable) done = init_pixhawk();



 return done;
}

void XMLCALL
pixhawkStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("pixhawk",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("pixhawk",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   Pixhawk: Pixhawk plugin disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(pixhawkds,attr[i+1],63); 
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate = atoi(attr[i+1]); 
    printf("   Pixhawk: Serial port %s at %d baud\n",pixhawkds,baudrate);
  } 
}

void XMLCALL
pixhawkEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}

