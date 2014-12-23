/** \file powercube.c
 *  \ingroup hwmodule
 *  \brief Hardware abstraction layer for Powercube control using CAN-BUS.
 *
 *  This library inplements the hardware abstraction layer
 *  for communicating using CAN-BUS on the Powercube modules.
 * 
 *  At the present state only a simple homing command is implemeted.
 *  This is only used to test communication of various CAN I/O-boards.
 *
 *  \author Nils A. Andersen & Anders Billesø Beck, DTU 
 *  \editor Soren Hansen, DTU
 *  
 *  $Rev: 59 $
 *  $Date: 2008-09-30 11:58:03 +0200 (Tue, 30 Sep 2008) $
 *  
 */
 /**************************************************************************
 *      Copyright 2008 Nils A. Andersen & Anders Billesø Beck, DTU         *
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
#define POWERCUBEVERSION 		"0.1"
/************************** Version control information ***************************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2008-09-30 11:58:03 +0200 (Tue, 30 Sep 2008) $:"
/**********************************************************************************/

#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
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
#include <net/if.h>
#include <math.h>

#include <linux/can.h>
#include <linux/can/raw.h>

//CAN-Driver definitions, placed in include folder
#include <rhd.h>
#include <database.h>
#include <globalfunc.h>

#include "powercube.h"

/******** Global variables *************/
///Index pointers for the variable database
int ipowercubetest,irot90,igripper,igrippervel,igrippercur,igripperacc,ilin,igs;
int irot70, igripperpos,istatus,ireset;

//PThread definitions
pthread_t canrx_thread, canrx_thread1;
pthread_attr_t attr;

///Number of CAN-messages to be send to CAN-bus in every cycle
#define CANTXBUFSIZE 4
///Number of CAN-messages to be received from the CAN-bus in every cycle
#define CANRXBUFSIZE 3
#define DFLTENGSP 50

#define BLOCK_MAX 200


///CAN port pointer
int can_dev;
///CAN port device identification string
char ifname[64];


 int s;
        struct sockaddr_can addr;
        struct can_filter rfilter[4];
        struct can_frame frame,rxframe;
        int nbytes, i;
        struct ifreq ifr;
        int ifindex;
	
///Flag to indicate if powercube is running or not    

static volatile int powercubeRunning = -1;

//Function prototypes
int initpowercube(void);
void *canrx_task(void *);

/** \brief Initialization of the CAN-bus and all rx/tx buffers
 *
 * All buffers are initilized and database variables are created.
 * Finally, the RX Thread is spawned
 *
 */
int initpowercube(void)
{

// Open CAN port

 if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
                perror("socket");
                return 1;
        }

        rfilter[0].can_id   = 0x123;
        rfilter[0].can_mask = CAN_SFF_MASK;
        rfilter[1].can_id   = 0x0A0;
        rfilter[1].can_mask = 0x0E0;
        rfilter[2].can_id   = 0x80123456;
        rfilter[2].can_mask = 0x1FFFF000;
        rfilter[3].can_id   = 0x80333333;
        rfilter[3].can_mask = CAN_EFF_MASK;

        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  //      if(set_loopback)
  //            setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  //      if(set_recv_own_msgs)
  //              setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

        strcpy(ifr.ifr_name, ifname);
        ioctl(s, SIOCGIFINDEX, &ifr);
        ifindex = ifr.ifr_ifindex;

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                perror("bind");
                return 1;
        }

	
  //Create CAN RX Thread
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  
 #if(1)
  if (pthread_create(&canrx_thread, &attr, canrx_task, 0)) 
  {
      perror("   Powercube: failed");
      return -1;
  }
  #endif

  //Create database variables (if everyting works)
  ipowercubetest = createVariable('w',1,"powercubetest");
  irot90= createVariable('w',1,"rot90");
  igripper= createVariable('w',1,"gripper");
  igrippervel= createVariable('w',1,"grippervel");
  igripperacc= createVariable('w',1,"gripperacc");
  igrippercur= createVariable('w',1,"grippercur");
  ilin= createVariable('w',1,"lin");
  irot70= createVariable('w',1,"rot70");
  igs= createVariable('w',1,"gs");
  ireset= createVariable('w',1,"reset");
  igripperpos= createVariable('r',1,"gripperpos");
  istatus= createVariable('r',1,"gripperstatus");
 
  return 1;
}

/** \brief Entry-point for Can-Bus RX Thread
 *
 * Responsible for reading and parsing all CAN-Bus messages
 *
 */
void *canrx_task(void *not_used){
  int j;

  fprintf(stderr, "   Powercube: Canrx_task running\n");

  //Lock memory - No more allocations
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
    if (sched_setscheduler(0, SCHED_RR, &param)) {
      perror("setscheduler");
      pthread_exit(NULL);
    };

  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, "signal: can't ignore SIGPIPE.\n");

  //Wait to make sure variables are created
  while (powercubeRunning < 0) usleep(100000);

  while (powercubeRunning) {
    int ret;
   	
   

  //receive one CAN message
	ret = read(s, &rxframe, sizeof(struct can_frame));
	if (ret<0) 
	{
		fprintf(stderr,"Error receiving message on CAN-bus\n");
                powercubeRunning = -1; //Shutdown!
	} 
	else 
	{
         //Printout input packages (add additional parsing below)
                //printf("CANRX: ID = 0x%x", rxframe.can_id);
                if ( (rxframe.can_id & 0x1f)==29){ 
		  if (rxframe.data[1]==60)
                    setVariable (igripperpos,0,(*(float*)&rxframe.data[2])*10000);
		  else  if (rxframe.data[1]==39){
                    setVariable (istatus,0,(*(int*)&rxframe.data[2]));
		    printf("\n %x  %x  %x  %x \n",rxframe.data[5],rxframe.data[4],rxframe.data[3],rxframe.data[2]);
		    }
		  else {
		     for(j = 0; j <= rxframe.can_dlc; j++)
                       printf(", 0x%x", rxframe.data[j]);
                     printf("\n");
		    
		    }
		}  
	        else {
	          for(j = 0; j <= rxframe.can_dlc; j++)
                     printf(", 0x%x", rxframe.data[j]);
                   printf("\n");
		}
        }
	
 } //RX Loop ends

  //Finish thread
  fprintf(stderr,"Powercube: Ending RX Thread!\n");
  powercubeRunning = -1;
  pthread_exit(NULL);
}



/** \brief Transmit messages to the CAN bus
 *
 * Periodic function to tramsmit variables and
 * requests to the can-bus
 *
 */
extern int periodic(int tick)
{  
  //Just return if CAN isn't running
  if (powercubeRunning < 0) return -1;

  int ret;
  union { float a;
  	  unsigned char  b[4];
	 }conv;
  
  
  if(isUpdated('w', ipowercubetest))
  {
    // Send homeming commando to module 0x02 and 0x08
    printf("-->  Powercube: Trying to send home\n");
     frame.can_id  = 0x08 + 0xE0;
     frame.can_dlc = 1;
     frame.data[0] = 0x01;
     nbytes = write(s, &frame, sizeof(struct can_frame));
     frame.can_id  = 0x02 + 0xE0;
     frame.can_dlc = 1;
     frame.data[0] = 0x01;
     nbytes = write(s, &frame, sizeof(struct can_frame));
     frame.can_id  = 29 + 0xE0;
     frame.can_dlc = 1;
     frame.data[0] = 0x01;
     nbytes = write(s, &frame, sizeof(struct can_frame));
     frame.can_id  = 9 + 0xE0;
     frame.can_dlc = 1;
     frame.data[0] = 0x00;
     nbytes = write(s, &frame, sizeof(struct can_frame));
     frame.can_id  = 9 + 0xE0;
     frame.can_dlc = 1;
     frame.data[0] = 0x01;
     nbytes = write(s, &frame, sizeof(struct can_frame));
    printf("-->  Powercube: Send done\n");
  }
  
   if(isUpdated('w', ireset)){
     frame.can_id  = 29 + 0xE0;
     frame.can_dlc = 1;
     frame.data[0] = 0x00;
     nbytes = write(s, &frame, sizeof(struct can_frame));
   }
   if(isUpdated('w', irot90))
  {
    
    printf("-->  Powercube: Trying to send position\n");
     conv.a=getWriteVariable(irot90,0)*0.01*M_PI/180.0;
     frame.can_id  = 0x08 + 0xE0;
     frame.can_dlc = 6;
     frame.data[0] = 0x0b;
     frame.data[1] = 0x04;
     frame.data[2] = conv.b[0];
     frame.data[3] = conv.b[1];
     frame.data[4] = conv.b[2];
     frame.data[5] = conv.b[3];
 
     nbytes = write(s, &frame, sizeof(struct can_frame));
   }
   if(isUpdated('w', irot70))
  {
    
    printf("-->  Powercube: Trying to send position\n");
     conv.a=getWriteVariable(irot70,0)*0.01*M_PI/180.0;
     frame.can_id  = 0x02 + 0xE0;
     frame.can_dlc = 6;
     frame.data[0] = 0x0b;
     frame.data[1] = 0x04;
     frame.data[2] = conv.b[0];
     frame.data[3] = conv.b[1];
     frame.data[4] = conv.b[2];
     frame.data[5] = conv.b[3];
 
     nbytes = write(s, &frame, sizeof(struct can_frame));
   }

 
 
  if(isUpdated('w', ilin))
  {
    
    printf("-->  Powercube: Trying to send position\n");
     conv.a=getWriteVariable(ilin,0)*0.0001;
     frame.can_id  = 9 + 0xE0;
     frame.can_dlc = 6;
     frame.data[0] = 0x0b;
     frame.data[1] = 0x04;
     frame.data[2] = conv.b[0];
     frame.data[3] = conv.b[1];
     frame.data[4] = conv.b[2];
     frame.data[5] = conv.b[3];
 
     nbytes = write(s, &frame, sizeof(struct can_frame));
   }
   if(isUpdated('w', igripper))
  {
    
    printf("-->  Powercube: Trying to send position\n");
     conv.a=getWriteVariable(igripper,0)*0.0001;
     frame.can_id  = 29 + 0xE0;
     frame.can_dlc = 6;
     frame.data[0] = 0x0b;
     frame.data[1] = 0x04;
     frame.data[2] = conv.b[0];
     frame.data[3] = conv.b[1];
     frame.data[4] = conv.b[2];
     frame.data[5] = conv.b[3];
 
     nbytes = write(s, &frame, sizeof(struct can_frame));
   }
  
    

  if(isUpdated('w', igrippercur))
  {
    
    printf("-->  Powercube: Trying to send current\n");
     conv.a=getWriteVariable(igrippercur,0)*0.001;
     frame.can_id  = 29 + 0xE0;
     frame.can_dlc = 6;
     frame.data[0] = 0x0b;
     frame.data[1] = 0x08;
     frame.data[2] = conv.b[0];
     frame.data[3] = conv.b[1];
     frame.data[4] = conv.b[2];
     frame.data[5] = conv.b[3];
 
     nbytes = write(s, &frame, sizeof(struct can_frame));
   }
  
      if(isUpdated('w', igrippervel))
  {
    
    printf("-->  Powercube: Trying to send velocity\n");
     conv.a=getWriteVariable(igrippervel,0)*0.0001;
     frame.can_id  = 29 + 0xE0;
     frame.can_dlc = 6;
     frame.data[0] = 0x08;
     frame.data[1] = 0x4f;
     frame.data[2] = conv.b[0];
     frame.data[3] = conv.b[1];
     frame.data[4] = conv.b[2];
     frame.data[5] = conv.b[3];
 
     nbytes = write(s, &frame, sizeof(struct can_frame));
   }
      if(isUpdated('w', igripperacc))
  {
    
    printf("-->  Powercube: Trying to send acceleration\n");
     conv.a=getWriteVariable(igripperacc,0)*0.0001;
     frame.can_id  = 29 + 0xE0;
     frame.can_dlc = 6;
     frame.data[0] = 0x08;
     frame.data[1] = 0x50;
     frame.data[2] = conv.b[0];
     frame.data[3] = conv.b[1];
     frame.data[4] = conv.b[2];
     frame.data[5] = conv.b[3];
 
     nbytes = write(s, &frame, sizeof(struct can_frame));
   }
   #if(1)
     frame.can_id  = 29 + 0xC0;
     frame.can_dlc = 2;
     frame.data[0] = 0x0a;
     frame.data[1] = 60;
     nbytes = write(s, &frame, sizeof(struct can_frame));
     if (isUpdated('w', igs)){
       frame.can_id  = 29 + 0xC0;
       frame.can_dlc = 2;
       frame.data[0] = 0x0a;
       frame.data[1] = 39;
       nbytes = write(s, &frame, sizeof(struct can_frame));
     }
  #endif
  
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
void XMLCALL powercubeStartTag(void *, const char *, const char **);
void XMLCALL powercubeEndTag(void *, const char *);


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
  printf("Powercube: Initializing Powercube CAN-Bus driver %s.%s\n",POWERCUBEVERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "Powercube: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, powercubeStartTag, powercubeEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("Powercube: Error reading: %s\n",filename);
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
    fprintf(stderr, "Powercube: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <powercube> XML tag found in plugins section\n");
		return -1;
	}

  //Start crossbow thread after init
  if (xmlParse.enable) powercubeRunning = initpowercube();


 return powercubeRunning;
}

///Handle XML Start tags
void XMLCALL
powercubeStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("powercube",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("powercube",el)) {
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   Powercube: Use of Powercube disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("controlcan",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) {
       if (strcmp("port",attr[i]) == 0) strncpy(ifname,attr[i+1],63);
    }
    printf("   Powercube: Using CAN-port %s\n",ifname);
  }  

}

///Handle XML End tags
void XMLCALL
powercubeEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
