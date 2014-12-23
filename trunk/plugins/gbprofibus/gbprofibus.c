 /** \file gbprofibus.c 
 *  \ingroup hwmodule
 *
 *   Interface for Guidebot Profibus Interface
 *
 * This plugin is based on the GBD (GuideBot Deamon), and provides
 * interface for the profibus controlled devices: (Motors, Encoders)
 *
 *
 *
 ***************************************************************************/
/***************************** Plugin version  *****************************/
#define GBPROFIBUSVERSION    "1.1"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 264 $:"
 #define DATE             "$Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $:"
 #define ID               "$Id: gbprofibus.c 264 2013-10-14 14:16:15Z sh $"
/***************************************************************************/

#include <sched.h>
#include <pthread.h>
#include <semaphore.h>
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
#include <database.h>
#include <globalfunc.h>
#include <scheduler.h>

#include "gbprofibus.h"
#include "profibus.h"


/******** Global variables *************/
int iEncl, iEncr, iSpl, iSpr, iJoy, iNoEnc, iDig, iFault, iBatt, iPeriod,iDigOut;
int gbRunning = -1;
volatile double motorl, motorr;
int    remoteHyst = 0, ioDelay = 20000, timeout = 0;
int	 reqDelayPeriods, txDelayPeriods;
sem_t  timingSem;
enum   {PROFIBUS_REQUEST, PROFIBUS_TRANSMIT} commState = PROFIBUS_REQUEST;
char	 fwFilename[256], ldFilename[256];

//SMRD Functions
void *gb_task(void *);
pthread_t gb_thread;
pthread_attr_t attr;

/// Init Guidebot profibus driver
int initGbProfibus(void) {

	//Start and initialize profibus
  if (profibus_start() <= 0) {
	  printf("GBProfibus: Initialization of profibus failed!\n");
	  return -1;
  }
  profibus_motors_param();
  motorr = 0.0;
  motorl = 0.0;
  printf("   GBProfibus: Profibus started and initialized\n");

  //Initialize timing semaphore
  sem_init(&timingSem,0,0); //Initialize to block (0)

  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  gbRunning = 2;
  if (pthread_create(&gb_thread, &attr, gb_task, 0)) {
      perror("   GBProfibus: Can't start Profibus thread");
      return -1;
  }

  /****** Create database variables if all is ok **************/
  iEncl    = createVariable('r',1,"encl");
  iEncr    = createVariable('r',1,"encr");
  iJoy     = createVariable('r',3,"joystick");
  iNoEnc   = createVariable('r',2,"noenc");
  iDig     = createVariable('r',6,"digital");
  iBatt    = createVariable('r',1,"batt");
  iFault   = createVariable('r',4,"fault");
  iPeriod  = createVariable('r',1,"period");
  iSpl     = createVariable('w',1,"speedl");
  iSpr     = createVariable('w',1,"speedr");
  iDigOut  = createVariable('w',1,"digitalout");
 
  

	int waitCount = 0;
	while (gbRunning > 1) {
		 usleep(100000); //Don't return before threads are running
		if (waitCount >= 50) return -1;
		waitCount++;
	}

  return 1;
}



///Guidebot control main thread
void *gb_task(void *not_used)
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

  printf("   GBProfibus: Profibus rx_task running\n");

	//Mark thread as running
	gbRunning = 1;

	//Initialize profibus interface

	/* Get initial encoder values */
	profibus_sensors_command(); /* seems to take 120ms */
	usleep(50000); /* 50ms */
	profibus_sensors_read();
	profibus_out.lspeed = 0.0;
	profibus_out.rspeed = 0.0;

	struct timeval tod;
	long int time1, time2;

	//Communications loop (runs as fast as possible)
	while (gbRunning) {

		commState = PROFIBUS_REQUEST;

		gettimeofday(&tod,NULL);
		time1 = tod.tv_sec * 1000000 + tod.tv_usec;

		//Start by executing sensors command (read from sensors)
	   profibus_sensors_command(); /* seems to take some time (10ms?) */

		//Wait until timing semaphore is posted

		sem_wait(&timingSem);

		//Switch to transmit state
		commState = PROFIBUS_TRANSMIT;

		//Parse the profibus input
		profibus_sensors_read(); /* seems to take < 1ms */

		//Update database variables with profibus values
		setVariable(iEncl, 0, (int32_t)profibus_in.lenc);
		setVariable(iEncr, 0, (int32_t)profibus_in.renc);
		setVariable(iJoy,0,(int32_t)profibus_in.Joystick);
		setVariable(iJoy,1,(int32_t)profibus_in.jdiff);
		setVariable(iJoy,2,(int32_t)profibus_in.jsp);
		setVariable(iNoEnc,0,(int32_t)profibus_in.NoLeftEnc);
		setVariable(iNoEnc,1,(int32_t)profibus_in.NoRightEnc);
		setVariable(iDig,0,(int32_t)profibus_in.digi_in[0]);
		setVariable(iDig,1,(int32_t)profibus_in.digi_in[1]);
		setVariable(iDig,2,(int32_t)profibus_in.digi_in[2]);
		setVariable(iDig,3,(int32_t)profibus_in.digi_in[3]);
		setVariable(iDig,4,(int32_t)profibus_in.digi_in[4]);
		setVariable(iDig,5,(int32_t)profibus_in.digi_in[5]);
		setVariable(iBatt,0,(int32_t)(profibus_in.batt * 1000));
		setVariable(iFault,0,(int32_t)profibus_in.lfault);
		setVariable(iFault,1,(int32_t)profibus_in.lfaultsamp);
		setVariable(iFault,2,(int32_t)profibus_in.rfault);
		setVariable(iFault,3,(int32_t)profibus_in.rfaultsamp);


		//Guidebot control
		//Operate on joystick if operation switch is set to manual
		
		/*
		if (profibus_in.Joystick) {
			if ((profibus_in.jsp > remoteHyst) || (profibus_in.jsp < -remoteHyst) ||
				 (profibus_in.jdiff > remoteHyst) || (profibus_in.jdiff < -remoteHyst)) {
					profibus_out.lspeed=((float)profibus_in.jsp-(float)profibus_in.jdiff)*0.3/200;
					profibus_out.rspeed=((float)profibus_in.jsp+(float)profibus_in.jdiff)*0.3/200;

			} else {
				profibus_out.lspeed=0;
				profibus_out.rspeed=0;
			}

		//Otherwise run on computer control
		} else if (isUpdated('w',iSpl) || isUpdated('w',iSpr))*/ {

		//	motorl =     getWriteVariable(iSpl,0) * 0.5 / 128;
		//	motorr =    (getWriteVariable(iSpr,0) * 0.5 / 128);
		//	motorl =    getWriteVariable(iSpl,0) * 0.0001;
		//	motorr =    getWriteVariable(iSpr,0) * 0.0001;
			/*
			symTableElement *st = getSymtable('w');
			
			printf("  !!!!  SIZE: %d\n", sizeof(symTableElement));
			
			printf("gbprofibus: pointer = %x\n", st->data);
			
  			for(i = 0; i < sizeof(symTableElement); i++)
    				printf("%x ", ((unsigned char *) &st)[0]);
  			printf(" \n");
			
		        printf("gbprofibus:  motorl = %lf, motorr = %lf\n", motorl, motorr);
		        */
			profibus_out.lspeed = motorl;
			profibus_out.rspeed = motorr;

			/*Reset status diodes*/
			profibus_out.signal_state = getWriteVariable(iDigOut,0);
		}

		/*Determine whether to update motor speeds or do fault handling*/
		if (profibus_in.lfault || profibus_in.rfault){
			posmo_fault_ack();
		}  else {
			profibus_motors_update();  /* update motor speeds */
		} /* seems to take 60 ms */

		//Wait for timing semaphore to allow data to be transmitted to profibus
		sem_wait(&timingSem);

		//save the time
		gettimeofday(&tod,NULL);
		time2 = tod.tv_sec * 1000000 + tod.tv_usec;
		setVariable(iPeriod,0,(int32_t)(time2-time1));
	}

   fprintf(stderr,"GBProfibus: Termiating profibus rx-thread\n");

	return NULL;
}

///Periodic update function, called by RHD Core
int periodic(int tick) {

	static int periodCounter = 0;

	//Increment period counter
	periodCounter++;

	motorl = getWriteVariable(iSpl, 0) * 0.0001;
	motorr = getWriteVariable(iSpr, 0) * 0.0001;
	
	//Handle the semaphore to control
	if (((commState == PROFIBUS_REQUEST) &&  (periodCounter >= reqDelayPeriods)) ||
		 ((commState == PROFIBUS_TRANSMIT) && (periodCounter >= txDelayPeriods))) {

		periodCounter = 0; //Reset period counter

		//Post to semaphore until it is >= 0  (just to be sure)
		sem_post(&timingSem); //Increment semaphore

	} else commState = PROFIBUS_REQUEST;

	return gbRunning;
}

///Shutdown function
int terminate (void) {

	//Trigger end-of-thread
	gbRunning = 0;

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
void XMLCALL gbStartTag(void *, const char *, const char **);
void XMLCALL gbEndTag(void *, const char *);

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
  printf("GBProfibus: Initializing Guidebot Profibus HAL %s.%s\n",GBPROFIBUSVERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "GBProfibus: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, gbStartTag, gbEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("GBProfibus: Error reading: %s\n",filename);
    return -1;
  }
  //Get the length of the file
	fseek(fp,0,SEEK_END);
	xmlFilelength = ftell(fp); //Get position
	fseek(fp,0,SEEK_SET); //Return to start of file

	//Allocate text buffer
	xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
	if (xmlBuf == NULL) {
		fprintf(stderr, "   GBProfibus: Couldn't allocate memory for XML File buffer\n");
		return -1;
	}
	memset(xmlBuf,0,xmlFilelength);
  len = fread(xmlBuf, 1, xmlFilelength, fp);
  fclose(fp);

  //Start parsing the XML file
  if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) {
    fprintf(stderr, "GBProfibus: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <gbprofibus> XML tag found in plugins section\n");
		return -1;
	}

  //Start function thread after init
  if (xmlParse.enable) done = initGbProfibus();



 return done;
}

void XMLCALL
gbStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("gbprofibus",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("gbprofibus",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1;
    }
    if (!info->enable) {
      printf("   GBProfibus: Use of GuidebotProfibus disabled in configuration\n");
      info->skip = info->depth;
    }
  } else if (strcmp("remotecontrol",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("hysteresis",attr[i]) == 0) remoteHyst = atoi(attr[i+1]);
    printf("   GBProfibus: Using remote control hysteris of %d \n",remoteHyst);

  }  else if (strcmp("timing",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("readperiods",attr[i]) == 0) reqDelayPeriods = atoi(attr[i+1]);
    printf("   GBProfibus: Request time of %d periods (%2.3f ms)\n",
		reqDelayPeriods,((double)(getSchedulerPeriod()*reqDelayPeriods)/1000.0));
	 for(i = 0; attr[i]; i+=2) if (strcmp("writeperiods",attr[i]) == 0) txDelayPeriods = atoi(attr[i+1]);
    printf("   GBProfibus: Transmit time of %d periods (%2.3f ms)\n",
		txDelayPeriods,((double)(getSchedulerPeriod()*txDelayPeriods)/1000.0));
  
  } else if (strcmp("firmware",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("filename",attr[i]) == 0) strncpy(fwFilename,attr[i+1],254);
    printf("   GBProfibus: Firmware file: %s \n",fwFilename);

  } else if (strcmp("configuration",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("filename",attr[i]) == 0) strncpy(ldFilename,attr[i+1],254);
    printf("   GBProfibus: Configuration file: %s \n",ldFilename);

  }

}

void XMLCALL
gbEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}


