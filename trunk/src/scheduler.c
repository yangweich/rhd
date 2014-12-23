/** \file sheduler.c
 *  \ingroup core
 *  \brief RT Sheduler
 *
 *  Real Time sheduler for RHD
 *
 *  The sheduler support a range of options for running a periodic task.
 *  Using the normal linux task sheduler, it supports sheduling by
 *  itimer-interrupts and a calculated usleep. Using the basic linux
 *  sheduler, the wait period must be a multiple of the timer frequency
 *  of the linux kernel (default 250Hz = 4 ms)
 *
 *  If complied with RTAI support, the third option is to use the LXRT
 *  userspace RT sheduler. Communication from the LXRT layer and normal
 *  userspace layer is performed by FIFO, ensuring that the main task
 *  does not need to move to the LXRT layer.
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1792 $
 *  $Date: 2012-01-15 08:51:45 +0100 (Sun, 15 Jan 2012) $
 */
 /**************************************************************************
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

/************************** Version control information ***************************/
 #define SHEDULER_VERSION "$Rev: 1792 $:"
 #define DATE             "$Date: 2012-01-15 08:51:45 +0100 (Sun, 15 Jan 2012) $:"
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
#include <sys/time.h>
#include <sys/mman.h>
#include <expat.h>
#include <poll.h>

//Includes for RTAI sheduler
#ifdef RTAI
  #include <rtai_lxrt.h>
  #include <rtai_sem.h>
  #include <rtai_usi.h>
  #include <rtai_fifos.h>
#endif

#include "scheduler.h"

//Global variables
int XMLdepth;
static volatile int timerTick = 0;
static volatile int endTimer = 0;
static volatile int timerPeriod = 0;  ///Timerperiod in us
int     usleepTime;
double  time1, time2;
int     realtimepriority = 0;
struct itimerval timerSetup;
struct timeval timeNow;
enum shedTypes {UNCONFIGURED, USLEEP, ITIMER, LXRT, FREERUNNING, SYNCHRONIZED} currentShed;

//function prototypes
void catchItimer(int signal);

//Function and variable declarations for RTAI
#ifdef RTAI
  int rtaififo;
  int rtThread;
  static RTIME tickTime;
  static void *timer_handler(void *);
#endif

/** \brief Perform the periodic wait
 *
 * \param[in] struct timeval* ticktime;
 * Filename of the XML file
 *
 * \return int status
 * Return success or not
 */
int waitPeriodic(struct timeval* ticktime) {

#ifdef RTAI
  struct pollfd pollData;
  pollData.events = POLLIN;
#endif
  
  //Return imidiately if itimer already has ticked
  if (timerTick) {
    gettimeofday(ticktime,NULL);
    timerTick = 0;
    return 1;
  }

  //Wait periodic for each timer type
  switch (currentShed) {
    case USLEEP : //Using calculated usleep
      gettimeofday(&timeNow,NULL);
      time1 = (double)timeNow.tv_sec + (double)timeNow.tv_usec / 1000000;
      usleepTime = timerPeriod - (int)((time1 - time2) * 1000000);
      if (usleepTime > 0) usleep(usleepTime);
      gettimeofday(&timeNow,NULL);
      time2 = (double)timeNow.tv_sec + (double)timeNow.tv_usec / 1000000;
      break;
    case LXRT :   //Using LXRT RTAI sheduler
#ifdef RTAI
      //Poll and empty fifo-buffer if any data already is there
      //(i.e. one/more timer period(s) missed)
      pollData.fd = rtaififo;
      if (poll(&pollData,1,0)) {
        while(poll(&pollData,1,0)) {
          if (read(rtaififo, &tickTime, sizeof(tickTime)) < 0) {
            shutdownSheduler();
            return -1;
          }
        }
        break; //Return...
      }
      //If no data is ready, wait periodic for it..
      if (read(rtaififo, &tickTime, sizeof(tickTime)) < 0) {
            shutdownSheduler();
            return -1;
      }
      break;
#endif
    case ITIMER : //Using itimer interrupt
      while(!timerTick) pause(); //Sleep until itimer ticks..
      break;
    case FREERUNNING : //Freerunning without scheduler delay
    case SYNCHRONIZED : //Synchronized scheduler gets timing from client
      //Simply do nothing
      break;
    case UNCONFIGURED :
    default: return -1;
  }
    gettimeofday(ticktime,NULL);
    timerTick = 0;
    return 1;


}

/** \brief Switch to hard realtime in scheduler
 *
 * Setup
 *
 * Switches to hard-realtime mode in scheduler. I.e. locks
 * memory and activates RTAI, if it is enabled
 *
 * \param[in] *int signal
 * Filename of the XML file
 *
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
int schedulerRealtime(void) {

  /*** Setup the different timers ***/
  switch (currentShed) {
    case USLEEP : //Using calculated usleep
      //Nothing to do here...
      break;
    case LXRT :   //Using LXRT RTAI sheduler
#ifdef RTAI

    //Open RTAI Fifo for timer communication
    // if ((rtaififo = rtf_open_sized("/dev/rtf0", O_RDWR, 2000)) < 0) {
     if((rtaififo = open("/dev/rtf0", O_RDWR)) < 0) {
      fprintf(stderr,"   Sheduler: ERROR OPENING RTAI FIFO-0. Returned: %d\n", rtaififo);
      return -1;
     }
     //Create RTAI thread
     timerTick = 0;
     printf("Creating timer thread\n");
     rtThread = rt_thread_create(timer_handler, NULL, 10000);
     if (!rtThread) return -1;

      //Check that the thread is hard realtime and running
      struct pollfd pollData;
      pollData.events = POLLIN;
      pollData.fd = rtaififo;
      int pollCounter = 0;
      //Check 10 times for data with timeout of timerperiod (minimim 1 ms)
      while (!poll(&pollData,1,(timerPeriod / 1000)+1)) {
        pollCounter++;
        if (pollCounter > 10) {
          fprintf(stderr,"   Scheduler: ERROR! RTAI Scheduler is NOT running\n");
          shutdownSheduler();
          return -1;
        }
      }
      printf("Reading from fifo\n");
      if (read(rtaififo, &tickTime, sizeof(tickTime)) < 0) {
            fprintf(stderr,"   Scheduler: ERROR reading from rt-fifo0\n");
            shutdownSheduler();
            return -1;
      }
     break;
#endif
    case ITIMER : //Using itimer interrupt
      signal(SIGALRM,catchItimer);
      getitimer(ITIMER_REAL,&timerSetup);
      timerSetup.it_interval.tv_sec   = timerPeriod/1000000; // period
      timerSetup.it_interval.tv_usec  = timerPeriod%1000000;
      timerSetup.it_value.tv_sec      = 1;    //Start in one second
      timerSetup.it_value.tv_usec     = 0;  //Start in one second
      setitimer(ITIMER_REAL,&timerSetup,NULL); //Start timer
      break;

    case FREERUNNING : //Freerunning without scheduler delay
    case SYNCHRONIZED: //Synchronized scheduler gets delay from client
      //Simply do nothing
      break;
    case UNCONFIGURED :
    default: return -1;
  }

  //Lock memory and set thread-priority
  if (realtimepriority) {
  /* use real-time (fixed priority) scheduler
   * set priority to one less than the maximum
   */
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
    if (sched_setscheduler(0, SCHED_RR, &param)){
      perror("setscheduler");
      printf("Scheduler: You must be root-user to run this!\n");
      return -1;
    }

    if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
      fprintf(stderr, "signal: can't ignore SIGPIPE.\n");

    //Goto soft-realtime mode (lock-memory)
    if (mlockall(MCL_CURRENT | MCL_FUTURE)) {
        perror("mlockall");
        printf("Scheduler: You must be root-user to run this!\n");
        return -1;
    }
    printf("   Scheduler: Running in realtime priority with memory locked\n");

  } else {
    printf("   Scheduler: Running in user priority without memory locked\n");
  }

  return 1;

}

/** \brief Itimer interrupt handler
 *
 * Catches the itimer signal
 *
 * \param[in] *int signal
 * Filename of the XML file
 *
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
void catchItimer(int signal) {

    timerTick = 1;
}

/** \brief RTAI Sheduler thread
 *
 * RT Thread for RTAI Sheduling
 *
 *
 * \returns int status
 * Status thread - negative on error
 */
#ifdef RTAI
static void *timer_handler(void *timerPer) {
  RT_TASK *handler;
  static RTIME unixtime;
  int rtfifo;
  RTIME period;
  rt_allow_nonroot_hrt();
  if (!(handler = rt_task_init_schmod(nam2num("TIMER"),
        0, 0, 0, SCHED_FIFO, 0xF))) {
    printf("   Sheduler: CANNOT INIT TIMER TASK > TIMER <\n");
    exit(1);
  }

  //Create FIFO0, 2000 byte
  rtf_create(0, 2000);
  rtfifo = 0;

  //Setup RT Timer
  //rt_set_oneshot_mode();
  rt_set_periodic_mode();
  start_rt_timer(nano2count(100000)); //RT Timer runs at 0.1 ms
  period = nano2count(timerPeriod * 1000);//(timerPeriod * 1000); //Setup period and convert to ns-> ticks

  //Goto linux soft-realtime
  if (mlockall(MCL_CURRENT | MCL_FUTURE)) {
      perror("mlockall");
      exit(-1);
  }

  printf("   Sheduler: LXRT Sheduler going to HARD real-time\n");
  rt_make_hard_real_time();
  timerTick = 1;
  rt_task_make_periodic(handler, rt_get_time() + period, period);
  rt_task_wait_period();

  //Timer loop:
  while (!endTimer) {
      unixtime = rt_get_time_ns();
      rtf_put(rtfifo, &unixtime, sizeof(unixtime) );
      rt_task_wait_period();
  }

  rt_make_soft_real_time();
  rt_task_delete(handler);
  rtf_destroy(rtfifo);
  printf("   Sheduler: RT Thread ended\n");
  endTimer = 0; //Set flag that timer is ended
  return 0;
}
#endif

/** \brief Shutdown sheduler
 *
 * \returns int status
 * Status thread - negative on error
 */
int shutdownSheduler(void) {

  printf("   Sheduler: Shutting down timers\n");
  endTimer = 1; //Set endflag

  switch(currentShed) {
  case LXRT :
#ifdef RTAI
  //Wait for RTAI thread to shut down
  while(endTimer) usleep(10000);//Wait for RT thread to terminate
  break;
#endif
  case ITIMER :
    setitimer(ITIMER,0,0); //Disable Itimer
    break;
  default: break;
  }

  return 1;
}

/** \brief Get period
 *
 * \returns int period
 * Sheduler period in us
 */
int getSchedulerPeriod(void) {
  return timerPeriod;
}

/** \brief Set the scheduler period
 *
 * \param int period
 * The new desired scheduler period in us
 */
int setSchedulerPeriod(int period) {

	//Set the timing period to new interval
	timerPeriod = period;

	//Specific timer setups for real-time interrups
	switch(currentShed) {
     case LXRT :
		  printf("   Scheduler: Adjustment of timer interval for RTAI is not yet implemented!\n");
		  return -1;
     case ITIMER :
		  getitimer(ITIMER_REAL,&timerSetup);
		  timerSetup.it_interval.tv_sec   = timerPeriod/1000000; // period
		  timerSetup.it_interval.tv_usec  = timerPeriod%1000000;
		  timerSetup.it_value.tv_sec      = 1;    //Start in one second
		  timerSetup.it_value.tv_usec     = 0;  //Start in one second
		  setitimer(ITIMER_REAL,&timerSetup,NULL); //Start timer
		  break;

	  default: break;
  }

	return 1;
}

/************************** XML Initialization **************************/
///Struct for shared parse data
typedef struct  {
    int depth;
    char skip;
    char enable;
  }parseInfo;

//Parsing functions
void XMLCALL shedulerStartTag(void *, const char *, const char **);
void XMLCALL shedulerEndTag(void *, const char *);

//XML File buffer size
#define XMLBUFLEN 8192
/** \brief Initialize the Sheduler
 *
 * Reads the XML file and sets up the sheduler settings
 *
 * If RTAI is selected, the sheduler-thread is started.
 *
 * \param[in] *char filename
 * Filename of the XML file
 *
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
int shedulerInitXML(char *filename) {

  parseInfo xmlParse;
  char xmlBuf[XMLBUFLEN];
  int done = 0;
  int len;
  FILE *fp;

  //Print initialization message
  //Find revision number from SVN Revision
  char *i,versionString[20] = SHEDULER_VERSION, tempString[10];
  i = strrchr (versionString,'$');
  strncpy(tempString,versionString+1,(i-versionString-1));
  tempString[(i-versionString-1)] = 0;
  printf("Sheduler: Initializing RHD RT Scheduler\n");

   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "Sheduler: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   XML_SetElementHandler(parser, shedulerStartTag, shedulerEndTag);
   //Setup shared data
   xmlParse.depth = 0;
   xmlParse.skip =  0;
   xmlParse.enable = 0;
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("Sheduler: Error reading: %s\n",filename);
    return -1;
  }
  len = fread(xmlBuf, 1, XMLBUFLEN, fp);
  fclose(fp);

  //Start parsing the XML file
  if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) {
    fprintf(stderr, "Sheduler: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);

 return 1;
}

void XMLCALL
shedulerStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1. and 2. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) &&
         ((strcmp("sheduler",el) != 0) && (strcmp("scheduler",el) != 0)))) {//Fix for previous typo..
      info->skip = info->depth;
      return;
    }
  } else return;

  //Branch to parse the elements of the XML file.
  if ((strcmp("sheduler",el) == 0) || (strcmp("scheduler",el) == 0)) {
    //Check for the correct depth for this tag
    if(info->depth != 2) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
  } else if (strcmp("type",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 3) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
#ifdef RTAI
    printf("   Scheduler: Compiled with RTAI Support\n");
#else
    printf("   Scheduler: Compiled without RTAI Support\n");
#endif
    for(i = 0; attr[i]; i+=2) if (strcmp("value",attr[i]) == 0) {
      if (strcmp("usleep",attr[i+1]) == 0) {
        currentShed = USLEEP;
        printf("   Scheduler: Using USLEEP scheduler\n");
      } else if (strcmp("itimer",attr[i+1]) == 0) {
        currentShed = ITIMER;
        printf("   Scheduler: Using ITIMER scheduler\n");
      } else if ((strcmp("LXRT",attr[i+1]) == 0) || (strcmp("RTAI",attr[i+1]) == 0)) {
#ifdef RTAI
        printf("   Scheduler: Using LXRT RTAI scheduler\n");
        currentShed = LXRT;
#else
        currentShed = ITIMER;
        printf("   Scheduler: NOTE! Compiled without RTAI Support\n");
        printf("   Scheduler: Using ITIMER instead of RTAI\n");
#endif
      } else if (strcmp("freerunning",attr[i+1]) == 0) {
        currentShed = FREERUNNING;
        printf("   Sheduler: Using freerunning mode!\n");
	printf("   Scheduler: WARNING! Are you really sure you want to run without scheduler?!\n");
      } else if (strcmp("synchronized",attr[i+1]) == 0) {
        currentShed = SYNCHRONIZED;
        printf("   Scheduler: Using scheduling fixed to master client!\n");
		  printf("   Scheduler: WARNING! Are you really sure you want to run without RHD scheduler?!\n");
      }else {
        currentShed = UNCONFIGURED;
        printf("   Scheduler: No type detected - Running unconfigured\n");
      }
    }
  } else if (strcmp("period",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 3) {
      printf("Error: Wrong depth for the %s tag %d\n",el, XMLdepth);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("value",attr[i]) == 0) timerPeriod = atoi(attr[i+1]);
    printf("   Scheduler: Timer period set to %2d.%03d ms\n",timerPeriod/1000, timerPeriod%1000);
    if (timerPeriod < 1000) printf("   Scheduler: WARNING! Using a period less than 1ms is not recomended!\n");
  } else if (strcmp("realtimepriority",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 3) {
      printf("Error: Wrong depth for the %s tag %d\n",el, XMLdepth);
    }
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      realtimepriority = 1;
    }

    if (realtimepriority) printf("   Scheduler: Running in realtime priority\n");
    else printf("   Scheduler: Running in user-priority mode (only recomended for simulations)\n");
  }
}

void XMLCALL
shedulerEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}

