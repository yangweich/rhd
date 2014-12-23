 /** \file rhdlog.c
 *  \ingroup hwmodule
 *
 * RHD plug-in to log all variable to a file with Linux time stamp
 * log interval can be set in rhdconfig.xml file (multiple of RHD sample time)
 * Log period can be limited by write-to-disk time.
 * Write to disk is performed in separate thread.
 * 
 *******************************************************************/
 
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 505 $:"
 #define ID               "$Id: rhdlog.c 505 2014-07-09 16:10:05Z immunt $"
 #define PLUGINNAME        "RHDlog"
/***************************************************************************/

#include <sched.h> 
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <linux/serial.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <expat.h>
#include <stdint.h>

//RHD Core headers
#include <rhd.h>
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "rhdlog.h"


///Struct for shared parse data
typedef struct  {
    int depth; // current XML tag level
    int skip;  // skip from this level up
    char enable;
    char found;
  } parseInfo;

/* prototypes */
int init(void);
/**
 * get index to a named variable - from another plugin
 * \param type is either 'r' or 'w'
 * \param name is the name of the variable
 * \returns -1 if the variable is not found, else the index (positive or zero) of the variable. */
int getDatabaseVariable(char type, const char * name);
/// Parsing function for XML start tag
void XMLCALL lsStartTag(void *, const char *, const char **);
/// parsing function for XML end tag
void XMLCALL lsEndTag(void *, const char *);
/// round a float to an integer

/******** Global variables *************/


void * log_task(void * not_used);
int tick = 0;
int debugFlag = 0;
/** flags for updating of write variables */
int settingUpd = 0;
struct timeval tickTime;
int loginterval = 1; // log at every tick by default
int varLoginterval_w;
int varLoginterval_r;
FILE * logfd = NULL;
pthread_t rhdlog_thread;
int running = 0;
int fileInitialized = 0;
static char rhdLogName[128] = "rhdlog.txt";
char rhdlogPathBase[128] = "";
char rhdLogPathFull[128] = "rhdlog.txt";

////////////////////////////////////////////////////////

/**
 * This is called in RHD main thread at every sample period
 * */
extern int periodic(int rhdTick)
{
  int returnValue = running;
  gettimeofday(&tickTime, NULL);
  tick = rhdTick;
  if (loginterval > 0)
  {
    if ((tick % loginterval) == 0)
      settingUpd = 1;
  }
  if (isUpdated('w', varLoginterval_w))
    loginterval = getWriteVariable(varLoginterval_w, 0);
  return returnValue;
}

//////////////////////////////////////////////////////
/**
 * Create RHD variable */

void createVariables()
{ // create variables in RHD
  varLoginterval_r = createVariable('r',1,"rhdlogging"); // actual log interval
  varLoginterval_w = createVariable('w',1,"rhdloginterval"); // requested log interval
}

////////////////////////////////////////////////////////////////

int terminate(void)
{
  running = 0;
  printf(PLUGINNAME ": stopping ... ");
  // stop motors - in either mode
  pthread_join(rhdlog_thread, NULL);
  if (logfd != NULL)
    fclose(logfd);
  printf("[OK]\n");
  return 0;
}

/************************** XML Initialization **************************/


/** \brief Initialize from XML file
 *
 * Reads the XML file and initializes plugin
 * after successfully reading the XML file.
 * 
 * \param[in] *char filename
 * Filename of the XML file
 * 
 * \returns int status 1 is OK and -1 is error
 * Status of the initialization process. Negative on error.
 */
extern int initXML(char *filename)
{
  int result;
  parseInfo xmlParse; 
  char *xmlBuf = NULL;
  int xmlFilelength;
  int done = 0;
  int len;
  FILE *fp;
  //Print initialization message
  //Find revision number from SVN Revision
  char * p1;
  char versionString[20] = REVISION;
  char tempString[10];
  p1 = strrchr(versionString, '$');
  strncpy(tempString, &versionString[6],(p1 - versionString - 6));
  tempString[(p1 - versionString - 6)] = '\0';
  printf(PLUGINNAME ": Initializing plug-in version %s\n", tempString);

  /* Initialize Expat parser*/
  XML_Parser parser = XML_ParserCreate(NULL);
  result = parser != 0;
  if (!result)
    fprintf(stderr, PLUGINNAME ": Couldn't allocate memory for XML parser\n");
  if (result)
  {  //Setup element handlers
    XML_SetElementHandler(parser, lsStartTag, lsEndTag);
    //Setup shared data
    memset(&xmlParse, 0, sizeof(parseInfo));
    //
    XML_SetUserData(parser, &xmlParse);

    //Open and read the XML file
    fp = fopen(filename,"r");
    result = fp != NULL;
    if(!result)
      printf(PLUGINNAME ": Error reading: %s\n",filename);
  }
  if (result)
  { //Get the length of the file
    fseek(fp,0,SEEK_END);
    xmlFilelength = ftell(fp); //Get position
    fseek(fp,0,SEEK_SET); //Return to start of file
    //Allocate text buffer for full file length
    xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
    result = (xmlBuf != NULL);
    if (!result)
        fprintf(stderr, PLUGINNAME ": Couldn't allocate memory for XML File buffer\n");
  }
  if (result)
  { // clear buffer
    memset(xmlBuf,0,xmlFilelength);
    // read full file
    len = fread(xmlBuf, 1, xmlFilelength, fp);
    fclose(fp);
    //Start parsing the XML file
    result = (XML_Parse(parser, xmlBuf, len, done) != XML_STATUS_ERROR);
    if (!result)
      fprintf(stderr, PLUGINNAME ": XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
  }
  if (parser != NULL)
    XML_ParserFree(parser);
  if (xmlBuf != NULL)
    free(xmlBuf);
  if (result && xmlParse.enable)
  { // all is fine - start plugin
    result = init();
  }
  if (result)
    return 1;
  else
    return -1;
}

//////////////////////////////////////////////////

/**
 * A start tag is detected by the XML parser
 \param data is a user defined context pointer.
 \param el is the tag name
 \param attr is the list of attributes in the start tag. */
void XMLCALL lsStartTag(void *data, const char *el, const char **attr)
{ // a start tag is detected
  int i;
  parseInfo *info = (parseInfo *) data;
  // detect context
  info->depth++;
  if (info->depth < info->skip || info->skip == 0)
  {
    switch (info->depth)
    {
      case 1:
        if (strcmp("rhd",el) == 0)
          ;
        else
          info->skip = info->depth;
        break;
      case 2:
        if (strcmp("plugins",el) == 0)
          ; // no attributes here - but at next level
        else
          // skip this group
          info->skip = info->depth;
        break;
      case 3:
        // this one handles interface to SaMe (Sabertooth and magnetic encoder)
        // through arduino interface - the only option is debug
        if (strcmp("rhdlog",el) == 0)
        { // get enable bit and device name
          const char * att;
          const char * val;
          for(i = 0; attr[i]; i+=2)
          {
            att = attr[i]; 
            val = attr[i + 1];
            if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
              info->enable = 1;
            else if (strcmp("logpath",att) == 0)
            {
              strcpy(rhdlogPathBase, val);
              if(strlen(rhdlogPathBase)>0) // Ensure that full path ends with '/'
              {
                if(rhdlogPathBase[strlen(rhdlogPathBase)-1] != '/') strcat(&rhdlogPathBase[strlen(rhdlogPathBase)-1], "/");
              }
              snprintf(rhdLogPathFull, 128, "%s%s", rhdlogPathBase, rhdLogName);
              printf(PLUGINNAME ": Path set to '%s'\r\n", rhdLogPathFull);
            }
            else if (strcmp("debug", att) == 0)
            {
              debugFlag = strtol(val, NULL, 0);
              if (debugFlag)
                printf(PLUGINNAME ": started in DEBUG mode!\n");
            }
            else if (strcasecmp("interval",att) == 0)
              // velocity offset in m/s to get zero velocity
              loginterval = strtol(val, NULL, 0);
          }
          if (!info->enable)
            printf(PLUGINNAME ": Use is disabled in configuration\n");
        }
        else
          info->skip = info->depth;
        break;
      default: // unknown tag series
        break;
    }
  }
}

///////////////////////////////////////////////////////

void XMLCALL lsEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
//  printf("endtag %s changed depth from %d", el, info->depth);
  info->depth--;
  if (info->depth < info->skip)
    // back to normal search for valid tags
    info->skip = 0;
//  printf(" to %d\n", info->depth);
}

////////////////////////////////////////////////////

int openAndSetMode()
{
  FILE *oldfd;
  char logTime[64] = {0};
  char str[256] = {0};  
  static struct timeval tv;
  time_t rawtime;
  struct tm *timeinfo;
  
  gettimeofday(&tv, 0);
  rawtime = tv.tv_sec;
  timeinfo = localtime ((time_t*)&rawtime);      
  
  oldfd = fopen(rhdLogPathFull, "r");  
  if(oldfd != NULL)
  {
    fclose(oldfd);
    strftime(logTime,64,"%G%m%d_%H%M%S.txt",timeinfo);
    snprintf(str, 128, "%srhdlog%s", rhdlogPathBase, logTime);
    rename( rhdLogPathFull , str );
    printf(PLUGINNAME ": Started log '%s', old log moved to '%s'\r\n", rhdLogPathFull, str);
  }
  else
  {
    printf(PLUGINNAME ": Started log '%s'\r\n", rhdLogPathFull);
  }
  
  logfd = fopen(rhdLogPathFull, "w");
  if(logfd == NULL) printf(PLUGINNAME ": Could not open log '%s'\r\n", rhdLogPathFull);
  
  fileInitialized = 0;
  return (logfd != NULL) || debugFlag;
}


//////////////////////////////////////////////

/**
 * Initialize the communication and start rx thread
 * \returns 1 on success (else 0) */
int init(void)
{ //Open first serial port
  int result = 1;
  running = 0;
  logfd = NULL;
  { // start thread to handle bus
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rhdlog_thread, &attr, log_task, 0))
    {
      perror(PLUGINNAME ": Can't start log thread");
      result = 0;
    }
  }
  if (result == 1 || debugFlag)
  { /****** Create database variables if all is ok **************/
    int waitCount = 0;
    // create RHD database variables
    createVariables();
    while (!running)
    { // wait a bit for thread to start
      usleep(20000); //Don't return before threads are running
      if (waitCount >= 50)
      {
        result = 0;
        break;
      }
      waitCount++;
    }
  }
  return result;
}

/**
 * Make this time into string */
char * getDateTimeAsString(struct timeval * time, char * info)
{
  struct tm ymd;
  //
  localtime_r(&time->tv_sec, &ymd);
  //
  sprintf(info, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
          ymd.tm_year+1900, ymd.tm_mon+1, ymd.tm_mday,
          ymd.tm_hour,
          ymd.tm_min, ymd.tm_sec, (int)time->tv_usec / 1000);
  return info;
}


///log thread
void * log_task(void * not_used)
{ // run in thread
  const int MSL = 50;
  char s[MSL];
  //
  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, PLUGINNAME ": signal: can't ignore SIGPIPE.\n");

  running = 1;
  fprintf(stderr, PLUGINNAME ": log_task running\n");
  // set initial values of variables
  //Mark thread as running
  while (running)
  { // wait for update
    if (logfd == NULL && loginterval > 0)
      openAndSetMode();
    else if (logfd != NULL && loginterval <= 0)
    {
      fclose(logfd);
      logfd = NULL;
    }
    if (logfd != NULL)
    { // may be time to log
      setVariable(varLoginterval_r, 0, loginterval);
      if (settingUpd == 0)
        // sleep a short while (2ms)
        usleep(2000);
      else
      {
        int i,j;
        symTableElement * syms;
        int symsCnt;
        char c = 'r';
        if (fileInitialized == 0)
        { // log headlines
          int n = 1;
          fprintf(logfd, "%lu.%06lu %s\n", tickTime.tv_sec, tickTime.tv_usec, getDateTimeAsString(&tickTime, s));
          fprintf(logfd, "# symbols: index r/w length name\n%d t 1 timestamp\n", n++);
          for (j = 0; j < 2; j++)
          { // write symbol list - read then write
            syms = getSymbolTable(c);
            symsCnt = getSymtableSize(c);
            for (i = 0; i < symsCnt; i++)
            {
              fprintf(logfd, "%d %c %d %s\n", n, c, syms->length, syms->name);
              n += syms->length;
              syms++;
            }
            c = 'w'; 
          }
          fileInitialized = 1;
        }
        // log values
        fprintf(logfd, "%lu.%03lu ", tickTime.tv_sec, tickTime.tv_usec/1000);
        c = 'r';
        int k;
        for (j = 0; j < 2; j++)
        { // write symbol list - read then write
          syms = getSymbolTable(c);
          symsCnt = getSymtableSize(c);
          for (i = 0; i < symsCnt; i++)
          {
            for (k = 0; k < syms->length; k++)
              fprintf(logfd, "%d ", syms->data[k]);
            syms++;
          }
          c = 'w';
        }
        fprintf(logfd, "\n");
        fflush(logfd);
        // ready for next update
        settingUpd = 0;
      }
    }
    else
    { // nothing to do
      setVariable(varLoginterval_r, 0, 0);
      sleep(1);
    }
  }
  running = 0;
  fprintf(stderr,PLUGINNAME ": Shutting down thread\n");
  pthread_exit(0);
  return NULL;
}



