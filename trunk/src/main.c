/** \file main.c
 *  \ingroup core 
 *  \brief Robot Hardware Daemon
 *
 * Main program for Robot Hardware Daemon
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1970 $
 *  $Date: 2012-08-07 15:11:42 +0200 (Tue, 07 Aug 2012) $
 *  
 */ 
 /**************************************************************************
 *                  Copyright 2008 Anders Billesø Beck DTU                 *
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
/***********************       RHD Version           ***********************/
#define  RHDVERSION  "2.4" 

/*********************** Version control information ***********************/
 #define REVISION "$Rev: 1970 $:"
 #define DATE     "$Date: 2012-08-07 15:11:42 +0200 (Tue, 07 Aug 2012) $:"
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
#include <dlfcn.h>


#include <rhd.h>
#include <smr.h>

#include "scheduler.h"
#include "database.h"
#include "server.h"
#include "globalfunc.h"

///Default configuration file
#define DEFAULTCONFIG "rhdconfig.xml"

typedef struct  {
	char name[128];		  //Name of plugin
	void *libHandle;	  //Handle for the lib
	int  (*initXML)(char *);  //Function pointer for init fuction
	int  (*periodic)(int);    //Function pointer for periodic function
	int  (*terminate)(void);  //Function pointer for shutdown function
	char enable;              //Enable flag for plugin
	char critical;		  //Critical flag for plugin
        int  safety;               //Safety priority (and flag) for plugin
} pluginFunc;

void print_sched(void);
void endRhd(int);
int rhdInitXML(char *);

volatile int rhdRunning = 1; //Termination flag

//Plugin function structure
pluginFunc *plugin 	= NULL;
int numPlugins		= 0;
int maxSafety           = -1;

/*********** RHD Main function ****************/
int main(int argc, char * argv[]) {

  //Find revision number from SVN Revision
  char *cPtr,versionString[20] = REVISION, tempString[10];
  cPtr = strrchr(versionString,'$');
  strncpy(tempString,versionString+6,(cPtr-versionString-6));
  tempString[(cPtr-versionString-6)] = 0;
  printf("**********************************************************************\n");
  printf("***************     Robot Hardware Daemon %s.%s    ***************\n",RHDVERSION,tempString);
  printf("**********************************************************************\n\n");

  struct timeval tod;
  int iTick, tick = 0;
  int i,j;

  //Catch interrupts to terminate properly
    signal(SIGTERM, endRhd);
    signal(SIGINT, endRhd);
    signal(SIGKILL, endRhd);

  //Get default configuration from arg
  char configFileString[64];
  if (argc >= 2) strncpy(configFileString,argv[1],63);
  else strncpy(configFileString,DEFAULTCONFIG,63);

  //Test open config file
    FILE *fp;
    fp = fopen(configFileString,"r");
    if(fp == NULL)
    {
      printf("RHD Core: Error reading configuration file: %s\n",configFileString);
      exit(1);
    } else printf("RHD Core: Using configuration file: %s\n",configFileString);
    fclose(fp);

          /******  Initialize RHD Core components  ********/
    if (shedulerInitXML(configFileString) < 0) { //initialize sheduler
      fprintf(stderr,"RHD Core: Error initializing scheduler\n");
      exit(1);
    }

    if (serverInitXML(configFileString) < 0) {
      fprintf(stderr,"RHD Core: Error initializing server\n");
      exit(1);
    }

    //Create tick variable in database
    iTick =  createVariable('r',1,"tick");

    /****** Initialize all hardware plgins here ********/
    //First load plugin libs from XML config file
    if (rhdInitXML(configFileString) < 0) { //initialize sheduler
            fprintf(stderr,"RHD Core: Error initializing plugins\n");
            exit(1);
    }
          
          //Sort plugins for execution order if there are any security override plugins
    if (maxSafety > 0) {
      int pluginCnt = 0;
      pluginFunc *pluginSort 	= NULL;
      pluginSort = realloc(pluginSort,sizeof(pluginFunc)*numPlugins);
      //First move safety plugin in reverse priority order
      for (j = maxSafety; j > 0; j--) {
        for (i = 0; i < numPlugins; i++) {
          if (plugin[i].safety == j) {
            pluginSort[pluginCnt] = plugin[i];
            pluginCnt++;
          }
        }
      }
      //Then the remaining plugins
      for (i = 0; i < numPlugins; i++) {
        if (plugin[i].safety <= 0) {
          pluginSort[pluginCnt] = plugin[i];
          pluginCnt++;
        }
      }
      
      //Copy sorted list back 
      memcpy(plugin,pluginSort,sizeof(pluginFunc)*numPlugins);
      free(pluginSort); //Free sorting memory
            
      //Print execition order
      printf("\nRHD Plugin: Initialization order for safety override\n");
      for (i = 0; i < numPlugins; i++) {
        printf("   %2d: %-16s ",i+1,plugin[i].name);
        if (plugin[i].safety > 0) {
            printf("Safety priority: %d\n",plugin[i].safety);
        } else printf("No safety priority\n");
      }
      printf("\nRHD Plugin: Periodic update is in reverse order to\n"
        "allow higher priority plugins to modify write values\n"
        "for lower priority plugins.\n");
    }

    printf("\nRHD Plugin: Initializing plugins\n");

    //Run init functions on plugins without safety property set.
    for (i = 0; i < numPlugins; i++) {
      if (plugin[i].safety <= 0) {
        if ((*plugin[i].initXML)(configFileString) < 1) {
          if (plugin[i].critical) {
            fprintf(stderr,"Failed initializing critical plugin: %s ... Exiting!\n",plugin[i].name);
            exit(1);
          } else {
            fprintf(stderr,"Failed initializing non-critical plugin: %s...  Proceeding however\n",plugin[i].name);
            plugin[i].enable = 0; //Disable plugin
          }
        }
      }
    }
          //Run THEN initialize safety plugin to gain access to control variables from previous plugins
    for (i = 0; i < numPlugins; i++) {
      if (plugin[i].safety > 0) {
        if ((*plugin[i].initXML)(configFileString) < 1) {
          if (plugin[i].critical) {
            fprintf(stderr,"Failed initializing critical plugin: %s ... Exiting!\n",plugin[i].name);
            exit(1);
          } else {
            fprintf(stderr,"Failed initializing non-critical plugin: %s...  Proceeding however\n",plugin[i].name);
            plugin[i].enable = 0; //Disable plugin
          }
        }
      }
    }
          
    //Lock database before core componets are initialized
    printf("\nRHD Core: Starting RHD Core components\n");
    usleep(100000); //Make sure all threads are started
    databaseSoftRealtime(); //Lock database for no further allocation
    if (startServer() < 0) {
      fprintf(stderr,"RHD Core: Error starting server... Exiting\n");
      exit(1);
    }

    /*** Go to Soft RT with locked memory - No more initialization ***/
    if (schedulerRealtime() < 0 ) {
      fprintf(stderr,"RHD Core: Error starting scheduler... Exiting\n");
      exit(-1);
    }

    /*** End of initialization ***/
    fprintf(stderr,"RHD Core: End of initialization - RHD is running\n\n");

  while (rhdRunning) {
    //Call periodic function in all plug-ins that has them 
    // call them in reverse order, so that the highest priority (1, e.g. joy-control)
    // gets called before the (slave) plugins (e.g. the implementors - drive control)
    // for(i = 0; i < numPlugins; i++) {
    for (i = numPlugins - 1; i >= 0; i--) {
      //Only run the periodic if plugin is enabled (and init did not fail
      if (plugin[i].enable) {
        //Check if a periodic function is defined
        if (plugin[i].periodic != NULL) {
          //Run periodic function, exit if it fails
          if ((*plugin[i].periodic)(tick) < 0) {
            fprintf(stderr,"RHD Plugin: Error in periodic for %s plugin - Shutting down!\n",plugin[i].name);
            rhdRunning = 0;
          }
        }
      }
    }

    if (waitPeriodic(&tod) < 0) {
      printf("RHD Core: Sheduler error - Shutting down\n");
      rhdRunning = 0;
    }
    //Update tick just before sync, to get good timestamp
    tick++;
    setVariable(iTick,0,tick); //U
    syncClients();
  }

  //Execute exit functions before shutting down
  for(i = 0; i < numPlugins; i++) {
    if (plugin[i].enable) {
      if (plugin[i].terminate != NULL) {
        printf("RHD Core: Shutdown %s\n",plugin[i].name);
        (*plugin[i].terminate)();
      }
    }
  }

  //Shutdown scheduler
  shutdownSheduler();

  return 0;
}


void print_sched()
{
  struct sched_param param;

  switch (sched_getscheduler(0))
    {
    case SCHED_OTHER:
      fprintf(stderr, "   Scheduler: other\n");
      break;
    case SCHED_FIFO:
      sched_getparam(0, &param);
      fprintf(stderr, "   Scheduler: FIFO, priority %d\n", param.sched_priority);
      break;
    case SCHED_RR:
      sched_getparam(0, &param);
      fprintf(stderr, "   Scheduler: RR, priority %d\n", param.sched_priority);
      break;
    default:
      perror("getscheduler");
      break;
    }
}

//Interrupt handler to end RHD
void endRhd (int sig) {

  printf("\n** Shutting down RHD **\n");
  disconnectServer(); //Shutdown server
  rhdRunning = 0;
}

/************************** XML Initialization **************************/

///Struct for shared parse data
typedef struct  {
    int depth;              /**< Depth of the XML tree */
    char skip;              /**< Skip all tags below this level */
    char enable;            /**< Enable use of this module */
		char pluginpath[128];
		struct pluginLoad {
			char name[128];
			char basepath[128];
			char libname[128];
			char enable;
			char critical;
                        int  safety;
		} plugin;
  }parseInfo;

//Parsing functions
void XMLCALL rhdStartTag(void *, const char *, const char **);
void XMLCALL rhdEndTag(void *, const char *);
int  loadPlugin(void *data);

/** \brief Initialize the Plugins structure
 *
 * Reads the XML file and loads plugin libs
 * 
 * Must only be used in initialization.
 * 
 * \param[in] *char filename
 * Filename of the XML file
 * 
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
int rhdInitXML(char *filename) {

  parseInfo xmlParse; 
  char *xmlBuf = NULL;
	int xmlFilelength;
  int done = 0;
  int len;
  FILE *fp;

  //Zero all structures
  memset(&xmlParse,0,sizeof(parseInfo));

  printf("\nRHD Plugin: Loading plugins from shared libs\n");


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "RHD Plugins: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, rhdStartTag, rhdEndTag);
   //Setup shared data
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("RHD Plugins: Error reading: %s\n",filename);
    return -1;
  }
	//Get the length of the file
	fseek(fp,0,SEEK_END);
	xmlFilelength = ftell(fp); //Get position
	fseek(fp,0,SEEK_SET); //Return to start of file

	//Allocate text buffer
	xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
	if (xmlBuf == NULL) {
		fprintf(stderr, "RHD Plugins: Couldn't allocate memory for XML File buffer\n");
		return -1;
	}
	memset(xmlBuf,0,xmlFilelength);

	//Read XML file
  len = fread(xmlBuf, 1, xmlFilelength, fp);
  fclose(fp);

  //Start parsing the XML file
  if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) {
    fprintf(stderr, "RHD Plugins: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
	//Free memory
  XML_ParserFree(parser);
	free(xmlBuf);

 return 1;
}

///Parse XML start tag
void XMLCALL
rhdStartTag(void *data, const char *el, const char **attr)
{
  int i = 0;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1. and 2. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0))) {
      info->skip = info->depth;
      return;
    }
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("plugins",el)) {
    //Check for the correct depth for this tag
    if(info->depth != 2) {
      printf("Error: Wrong depth for the %s tag\n",el);
      info->skip = info->depth; //Skip following tags in this depth
    } else {
			//Load plugins basepath
    	for(i = 0; attr[i]; i+=2) if ((strcmp("basepath",attr[i]) == 0)) {
      	strncpy(info->pluginpath,attr[i+1],127);
    	}
		}
  } 
  //Load plugins (only in depth 3
  else if(info->depth == 3) {
		//Clear plugin info struct
		memset(&(info->plugin),0,sizeof(struct pluginLoad));
		//Load plugin name
		strncpy(info->plugin.name,el,127);
		//Copy basepath back into plugin structure
		strncpy(info->plugin.basepath,info->pluginpath,127); 
                //Set safety parameter to -1 (for default no safety override)
                info->plugin.safety = -1;

		//Load plugins basepath
    for(i = 0; attr[i]; i+=2) if ((strcmp("lib",attr[i]) == 0)) {
     	strncpy(info->plugin.libname,attr[i+1],127);
    }
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
     	info->plugin.enable = 1; 
    }
    for(i = 0; attr[i]; i+=2) if ((strcmp("critical",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
    	 info->plugin.critical = 1; 
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("safety",attr[i]) == 0) {
    	 info->plugin.safety = atoi(attr[i+1]); 
    }

		//Load the plugin
		loadPlugin(&(info->plugin));

  }
	//Skip all plugin-childs
	else if(info->depth > 3) {
		info->skip = info->depth;
	}
}

///Parse XML Endtag
void XMLCALL
rhdEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  //Only process end tag if not skipping
  if (info->skip == 0) { 
  }

  //Stop skipping, when back to level before skip
  if (info->depth < info->skip) {
    info->skip = 0;
  } 
}

int loadPlugin (void *data) {

	struct pluginLoad *loadInfo = (struct pluginLoad *) data;
	char totalLibpath[256];
  char *error;

	//Only load plugin if it is enabled
	if (loadInfo->enable) {
		printf("   Loading %-16s ",loadInfo->libname);
		if (loadInfo->critical) printf("(Critical");
		else printf("(");

		//Allocate space for plugin
		numPlugins++;
		plugin = realloc(plugin,sizeof(pluginFunc)*numPlugins);
		memset(&plugin[numPlugins-1],0,sizeof(pluginFunc));
		
		//** Open lib
		//Make open-string
		sprintf(totalLibpath,"%s%s",loadInfo->basepath,loadInfo->libname);
		//Load plugin, resolving all symbols and using local visibility
		plugin[numPlugins-1].libHandle = dlopen (totalLibpath, RTLD_NOW | RTLD_LOCAL);
    if (plugin[numPlugins-1].libHandle == NULL) {
			printf("\nError loading lib: %s - Exiting\n",totalLibpath);
			printf("Error message: %s\n",dlerror());
			exit(1);
     }

		//Load initXML function
		plugin[numPlugins-1].initXML = dlsym(plugin[numPlugins-1].libHandle,"initXML");
		if ((error = dlerror()) != NULL) {
			printf("\nError: No initXML() function found in %s - Exiting\n",loadInfo->libname);
			printf("Error message: %s\n",dlerror());
			exit(1);
		}

		//Load periodic and terminate functions 
		//These are only optional functions, so no error check
		plugin[numPlugins-1].periodic = dlsym(plugin[numPlugins-1].libHandle,"periodic");
		if(loadInfo->critical) printf(", ");
                if ((error = dlerror()) != NULL) {
                        
			printf("No periodic");
			plugin[numPlugins-1].periodic = NULL; //disable this call
		}	
		else printf("Periodic");
		plugin[numPlugins-1].terminate = dlsym(plugin[numPlugins-1].libHandle,"terminate");
		if ((error = dlerror()) != NULL) {
			printf(", No terminate");
			plugin[numPlugins-1].terminate = NULL;
		}	else printf(", Terminate");
                //Print possible safety override
                if (loadInfo->safety > 0) {
                    printf(", Safety override pri.:%d)\n",loadInfo->safety);
                }else printf(")\n");

		//Transfer the last details to plugin struct
		plugin[numPlugins-1].critical = loadInfo->critical; 
		plugin[numPlugins-1].enable = loadInfo->enable;
		strncpy(plugin[numPlugins-1].name,loadInfo->name,127);
                plugin[numPlugins-1].safety = loadInfo->safety;
                if (loadInfo->safety > maxSafety) maxSafety = loadInfo->safety;


	} else {
		printf("   Not loading %s\t It's disabled\n",loadInfo->name);
	}

return 1;
}


