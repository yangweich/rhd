/**************************************************
 *
 *    RHD Plug-in testprogram
 *
 *   Reads a plugin from argument, initializes it,
 *   and executes the periodic function at a given 
 *   interval.
 ***************************************************/

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
#include <curses.h>

#include <rhd.h>

#include "gui.h"


void end(int);
int connectRhd(char *hostname);

int main(int argc, char * argv[]) {


  printf("***********   %s   ***********\n\n",TITLE);

//Catch interrupts to terminate properly
  signal(SIGTERM, end);
  signal(SIGINT, end);
  signal(SIGKILL, end); 
	
  //Connect to RHD
  if (argc > 1) {
    if (connectRhd(argv[1]) < 0) {
      exit(0);
    }
  } else {
    printf("Error: Robot hostname must be given as parameter\n");
    printf("       i.e.:   battcalib smr2\n\n");
    exit(0);
  }
  
	//Initialize ncurses GUI
	initGUI(argv[1]);

  cmdParser();

	endwin(); //end ncurses



	return 0;
}

//Handler to connect to RHD
int connectRhd(char *hostname) {

  int /*status,*/i, battCapRet = 0;
  
  if (rhdConnect('w',hostname,DEFAULTPORT) <= 0) {
      printf("Error: Failed to connect to RHD on host: %s\n",hostname);
      return -1;
   } else {
      printf("Successfully connected to RHD\n");
   }
   
  //Sync with RHD and check if battcapacity gets updated when requested
  for (i = 0; i < 20; i++) {
      writeValueNamed("getbattcapacity", 0, 1); //Request battery capacity
      /*status =*/ rhdSync();
      battCapRet += isReadNamed("battcapacity"); //Check if battcapacity variable is updated
  }
  
  if (battCapRet < 5) {
    printf("Error: Power supply not responding to batterycapacity request (%d)\n",battCapRet);
    printf("       Are you sure that the SMR has a new power supply model?\n");
    printf("       (Otherwise it might need a firmware update (v3.31+ required)\n");
    rhdDisconnect();
    return -1;
  } else {
    printf("Powersupply is supporting calibration \n");
  }
  
  return 1;

}

//Interrupt handler to end RHD
void end (int sig) {

	endwin(); //end ncurses
	running = 0;
  printf("Shutting down SMR Battery calibrator\n");
  exit(1);


}

