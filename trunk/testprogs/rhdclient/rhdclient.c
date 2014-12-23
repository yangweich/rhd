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
#include "cmdparser.h"

pthread_mutex_t drawMutex = PTHREAD_MUTEX_INITIALIZER;
void end(int);

int main(int argc, char * argv[]) {

//Catch interrupts to terminate properly
  signal(SIGTERM, end);
  signal(SIGINT, end);
  signal(SIGKILL, end);
	
  //Initialize ncurses GUI and spawn the RHD update thread
  initGUI(&drawMutex);

  refresh();

  //Run the command parser in the main thread
  cmdParser(&drawMutex);

  endwin(); //end ncurses
  return 0;
}

//Interrupt handler to end RHD
void end (int sig) {

  endwin(); //end ncurses
  running = 0;
  printf("Shutting down RHD Client\n");
  exit(1);
}

