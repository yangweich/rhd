/** \file gui.c
 *  \brief Graphical user interface for RHDTest client program
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1965 $
 *  $Date: 2012-07-29 07:05:27 +0200 (Sun, 29 Jul 2012) $
 */
 /***************************************************************************
 *                  Copyright 2009 Anders Billesø Beck                     *
 *                  anders.billeso.beck@teknologisk.dk                     *
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
 *   License along with this program; if not,  to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
/************************** Version control information ***************************/
 #define VERSION          "$Rev: 1965 $:"
 #define DATE             "$Date: 2012-07-29 07:05:27 +0200 (Sun, 29 Jul 2012) $:"
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
#include <linux/serial.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <expat.h>
#include <dlfcn.h>
#include <curses.h>

#include <rhd.h>

#include "gui.h"


#define RHDCLIENTTITLE "RHD Client v1.0"
#define SCREENWIDTH 200
#define SCREENHEIGTH 80

int maxX, maxY;

void *rhdTask(void *something);
pthread_t rhdThread;
pthread_attr_t attr;


//RHD parameters
volatile int running = 1;
char rhdStatus = -1;
char rhdHost[128], msgText[512];
int  varPage = 0;
double runTime = 0, msgTime = 0;

void initGUI(pthread_mutex_t *dMutex) {
  pthread_mutex_t *drawMutex = dMutex;

  //Init ncurses
  (void) initscr();      /* initialize the curses library */
  getmaxyx(stdscr,maxY,maxX);

  if (has_colors())
  {
    start_color();
    /*
    * Simple color assignment, often all we need.  Color pair 0 cannot
    * be redefined.  This example uses the same value for the color
    * pair as for the foreground color, though of course that is not
    * necessary:
    */
    init_pair(1, COLOR_RED,     COLOR_BLACK);
    init_pair(2, COLOR_GREEN,   COLOR_BLACK);
    init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
    init_pair(4, COLOR_BLUE,    COLOR_BLACK);
    init_pair(5, COLOR_CYAN,    COLOR_BLACK);
    init_pair(6, COLOR_MAGENTA, COLOR_BLACK);
    init_pair(7, COLOR_WHITE,   COLOR_BLACK);
  }
  clearGui();

  //Start rhd thread
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED); 
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  if (pthread_create(&rhdThread, &attr, rhdTask, drawMutex))
  {
    perror("   Can't start RHD thread");
  }
}

void clearGui(void) {
  int tmpX, tmpY;
  getyx(stdscr,tmpY, tmpX);
  clear();
  drawFrame();
  printStatus();
  refresh();
  move(tmpY,tmpX);
}

/**
 * Get time since passed in us since this reference time
 * \param refTime reference time
 * \returns time passed in microseconds. */
int32_t getTimePassed(struct timeval refTime)
{
  struct timeval tv;
  //
  gettimeofday(&tv, NULL);
  int ds = tv.tv_sec - refTime.tv_sec;
  if (ds > INT32_MAX / 1000000)
    return INT32_MAX;
  else
  {
    ds = ds * 1000000 + tv.tv_usec - refTime.tv_usec;
    return ds;
  }
}


/** Drawing task to continously update the RHD variables **/
void *rhdTask(void *dMutex) {
  pthread_mutex_t *drawMutex = (pthread_mutex_t*) dMutex;
  //int rhdCnt = 0;
  int tmpX, tmpY;
  struct timeval tv;
  int32_t synctime;
  //
  while (running) {
    gettimeofday(&tv, NULL);
    if (rhdStatus > 0) {
        rhdStatus = rhdSync();
    } else {
        usleep(100000);
    }
    synctime = getTimePassed(tv);
    runTime += (double)synctime / 1000000.0;
    //Lock mutex, when drawing..
    pthread_mutex_lock(drawMutex);
      getyx(stdscr,tmpY, tmpX);
      if (rhdStatus < 0) {
          printStatus();
      } else if (getSymbolTableSize('r') > 0) { //only if any variables are in database
          printVariables();
          printPeriod();
      }
      printMessage(); //Print the popups
      move(tmpY,tmpX);
      refresh();
    pthread_mutex_unlock(drawMutex);
  }
  return NULL;
}


void drawFrame(void) {
  int dotWidth = (maxX - strlen(RHDCLIENTTITLE) - 4);
  int i;
  char tmpStr[64];

  //print title
  move(0,0);
  for(i = 0; i < (dotWidth/2); i++) printw("*");
  attron(A_BOLD);
  printw("  %s  ",RHDCLIENTTITLE);
  attroff(A_BOLD);
  for(i = 0; i < (dotWidth/2); i++) printw("*");

  //print sides
  for(i = 0; i < maxY; i++) {
          mvprintw(i,0,"*");
          mvprintw(i,maxX-1,"*");
  }

  //print bottom
  move(maxY-1,0);
  for(i = 0; i < maxX; i++) printw("*");

  //Create command field
  move(maxY-3,0);
  for(i = 0; i < maxX; i++) printw("*");

  //Create status field
  move(maxY-5,0);
  for(i = 0; i < maxX; i++) printw("*");

  snprintf(tmpStr,64," Press F1 for help ****");
  move(maxY-1,maxX-strlen(tmpStr));
  printw("%s",tmpStr);

  snprintf(tmpStr,64,">> ");
  move(maxY-2,2);
  printw("%s",tmpStr);
}

void printStatus(void) {

  int i;
  move(maxY-4,3);
  for (i = 0; i < maxX-5; i++) printw(" ");

  if (rhdStatus < 0) {
          mvprintw(maxY-4,3,"Not connected to RHD Server...");
          } else {
          mvprintw(maxY-4,3," Host: %s    Access: %c",rhdHost,rhdStatus);
          printw("    Database: %d r / %d w",getSymbolTableSize('r'),getSymbolTableSize('w'));
  } 
  refresh();
}


void printVariables(void) {
  int varPrPage = maxY - 6;
  int firstVar = varPage * varPrPage;
  int lastVar = ((varPage+1) * varPrPage)-1;
  int n, thisVar, i;
  int charLeft;
  char dir;
  char tmpStr[512];
  symTableElement *sTable;
  int tmpX, tmpY;
  getyx(stdscr,tmpY, tmpX);
  mvprintw(1,maxX-13,"Page %d / %d",varPage+1, (getSymbolTableSize('w')+ getSymbolTableSize('r'))/varPrPage +1);
  for (n = firstVar; n < lastVar; n++) {
    charLeft = maxX - 4;
    if (n < getSymbolTableSize('r')) {
            thisVar = n;
            dir = 'r';
    } else if (getSymbolTableSize('w') > 0) {
            thisVar = n - getSymbolTableSize('r');
            dir = 'w';
            if (thisVar >= getSymbolTableSize('w')) break;
    } else break;
    sTable = getSymbolTable(dir);
    move(n+2,3);
    sprintf(tmpStr,"%c: (%d) %s[%d]: ",dir,sTable[thisVar].updated, sTable[thisVar].name,sTable[thisVar].length);
    printw("%s",tmpStr);
    charLeft -= strlen(tmpStr);
    // and the variable values    
    for(i = 0; i < sTable[thisVar].length; i++) {
      sprintf(tmpStr,"(%d)", sTable[thisVar].data[i]);
      charLeft -= strlen(tmpStr);
      if (charLeft < 0) 
        break;
      printw("%s",tmpStr);
    }
    if (charLeft > 0) {
      for (i = 0; i < charLeft; i++) printw(" ");
    }
  }
  move(tmpY,tmpX);
  refresh();	
}


void printPeriod(void) {

static double oldTimestamp = 0;
double timeStamp, currentTime = 0;
char tmpStr[32];


symTableElement *sTable = getSymbolTable('r');

timeStamp = (double)sTable[0].timestamp->tv_sec; 
timeStamp += (double)(sTable[0].timestamp->tv_usec) / 1000000;

currentTime = (timeStamp - oldTimestamp);
if (currentTime > 10.0) currentTime = 9.9;
if (currentTime < 0.0) currentTime = 0.0; 

sprintf(tmpStr,"Period: %04.3f s",currentTime);

if (currentTime < 1) runTime += currentTime;

mvprintw(maxY-4,maxX-strlen(tmpStr)-2,"%s",tmpStr);

oldTimestamp = timeStamp;	
}

/** Print a pop-up message for a short duration */
void popup(char *msg, double time) {
    msgTime = runTime + time;
    strncpy(msgText,msg,maxX-10);
}

/** Print a pop-up message for a short duration */
void printMessage(void) {
  static char lastPrint = 0;
  if (msgTime > runTime) {
    mvprintw(maxY-2,5,"%s",msgText);
    lastPrint = 1;
  } else if (lastPrint) {
    move(maxY-2,5);
    memset(msgText,' ', 128);
    msgText[maxX - 7] = 0;
    printw("%s", msgText);
    //for(i = 5; i < maxX-2;i++) printw(" ");
    lastPrint = 0;
  }
}

//Interface functions
/** Return max Y value on the GUI */
int getMaxY(void) {return maxY;}
/** Return max X value on the GUI */
int getMaxX(void) {return maxX;}
/** Set rhdStatus for GUI */
int setRhdStatus(char st) {
    rhdStatus = st;
    printStatus();
    return 1;
}
/** Set rhdHost */
int setRhdHost(char *rhdh) {
    strncpy(rhdHost,rhdh,128);
    printStatus();
    return 1;
}

