



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

#define FILTERLEN 100
#define SCREENWIDTH 80
#define SCREENHEIGTH 24

int maxX, maxY;

void *rhdTask(void *something);
pthread_t rhdThread;
pthread_attr_t attr;

//RHD parameters
volatile int running = 1;
char rhdStatus = -1;
char rhdHost[128];
int  varPage = 0;
int  voltFilter[FILTERLEN];
int  currFilter[FILTERLEN];
int  psuFilter[FILTERLEN];
int  calibState = 0;
int  times[6] = {0,0,0,0,0,0};
int  startBatt[2] = {0,0};
int  endBatt[2]   = {0,0};

void printTurnThing(void);
void printCalib(void);

void initGUI(char *hostname) {

    //Save hostname
    strncpy(rhdHost,hostname,128);

    //Init ncurses
    (void) initscr();      /* initialize the curses library */
    keypad(stdscr, TRUE);  /* enable keyboard mapping */
    (void) nonl();         /* tell curses not to do NL->CR/NL on output */
    (void) cbreak();       /* take input chars one at a time, no wait for \n */
    (void) noecho();         /* echo input - in color */
		
    //getmaxyx(stdscr,maxY,maxX);
    maxY = SCREENHEIGTH;
    maxX = SCREENWIDTH;
    
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

  if (pthread_create(&rhdThread, &attr, rhdTask, 0))
    {
      perror("   Can't start RHD thread");
      exit(0);
    }
    
    //Test if robot already is calibrating
    if (readValueNamed("digital",3)) { //Calibration flag
      calibState = 2;
    } else calibState = 0;
    
    struct timeval tempTime;
    gettimeofday(&tempTime,NULL);
    times[0] = tempTime.tv_sec;
    times[1] = tempTime.tv_sec;
    times[2] = tempTime.tv_sec;
    times[3] = tempTime.tv_sec;
    
    rhdStatus = 1;
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


void *rhdTask(void *something) {

int cnt;

  //Pre-fill filter
for (cnt = 0; cnt < FILTERLEN; cnt++) {
   //Update voltage and current filter
   voltFilter[cnt] = readValueNamed("analog",0);
   psuFilter[cnt]  = readValueNamed("analog",1);
   currFilter[cnt]  = readValueNamed("analog",2);
}

//reset cnt-variable
cnt = 0;

	while (running) {

      writeValueNamed("getbattcapacity", 0, 1); //Request battery capacity
			rhdStatus = rhdSync();
      
      //Update voltage and current filter
      voltFilter[cnt] = readValueNamed("analog",0);
      psuFilter[cnt]  = readValueNamed("analog",1);
      currFilter[cnt]  = readValueNamed("analog",2);
      
      cnt++;
      if (cnt >= FILTERLEN) cnt = 0;
      
      //if (cnt%10 == 0) {
        printCalib();
        printVariables();
      //}
      usleep(100000);
  }

	return something;
}


void drawFrame(void) {

int dotWidth = (maxX - strlen(TITLE) - 4);
int i;

//print title
move(0,0);
for(i = 0; i < (dotWidth/2); i++) printw("*");
attron(A_BOLD);
printw("  %s  ",TITLE);
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

//Create status field
move(maxY-4,0);
for(i = 0; i < maxX; i++) printw("*");

}

void printStatus(void) {

	int tmpX, tmpY;
	getyx(stdscr,tmpY, tmpX);

	mvprintw(maxY-3,40,"Menu:");
  mvprintw(maxY-3,50," c    Start calibration");
  mvprintw(maxY-2,50," q    Quit program");

	move(tmpY,tmpX);
	refresh();	

}

void printVariables(void) {

int tmpX, tmpY, i;
int volt = 0, psu = 0, curr = 0;
	getyx(stdscr,tmpY, tmpX);

    //Calculate running average on voltage and current
    for (i = 0; i < FILTERLEN; i++) {
      volt += voltFilter[i];
      curr += currFilter[i];
      psu  += psuFilter[i];
    }
    volt = volt / FILTERLEN;
    curr = curr / FILTERLEN;
    psu  = psu / FILTERLEN;

    mvprintw(1,3,"Connected to smr: %s",rhdHost);
    mvprintw(3,3,"Power supply:");
    mvprintw(4,5,"Battery voltage:  %4.02f V",(double)volt * 0.01273970);
    if (((double)volt * 0.01273970) > 13.00) printw(" (limit)"); //10-bit volage limits to 13.03 V
    mvprintw(5,5,"Robot voltage:    %4.02f V",(double)psu * 0.01273970);
    mvprintw(6,5,"Charge current:   %4.03f A",((double)curr * 3.0)/1000);
    mvprintw(4,40,"Battery D-capacity: %4d mAh",readValueNamed("battcapacity",0));
    mvprintw(5,40,"Battery C-capacity: %4d mAh",readValueNamed("battchgcapacity",0));
    mvprintw(6,40,"Battery SOC:        %4d pct.",readValueNamed("analog",4));

	move(tmpY,tmpX);
	refresh();	
}

void printCalib(void) {

  int tmpX, tmpY;
  getyx(stdscr,tmpY, tmpX);
  struct timeval tempTime;
  int curr = 0, i;
  
  gettimeofday(&tempTime,NULL);
  times[calibState] = tempTime.tv_sec; 

  if (calibState >= 1) {
    mvprintw(8,3,"Calibration status: ");
    mvprintw(8,60,"Runtime: ");
    mvprintw(9,5,"Stage 1: Charging batteri to full.. ");
    if (calibState == 1)  printTurnThing();
    else if (calibState > 1) printw("Done!");
    mvprintw(9,61,"%3d.%02d",(times[1]-times[0])/60,(times[1]-times[0])%60);
    
    if (readValueNamed("digital",3) && (calibState==1)) {
      calibState++;
      //Save battery capacity from start
      startBatt[0] = readValueNamed("battcapacity",0);
      startBatt[1] = readValueNamed("battchgcapacity",0);
      gettimeofday(&tempTime,NULL);
      times[calibState] = tempTime.tv_sec; 
    }
  } 
  if (calibState >= 2) {
    if (startBatt[0] != 0) {
      mvprintw(10,7,"Discharge capacity before : %4d mAh",startBatt[0]);
      mvprintw(11,7,"Charge capacity before    : %4d mAh",startBatt[1]);
    } else {
      mvprintw(10,7,"Discharge capacity before : ----");
      mvprintw(11,7,"Charge capacity before    : ----");
    }
    mvprintw(13,5,"Stage 2: Discharging battery to 10.650 V... ");
    if (calibState == 2)  printTurnThing();
    else if (calibState > 2) printw("Done!");
    mvprintw(13,61,"%3d.%02d",(times[2]-times[1])/60,(times[2]-times[1])%60);
        
    //Calculate running average on current
    for (i = 0; i < FILTERLEN; i++) {
      curr += currFilter[i];
    }
    curr = curr / FILTERLEN;
    
    if ((curr >= 100)  && (calibState==2)) {
      calibState++;
      endBatt[0] = readValueNamed("battcapacity",0);
      gettimeofday(&tempTime,NULL);
      times[calibState] = tempTime.tv_sec; 
    }
  }
  if (calibState >= 3) {
      
    mvprintw(14,7,"Discharge capacity after : %4d mAh",endBatt[0]);

    mvprintw(16,5,"Stage 3: Recharging battery... ");
    if (calibState == 3)  printTurnThing();
    else if (calibState > 2) printw("Done!");
    mvprintw(16,61,"%3d.%02d",(times[3]-times[2])/60,(times[3]-times[2])%60);
  
    if (!readValueNamed("digital",3) && (calibState==3)) {
      calibState++;
      endBatt[1] = readValueNamed("battchgcapacity",0);
      gettimeofday(&tempTime,NULL);
      times[calibState] = tempTime.tv_sec; 
    }
  } if (calibState >= 4) {
      mvprintw(17,7,"Charge capacity after    : %4d mAh",endBatt[1]);
      mvprintw(19,5,"Calibration done... ");
  
  }

  move(tmpY,tmpX);
  refresh();  

}

void printTurnThing(void) {

  static char cnt = 0;
  
  switch (cnt) {
    case 0 :
      printw("|");
      break;
   case 1 :
      printw("/");
      break;
   case 2 : 
      printw("-");
      break;
   case 3 :
      printw("\\");
      break;
   };
   
   cnt++;
   if (cnt >= 4) cnt = 0;
}


void cmdParser(void) {

//Startposition of command input
#define CMDLINESTART 18

char cmdBuffer[256];
int  tmp;
int n = 0;
int running = 1;

mvprintw(maxY-2,3,"Enter command: ");

	while (running) {
		//Get char
		tmp = getch();
		mvprintw(maxY-2,CMDLINESTART+n,"%c",tmp);
		refresh();
		
		//Special functions
		if (tmp == KEY_F(1)) {
			//Run help
		} else if ((tmp == KEY_ENTER) || (tmp == 13)) { //ENTER has been entered! Process command...
			cmdBuffer[n] = 0; //Terminate string
			if (processCmd(cmdBuffer) < 0) return;
	
			//Command line reset
			mvprintw(maxY-2,0,"*  Enter command:");
			for(n = 0; n < 10; n++) printw(" ");
			n = 0;
			move(maxY-2,CMDLINESTART);
			refresh();

		} else if (tmp < 128) { //Save input in buffer
			cmdBuffer[n] = (unsigned char) tmp;
			n++;
			if (n >= 256) n = 0;
		}
	} //While loop ends
}


int processCmd(char *cmd) {

struct timeval tempTime;

	//Command actions
  if (strncmp("q",cmd,strlen("q")) == 0) {
    return -1;
	} else if (strncmp("c",cmd,strlen("c")) == 0) {
    calibState = 1;
    gettimeofday(&tempTime,NULL);
    times[0] = tempTime.tv_sec; 
    //Start calibration
    writeValueNamed("calibratebattery", 0, 1); //Request battery calibration
	} else if (strncmp("+",cmd,strlen("c")) == 0) {
    calibState++;
  } else {
      mvprintw(maxY-2,CMDLINESTART,"Command not recognized");
      //Wait for 500ms
      refresh();
      usleep(500000);
	}
  
  return 1;

}
