/** \file cmdparser.c
 *  \brief Command input-handler and parser for rhdtest
 *
 *  \author Anders Billesø Beck
 *  $Rev: 602 $
 *  $Date: 2009-07-08 15:19:39 +0200 (Wed, 08 Jul 2009) $
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
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
/************************** Version control information ***************************/
 #define VERSION          "$Rev: 602 $:"
 #define DATE             "$Date: 2009-07-08 15:19:39 +0200 (Wed, 08 Jul 2009) $:"
/**********************************************************************************/

#include <sched.h>
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

#include "cmdparser.h"
#include "gui.h"

void printline(char *line, int xPos, pthread_mutex_t *dMutex);

void cmdParser(pthread_mutex_t *dMutex) {
    
    //SEtup the ncurses keyboard handling..
    keypad(stdscr, TRUE);  /* enable keyboard mapping */
    (void) nonl();         /* tell curses not to do NL->CR/NL on output */
    (void) cbreak();       /* take input chars one at a time, no wait for \n */
    (void) noecho();         /* echo input - in color */

    char lBuf[BUFFERLINES][BUFLEN];
    char cBuf[BUFLEN];
    int  lPos = 0, bPos = 0, cPos = 0;
    int  tmp;
    int  i;
    int running = 1, edit = 0;

    //Clear buffers
    memset(lBuf[0],0,BUFFERLINES*BUFLEN*sizeof(char));
    memset(cBuf,0,BUFLEN*sizeof(char));

    //Print an empty line
    printline(cBuf,cPos, dMutex);

    while (running) {
        //refresh();
        tmp = getch();
        //Input key parser
        switch (tmp) {
            case (KEY_F(1)):
                //display help screen
                popup("Help not implemented yet...",2.0);
                break;

            case (KEY_ENTER): //Process the buffer
            case (13):
                if (cBuf[0] == '\0') break; //Don't parse an empty string
                strncpy(lBuf[bPos],cBuf,BUFLEN);
                bPos++;
                if (bPos >= BUFFERLINES) bPos = 0;
                lPos = bPos;
                if (lPos < 0) lPos = BUFFERLINES -1;
                cPos = 0;
                edit = 0;
                if (parseCmd(cBuf) <= 0) return; //Parse the command
                memset(cBuf,0,BUFLEN);
                break;

            case (KEY_DOWN):
                if (edit) {
                    lPos = bPos;
                    strncpy(lBuf[bPos],cBuf,BUFLEN);
                    bPos++;
                    if (bPos >= BUFFERLINES) bPos = 0;
                }
                //Clear the text-line if no more lines in buffer...
                if (lBuf[(lPos+1) % BUFFERLINES][0] == '\0') 
                  memset(cBuf,0,BUFLEN);
                lPos++;
                if (lPos >= BUFFERLINES) lPos = 0;
                //if (lPos < 0) lPos = BUFFERLINES -1;
                strncpy(cBuf,lBuf[lPos],BUFLEN);
                cPos = strlen(cBuf);
                edit = 0;
                break;

            case (KEY_UP):
                if ((lPos > 0) && (lBuf[lPos-1][0] == '\0')) break; //Do not show enpty buffers
                if ((lPos == 0) && (lBuf[BUFFERLINES-1][0] == '\0')) break;//Do not show enpty buffers
                //Save the current buffer, if it has been edited..
                if (edit) {
                    lPos = bPos;
                    strncpy(lBuf[bPos],cBuf,BUFLEN);
                    bPos++;;
                    if (bPos >= BUFFERLINES) bPos = 0;
                }
                lPos--;
                //if (lPos >= BUFFERLINES) lPos = 0;
                if (lPos < 0) lPos = BUFFERLINES -1;
                strncpy(cBuf,lBuf[lPos],BUFLEN);
                cPos = strlen(cBuf);
                edit = 0;
                break;

            case (KEY_LEFT) :
                if (cPos > 0) cPos--;
                break;

            case (KEY_RIGHT):
                if (cPos < getMaxX()) cPos++;
                break;

            case (KEY_BACKSPACE):
                if (cPos == 0) break;
                for (i = cPos; cBuf[i] != '\0'; i++) cBuf[i-1] = cBuf[i];
                cBuf[i-1] = 0;
                cPos--;
                edit = 1;
                break;

            case (KEY_DC): //Delete key
                for (i = cPos; cBuf[i+1] != '\0'; i++) cBuf[i] = cBuf[i+1];
                cBuf[i] = 0;
                edit = 1;
                break;

            default: //Add the charecter to the buffer
                if (tmp < 32 ) break; //No alph-ASCII char...
                if (tmp > 125) break; //No alph-ASCII char...
                for (i = cPos; cBuf[i] != '\0'; i++); //Find last buffer index with contents
                for (; i >= cPos; i--) cBuf[i+1] = cBuf[i]; //Move chars one place
                cBuf[cPos] = tmp; //Add char to buffer
                cPos++;
                edit = 1;
                break;
        }
        printline(cBuf,cPos, dMutex); //Print the line to display

    } //While loop ends
}

/** Parse the command string */
int parseCmd(char *cmd) {
    
    char    *ptr,*ptr2;
    char    popupMsg[BUFLEN];

    //mvprintw(9,10,"Parsing: %s| ",cmd);
    
    //Seperate the string using the tokeniser
    ptr = strtok(cmd," ="); //Split by space and =
    //mvprintw(10,10,"Cmd: %s| ",ptr);

    if ((strcmp("exit",ptr) == 0) || (strcmp("e",ptr) == 0)) {
        return 0;
    } else if ((strcmp("quit",ptr) == 0) || (strcmp("q",ptr) == 0)) {
        return 0;

    //Connect to rhd...
    } else if ((strcmp("connect",ptr) == 0) || (strcmp("c",ptr) == 0)) {
        char    rhdHost[BUFLEN];
        int     rhdPort = 0;
        char    dir;
        ptr = strtok(NULL," =:");
        //Get direction
        if ((ptr != NULL) && ((strcmp("write",ptr) == 0) || (strcmp("w",ptr) == 0))) {
            dir = 'w'; //Write connection
            printw("Write ");
            ptr = strtok(NULL," =:");
        } else {
            dir = 'r'; //Read connection
        }
        if (ptr == NULL) {
            strncpy(rhdHost,DEFAULTHOST,BUFLEN);
            rhdPort = DEFAULTPORT;
        } else { //A host is assigned
            strncpy(rhdHost,ptr,BUFLEN);
            ptr = strtok(NULL," =:");
            if (ptr != NULL) {
                rhdPort = atoi(ptr);
                setRhdStatus(rhdConnect(dir,rhdHost,rhdPort));
            } else { //No port assigned
                rhdPort = DEFAULTPORT;
            }
        }
        dir = rhdConnect(dir,rhdHost,rhdPort);
        setRhdStatus(dir);
        setRhdHost(rhdHost);
        if (dir > 0) snprintf(popupMsg,BUFLEN,"Connected successfully...");
        else snprintf(popupMsg,BUFLEN,"Connection failed!!");
        popup(popupMsg,2.0);
    //Disconnect from RHD
    } else if ((strcmp("disconnect",ptr) == 0) || (strcmp("d",ptr) == 0)) {
        rhdDisconnect();
        setRhdStatus(-1); //Set disconnected status
        snprintf(popupMsg,BUFLEN,"Disconnected");
        popup(popupMsg,2.0);
    //Write variables to RHD
    } else if ((strcmp("set",ptr) == 0) || (strcmp("s",ptr) == 0)) {
        char varName[BUFLEN];
        int32_t index, value, success;
        ptr = strtok(NULL," =");
        if (ptr == NULL)
          success = -2;
        else
        {
          strncpy(varName,ptr,BUFLEN);
          //Look for the index bracket [
          ptr2 = strchr(varName,'[');
          if (ptr2 != NULL)
            *ptr2 = 0;
          if (ptr2 != NULL)
          { // read index and assigned value
            index = strtol(++ptr2, &ptr2, 0);
            ptr = strtok(NULL,"=:[]");
            if (ptr == NULL)
              // no end bracket
              success = -2;
            else
            {
              value = strtol(ptr, NULL, 0);
              success = writeValueNamed(varName,index, value);
            }
          } else  { // no index, so look for value after a space or an '='
              ptr = strtok(NULL," =");
              if (ptr == NULL)
                // no value
                success = -3;
              else
              {
                value = atoi(ptr);
                printw("Var: %s = %d",varName,value);
                success = writeValueNamed(varName,0, value);
              }
          }
        }
        if (success < 0)
        {
          if (success == -1)
            snprintf(popupMsg,BUFLEN,"Unknown variable: %s      ",varName);
          else
            snprintf(popupMsg,BUFLEN,"syntax error");
          //else
          //snprintf(popupMsg,BUFLEN,"Variable write successful    ");
          popup(popupMsg,3.0);
        }

    } else {
        popup("Unknown command",3.0);
    }



    return 1;
  //Command actions
  /*if (strncmp("connect write",cmd,strlen("connect write")) == 0) {
		if (strlen(cmd) == strlen("connect write")) {
			strncpy(rhdHost,DEFAULTHOST,128);
		} else {
			strncpy(rhdHost,&cmd[strlen("connect")+1],128);
		}
		
		refresh();
		printStatus();
	} else if (strncmp("connect",cmd,strlen("connect")) == 0) {
		if (strlen(cmd) == strlen("connect")) {
			strncpy(rhdHost,DEFAULTHOST,128);
		} else {
			strncpy(rhdHost,&cmd[strlen("connect")+1],128);
		}
		move(2,2);
		rhdStatus = rhdConnect('r',rhdHost,DEFAULTPORT);
		refresh();
		printStatus();
	} else if (strncmp("disconnect",cmd,strlen("disconnect")) == 0) {
		rhdDisconnect();
		rhdStatus = -1;
		printStatus();

	} else if ((strncmp("write",cmd,strlen("write")) == 0) ||
					 (strncmp("set",cmd,strlen("set")) == 0)){
		if (getSymbolTableSize('w') > 0) {
			int i;
			char *tok = strtok(cmd," ");
			symTableElement *wTable = getSymbolTable('w');
			tok = strtok(NULL," ");

			for(i = 0; i < getSymbolTableSize('w'); i++) {
				if (strncmp(tok,wTable[i].name,strlen(wTable[i].name)) == 0) break;
				else if (i == getSymbolTableSize('w') -1) {
					i = -1;
					break;
				}
			}
			if (i >= 0) {
				tok = strtok(NULL," ");
				int value = atoi(tok);
				wTable[i].data[0] = value;
				wTable[i].updated = 1;
			} else {
				mvprintw(maxY-2,CMDLINESTART,"Write variable not found");
				//Wait for 500ms
				refresh();
				usleep(500000);
			}
		} else {
				mvprintw(maxY-2,CMDLINESTART,"Not connected as write client");
				//Wait for 500ms
				refresh();
				usleep(500000);
		}
	}else {
		mvprintw(maxY-2,CMDLINESTART,"Command not recognized");
		//Wait for 500ms
		refresh();
		usleep(500000);
	}*/

}

/** Print the command line */
void printline(char *line, int xPos, pthread_mutex_t *dMutex) {

    int i;

    //Break the line if it is too long...
    if (strlen(line) > getMaxX()-XCMDLINESTART-2) line[getMaxX()-XCMDLINESTART-3] = '\0';

    pthread_mutex_lock(dMutex);
        mvprintw(YCMDLINESTART,XCMDLINESTART,"%s",line);
        //Clear the remaining stuff on the line...
        for (i = strlen(line)+XCMDLINESTART; i < getMaxX()-10;i++) printw(" ");
        move(YCMDLINESTART,XCMDLINESTART+xPos); //return cursor to cPos
        refresh();
    pthread_mutex_unlock(dMutex);

}

