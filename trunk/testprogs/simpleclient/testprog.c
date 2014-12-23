
/**********************************************
 * Testprogram for Robot Hardware Daemon Client
 **********************************************/

/*********************** Version control information ***********************/
 #define VERSION "$Rev: 91 $:"
 #define DATE    "$Date: 2008-09-26 15:47:28 +0200 (Fri, 26 Sep 2008) $:"
/***************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include <rhd.h>


int main(int argc, char * argv[]) {

symTableElement *symTable;
double oldTime = 0, newTime = 0;
double *timestamps = NULL;

//Loggin to file
FILE *logFile = NULL;
int samples = 0, counter = 0;

//Find revision number from SVN Revision
char *i,versionString[64] = VERSION, tempString[10];
int cnt = 0, cnt2 = 0;
i = strrchr (versionString,'$');
strncpy(tempString,versionString+1,(i-versionString-1));
tempString[(i-versionString-1)] = 0;
printf("**********    Robot Hardware Daemon Client Test %s   **********\n\n",tempString);

//Hostname given as input
if (argc >= 2) {
  strncpy(versionString,argv[1],64);
  printf("Hosename read from input : %s\n",versionString);
} else {
  strncpy(versionString,"127.0.0.1",64);
  printf("No hostname in input. Connecting to localhost\n");
}

//Logging 
if (argc >= 3) {
  printf("Logging enabled\n");
  samples = atoi(argv[2]);
  printf("Logging %d samples then quitting\n",samples);
  logFile = fopen("logfile.dat","w");
  timestamps = realloc(timestamps,(samples+10) * sizeof(double));
}

 if(rhdConnect('r',versionString,0) > 0) {
    printf("Connect completed\n\n");
    symTable = getSymbolTable('r');

    while((rhdSync() > 0) && (counter <= samples)) {

                //Slow down a bit
//                usleep(100000);

      //Print time
      symTable = getSymbolTable('r');
      newTime = (double)symTable[0].timestamp[0].tv_sec + (double)symTable[0].timestamp[0].tv_usec / 1000000;
      
      if (!(logFile != NULL)) { 
        printf("Period time %6.4f %d %d\n",newTime - oldTime,counter,samples);
        oldTime = newTime;
        //Print read buffer
        printf("Read:\n");
        for(cnt = 0; cnt < getSymbolTableSize('r');cnt++) {
          printf("  (%d)  %s[%d]: ",symTable[cnt].updated,symTable[cnt].name,symTable[cnt].length);
          for(cnt2 = 0;(cnt2 < symTable[cnt].length); cnt2++) printf("[%d]",symTable[cnt].data[cnt2]);
          printf("  (%d.%06d)",(int)symTable[cnt].timestamp[0].tv_sec,(int)symTable[cnt].timestamp[0].tv_usec);
          printf("\n");
        }
    
        //Do any I/O here:
        //writeValue(0,0,33);
        //writeValue(0,0,10);

        //print write buffer
        symTable = getSymbolTable('w');
        printf("Write:\n");
        for(cnt = 0; cnt < getSymbolTableSize('w'); cnt++) {
          printf("  (%d)  %s[%d]: ",symTable[cnt].updated,symTable[cnt].name,symTable[cnt].length);
          for(cnt2 = 0;(cnt2 < symTable[cnt].length); cnt2++) printf("[%d]",symTable[cnt].data[cnt2]);
          printf("  (%d.%06d)",(int)symTable[cnt].timestamp[0].tv_sec,(int)symTable[cnt].timestamp[0].tv_usec);
          printf("\n");
        }

      }
      
      //Log if samples are defined
      if ((logFile != NULL)) {
          //fprintf(logFile,"%d %d %d\n",readValue(5,0),readValue(5,1),readValue(5,2));
          timestamps[counter] = newTime;
          if (samples > 0) counter++;
      }
    }
  } else {
    printf("Connect failed\n");
  }
  
  if (logFile != NULL) {
    printf("Saving log to file...\n");
    for (counter = 0; counter < samples; counter++) 
      fprintf(logFile,"%f\n",timestamps[counter]);
  }

  printf("Shutting client down\n");
  rhdDisconnect();
  fclose(logFile);
return 0;

}
