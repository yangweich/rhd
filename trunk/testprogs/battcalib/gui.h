






#ifndef GUI_H
	#define GUI_H
  
  #define TITLE "SMR Battery Calibrator v0.1"


extern volatile int running;

void initGUI(char *);
void clearGui(void);
void drawFrame(void);
void cmdParser(void);
int  processCmd(char *cmd);
void printStatus(void);
void printVariables(void);
#endif


