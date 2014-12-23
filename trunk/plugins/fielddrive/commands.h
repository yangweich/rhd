//##########################################################
//##                      Commands.h	                  ##
//##         Servo control Commands for robotarm          ##
//##                                   Version 2012.06.01 ##
//##########################################################

#ifndef _COMMANDS_HEADER
#define _COMMANDS_HEADER


#ifdef __cplusplus
extern "C" {
#endif
//start connection
int init_connection(int baudrate,int speed,int deviceIndex);
// change position in degrees (0..360)
//void changePosL(int Id, int Pos);
// change position in servo units  (0..4095)
void changeGoalPos(int Id, int Pos);

// position in degrees
int getPos(int Id);
// position in servo units
int getCurrentPos(int Id);

//Print status. Status meaning in dynamixel.h
void PrintCommStatus(int CommStatus);
//print error
void PrintErrorCode(void );

int getCurrentVoltage(int Id);

int getCurrentTemperature(int Id);

int getMoving(int Id);

/**
 * \Returns load is value in pct, positive is CCW and negative is CW
 */
int getLoad(int Id);
// current load in servo units 0 and 1024 is no load 0..1023 is CCV load, 1024..2047 is CV load
int getCurrentLoad(int Id);

int MaxLoad(int Id);
void setMaxLoad(int Id, int load);

int TorqueEnable(int Id);
// desired move speed
int setMoveSpeed(int Id, int rpm);
// current speed in servo units 0..1023 and 1024..2047
int getCurrentSpeed(int Id);
// ???
//int getPressentSpeed(int Id);

//early functions. not used
/*
int getAngle(int ID,int direction);
void getID(int ID);
*/
//Get and set PID parameters. first 3 not used
/*void getP(int ID);
void getI(int ID);
void getD(int ID);*/
void changeP(int Id, int pValue);
void changeI(int Id, int iValue);
void changeD(int Id, int dValue);


void changeReturnDelayTime(int Id, int DelayTime);

//NOT used in plugin only for configuration
void changeID(int Id, int IdNew);
//void getBaudRate(int ID);
void changeBaudRate(int Id, int Baud);



#ifdef __cplusplus
}
#endif

#endif
