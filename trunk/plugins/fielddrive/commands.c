//##########################################################
//##                      Commands.c	                  ##
//##         Servo control Commands for robotarm          ##
//##                                   Version 2012.06.01 ##
//##########################################################
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <termio.h>

#include "dynamixel.h"

//Defined values for calculation
#define PI				3.141592f
#define NUM_ACTUATOR	     	1
#define DEGREE 		   0.87890625//Deci-degrees. A rotation is 3600.
#define RPM					0.114
#define PROCENT		   0.09765625

// Control table address. L[Low]/H[High]
// Values are used future all like this for more description.
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_GOAL_SPEED_L			32
#define P_GOAL_SPEED_H			33

//Debugging flag - Enables printout
//#define DEBUG 0				//!<Set to 1 to enable debug printout

void PrintErrorCode(void);

//Value from Dynamixel regarding packet status.
extern int gbCommStatus;

int init_connection(int baudrate, int speed, int deviceIndex){
	#ifdef DEBUG
	printf("This Function will init the connection of the USB2Dynamixel Dongle.\n Furthermore it will put servos into position 0.");
	printf("Beaware of the baudrate that you initialise from XML file. \n Since if this value does not correlate with servos, no connection will be obtained.\n");
	#endif

	///////// Open USB2Dynamixel ////////////
	if( dxl_initialize(deviceIndex, baudrate) == 0 )
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		printf( "Press Enter key to terminate...\n" );
		getchar();
		return 0;
	}
	else
		printf( "Succeed to open USB2Dynamixel!\n" );

	// Set goal speed
	dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, speed );

	// Set goal position, NOT used!
	//dxl_write_word( BROADCAST_ID, P_GOAL_POSITION_L, 0 );

	return 1;
}

/**
 * \param servo ID
 * \returns servo voltage in Volt x 10 */
int getCurrentVoltage(int Id)
{ // get servo voltage (x10)
	int CurrentVolt;
	CurrentVolt = dxl_read_byte(Id,42);
	PrintErrorCode();
	return CurrentVolt;
}

/**
 * \param id is servo ID
 * \returns current temperature in degrees celcius */
int getCurrentTemperature(int Id)
{
	int CurrentTemp;
	CurrentTemp = dxl_read_byte(Id, 43);
	PrintErrorCode();
	return CurrentTemp;
}

int getMoving(int Id)
{

	#ifdef DEBUG
	printf("This Function will give you information if servo is moving");
	#endif
	int CurrentMove;
	CurrentMove = dxl_read_byte(Id,46);
	#ifdef DEBUG
	if(CurrentMove == 1){
	  printf("Moving = TRUE ( %d )\n",CurrentMove);
	}
	else
	  printf("Moving = FALSE ( %d )\n",CurrentMove);
	//printf("Position test2 degree = %d (%X)\n",GoalPos1, GoalPos1);
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
	return CurrentMove;
}

int getLoad(int Id)
{ // load in % +/- 100
	#ifdef DEBUG
	printf("This Function will give you information how many procent of torque is used");
	#endif
	int CurrentLoad;
	CurrentLoad= dxl_read_word(Id,40);
	if(CurrentLoad >= 1024){
		CurrentLoad = -(CurrentLoad - 1024);
	}
	CurrentLoad = CurrentLoad * PROCENT;
	#ifdef DEBUG
	printf("Used Torque in Procent = %d (negative is CW)\n", CurrentLoad);
	//printf("Position test2 degree = %d (%X)\n",GoalPos1, GoalPos1);
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
	return CurrentLoad;
}

/**
 * Get current load in servo units 0..1023 CCV, 1024..2047 CV, 0 and 1024 is no load */
int getCurrentLoad(int Id)
{

  #ifdef DEBUG
  printf("This Function will give you information how many procent of torque is used");
  #endif
  int CurrentLoad;
  CurrentLoad= dxl_read_word(Id,40);
//   if(CurrentLoad >= 1024){
//     CurrentLoad = -(CurrentLoad - 1024);
//   }
//   CurrentLoad = CurrentLoad * PROCENT;
  #ifdef DEBUG
  printf("Used Torque in Procent = %d (negative is CW)\n", CurrentLoad);
  //printf("Position test2 degree = %d (%X)\n",GoalPos1, GoalPos1);
  printf("status = %d\n",gbCommStatus);
  PrintCommStatus(gbCommStatus);
  #endif
  PrintErrorCode();
  return CurrentLoad;
}

int MaxLoad(int Id)
{

	#ifdef DEBUG
	printf("This Function will give you information how many procent of torque is used");
	#endif
	int MAXload;
	MAXload = dxl_read_word(Id,14);
	//CurrentPos = dxl_makeword(getAngle(ID,36),getAngle(ID,37));
	#ifdef DEBUG
	printf("MaxLOAD = %d (%x)\n",MAXload,MAXload);
	//printf("Position test2 degree = %d (%X)\n",GoalPos1, GoalPos1);
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
	return MAXload;
}
void setMaxLoad(int Id, int load)
{
	#ifdef DEBUG
	printf("This Function will let you change Max load");
	#endif
	//This is only for user information.
	if(Id==254){
	  Id = BROADCAST_ID;
	}
	load = (int)load/PROCENT;
	dxl_write_word( Id, 14, load);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
}
int TorqueEnable(int Id)
{

	#ifdef DEBUG
	printf("This Function will give you information if Torque is enabled");
	#endif
	int Torque;
	Torque = dxl_read_byte(Id,24);
	//CurrentPos = dxl_makeword(getAngle(ID,36),getAngle(ID,37));
	#ifdef DEBUG
	if(Torque == 1){
	  printf("Torque = Enabled ( %d (%x))\n",Torque,Torque);
	}
	else
	  printf("Torque = Unenabled ( %d (%x))\n",Torque,Torque);
	//printf("Position test2 degree = %d (%X)\n",GoalPos1, GoalPos1);
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
	return Torque;
}
//Commands
// void changePosL(int Id, int Pos)
// {
// 	#ifdef DEBUG
// 	printf("This Function will let you change position to a given angle.\n Notice that it cannot cross 360 degrees.\n");
// 	printf("Therefore 361 does not exist, put 1, and it will go all the way back.");
// 	#endif
// 	//For the user
// 	if(Id==254){
// 	  Id = BROADCAST_ID;
// 	}
// 	Pos = (int)Pos/DEGREE;
// 	dxl_write_word( Id, P_GOAL_POSITION_L, Pos);
// 	#ifdef DEBUG
// 	printf("status = %d\n",gbCommStatus);
// 	PrintCommStatus(gbCommStatus);
// 	#endif
// 	PrintErrorCode();
// 	}
//Commands
/** Change position of servo in servo units
 *  \param Id is the ID of the servo
 *  \param Pos is the (positive) integer position (0..4095) to go to */
void changeGoalPos(int Id, int Pos)
{
  #ifdef DEBUG
  printf("This Function will let you change position to a given angle.\n Notice that it cannot cross 360 degrees.\n");
  printf("Therefore 361 does not exist, put 1, and it will go all the way back.");
  #endif
  //For the user
  if(Id==254){
    Id = BROADCAST_ID;
  }
  //Pos = (int)Pos/DEGREE;
  dxl_write_word( Id, P_GOAL_POSITION_L, Pos);
  #ifdef DEBUG
  printf("status = %d\n",gbCommStatus);
  PrintCommStatus(gbCommStatus);
  #endif
  PrintErrorCode();
  }

int getPos(int Id)
{
	#ifdef DEBUG
	printf("This Function will give you information on what angle the joint is in");
	#endif
	int CurrentPos;
	CurrentPos = dxl_read_word(Id,36);
	#ifdef DEBUG
	printf("Position test1 degree = %d (%x)\n",CurrentPos,CurrentPos);
	//printf("Position test2 degree = %d (%X)\n",GoalPos1, GoalPos1);
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
	return round(CurrentPos*DEGREE);
}
/**
 * Get current position in servo units (0..4095)
 * \param id is servo ID
 * \returns integer position */
int getCurrentPos(int Id)
{
  #ifdef DEBUG
  printf("This Function will give you information on what angle the joint is in");
  #endif
  int CurrentPos;
  CurrentPos = dxl_read_word(Id,36);
  #ifdef DEBUG
  printf("Position test1 degree = %d (%x)\n",CurrentPos,CurrentPos);
  //printf("Position test2 degree = %d (%X)\n",GoalPos1, GoalPos1);
  printf("status = %d\n",gbCommStatus);
  PrintCommStatus(gbCommStatus);
  #endif
  PrintErrorCode();
  return CurrentPos;
}

/**
 * Get goal position in servo units
 * \param Id is servo ID
 * \returns goal position */
int getGoalPos(int Id)
{
	#ifdef DEBUG
	printf("This Function will give you information on what angle the joint is going to be");
	#endif
	int Pos;
	Pos = dxl_read_word(Id,30);
	#ifdef DEBUG
	printf("GoalPosition degree = %d (%x)\n",Pos,Pos);
	//printf("Position test2 degree = %d (%X)\n",GoalPos1, GoalPos1);
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
	return Pos;
}
int setMoveSpeed(int Id, int rpm)
{
	#ifdef DEBUG
	printf("This Function will give you information on max move speed");
	#endif
	rpm = rpm/RPM;
	dxl_write_word( Id, 32, rpm);
	//CurrentPos = dxl_makeword(getAngle(ID,36),getAngle(ID,37));
	#ifdef DEBUG
	printf("MoveSpeed = %d (%x)\n",rpm,rpm);
	//printf("Position test2 degree = %d (%X)\n",GoalPos1, GoalPos1);
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
	return rpm;
}

// int getPressentSpeed(int Id)
// {
// 	#ifdef DEBUG
// 	printf("This Function will give you information on current move speed");
// 	#endif
// 	int CurrentSpeed;
// 	CurrentSpeed = dxl_read_word(Id,38)*RPM;
// 	#ifdef DEBUG
// 	printf("speed= %d (%x)\n",CurrentSpeed,CurrentSpeed);
// 	//printf("Position test2 degree = %d (%X)\n",GoalPos1, GoalPos1);
// 	printf("status = %d\n",gbCommStatus);
// 	PrintCommStatus(gbCommStatus);
// 	#endif
// 	PrintErrorCode();
// 	return CurrentSpeed;
// }

/**
 * Get current speed in servo units
 * \param id servo ID
 * \returns a number from 0 to 1023 (CCV) and 1024 to 2047 (CV) 0 and 1024 are zero */
int getCurrentSpeed(int Id)
{
  int CurrentSpeed;
  CurrentSpeed = dxl_read_word(Id, 38);
  PrintErrorCode();
  return CurrentSpeed;
}

int getAngle(int ID,int direction)
{
	#ifdef DEBUG
	printf("This Function will get individuel byte of the joints Angle");
	#endif
	direction = dxl_read_byte(ID,direction);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
	return direction;
}
//
//The following functions, only debugging methods.
// #ifdef DEBUG
// int getValueByte()
// {
// 	#ifdef DEBUG
// 	printf("This Function will give you information on what a byte you specify. See information paper.");
// 	#endif
// 	int setID;
// 	int myvariable;
// 	int GoalPos;
// 	printf("input Value return :    ");
// 	scanf("%d",&myvariable);
// 	printf("Set ID :    ");
// 	scanf("%d",&setID);
// 	GoalPos = dxl_read_byte(setID,myvariable);
// 	#ifdef DEBUG
// 	printf("hex : %x andet forsøg",GoalPos);
// 	printf("status = %d\n",gbCommStatus);
// 	PrintCommStatus(gbCommStatus);
// 	#endif
// 	PrintErrorCode();
// 	return GoalPos;
// }
// int getValueWord()
// {
// 	#ifdef DEBUG
// 	printf("This Function will give you information on what a word(2bytes) you specify. See information paper.");
// 	#endif
// 	int setID = 0;
// 	int myvariable;
// 	int GoalPos;
// 	printf("input Value return :    ");
// 	scanf("%d",&myvariable);
// 	printf("Set ID :    ");
// 	scanf("%d",&setID);
// 	GoalPos = dxl_read_word(setID,myvariable);
// 	#ifdef DEBUG
// 	printf("hex : %x andet forsøg",dxl_read_word(setID,myvariable));
// 	printf("status = %d\n",gbCommStatus);
// 	PrintCommStatus(gbCommStatus);
// 	#endif
// 	PrintErrorCode();
// 	return GoalPos;
// }
// #endif

//Ping function if status return level is 2. P
int getID(int ID)
{
	#ifdef DEBUG
	printf("This Function will give you the current ID of the Servo");
	#endif
	ID = dxl_read_byte(ID,3);
	#ifdef DEBUG
	printf("Current ID of this servo: %x",ID);
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
	return ID;
}

void changeID(int Id, int IdNew)
{
	#ifdef DEBUG
	printf("This Function will enable you to change the ID of the servo");
	#endif
	dxl_write_byte(Id,3,IdNew);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
}

void getBaudRate(int ID)
{
	#ifdef DEBUG
	printf("This Function will give you the current BaudRate which the Servo communicates");
	#endif
	int Baud;
	Baud = dxl_read_byte(ID,4);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
	printf("New operational Baud Rate : %x",Baud);
}

//SHOULD ONLY BE USED IN CONFIGURATION
/*
void changeBaudRate(int Id, float Baud)
{
	#ifdef DEBUG
	printf("This Function will change the BaudRate for the Servo communicates");
	#endif
	printf("Grundet sikkerhed i systemet er det ikke muligt at stille Baudrate individuelt for servoerne.");
	  Id = BROADCAST_ID;

	printf("New Baud Rate(Must follow true values, for help press 0):    ");
	//scanf("%d",&Baud);
	if(Baud == 0){
	  printf("1   for 1000000 BPS\n \
		  3   for  500000 BPS\n \
		  4   for  400000 BPS\n \
		  7   for  250000 BPS\n \
		  9   for  200000 BPS\n \
		  16  for  115200 BPS\n \
		  34  for   57600 BPS\n \
		  103 for   19200 BPS\n \
		  207 for    9600 BPS");

	}else if(Baud==1 || Baud==3 || Baud==4 || Baud==7 || Baud==9 || Baud==16 || Baud==34 || Baud==103 || Baud==207){
 	  dxl_write_byte(Id,4,Baud);
	  Baud = 2000000.0f / (Baud + 1);
 	  dxl_hal_set_baud(Baud);
	}
	else{
	  printf("Wrong value, start function again. Press 0 for help");
	}
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
}*/

//controller parameters
void getP(int ID)
{
	#ifdef DEBUG
	printf("This Function will give you the current P parameter for the servo");
	#endif
	int pVal;
	pVal = dxl_read_byte(ID,26);
  if (0)
    printf("dynamixel pVal %d\n", pVal);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	printf("New P value : %x",pVal);
	#endif
	PrintErrorCode();
}
void getI(int ID)
{
	#ifdef DEBUG
	printf("This Function will give you the current I parameter for the servo");
	#endif
	int iVal;
	iVal = dxl_read_byte(ID,27);
  if (0)
    printf("dynamixel iVal %d\n", iVal);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	printf("New I value : %x",iVal);
	#endif
	PrintErrorCode();
}
void getD(int ID)
{
	#ifdef DEBUG
	printf("This Function will give you the current D parameter for the servo");
	#endif
	int dVal;
	dVal = dxl_read_byte(ID,28);
  if (0)
    printf("dynamixel dVal %d\n", dVal);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	printf("New D value : %x",dVal);
	#endif
	PrintErrorCode();
}
void changeP(int Id, int pValue)
{
	#ifdef DEBUG
	printf("This Function will change the P parameter for the Servo communicates");
	#endif
	 dxl_write_byte(Id,28,pValue);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
}
void changeI(int Id, int iValue)
{
	#ifdef DEBUG
	printf("This Function will change the I parameter for the Servo communicates");
	#endif
	  dxl_write_byte(Id,27,iValue);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
}
void changeD(int Id, int dValue)
{
	#ifdef DEBUG
	printf("This Function will change the D parameter for the Servo communicates");
	#endif
	 dxl_write_byte(Id,26,dValue);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
}
void changeReturnDelayTime(int Id, int DelayTime){
#ifdef DEBUG
	printf("This Function will change the Delay time for the server to answer");
	#endif
	 dxl_write_byte(Id,5,DelayTime);
	#ifdef DEBUG
	printf("status = %d\n",gbCommStatus);
	PrintCommStatus(gbCommStatus);
	#endif
	PrintErrorCode();
}

// Print communication status
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXSUCCESS:
			printf("COMM_TXSUCCESS: Transmit success!\n");
			break;

	case COMM_RXSUCCESS:
			printf("COMM_RXSUCCESS: receiving success!\n");
			break;

	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now receiving status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode(void)
{
  if (dxl_get_rxpacket_error(0x7f))
  {
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
      printf("Input voltage error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
      printf("Angle limit error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
      printf("Overheat error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
      printf("Out of range error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
      printf("Checksum error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
      printf("Overload error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
      printf("Instruction code error!\n");
  }
}
