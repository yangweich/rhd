/** \file SMRRobotArm.c
 *  \brief Robot Arm for SMR - 5 joints (DTU)
 *	
//##########################################################
//##                  SMRRobotArm.h		                  ##
//##     	 XML - reading and RHD - Interface	          ##
//##                                   Version 2012.06.01 ##
//##########################################################
 */

/**********************************************************************
 *             Copyright 2012 Johann Thor Ingibergsson Mogensen       *
 *                         			                   			      *
 *                          Johann.beggi@gmail.com                    *
 *                                                                    *
 **********************************************************************/

/***************************** Plugin version  *****************************/
 #define SMRROBOTARM		     "1.1"

/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 5:"
 #define DATE             "$Date: 01.06.2012:"
 #define ID               "$Id: SMRRobotArm.c 112 2013-02-09 15:47:45Z jcan $"
/***************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <linux/serial.h>
#include <expat.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <signal.h>

//#include <netdb.h>

//RHD Core headers>
#include <database.h>
#include <globalfunc.h>

//Core Plugin
#include "dynamixel.h"
#include "dxl_hal.h"
#include "SMRRobotArm.h"
#include "commands.h"

//Debugging flag - Enables printout
//#define DEBUG 0					//!<Set to 1 to enable debug printout

/******** Global Variables *************/
static volatile char smrraRunning = 0;
typedef struct {
  int alarm;
  int kin;
  int posMIN;
  int posMAX;
  int posMOVEto;
  int posStart;
  int posPark;
  int posCURRENT;
  int speedPRESENT;
  int speedMOVE;
  int posGOAL;
  int pVal;
  int iVal;
  int dVal;
  int loadCCW;
  int loadCW;
  int loadCURRENT;
  int moving;
  int currentVOLT;
  int currentTEMP;
  int id;
} servoDEF;

typedef struct {
  servoDEF servo1;
  servoDEF servo2;
  servoDEF servo3;
  servoDEF servo4;
  servoDEF servo5;
  int BaudRate;
  int Speed;
  int DeviceIndex;
  int ReturnDelayTime;
  int start;
  int servosCnt;
  //int start;
  //To check if there is value for all servos.
  unsigned char layer0;
  unsigned char layer1;
  unsigned char layer2;
  unsigned char layer3;
  unsigned char layer4;
  unsigned char layer5;
} VaaL;
VaaL servos;


/*************** RHD Variables *****************/
int kinematserv;
int errorserv;							//!< Error message - compare with
int servopark;							//!< parking og startinger
int servid; 							//!< Get Servo ID
int servpos; 							//!< Get Current position
int servmove; 						//!< Move
int servgoalpos; 					//!< position to change to.
int servmoves; 						//!< Moving true/false
int servspeed; 						//!< Current speed
int servgoalspeed; 				//!< final position
int PIDserv1; 						//!< Get parameters
int PIDserv2; 						//!< Get parameters
int PIDserv3; 						//!< Get parameters
int PIDserv4;			 				//!< Get parameters
int PIDserv5;							//!< Get parameters
int pparameter;								//!< Show parameters
int iparameter;								//!< Show parameters
int dparameter;								//!< Show parameters
int servload;						//!<current usage of max
int servBaud;					//!<servos.BaudRate for the communication
int servvolt;					//!<Voltage
int servtemp;				//!<Temperaute
/******** RHD Variable - thread sleep *************/
typedef struct
{ /** RHD variables for tSleep controller */
  int Cycles;
  pthread_mutex_t mLock; // pthread_mutex_lock
} VAL;
VAL tSleep;
/*********************NOT IN USE*******************/
//int servo_change_ID; 					//!< change Servo ID
//int change_BaudRate;					//!<servos.BaudRate for the communication
//int Torque_ENABLED;					//!<
//int max_LOAD;							//!<maximum load usage

/********************Enable change of write variables in RHD****************/
symTableElement * sservgoalpos = NULL, * sservo1 = NULL;
symTableElement * sservo2 = NULL, * sservo3 = NULL;
symTableElement * sservo4 = NULL, * sservo5 = NULL;
symTableElement * schange_SPEED = NULL, *   sservopark = NULL;
symTableElement * skinematserv = NULL;

/******** XML Variables *************/
///(loaded by XML)
char smrraDataDevString[64];
///(loaded by XML)
char smrraConfigDevString[64];

/******** Variables *************/
static double pi = 3.14159265358979;
static double L1 = 109;
static double L2 = 185;

/************ PThread definitions ************/
pthread_t smrra_thread;			//!<Main thread for smrra
pthread_attr_t attr;

/************ Functions ************/
int init_com(void);
void *SMRRobotArm_thread(void *);
void ARMposition(void);
void ARMspeed(void);
void ARMpid(void);
void ARMBaudrate(void);
void ARMload(void);
void ARMstart(void);
void ARMkin(void);
void errorMessage(int id);
void InverseKinematic2(double X, double Y, double * a1, double * a2);

/****************************** CODE ***************************/
int init_com(void){
    //************ Create database variables if all is ok ******************//
	errorserv = createVariable('r',5,"errorserv");								//!< error message compare with datasheet
	kinematserv = createVariable('w',2,"kinematserv");
	servopark = createVariable('w',1,"servopark"); 						//!< Servo start
	servid = createVariable('r',5,"servid"); 						//!< Get Servo ID
	servpos = createVariable('r',5,"servpos"); 						//!< Get Current position
	servgoalpos = createVariable('w',5,"servgoalpos"); 			//!< position to change to.
	servmoves = createVariable('r',5,"servmoves"); 				//!< Moving true/false
	servspeed = createVariable('r',5,"servspeed"); 					//!< Current speed
	servgoalspeed = createVariable('w',5,"servgoalspeed"); 	//!< final position
	PIDserv1 = createVariable('w',3,"PIDserv1"); 					//!< Get parameters
	PIDserv2 = createVariable('w',3,"PIDserv2"); 					//!< Get parameters
	PIDserv3 = createVariable('w',3,"PIDserv3"); 					//!< Get parameters
	PIDserv4 = createVariable('w',3,"PIDserv4"); 					//!< Get parameters
	PIDserv5 = createVariable('w',3,"PIDserv5"); 					//!< Get parameters
	pparameter = createVariable('r',5,"pparameter"); 								//!< Show parameters
	iparameter = createVariable('r',5,"iparameter"); 								//!< Show parameters
	dparameter = createVariable('r',5,"dparameter"); 								//!< Show parameters
	servload = createVariable('r',5,"servload");				//!<current usage of max
	servBaud = createVariable('r',1,"servBaud");		//!<servos.BaudRate for the communication
	servvolt = createVariable('r',5,"servvolt");			//!<Voltage
	servtemp = createVariable('r',5,"servtemp");	//!<Temperaute
	//*********************** pthread struct *****************************//

	pthread_mutex_init(&tSleep.mLock, NULL);
	pthread_mutex_lock(&tSleep.mLock);
	tSleep.Cycles = 0;
	//************************ NOT IN USE*********************************//
	//servo_change_ID = createVariable('w',5,"servo_change_ID"); 		//!< change Servo ID
	//change_BaudRate = createVariable('w',1,"change_BaudRate");		//!<servos.BaudRate for the communication
	//Torque_ENABLED = createVariable('w',5,"Torque_ENABLED");			//!<
	//max_LOAD = createVariable('w',5,"max_LOAD");						//!<maximum load usage

	//*************Initialization of Communication*************************//
	smrraRunning = init_connection(servos.BaudRate, servos.Speed, servos.DeviceIndex);

	//******************Extract symtableelements***************************//
	//********* Enables the possibility of changing write variables *******//
	symTableElement *wtab = getSymtable('w');
	int wtSize = getSymtableSize('w');
	int i;
	for(i = 0; i < wtSize; i++) {
		if (strncmp("servgoalpos",wtab[i].name,strlen("servgoalpos")) == 0) {
			sservgoalpos = wtab + i;
		} else if (strncmp("PIDserv1",wtab[i].name,strlen("PIDserv1")) == 0) {
			sservo1 = wtab + i;
		}
		else if (strncmp("PIDserv2",wtab[i].name,strlen("PIDserv2")) == 0){
		  sservo2 = wtab + i;
		}
		else if (strncmp("PIDserv3",wtab[i].name,strlen("PIDserv3")) == 0){
		  sservo3 = wtab + i;
		}
		else if (strncmp("PIDserv4",wtab[i].name,strlen("PIDserv4")) == 0){
		  sservo4 = wtab + i;
		}
		else if (strncmp("PIDserv5",wtab[i].name,strlen("PIDserv5")) == 0){
		  sservo5 = wtab + i;
		}else if (strncmp("servgoalspeed",wtab[i].name,strlen("servgoalspeed")) == 0){
		  schange_SPEED = wtab + i;
		}
		else if (strncmp("servopark",wtab[i].name,strlen("servopark")) == 0) {
					sservopark = wtab + i;
		}
		else if (strncmp("kinematserv",wtab[i].name,strlen("kinematserv")) == 0) {
					skinematserv = wtab + i;
		}
	}

	//**************Initialization and starting of threads**********************//
	if(smrraRunning == 1){
	  pthread_attr_init(&attr);
	  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
	  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	  if (pthread_create(&smrra_thread, &attr, SMRRobotArm_thread, 0) || servos.layer0 != 1 || servos.layer1 != 1 || servos.layer2 != 1 || servos.layer3 != 1 || servos.layer4 != 1 || servos.layer5 != 1)
	  {
		  perror("SMRRA: Can't start SMRRA thread");
		  return smrraRunning = -1;
	  }
	  smrraRunning = 1;
	  return 1;
	}else return smrraRunning = -1;
}

//**************Thread is the main program of the RHD-plugin**********************//
void *SMRRobotArm_thread(void *not_used){
    //TempVal
    int tempVal = 0;
    if(smrraRunning == 1){
		printf("SMR-Plugin Running\n");

/*	*	*	*	*	*	*	*	*	ReturnDelay	*	*	*	*	*	*	*	*	*	*	*/
		changeReturnDelayTime(254,servos.ReturnDelayTime);
/*	*	*	*	*	*	*	*	*	Kinematik	*	*	*	*	*	*	*	*	*	*	*/
		skinematserv->data[0] = (int32_t)L1+L2;
		skinematserv->data[1] = (int32_t)0;
		skinematserv->updated = 1;
		servos.servo1.kin = L1+L2;
		servos.servo3.kin = 0;
/*	*	*	*	*	*	*	*	*	*	I D	*	*	*	*	*	*	*	*	*	*	*	*/
		setVariable(servid,0,servos.servo1.id);
		setVariable(servid,1,servos.servo2.id);
		setVariable(servid,2,servos.servo3.id);
		setVariable(servid,3,servos.servo4.id);
		setVariable(servid,4,servos.servo5.id);
/*	*	*	*	*	*	*	*	P O S I T I O N	*	*	*	*	*	*	*	*	*	*	*/
		sservgoalpos->data[0] = (int32_t)servos.servo1.posStart;
		sservgoalpos->data[1] = (int32_t)servos.servo2.posStart;
		sservgoalpos->data[2] = (int32_t)servos.servo3.posStart;
		sservgoalpos->data[3] = (int32_t)servos.servo4.posStart;
		sservgoalpos->data[4] = (int32_t)servos.servo5.posStart;
		sservgoalpos->updated = 1;
		servos.start = 1;
		ARMstart();

/*	*	*	*	*	*	*	*	S P E E D and B A U D R A T E	*	*	*	*	*	*	*/
		setVariable(servBaud,0, servos.BaudRate);
		setVariable(servspeed,0,servos.Speed);
		setVariable(servspeed,1,servos.Speed);
		setVariable(servspeed,2,servos.Speed);
		setVariable(servspeed,3,servos.Speed);
		setVariable(servspeed,4,servos.Speed);

		schange_SPEED->data[0]= (int32_t)servos.Speed;
		schange_SPEED->data[1] = (int32_t)servos.Speed;
		schange_SPEED->data[2] = (int32_t)servos.Speed;
		schange_SPEED->data[3] = (int32_t)servos.Speed;
		schange_SPEED->data[4] = (int32_t)servos.Speed;
		schange_SPEED->updated = 1;
/*	*	*	*	*	*	*	*	C O N T R O L L E R	*	*	*	*	*	*	*	*	*	*/
		sservo1->data[0] = (int32_t)servos.servo1.pVal;
		sservo1->data[1] = (int32_t)servos.servo1.iVal;
		sservo1->data[2] = (int32_t)servos.servo1.dVal;
		sservo1->updated = 1;
		changeP(servos.servo1.id, servos.servo1.pVal);
		changeI(servos.servo1.id, servos.servo1.iVal);
		changeD(servos.servo1.id, servos.servo1.dVal);
		setVariable(pparameter,0,servos.servo1.pVal);
		setVariable(iparameter,0,servos.servo1.iVal);
		setVariable(dparameter,0,servos.servo1.dVal);

		sservo2->data[0] = (int32_t)servos.servo2.pVal;
		sservo2->data[1] = (int32_t)servos.servo2.iVal;
		sservo2->data[2] = (int32_t)servos.servo2.dVal;
		sservo2->updated = 1;
		changeP(servos.servo2.id, servos.servo2.pVal);
		changeI(servos.servo2.id, servos.servo2.iVal);
		changeD(servos.servo2.id, servos.servo2.dVal);
		setVariable(pparameter,1,servos.servo2.pVal);
		setVariable(iparameter,1,servos.servo2.iVal);
		setVariable(dparameter,1,servos.servo2.dVal);

		sservo3->data[0] = (int32_t)servos.servo3.pVal;
		sservo3->data[1] = (int32_t)servos.servo3.iVal;
		sservo3->data[2] = (int32_t)servos.servo3.dVal;
		sservo3->updated = 1;
		changeP(servos.servo3.id, servos.servo3.pVal);
		changeI(servos.servo3.id, servos.servo3.iVal);
		changeD(servos.servo3.id, servos.servo3.dVal);
		setVariable(pparameter,2,servos.servo3.pVal);
		setVariable(iparameter,2,servos.servo3.iVal);
		setVariable(dparameter,2,servos.servo3.dVal);

		sservo4->data[0] = (int32_t)servos.servo4.pVal;
		sservo4->data[1] = (int32_t)servos.servo4.iVal;
		sservo4->data[2] = (int32_t)servos.servo4.dVal;
		sservo4->updated = 1;
		changeP(servos.servo4.id, servos.servo4.pVal);
		changeI(servos.servo4.id, servos.servo4.iVal);
		changeD(servos.servo4.id, servos.servo4.dVal);
		setVariable(pparameter,3,servos.servo4.pVal);
		setVariable(iparameter,3,servos.servo4.iVal);
		setVariable(dparameter,3,servos.servo4.dVal);

		sservo5->data[0] = (int32_t)servos.servo5.pVal;
		sservo5->data[1] = (int32_t)servos.servo5.iVal;
		sservo5->data[2] = (int32_t)servos.servo5.dVal;
		sservo5->updated = 1;
		changeP(servos.servo5.id, servos.servo5.pVal);
		changeI(servos.servo5.id, servos.servo5.iVal);
		changeD(servos.servo5.id, servos.servo5.dVal);
		setVariable(pparameter,4,servos.servo5.pVal);
		setVariable(iparameter,4,servos.servo5.iVal);
		setVariable(dparameter,4,servos.servo5.dVal);
/*	*	*	*	*	*	*	*	L O A D	*	*	*	*	*	*	*	*	*	*	*	*	*/
		setVariable(servload,0,0);
		setVariable(servload,1,0);
		setVariable(servload,2,0);
		setVariable(servload,3,0);
		setVariable(servload,4,0);
/*	*	*	*	*	*	*	*	R E S E T I N G	*	*	*	*	*	*	*	*	*	*	*/
//All variables that have anything to do with write is set to 0, because of RHD.
		/*servos.start = 0;
		servos.servo1.pVal = 0;
		servos.servo1.iVal = 0;
		servos.servo1.posMOVEto = 0;
		servos.servo1.speedMOVE = 0;
		servos.servo2.pVal = 0;
		servos.servo2.iVal = 0;
		servos.servo1.dVal = 0;
		servos.servo2.dVal = 0;
		servos.servo2.posMOVEto = 0;
		servos.servo2.speedMOVE = 0;
		servos.servo3.pVal = 0;
		servos.servo3.iVal = 0;
		servos.servo3.dVal = 0;
		servos.servo3.posMOVEto = 0;
		servos.servo3.speedMOVE = 0;
		servos.servo4.pVal = 0;
		servos.servo4.iVal = 0;
		servos.servo4.dVal = 0;
		servos.servo4.posMOVEto = 0;
		servos.servo4.speedMOVE = 0;
		servos.servo5.pVal = 0;
		servos.servo5.iVal = 0;
		servos.servo5.dVal = 0;
		servos.servo5.posMOVEto = 0;
		servos.servo5.speedMOVE = 0;*/

    }else{
	printf("SMR-Plugin error\n");
    }

    printf("version 30\n\n\n");
    while(smrraRunning){

        pthread_mutex_lock(&tSleep.mLock);

		if (tSleep.Cycles >= 7){ //!1 equals ca. 0.01s
		  //printf("id = %d",servos.servo1.id);
		  tempVal = getPressentVoltage(servos.servo1.id);
		  setVariable(servvolt,0,tempVal);
		  tempVal = getPressentTemperature(servos.servo1.id);
		  setVariable(servtemp,0,tempVal);
		  tempVal = getMoving(servos.servo1.id);
		  setVariable(servmoves,0,tempVal);
		  errorMessage(0);

		  //printf("id = %d",servos.servo2.id);
		  tempVal = getPressentVoltage(servos.servo2.id);
		  setVariable(servvolt,1,tempVal);
		  tempVal = getPressentTemperature(servos.servo2.id);
		  setVariable(servtemp,1,tempVal);
		  tempVal = getMoving(servos.servo2.id);
		  setVariable(servmoves,1,tempVal);
		  errorMessage(1);

		  //printf("id = %d",servos.servo3.id);
		  tempVal = getPressentVoltage(servos.servo3.id);
		  setVariable(servvolt,2,tempVal);
		  tempVal = getPressentTemperature(servos.servo3.id);
		  setVariable(servtemp,2,tempVal);
		  tempVal = getMoving(servos.servo3.id);
		  setVariable(servmoves,2,tempVal);
		  errorMessage(2);

		  //printf("id = %d",servos.servo4.id);
		  tempVal = getPressentVoltage(servos.servo4.id);
		  setVariable(servvolt,3,tempVal);
		  tempVal = getPressentTemperature(servos.servo4.id);
		  setVariable(servtemp,3,tempVal);
		  tempVal = getMoving(servos.servo4.id);
		  setVariable(servmoves,3,tempVal);
		  errorMessage(3);

		  /*printf("id = %d",servos.servo5.id);//*/
		  tempVal = getPressentVoltage(servos.servo5.id);
		  setVariable(servvolt,4,tempVal);
		  tempVal = getPressentTemperature(servos.servo5.id);
		  setVariable(servtemp,4,tempVal);
		  tempVal = getMoving(servos.servo5.id);
		  setVariable(servmoves,4,tempVal);
		  errorMessage(4);


		  ARMload();
		  tSleep.Cycles = 0;
		 }
		if(getWriteVariable(servopark,0)==1){
			ARMposition();
			servos.start = 1;
		}
		else if(getWriteVariable(servopark,0)==2)
		{
			ARMkin();
			servos.start = 2;
		}else{
			ARMstart();
		}
		ARMspeed();
		ARMpid();
    }
    pthread_exit(NULL);
    exit(0);
}


extern int periodic(int tick)
{
	pthread_mutex_unlock(&tSleep.mLock);
	tSleep.Cycles++;
	return 1;
}
void errorMessage(int id){
	  //get error message
	  int tempVal = 0;
		if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
	  		tempVal+=1;

	  	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
	  		tempVal+=2;

	  	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
	  		tempVal +=4;

	  	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
	  		tempVal +=8;

	  	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
	  		tempVal +=16;

	  	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
	  		tempVal +=32;

	  	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
	  		tempVal += 64;
	  	setVariable(errorserv,id,tempVal);

}

void ARMkin(){
	/*
	 * This function is made on the grounds of Forward Kinematics, and with the limitations of only utilizing 2 servos. Which implies only moving on
	 * one axis. The coordinate system has the x axis pointing forward from the robot, and z axis perpendicular to the actuator.
	 * The axis that is rotated about, is the before mentioned z-axis.
	 */

	double a1,a2;

		//InverseKinematic1(X,Y,&a1,&a2);
	if(servos.servo1.kin != getWriteVariable(kinematserv,0) && servos.servo3.kin != getWriteVariable(kinematserv,1) ){
			InverseKinematic2(getWriteVariable(kinematserv,0),getWriteVariable(kinematserv,1),&a1,&a2);
		if(a1 >= 0){
			servos.servo1.posMOVEto = 1060 - a1*10;
		}else{
			servos.servo1.posMOVEto = 1060 + a1*10;
		}
		if(a2 >= 0){
			servos.servo3.posMOVEto = 1060 + a2*10;
		}else{
			servos.servo3.posMOVEto = 1060 - a2*10;
		}
		changePosL(servos.servo1.id, servos.servo1.posMOVEto);
		changePosL(servos.servo2.id, 900);
		changePosL(servos.servo3.id, servos.servo3.posMOVEto);
		servos.servo1.kin = getWriteVariable(kinematserv,0);
		servos.servo3.kin = getWriteVariable(kinematserv,1);
	}
}

void ARMstart()//start og slut pos
{
	int tempVal1;
	tempVal1 = getWriteVariable(servopark,0);
	if(servos.start != tempVal1)
	{

		printf("SKIFTER POSITION     tempVAL = %d\n",tempVal1);
			//park
		if(tempVal1 == 0)
		{
			changePosL(servos.servo1.id, servos.servo1.posPark+300);
			changePosL(servos.servo2.id, servos.servo2.posPark);
			changePosL(servos.servo3.id, servos.servo3.posPark+300);
			changePosL(servos.servo4.id, servos.servo4.posPark);
			changePosL(servos.servo5.id, servos.servo5.posPark);
			servos.servo5.posMOVEto = servos.servo5.posPark;

			#ifdef DEBUG
			printf("servo update start = %d\n\n",sservgoalpos->updated);

			printf("parkpos 1 = %d\n",servos.servo1.posPark);

			printf("parkpos 2 = %d\n",servos.servo2.posPark);

			printf("parkpos 3 = %d\n",servos.servo3.posPark);

			printf("parkpos 4 = %d\n",servos.servo4.posPark);

			printf("parkpos 5 = %d\n",servos.servo5.posPark);
			#endif

			while(((getPos(servos.servo2.id) < servos.servo2.posPark-20) || (getPos(servos.servo2.id) > servos.servo2.posPark+20)) || ((getPos(servos.servo4.id) < servos.servo4.posPark-20) || (getPos(servos.servo4.id) > servos.servo4.posPark+20)) || ((getPos(servos.servo5.id) < servos.servo5.posPark-20) || (getPos(servos.servo5.id) > servos.servo5.posPark+20))){
				#ifndef DEBUG
		        pthread_mutex_lock(&tSleep.mLock);
				#endif
				#ifdef DEBUG
				printf("pos2 %d     ",getPos(servos.servo2.id));
				printf("pos4 %d     ",getPos(servos.servo4.id));
				printf("pos5 %d\n",getPos(servos.servo5.id));
				#endif
			}
			changePosL(servos.servo1.id, servos.servo1.posPark);
			changePosL(servos.servo3.id, servos.servo3.posPark);
			servos.start = 0;
			setVariable(servpos,0,servos.servo1.posPark);
			setVariable(servpos,1,servos.servo2.posPark);
			setVariable(servpos,2,servos.servo3.posPark);
			setVariable(servpos,3,servos.servo4.posPark);
			setVariable(servpos,4,servos.servo5.posPark);
		}
		//start
		if(tempVal1 == 1)
		{
			changePosL(servos.servo1.id,servos.servo1.posPark+300);
			changePosL(servos.servo2.id,servos.servo2.posPark+300);

			while(((getPos(servos.servo1.posPark+300) <  servos.servo1.posPark+280) || (getPos(servos.servo1.posPark+300) >  servos.servo1.posPark+320)) || ((getPos(servos.servo3.posPark+300) <  servos.servo3.posPark+280) || (getPos(servos.servo3.posPark+300) >  servos.servo3.posPark+320))){
				#ifndef DEBUG
		        pthread_mutex_lock(&tSleep.mLock);
				#endif
		        #ifdef DEBUG
				printf("pos1 %d     ",getPos(servos.servo1.id));
				printf("pos3 %d\n",getPos(servos.servo3.id));
				#endif
			}

			changePosL(servos.servo1.id, getWriteVariable(servgoalpos,0));
			changePosL(servos.servo2.id, getWriteVariable(servgoalpos,1));
			changePosL(servos.servo3.id, getWriteVariable(servgoalpos,2));
			changePosL(servos.servo4.id, getWriteVariable(servgoalpos,3));
			changePosL(servos.servo5.id, getWriteVariable(servgoalpos,4));

			servos.start =1 ;

		}
	}
}

void InverseKinematic2(double X, double Y, double * a1, double * a2)
{
	double c2 = (X*X+Y*Y-L1*L1-L2*L2)/(2*L1*L2);
	double s2 = sqrt(1-c2*c2);
	double k1 = L1+L2*c2;
	double k2 = L2*s2;

	double a1r = atan2(X,Y)-atan2(k1,k2);
	double a2r = atan2(c2,s2)-pi/2;
	*a1 = a1r * 180.0/pi;
	*a2 = a2r * 180.0/pi;
	printf("   X     Y        a1   a2     a1r     a2r\n");
	printf(" %.2f  %.2f  %.3f  %.3f  %.3f  %.3f\n",X,Y,*a1,*a2,a1r,a2r);
}


void ARMposition()
{

	int tempVal;
	tempVal = getWriteVariable(servgoalpos,0);
	//printf("posMOVEto %d ----  tempVal1 %d\n", servos.servo1.posMOVEto,tempVal);
	if (((servos.servo1.posMOVEto != tempVal) & (tempVal <= servos.servo1.posMAX)) & (tempVal >= servos.servo1.posMIN) ){

		changePosL(servos.servo1.id, tempVal);
		setVariable(servmoves,0,getMoving(servos.servo1.id));
		servos.servo1.posMOVEto = tempVal;
		printf("tempVal");
	}	
	tempVal = getWriteVariable(servgoalpos,1);
	//printf("posMOVEto %d ----  tempVal1 %d\n", servos.servo2.posMOVEto,tempVal);
	if (((servos.servo2.posMOVEto != tempVal) & (tempVal <= servos.servo2.posMAX)) & (tempVal >= servos.servo2.posMIN)){
		changePosL(servos.servo2.id, tempVal);
		setVariable(servmoves,1,getMoving(servos.servo2.id));
		servos.servo2.posMOVEto = tempVal;
	}
	tempVal = getWriteVariable(servgoalpos,2);
	//printf("posMOVEto %d ----  tempVal1 %d\n", servos.servo3.posMOVEto,tempVal);
	if (((servos.servo3.posMOVEto != tempVal) & (tempVal <= servos.servo3.posMAX)) & (tempVal >= servos.servo3.posMIN)){
		
		changePosL(servos.servo3.id, tempVal);
		setVariable(servmoves,2,getMoving(servos.servo3.id));
		servos.servo3.posMOVEto = tempVal;
	}
	tempVal = getWriteVariable(servgoalpos,3);
	//printf("posMOVEto %d ----  tempVal1 %d\n", servos.servo4.posMOVEto,tempVal);
	if (((servos.servo4.posMOVEto != tempVal) & (tempVal<= servos.servo4.posMAX)) & (tempVal >= servos.servo4.posMIN)){
		
		changePosL(servos.servo4.id, tempVal);
		setVariable(servmoves,3,getMoving(servos.servo4.id));
		servos.servo4.posMOVEto = tempVal;
	}
	tempVal = getWriteVariable(servgoalpos,4);
	//printf("posMOVEto %d ----  tempVal1 %d\n", servos.servo5.posMOVEto,tempVal);
	if (((servos.servo5.posMOVEto != tempVal) & (tempVal <= servos.servo5.posMAX)) & (tempVal >= servos.servo5.posMIN)){
		changePosL(servos.servo5.id, tempVal);
		setVariable(servmoves,4,getMoving(servos.servo5.id));
		servos.servo5.posMOVEto = tempVal;
	}

	//current position
	tempVal = getWriteVariable(servgoalpos,0);
	if (getReadVariable(servpos,0) != tempVal){// && getReadVariable(servmoves,0)==1){

		setVariable(servpos,0,getPos(servos.servo1.id));
		//setVariable(servmoves,0,getMoving(servos.servo1.id));
	}	
	tempVal = getWriteVariable(servgoalpos,1);
	if (getReadVariable(servpos,1)  != tempVal && getReadVariable(servmoves,1)==1){
		setVariable(servpos,1,getPos(servos.servo2.id));
		//setVariable(servmoves,1,getMoving(servos.servo2.id));
	}
	tempVal = getWriteVariable(servgoalpos,2);
	if (getReadVariable(servpos,2) != tempVal && getReadVariable(servmoves,2)==1){
		
		setVariable(servpos,2,getPos(servos.servo3.id));
		//setVariable(servmoves,2,getMoving(servos.servo3.id));
	}
	tempVal = getWriteVariable(servgoalpos,3);
	if (getReadVariable(servpos,3) != tempVal && getReadVariable(servmoves,3)==1){
		
		setVariable(servpos,3,getPos(servos.servo4.id));
		//setVariable(servmoves,3,getMoving(servos.servo4.id));
	}
	tempVal = getWriteVariable(servgoalpos,4);
	if (getReadVariable(servpos,4) != tempVal && getReadVariable(servmoves,4)==1){
		//tempVal = getPos(servos.servo5.id);
		setVariable(servpos,4,getPos(servos.servo5.id));
		
		//setVariable(servmoves,4,getMoving(servos.servo5.id));
	}
}

void ARMspeed()
{
	int tempVal;
	tempVal = getWriteVariable(servgoalspeed,0);
	if (servos.servo1.speedMOVE != tempVal){
		setMoveSpeed(servos.servo1.id, tempVal);
		servos.servo1.speedMOVE = tempVal;
		setVariable(servspeed,0,tempVal);
	}	
	tempVal = getWriteVariable(servgoalspeed,1);
	if (servos.servo2.speedMOVE != tempVal){
		setMoveSpeed(servos.servo2.id, tempVal);
		servos.servo2.speedMOVE = tempVal;
	}
	tempVal = getWriteVariable(servgoalspeed,2);
	if (servos.servo3.speedMOVE != tempVal){
		setMoveSpeed(servos.servo3.id, tempVal);
		servos.servo3.speedMOVE = tempVal;
	}
	tempVal = getWriteVariable(servgoalspeed,3);
	if (servos.servo4.speedMOVE != tempVal){
		setMoveSpeed(servos.servo4.id, tempVal);
		servos.servo4.speedMOVE = tempVal;
	}
	tempVal = getWriteVariable(servgoalspeed,4);
	if (servos.servo5.speedMOVE != tempVal){
		setMoveSpeed(servos.servo5.id, tempVal);
		servos.servo5.speedMOVE = tempVal;

	}
	//current speed
	tempVal = getWriteVariable(servgoalspeed,0);
	if (getReadVariable(servspeed,0) != tempVal && getReadVariable(servmoves,0)==1){
		setVariable(servspeed,0,getPressentSpeed(servos.servo1.id));
	}	
	tempVal = getWriteVariable(servgoalspeed,1);
	if (getReadVariable(servspeed,1)  != tempVal && getReadVariable(servmoves,1)==1){
		setVariable(servspeed,1,getPressentSpeed(servos.servo2.id));
	}
	tempVal = getWriteVariable(servgoalspeed,2);
	if (getReadVariable(servspeed,2) != tempVal && getReadVariable(servmoves,2)==1){
		setVariable(servspeed,2,getPressentSpeed(servos.servo3.id));
	}
	tempVal = getWriteVariable(servgoalspeed,3);
	if (getReadVariable(servspeed,3) != tempVal && getReadVariable(servmoves,3)==1){
		setVariable(servspeed,3,getPressentSpeed(servos.servo4.id));
	}
	tempVal = getWriteVariable(servgoalspeed,4);
	if (getReadVariable(servspeed,4) != tempVal && getReadVariable(servmoves,4)==1){
		setVariable(servspeed,4,getPressentSpeed(servos.servo5.id));
	}
}

void ARMpid()
{

	//servo1
	int tempVal;
	tempVal = getWriteVariable(PIDserv1,0);
	if ((servos.servo1.pVal != tempVal) & (tempVal != 0)){
		changeP(servos.servo1.id, tempVal);
		setVariable(pparameter,0,tempVal);
		servos.servo1.pVal = tempVal;
	}
	tempVal = getWriteVariable(PIDserv1,1);
	if ((servos.servo1.iVal != tempVal) & (tempVal != 0)){
		changeI(servos.servo1.id, tempVal);
		setVariable(iparameter,0,tempVal);
		servos.servo1.iVal = tempVal;
	}
	tempVal = getWriteVariable(PIDserv1,2);
	if ((servos.servo1.dVal != tempVal) & (tempVal != 0)){
		setVariable(dparameter,0,tempVal);
		changeD(servos.servo1.id, tempVal);
		servos.servo1.dVal = tempVal;
	}
	//servo2
	tempVal = getWriteVariable(PIDserv2,0);
	if ((servos.servo2.pVal != tempVal) & (tempVal != 0)){
		changeP(servos.servo2.id, tempVal);
		setVariable(pparameter,1,tempVal);
		servos.servo2.pVal = tempVal;
	}
	tempVal = getWriteVariable(PIDserv2,1);
	if ((servos.servo2.iVal != tempVal) & (tempVal != 0)){
		changeI(servos.servo2.id, tempVal);
		setVariable(iparameter,1,tempVal);
		servos.servo2.iVal = tempVal;
	}
	tempVal = getWriteVariable(PIDserv2,2);
	if ((servos.servo2.dVal != tempVal) & (tempVal != 0)){
		changeD(servos.servo2.id, tempVal);
		setVariable(dparameter,1,tempVal);
		servos.servo2.dVal = tempVal;
	}
	//servo3
	tempVal = getWriteVariable(PIDserv3,0);
	if ((servos.servo3.pVal != tempVal) & (tempVal != 0)){
		changeP(servos.servo3.id, tempVal);
		setVariable(pparameter,2,tempVal);
		servos.servo3.pVal = tempVal;
	}
	tempVal = getWriteVariable(PIDserv3,1);
	if ((servos.servo3.iVal != tempVal) & (tempVal != 0)){
		changeP(servos.servo3.id, tempVal);
		setVariable(iparameter,2,tempVal);
		servos.servo3.iVal = tempVal;
	}
	tempVal = getWriteVariable(PIDserv3,2);
	if ((servos.servo3.dVal != tempVal) & (tempVal != 0)){
		changeD(servos.servo3.id, tempVal);
		setVariable(dparameter,2,tempVal);
		servos.servo3.dVal = tempVal;
	}
	//servo4
	tempVal = getWriteVariable(PIDserv4,0);
	if ((servos.servo4.pVal != tempVal) & (tempVal != 0)){
		changeP(servos.servo4.id, tempVal);
		setVariable(pparameter,3,tempVal);
		servos.servo4.pVal = tempVal;
	}
	tempVal = getWriteVariable(PIDserv4,1);
	if ((servos.servo4.iVal != tempVal) & (tempVal != 0)){
		changeI(servos.servo4.id, tempVal);
		setVariable(iparameter,3,tempVal);
		servos.servo4.iVal = tempVal;
	}
	tempVal = getWriteVariable(PIDserv4,2);
	if ((servos.servo4.dVal != tempVal) & (tempVal != 0)){
		changeD(servos.servo4.id, tempVal);
		setVariable(dparameter,3,tempVal);
		servos.servo4.dVal = tempVal;
	}
	//servo5
	tempVal = getWriteVariable(PIDserv5,0);
	if ((servos.servo5.pVal != tempVal) & (tempVal != 0)){
		changeP(servos.servo5.id, tempVal);
		setVariable(pparameter,4,tempVal);
		servos.servo5.pVal = tempVal;
	}
	tempVal = getWriteVariable(PIDserv5,1);
	if ((servos.servo5.iVal != tempVal) & (tempVal != 0)){
		changeI(servos.servo5.id, tempVal);
		setVariable(iparameter,4,tempVal);
		servos.servo5.iVal = tempVal;
	}
	tempVal = getWriteVariable(PIDserv5,2);
	if ((servos.servo5.dVal != tempVal) & (tempVal != 0)){
		changeD(servos.servo5.id, tempVal);
		setVariable(iparameter,4,tempVal);
		servos.servo5.dVal = tempVal;
	}
}
//This function should not be used, Since it can create problems with communication.
//Change baudrate individually in configuration file instead.
/*void ARMBaudrate()
{
	int tempVal;
	tempVal = getWriteVariable(change_BaudRate,0);
	if (servos.BaudRate != tempVal){
		changeBaudRate(254, tempVal);
		setVariable(servBaud,0,tempVal);
		servos.BaudRate = tempVal;
	}
}*/

void ARMload()
{
	int tempVal;
  int load;
  int si;
  servoDEF * servo[5] = {&servos.servo1, &servos.servo2, &servos.servo3, &servos.servo4, &servos.servo5};
  for (si = 0; si < servos.servosCnt; si++)
  {
    load = getLoad(servo[si]->id);
    tempVal = labs(load);
    setVariable(servload, 0, tempVal);
    servos.servo1.loadCURRENT = tempVal;
    if(((tempVal >= servo[si]->loadCW) & (tempVal < 100)) || (tempVal >= servo[si]->loadCCW))
    {
      tempVal = getPos(servo[si]->id);
      if(load < 0){
        sservgoalpos->data[0] = (int32_t)tempVal-20;
      }else
        sservgoalpos->data[0] = (int32_t)tempVal+20;
          sservgoalpos->updated = 1;
    }
  }
// 	tempVal = (int *) getLoad(servos.servo2.id);
// 	setVariable(servload,1,tempVal);
// 	servos.servo2.loadCURRENT = tempVal;
// 	if(((tempVal >= servos.servo2.loadCW) & (tempVal < 100)) || (tempVal >= servos.servo2.loadCCW) )
// 	{
// 		tempVal = getPos(servos.servo2.id);
// 		if(load < 0){
// 			sservgoalpos->data[1] = (int32_t)tempVal+20;
// 		}else
// 			sservgoalpos->data[1] = (int32_t)tempVal-20;
//         sservgoalpos->updated = 1;
// 	}
// 	tempVal = (int *) getLoad(servos.servo3.id);
// 	setVariable(servload,2,tempVal[0]);
// 	servos.servo3.loadCURRENT = tempVal[0];
// 	if(((tempVal[0] >= servos.servo3.loadCW) & (tempVal[0] < 100)) || (tempVal[0] >= servos.servo3.loadCCW ))
// 	{
// 		tempVal[0] = getPos(servos.servo3.id);
// 		if(tempVal[1] == 1){
// 			sservgoalpos->data[2] = (int32_t)tempVal[0]-20;
// 		}else
// 			sservgoalpos->data[2] = (int32_t)tempVal[0]+20;
//         sservgoalpos->updated = 1;
// 	}
// 	tempVal = (int *) getLoad(servos.servo4.id);
// 	setVariable(servload,3,tempVal[0]);
// 	servos.servo4.loadCURRENT = tempVal[0];
// 	if(((tempVal[0] >= servos.servo4.loadCW) & (tempVal[0] < 100)) || (tempVal[0] >= servos.servo4.loadCCW) )
// 	{
// 		tempVal[0] = getPos(servos.servo4.id);
// 		if(tempVal[1] == 1){
// 			sservgoalpos->data[3] = (int32_t)tempVal[0]+20;
// 		}else
// 			sservgoalpos->data[3] = (int32_t)tempVal[0]-20;
//         sservgoalpos->updated = 1;
// 	}
// 	tempVal = (int *) getLoad(servos.servo5.id);
// 	setVariable(servload,4,tempVal[0]);
// 	servos.servo5.loadCURRENT = tempVal[0];
// 	if(((tempVal[0] >= servos.servo5.loadCW) & (tempVal[0] < 100)) || (tempVal[0] >= servos.servo5.loadCCW) )
// 	{
// 		tempVal[0] = getPos(servos.servo5.id);
// 		if(tempVal[1] == 1){
// 			sservgoalpos->data[4] = (int32_t)tempVal[0]+20;
// 		}else
// 			sservgoalpos->data[4] = (int32_t)tempVal[0]-20;
//         sservgoalpos->updated = 1;
// 	}
}

/************************** XML Initialization **************************/
///Struct for shared parse data
typedef struct  {
    int depth;
    char skip;
    char enable;
		char found;
  }parseInfo;

//Parsing functions
void XMLCALL smrraStartTag(void *, const char *, const char **);
void XMLCALL smrraEndTag(void *, const char *);

/** \brief Initialize the SMRRA with settings from configuration file.
 *
 * Reads the XML file and sets up the SMRRA settings
 *
 * Finally the initialization of the SMRRA plugin is started.
 *
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
extern int initXML(char *filename) {

  parseInfo xmlParse;
  char *xmlBuf = NULL;
	int xmlFilelength;
  int done = 0;
  int len;
  FILE *fp;
  servos.servosCnt = 0;
  //setting all values to 0
  memset(&servos,0,sizeof(servos));

  printf("initXML function\n");

     /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "SMRRA: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, smrraStartTag, smrraEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("SMRRA: Error reading: %s\n",filename);
    return -1;
  }
  //Get the length of the file
	fseek(fp,0,SEEK_END);
	xmlFilelength = ftell(fp); //Get position
	fseek(fp,0,SEEK_SET); //Return to start of file
	//Allocate text buffer
	xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
	if (xmlBuf == NULL) {
		fprintf(stderr, "   Couldn't allocate memory for XML File buffer\n");
		return -1;
	}
	memset(xmlBuf,0,xmlFilelength);
  len = fread(xmlBuf, 1, xmlFilelength, fp);
  fclose(fp);

  //Start parsing the XML file
  if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) {
    fprintf(stderr, "SMRRA: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <servos> XML tag found in plugins section\n");
		return -1;
	}

  //Start smrra thread after init
  if (xmlParse.enable) done = init_com();



 return done;
}
///Handle XML Start tags
void XMLCALL 
smrraStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;
  //printf("Depth = %d Skip = %d el=%s \n",info->depth,info->skip,el);//Debug Message
  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("sensors",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  }else {
	  return;
  }
  //Branch to parse the elements of the XML file.
  if (!strcmp("sensors",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=3) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
         

      info->enable = 1;
    }
    if (!info->enable) {
      printf("   smrra: Use of smrra disabled in configuration, %s\n",el);
      info->skip = info->depth;
    }
  } else if (strcmp("serial",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
  } else if (strcmp("servo",el) == 0) 
    {
      for(i = 0; attr[i]; i+=8) {
		  if((strcmp("BaudRateVal",attr[i]) == 0)){
			  servos.layer0 = 1;
			  servos.BaudRate = atoi(attr[i+1]);
			  servos.Speed =  atoi(attr[i+3]);
			  servos.DeviceIndex = atoi(attr[i+5]);
			  servos.ReturnDelayTime = atoi(attr[i+7]);
			  #ifdef DEBUG
				printf("baudrate = %d\n",servos.BaudRate);
				printf("speed = %d\n",servos.Speed);
				printf("DeviceIndex = %d\n",servos.DeviceIndex);
				printf("ReturnDelayTime = %d\n",servos.ReturnDelayTime);
			  #endif
		  }
      //info->enable = 1;
    }
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
  }  else if (strcmp("servo1Val",el) == 0)
    {
      for(i = 0; attr[i]; i+=20) {
      if((strcmp("servo1",attr[i]) == 0)){
    	  servos.layer1 = 1;
    	  servos.servo1.id = atoi(attr[i+1]);
    	  servos.servo1.pVal = atoi(attr[i+3]);
    	  servos.servo1.iVal = atoi(attr[i+5]);
    	  servos.servo1.dVal = atoi(attr[i+7]);
    	  servos.servo1.posMIN = atoi(attr[i+9]);
    	  servos.servo1.posMAX = atoi(attr[i+11]);
		  servos.servo1.posStart = atoi(attr[i+13]);
		  servos.servo1.posPark = atoi(attr[i+15]);
		  servos.servo1.loadCW = atoi(attr[i+17]);
		  servos.servo1.loadCCW = atoi(attr[i+19]);
      servos.servosCnt = 1;
		  #ifdef DEBUG
			printf("servo1\n");
			printf("id %d\n",servos.servo1.id);
			printf("p %d\n",servos.servo1.pVal);
			printf("i %d\n",servos.servo1.iVal);
			printf("d %d\n",servos.servo1.dVal);
			printf("posMIN %d\n",servos.servo1.posMIN);
			printf("posMax %d\n",servos.servo1.posMAX);
			printf("startPos %d\n",servos.servo1.posStart);
			printf("parkPOS %d\n",servos.servo1.posPark);
			printf("LoadCW %d\n",servos.servo1.loadCW);
			printf("LoadCCW %d\n",servos.servo1.loadCCW);
		  #endif
    }
     // info->skip = 1;
    }
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
  } else if (strcmp("servo2Val",el) == 0)
    {
        for(i = 0; attr[i]; i+=20) {
        if((strcmp("servo2",attr[i]) == 0)){
			  servos.layer2 = 1;
			  servos.servo2.id = atoi(attr[i+1]);
			  servos.servo2.pVal = atoi(attr[i+3]);
			  servos.servo2.iVal = atoi(attr[i+5]);
			  servos.servo2.dVal = atoi(attr[i+7]);
			  servos.servo2.posMIN = atoi(attr[i+9]);
			  servos.servo2.posMAX = atoi(attr[i+11]);
			  servos.servo2.posStart = atoi(attr[i+13]);
			  servos.servo2.posPark = atoi(attr[i+15]);
			  servos.servo2.loadCW = atoi(attr[i+17]);
			  servos.servo2.loadCCW = atoi(attr[i+19]);
        servos.servosCnt = 2;
			  #ifdef DEBUG
				printf("servo2\n");
				printf("id %d\n",servos.servo2.id);
				printf("p %d\n",servos.servo2.pVal);
				printf("i %d\n",servos.servo2.iVal);
				printf("d %d\n",servos.servo2.dVal);
				printf("posMIN %d\n",servos.servo2.posMIN);
				printf("posMax %d\n",servos.servo2.posMAX);
				printf("startPOS %d\n",servos.servo2.posStart);
				printf("ParkPOS %d\n",servos.servo2.posPark);
				printf("LoadCW %d\n",servos.servo2.loadCW);
				printf("LoadCCW %d\n",servos.servo2.loadCCW);
			  #endif
        }
        //info->skip = 1;
      }
      //Check for the correct depth for this tag
      if(info->depth != 4) {
        printf("Error: Wrong depth for the %s tag\n",el);
      }
    }
  else if (strcmp("servo3Val",el) == 0)
      {
        for(i = 0; attr[i]; i+=18) {
		if((strcmp("servo3",attr[i]) == 0)){
			  servos.layer3 = 1;
			  servos.servo3.id = atoi(attr[i+1]);
			  servos.servo3.pVal = atoi(attr[i+3]);
			  servos.servo3.iVal = atoi(attr[i+5]);
			  servos.servo3.dVal = atoi(attr[i+7]);
			  servos.servo3.posMIN = atoi(attr[i+9]);
			  servos.servo3.posMAX = atoi(attr[i+11]);
			  servos.servo3.posStart = atoi(attr[i+13]);
			  servos.servo3.posPark = atoi(attr[i+15]);
			  servos.servo3.loadCW = atoi(attr[i+17]);
			  servos.servo3.loadCCW = atoi(attr[i+19]);
      servos.servosCnt = 3;
			  #ifdef DEBUG
				printf("servo3\n");
				printf("id %d\n",servos.servo3.id);
				printf("p %d\n",servos.servo3.pVal);
				printf("i %d\n",servos.servo3.iVal);
				printf("d %d\n",servos.servo3.dVal);
				printf("posMIN %d\n",servos.servo3.posMIN);
				printf("posMax %d\n",servos.servo3.posMAX);
				printf("startPOS %d\n",servos.servo3.posStart);
				printf("parkPOS %d\n",servos.servo3.posPark);
				printf("LoadCW %d\n",servos.servo3.loadCW);
				printf("LoadCCW %d\n",servos.servo3.loadCCW);
			  #endif
		}
        //info->skip = 1;
      }
      //Check for the correct depth for this tag
      if(info->depth != 4) {
        printf("Error: Wrong depth for the %s tag\n",el);
      }
    }
  else if (strcmp("servo4Val",el) == 0)
      {
        for(i = 0; attr[i]; i+=20) {
		if((strcmp("servo4",attr[i]) == 0)){
			  servos.layer4 = 1;
			  servos.servo4.id = atoi(attr[i+1]);
			  servos.servo4.pVal = atoi(attr[i+3]);
			  servos.servo4.iVal = atoi(attr[i+5]);
			  servos.servo4.dVal = atoi(attr[i+7]);
			  servos.servo4.posMIN = atoi(attr[i+9]);
			  servos.servo4.posMAX = atoi(attr[i+11]);
			  servos.servo4.posStart = atoi(attr[i+13]);
			  servos.servo4.posPark = atoi(attr[i+15]);
			  servos.servo4.loadCW = atoi(attr[i+17]);
			  servos.servo4.loadCCW = atoi(attr[i+19]);
        servos.servosCnt = 4;
			  #ifdef DEBUG
				printf("servo4\n");
				printf("id %d\n",servos.servo4.id);
				printf("p %d\n",servos.servo4.pVal);
				printf("i %d\n",servos.servo4.iVal);
				printf("d %d\n",servos.servo4.dVal);
				printf("posMIN %d\n",servos.servo4.posMIN);
				printf("posMax %d\n",servos.servo4.posMAX);
				printf("startPOS %d\n",servos.servo4.posStart);
				printf("ParkPOS %d\n",servos.servo4.posPark);
				printf("LoadCW %d\n",servos.servo4.loadCW);
				printf("LoadCCW %d\n",servos.servo4.loadCCW);
			  #endif
		}
        //info->skip = 1;
      }
      //Check for the correct depth for this tag
      if(info->depth != 4) {
        printf("Error: Wrong depth for the %s tag\n",el);
      }
    }
  else if (strcmp("servo5Val",el) == 0)
      {
        for(i = 0; attr[i]; i+=20) {
    		if((strcmp("servo5",attr[i]) == 0)){
    			  servos.layer5 = 1;
    			  servos.servo5.id = atoi(attr[i+1]);
    			  servos.servo5.pVal = atoi(attr[i+3]);
    			  servos.servo5.iVal = atoi(attr[i+5]);
    			  servos.servo5.dVal = atoi(attr[i+7]);
    			  servos.servo5.posMIN = atoi(attr[i+9]);
    			  servos.servo5.posMAX = atoi(attr[i+11]);
    			  servos.servo5.posStart = atoi(attr[i+13]);
    			  servos.servo5.posPark = atoi(attr[i+15]);
    			  servos.servo5.loadCW = atoi(attr[i+17]);
    			  servos.servo5.loadCCW = atoi(attr[i+19]);
           servos.servosCnt = 5;
    			  #ifdef DEBUG
    				printf("servo5\n");
    				printf("id %d\n",servos.servo5.id);
    				printf("p %d\n",servos.servo5.pVal);
    				printf("i %d\n",servos.servo5.iVal);
    				printf("d %d\n",servos.servo5.dVal);
    				printf("posMIN %d\n",servos.servo5.posMIN);
    				printf("posMax %d\n",servos.servo5.posMAX);
    				printf("startPOS %d\n",servos.servo5.posStart);
    				printf("ParkPOS %d\n",servos.servo5.posPark);
    				printf("LoadCW %d\n",servos.servo5.loadCW);
    				printf("LoadCCW %d\n",servos.servo5.loadCCW);
    			  #endif
    		}
        info->skip = 1;
      }
      //Check for the correct depth for this tag
      if(info->depth != 4) {
        printf("Error: Wrong depth for the %s tag\n",el);
      }
    }
}

///Handle XML End tags
void XMLCALL
smrraEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
