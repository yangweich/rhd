/** \file hakocan.c
 *  \ingroup hwmodule
 *  \brief Hardware abstraction layer for HAKO Tractor can-bus control
 *
 *  This liberary inplements the hardware abstraction layer
 *  for communicating using CAN-BUS on the KU-Life
 *  HAKO Automated tractor
 * 
 *  Communication is adjusted directly for the instruction-set
 *  on the HAKO ECU and is based on the HAKO implemention
 *  project by Anders Reeske Nilsen and Asbjørn Mejnertsen in 2006
 *
 *  \author Nils A. Andersen & Anders Billesø Beck, DTU 
 *  $Rev: 252 $
 *  $Date: 2013-12-12 14:23:45 +0100 (Thu, 12 Dec 2013) $
 *  
 */
 /**************************************************************************
 *      Copyright 2008 DTU-Elektro, Automation and Control 
 *      Programmers: Nils A. Andersen 
 *		     Anders Billesø Beck, DTU         
 *                   anders.beck@get2net.dk                            
 *                                                                         
 ***************************************************************************/
/************************** Library version  ***************************/
#define HAKOCANVERSION 		"1.1030"
/************************** Version control information ***************************/
 #define REVISION         "$Rev: 252 $:"
 #define DATE             "$Date: 2013-12-12 14:23:45 +0100 (Thu, 12 Dec 2013) $:"
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
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <linux/serial.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <expat.h>

//CAN-Driver definitions, placed in include folder
#include <rhd.h>
#include <database.h>
#include <globalfunc.h>

#include "canmsg.h"
#include "hakocan.h"

/******** Global variables *************/
///Index pointers for the variable database
int iSteeringangleref,
iSteeringangle,iSpeedref,iEnginespeedref,iNavigationmoderef,icurvatureref,icurvature,
iNavigationmode,iOdopulses,iOdopulses1,iEnginespeed,iHakostate0,
iHakostate1,iHakostate2,iHakostate3,
iliftinggearpos,ipowertakeoffspeed,iliftinggearstateref,ipowertakeoffstateref,ihitchposref,icvtack,isteeringack,
ig01,ig02,iptor,iauxr,ifll,iflr,ihlght,irlght,ihorn,ihornack,iswitchack,size;

//PThread definitions
pthread_t canrx_thread, canrx_thread1;
pthread_attr_t attr;

///Number of CAN-messages to be send to CAN-bus in every cycle
#define CANTXBUFSIZE 4
///Number of CAN-messages to be received from the CAN-bus in every cycle
#define CANRXBUFSIZE 3
#define DFLTENGSP 50

#define BLOCK_MAX 200

// Creation of communication buffers 
///Receive buffer for CAN-messages
struct canmsg_t canrxbuf[CANRXBUFSIZE];
///Transmit buffer for CAN-messages
struct canmsg_t cantxbuf[CANTXBUFSIZE];
///CAN port pointer
int can_dev,can_dev1;
///CAN port device identification string
char canDevString[64];
char canDevString1[64];
///Flag to indicate if HakoCan is running or not
static volatile int hakocanRunning = -1;

//Function prototypes
int initHakocan(void);
void *canrx_task(void *);
void *canrx_task1(void *);

/** \brief Initialization of the CAN-bus and all rx/tx buffers
 *
 * All buffers are initilized and database variables are created.
 * Finally, the RX Thread is spawned
 *
 */
int initHakocan(void)
{
//struct canmsg_t msg;
//int ret;
//union {float a;char b[4];}conv;


// Open CAN port
	can_dev=open(canDevString,O_RDWR);
	if (can_dev<0)  
	{
		fprintf(stderr,"   HakoCan: Error CAN-BUS on %s\n",canDevString);
		return -1;
	}
	if (canDevString1[0]!=0)
	{
		can_dev1=open(canDevString1,O_RDWR);
		if (can_dev1<0)
		{
			fprintf(stderr,"   HakoCan: Error CAN-BUS on %s\n",canDevString1);
			return -1;
		}
		else
		{
			printf("   HakoCan: Ports %s and %s open\n",canDevString,canDevString1) ;
		}
	} 

	//Create CAN RX Thread
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	if (pthread_create(&canrx_thread, &attr, canrx_task, 0)) 
	{
		perror("   HakoCan: failed");
		return -1;
    }
	if (can_dev1)
	{
		pthread_attr_init(&attr);
		pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
		if (pthread_create(&canrx_thread1, &attr, canrx_task1, 0)) 
		{
			perror("   HakoCan1: failed");
			return -1;
		}
	}


	//Create database variables (if everyting works)
	iSpeedref=createVariable('w',1,"speedref");
	iSteeringangleref  = createVariable('w',1,"steeringangleref");
	icurvatureref  = createVariable('w',1,"curvatureref");
	iEnginespeedref  = createVariable('w',1,"enginespeedref");
	ihitchposref  = createVariable('w',1,"hitchposref");
	iNavigationmoderef  = createVariable('w',1,"hakomanual");
	iliftinggearstateref  = createVariable('w',1,"liftinggearstateref");
	ipowertakeoffstateref  = createVariable('w',1,"powertakeoffstateref");
	ig01  = createVariable('w',1,"g01");
	ig02  = createVariable('w',1,"g02");
	iptor  = createVariable('w',1,"ptor");
	iauxr  = createVariable('w',1,"auxr");
	ifll  = createVariable('w',1,"fll");
	iflr  = createVariable('w',1,"flr");
	ihlght  = createVariable('w',1,"hlght");
	irlght  = createVariable('w',1,"rlght");
	ihorn  = createVariable('w',1,"horn");
	iNavigationmode  = createVariable('r',1,"hakonavigationmode");
	iSteeringangle     = createVariable('r',1,"hakosteeringangle");
	icurvature     = createVariable('r',1,"curvature");
	iOdopulses= createVariable('r',1,"hakoodopulses");
	iEnginespeed= createVariable('r',1,"enginespeed");
	iHakostate0= createVariable('r',1,"hakostate0");
	iHakostate1= createVariable('r',1,"hakostate1");
	iHakostate2= createVariable('r',1,"hakostate2");
	iHakostate3= createVariable('r',1,"hakostate3");
	iliftinggearpos= createVariable('r',1,"liftinggearpos");
	ipowertakeoffspeed= createVariable('r',1,"powertakeoffspeed");
	ihornack= createVariable('r',1,"hornack");
	iswitchack=createVariable('r',1,"switchack");
	icvtack=createVariable('r',1,"cvtack");
	isteeringack=createVariable('r',1,"steringack");
	if (can_dev1)
	{
		iOdopulses1= createVariable('r',1,"hakoodopulses1");
		printf("hakoodopuses1 created\n");
	}  
	
	//initialize to manual mode
	setVariable(iNavigationmode, 0, 1); //manual mode

	return 1;
}

/** \brief Entry-point for Can-Bus RX Thread
 *
 * Responsible for reading and parsing all CAN-Bus messages
 * on the first port.
 */
void *canrx_task(void *not_used)
{
	fprintf(stderr, "   HakoCan: Canrx_task running (1030)\n");

	//Lock memory - No more allocations
	if (mlockall(MCL_CURRENT | MCL_FUTURE))
	{
		perror("mlockall");
		exit(-1);
	}

    /* use real-time (fixed priority) scheduler
     * set priority to one less than the maximum
     */
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
    if (sched_setscheduler(0, SCHED_RR, &param)) 
    {
		perror("setscheduler");
		pthread_exit(NULL);
    };

	if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
		fprintf(stderr, "signal: can't ignore SIGPIPE.\n");

	//Wait to make sure variables are created
	while (hakocanRunning < 0) usleep(100000);
	
	while (hakocanRunning) 
	{
		int ret, tmp;
		struct canmsg_t msg;	
		union 
		{
			short s;
			char b[2];
		}conv;

		//receive one CAN message
		ret=secureRead(can_dev, &msg, sizeof(struct canmsg_t));
		if (ret<0) 
		{
			fprintf(stderr,"Error receiving message on CAN-bus\n");
			hakocanRunning = -1; //Shutdown!
		} 
		else 
		{
			//Parse input packages
			printf("can0 Msg.id: %#X\n",msg.id);
			switch(msg.id) 
			{
				case QUADENCCMD: //husk at rette den her!!
					// read quadrature encoder reading
					conv.b[0]=msg.data[0]; 
					conv.b[1]=msg.data[1]; 
					conv.b[2]=msg.data[2]; 
					conv.b[3]=msg.data[3]; 
					// setVariable(iOdopulses1, 0,conv.i);	 
				
				break;
				case 0x105: //Kan ikke finde dennes dokumentation
					conv.b[0]=msg.data[0];
					conv.b[1]=msg.data[1];    	 
					//    setVariable(iSteeringangle, 0, -conv.s);	  
					conv.b[0]=msg.data[2];
					conv.b[1]=msg.data[3];
					tmp=conv.s;
					if ((msg.data[7] &0xA0)==0x80)
						tmp=-tmp;    	 
					setVariable(iOdopulses, 0, tmp);
					//  setVariable(iEnginespeed, 0, msg.data[4]);
					if (msg.data[6]=='M') 
						setVariable(iNavigationmode, 0, 1);
					else
						setVariable(iNavigationmode, 0, 0);
				break;
	
				case 0x50:  //Kan ikke finde dennes dokumentation
					conv.b[0]=msg.data[0];
					conv.b[1]=msg.data[1];    	 
						setVariable(iHakostate0, 0, conv.s);
					conv.b[0]=msg.data[2];
					conv.b[1]=msg.data[3];    	 
						setVariable(iHakostate1, 0, conv.s);
					conv.b[0]=msg.data[4];
					conv.b[1]=msg.data[5];    	 
						setVariable(iHakostate2, 0, conv.s);
					conv.b[0]=msg.data[6];
					conv.b[1]=msg.data[7];    	 
						setVariable(iHakostate3, 0, conv.s);
				break;
	
				case 0x145:  //Kan ikke finde dennes dokumentation
					setVariable(iliftinggearpos, 0,msg.data[0]); 	  
					setVariable(ipowertakeoffspeed, 0, msg.data[1]);
				break;  

				default: 
				break;
			}
		}
	} //RX Loop ends

	//Finish thread
	fprintf(stderr,"HakoCan: Ending RX Thread!\n");
	hakocanRunning = -1;
	pthread_exit(NULL);
}
/** \brief Entry-point for Can-Bus RX Thread
 *
 * Responsible for reading and parsing all CAN-Bus messages
 * on the second port.
 */
void *canrx_task1(void *not_used)
{
	fprintf(stderr, "   HakoCan: Canrx_task1 running\n");

	//Lock memory - No more allocations
	if (mlockall(MCL_CURRENT | MCL_FUTURE))
	{
		perror("mlockall");
		exit(-1);
	}

    /* use real-time (fixed priority) scheduler
     * set priority to one less than the maximum
     */
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
    if (sched_setscheduler(0, SCHED_RR, &param)) 
    {
		perror("setscheduler");
		pthread_exit(NULL);
    };

	if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
		fprintf(stderr, "signal: can't ignore SIGPIPE.\n");

	//Wait to make sure variables are created
	while (hakocanRunning < 0) usleep(100000);

	while (hakocanRunning) 
	{
		int ret;
		//int tmp;
		struct canmsg_t msg;	
		union 
		{
			short s;
			float f;
			int i;
			unsigned char b[4];
		}conv;

		//receive one CAN message
		ret=secureRead(can_dev1, &msg, sizeof(struct canmsg_t));
		if (ret<0) 
		{
			fprintf(stderr,"Error receiving message on CAN-bus\n");
			hakocanRunning = -1; //Shutdown!
		} 
		else 
		{
			// Parse input packages
			// printf("can1 Msg.id: %#X\n",msg.id);
			switch(msg.id) 
			{
				case QUADENCACK: // read quadrature encoder reading
				//	printf("can1: QUADENCMD recived\n");
				
					conv.b[0]=msg.data[0]; //printf( "Quad: %#X\n",msg.data[0] );
					conv.b[1]=msg.data[1]; //printf( "Quad: %#X\n",msg.data[1] );
					conv.b[2]=msg.data[2]; //printf( "Quad: %#X\n",msg.data[2] );
					conv.b[3]=msg.data[3]; //printf( "Quad: %#X\n",msg.data[3] ); 
				//		printf("Quad: %d",conv.i);
					setVariable(iOdopulses1, 0,conv.i);	  
				break;
				
				case STEERINGREPORT1:
				//printf("can1: STEERINGREPORT1 recived\n");
					conv.b[0]=msg.data[0];
					conv.b[1]=msg.data[1];
					conv.b[2]=msg.data[2];
					conv.b[3]=msg.data[3];  
					setVariable(icurvature, 0,(int)(conv.f*10000.0));	  
				break;
				
				case CVTACK:
				//	printf("can1: CVTACK recived\n");
					conv.b[0]=msg.data[0];
					conv.b[1]=msg.data[1];
					conv.b[2]=msg.data[2];
					conv.b[3]=msg.data[3];  
					setVariable(icvtack, 0,conv.i);	  
				break;

				case STEERINGACK:
				//	printf("can1: STEERingACk recived\n");
					conv.b[0]=msg.data[0];
					conv.b[1]=msg.data[1];
					setVariable(isteeringack, 0,conv.s);	  
				break;

				case STEERINGANGLEACK:
				//	printf("can1: STEERINGANGLEACK recived\n");
					conv.b[0]=msg.data[0];
					conv.b[1]=msg.data[1];
					setVariable(iSteeringangle, 0,conv.s);	  
				break;
				
				case ESXREADYAUTOMODE:
					//printf("got esxready \n");
					if (msg.data[0]==0xFF) 
						setVariable(iNavigationmode, 0, 0); //automatic mode
					else
						setVariable(iNavigationmode, 0, 1); //manual mode
				break;
				
				case KEEPAUTOMODEACK:
				//	printf("can1: KEEPAUTOMODEACK recived\n");
				break;   
				  
				case SWITCHACK:
				//	printf("can1: SWITCHACK recived\n");
					setVariable(iswitchack,0, msg.data[1]);
				break;
				
				case HORNACK:
				//	printf("can1: HORNACK recived\n");
					setVariable(ihornack,0, msg.data[1]);
				break;


				default : 
				//	printf("msg.id: %#X\n", (int) msg.id);
				break;
			}
		}
	} //RX Loop ends

	//Finish thread
	fprintf(stderr,"HakoCan: Ending RX Thread!\n");
	hakocanRunning = -1;
	pthread_exit(NULL);
}


 void setmask(  struct canmsg_t *msg,int var,int bit)
 {
	char tmp;
	tmp=(1 << bit);
	if (getWriteVariable(var,0)) 
		{msg->data[0]|=tmp; msg->data[1]&=~tmp;}
	else 
		{msg->data[1]|=tmp; msg->data[0]&= ~tmp;}
 }


/** \brief Transmit messages to the CAN bus
 *
 * Periodic function to transmit variables and
 * requests to the can-bus
 *
 */
extern int periodic(int tick)
{  
	//Just return if CAN isn't running
	if (hakocanRunning < 0) return -1;

	int ret;
	struct canmsg_t msg;
	union 
	{
		short s;
		char b[2];
	}conv;

	union 
	{
		float f;
		int i;
		char b[4];
	}conv1;
	
	/* send on Can Port 0 */
	if(can_dev) // is port open, if not send error.
	{  
		/* What does this do? */
		msg.id=0x150;
		msg.flags=0;
		msg.length=0;
		msg.data[0]=0x0;
		msg.data[1]=0x0;
		ret=secureWrite(can_dev,&msg, sizeof(struct canmsg_t));

		if (ret<0) 
		{
			fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
			hakocanRunning = -1;
			return -1;
		}
  
  
		/* What does this do?! */
		if (isUpdated('w',iSteeringangleref) || isUpdated('w',iSpeedref)||
			isUpdated('w',iEnginespeedref) || isUpdated('w',iNavigationmoderef)) 
		{
			conv.s= -getWriteVariable(iSteeringangleref,0);
			msg.id=0x100; // Rigtigt nummer??
			msg.flags=0;
			msg.length=8;
			msg.data[0]=conv.b[0];
			msg.data[1]=conv.b[1];
			conv.s= getWriteVariable(iSpeedref,0);
			msg.data[2]=conv.b[0];
			msg.data[3]=conv.b[1];
			msg.data[4]=getWriteVariable(iEnginespeedref,0);
			msg.data[5]=0;
			if (getWriteVariable(iNavigationmoderef,0)==1)
				msg.data[6]='M';
			else
				msg.data[6]='A';   
			msg.data[7]=0;
			ret=secureWrite(can_dev,&msg, sizeof(struct canmsg_t));
			if (ret<0) 
			{
				fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
				hakocanRunning = -1;
				return -1;
			}
		}
		/* What does this do?! */
		if (isUpdated('w',iliftinggearstateref)||isUpdated('w',ipowertakeoffstateref))
		{
			msg.id=0x130;
			msg.flags=0;
			msg.length=2;
			msg.data[0]=getWriteVariable(iliftinggearstateref,0);
			msg.data[1]=getWriteVariable(ipowertakeoffstateref,0);
			ret=secureWrite(can_dev,&msg, sizeof(struct canmsg_t));
			if (ret<0) 
			{
				fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
				hakocanRunning = -1;
				return -1;
			}	
		}
	}
	else
	{
			fprintf(stderr,"Error: Can port 0: %d, not open \n",(int) can_dev);
	}
	
	if (can_dev1) // is port open, if not send error.
	{
		/* Send Keep-Auto-Mode command, every cycle */
		msg.id=KEEPAUTOMODE;
		msg.flags=0;
		msg.length=1;
		msg.data[0] = 0xA5; // 0xA5:Keep Auto Mode -- 0xCC:Not Auto Mode
		ret=secureWrite(can_dev1,&msg, sizeof(struct canmsg_t)); // Can1 port
		
		if (ret<0) 
		{
			fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
			hakocanRunning = -1;
			return -1;
		}
		
		// Ask for encoderdata
		msg.id=0x110;
		msg.flags=0;
		msg.length=1;
		msg.data[0]=0x0;
		msg.data[1]=0x0;
		
		ret=secureWrite(can_dev1,&msg, sizeof(struct canmsg_t));
		if (ret<0) 
		{
			fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
			hakocanRunning = -1;
			return -1;
		}
		 // Ask for Steeringreport
		msg.id=STEERINGREPORTREQ;
		msg.flags=0;
		msg.length=0;
		msg.data[0]=0x0;   
		ret=secureWrite(can_dev1,&msg, sizeof(struct canmsg_t));
		if (ret<0) 
		{
			fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
			hakocanRunning = -1;
			return -1;
		}
		/* Sends a command to the SWITCH */
		msg.id=SWITCHCMD;
		msg.flags=0;
		msg.length=2;
	   
		setmask(&msg,irlght,0);
		setmask(&msg,ihlght,1);
		setmask(&msg,iflr,2);
		setmask(&msg,ifll,3);
		setmask(&msg,iauxr,4);
		setmask(&msg,iptor,5);
		setmask(&msg,ig01,6);
		setmask(&msg,ig02,7);
		ret=secureWrite(can_dev1,&msg, sizeof(struct canmsg_t));
		if (ret<0) 
		{
			fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
			hakocanRunning = -1;
			return -1;
		}
		
		/* Send a command to Engine RPM */
		if (isUpdated('w',iEnginespeedref))
		{
			msg.id=RPMCMD;
			msg.flags=0;
			msg.length=8;
			conv1.i=(getWriteVariable(iEnginespeedref,0) * 1000) / 60;
			// convert to mHz
			// conv.i = (conv.i * 1000) / 60;
			msg.data[0]=0xc5;
			msg.data[1]=0;
			msg.data[4]=conv1.b[0];
			msg.data[5]=conv1.b[1];
			msg.data[6]=conv1.b[2];
			msg.data[7]=conv1.b[3];   
			ret=secureWrite(can_dev1,&msg, sizeof(struct canmsg_t));
			if (ret<0) 
			{
				fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
				hakocanRunning = -1;
				return -1;
			}	
		}
		
		/* Send a command to Curvature */
		if (isUpdated('w',icurvatureref))
		{
			msg.id=CURVATURECMD;
			msg.flags=0;
			msg.length=4;
			conv1.f=getWriteVariable(icurvatureref,0)/10000.0;
			msg.data[0]=conv1.b[0];
			msg.data[1]=conv1.b[1];
			msg.data[2]=conv1.b[2];
			msg.data[3]=conv1.b[3];   
			ret=secureWrite(can_dev1,&msg, sizeof(struct canmsg_t));
			if (ret<0) 
			{
				fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
				hakocanRunning = -1;
				return -1;
			}	
		}
		
		/* Send a command to CVT Control*/
		if (isUpdated('w',iSpeedref))
		{
			msg.id=CVTCONTROLCMD;
			msg.flags=0;
			msg.length=4;
			conv1.i=getWriteVariable(iSpeedref,0);
			msg.data[0]=conv1.b[0];
			msg.data[1]=conv1.b[1];
			msg.data[2]=conv1.b[2];
			msg.data[3]=conv1.b[3];   
			msg.data[3]&=0x3f;
			msg.data[3]|=0xc0;
			ret=secureWrite(can_dev1,&msg, sizeof(struct canmsg_t));
			if (ret<0) 
			{
				fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
				hakocanRunning = -1;
				return -1;
			}	
		} 
		
		/* Send a command to correct Steeringangle*/
		if (isUpdated('w',iSteeringangleref))
		{
			msg.id=STEERINGANGLECMD;
			msg.flags=0;
			msg.length=2;
			conv.s=getWriteVariable(iSteeringangleref,0);
			msg.data[0]=conv.b[0];
			msg.data[1]=conv.b[1];

			ret=secureWrite(can_dev1,&msg, sizeof(struct canmsg_t));
			if (ret<0) 
			{
				fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
				hakocanRunning = -1;
				return -1;
			}	
		} 
		
		/* Send a command to rear Hitch*/
		if (isUpdated('w',ihitchposref))
		{
			msg.id=HITCHCMD;
			msg.flags=0;
			msg.length=4;
			conv1.i=getWriteVariable(ihitchposref,0);
			msg.data[0]=conv1.b[0];
			msg.data[1]=conv1.b[1];
			msg.data[2]=conv1.b[0];
			msg.data[3]=conv1.b[1];   
			ret=secureWrite(can_dev1,&msg, sizeof(struct canmsg_t));
			if (ret<0) 
			{
				fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
				hakocanRunning = -1;
				return -1;
			}	
		} 
	 
		/* Send a Command to the horn */   
		msg.id=HORNCMD;
		msg.flags=0;
		msg.length=2;
		setmask(&msg,ihorn,0);
		ret=secureWrite(can_dev1,&msg, sizeof(struct canmsg_t));
		if (ret<0) 
		{
			fprintf(stderr,"Error sending message with id:0x%4x\n",(int)msg.id);
			hakocanRunning = -1;
			return -1;
		}
	}
	else
	{
		fprintf(stderr,"Error Can Port 1: %d not open\n",(int)can_dev1);
	}
  
	return 1;
}

/************************** XML Initialization **************************/
///Struct for shared parse data
typedef struct  
{
	int depth;
	char skip;
	char enable;
	char found;
}parseInfo;

//Parsing functions
void XMLCALL hakocanStartTag(void *, const char *, const char **);
void XMLCALL hakocanEndTag(void *, const char *);


/** \brief Initialize the Crossbow HAL
 *
 * Reads the XML file and sets up the Crossbow settings
 * 
 * Finally the rx thread is started and the server 
 * is ready to accept connections
 * 
 * \param[in] *char filename
 * Filename of the XML file
 * 
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
extern int initXML(char *filename) 
{
	parseInfo xmlParse; 
	char *xmlBuf = NULL;
	int xmlFilelength;
	int done = 0;
	int len;
	FILE *fp;

	//Print initialization message
	//Find revision number from SVN Revision
	char *i,versionString[20] = REVISION, tempString[10];
	i = strrchr(versionString,'$');
	strncpy(tempString,versionString+6,(i-versionString-6));
	tempString[(i-versionString-6)] = 0;
	printf("HakoCan: Initializing HAKO CAN-Bus driver %s.%s\n",HAKOCANVERSION,tempString);


	/* Initialize Expat parser*/
	XML_Parser parser = XML_ParserCreate(NULL);
	if (! parser) {
		fprintf(stderr, "HakoCan: Couldn't allocate memory for XML parser\n");
		return -1;
	}

	//Setup element handlers
	XML_SetElementHandler(parser, hakocanStartTag, hakocanEndTag);
	//Setup shared data
	memset(&xmlParse,0,sizeof(parseInfo));
	XML_SetUserData(parser,&xmlParse);

	//Open and read the XML file
	fp = fopen(filename,"r");
	if(fp == NULL)
	{
		printf("HakoCan: Error reading: %s\n",filename);
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
	if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) 
	{
		fprintf(stderr, "HakoCan: XML Parse error at line %d: %s\n",
				(int)XML_GetCurrentLineNumber(parser),
				XML_ErrorString(XML_GetErrorCode(parser)));
		return -1;
	}
	XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <hakocan> XML tag found in plugins section\n");
		return -1;
	}

	//Start crossbow thread after init
	if (xmlParse.enable) hakocanRunning = initHakocan();


	return hakocanRunning;
}

///Handle XML Start tags
void XMLCALL
hakocanStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("hakocan",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("hakocan",el)) {
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   HakoCan: Use of HakoCan disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("controlcan",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) {
       if (strcmp("port",attr[i]) == 0) strncpy(canDevString,attr[i+1],63);
       if (strcmp("port1",attr[i]) == 0) strncpy(canDevString1,attr[i+1],63); 
    }
    printf("   HakoCan: Using Control CAN-port %s  %s\n",canDevString,canDevString1);
  }  

}

///Handle XML End tags
void XMLCALL
hakocanEndTag(void *data, const char *el)
{
	parseInfo *info = (parseInfo *) data;
	info->depth--;

	if (info->depth < info->skip) info->skip = 0;
}
