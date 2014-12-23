/** \file simStage3.cpp
 *  \ingroup hwmodule
 *  \brief Driver for Player/Stage simulator v. 3.2.2
 *
 *  This plugin provides a connection to the Stage simulator
 *  for simulating a single robot.
 *
 *  \author Lars Valdemar Mogensen
 *  \author Anders Billesø Beck
 *  $Rev: 1476 $
 *  $Date: 2011-05-17 13:07:21 +0200 (Tue, 17 May 2011) $
 *  
 */
/***************************************************************************
 *              Copyright 2010 Lars Valdemar Mogensen, DTU                 *
 *                         lvm@elektro.dtu.dk                              *
 *              Copyright 2010 Anders Billesø Beck, DTU                    *
 *                         abb@elektro.dtu.dk                              *
 *                                                                         *
 ***************************************************************************/
/************************** Library version  ***************************/
 #define SIMSTAGE_VERSION  	"0.3"
/************************** Version control information ***************************/
 #define REVISION         "$Rev: 1476 $:"
 #define DATE             "$Date: 2011-05-17 13:07:21 +0200 (Tue, 17 May 2011) $:"
/**********************************************************************************/

//#include <sched.h>
//#include <pthread.h>
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
#include <algorithm>
#include <dlfcn.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/poll.h>
		  
#include <rhd.h>
#include <smr.h>
#include <database.h>
#include <globalfunc.h>
#include <scheduler.h>

//#include <Stage-3.2/stage.hh>

#include "simStage3.h"
#include "laserServer.h"
#include "stage-3.2.2.DTU/libstage/stage.hh"

//#define PRNT_DEBUG

using namespace Stg;

#define STRLEN 128

/******** Global variables *************/
int iEncl, iEncr, iLs, iIr, iGyro, iGyroTemp, iSpl, iSpr,iSteeringangleref,iRstl,iRstr;
int iSimTime;
symTableElement *writeTable;
int simStageRunning = -1;
int defaultTimerPeriod;
float speedupFact;

  int posServer = -1;
  int posClient = -1;

// Stage variables
const size_t MAX_LASER_SAMPLES = 361;

double minfrontdistance = 0.750;
double speed = 0.400;
double turnrate = 60.0/180.0*3.1415;

int randint;
int randcount = 0;
int avoidcount = 0;
int obs = false;

double restR=0,restL=0;
double newspeed = 0.0;
double newturnrate = 0.0;
unsigned short int sEncr=0;
unsigned short int sEncl=0;

typedef struct
{
  ModelLaser* laser;
  ModelPosition* position;
  ModelRanger* ranger;
  ModelFiducial* fiducial;
  ModelBlobfinder* blobfinder;
  ModelCamera* camera;
} robot_t;

//Configuration parameters for the robot model
typedef struct {
	double wheelbase;
	double encres;
	double ka;
	double kb;

} robotConfig;

//Instanciation of configuration parameters
robotConfig robotCfg;
int laserport;

//Stage Data variables
robot_t robot;
WorldGui *world;
Pose oldRobotPose;


//stg_pose_t oldPos = {0,0,0,0};
//stg_velocity_t tmpVel = {1,0,0,0};
char progName[STRLEN];
char worldFileName[STRLEN];
char robotName[STRLEN];

time_t begin;
float now;

//simStage Functions
int initSimStage(void);

#define BLOCK_MAX 200
#define XMIT_BYTES 104    /* about 10ms at 115.2 Kbaud */

/// Init SimStage
int initSimStage(void) {

  char* argv[] = {progName,worldFileName,robotName};
  char** pArgv = argv;
  int argc=3;

  printf("\nInitialising Stage World\n");

    // initialize libstage
  Stg::Init( &argc, &pArgv);
  //StgWorld world;
  world = new WorldGui(400, 300, "Testing RHD Stage");

  world->Load( argv[1] );

  printf("   SimStage3: Loading %s model \n",robotName);
  //robot = new StgModelLaser( worldGui, NULL );
  // robot = // LVM add connect to SMR0 here.
  //robot = NULL;
  robot.position = (ModelPosition*)world->GetModel(robotName);
  if(robot.position == NULL) {
    printf("    Robot %s not found - Exiting!",robotName);
	 return -1;
  }
  else {
    printf("    Robot %s found: ",robotName);
    assert(robot.position);
    robot.position->Subscribe();
/*    robot->SetGlobalPose(oldPos);
    robot->SetGlobalVelocity(tmpVel);*/
  }

  robot.laser = (ModelLaser*)robot.position->GetUnsubscribedModelOfType("laser");
  if(robot.laser == NULL) {
    printf("laser not found");
    //exit(-1);
  }
  else {
    printf(" laser found");
    assert(robot.laser);
    robot.laser->Subscribe();
  }

  robot.ranger = (ModelRanger*)robot.position->GetUnsubscribedModelOfType("ranger");
  if(robot.ranger == NULL) {
    printf(", ranger not found");
    //exit(-1);
  }
  else {
    printf(", ranger found");
    assert(robot.ranger);
    robot.ranger->Subscribe();
  }

  robot.camera = (ModelCamera*)robot.position->GetUnsubscribedModelOfType("camera");
  if(robot.camera == NULL) {
    printf(", camera not found\n");
    //exit(-1);
  }
  else {
    printf(", camera found\n");
    assert(robot.camera);
    robot.camera->Subscribe();
  }


  /****** Create database variables if all is ok **************/
  iEncl		= createVariable('r',1,"encl");
  iEncr		= createVariable('r',1,"encr");
  iLs			= createVariable('r',8,"linesensor");
  iIr			= createVariable('r',6,"irsensor");
  iGyro		= createVariable('r',3,"gyro");
  iGyroTemp = createVariable('r',3,"gyrotemp");
  iSimTime	= createVariable('r',2,"simtime");
  iSpl		= createVariable('w',1,"speedl");
  iSpr		= createVariable('w',1,"speedr");
  iRstl		= createVariable('w',1,"resetmotorl");
  iRstr		= createVariable('w',1,"resetmotorr");
  iSteeringangleref  = createVariable('w',1,"steeringangleref");

  //Get the default timing period from scheduler
  defaultTimerPeriod = getSchedulerPeriod();

  printf("   SimStage3: SimInterval: %llu us, RealTimeInterval: %d us, Target %3.2f of realtime\n",world->sim_interval,defaultTimerPeriod, (double)world->sim_interval/(double)defaultTimerPeriod);

  //Get simulator default speedup factor
  speedupFact = world->GetSpeedup();

  //Start laserscanner server
  initLaserServer(laserport);

  usleep(100000); //Don't return before threads are running

  world->Start(); //Start the world

  Fl::check(); //Update the GUI once (Using the direct call to FLTK gui
  world->UpdateModel(); //Update the world model once

  //Sample the initial robot pose
  Pose oldRobotPose = robot.position->GetGlobalPose();

  if ((posServer = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
      perror("LaserServer: socket creating error");
      pthread_exit(0);
    }

  //Setup socket and bind
  struct sockaddr_in sa;
  sa.sin_family =  AF_INET;
  sa.sin_port = htons((uint16_t)40000);
  sa.sin_addr.s_addr = INADDR_ANY;


  if (bind(posServer, (struct sockaddr *) &sa, sizeof(sa))) {
    perror("SimStage3: Position Error bind");
    pthread_exit(0);
  }

  if (listen(posServer, 2)) {
    perror("SimStage3: Position socket listen error");
    pthread_exit(0);
  }

  return 1;
}


/// Clean up and exit Stage simulator
extern "C" int terminate(void)
{
  fprintf(stdout,"Shutting down Stage Simulator\n");

  world->QuitAll();

  return 0;
}

///Transmit all data to serial bus (called periodically)
extern "C" int periodic(int tick)
{
	static double oldTime = 0, newTime, performance = (double)world->sim_interval;
	static Stg::stg_usec_t oldSimTime = 0, newSimTime;

	//Test if quit has been pressed, otherwise update the world
	if (world->TestQuit()) return -1;

	//Stage's clock is driven by callback functions from the FLTK GUI
	//the check() checks for any pending callbacks, then it returns.
	Fl::check();

	//The stage model is manually updated
	world->UpdateModel();

	//Adjust RHD timer to speedup from Stage
	if (world->GetSpeedup() != speedupFact) {
		speedupFact = world->GetSpeedup(); //Sample new speedup factor
		
		//Set the scheduler period accordingly
		setSchedulerPeriod((int)((double) defaultTimerPeriod / speedupFact));

		printf("   Simstage3: Period set to %d ms",(int)((double) defaultTimerPeriod / speedupFact));
		printf(" Running at %3.2f of realtime\n",(double)world->sim_interval / ((double) defaultTimerPeriod / speedupFact));
	}
	

	//Transmit laserscanner data to connected AURS clients
	laserServerTransmit(robot.laser,world->SimTimeNow());

	//Update simulation time
	setVariable(iSimTime,0,(int32_t)(world->SimTimeNow() / 1000000));
	setVariable(iSimTime,1,(int32_t)(world->SimTimeNow() % 1000000));


	//ROBOT MOTORS: Set the robot speed if either motor command is updated
	int spl, spr;
	double fwdSpeed, turnRate;

	if (isUpdated('w',iSpr) || isUpdated('w',iSpl)) {
		spl = getWriteVariable(iSpl,0);
		spl = (spl == -128) ? -127 : spl;
		spr = getWriteVariable(iSpr,0);
		spr = (spr == -128) ? -127 : spr;
		fwdSpeed = ((spl + spr) / 2) * 0.01; //Convert from speed cmd to m/s
		turnRate = ((spl - spr) / robotCfg.wheelbase) * 0.01;

		//Set speed to Stage model
		robot.position->SetSpeed(fwdSpeed,0,-turnRate);
	}

	//ENCODERS: Reverse calculate encoder values
	double			dx, dy, dTh;
	double			dWr=0,dWl=0;
	static double	restR = 0, restL = 0; //Remainder when calculating from doubles to enc values
	int32_t			encL, encR;
	static int		sEncl, sEncr;

	Pose robotPose		= robot.position->GetPose();
	Velocity robotVel	= robot.position->GetVelocity();

	//Calculate moves
	dx  = oldRobotPose.x - robotPose.x;
   dy  = oldRobotPose.y - robotPose.y;
   dTh = oldRobotPose.a - robotPose.a; 
	if (dTh > M_PI)			dTh -= 2*M_PI; //Keep angles within +/- PI
	else if (dTh < - M_PI)	dTh += 2*M_PI;

	//Save old robot pose
	oldRobotPose = robotPose;

	if(robotVel.x < 0) {
		dWr = -sqrt(dx*dx+dy*dy) + robotCfg.wheelbase/2.0*dTh;
		dWl = -sqrt(dx*dx+dy*dy) - robotCfg.wheelbase/2.0*dTh;
	}
	else {
		dWr = sqrt(dx*dx+dy*dy) + robotCfg.wheelbase/2.0*dTh;
		dWl = sqrt(dx*dx+dy*dy) - robotCfg.wheelbase/2.0*dTh;
	}

	//Alotta stuff for printing and timing...
	timeval tm1;
	gettimeofday(&tm1,NULL);
	newTime = tm1.tv_sec + (double)tm1.tv_usec / 1e6;
	if (oldTime == 0) oldTime = newTime;
	newSimTime = world->SimTimeNow();
	performance = 0.999 * performance + ((newTime-oldTime) * 1e6) * 0.001;
	//printf("Speed %4.3f Movement Wr %4.3f Wl %4.3f (Vel %4.3f,%4.3f) Sims:%lld Lups: %d Interval %lld Real:%6.2f P:%3.2f\n",fwdSpeed, dWr,dWl,robotVel.x,robotVel.y,world->GetUpdateCount(),robot.laser->GetUpdates(), (newSimTime-oldSimTime),(newTime-oldTime)*1e6,(double)world->sim_interval / performance);
	oldTime = newTime;
	oldSimTime = newSimTime;


	//Add the remanining part from discretization
	dWr=dWr+restR;
   dWl=dWl+restL;

	//Calculate encoder values
   encR = (int)(dWr / robotCfg.encres);
   encL = (int)(dWl / robotCfg.encres);

	//Save the remainder from discretization
	restR = dWr - encR*robotCfg.encres;
	restL = dWl - encL*robotCfg.encres;
	
	//Update, limit and signextend encoder values
	//Right
	sEncr = (sEncr + encR);
	if (sEncr > 0x7FFF)			sEncr -= 0x10000;
	else if (sEncr < -0x7FFF)  sEncr += 0x10000;
	//Left
	sEncl = (sEncl + encL);
	if (sEncl > 0x7FFF)			sEncl -= 0x10000;
	else if (sEncl < -0x7FFF)	sEncl += 0x10000;
	
	

	//Push encoders to rhd database
	setVariable(iEncl,0,sEncr);
	setVariable(iEncr,0,sEncl);


	//IRSENSORS: Extract and reverse calculate IR sensors
	double sensDist;
	int    irValue;
	if (robot.ranger != NULL ) {
		for (size_t i = 0; i < 6; i++) {
			if (i < robot.ranger->sensors.size()) {
				sensDist = robot.ranger->sensors[i].range;
				if (sensDist < 0.05) sensDist = 0.05;

				//Convert to raw IR sensor measurement value
				irValue = robotCfg.ka / sensDist + robotCfg.kb;

				//Put ensor value in RHD Database
				setVariable(iIr,i,irValue);
			}
		}

	}
 
	//Poll for accept
	int i;
	char rx, str[1024];

	if (posClient == -1) {
		struct pollfd pfd;
		pfd.fd = posServer;
		pfd.events = POLLIN;

		if (poll(&pfd, 1, 0)) {
			posClient = accept(posServer, NULL, NULL);
			printf("   SimStage3: Position client connected\n");
		}
	} else {
		struct pollfd pfd;
		pfd.fd = posClient;
		pfd.events = POLLIN;
		if (poll(&pfd, 1, 0)) {
			i = recv(posClient, &rx, 1, MSG_NOSIGNAL);
			sprintf(str,"%f %f\n",robotPose.x, robotPose.y);
			printf("Sending pos: %s",str);
			i = secureSend(posClient,str,strlen(str));
		}
		

	}

  return 1;
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
void XMLCALL startTag(void *, const char *, const char **);
void XMLCALL endTag(void *, const char *);

/** \brief Initialize the SMRD HAL
 *
 * Reads the XML file and sets up the SMRD settings
 * 
 * Finally the rx threads is started and the driver 
 * is ready to read data
 * 
 * \param[in] *char filename
 * Filename of the XML file
 * 
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
extern "C" int initXML(char *filename) {

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
  printf("simStage: Initializing Player/Stage simulator interface %s.%s\n",SIMSTAGE_VERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "   Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, startTag, endTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("   Error reading: %s\n",filename);
    return -1;
  }
  //Get the length of the file
	fseek(fp,0,SEEK_END);
	xmlFilelength = ftell(fp); //Get position
	fseek(fp,0,SEEK_SET); //Return to start of file

	//Allocate text buffer
	xmlBuf = (char *) realloc((char *) xmlBuf,xmlFilelength+10); //Allocate memory
	if (xmlBuf == NULL) {
		fprintf(stderr, "   Couldn't allocate memory for XML File buffer\n");
		return -1;
	}
	memset(xmlBuf,0,xmlFilelength);
  len = fread(xmlBuf, 1, xmlFilelength, fp);
  fclose(fp);

  //Start parsing the XML file
  if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) {
    fprintf(stderr, "   XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <simStage> XML tag found in plugins section\n");
		return -1;
	}

  //Initialize rFLEX, if XML parsed properly
  if (xmlParse.enable) done = initSimStage();

 return done;
}

void XMLCALL
startTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 	((info->depth == 3) && (strcmp("simStage",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("simStage",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   Use of simStage disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("world",el) == 0) { //Load world infomation
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("file",attr[i]) == 0) strncpy(worldFileName,attr[i+1],STRLEN-1);
    for(i = 0; attr[i]; i+=2) if (strcmp("robotName",attr[i]) == 0) strncpy(robotName,attr[i+1],STRLEN-1);  
    printf("   Using world: %s, Robot: %s \n",worldFileName,robotName);

  } else if (strcmp("robot",el) == 0) { //Load robot specifications
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("wheelbase",attr[i]) == 0) robotCfg.wheelbase = strtod(attr[i+1],NULL);
    for(i = 0; attr[i]; i+=2) if (strcmp("encresolution",attr[i]) == 0) robotCfg.encres = strtod(attr[i+1],NULL);
    printf("   Robot has a wheelbase of %04.3f m and an encoder resolution of %08.7f m/tick\n",robotCfg.wheelbase,robotCfg.encres);
  
  } else if (strcmp("irsensor",el) == 0) { //Load robot specifications
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("ka",attr[i]) == 0) robotCfg.ka = strtod(attr[i+1],NULL);
    for(i = 0; attr[i]; i+=2) if (strcmp("kb",attr[i]) == 0) robotCfg.kb = strtod(attr[i+1],NULL);
    printf("   Irsensor calibration - ka: %4.2f  kb: %2.4f\n",robotCfg.ka,robotCfg.kb);

  } else if (strcmp("laserscanner",el) == 0) { //Load robot specifications
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) laserport = atoi(attr[i+1]);
    printf("   Laserscanner server accepting clients on port %d\n",laserport);
  }

/* else if (strcmp("odometry",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("period",attr[i]) == 0) odoPeriod = atoi(attr[i+1]);  
    printf("   Odometry period: %d.%03d ms\n",odoPeriod/1000,odoPeriod%1000);
	} else if (strcmp("translation",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("torque",attr[i]) == 0) defTransTorque = atoi(attr[i+1]);
		for(i = 0; attr[i]; i+=2) if (strcmp("accel",attr[i]) == 0) defTransAccl = atoi(attr[i+1]);  
    printf("   Translation default: torque: %u accel: %u\n",defTransAccl,defTransTorque);
  } else if (strcmp("rotation",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("torque",attr[i]) == 0) defRotTorque = atoi(attr[i+1]);
		for(i = 0; attr[i]; i+=2) if (strcmp("accel",attr[i]) == 0) defRotAccl = atoi(attr[i+1]);  
    printf("   Rotation default: torque: %u accel: %u\n",defRotTorque,defRotAccl);
  } else if (strcmp("bumperbrake",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if ((strcmp("default",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
			bumpBrake = 1;
		} else {
			bumpBrake = 0;
		}
  }*/ 

}

void XMLCALL
endTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}
