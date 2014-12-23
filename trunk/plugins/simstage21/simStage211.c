/** \file simStage.c
*  \ingroup hwmodule
*
*   Stage simulator interface
*******************************************************************/
/************************** Library version  ***************************/
 #define SIMSTAGE_VERSION  	"0.2"
/************************** Version control information ***************************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2008-09-30 11:58:03 +0200 (Tue, 30 Sep 2008) $:"
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
#include <math.h>

#include <sys/poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include <dlfcn.h>

#include <rhd.h>
#include <smr.h>

//#include <algorithm>
#include <database.h>
#include <globalfunc.h>

#include "simStage211.h"

//#define PRNT_DEBUG
#define CLIENTS 1

/******** Global variables *************/
int iEncl, iEncr, iLs, iIr, iGyro, iGyroTemp, iSpl, iSpr,iSteeringangleref,iRstl,iRstr,iSimTime;
int ieasting, inorthing, iquality, isatellites, idop;
uint8_t txBuf[32];
int iterator;
int32_t inputRS485[32], inputRS232[32];
symTableElement *writeTable;
int rs485, rs232;  //File descriptors
int simStageRunning = -1;
char serial1Dev[64], serial2Dev[64];
int baudrate1, baudrate2;

// Stage variables
const size_t MAX_LASER_SAMPLES = 361;

double minfrontdistance = 0.750;
double speed = 0.400;
double turnrate = 60.0/180.0*3.1415;

int randint;
int randcount = 0;
int avoidcount = 0;
int obs = FALSE;

double restR=0,restL=0;
double newspeed = 0.0;
double newturnrate = 0.0;
short sEncr=0;
short sEncl=0;

// Simulator and robot structs
stg_world_t* world;
stg_model_t* position;
stg_model_t* laser;
stg_model_t* ranger;
stg_position_cmd_t cmd;
//stg_velocity_cmd_t vel;
stg_velocity_t vel;
stg_pose_t pos;

// Structs for calculations
stg_pose_t oldPos={0,0,0};
time_t begin;
float now;

char worldFileName[STRLEN];
char robotName[STRLEN];
char stagePath[STRLEN];
int laserPort;
int serverSocket;
struct client {
  int fd;
  volatile int ready;     /* >0 to enable data to client */
};
struct client aclients;
char rcvStrn[STRLEN];
char sndStrn[STRLEN*STRLEN];

//simStage Functions
int initSimStage(void);
void *laser_tx_task(void *);
pthread_t laser_tx_thread;
pthread_attr_t attr;

#define BLOCK_MAX 200
#define XMIT_BYTES 104    /* about 10ms at 115.2 Kbaud */

// Fake load
void *stageHandle;

/// Init SimStage
int initSimStage(void) {

  char progName[STRLEN];
  strncpy(progName,"rhdsim",STRLEN-1);

  char* argv[] = {progName,worldFileName,robotName};
  int argc=3;

    //Load libstage explicitly
    //Load plugin, resolving all symbols and using global visibility
    stageHandle = dlopen (stagePath,RTLD_NOW | RTLD_GLOBAL);
    if (stageHandle == NULL) {
        printf("\nError loading libstage - Exiting\n");
        printf("Error message: %s\n",dlerror());
        exit(1);
    }
    else {
	printf("\nLoaded libstage\n");
    }
        
  // initialize libstage
  stg_init( argc, argv );

  world = stg_world_create_from_file( argv[1] );

  char* robotname = argv[2];

  // generate the name of the laser attached to the robot
  char lasername[64];
  snprintf( lasername, 63, "%s.laser:0", robotname ); 
  
  char rangername[64];
  snprintf( rangername, 63, "%s.ranger:0", robotname ); 
  
  position = stg_world_model_name_lookup( world, robotname );  
  laser = stg_world_model_name_lookup( world, lasername );
  ranger = stg_world_model_name_lookup( world, rangername );
  
  if(position == NULL ) {
    printf("Position == NULL\n");
    return -1;
  }
  else {
    stg_model_subscribe( position );
    stg_model_print( position, "Subscribed to model" );
  }

  if(laser == NULL ) {
    printf("Laser == NULL\n");
  }
  else {
    // subscribe to the laser - starts it collecting data
    stg_model_subscribe( laser );
    stg_model_print( laser, "Subscribed to model" );  
  }
  
  if(ranger == NULL ) {
    printf("Ranger == NULL\n");
  }
  else {
    // subscribe to the ranger - starts it collecting data
    stg_model_subscribe( ranger );
    stg_model_print( ranger, "Subscribed to model" );
  }

  // start the clock
  begin=time(NULL);
  now=begin;
  stg_world_start( world );

  stg_world_set_interval_real( world, 10 );
  stg_world_set_interval_sim( world, 10 );

  /****** Create database variables if all is ok **************/
  iEncl = createVariable('r',1,"encl");
  iEncr = createVariable('r',1,"encr");
  iLs   = createVariable('r',8,"linesensor");
  iIr   = createVariable('r',6,"irsensor");
  iGyro = createVariable('r',3,"gyro");
  iGyroTemp = createVariable('r',3,"gyrotemp");
  iSimTime = createVariable('r',2,"simtime");

  iSpl  = createVariable('w',1,"speedl");
  iSpr  = createVariable('w',1,"speedr");
  iRstl  = createVariable('w',1,"resetmotorl");
  iRstr  = createVariable('w',1,"resetmotorr");
  iSteeringangleref  = createVariable('w',1,"steeringangleref");

  // HAKO RTK-GPS simulation
  ieasting = createVariable('r',2,"GPSeasting");
  inorthing = createVariable('r',2,"GPSnorthing");
  iquality = createVariable('r',1,"GPSquality");
  isatellites = createVariable('r',1,"GPSsatused");
  idop = createVariable('r',2,"GPSdop");
  
  //Get table pointers
  writeTable = getSymtable('w');

  usleep(100000); //Don't return before threads are running

// stg_world_update( world,0 );
// fprintf(stdout,"After Stage update\n");
//   while( (stg_world_update( world,0 )==0) )
//     {
// fprintf(stdout,"In while loop\n");
// //  usleep(10000); //Don't return before threads are running
// }
  stg_world_set_interval_real( world, 0 );
  stg_world_update( world,1 );
  stg_model_get_pose(position,&oldPos);
  
  printf("Initialised simStage\n");
  

  // Initialization of threads
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  
  aclients.fd=-1;
  // Spawning transmit thread for Laser scanner data
    if (pthread_create(&laser_tx_thread, &attr, laser_tx_task, 0)) {
      perror("simStage: Error can't start laser TX thread");
    return -1;
  }

  return 1;
}

/// Clean up and exit Stage simulator
extern void terminate(void)
{
  fprintf(stdout,"Shutting down Stage Simulator\n");
  // Quit thread
  // TODO: Write quit thread
/*
  fprintf(stdout,"   Server: Shutting down server and clients\n");
  if (aclients.fd != -1) { close(aclients.fd);
    aclients.fd  = -1;
  }
  close(serverSocket);
*/

  // Quit simulator
  stg_quit_request();
}

int writeStartTags(char* buf)
{
    char* begin = buf;
    char tmp[STRLEN];
    int len=0;
    sprintf(tmp,"<scanget ");len=strlen(tmp);strncpy(buf,tmp,len);buf+=len;
    sprintf(tmp,"interval=\"0.36\" ");len=strlen(tmp);strncpy(buf,tmp,len);buf+=len;
    sprintf(tmp,"count=\"665\" ");len=strlen(tmp);strncpy(buf,tmp,len);buf+=len;
    sprintf(tmp,"tod=\"%f\" ",(double)stg_timenow()/1000);len=strlen(tmp);strncpy(buf,tmp,len);buf+=len;
    sprintf(tmp,"unit=\"mm\" ");len=strlen(tmp);strncpy(buf,tmp,len);buf+=len;
    sprintf(tmp,"min=\"-120\" ");len=strlen(tmp);strncpy(buf,tmp,len);buf+=len;
    sprintf(tmp,"max=\"120\" >");len=strlen(tmp);strncpy(buf,tmp,len);buf+=len;
    sprintf(tmp,"<bin size=\"%d\" ",665*4);len=strlen(tmp);strncpy(buf,tmp,len);buf+=len;
    sprintf(tmp,"codex=\"HEX\">");len=strlen(tmp);strncpy(buf,tmp,len);buf+=len;
    
    buf = begin;
    return strlen(buf);
}

int writeLaserData(stg_laser_sample_t* laserdata, char* buf) {
    
    char* begin = buf;
    char tmp[STRLEN],tmp2[STRLEN];
    int len=0,i=0;
    buf+=strlen(buf);
    for(i=0;i<665;i++) {
       sprintf(tmp,"%04x",(int)(laserdata[i].range*1000));
       // Transform to little endian
       tmp2[0]=tmp[2];
       tmp2[1]=tmp[3];
       tmp2[2]=tmp[0];
       tmp2[3]=tmp[1];
       tmp2[4]=0;
       //printf("%s %s\n",tmp,tmp2);
       len=strlen(tmp);
       strncpy(buf,tmp2,len);
       buf+=len; 
    }
    return strlen(begin);
}

int writeEndTags(char* buf)
{
    char* begin = buf;
    char tmp[STRLEN];
    int len=0;
    buf+=strlen(buf);
    sprintf(tmp,"</bin></scanget>");len=strlen(tmp);strncpy(buf,tmp,len);buf+=len;
    return strlen(begin);
}

///laser_tx_task thread
void *laser_tx_task(void *not_used)
{
  fprintf(stderr, "   simstage: laser tx_task initiating\n");

  int i, client;

  if ((serverSocket = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
      perror("Server: socket creating error");
      //serverRunning = -1;
      pthread_exit(0);
    }

  //Setup socket and bind
  struct sockaddr_in sa;
  sa.sin_family =  AF_INET;
  sa.sin_port = htons((uint16_t)laserPort);
  sa.sin_addr.s_addr = INADDR_ANY;
  //inet_aton("127.0.0.1", &sa.sin_addr); //If host limitation should be used

  if (bind(serverSocket, (struct sockaddr *) &sa, sizeof(sa))) {
    perror("Server: Error bind");
    //serverRunning = -1;
    pthread_exit(0);
  }

  if (listen(serverSocket, 1)) {
    perror("Server: socket listen error");
    //serverRunning = -1;
    pthread_exit(0);
  }
  //Init went ok, mark server as running
  fprintf(stdout,"   simstage: laser tx_task running\n");
  //Thread main loop - Wait for clients and connect them
  while ((client = accept(serverSocket, 0, 0)) >= 0) {
    fprintf(stdout,"Running accept loop\n");
    for (i = 0; i < CLIENTS; i++) {
      if (aclients.fd == -1)
        {
          printf("Server: Client %d connected!!\n",i);
          aclients.fd = client;
          aclients.ready = 0;

          aclients.ready = 1; //Move this to recieve end
          break;
      } else {
          printf("Server: Client %d error!!\n",i);
      }
    }
    if (i == CLIENTS) {
        close(client);
        printf("Server: Closed client \n");
    }
  }
  fprintf(stdout,"Accept error\n");

  for(i = 0; i < CLIENTS; i++) {
    if (aclients.fd != -1) {
       close(aclients.fd);
       aclients.fd = -1;    //Mark client slot as free
       aclients.ready = 0;
    }
  }

  //Close server socket
  //serverRunning = -1;
  //close(serverSocket);

  perror("Server: Error accepting client");
  fprintf(stderr,"Server: Shutting down server!\n");
  exit(0); //Shutdown application!
}

///Transmit all data to serial bus (called periodically)
extern int periodic(int in)
{
  int returnValue = 0;
  //int rs232_flag = 0;
  int rcvn = 0;
  writeTable = getSymtable('w');
  int tempData;
  int tmp2=0;
  double fwdSpeed=0;
  double turnRate=0;
  double dx=0,dy=0,dTh=0;

  double wheelSep = 0.26;
  double mPrEnco = 0.00010245;
  unsigned long simTime=0;
  double calibration_ka = 15.0;
  double calibration_kb = 0.0;
  int i=0;

//   memset(&cmd,0,sizeof(cmd));
//   newspeed=0.1;
//   newturnrate=30/180*3.1415;

   //fprintf(stdout,"Pause...  ");sleep(1);

  simStageRunning=1;
  simStageRunning=stg_quit_test();
  if (simStageRunning != 0) {
    fprintf(stdout,"simStageRunning < 0\n");
    return -1;
  }

  // TODO: Empty read socket

/*
  if(aclients.fd != -1) {
      rcvn = read(aclients.fd, rcvStrn, STRLEN);
      if(rcvn < 0){
          perror("read");
      }
      else if(rcvn > 0) {
         fprintf(stdout,"rcvn=%4d %s\n",rcvn,rcvStrn);
         rcvn=0;
      }
      else {
         fprintf(stdout,"rcvn=%4d %s\n",rcvn,rcvStrn); 
      }
  }
*/


  // Send motor speed if both motor variables has been updated
  if (isUpdated('w',iSpl) && isUpdated('w',iSpr)) {
    tempData = getWriteVariable(iSpl,0);
    tempData = (tempData == -128) ? -127 : tempData;
    tmp2 = getWriteVariable(iSpr,0);
    tmp2 = (tmp2 == -128) ? -127 : tmp2;
   
#ifdef PRNT_DEBUG
printf("Corrected iSpl %d iSpr %d\n",tempData,tmp2);
#endif

    fwdSpeed = (tempData+tmp2)/2 * 0.01;
    
    turnRate = (tempData-tmp2)/wheelSep * 0.01; // Turnrate based on wheel separation and motor speed;
  #ifdef PRNT_DEBUG    
fprintf(stdout,"motor speed fwdSpeed %f turnRate %f \n", fwdSpeed,turnRate);
#endif
//     txBuf[0] = 2;
//     txBuf[1] = 0x11;
//     txBuf[2] = tempData;   
//     xcount += msg_write(rs485, txBuf);
   
  }


  // Send motor speed l
  if (isUpdated('w',iSpl)) {
    tempData = getWriteVariable(iSpl,0);
    tempData = (tempData == -128) ? -127 : tempData;


    
//     txBuf[0] = 2;
//     txBuf[1] = 0x11;
//     txBuf[2] = tempData;   
//     xcount += msg_write(rs485, txBuf);
  }
  // Send motor speed right
  if (isUpdated('w',iSpr)) {
    tempData = - getWriteVariable(iSpr,0);
    tempData = (tempData == -128) ? -127 : tempData;
//     txBuf[0] = 2;
//     txBuf[1] = 0x12;
//     txBuf[2] = tempData;   
//     xcount += msg_write(rs485, txBuf);
  }
 // Send ackerman steering angle
  if (isUpdated('w',iSteeringangleref)) {
    tempData = getWriteVariable(iSteeringangleref,0);
    tempData = (tempData == -128) ? -127 : tempData;
//     txBuf[0] = 3;
//     txBuf[1] = 0xF1;
//     txBuf[2] = 0x0B;
//     txBuf[3] = tempData;
//     xcount += msg_write(rs485, txBuf);
  }
  if (isUpdated('w',iRstl)) {
    txBuf[0] = 1;
    txBuf[1] = 0x01;
//     xcount += msg_write(rs485, txBuf);
  }
  if (isUpdated('w',iRstr)) {
    txBuf[0] = 1;
    txBuf[1] = 0x02;
//     xcount += msg_write(rs485, txBuf);
  }

// Dummy value load
//    fwdSpeed=0.1;
//    turnRate=15.0/180.0*3.1415;

// Pack stage struct with new commands
  memset(&cmd,0,sizeof(cmd));
  cmd.x = fwdSpeed;
  cmd.y = 0;
  cmd.a = -turnRate;

  vel.x = fwdSpeed;
  vel.y = 0;
  vel.a = -turnRate;

  stg_model_set_cmd( position, &cmd, sizeof(cmd));

  #ifdef PRNT_DEBUG
  fprintf(stdout,"cmd x,a %4f, %4f\n",cmd.x,cmd.a);
  #endif


  // Update stage world
  returnValue = stg_world_update( world,0 );


  simTime=stg_timenow();
  //simTime=stg_world_get_simtime(world);
  //simTime=stg_timenow();
  now=simTime * 0.01;
  setVariable(iSimTime, 0, (simTime/1000));
  setVariable(iSimTime, 1, (simTime*1000)%1000000);

//  printf("simTime %f %d\n",now,simTime);
// Read out updated data from stage world

  vel.x = 0;
  vel.y = 0;
  vel.a = 0;

  stg_model_get_velocity(position,&vel);
  stg_model_get_pose(position,&pos);
  
  #ifdef PRNT_DEBUG
  fprintf(stdout,"vel x,y,a %4f,%4f,%4f\n",vel.x,vel.y,vel.a);
  fprintf(stdout,"pos x,y,a %4f,%4f,%4f\n",pos.x,pos.y,pos.a);
  #endif

  // Old version of update
  // Calculate encoder ticks
{
//   int encL = ((vel.x - vel.a*(wheelSep/2))*1/100)/mPrEnco;
//   int encR = ((vel.x + vel.a*(wheelSep/2))*1/100)/mPrEnco; 
//   //fprintf(stdout,"enc R,L %4d, %4d\n",encR,encL);
//  //wheelSep + mPrEnco;
//   sEncl=sEncl+encL;
//   sEncr=sEncr+encR;
}

  dx=oldPos.x-pos.x;
  dy=oldPos.y-pos.y;
  dTh=oldPos.a-pos.a; // Kan være farlig

  double dWr=0,dWl=0;

  if(vel.x < 0) {
    dWr = -sqrt(dx*dx+dy*dy) - wheelSep/2.0*dTh;
    dWl = -sqrt(dx*dx+dy*dy) + wheelSep/2.0*dTh;
  }
  else {
    dWr = sqrt(dx*dx+dy*dy) - wheelSep/2.0*dTh;
    dWl = sqrt(dx*dx+dy*dy) + wheelSep/2.0*dTh;
  }
  
  dWr=dWr+restR;
  dWl=dWl+restL;


  #ifdef PRNT_DEBUG
  fprintf(stdout,"enc dWl,dWr %4f,%4f\n",dWl,dWr);
  #endif
  //fprintf(stdout,"vel x,a %4f, %4f\n",vel.x,vel.a);
  #ifdef PRNT_DEBUG
  fprintf(stdout,"pos x,y %4f, %4f\n",pos.x,pos.y);
  #endif

  // Calculate encoder ticks from velocity and accelaration
  //int encL = ((vel.x - vel.a*(wheelSep/2))*1/100)/mPrEnco;
  //int encR = ((vel.x + vel.a*(wheelSep/2))*1/100)/mPrEnco; 
  
// Calculate encoder ticks from old and new position
  int encL = dWl/mPrEnco;
  int encR = dWr/mPrEnco; 
  

  restR=dWr-encR*mPrEnco;
  restL=dWl-encL*mPrEnco;
  #ifdef PRNT_DEBUG
  fprintf(stdout,"Diff: dWr %f dWl %f \n", restR,restL);
  #endif


  #ifdef PRNT_DEBUG
  fprintf(stdout,"enc R,L %4d, %4d\n\n",encR,encL);
  #endif

 //wheelSep + mPrEnco;
  sEncl=sEncl+encL;
  sEncr=sEncr+encR;

  oldPos = pos;

  // Set encoder ticks
  setVariable(iEncl, 0, sEncl);
  setVariable(iEncr, 0, sEncr);
  setVariable(iGyro,0,now);

  // get some laser data
  if(laser != NULL) {
    size_t laser_sample_count = 0;
    stg_laser_sample_t* laserdata = 
    stg_model_get_data( laser, &laser_sample_count );
    laser_sample_count /= sizeof(stg_laser_sample_t);

    memset(sndStrn,0,sizeof(char)*STRLEN*STRLEN);
    writeStartTags(sndStrn);
    writeLaserData(laserdata,sndStrn);
    writeEndTags(sndStrn);
    
      //fprintf(stdout,"%d %s\n",strlen(sndStrn),sndStrn);
      
    if(aclients.fd != -1) {
      rcvn = write(aclients.fd, sndStrn, strlen(sndStrn));
    }
  }

  // Get some dinstance sensor data
  if(ranger != NULL) {
    size_t ranger_sample_count = 0;
    stg_ranger_sample_t* rangerdata = 
    stg_model_get_data( ranger, &ranger_sample_count );
    ranger_sample_count /= sizeof(stg_ranger_sample_t);
    
    for(i=0;i<(int)ranger_sample_count;i++) {
      double code;
      if(rangerdata[i].range < 0.05)
	code = calibration_ka/0.05 + calibration_kb;
      else {
	code = calibration_ka/rangerdata[i].range + calibration_kb;
      }
      if(code < 0)
	code = 0;
      if(code > 255)
	code = 255;
      setVariable(iIr, i, (int)code);
      #ifdef PRNT_DEBUG
      printf("ranger count %d: %f  %i\n",ranger_sample_count,rangerdata[i].range,(int)code);
    #endif
    }
  }

  // Get and pack position as RTK-GPS
  //stg_position_data_t* odom = (stg_position_data_t*)position->data;
  //stg_model_get_global_pose(position,&pos);
  //printf("y coordinate %.2f\n",odom->pose.y);
  setVariable(iquality, 0, 3); // Quality set to "3" (RTK fix)
  setVariable(isatellites, 0, 7); // Satellites set to "7" (Good reception)
  setVariable(idop, 0, 2); // DOP set to "2.7" (taken from log file)
  setVariable(idop, 1, 0.7*1e6);
  setVariable(ieasting, 0, (int)pos.y); // Y-coordinate maps directly to easting. Floating point number.
  setVariable(ieasting, 1, (int)(pos.y*1000000)%1000000);
  setVariable(inorthing, 0, (int)pos.x); // X-coordinate maps directly to northing. Floating point number.
  setVariable(inorthing, 1, (int)(pos.x*1000000)%1000000);

  return returnValue;
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
void XMLCALL simstageStartTag(void *, const char *, const char **);
void XMLCALL simstageEndTag(void *, const char *);

//XML File buffer size
#define XMLBUFLEN 8192

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
extern int initXML(char *filename) {

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
   XML_SetElementHandler(parser, simstageStartTag, simstageEndTag);
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
simstageStartTag(void *data, const char *el, const char **attr)
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
  } else if (strcmp("conf",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("worldFile",attr[i]) == 0) strncpy(worldFileName,attr[i+1],STRLEN-1);
    for(i = 0; attr[i]; i+=2) if (strcmp("robotName",attr[i]) == 0) strncpy(robotName,attr[i+1],STRLEN-1);
    for(i = 0; attr[i]; i+=2) if (strcmp("stagePath",attr[i]) == 0) strncpy(stagePath,attr[i+1],STRLEN-1);
    for(i = 0; attr[i]; i+=2) if (strcmp("laserPort",attr[i]) == 0) laserPort = atoi(attr[i+1]);
 
    printf("   Config: worldfile %s \n   Robot:  %s \n",worldFileName,robotName);
  }

}

void XMLCALL
simstageEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}


