#define REVISION        "$Rev: 105 $:"
#define DATE            "$Date: 2013-01-20 08:09:42 +0100 (Sun, 20 Jan 2013) $"
#define ID              "$Id: hexakopter.c 105 2013-01-20 07:09:42Z jcan $"
#define PLUGINNAME        "HexKop"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> 
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <semaphore.h>
#include <expat.h>
#include <string.h>

#include <smr.h>
#include <database.h>
#include <rhd.h>
#include <globalfunc.h>
#include "encode.h"
#include "hexakopter.h"

//
// XML config file decode structure
int debugFlag=0;

typedef struct
{
    int depth;
    int skip;
    char enable;
    char found;
}   parseInfo;
//
/** Tick 1 sends the Debug request to the Hexacopter,
 * Tick 2 initialises the Send ExternControl code, to make the Hexacopter initialise in 0 state */
int tick1 = 0;
int tick2 = 0;

/// create variables
void createRHDvariables();
/// init the device interface
int initPlugin(void);
//
/// structure for controlling the kopter
struct str_ExternControl externCtrl;
struct str_ExternControl externCtrlGot;
//
/// structure for all analog measurements from flight controller
struct str_DebugOut DebugOut;
//
/// structure for main 3D pose
struct str_Data3D data3D;
//
/// structure for version info for flight controller
struct str_VersionInfo versionFC;
//
/// buffers for receive and transmit
//#define MK_MAX_TX_BUFFER 170
//char txBuffer[MK_MAX_TX_BUFFER];
/*rxBuffer holds the encoded data recieved from the Hexacopter*/
#define MK_MAX_RX_BUFFER 170
unsigned char rxBuffer[MK_MAX_RX_BUFFER];
/*decoded holds the decoded data recieved from the Hexacopter*/
#define MK_MAX_DE_BUFFER 170
unsigned char decoded[MK_MAX_DE_BUFFER];
//
/// local functions
void * read_task(void *);
void XMLCALL lsStartTag(void *, const char *, const char **);
void XMLCALL lsEndTag(void *, const char *);
/// update flag structure
// typedef struct
// {
//   int Gas;
//   int Gier;
//   int Roll;
//   int Nick;
//   int Config;
// }  up;
// /*Updates struct holds the update flag variables*/
// up updates;
// variables related to thread execution
struct
{ /** is receive thread running */
    int running;
    /** Trigger polling of data from units - when set to 1 */
    int startNewRxCycle;
    /** name of serial device */
    /** thread handle and attributes */
    pthread_t serial_task;
} rxtask;
/*Gas Limiting */
int gasUpperLimit = 240;
int gasLowerLimit = 1;
int initSerial(void);
void * Hexacopter(void * not_used);

/**
 * limit integer to this range.
 * \param value is the value to be tested
 * \param min is the minimum value returned
 * \param max is the maximum value returned
 * \returns value except if it exceeds minimum or maximum, if so it returns the maximum or minimum value */
int limitInt(int value, int min, int max);
unsigned char limitUnsignedChar(int value, int min, int max);
char limitSignedChar(int value, int min, int max);



/** Periodic function is called by RHD.
 * It checks if any of the writeable in variables in RHD are 
 * updated, if so it makes sure they are within the boundries
 * and sends them to the Hexacopter */ 
int periodic(int tick)
{
  tick1++;
  tick2++; 
  int n, update = 0;
  //
  update = isUpdated('w',var.nick); // pitch
  if(!update)
    update = isUpdated('w',var.roll);
  if(!update)
    update = isUpdated('w',var.yaw); // YAW
  if(!update)
    update = isUpdated('w',var.gasref); // trust
  if(!update)
    update = isUpdated('w',var.Config); // 0 = RC or 1 = computer
  if(!update)
    update = isUpdated('w',var.height); 
  // any action needed?
  if(update || (tick2 > 180))
  { // update is needed
    int n, m;
    char * buf = devif.txBuffer;
    int tempData;
    // get RHD data
    tempData = getWriteVariable(var.nick,0);
    externCtrl.Nick = limitUnsignedChar(tempData, -128, 127);
    tempData = getWriteVariable(var.roll,0);
    externCtrl.Roll = limitSignedChar(tempData, -128, 127);
    tempData = getWriteVariable(var.yaw, 0);
    externCtrl.Gier = limitSignedChar(tempData, -128, 127);
    tempData = getWriteVariable(var.gasref,0);
    externCtrl.Gas = limitUnsignedChar(tempData, gasLowerLimit, gasUpperLimit);
    tempData = getWriteVariable(var.height,0);
    externCtrl.Hight = limitInt(tempData, 0, 4000);
    tempData = getWriteVariable(var.Config,0);
    externCtrl.Config = tempData;
    externCtrl.Frame++;
    // pack message
    m = sizeof(externCtrl);
    n = packData(buf, 'b', FC_ADDRESS, (unsigned char *)&externCtrl, m);
    // debug
//     buf[n] = '\0';
//     printf("kopter::control pitch%4d, roll:%4d, yaw%4d, trust%4d, ctrl%2d : packed '%s'\n",
//            externCtrl.Nick, externCtrl.Roll, externCtrl.Gier,
//            externCtrl.Gas, externCtrl.Config, buf);
    // debug end
    secureWrite(devif.ttyDev, buf, n);
    tick2 =0;
  }
  if(tick1 > 99)
  { // must be send at least every 4 seconds
    // to request analog values
    int n;
    char * buf = devif.txBuffer;
    /*Send the Debug Request*/
    unsigned char dataReq = 30; // update rate x 10 ms
    n = packData(buf, 'd', FC_ADDRESS, &dataReq, 1);
    secureWrite(devif.ttyDev, buf, n);
    /*Reset tick1*/
    tick1 = 0;
  }
  if(tick1 == 95)
  { // Request 3D pose update as fast as possible
    char * buf = devif.txBuffer;
    unsigned char dataReq = 2; // update rate x 10 ms
    n = packData(buf, 'c', FC_ADDRESS, &dataReq, 1);
    // debug
//     buf[n] = '\0';
//     printf(PLUGINNAME ": sending 3D req to %d as '%s'\n", FC_ADDRESS, buf);
    // debug end
    secureWrite(devif.ttyDev, buf, n);
  }
  if(tick1 == 87)
  { // get external control status
    char * buf = devif.txBuffer;
    n = packData(buf, 'g', FC_ADDRESS, NULL, 0);
    // debug
//     buf[n] = '\0';
//     printf(PLUGINNAME ": sending external control req to %d as '%s'\n", FC_ADDRESS, buf);
    // debug end
    secureWrite(devif.ttyDev, buf, n);
  }
  if(tick1 == 10 && tick < 200)
  { // redirect serial to FC
    int n;
    char * buf = devif.txBuffer;
    unsigned char dataReq = 0; // redirect to 0=FC, 1=MAG, 2=GPS
    n = packData(buf, 'u', NC_ADDRESS, &dataReq, 1);
    secureWrite(devif.ttyDev, buf, n);
    // debug
    buf[n] = '\0';
    printf(PLUGINNAME ": sending serial redirect request to FC as %s\n", buf);
    // debug end
  }
  if(tick1 == 45 && tick < 200)
  { // vesion number once
    int n;
    char * buf = devif.txBuffer;
    n = packData(buf, 'v', FC_ADDRESS, NULL, 0);
    secureWrite(devif.ttyDev, buf, n);
  }
  if(tick1 >= 50 && tick1 < 50+32 && tick < 140)
  { // send debug request of analog names - to ckeck i versions are compatible
    // these names must (should) match the hard coded names in RHD variables - see header-file
    int n;
    char * buf = devif.txBuffer;
    /*Send the Debug Request*/
    unsigned char dataReq = tick1 - 50;
    n = packData(buf, 'a', FC_ADDRESS, &dataReq, 1);
    buf[n] = '\0';
    // debug
//     buf[n] = '\0';
//     printf(PLUGINNAME ": sending label %d req to %d as '%s'\n", dataReq, NC_ADDRESS, buf);
    // debug end
    secureWrite(devif.ttyDev, buf, n);
  }
  return 0;
}

int initPlugin(){
    //Initserial will initialize the connection to the hexakopters
    initSerial();
    /*Initiate Gas as 1, to make sure the motors keeps going
     * Config must be 1 to control the Hexacopter throug
     * ExternControl*/
    memset(&externCtrl, 0, sizeof(externCtrl));
    externCtrl.Gas = 1;
    externCtrl.Config = 1;
    return 1;
}

int OpenAndSetMode(){
	int error;
	devif.ttyDev = open(devif.devName,O_RDWR);
	error = set_serial(devif.ttyDev,devif.baudrate); 
	if(error!=0){
		printf(PLUGINNAME "Error, Port could not open\n");
		exit(1);
	}
	else{
		printf(PLUGINNAME "Port open: %s BAUD: %d\n",devif.devName, devif.baudrate);
		return 1;
	}
}

int initSerial(void)
{ //Open serial port
  int result;
  //
  rxtask.running = 0;
  rxtask.startNewRxCycle = 0;
  result=OpenAndSetMode();
  //
  if (result == 1)
  { // start thread to handle bus
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&rxtask.serial_task, &attr, Hexacopter, 0))
    {
      perror(PLUGINNAME ": Can't start serial receive thread");
      result = 0;
    }
  }
  if (result == 1)
  { /****** Create database variables if all is ok **************/
    createRHDvariables();
    int waitCount = 0;
    while (!rxtask.running)
    { // wait a bit for thread to start
      usleep(20000); //Don't return before threads are running
      if (waitCount >= 50)
      {
        result = 0;
        break;
      }
      waitCount++;
    }
  }
  return result;
}

/** The Hexacopter thread reads the input recieved from the Hexacopter
 * First it waits for the carriage return after start byte (#)
 * then it sees if the package has the right command ID
 * If so it decodes the package and memcpy it into the 
 * DebugOut struct, and then updates the RHD variables.*/
void * Hexacopter(void * not_used){
  printf(PLUGINNAME " Read Thread started\n");
  rxtask.running = 1;
  unsigned char rx, end;
  struct timeval t;
  double now;
  while(rxtask.running)
  {
    rxBuffer[2] = '\0';
    rx = 0;
    end = 0;
    /*Recieve until carriage return*/
    while(rx!='\r')
    { // look for data from # to 'carriage return' character
      read(devif.ttyDev,&rx,1);
      rxBuffer[end] = rx;
      if (end > 0 || rx == '#')
        end++;
      if (end >= MK_MAX_RX_BUFFER)
      { // too long, must be garbage, restart
        end = 0;
        break;
      }
    }
    /*Check if the Command ID is D for Debug*/
    if(rxBuffer[2] == 'A')
    {
      int n;
      n = decode64(decoded, sizeof(decoded),3, end, rxBuffer);
      decoded[n] = '\0';
      rxBuffer[end] = '\0';
      printf(PLUGINNAME " %d analog %d: decoded '%s'\n", rxBuffer[1]-'a', decoded[0], &decoded[1]);
    }
    if(rxBuffer[2] == 'C')
    {
//      int n =
      decode64((unsigned char *)&data3D, sizeof(data3D),3, end, rxBuffer);
      setVariable(var.pose, 0, data3D.Centroid[0]);
      setVariable(var.pose, 1, data3D.Centroid[1]);
      setVariable(var.pose, 2, data3D.Centroid[2]);
      setVariable(var.pose, 3, data3D.Winkel[0]);
      setVariable(var.pose, 4, data3D.Winkel[1]);
      setVariable(var.pose, 5, data3D.Winkel[2]);
      // rate
      gettimeofday(&t, NULL);
      now = t.tv_sec + t.tv_usec * 1e-6;
      data3D.dataCnt++;
      if (now - data3D.rateTime > 1.0)
      {
        setVariable(var.poseRate, 0, data3D.dataCnt);
        data3D.dataCnt = 0;
        data3D.rateTime = now;
      }
      // debug
//       printf(PLUGINNAME ":: got a C message from %d: Winkel: %4d %4d %4d, centr: %4d %4d %4d (%d bytes)\n", rxBuffer[1] - 'a',
//              data3D.Winkel[0], data3D.Winkel[1], data3D.Winkel[2],
//              data3D.Centroid[0], data3D.Centroid[1], data3D.Centroid[2],
//              n);
      // debug end
    }
    if(rxBuffer[2] == 'G')
    { // external control status
//      int n =
      decode64((unsigned char *)&externCtrlGot, sizeof(externCtrlGot),3, end, rxBuffer);
      setVariable(var.externCtrlGot, 0, externCtrlGot.Nick);
      setVariable(var.externCtrlGot, 1, externCtrlGot.Roll);
      setVariable(var.externCtrlGot, 2, externCtrlGot.Gier);
      setVariable(var.externCtrlGot, 3, externCtrlGot.Gas);
      setVariable(var.externCtrlGot, 4, externCtrlGot.Hight);
      setVariable(var.externCtrlGot, 5, externCtrlGot.Config);
      setVariable(var.externCtrlGot, 6, externCtrlGot.Frame);
    }
    if(rxBuffer[2] == 'V')
    {
//      int n = 
      decode64((unsigned char *)&versionFC, sizeof(versionFC),3, end, rxBuffer);
      setVariable(var.versionFC, 0, versionFC.SWMajor);
      setVariable(var.versionFC, 1, versionFC.SWMinor);
      setVariable(var.versionFC, 2, versionFC.SWPatch);
      // debug
//       printf(PLUGINNAME ":: got a V message from %d: SW: %d.%d.%d, Proto %d.%d err %x %x %x %x %x (%d bytes)\n", rxBuffer[1] - 'a',
//             versionFC.SWMajor, versionFC.SWMinor, versionFC.SWPatch, versionFC.ProtoMajor, versionFC.ProtoMinor,
//             versionFC.HardwareError[0], versionFC.HardwareError[1], versionFC.HardwareError[2], versionFC.HardwareError[3],
//             versionFC.HardwareError[4], n);
      // debug end
    }
    if(rxBuffer[2] == 'D')
    { 
//      int n =
      decode64((unsigned char *)&DebugOut, sizeof(DebugOut),3, end, rxBuffer);
      // debug
//       rxBuffer[end] = '\0';
//       printf(PLUGINNAME ":: got a D message from %d '%s' (%d bytes)\n", rxBuffer[1] - 'a', rxBuffer, n);
      // debug end
      //memcpy(&DebugOut, decoded, sizeof(DebugOut));
      /*Update RHD*/
      setVariable(var.AccNick,0,DebugOut.AccNick);
      setVariable(var.AccRoll,0,DebugOut.AccRoll);
      setVariable(var.AccZ,0,DebugOut.AccZ);
      setVariable(var.AngleNick,0,DebugOut.AngleNick);
      setVariable(var.AngleRoll,0,DebugOut.AngleRoll);
      setVariable(var.batVoltage,0,DebugOut.batVoltage);
      setVariable(var.BL_Limit,0,DebugOut.BL_Limit);
      setVariable(var.Capacity,0,DebugOut.Capacity);
      setVariable(var.CompassSetpoint,0,DebugOut.CompassSetpoint);
      setVariable(var.CompassValue,0,DebugOut.CompassValue);
      setVariable(var.CPUOverLoad,0,DebugOut.CPUOverLoad);
      setVariable(var.Current,0,DebugOut.Current);
      setVariable(var.gas,0,DebugOut.Gas);
      setVariable(var.GPS_Nick,0,DebugOut.GPS_Nick);
      setVariable(var.GPS_Roll,0,DebugOut.GPS_Roll);
      setVariable(var.GyroCompass,0,DebugOut.GyroCompass);
      setVariable(var.HeightSetpoint,0,DebugOut.HeightSetpoint);
      setVariable(var.HeightValue,0,DebugOut.HeightValue);
      setVariable(var.Hovergas,0,DebugOut.Hovergas);
      setVariable(var.I2C_Error,0,DebugOut.I2C_Error);
      setVariable(var.Motor,0,DebugOut.Motor1);
      setVariable(var.Motor,1,DebugOut.Motor2);
      setVariable(var.Motor,2,DebugOut.Motor3);
      setVariable(var.Motor,3,DebugOut.Motor4);
      setVariable(var.Motor,4,DebugOut.Motor5);
      setVariable(var.Motor,5,DebugOut.Motor6);
      setVariable(var.Motor,6,DebugOut.Motor7);
      setVariable(var.Motor,7,DebugOut.Motor8);
      setVariable(var.ReceiverLevel,0,DebugOut.ReceiverLevel);
      setVariable(var.Servo,0,DebugOut.Servo);
      setVariable(var.YawGyro,0,DebugOut.YawGyro);
      // rate
      gettimeofday(&t, NULL);
      now = t.tv_sec + t.tv_usec * 1e-6;
      data3D.statDataCnt++;
      if (now - data3D.statDataTime > 1.0)
      {
        setVariable(var.statRate, 0, data3D.statDataCnt);
        data3D.statDataCnt = 0;
        data3D.statDataTime = now;
      }
    }
  }
  rxtask.running = 0;
  printf(PLUGINNAME " Read Thread ended\n");
  return NULL;
}

/************************** XML Initialization **************************/

/** \brief Initialize plug-in from XML configuration file
 *
 * Reads the XML file and initializes plugin
 * after successfully reading the XML file.
 *
 * \param[in] *char filename
 * Filename of the XML file
 *
 * \returns int status 1 is OK and -1 is error
 * Status of the initialization process. Negative on error.
 */
extern int initXML(char *filename)
{
    int result;
    parseInfo xmlParse;
    char *xmlBuf = NULL;
    int xmlFilelength;
    int done = 0;
    int len;
    FILE *fp;
    //Find revision number from SVN Revision
    char * p1;
    char versionString[20] = REVISION;
    char tempString[10];
    p1 = strrchr(versionString, '$');
    strncpy(tempString, &versionString[6],(p1 - versionString - 6));
    tempString[(p1 - versionString - 6)] = '\0';
    printf(PLUGINNAME ": Initializing plug-in version %s.%s\n",REVISION, tempString);
    /* Initialize Expat parser*/
    XML_Parser parser = XML_ParserCreate(NULL);
    result = parser != 0;
    devif.baudrate = 57600;
    strncpy(devif.devName, "/dev/ttyUSB0", MxDL);
    if (!result)
        fprintf(stderr, PLUGINNAME ": Couldn't allocate memory for XML parser\n");
    if (result)
    {  //Setup element handlers
        XML_SetElementHandler(parser, lsStartTag, lsEndTag);
        //Setup shared data
        memset(&xmlParse, 0, sizeof(parseInfo));
        //
        XML_SetUserData(parser, &xmlParse);

        //Open and read the XML file
        fp = fopen(filename,"r");
        result = fp != NULL;
        if(!result)
            printf(PLUGINNAME ": Error reading: %s\n",filename);
    }
    if (result)
    { //Get the length of the file
        fseek(fp,0,SEEK_END);
        xmlFilelength = ftell(fp); //Get position
        fseek(fp,0,SEEK_SET); //Return to start of file
        //Allocate text buffer for full file length
        xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
        result = (xmlBuf != NULL);
        if (!result)
            fprintf(stderr, "   hexakopter: Couldn't allocate memory for XML File buffer\n");
    }
    if (result)
    { // clear buffer
        memset(xmlBuf,0,xmlFilelength);
        // read full file
        len = fread(xmlBuf, 1, xmlFilelength, fp);
        fclose(fp);
        //Start parsing the XML file
        result = (XML_Parse(parser, xmlBuf, len, done) != XML_STATUS_ERROR);
        if (!result)
            fprintf(stderr, PLUGINNAME ": XML Parse error at line %d: %s\n",
                    (int)XML_GetCurrentLineNumber(parser),
                    XML_ErrorString(XML_GetErrorCode(parser)));
    }
    if (parser != NULL)
        XML_ParserFree(parser);
    if (xmlBuf != NULL)
        free(xmlBuf);
    if (result && xmlParse.enable)
    { // all is fine - start plugin
        result = initPlugin();
    }
    if (result)
        return 1;
    else
        return -1;
}
//////////////////////////////////////////////////////

int terminate(void)
{
    rxtask.running = 0;
    printf(PLUGINNAME ":: stopping ... ");
    fflush(stdout);
    pthread_join(rxtask.serial_task, NULL);
    close(devif.ttyDev);
    printf("[OK]\n");
    return 0;
}
//////////////////////////////////////////////////

/**
 * A start tag is detected by the XML parser
 \param data is a user defined context pointer.
 \param el is the tag name
 \param attr is the list of attributes in the start tag. */
void XMLCALL lsStartTag(void *data, const char *el, const char **attr)
{ // a start tag is detected
    int i;
    parseInfo *info = (parseInfo *) data;
    // detect context
    info->depth++;
    if (info->depth < info->skip || info->skip == 0)
    {
        switch (info->depth)
        {
        case 1:
            if (strcmp("rhd",el) != 0)
                info->skip = info->depth;
            break;
        case 2:
            if (strcmp("plugins",el) == 0)
                ;
            else
                info->skip = info->depth;
            break;
        case 3:
            // this one handles hexakopter stuff only
            if (strcmp("hexakopter",el) == 0)
            { // is it enabled, the only info needed
                for(i = 0; attr[i]; i+=2)
                {
                    const char * att, * val;
                    att = attr[i];
                    val = attr[i + 1];
                    if ((strcmp("enable",att) == 0) && (strcmp("true",val) == 0))
                        info->enable = 1;
                    else if (strcasecmp("dev", att) == 0)
                    {
                        strncpy(devif.devName, val, MxDL);
                    }
                    else if (strcasecmp("baudRate", att) == 0)
                    {
                        devif.baudrate = strtol(val, NULL, 10);
                    }
                    else if (strcmp("debug", att) == 0)
                    {
                        debugFlag = strtol(val, NULL, 0);
                        if (debugFlag)
                            printf(PLUGINNAME ": started in DEBUG mode!\n");
                    }
                }
                if (!info->enable)
                    printf(PLUGINNAME ": Use is disabled in configuration\n");
            }
            else
                info->skip = info->depth;
            break;
        default: // unknown tag series
            break;
        }
    }
}

///////////////////////////////////////////////////////

void XMLCALL lsEndTag(void *data, const char *el)
{
    parseInfo *info = (parseInfo *) data;
    //  printf("endtag %s changed depth from %d", el, info->depth);
    info->depth--;
    if (info->depth < info->skip)
        // back to normal search for valid tags
        info->skip = 0;
    //  printf(" to %d\n", info->depth);
}
void createRHDvariables()
{ // RHD variables
  var.versionFC = createVariable('r',3,"versionfc");

  var.nick    =   createVariable('w',1,"nickref");
  var.roll    =   createVariable('w',1,"rollref");
  var.yaw     =   createVariable('w',1,"yawref");
  var.gasref  =   createVariable('w',1,"thrustref");
  var.height  =   createVariable('w',1,"heightref");
  var.Config  =   createVariable('w',1,"control");

  var.AngleNick     =   createVariable('r',1,"anglenick");
  var.AngleRoll     =   createVariable('r',1,"angleroll");
  var.AccNick       =   createVariable('r',1,"accnick");
  var.AccRoll       =   createVariable('r',1,"accroll");
  var.YawGyro       =   createVariable('r',1,"yawgyro");
  var.HeightValue   =   createVariable('r',1,"height");
  var.AccZ          =   createVariable('r',1,"accz");
  var.gas           =   createVariable('r',1,"gas");
  var.CompassValue  =   createVariable('r',1,"compas");
  var.batVoltage    =   createVariable('r',1,"batvoltage");
  var.ReceiverLevel =   createVariable('r',1,"receiverlevel");
  var.GyroCompass   =   createVariable('r',1,"gyrocompass");
  var.Motor         =   createVariable('r',8,"motor");
  var.Servo         =   createVariable('r',1,"servo");
  var.Hovergas      =   createVariable('r',1,"hovergas");
  var.Current       =   createVariable('r',1,"current");
  var.Capacity      =   createVariable('r',1,"capacity");
  var.HeightSetpoint=   createVariable('r',1,"heightsetpoint");
//  var.analog25  =   createVariable('r',1,"");
  var.CPUOverLoad   =   createVariable('r',1,"cpuoverload");
  var.CompassSetpoint  =   createVariable('r',1,"compasssetpoint");
  var.I2C_Error     =   createVariable('r',1,"i2cerror");
  var.BL_Limit      =   createVariable('r',1,"bllimit");
  var.GPS_Nick      =   createVariable('r',1,"gpsnick");
  var.GPS_Roll      =   createVariable('r',1,"gpsroll");
  var.statRate      =   createVariable('r',1,"statusRate");

  var.pose          = createVariable('r', 6,"pose3d");
  var.poseRate      = createVariable('r', 1,"pose3drate");
  var.externCtrlGot = createVariable('r', 7, "externalctrl"); /// nick,roll,yaw,trust,height,control,frame
}

////////////////////////////////////////////////////

char limitSignedChar(int value, int min, int max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}

////////////////////////////////////////////////////

int limitInt(int value, int min, int max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}

////////////////////////////////////////////////////

unsigned char limitUnsignedChar(int value, int min, int max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}



