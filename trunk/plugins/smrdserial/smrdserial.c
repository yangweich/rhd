/************************ SMRD Bus driver **************************/
 /** \file smrdSerial.c
 *  \ingroup hwmodule
 *
 *   Old SMRD interface - used for fast prototyping
 *******************************************************************/
/***************************** Plugin version  *****************************/
#define SMRDVERSION     	"1.0"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2012-01-14 09:49:21 +0100 (Sat, 14 Jan 2012) $:"
 #define ID               "$Id: smrdserial.c 59 2012-10-21 06:25:02Z jcan $"
/***************************************************************************/

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

//RHD Core headers
#include <rhd.h>
#include <smr.h>
#include <database.h>
#include <globalfunc.h>

#include "smrdserial.h"


/******** Global variables *************/
int iEncl, iEncr, iLs, iIr, iGyro, iGyroTemp, iSpl, iSpr,iSteeringangleref,iRstl,iRstr;
uint8_t txBuf[32];
int iterator;
int32_t inputRS485[32], inputRS232[32];
symTableElement *writeTable;
int rs485, rs232;  //File descriptors
int smrdRunning = -1;
char serial1Dev[64], serial2Dev[64];
int baudrate1, baudrate2;

//SMRD Functions
int initSmrd(void);
void smrd_print_sched(void);
void *rs485_task(void *), *rs232_task(void *);
pthread_t rs485_thread, rs232_thread;
pthread_attr_t attr;

#define BLOCK_MAX 200
#define XMIT_BYTES 104    /* about 10ms at 115.2 Kbaud */

/// Init SMRD
int initSmrd(void) {

  //Configure serial ports
    int ttyS0, ttyS1;

   //Open first serial port
   if ((ttyS0 = open (serial1Dev, O_RDWR /*| O_NONBLOCK*/)) == -1) {
     fprintf(stderr,"   smrdSerial: Can't open first serial port: %s\n",serial1Dev);
      smrdRunning = -1;
      return -1;
   } else if (set_serial(ttyS0,baudrate1) == -1) {
    fprintf(stderr,"   smrdSerial: Can't set first serial port parameters\n");
    fprintf(stderr,"   smrdSerial: SMRD is NOT running!\n");
    smrdRunning = -1;
    return -1;
   } else smrdRunning = 1;
 
  //Open second serial port
   if ((ttyS1 = open (serial2Dev, O_RDWR /*| O_NONBLOCK*/)) == -1)  {
    fprintf(stderr,"   smrdSerial:can't open second serial port\n");
   } else if (set_serial(ttyS1,baudrate2) == -1) {
      fprintf(stderr,"   smrdSerial:Can't set second serial port parameters\n");
      close(ttyS1);
      ttyS1 = -1;
   }
   if (ttyS1 == -1) {
     rs485 = ttyS0;
     rs232 = -1;
   } else {
    rs485 = ttyS1;
    rs232 = ttyS0;
   }
  

  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED); 
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  if (pthread_create(&rs485_thread, &attr, rs485_task, 0))
    {
      perror("   smrdSerial: Can't start RS485 receive thread");
      smrdRunning = -1;
      return -1;
    }

  if (rs232 != -1)
    if (pthread_create(&rs232_thread, &attr, rs232_task, 0)) {
      perror("   smrdSerial: Can't start RS232 linesensor receive thread");
      return -1;
    }

  /****** Create database variables if all is ok **************/
  iEncl = createVariable('r',1,"encl");
  iEncr = createVariable('r',1,"encr");
  iLs   = createVariable('r',8,"linesensor");
  iIr   = createVariable('r',6,"irsensor");
  iGyro = createVariable('r',3,"gyro");
  iGyroTemp = createVariable('r',3,"gyrotemp");
  iSpl  = createVariable('w',1,"speedl");
  iSpr  = createVariable('w',1,"speedr");
  iRstl  = createVariable('w',1,"resetmotorl");
  iRstr  = createVariable('w',1,"resetmotorr");
  iSteeringangleref  = createVariable('w',1,"steeringangleref");

  
  //Get table pointers
  writeTable = getSymtable('w');

  usleep(100000); //Don't return before threads are running

  return 0;
}

void smrd_print_sched()
{
  struct sched_param param;

  switch (sched_getscheduler(0))
    {
    case SCHED_OTHER:
      fprintf(stderr, "Scheduler: other\n");
      break;
    case SCHED_FIFO:
      sched_getparam(0, &param);
      fprintf(stderr, "   smrdSerial: Scheduler fifo, priority %d\n", param.sched_priority);
      break;
    case SCHED_RR:
      sched_getparam(0, &param);
      fprintf(stderr, "   smrdSerial: Scheduler rr, priority %d\n", param.sched_priority);
      break;
    default:
      perror("getscheduler");
      break;
    }
}


static int msg_read(int fd, unsigned char *p)
{
  int n, c, ret;

  n = read(fd, p, 1);
  if (n <= 0) return n;
  ret = ENET_BYTES(p);
  c = ret - 1;
  p++;
  while (c)
    {
      n = read(fd, p, c);
      if (n <= 0) return n;
      p += n;
      c -= n;
    }
  return ret;
}

static int msg_write(int fd, unsigned char *p)
{
  int c = ENET_BYTES(p);
  //char ret;
  /*ret =*/ write(fd, p, c);
  return c;
}

static int rs485_pad(int c)
{
  int i;
  char z = 0; //, ret;
  for (i = 0; i < c; i++)
    /*ret =*/ write(rs485, &z, 1);
  return c;
}

///RS485 Recieve thread
void *rs485_task(void *not_used)
{
  fprintf(stderr, "   smrdSerial: RS485 rx_task running\n");

  if (mlockall(MCL_CURRENT | MCL_FUTURE))
    {
      perror("mlockall");
      exit(-1);
    }
  { /* use real-time (fixed priority) scheduler
     * set priority to one less than the maximum
     */
    struct sched_param param;

    param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
    if (sched_setscheduler(0, SCHED_RR, &param))
      {
  perror("setscheduler");
  exit(-1);
      };

  }

  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, "signal: can't ignore SIGPIPE.\n");

  tcflush(rs485, TCIFLUSH);

  //Wait to make sure variables are created
  usleep(100000);

  while (1)
    {
      unsigned char buf[ENET_BYTES_MAX];
      int n;

      if ((n = msg_read(rs485, buf)) < 0) perror("Smrd: Error reading RS485");

      //Parse input packages
      switch(buf[1]) {
        case SMR_LEFT_ENCODER_RET:
          setVariable(iEncl, 0, ((int)buf[2] << 8) + buf[3]);
          break;
        case SMR_RIGHT_ENCODER_RET:
          setVariable(iEncr, 0, -((int)(buf[2] << 8) + buf[3]));
          break;
        case SMR_LS_RET:
          if (ENET_BYTES(buf) >= 9)
          {
            int len,i;
            len=ENET_BYTES(buf);
            for (i = 0; i < len-1; i++)  inputRS485[i] = buf[2+i];
            setArray(iLs, SMR_LS_N, inputRS485);
          }
          break;
        case SMR_IR_RET:
          if (ENET_BYTES(buf) >= 1+SMR_IR_N)  {
            int len,i;
            len=ENET_BYTES(buf);
            for (i = 0; i < len-1; i++)  inputRS485[i] = buf[2+i];
            setArray(iIr, SMR_IR_N, inputRS485);
          }
          break;
        case SMR_GYRO_POS_RET:
          if (ENET_BYTES(buf) >= 7)  {
            int /*len,*/i;
            //len=ENET_BYTES(buf);
            for (i = 0; i < 3; i++)  inputRS485[i] = (((int)buf[2+(i*2)] << 8) + buf[3+(2*i)]);
            setArray(iGyro, 3, inputRS485);
            break;
          }
        case SMR_GYRO_RATE_RET: //Sorry, bug in gyro, returns rate-header with temperature
          if (ENET_BYTES(buf) >= 7)  {
            int /*len,*/ i;
            //len=ENET_BYTES(buf);
            for (i = 0; i < 3; i++)  inputRS485[i] = (((int)buf[2+(i*2)] << 8) + buf[3+(2*i)]);
            setArray(iGyroTemp, 3, inputRS485);
          }
          break;
        default : break;
      }

#define POWER_OFF_COUNT 25

      if ((buf[0] == 0x08) && (buf[1] == 0x19)) /* power supply data message */
  {
    static int power_off = 0;

    if (power_off != -1)
      {
        if (buf[2] & 0x04)  /* power switch on */
    {
      if (power_off > 0)
        fprintf(stderr, "Reseting power_off (%d)!\n", power_off);

      power_off = 0;
    }
        else    /* power switch off */
    {
      power_off++;
      if (power_off == POWER_OFF_COUNT)
        {
          fprintf(stderr, "Power_off count reached limit (%d).\n",
            POWER_OFF_COUNT);

          power_off = -1;
          //char ret;
          /*ret =*/ system("/sbin/poweroff &");
        }
    }
      }
  }
    }
}


#define LS_READ_BYTES 20
char rs232_command = '?';

void rs232_send(unsigned char *p)
{
  rs232_command = p[1];

  if (write(rs232, &rs232_command, 1) != 1)
    {
      perror("rs232_send");
    }
}

///RS232  Recieve thread
void *rs232_task(void *not_used)
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE))
    {
      perror("mlockall");
      exit(-1);
    }

  { /* use real-time (fixed priority) scheduler
     * set priority to one less than the maximum
     */
    struct sched_param param;

    param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
    if (sched_setscheduler(0, SCHED_RR, &param))
      {
  perror("setscheduler");
  exit(-1);
      }

  }

  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, "signal: can't ignore SIGPIPE.\n");

  fprintf(stderr, "   smrdSerial: RS232 LS rx_task running\n");

    //Wait to make sure variables are created
  usleep(100000);
  while (1)
    {
      unsigned char buf[ENET_BYTES_MAX];

      int c = read(rs232, buf + 2, LS_READ_BYTES);
      if (c <= 0)
  {
    fprintf(stderr, "error reading rs232");
    pthread_exit(0);
  }
      buf[0] = (c + 1) | ENET_TYPE_232;
      buf[1] = rs232_command;
      if ((ENET_BYTES(buf) >= 2+SMR_LS_N) && (buf[1] == 's')
      && !memchr(buf+2, 0, SMR_LS_N)) /* filter out zero data */
      {
        int len,i;
        len=ENET_BYTES(buf);
        for (i = 0; i < len-1; i++)  inputRS485[i] = buf[2+i];
        setArray(iLs, SMR_LS_N, inputRS485);
    }
    }
}

///Transmit all data to serial bus (called periodically)
extern int periodic(int tick)
{
  int rs232_flag = 0;
  int xcount = 0;
  writeTable = getSymtable('w');
  int tempData;

  if (smrdRunning < 0) return -1;

  //Send motor commands to the bus;
  if (isUpdated('w',iSpl)) {
    tempData = getWriteVariable(iSpl,0);
    tempData = (tempData == -128) ? -127 : tempData;
    txBuf[0] = 2;
    txBuf[1] = 0x11;
    txBuf[2] = tempData;   
    xcount += msg_write(rs485, txBuf);
  }
  if (isUpdated('w',iSpr)) {
    tempData = - getWriteVariable(iSpr,0);
    tempData = (tempData == -128) ? -127 : tempData;
    txBuf[0] = 2;
    txBuf[1] = 0x12;
    txBuf[2] = tempData;   
    xcount += msg_write(rs485, txBuf);
  }
  if (isUpdated('w',iSteeringangleref)) {
    tempData = getWriteVariable(iSteeringangleref,0);
    tempData = (tempData == -128) ? -127 : tempData;
    txBuf[0] = 3;
    txBuf[1] = 0xF1;
    txBuf[2] = 0x0B;
    txBuf[3] = tempData;
    xcount += msg_write(rs485, txBuf);
  }
  if (isUpdated('w',iRstl)) {
    txBuf[0] = 1;
    txBuf[1] = 0x01;
    xcount += msg_write(rs485, txBuf);
  }
  if (isUpdated('w',iRstr)) {
    txBuf[0] = 1;
    txBuf[1] = 0x02;
    xcount += msg_write(rs485, txBuf);
  }

  /*** Request all variables from the bus ***/
  if ((rs232 != -1) && !rs232_flag)
  {       /* get line sensor data */
    static unsigned char msg[] = {ENET_TYPE_232 | 1, 's'};
    rs232_send(msg);
  }
  {       /* get right encoder */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, 0x21};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(10);
  }
  {       /* get left encoder */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, 0x22};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(10);
  }
  {       /* get (rs485) linesensor */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, 0x17};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(16);
  }
  {       /* get proximity encoders */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, 0x88};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(16);
  }
  {       /* get power status */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, 0x19};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(16);
  }
  {       /* get Beck-Gyro Position */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, SMR_GYRO_POS_RET};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(7);
  }
  {       /* get Beck-Gyro Temperature */
    //static unsigned char msg[] = {ENET_TYPE_485 | 1, SMR_GYRO_TEMP_RET};
    //xcount += msg_write(rs485, msg);
    //xcount += rs485_pad(7);
  }
  
  //Pad the bus with the remaining bytes
    int x = XMIT_BYTES - xcount;
    if (x < 0)
      fprintf(stderr, "xmit overflow %d\n", -x);
    else
      rs485_pad(x);

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
void XMLCALL smrdStartTag(void *, const char *, const char **);
void XMLCALL smrdEndTag(void *, const char *);

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
  printf("smrdSerial: Initializing SMRD Serial bus HAL%s. %s\n",SMRDVERSION,tempString);


   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "smrdSerial: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   //Setup element handlers
   XML_SetElementHandler(parser, smrdStartTag, smrdEndTag);
   //Setup shared data
   memset(&xmlParse,0,sizeof(parseInfo));
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("smrdSerial: Error reading: %s\n",filename);
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
    fprintf(stderr, "smrdSerial: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
	free(xmlBuf);

	//Print error, if no XML tag found
	if (xmlParse.found <= 0) {
		printf("   Error: No <smrd> XML tag found in plugins section\n");
		return -1;
	}

  //Start crossbow thread after init
  if (xmlParse.enable) done = initSmrd();



 return done;
}

void XMLCALL
smrdStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
 				((info->depth == 3) && (strcmp("smrd",el) != 0))) {
      info->skip = info->depth;
      return;
    } else if (info->depth == 3) info->found = 1;
  } else return;

  //Branch to parse the elements of the XML file.
  if (!strcmp("smrd",el)) {
    //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) {
      info->enable = 1; 
    }
    if (!info->enable) {
      printf("   smrdSerial: Use of SMRD disabled in configuration\n"); 
      info->skip = info->depth;
    }
  } else if (strcmp("serial1",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(serial1Dev,attr[i+1],63);
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate1 = atoi(attr[i+1]);  
    printf("   smrdSerial: RS485 port %s at %d baud\n",serial1Dev,baudrate1);
  } else if (strcmp("serial2",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 4) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(serial2Dev,attr[i+1],63);
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate2 = atoi(attr[i+1]);  
    printf("   smrdSerial: RS232 port %s at %d baud\n",serial2Dev,baudrate2);
  }

}

void XMLCALL
smrdEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}


