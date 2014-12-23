/** \file gps.h
 *  \ingroup hwmodule
 *  \brief Serial SLUGS Module
 *
 * SLUGS Module for RHD. 
 * 
 * Based on HAKOD by Asbjørn Mejnertsen and Anders Reeske Nielsen
 * and AuGps by Lars Mogensen and Christian Andersen
 * GPS module by Anders Billesø Beck
 *
 *  \author Jonathan Løwert
 *  $Rev: 59 $
 *  $Date: 2009-12-21 14:12:33 +0200 (Mon, 21 Dec 2009) $
 */
 /***************************************************************************
 *                  Copyright 2009 Jonathan Løwert                         *
 *                       jonathan@lowert.dk                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/* ==================================================
This is the definition file for the the UCSC AP code
it creates all the common defines, unions, enumerations
and data types.

 Code by: Mariano I. Lizarraga
 First Revision: Aug 18 2008 @ 17:42
 ====================================================*/


#ifndef SLUGS_H
  #define SLUGS_H

int initXML(char *filename);





#ifdef __cplusplus
       extern "C"{
#endif

// =========== Global Definitions ==========

// Circular Buffer Size
// ===================
#ifdef __cplusplus
       #define BSIZE			1024
#else
       #define BSIZE			512
#endif

// GPS Checksum Messages
// =====================
#define GGACS			86
#define RMCCS			75

// GPS Header IDs
// ==============
#define GGAID			1
#define RMCID			2
#define UNKID			254

// GPS Circular Buffers
// ====================
#define MSIZE			150
//#define CSIZE			26 //[newBytes payload remaingBytes]  (comms buffer out of readGPS)

// Data Logger Values
// ================
#define LOGSEND					8
#define MAXSEND					103
#define MAXLOGSEND				56

#ifdef __cplusplus
       #define MAXLOGLEN		500
#else
       #define MAXLOGLEN		99
#endif

// Define log raw data at 100 hz. Comment out to have
// XYZ data come at 100 Hz instead. COMMENT not Change to 0 (using #ifdef)
//#define LOGRAW100	1

// Define diagnostic data at 100 hz. Comment out to have
// XYZ data come at 100 Hz instead. COMMENT not Change to 0 (using #ifdef)
#define DIAG100		1

// Message Protocol Lengths and IDs
// ================================

// SENSOR MCU
// ==========
#define GPS_START 		0
#define GPSMSG_ID		1
#define GPSMSG_LEN		27 

#define LOAD_START		27
#define LOADMSG_ID		2
#define LOADMSG_LEN		4

#define RAW_START 		31
#define RAWMSG_ID		3
#define RAWMSG_LEN		26

#define ATT_START		49
#define ATTMSG_ID		4
#define ATTMSG_LEN		26

#define XYZ_START		75
#define XYZMSG_ID		11
#define XYZMSG_LEN		24

#define DYN_START		99
#define DYNMSG_ID		5
#define DYNMSG_LEN		10

#define BIA_START		109
#define BIAMSG_ID		6
#define BIAMSG_LEN		24

#define DIA_START		133
#define DIAMSG_ID		7
#define DIAMSG_LEN		18

#define HIL_START		151

#define PIL_START		152
#define PILMSG_ID		9
#define PILMSG_LEN		10

#define SENMSG_ID		25
#define SENMSG_LEN		24

#define LOGMSG_ID		26
#define LOGMSG_LEN	24

// CONTROL MCU
// ===========
#define AKNMSG_ID		105
#define AKNMSG_LEN		6

#define PWMMSG_ID		100
#define PWMMSG_LEN		20 

#define CALMSG_ID		102
#define CALMSG_LEN		17

#define APSMSG_ID		101
#define APSMSG_LEN		20

#define NAVMSG_ID		120
#define	NAVMSG_LEN	30





// Identifier values for messages that have a type ID
// ==================================================

// Calibration and Query type IDs
#define PIDTYPE_ID		1
#define WPSTYPE_ID		2
#define PASTYPE_ID		3
#define COMTYPE_ID		4

// Control Types
#define CTRL_TYPE_MANUAL	1		// Pilot in full control of aircraft
#define CTRL_TYPE_AP_COMM	2		// Low level commands: airspeed, height and turnrate
#define CTRL_TYPE_AP_WP		3		// Waypoint following
#define CTRL_TYPE_PASS		4		// Passthrough the commands from the pilot console
#define CTRL_TYPE_SEL_PIL	5		// Pass some from the pilot and some from the AP
#define CTRL_TYPE_SEL_AP	6

#define PIL_FAILSAFE		6000	// IC direct reading from the pilot console when failsafe is
									// is turned ON i.e. the pilot is in control. 

// Commands to AP ids
#define COMM_TYPE_HEIGHT	1
#define	COMM_TYPE_TURNRATE	2
#define	COMM_TYPE_AIRSPEED	3
#define COMM_TYPE_GOTO_WP	4



// ERROR MESSAGES
// ==============

// PID EEPROM Error Messages
#define PIDEEP_WRITE_FAIL	11
#define PIDEEP_PAGE_EXP		12
#define PIDEEP_MEMORY_CORR	13

// WP EEPROM Error Messages
#define WPSEEP_WRITE_FAIL	21
#define WPSEEP_PAGE_EXP		22
#define WPSEEP_MEMORY_CORR	23

// EEPROM Emulation Address Offsets
// ================================
#define PID_OFFSET		0
#define WPS_OFFSET		60

// Communication Protocol Merging Offsets
// ======================================
#define GSMSG_IDX		99
#define AKMSG_IDX		202


// Standard characters used in the parsing of messages
// ===================================================
#define DOLLAR			36
#define STAR			42
#define CR				13
#define LF				10
#define AT				64

// Interprocessor Communication
// ============================
#define BEGINSPI		0xFFFF
#define ENDSPI			0xBEEF
#define SPIBUFSIZE		46

// Standard Units
// ==============
#define KTS2MPS 		0.514444444
#define PI              3.141592653589793
#define DEG2RAD			0.017453292519943
#define RAD2DEG			57.29577951308232

// Periphereal Configurations
// ==========================
#define APFCY			40000000

	
#define GPSBAUDF		19200
#define GPSBAUDI		4800
#define UCSCAP_UBRGF 	129
#define UCSCAP_UBRGI 	520

#define LOGBAUD			115200
#define LOG_UBRG		21

#define LOGMAT_BAUD		57600
#define LOGOMAT_UBRG	42


// ifdef switches for debugging and conditional inclusion
// ======================================================
#define __IN_DSPIC__ 	1 // switch for use in PC

#if __IN_DSPIC__
	#ifdef DEBUG
		#undef DEBUG
	#endif	
#else
	#define DEBUG 1
#endif

// Uncomment if there is no magentometers
//#define NO_MAGNETO 

// Uncomment to allow full gyro calibration
#define DO_FULL_CALL


/*// ============= Unions Used for Data Transmission ====
//Type definitions for standard unions used in sending
// and receiving data
typedef union{
	unsigned char    chData[2];
	unsigned short   usData;
} tUnsignedShortToChar; 

typedef union{
	unsigned char    chData[2];
 	short   		 shData;
} tShortToChar; 

typedef union{
	unsigned char   chData[4];
 	unsigned int   	uiData;
} tUnsignedIntToChar; 

typedef union{
	unsigned char   chData[4];
 	int   			inData;
} tIntToChar; 

typedef union{
	unsigned char   chData[4];
 	float   		flData;
	unsigned short	shData[2];
} tFloatToChar; 

// ============= Structures used for data ============
typedef struct tGpsData{
	unsigned char	 		year;
	unsigned char			month;
	unsigned char			day;
	unsigned char			hour;
	unsigned char			min;
	unsigned char			sec;	 
	tFloatToChar 			lat;
	tFloatToChar 			lon;
	tFloatToChar 			height;
	tUnsignedShortToChar	cog;
	tUnsignedShortToChar	sog;
	tUnsignedShortToChar	hdop;	
	unsigned char			fix;
	unsigned char 			sats;	
	unsigned char			newValue;
}tGpsData;

typedef struct tRawData{
	tShortToChar 	gyroX;
	tShortToChar 	gyroY;
	tShortToChar 	gyroZ;
	tShortToChar 	accelX;
	tShortToChar 	accelY;
	tShortToChar 	accelZ;
	tShortToChar 	magX;
	tShortToChar 	magY;
	tShortToChar 	magZ;
	// included in SLUGS MKII
	tShortToChar 	baro;
	tShortToChar 	pito;
	tShortToChar 	powr;
	tShortToChar 	ther;
}tRawData;

typedef struct tAttitudeData{
	tFloatToChar			roll;
	tFloatToChar			pitch;
	tFloatToChar			yaw;
	tFloatToChar			p;
	tFloatToChar			q;
	tFloatToChar			r;
	tUnsignedShortToChar	timeStamp;
}tAttitudeData;

typedef struct tSensStatus{
	char					load;
	char					vdetect;
	tUnsignedShortToChar	bVolt;
}tSensStatus;

typedef struct tDynTempData{
	tFloatToChar	dynamic;
	tFloatToChar	stat;
	tShortToChar	temp;
}tDynTempData;

typedef struct tBiasData{
	tFloatToChar			axb;
	tFloatToChar			ayb;
	tFloatToChar			azb;
	tFloatToChar			gxb;
	tFloatToChar			gyb;
	tFloatToChar			gzb;	
}tBiasData;

typedef struct tDiagData{
	tFloatToChar			fl1;
	tFloatToChar			fl2;
	tFloatToChar			fl3;
	tShortToChar			sh1;
	tShortToChar			sh2;
	tShortToChar			sh3;	
}tDiagData;

typedef struct tXYZData{
	tFloatToChar	Xcoord;
	tFloatToChar	Ycoord;
	tFloatToChar	Zcoord;	
	tFloatToChar	VX;
	tFloatToChar	VY;
	tFloatToChar	VZ;	
}tXYZData;

typedef struct tAknData{
	unsigned char 	WP;
	unsigned char 	commands;
	unsigned char 	pidCal;
	unsigned char 	sensorReboot;
	unsigned char 	filOnOff;	
	unsigned char	reboot;
}tAknData;

typedef struct tPilotData{
	tUnsignedShortToChar	dt ;
	tUnsignedShortToChar	dla;
	tUnsignedShortToChar	dra;
	tUnsignedShortToChar	dr ; 
	tUnsignedShortToChar	de ; 
}tPilotData;

typedef struct tPWMData{
	tUnsignedShortToChar	dt_c;
	tUnsignedShortToChar	dla_c;
	tUnsignedShortToChar	dra_c;
	tUnsignedShortToChar	dr_c; 
	tUnsignedShortToChar	dle_c; 
	tUnsignedShortToChar	dre_c; 
	tUnsignedShortToChar	dlf_c; 
	tUnsignedShortToChar	drf_c; 
	tUnsignedShortToChar	da1_c; 
	tUnsignedShortToChar	da2_c; 
}tPWMData;

typedef struct tPIDData{
	tFloatToChar   P[10];
	tFloatToChar   I[10];
	tFloatToChar   D[10];	
}tPIDData;


typedef struct tQueryData{
	unsigned char 	pendingRequest;
	unsigned char 	idReq;
	unsigned char 	indxReq;
}tQueryData;

typedef struct tWPData{
	tFloatToChar	lat[11];
	tFloatToChar	lon[11];
	tFloatToChar	hei[11];
	unsigned char	typ[11];
	tShortToChar	val[11];
	unsigned char	wpCount;
}tWPData;

typedef struct tCommandsData{
	tFloatToChar	hCommand;
	tFloatToChar	airspeedCommand;
	
	tFloatToChar	phiCommand;
	tFloatToChar	thetaCommand;
	tFloatToChar	psiCommand;
	
	tFloatToChar	pCommand;
	tFloatToChar	qCommand;
	tFloatToChar	rCommand;
	
	char			currWPCommand;
	char			nextWPCommand;	
}tCommandsData;

typedef struct tAPStatusData{
	unsigned char	controlType;
	unsigned char	beaconStatus;
	unsigned char hilStatus;
	
	unsigned char	dt_pass; 
	unsigned char	dla_pass;
	unsigned char	dra_pass;
	unsigned char	dr_pass; 
	unsigned char	dle_pass;
	unsigned char dre_pass;
	unsigned char	dlf_pass;
	unsigned char	drf_pass;
}tAPStatusData;

typedef struct tCubeBuffer {
  tShortToChar  ax[4];
  tShortToChar  ay[4];
  tShortToChar  az[4];
  tShortToChar  gx[4];
  tShortToChar  gy[4];
  tShortToChar  gz[4];
  unsigned char sampleCount;
}tCubeBuffer;

typedef struct tNavData {
	tFloatToChar	uMeasured;
	tFloatToChar	thetaCommanded;
	tFloatToChar	psiDotCommanded;
	tFloatToChar	phiCommanded;
	tFloatToChar	rHighPass;
	tFloatToChar	totRun;
	tFloatToChar	distance2Go;
	unsigned char	fromWp;
	unsigned char toWp;	
}tNavData;

typedef struct tSensData{
	tFloatToChar	Ax;
	tFloatToChar	Ay;
	tFloatToChar	Az;
	tFloatToChar	Mx;
	tFloatToChar	My;
	tFloatToChar	Mz;
}tSensData;

typedef struct tLogFloats{
	tFloatToChar	fl1;
	tFloatToChar	fl2;
	tFloatToChar	fl3;
	tFloatToChar	fl4;
	tFloatToChar	fl5;
	tFloatToChar	fl6;
}tLogFloats;
*/

//----------CircBuffer.h------------------

# if __IN_DSPIC__
	typedef struct CircBuffer{
		unsigned char buffer[BSIZE];
		int head;
		int tail;
		unsigned int size;
		unsigned char overflowCount;
	}CircBuffer;
	
#else
	typedef struct CircBuffer{
		unsigned char* buffer;
		int head;
		int tail;
		unsigned int size;
		unsigned char overflowCount;
	}CircBuffer;
#endif

// Exported Types
// ==============
typedef struct CircBuffer* CBRef;
	
// Constructors - Destructors
// ==========================
// this Function returns a pointer to a new Circular Buffer of size pm_size 

#if __IN_DSPIC__
	void newCircBuffer (CBRef cB);
#else
	CBRef newCircBuffer (int pm_size);
#endif

// this function frees the Circular Buffer CB Ref
void freeCircBuffer (CBRef* cB);


// Accesor Methods
// ===============

// returns the amount of unread bytes in the circular buffer
unsigned int getLength (CBRef cB);

// returns the actual index of the head
int readHead (CBRef cB);

// returns the actual index of the tail
int readTail (CBRef cB);

// returns the byte (actual value) that the head points to. this
// does not mark the byte as read, so succesive calls to peak will
// always return the same value
unsigned char peak(CBRef cB);


// Manipulation Procedures
// ======================
// returns the front of the circular buffer and marks the byte as read
unsigned char readFront (CBRef cB);

// writes one byte at the end of the circular buffer, returns 1 if overflow occured
unsigned char writeBack (CBRef cB, unsigned char data);

// empties the circular buffer. It does not change the size. use with caution!!
void makeEmpty (CBRef cB);

// returns the amount of times the CB has overflown;
unsigned char getOverflow(CBRef cB);


//Convertions

// ============= Unions Used for Data Transmission ====
//Type definitions for standard unions used in sending
// and receiving data
typedef union{
	unsigned char    chData[2];
	unsigned short   usData;
} tUnsignedShortToChar; 

typedef union{
	unsigned char    chData[2];
 	short   		 shData;
} tShortToChar; 

typedef union{
	unsigned char   chData[4];
 	unsigned int   	uiData;
} tUnsignedIntToChar; 

typedef union{
	unsigned char   chData[4];
 	int   			inData;
} tIntToChar; 

typedef union{
	unsigned char   chData[4];
 	float   		flData;
	unsigned short	shData[2];
} tFloatToChar; 


#if DEBUG
	// Other Functions
	// ===============
	// prints the circular buffer, used for debug
	void printCircBuf(CBRef cB);
#endif /* DEBUG */


#ifdef __cplusplus
      }
#endif


#endif

