/** \file hakocan.h
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
 *  $Rev: 203 $
 *  $Date: 2013-12-12 14:01:25 +0100 (Thu, 12 Dec 2013) $
 *  
 */

#ifndef HAKOCAN_H
  #define HAKOCAN_H

	extern int initXML(char *);
	extern int periodic(int);
	//extern ”C” int terminate (void) ; //No shutdown function
	#define KEEPAUTOMODE		0x070
	#define CVTCONTROLCMD		0x080
	#define FRNTWHEELCMD		0x081
	#define HITCHCMD			0x090
	#define QUADENCACK			0x100
	#define QUADENCCMD			0x110	
	#define STEERINGREFCMD		0x140
	#define CURVATURECMD 		0x141
	#define STEERINGANGLECMD	0x142
	#define RPMCMD				0x200
	#define STEERINGREPORTREQ	0x408 
	#define STEERINGREPORT1		0x409
	#define STEERINGREPORT2		0x40A
	#define STEERINGREPORT3		0x40B
	#define HITCHREARREQ		0x410
	#define SWITCHCMD			0x600
	#define HORNCMD				0x610
	#define RPMACK				0x700
	#define SWITCHACK			0x730
	#define HORNACK				0x740
	#define CVTACK				0x750
	#define FRNTWHEELACK		0x751
	#define STEERINGACK			0x758
	#define STEERINGANGLEACK	0x75A
	#define HITCHACK			0x760
	#define ESXREADYAUTOMODE	0x770
	#define KEEPAUTOMODEACK		0x771
	#define NEUTRALREADREQ		0x7A0
	#define NEUTRALREADACK		0x7A1

#endif

