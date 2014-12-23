 /** \file profibus.h
 *  \ingroup hwmodule
 *
 * Interface for Guidebot Profibus Interface
 *
 * Elements from this header file is extracted from the guidebot.h
 * file from the original gbd program. All non-profibus elements
 * have been removed.
 *
 ***************************************************************************/


#ifndef _PROFIBUS_H
#define	_PROFIBUS_H

 /* Definitions */
 #define DATASTOR 10000

/*Profibus inputs*/
extern struct pbin
{
	int Joystick;
	int NoLeftEnc, NoRightEnc;
	int EmStop;
	int Bumper;
	int Sick;
	int Obstacle;
	int jsp;
	int jdiff;
	int LowBatt;
	int princess;
	float batt;
	long lenc;
	long renc;
	long logenc [DATASTOR][2];
	int loglinedata;
	int lfault,rfault,lfaultsamp,rfaultsamp;
	int digi_in[6];
}
profibus_in;

/*Profibus outputs*/
extern struct pbout
{
	float lspeed;
	float rspeed;
	char signal_state;
	int lreset,rreset;
	
}
profibus_out;

/*
 * Profibus
 */
int profibus_start(void);
void profibus_stop(void);
void profibus_sensors_command(void);
void profibus_sensors_read(void);
void profibus_motors_update(void);
void profibus_motors_param(void);
void posmo_fault_ack(void);


#endif	/* _PROFIBUS_H */

