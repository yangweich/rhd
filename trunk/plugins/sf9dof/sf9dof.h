 /** \file sf9dof.h
 *  \ingroup hwmodule
 *
 *   Interface for sf9dof multi IO board
 *
 *
 *******************************************************************/

#ifndef SD84_H
#define SD84_H

extern int initXML(char *);
extern int periodic(int);
extern int terminate (void) ; //No shutdown function

/// communication variables
struct
{ /** buffer for communication */
  #define MxBL 128
  char rxBuf[MxBL];
  /** device file descriptor */
  int ttyDev;  //File descriptors
  #define MxDL 64
  char serialDev[MxDL];
  /// stuck flag 0=not stuck, 1 = struk
  int stuck;
  /// baudrate
  int baudrate;
  /// debug mode
  int debugFlag;
} serif;


/// variables for 9dof controller
struct
{ /** RHD variables for 9dof controller */
  /// version info - all CPUs
  int varVersion;
  int varAcc;
  int varGyro;
  int varMag;
  int varRoll;
  int varPitch;
  int varCompas;
  int varUpdRate;
} sf9dof;



#endif

