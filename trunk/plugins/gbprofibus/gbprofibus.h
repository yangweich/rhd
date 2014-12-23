 /** \file gbprofibus.h
 *  \ingroup hwmodule
 *
 *   Interface for RS232 Linesensor module
 *
 * This plugin is based on SMRDSerial and is generally deprecated
 * as the linesensor should be placed on the RS485 bus instead.
 *
 * However, this plugin still provides the support for the module
 * for legacy purposes
 *
 *******************************************************************/

#ifndef RS232LINESENSOR_H
  #define RS232LINESENSOR_H

extern int initXML(char *);
extern int periodic(int);
extern int terminate (void) ; //No shutdown function
#endif

