/** \file powercube.h
 *  \ingroup hwmodule
 *  \brief Hardware abstraction layer for Powercube control using CAN-BUS.
 *
 *  This library inplements the hardware abstraction layer
 *  for communicating using CAN-BUS on the Powercube modules.
 * 
 *  At the present state only a simple homing command is implemeted.
 *  This is only used to test communication of various CAN I/O-boards.
 *
 *  \author Nils A. Andersen & Anders Billesø Beck, DTU 
 *  \editor Soren Hansen, DTU
 *  
 *  $Rev: 59 $
 *  $Date: 2008-09-30 11:58:03 +0200 (Tue, 30 Sep 2008) $
 *  
 */
 
#ifndef POWERCUBE_H
#define POWERCUBE_H

extern int initXML(char *);
extern int periodic(int);

#endif  // POWERCUBE_H

