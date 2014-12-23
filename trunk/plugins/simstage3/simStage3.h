/** \file simStage3.h
 *  \ingroup hwmodule
 *  \brief Driver for Player/Stage simulator v. 3.2.2
 *
 *  This plugin provides a connection to the Stage simulator
 *  for simulating a single robot.
 *
 *  \author Anders Billesø Beck
 *  $Rev: 59 $
 *  $Date: 2010-03-17 21:07:39 +0100 (Wed, 17 Mar 2010) $
 *  
 */
/***************************************************************************
 *              Copyright 2010 Anders Billesø Beck, DTU                    *
 *                         abb@elektro.dtu.dk                              *
 ***************************************************************************/

#ifndef SIMSTAGE3_H
  #define SIMSTAGE3_H



  //Plugin function definition
  extern "C" int initXML(char*);
  extern "C" int periodic(int );
  extern "C" int terminate(void);

#endif

