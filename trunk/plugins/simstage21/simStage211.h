/** \file simStage.h
*  \ingroup hwmodule
*
*   Stage simulator interface
*******************************************************************/

#ifndef SIMSTAGE_H
  #define SIMSTAGE_H

#define STRLEN 128

# include <stage.h>

  extern int initXML(char*);
  extern int periodic(int);
  extern void terminate(void);
#endif

