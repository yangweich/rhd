 /** \file sableandmagenc.h
 *  \ingroup hwmodule
 *
 * Interface for sabertooth motor controller and AS4050 magnetic encoders
 * through an arduino nano velocity controller
 *
 * $Rev: 85 $
 * $Id: rhdlog.h 85 2012-12-29 10:28:02Z jcan $
 *
 *******************************************************************/

#ifndef SAME_H
#define SAME_H

/** RHD calls initXML once just after the plugin is loaded
    used to read configuration file and set initial state
    and open and configure connection to device */
extern int initXML(char *);
/**
 * RHD calls this function every time write variables may have beed updated
 * normally every 10ms (RHD sample time)
 * \param RHDtick is RHD sample count. */
extern int periodic(int RHDtick);
/**
 * Called by RHD when RHD is shutting down. 
 * Should be used to close files and connections */
extern int terminate (void) ; //No shutdown function

#endif

