/** \file laserServer.h
 *  \ingroup hwmodule
 *  \brief Laserscanner server for sharing laserscanner data from stage
 *
 *  This plugin provides the laserscanner interface to share laserscanner
 * data from the stage server
 *
 *  \author Anders Billesø Beck
 *  $Rev: 411 $
 *  $Date: 2010-03-17 21:07:39 +0100 (Wed, 17 Mar 2010) $
 *
 */
/***************************************************************************
 *              Copyright 2010 Anders Billesø Beck, DTU                    *
 *                         abb@elektro.dtu.dk                              *
 *                                                                         *
 ***************************************************************************/


#ifndef _LASERSERVER_H
#define	_LASERSERVER_H

	//Stage includes
	#include <libstage/stage.hh>

	int laserServerTransmit(Stg::ModelLaser *laserscan, Stg::stg_usec_t simtime);
	int initLaserServer(int port);

#endif	/* _LASERSERVER_H */

