/** \file roboteq.h
 *  \ingroup hwmodule
 *  \brief RoboTeQ Motor Controller Interface (Armadillo Scout - University of Hohenheim)
 *
 * RoboTeQ Motor Controller Interface for RHD.
 * RobotTeQ will be referred to as rtq.
 *
 *
 *
 *  \author Claes Jæger-Hansen
 *  $Rev: 59 $
 *  $Date: 2012-03-19 18:38:29 +0100 (Mon, 19 Mar 2012) $
 */

/***************************************************************************
 *                  Copyright 2012 Claes Jæger-Hansen                      *
 *                                 cjh@uni-hohenheim.de                    *
 *                                 claeslund@gmail.com                     *
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

#ifndef ROBOTEQ_H
	#define ROBOTEQ_H

	/* \brief Read settings from XML-configuration file
	 *
	 *
	 */
	int initXML(char *);
	extern int periodic(int);
	//extern ”C” in t initXML ( char ∗ x m l f i l e n a m e ) ;
	//extern ”C” int shutdown ( void ) ;


#endif /* ROBOTEQ_H_ */
