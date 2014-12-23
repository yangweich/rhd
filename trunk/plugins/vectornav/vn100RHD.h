/** \file vectornav.h
 *  \ingroup hwmodule
 *  \brief VectorNav VM100-Rugged IMU
 *
 *
 *
 *
 *  \author Claes Dühring Jæger
 *  $Rev: 193 $
 *  $Date: 2013-05-16 16:43:25 +0200 (Thu, 16 May 2013) $
 */

/***************************************************************************
 *                  Copyright 2013 Claes Dühring Jæger                     *
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

#ifndef VN100_H
	#define VN100_H

	/* \brief Read settings from XML-configuration file
	 *
	 *
	 */
	int initXML(char *);
	extern int periodic(int);
	//extern ”C” in t initXML ( char ∗ x m l f i l e n a m e ) ;
	//extern ”C” int shutdown ( void ) ;

	/********* Function Prototype *********/
	int initvn100(void);
	int shutdownVN(void);
	void *vn_task(void *);
	//int readSerial(int,char*);

	int readDefault(void);
	int parseDefault(void);
	int updateDefaultValues(void);

	char *mystrtok(char* string,const char *delim);


	/*
	 * \brief Holds the default values read from the vn100.
	 *
	 * The default string is $VNYMR see page 69 in UM100.pdf
	 * $VNYMR,+029.694,+003.114,+115.116,+00.5800,+02.5299,-00.5768,+00.527,-09.024,+04.208,-00.002434,-00.003153,-00.000194*60
	 * $VNYMR,yaw,pitch,roll,magx,magy,magz,accx,accy,accz,gyrox,gyroy,gyroz
	 */
	typedef struct{
		char *command;
		double yaw;
		double pitch;
		double roll;
		double magX;
		double magY;
		double magZ;
		double accX;
		double accY;
		double accZ;
		double gyroX;
		double gyroY;
		double gyroZ;
	} vn100Default;


#endif /* VN100_H */
