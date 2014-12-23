/** \file tcmxb.h
 *  \ingroup hwmodule
 *  \brief PNI ForceField TCM XB sensor (Armadillo Scout - University of Hohenheim)
 *
 * Tilt-compensated compass modules provide reliable, pinpoint-accurate pitch,
 * roll and compass heading. The TCMs use advanced algorithms to counter the effects
 * of hard and soft iron interference, providing highly accurate heading information
 *  in most any environment and any orientation.  PNI's patented magneto-inductive
 *  sensors and pioneering measurement technology combine to provide all this
 *  performance under a low power budget that extends mission duration.
 *
 *  \author Claes Jæger-Hansen
 *  $Rev: 284 $
 *  $Date: 2012-05-30 13:30:06 +0200 (Wed, 30 May 2012) $
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

#ifndef TCMXB_H
	#define TCMXB_H

	/* \brief Read settings from XML-configuration file
	 *
	 *
	 */
	int initXML(char *);
	extern int periodic(int);
	//extern ”C” in t initXML ( char ∗ x m l f i l e n a m e ) ;
	//extern ”C” int shutdown ( void ) ;
//	#define kMountedZDownPlus270 0x0A
	struct readFFS 
	{
		int 			datalen;// = sizeof(data)+crc;
		int				crc;
		unsigned char   data1[4092];
	};
	
	struct writeFFS 
	{
		char	datalenght[2];
		char 	frameid;
		char 	*payload;
		char	crc[2];
	};

	
	enum{
		// Frame IDs (Commands)
		kGetModInfo = 1, 		// 1
		kModInfoResp,			// 2
		kSetDataComponents,		// 3
		kGetData, 				// 4
		kDataResp, 				// 5
		kSetConfig, 			// 6
		kGetConfig, 			// 7
		kConfigResp,			// 8
		kSave,					// 9
		kStartCal, 				// 10
		kStopCal, 				// 11
		kSetParam, 				// 12
		kGetParam, 				// 13
		kParamResp, 			// 14
		kPowerDown, 			// 15
		kSaveDone, 				// 16
		kUserCalSampCount,		// 17
		kUserCalScore,			// 18
		kSetConfigDone, 		// 19
		kSetParamDone,			// 20
		kStartIntervalMode,		// 21
		kStopIntervalMode,		// 22
		kPowerUp, 				// 23
		kSetAcqParams,			// 24
		kGetAcqParams,			// 25
		kAcqParamsDone, 		// 26
		kAcqParamsResp, 		// 27
		kPowerDoneDown, 		// 28
		kFactoryUserCal, 		// 29
		kFactoryUserCalDone,	// 30
		kTakeUserCalSample,		// 31
		kFactoryInclCal = 36,
		kFactoryInclCalDone,	// 37
		kSetMode = 46,			// 46
		kSetModeDone,			// 47
		kSyncRead = 49, 		// 49

		// Cal Option IDs
		kFullRangeCal = 10,		//  10 - type Float32
		k2DCal = 20,			//  20 - type Float32
		kHIOnlyCal = 30, 		//  30 - type Float32
		kLimitedTiltCal = 40, 	//  40 - type Float32
		kAccelCalOnly = 100,	// 100 - type Float32
		kAccelCalwithMag = 110,	// 110 - type Float32

		// Param IDs
		kFIRConfig = 3,			// 3- AxisID(UInt8)+Count(UInt8)+Value(Float64)+...

		// Data Component IDs
		kHeading = 5,			// 5 - type Float32
		kTemperature = 7, 		// 7 - type Float32
		kDistortion = 8, 		// 8 - type boolean
		kPAligned = 21, 		// 21 - type Float32
		kRAligned, 				// 22 - type Float32
		kIZAligned, 			// 23 - type Float32
		kPAngle,				// 24 - type Float32
		kRAngle,				// 25 - type Float32
		kXAligned = 27, 		// 27 - type Float32
		kYAligned, 				// 28 - type Float32
		kZAligned, 				// 29 - type Float32

		// Configuration Parameter IDs
		kDeclination = 1, 		// 1 - type Float32
		kTrueNorth, 			// 2 - type boolean
		kMountingRef = 10,		// 10 - type UInt8
		kUserCalStableCheck,	// 11 - type boolean
		kUserCalNumPoints,		// 12 - type UInt32
		kUserCalAutoSampling, 	// 13 - type boolean
		kBaudRate, 				// 14 - UInt8
		kMilOutPut = 15, 		// 15 - type Boolean
		kDataCal,				// 16 - type Boolean
		kCoeffCopySet = 18,		// 18 - type UInt32
		kAccelCoeffCopySet,		// 19 - type UInt32

		// Mounting Reference IDs
		kMountedStandard = 1, 	// 1
		kMountedXUp,			// 2
		kMountedYUp,			// 3
		kMountedStdPlus90,		// 4
		kMountedStdPlus180,		// 5
		kMountedStdPlus270,		// 6
		kMountedZDown,			// 7
		kMountedXUpPlus90, 		// 8
		kMountedXUpPlus180,		// 9
		kMountedXUpPlus270,		// 10
		kMountedYUpPlus90, 		// 11
		kMountedYUpPlus180,		// 12
		kMountedYUpPlus270,		// 13
		kMountedZDownPlus90,	// 14
		kMountedZDownPlus180,	// 15
		kMountedZDownPlus270,	// 16

		// Result IDs
		kErrNone = 0,			// 0
		kErrSave, 				// 1
	};




#endif /* TCMXB_H */
