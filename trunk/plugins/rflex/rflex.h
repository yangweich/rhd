/** \file rflex.c
 *  \ingroup hwmodule
 *  \brief Driver for iRobot rFlex research robots
 *
 *  This plug-in provides connection to the iRobot rFLEX robot-
 *  control system. It should basically support all rFLEX robots
 *  but is only tested on the iRobot ATRV-Jr
 * 
 *  Most code is ported from Carnegie Mellon Robot
 *  Navigation Toolkit (CARMEN) rflex-lib by Michael Montemerlo, 
 *  Nicholas Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 *  and Jared Glover (thank you guys!!)
 * 
 *  \author Anders Billesø Beck
 *  $Rev: 240 $
 *  $Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $
 *  
 */
/***************************************************************************
 *                  Copyright 2008 Anders Billesø Beck, DTU                *
 *                       anders.beck@get2net.dk                            *
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

#ifndef RFLEX_H
  #define RFLEX_H

/********* rFLEX definitions  *********/

///Memory to allocate for rx/tx buffers
#define MAX_COMMAND_LENGTH 256

//Communication headers
///Escape carecter is combined with STX / ETX
#define B_ESC              0x1b
///Start of rFLEX package indicator
#define B_STX              0x02
///End of rFLEX package indicator
#define B_ETX              0x03
#define B_NUL              0x00
#define B_SOH              0x01

//Standard values for motion control
#define STD_TRANS_TORQUE    30000
#define STD_ROT_ACC         100000
#define STD_ROT_TORQUE      35000

//DIO adress for bumpers
#define BUMPER_ADDR					0x40

//Maximum number of sonars
#define MAX_NUM_SONARS                 256

//Port definitions
#define SYS_PORT                 1
#define MOT_PORT                 2
#define JSTK_PORT                3
#define SONAR_PORT               4
#define DIO_PORT                 5
#define IR_PORT                  6

//System opcodes
#define SYS_LCD_DUMP             0
#define SYS_STATUS               1

//Motion control opcodes
#define MOT_AXIS_GET_SYSTEM      0
#define MOT_AXIS_GET_MODEL       1
#define MOT_AXIS_GET_TARGET      2
#define MOT_AXIS_SET_LIMITS      3
#define MOT_AXIS_GET_LIMITS      4
#define MOT_AXIS_SET_POS_LIMITS  5
#define MOT_AXIS_GET_POS_LIMITS  6
#define MOT_AXIS_SET_DIR         7
#define MOT_AXIS_SET_POS         8
#define MOT_AXIS_GET_MODE        9
#define MOT_SET_DEFAULTS        10
#define MOT_BRAKE_SET           11
#define MOT_BRAKE_RELEASE       12
#define MOT_SYSTEM_REPORT       33
#define MOT_SYSTEM_REPORT_REQ   34
#define MOT_GET_NAXES           65
#define MOT_SET_GEARING         66
#define MOT_GET_GEARING         67
#define MOT_MOTOR_SET_MODE      68
#define MOT_MOTOR_GET_MODE      69
#define MOT_MOTOR_SET_PARMS     70
#define MOT_MOTOR_GET_PARMS     71
#define MOT_MOTOR_SET_LIMITS    72
#define MOT_MOTOR_GET_LIMITS    73
#define MOT_MOTOR_GET_DATA      74
#define MOT_AXIS_SET_PARMS      75
#define MOT_AXIS_GET_PARMS      76
#define MOT_AXIS_SET_PWM_LIMIT  77
#define MOT_AXIS_GET_PWM_LIMIT  78
#define MOT_AXIS_SET_PWM        79
#define MOT_AXIS_GET_PWM        80

//Sonar opcodes
#define SONAR_RUN                0
#define SONAR_GET_UPDATE         1
#define SONAR_REPORT             2

//Digital I/O opcodes
#define DIO_REPORTS_REQ          0
#define DIO_REPORT               1
#define DIO_GET_UPDATE           2
#define DIO_UPDATE               3
#define DIO_SET                  4

//IR Opcodes (no IR on ATRV-Jr)
#define IR_RUN                   0
#define IR_REPORT                1


	//Plugin function definition
  extern int initXML(char*);
  extern int periodic(int );
	extern int terminate(void);
#endif

