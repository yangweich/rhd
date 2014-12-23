/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Version:    1.0                                                          */
/*  Module:     dps_xtrc.c                                                   */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains functions that allowing DP requests to be recorded    */
/*  in a trace. The relevant information of the DP requests is obtained.     */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*  History of changes:                                                      */
/*  ===================                                                      */
/*                                                                           */
/*  Date     Comment                                                         */
/*----------!----------------------------------------------------------------*/
/*  30.06.98 first release                                                   */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Dieses Programm ist Freeware. Jedem Benutzer steht es frei, dieses        */
/* Programm unentgeltlich zu nutzen, zu kopieren, zu verändern, in andere    */
/* Applikationen zu integrieren und/oder weiterzugeben, vorausgesetzt, dass  */
/* die im Programm enthaltenen Urheberrechtsvermerke und Marken unverändert  */
/* übernommen werden und jede Änderung des Programms als solche bezeichnet   */
/* wird.                                                                     */
/*                                                                           */
/* JEGLICHE GEWÄHRLEISTUNG FÜR DIE FUNKTIONSTÜCHTIGKEIT ODER KOMPATIBILITÄT  */
/* DIESES PROGRAMMS IST AUSGESCHLOSSEN. DIE BENUTZUNG ERFOLGT AUF EIGENE     */
/* VERANTWORTUNG UND GEFAHR.                                                 */ 
/*                                                                           */
/*                                                                           */
/* This software is Freeware. You may copy, modify, integrate it into        */
/* another application and use it for free as well as distribute it to       */
/* others, provided however, that all trademarks and copyright notices       */
/* remain unchanged and any modification of the software is marked.          */
/*                                                                           */
/* SINCE THIS SOFTWARE IS DISTRIBUTED FOR FREE, IT IS PROVIDED "AS IS"       */
/* WITHOUT ANY REPRESENTATION OR WARRANTY OF ANY KIND EITHER EXPRESSED OR    */
/* IMPLIED INCLUDING BUT NOT LIMITED TO IMPLIED WARRANTIES FOR               */
/* MERCHANTIBILITY OR FITNESS FOR USE. ANY USE OF THE SOFTWARE IS ON YOUR    */
/* OWN RISK AND RESPONSIBILITY.                                              */
/*                                                                           */
/*****************************************************************************/

#ifdef WIN32
#pragma warning( disable : 4201 4214 4115 4100 4514)
#endif

#ifdef WIN32
#include <windows.h>
#include <stdio.h>
#endif


#ifdef WIN32
#include "trace.h"
#include "dp_5613.h"
#include "dps_5614.h"
#include "5613_ret.h"
#include "5614_ret.h"
#include "ci_5613.h"
#include "dps_base.h"


DPR_WORD  DpsTrcIndex = 0;

/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpTrcGetResult                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Common trace functions between DP_BASE.dll ans DPS_BASE.dll              */
/*                                                                           */
/*****************************************************************************/




CT W32EXPORT_CX unsigned short CLX X_Trc_init (char* LevelKeyName,
											   char* SelectKeyName,
											   char* FilePathKeyName, // File-Pfad
											   char* TrcNumKeyName,
											   // ---------------
											   char* TextKeyName,
											   char* TextTitleKeyName,
											   char* TargetKeyName,
											   char* HeaderName,
											   char* RegistryPath,
											   char* FileName,
											   // ---------------
											   unsigned short DefaultLevel,
											   unsigned long  DefaultSelect,
											   unsigned short DefaultTrcNum)
{
	
	DpsTrcIndex = Trc_init (LevelKeyName,
		SelectKeyName,
		FilePathKeyName, // File-Pfad
		TrcNumKeyName,
		TextKeyName,
		TextTitleKeyName,
		TargetKeyName,
		HeaderName,
		RegistryPath,
		FileName,
		DefaultLevel,
		DefaultSelect,
		DefaultTrcNum);
	return(DpsTrcIndex);
}



CT W32EXPORT_CX void CLX X_Trc_close (unsigned short MyIndex)
{
	Trc_close (MyIndex);
	DpsTrcIndex = 0;
}


CT W32EXPORT_CX void CLX X_Trc_write (unsigned short MyIndex,
                                      unsigned short MyLevel,
                                      unsigned long  MySelect,
                                      char *         MyTextBuf,
                                      char *         MyDumpBuf,
                                      unsigned short MyNumOfBytesToRead)
									  
{
	Trc_write (MyIndex,
		MyLevel,
		MySelect,
		MyTextBuf,
		MyDumpBuf,
		MyNumOfBytesToRead);
}


CT W32EXPORT_CX void CLX X_Trc_write_txt (unsigned short MyIndex,
                                          unsigned short MyLevel,
                                          unsigned long  MySelect,
                                          char *         MyTextBuf)
{
	Trc_write_txt (MyIndex,
		MyLevel,
		MySelect,
		MyTextBuf);
}


#endif  // WIN32






#ifdef WIN32
#pragma warning( default : 4201 4214 4115 4100)
#endif

