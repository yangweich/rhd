/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Module:     dp_prtab.c                                                   */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains functions of the DP_BASE.DLL specific to              */ 
/*  the compiler.                                                            */
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
#else
#include <string.h>
#endif

#include "dp_5613.h"
#include "ci_5613.h"

#include "dp_base.h"
#include "fkt_def.h"
#include "trace.h"
#include "xtrace.h"



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   get_fw_path()                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function obtains the path of the firmware that is downloaded to     */ 
/*  CP5613\5614. If the value '0' is transferred, the path is taken from     */
/*  the registry.                                                            */
/*                                                                           */
/*****************************************************************************/


void  get_fw_path  (DPR_STRING** FwPathPtr)
{
#ifdef WIN32
	*FwPathPtr = 0; /* 0 = value from registry */
#else
	*FwPathPtr = 0; /* default value           */
#endif
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DllMain()                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  With DLL_PROCESS_DETACH, all the open handles are closed with DP_close.  */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/


#ifdef WIN32

BOOL APIENTRY DllMain (HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
	
	DPR_DWORD  MyUserHandle;
	DPR_DWORD  result;
	DP_ERROR_T error; 
	
	switch (dwReason)
	{
	case DLL_PROCESS_ATTACH:
		{
			
			InitializeCriticalSection(&DP_CriticalSection);
			
			/* init trace handling, return value 0: no trace activated */
			
			DpTrcIndex = X_Trc_init ("DP_BASE_TRC_LEVEL",  /* optionally keys in registry */
				"DP_BASE_TRC_SELECT",
				"DP_BASE_TRC_PATH",
				"DP_BASE_TRC_NUM_ENTRY",
				// ---------------
				DP_BASE_TRC_TEXT,     /* path in registry for warning text */
				DP_BASE_TRC_TITLE_TEXT, /* Titel of warning text           */
				DP_BASE_TRC_TARGET,   /* mandatory for trace in registry */
				"DP_TRACE",           /* titel in trace file             */
				DP_BASE_TRC_REGISTRY, /* path in registry under HKEY_L.M */
				"dp_trc.txt",
				// ---------------
				0xffff,               /* DefaultLevel */
				TRACE_SELECT_ALL,
				1000);                /* default number of entries */
			break;	
		}
		
	case DLL_THREAD_ATTACH:
		{
			break;	
		}
		
	case DLL_PROCESS_DETACH:
		{
			
			/* close all open connections */
			do
			{
                result = get_next_user(&MyUserHandle);
                if (result == DP_OK)
                {
					if( DP_close (MyUserHandle,&error) != DP_OK)
					{
						(void)release_user (MyUserHandle);
					}
                }
			}
			while (result == DP_OK);
			
			/* close trace if necessary */
			if( DpTrcIndex != 0)
			{
                X_Trc_close(DpTrcIndex);
                DpTrcIndex = 0;
			}
			
			DeleteCriticalSection(&DP_CriticalSection);
			
			break;
		}  	
		
	case DLL_THREAD_DETACH:
		{
			break;
		}
	default:
		{
			break;
		}
	}
	
	return (TRUE);
}

#endif

#ifdef WIN32
#pragma warning( default : 4201 4214 4115 4100)
#endif
