/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Version:    1.0                                                          */
/*  Modul:      dps_prtab.c                                                  */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains compiler specific functions  of the DP_BASE.DLL       */
/*                                                                           */
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
/*                                                                           */
/*                                                                           */
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
#include "dps_5614.h"
#include "ci_5613.h"

#include "dps_base.h"
#include "fkt_def.h"





/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dps_get_fw_path()                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Mit dieser Funktion wird der Pfad der Firmware, die in den CP5613        */
/*  geladen wird, ermittelt. Wird der Wert '0' uebergeben, wird der Default- */
/*  Pfad der ci_base.dll genommen.                                           */
/*                                                                           */
/*****************************************************************************/

void  dps_get_fw_path  (DPR_STRING** FwPathPtr)
{
#ifdef WIN32
	*FwPathPtr = 0; /* 0 = value from registry */
#else
	*FwPathPtr = 0; /* default value           */
#endif
}



/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DllMain()                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Bei DLL_PROCESS_DETACH werden alle geoffneten Handles mit DP_close       */
/*  geschlossen.                                                             */
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
			InitializeCriticalSection(&DpsCritSec);
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
                result = dps_get_next_user(&MyUserHandle);
                if (result == DP_OK)
                {
					if( DPS_close (MyUserHandle,&error) != DP_OK)
					{
						result = dps_release_user (MyUserHandle);
					}
                }
			}
			while (result == DP_OK);
			DeleteCriticalSection(&DpsCritSec);
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
