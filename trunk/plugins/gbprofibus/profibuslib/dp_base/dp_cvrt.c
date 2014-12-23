/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Module:     dp_cvrt.c                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains special functions for dpmconv.dll                     */
/*                                                                           */
/*                                                                           */
/*  // in  = job parameter                                                   */
/*  // out = return parameter                                                */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*  History of changes:                                                      */
/*  ===================                                                      */
/*                                                                           */
/*  Date     Comment                                                         */
/*----------!----------------------------------------------------------------*/
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
#endif

#include <string.h>

#include "dp_5613.h"
#include "ci_5613.h"
#include "dp_base.h"
#include "fkt_def.h"
#include "cvrt_def.h"


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_register_slv_cvrt()                                       */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Using this function, the DP Konverter registers the slaves with the dr.  */
/*                                                                           */
/*****************************************************************************/



DPR_DWORD  DP_register_slv_cvrt (DPR_DWORD      user_handle,   /* in  */
                                 DPR_BYTE       slv[127],      /* in  */
                                 DP_ERROR_T     *error)        /* out */
{
	DPR_DWORD       CiResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	DPR_WORD        i;
	
	
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	build_dp_header (&MyCiRequest,CI_DP_REGISTER_SLV,
		DpUserIdTable[user_handle].DpSyncHandle,
		DpUserIdTable[user_handle].CpIndex);
	
	MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
	
	MyDpOrderPtr -> DatTyp   = 0xffff;
	MyDpOrderPtr -> DatLen   = 127;
	for(i=0;i<127;i++)
	{
		MyDpOrderPtr->DatBuf[i]=(unsigned char)slv[i];
	}
	
	/* send DP request and receive confirmation */
	
	CiResult = dp_send_and_receive (&MyCiRequest);
	
	build_result (CiResult, &MyCiRequest, error);
	
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return (error -> error_class);
	
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_open_cvrt()                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Using this function, the DP application registers with the driver.       */
/*  If successful, the function returns a user handle.                       */
/*  The user handle must be used in all subsequent function calls            */
/*                                                                           */
/*****************************************************************************/





DPR_DWORD  DP_open_cvrt (DPR_STRING *cp_name,      /* in  */
                         DPR_DWORD  *user_handle,  /* out */
                         DP_ERROR_T *error)        /* out */
{
	return DP_open_int(cp_name,user_handle,error,0xffff);
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_get_pointer                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Using this function, a DP application obtains the pointer to the         */
/*  dual-port RAM. The application can only access the dual-port RAM         */
/*  directly after it has obtained the pointer.                              */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_get_pointer_cvrt (DPR_DWORD user_handle,                        /* in  */
                                            DPR_DWORD timeout,                            /* in  */ 
                                            DPR_CP5613_DP_T volatile  DP_MEM_ATTR  **dpr, /* out */
                                            DP_ERROR_T  DP_MEM_ATTR    *error)            /* out */
{
	return DP_get_pointer_int(user_handle,
		timeout,
		dpr,
		error,
		0xffff);
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_release_pointer                                           */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  With this function, a DP application releases the pointer to the         */
/*  dual-port RAM again. After it has released the pointer, the application  */
/*  can no longer access the dual-port RAM                    .              */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DP_CODE_ATTR DP_release_pointer_cvrt (DPR_DWORD user_handle,           /* in  */
                                                DP_ERROR_T DP_MEM_ATTR *error)   /* out */
{
	return DP_release_pointer_int(user_handle,error,0xffff);
}

