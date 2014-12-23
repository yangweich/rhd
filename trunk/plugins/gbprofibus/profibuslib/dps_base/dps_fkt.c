/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Version:    1.0                                                          */
/*  Modul:      dps_fkt.c                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains help functions for dp_uif.c                           */
/*                                                                           */
/*                                                                           */
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
#include <windows.h>
#endif
#include <string.h>


#include "dp_5613.h"
#include "dps_5614.h"
#include "5613_ret.h"
#include "ci_5613.h"

#include "dps_base.h"
#include "uif_def.h"
#include "fkt_def.h"
//#include "slv_err_txt.h"

#define DPS_DEFAULT_TIMEOUT     10000   // 10 sec

/* Global counter for user instances */
DPR_WORD DpsUserCtr = 0;

/* Global table with actual Ids */
DP_USER_ID_TABLE_T DpsUserIdTable[DP_MAX_USER_INST+1];

#ifdef WIN32
/* Global critical section for synchronous requests */
CRITICAL_SECTION    DpsCritSec;
#endif

/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   build_dps_header()                                           */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Mit dieser Funktion wird der Header fuer die ci-send-Auftraege           */
/*  aufgebaut.                                                               */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

void build_dps_header (CI_DPS_REQUEST_T* MyCiRequest,
					   DPR_DWORD MyDpOpcode,
					   DPR_DWORD MyCiUsrHandle)
{
	MyCiRequest->header.user_handle       = MyCiUsrHandle;
	
	
	/* low-word: opcode, high word: subsystem */
	MyCiRequest->header.opcode            = MyDpOpcode | SUBSYSTEM_DPC31;
	MyCiRequest->header.order_id          = MyDpOpcode;
	
	/* maximum tine for wait! */
	MyCiRequest->header.timeout = DPS_DEFAULT_TIMEOUT;
	
	
	switch(MyDpOpcode)
	{
	case CI_DPS_OPEN:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= DPS_ORDER_HEADER_SIZE + sizeof(CI_DPS_OPEN_T);
		break;
	case CI_DPS_CLOSE:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= DPS_ORDER_HEADER_SIZE;
		break;
	case CI_DPS_START:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= DPS_ORDER_HEADER_SIZE;
		break;
	case CI_DPS_STOP:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= DPS_ORDER_HEADER_SIZE;
		break;
	case CI_DPS_GET_BAUD_RATE:
	case CI_DPS_GET_GC_COMMAND:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= 0; // direct DP-RAM access
		break;
	case CI_DPS_GET_STATE:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= DPS_ORDER_HEADER_SIZE + sizeof(CI_DPS_GET_STATE_T);
		break;
#if 0
	case CI_DPS_TRIGGER_USER_WD:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= DPS_ORDER_HEADER_SIZE + sizeof(CI_DPS__T);
		break;
#endif
	case CI_DPS_RCV_IND:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= DPS_ORDER_HEADER_SIZE + sizeof(CI_DPS_RCV_IND_T);
		break;
	case CI_DPS_SETUP_INDS:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= DPS_ORDER_HEADER_SIZE + sizeof(CI_DPS_SETUP_INDS_T);
		break;
	case CI_DPS_SET_RESP:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= DPS_ORDER_HEADER_SIZE + sizeof(CI_DPS_SET_RESP_T);
		break;
	default:
		MyCiRequest->header.buf_length = MyCiRequest->header.buf_fill_length
			= 0;
		break;
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dps_send_and_receive()                                       */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion uebergibt den DP-Auftrag mittels CI_send an die           */
/*  CP 5613 und holt im Erfolgsfall die Quittung mit CI_receive ab.          */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_send_and_receive (CI_DPS_REQUEST_T* MyCiRequestPtr)
{
	DPR_DWORD Result;
	
	
	Result = CI_send( (CI_REQUEST_T DP_MEM_ATTR*)MyCiRequestPtr);
	
	if (Result == CI_RET_OK)
	{
		/* send was successfully (no data available), now wait for confirmation */
		Result = CI_receive ((CI_REQUEST_T DP_MEM_ATTR*)MyCiRequestPtr);
	}
	
	return(Result);
}

/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dps_send()                                                   */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion uebergibt den DP-Auftrag mittels CI_send an den           */
/*  CP 5613 ab.                                                              */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_send (CI_DPS_REQUEST_T* MyCiRequestPtr)
{
	DPR_DWORD Result;
	
	Result = CI_send( (CI_REQUEST_T DP_MEM_ATTR*)MyCiRequestPtr);
	
	return(Result);
}



/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dps_receive()                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion holt eine asynchrone Quittung mit CI_receive ab.          */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_receive (CI_DPS_REQUEST_T* MyCiResultPtr,DPR_DWORD user_handle,DPR_DWORD timeout)
{
	DPR_DWORD Result;
	
	MyCiResultPtr->header.user_handle        = (user_handle & 0x00ffffff);
	MyCiResultPtr->header.buf_length         = sizeof(CI_DPS_ORDER_T);
	MyCiResultPtr->header.timeout            = timeout;
	
	Result = CI_receive ((CI_REQUEST_T DP_MEM_ATTR*)MyCiResultPtr);
	
	return(Result);
}




/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dps_build_result()                                           */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion aktualisiert die Struktur DP_Error nach Abschluss         */
/*  des CI_receive oder CI_send bei synchronen Aufrufen                      */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

void dps_build_result (DPR_DWORD CiResult, CI_DPS_REQUEST_T* MyCiRequestPtr,DP_ERROR_T* MyErrorPtr)
{
	CI_DPS_ORDER_T* MyDpOrderPtr;
	
	switch (CiResult)
	{
	case CI_RET_OK_DATA:
		{
			/* Data vailable, check the result */
			{
				MyDpOrderPtr = (CI_DPS_ORDER_T*)&MyCiRequestPtr ->order;
				
				MyErrorPtr -> error_class  = MyDpOrderPtr -> ErrClass;
				MyErrorPtr -> error_code   = MyDpOrderPtr -> ErrCode;
				MyErrorPtr -> error_decode = MyDpOrderPtr -> ErrDecode;
				MyErrorPtr -> error_code_1 = MyDpOrderPtr -> ErrCode1;
				MyErrorPtr -> error_code_2 = MyDpOrderPtr -> ErrCode2;
			}
			break;
		}
	case CI_RET_OK:
		{
			/* TimeOut failure: no data available */
			MyErrorPtr -> error_class  = DP_ERROR_CI;
			MyErrorPtr -> error_code   = CiResult;
			MyErrorPtr -> error_decode = 0;
			MyErrorPtr -> error_code_1 = 0;
			MyErrorPtr -> error_code_2 = 0;
			break;
			
		}
	default:
		{
			/* driver error */
			MyErrorPtr -> error_class  = DP_ERROR_CI;
			MyErrorPtr -> error_code   = CiResult;
			MyErrorPtr -> error_decode = 0;
			MyErrorPtr -> error_code_1 = 0;
			MyErrorPtr -> error_code_2 = 0;
			break;
			
		}
	}
}



/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dps_build_result_async()                                     */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion aktualisiert die Struktur DP_Error nach Abschluss         */
/*  des CI_send bei asynchronen Aufrufen                                     */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

void dps_build_result_async (DPR_DWORD CiResult, CI_DPS_REQUEST_T* MyCiRequestPtr,DP_ERROR_T* MyErrorPtr)
{
	CI_DPS_ORDER_T* MyDpOrderPtr;
	
	switch (CiResult)
	{
	case CI_RET_OK_DATA:
		{
			/* data avilable, check the result */
			{
				MyDpOrderPtr = (CI_DPS_ORDER_T*)&MyCiRequestPtr ->order;
				
				MyErrorPtr -> error_class  = MyDpOrderPtr -> ErrClass;
				MyErrorPtr -> error_code   = MyDpOrderPtr -> ErrCode;
				MyErrorPtr -> error_decode = MyDpOrderPtr -> ErrDecode;
				MyErrorPtr -> error_code_1 = MyDpOrderPtr -> ErrCode1;
				MyErrorPtr -> error_code_2 = MyDpOrderPtr -> ErrCode2;
			}
			break;
		}
	case CI_RET_OK:
	case CI_RET_RECEIVE_TIMEOUT_NO_DATA:
		{
			/* no data available, use DP_get_result to get the confirmation */
			MyErrorPtr -> error_class  = DP_OK_ASYNC;
			MyErrorPtr -> error_code   = DP_OK;
			MyErrorPtr -> error_decode = 0;
			MyErrorPtr -> error_code_1 = 0;
			MyErrorPtr -> error_code_2 = 0;
			break;
			
		}
	default:
		{
			/* driver error */
			MyErrorPtr -> error_class  = DP_ERROR_CI;
			MyErrorPtr -> error_code   = CiResult;
			MyErrorPtr -> error_decode = 0;
			MyErrorPtr -> error_code_1 = 0;
			MyErrorPtr -> error_code_2 = 0;
			break;
			
		}
	}
}


/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dps_store_user()                                             */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion speichert ein User-Handle in einer verketteten Liste ab   */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD dps_store_user (DPR_DWORD sync_handle,DPR_DWORD async_handle,
						  DPR_DWORD *user_handle)
{
	DPR_DWORD result;
	DPR_WORD  i;
	
	result = DP_RET_MEMORY;
	
	/* first user instance? */
	if (DpsUserCtr == 0)
	{
		/* clear array */
		for(i=0;i <= DP_MAX_USER_INST;i++)
		{
			memset ( &(DpsUserIdTable[i]),0,sizeof(DP_USER_ID_TABLE_T));
		}
	}
	
	
	/* search next free index, index 0 is reserved */
	
	for(i=1;i<= DP_MAX_USER_INST;i++)
	{
		if (DpsUserIdTable[i].Used == 0)
		{
			DpsUserIdTable[i].Used          = 0xff;
			DpsUserIdTable[i].CpIndex       = 0;
			DpsUserIdTable[i].DpSyncHandle  = sync_handle;
			DpsUserIdTable[i].DpAsyncHandle = async_handle;
			
			*user_handle = i;
			DpsUserCtr++;
			
			result = DP_OK;
			break;
		}
	}
	
	return(result);
}




/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dps_release_user()                                           */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion gibt ein User-Handle in einer verketteten Liste wieder    */
/*  frei.                                                                    */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD dps_release_user (DPR_DWORD user_handle)
{
	DPR_DWORD result;
	
	if ((DpsUserCtr > 0) && (user_handle <= DP_MAX_USER_INST))
	{
		/* clear array */
		memset ( &(DpsUserIdTable[user_handle]),0,sizeof(DP_USER_ID_TABLE_T));
		DpsUserCtr--;
		result = DP_OK;
	}
	else
	{
		result = DP_RET_MEMORY;
	}
	
	return(result);
}





/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dps_get_next_user()                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion ermittelt das naechste  User-Handle                       */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD dps_get_next_user  (DPR_DWORD *user_handle)
{
	DPR_DWORD result;
	DPR_DWORD i;
	
	result = DP_RET_MEMORY;
	
	if (DpsUserCtr == 0)
	{
		*user_handle = 0;
	}
	else
	{
		for(i=1;i<=DP_MAX_USER_INST;i++) /* index 0 is reserved */
		{
			if (DpsUserIdTable[i].Used == 0xff)
			{
				*user_handle  = i;
				result = DP_OK;
				break;
			}
		}
	}
	
	return(result);
}

