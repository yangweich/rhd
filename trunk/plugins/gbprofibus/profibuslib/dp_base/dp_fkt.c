/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Module:     dp_fkt.c                                                     */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains help functions for dp_uif.c                           */
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

#ifdef linux
#define stricmp strcasecmp
#endif

#include <string.h>
#include <stdio.h>


#include "dp_5613.h"
#include "5613_ret.h"
#include "ci_5613.h"

#include "dp_base.h"
#include "uif_def.h"
#include "fkt_def.h"
#include "err_txt.h"

#include "5614_ret.h"
#include "serr_txt.h"

/* array size for getting err number */
#define ERR_NUM_SIZE 15

/* Global counter for user instances */
DPR_WORD DpUserCtr = 0;

/* Global table with actual Ids */
DP_USER_ID_TABLE_T DpUserIdTable[DP_MAX_USER_INST+1];

/* global variable for trace handling */
#ifdef WIN32

CRITICAL_SECTION   DP_CriticalSection;

DPR_WORD           DpTrcIndex = 0;
#endif

/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   build_dp_header()                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function builds the header for the CI_send jobs.                    */
/*                                                                           */
/*****************************************************************************/


void build_dp_header (CI_DP_REQUEST_T* MyCiRequest,
                      DPR_DWORD MyDpOpcode,
                      DPR_DWORD MyCiUsrHandle,
                      DPR_WORD  MyCpIndex)
{
	MyCiRequest->header.user_handle       = MyCiUsrHandle;
	
	
	/* low-word: opcode, high word: subsystem */
	MyCiRequest->header.opcode            = MyDpOpcode | SUBSYSTEM_DPMC;
	MyCiRequest->header.order_id          = MyDpOpcode; 
	
	MyCiRequest->header.buf_length        = sizeof(CI_DP_ORDER_T);
	MyCiRequest->header.buf_fill_length   = sizeof(CI_DP_ORDER_T);
	
	/* DP user index */
	((CI_DP_ORDER_T*)(MyCiRequest->data))->UsrIndex = MyCpIndex;
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_send_and_receive()                                        */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function transfers the DP job to the CP 5613\5614 with CI_send      */
/*  and if successful fetches the acknowledgment with CI_receive.            */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_send_and_receive (CI_DP_REQUEST_T* MyCiRequestPtr)
{
	DPR_DWORD Result;
	
	
	Result = CI_send( (CI_REQUEST_T*)MyCiRequestPtr);
	
	if (Result == CI_RET_OK)
	{
		/* send was successfully (no data available), now wait for confirmation	*/
		
		/* maximum tine for wait! */
		
		MyCiRequestPtr->header.timeout = DP_WAIT_RCV_TIMEOUT;
		
		Result = CI_receive ((CI_REQUEST_T*)MyCiRequestPtr);
		
	}
	
	return(Result);
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_send()                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function transfers the DP job to the CP 5613\5614 with CI_send.     */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_send (CI_DP_REQUEST_T* MyCiRequestPtr)
{
	DPR_DWORD Result;
	
	Result = CI_send( (CI_REQUEST_T*)MyCiRequestPtr);
	
	return(Result);
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_receive()                                                 */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function fetches an asynchronous acknowledgment with CI_receive.    */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_receive (CI_DP_REQUEST_T* MyCiResultPtr,DPR_DWORD user_handle,DPR_DWORD timeout)
{
	DPR_DWORD Result;
	
	MyCiResultPtr->header.user_handle        = (user_handle & 0x00ffffff);
	MyCiResultPtr->header.buf_length         = sizeof(CI_DP_ORDER_T);
	MyCiResultPtr->header.timeout            = timeout;
	
	Result = CI_receive ((CI_REQUEST_T*)MyCiResultPtr);
	
	return(Result);
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   build_result()                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function updates the DP_ERROR_T structure on completion of          */
/*  CI_receive or CI_send with asynchronous calls.                           */
/*                                                                           */
/*****************************************************************************/


void build_result (DPR_DWORD CiResult, CI_DP_REQUEST_T* MyCiRequestPtr,DP_ERROR_T* MyErrorPtr)
{
    CI_DP_ORDER_T* MyDpOrderPtr; 
	
    switch (CiResult)
    {
	case CI_RET_OK_DATA:
		{
			/* Data vailable, check the result */
			{
				MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequestPtr ->data;
				
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
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   build_result_async()                                         */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function updates the DP_ERROR_T structure on completion of the      */
/*  CI_send with asynchronous calls.                                         */
/*                                                                           */
/*****************************************************************************/


void build_result_async (DPR_DWORD CiResult, CI_DP_REQUEST_T* MyCiRequestPtr,DP_ERROR_T* MyErrorPtr)
{
    CI_DP_ORDER_T* MyDpOrderPtr; 
	
    switch (CiResult)
    {
	case CI_RET_OK_DATA:
		{
			/* data avilable, check the result */
			{
				MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequestPtr ->data;
				
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
			/* no data available, use DP_get_result to get the confirmation */
			MyErrorPtr -> error_class  = DP_OK_ASYNC; 
			MyErrorPtr -> error_code   = DP_OK;
			MyErrorPtr -> error_decode = 0;
			MyErrorPtr -> error_code_1 = 0;
			MyErrorPtr -> error_code_2 = 0;
			break;
			
		}
	case CI_RET_RECEIVE_TIMEOUT_NO_DATA:
		{
			/* no data available, use DP_get_result to get the confirmation */
			MyErrorPtr -> error_class  = DP_ERROR_EVENT_NET; 
			MyErrorPtr -> error_code   = DP_RET_TIMEOUT;
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
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   fetch_err_txt()                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function fetches the text for the DP error classes.                 */
/*                                                                           */
/*****************************************************************************/


size_t  fetch_err_txt (DPR_STRING* language, DPR_STRING* text, DP_ERROR_T* error)
{
    DPR_WORD   j;
    DPR_WORD   found;
    size_t     TotalLength;
    char       HlpTxt[ERR_NUM_SIZE];
    ERR_INF_T* ErrTxtPtr;
    DPR_WORD   BufLen;
    char       CiErrTxt[DP_ERR_TXT_SIZE];
    DPR_DWORD  HlpResult;
    DPR_DWORD  HlpErrorClass;
	
    BufLen      = DP_ERR_TXT_SIZE-1;
    TotalLength = 0;
    found = ERR_TXT_NOT_FOUND;
	
    /* text for error class */
	
    ErrTxtPtr = (ERR_INF_T*)&(ErrClass[0]);
	
	
    /* special cases: CI, DP_Error-Classes: Warning text instead of error text */
	
    if(error->error_code & INFO_MASK)
    {
		/* Info instead of error message, internal defines */
		switch (error->error_class)
		{
		case DP_ERROR_CI:
			{
				HlpErrorClass = DP_ERROR_CI_INFO;
				break;
			}
		case DP_ERROR_EVENT:
			{
				HlpErrorClass = DP_ERROR_EVENT_INFO;
				break;
			}
		case DP_ERROR_EVENT_NET:
			{
				HlpErrorClass = DP_ERROR_EVENT_NET_INFO;
				break;
			}
		case DP_ERROR_REQ_PAR:
			{
				HlpErrorClass = DP_ERROR_REQ_PAR_INFO;
				break;
			}
		case DP_ERROR_RES:
			{
				HlpErrorClass = DP_ERROR_RES_INFO;
				break;
			}
		case DP_ERROR_USR_ABORT:
			{
				HlpErrorClass = DP_ERROR_USR_ABORT_INFO;
				break;
			}
			
		default:
			{
				HlpErrorClass = error->error_class;
				break;
			}
		}
    }
    else
    {
		HlpErrorClass = error->error_class;
    }
	
	
    while(found == ERR_TXT_NOT_FOUND)
    {   
        if((HlpErrorClass == ErrTxtPtr -> errClassOrNum ) ||
            (ErrTxtPtr -> errClassOrNum == ERR_TXT_END))
        {
            found = ERR_TXT_FOUND;
            if(!stricmp (language, "German"))
            {
                TotalLength = strlen (ErrTxtPtr -> header_d) + ERR_NUM_SIZE;
                for (j=0;j<4;j++)
                {
					TotalLength += strlen (ErrTxtPtr -> errTxt_d[j]);
                }
                
                text[0] = '\0';
                if(TotalLength < BufLen)
                {
                    strcat (text,ErrTxtPtr -> header_d);
                    sprintf(HlpTxt," (%lxh):\n",error->error_class); /* here:error class! */
                    strcat(text,HlpTxt);
                    for (j=0;j<4;j++)
                    {
						strcat (text,ErrTxtPtr -> errTxt_d[j]);
                    } 
                    strcat (text,"\n");
					
                }
            }
            else /* english */
            {
                TotalLength = strlen (ErrTxtPtr -> header_e) + ERR_NUM_SIZE;
                for (j=0;j<4;j++)
                {
					TotalLength += strlen (ErrTxtPtr -> errTxt_e[j]);
                }
                
                text[0] = '\0';
                if(TotalLength < BufLen)
                {
                    strcat (text,ErrTxtPtr -> header_e);
                    sprintf(HlpTxt," (%lxh):\n",error->error_class);
                    strcat(text,HlpTxt);
                    for (j=0;j<4;j++)
                    {
						strcat (text,ErrTxtPtr -> errTxt_e[j]);
                    }
                    strcat (text,"\n");
					
                }
            }
        }
        ErrTxtPtr++;
    }
	
	
#ifdef LARGE_TEXT
	
    /* text for error code (depending on error class) */
    switch (error -> error_class)
    {
	case DP_OK:
	case DP_OK_ASYNC:
	case DP_ERROR_EVENT_NET:
	case DP_ERROR_REQ_PAR:
	case DP_ERROR_RES:
	case DP_ERROR_USR_ABORT:
		{
			/* mask here info-bit */
			if ((error->error_code & (~INFO_MASK)) < DPS_ERR_BEGIN)
			{
				/* Master Codes,  */
				ErrTxtPtr = (ERR_INF_T*)&(ErrCodeDp[0]);
			}
			else
			{
				/* Slave Codes */
				ErrTxtPtr = (ERR_INF_T*)&(ErrCodeSlv[0]);
			}
			break;
		}
	case DP_ERROR_EVENT:
		{
			ErrTxtPtr = (ERR_INF_T*)&(ErrCodeDpEvent[0]);
			break;
		}
	case DP_ERROR_CI:
		{
			/* CI has own function for errorcodes */
			CiErrTxt[0] = 0;
			HlpResult = CI_get_err_txt(error->error_code,language,CiErrTxt);
			if(HlpResult == CI_RET_OK)
			{
				TotalLength =  strlen(text) + strlen(CiErrTxt) + 1;
				if( TotalLength <=  DP_ERR_TXT_SIZE)
				{
					strcat(text,CiErrTxt);
				}
			}
			break;
		}
	default:
		{
			ErrTxtPtr = (ERR_INF_T*)&(ErrCodeDp[0]);
			break;
		}
    }
#else /* minimal text for error codes */
	
    ErrTxtPtr = (ERR_INF_T*)&(ErrCodeMinimal[0]);
#endif
	
	
    if(error->error_class != DP_ERROR_CI)
    {
        found = ERR_TXT_NOT_FOUND;
		
        while(found == ERR_TXT_NOT_FOUND)
        {   
            if((error->error_code == ErrTxtPtr -> errClassOrNum ) ||
                (ErrTxtPtr -> errClassOrNum == ERR_TXT_END))
            {
                found = ERR_TXT_FOUND;
                if(!stricmp (language, "German"))
                {
                    TotalLength += strlen (ErrTxtPtr -> header_d) + ERR_NUM_SIZE;
                    for (j=0;j<4;j++)
                    {
						TotalLength += strlen (ErrTxtPtr -> errTxt_d[j]);
                    }
					
                    if(TotalLength < BufLen)
                    {
                        strcat (text,ErrTxtPtr -> header_d);
#ifdef LARGE_TEXT
						sprintf(HlpTxt," (%lxh):\n",error->error_code);
#else
						sprintf(HlpTxt," (%lxh):\n",error->error_code);
#endif
                        strcat(text,HlpTxt);
                        for (j=0;j<4;j++)
                        {
							strcat (text,ErrTxtPtr -> errTxt_d[j]);
                        } 
                    }
                }
                else /* english */
                {
                    TotalLength += strlen (ErrTxtPtr -> header_e) + ERR_NUM_SIZE;
                    for (j=0;j<4;j++)
                    {
						TotalLength += strlen (ErrTxtPtr -> errTxt_e[j]);
                    }
					
                    if(TotalLength < BufLen)
                    {
                        strcat (text,ErrTxtPtr -> header_e);
#ifdef LARGE_TEXT
						sprintf(HlpTxt," (%lxh):\n",error->error_code);
#else
						sprintf(HlpTxt," %lxh:\n",error->error_code);
#endif
                        strcat(text,HlpTxt);
                        for (j=0;j<4;j++)
                        {
							strcat (text,ErrTxtPtr -> errTxt_e[j]);
                        } 
                    }
                }
            }
            ErrTxtPtr++;
        }
		
    }
	
	
    return(TotalLength);
} 



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   store_user()                                                 */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function stores a user handle in an array.                          */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD store_user (DPR_DWORD sync_handle,DPR_DWORD async_handle,
                      DPR_WORD cp_index,DPR_DWORD *user_handle)
{
	DPR_DWORD result;
	DPR_WORD  i;
	
	result = DP_RET_MEMORY;
	
	/* first user instance? */
	if (DpUserCtr == 0)
	{
		/* clear array */
		for(i=0;i <= DP_MAX_USER_INST;i++)
		{
			memset ( &(DpUserIdTable[i]),0,sizeof(DP_USER_ID_TABLE_T));
		}
	}
	
	
	/* search next free index, index 0 is reserved */
	
	for(i=1;i<= DP_MAX_USER_INST;i++)
	{
		if (DpUserIdTable[i].Used == 0)
		{
			DpUserIdTable[i].Used          = 0xff;
			DpUserIdTable[i].CpIndex       = cp_index;
			DpUserIdTable[i].DpSyncHandle  = sync_handle;
			DpUserIdTable[i].DpAsyncHandle = async_handle;
			
			*user_handle = i;
			DpUserCtr++;
			
			result = DP_OK;
			break;
		}
	}
	
	return(result);
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   release_user()                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function releases the user handle again.                            */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD release_user (DPR_DWORD user_handle)
{
	DPR_DWORD result;
	
	if ((DpUserCtr > 0) && (user_handle <= DP_MAX_USER_INST))
	{
		/* clear array */
		memset ( &(DpUserIdTable[user_handle]),0,sizeof(DP_USER_ID_TABLE_T));
		DpUserCtr--;
		result = DP_OK;
	}
	else
	{
		result = DP_RET_MEMORY;
	}
	
	return(result);
}





/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   get_next_user()                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function obtains the next user handle                               */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD get_next_user  (DPR_DWORD *user_handle)
{
	DPR_DWORD result;
	DPR_DWORD i;
	
	result = DP_RET_MEMORY;
	
	if (DpUserCtr == 0)
	{
		*user_handle = 0;
	}
	else
	{
		for(i=1;i<=DP_MAX_USER_INST;i++) /* index 0 is reserved */
		{
			if (DpUserIdTable[i].Used == 0xff)
			{
				*user_handle  = i;
				result = DP_OK;
				break;
			}
		}
	}
	
	return(result);
}




/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   get_slv_add_from_cref()                                      */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function obtains the slva address from the c_ref                    */
/*                                                                           */
/*****************************************************************************/


DPR_WORD get_slv_add_from_cref  (DPR_DWORD  c_ref)

{
	DPR_WORD SlvAdd;
	SlvAdd =  (DPR_WORD) ((c_ref & 0xffff0000) >> 16);
	if (SlvAdd > DPR_MAX_SLAVE_NR)
	{
		SlvAdd = DPR_MAX_SLAVE_ADDR; /* unused address */
	}
	return (SlvAdd);
}



