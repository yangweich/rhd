/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Version:    1.0                                                          */
/*  Module:     dpS_trc.c                                                    */
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

#include "dp_5613.h"
#include "dps_5614.h"
#include "ci_5613.h"

#include "dps_base.h"
#include "uif_def.h"
#include "fkt_def.h"
#include "trace.h"
#include "xtrace.h"


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcGetString                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Puts strings together.                                                   */
/*                                                                           */
/*****************************************************************************/

void DpsTrcGetString (DPR_STRING* MyBuf,DPR_STRING* MyText,DPR_STRING* string_var)
{
	
	if ( ( (strlen(MyBuf)) + (strlen(MyText)) ) < TRC_TEXT_BUF_SIZE)
	{
		strcat(MyBuf,MyText);
		if (string_var)
		{
			if ( ( (strlen(MyBuf)) + (strlen(string_var)) +1) < TRC_TEXT_BUF_SIZE)
			{
				strcat(MyBuf,string_var);
				strcat(MyBuf,", ");
				
			}
		}
		else  /* Null-Pointer */
		{
			if ( ( (strlen(MyBuf)) + (strlen("NULL")) +1) < TRC_TEXT_BUF_SIZE)
			{
				strcat(MyBuf,"NULL");
				strcat(MyBuf,", ");
				
			}
		}
	}
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcSetString                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Copies a string.                                                         */
/*                                                                           */
/*****************************************************************************/

void DpsTrcSetString (DPR_STRING* MyBuf,DPR_STRING* MyText)
{
	if ( ( (strlen(MyBuf)) + (strlen(MyText)) + 1 ) < TRC_TEXT_BUF_SIZE)
	{
		strcat(MyBuf,MyText);
		strcat(MyBuf,", ");
	}
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcGetVal                                                 */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Creates a formatted string.                                              */
/*                                                                           */
/*****************************************************************************/

void DpsTrcGetVal(DPR_STRING* MyBuf,DPR_STRING* MyFormatString,DPR_DWORD MyValue)
{
	DPR_STRING MyHlp[TRC_TEXT_BUF_SIZE];
	
	sprintf(MyHlp,MyFormatString,MyValue);
	if ( ( (strlen(MyBuf)) + (strlen(MyHlp)) + 1 ) < TRC_TEXT_BUF_SIZE)
	{
		strcat(MyBuf,MyHlp);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcGetValByPtr                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Creates a formatted string.                                              */
/*                                                                           */
/*****************************************************************************/

void DpsTrcGetValByPtr(DPR_STRING* MyBuf,DPR_STRING* MyFormatString,DPR_DWORD *MyValuePtr)
{
	DPR_STRING MyHlp[TRC_TEXT_BUF_SIZE];
	
	if (MyValuePtr)
	{
		sprintf(MyHlp,MyFormatString,*MyValuePtr);
	}
	else
	{
		sprintf(MyHlp,MyFormatString,0);
		strcat (MyHlp," (Null-Pointer)");
	}
	if ( ( (strlen(MyBuf)) + (strlen(MyHlp)) + 1) < TRC_TEXT_BUF_SIZE)
	{
		strcat(MyBuf,MyHlp);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcGetErrVal                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets error information from the DP_ERROR_T structure.                    */
/*                                                                           */
/*****************************************************************************/

void DpsTrcGetErrVal (DPR_STRING* MyBuf, DP_ERROR_T* error)
{
	if (error)
	{
		DpsTrcGetVal(MyBuf,"error(%x,",(DPR_DWORD)(error->error_class));
		DpsTrcGetVal(MyBuf,"%x,",      (DPR_DWORD)(error->error_code));
		DpsTrcGetVal(MyBuf,"%x,",      (DPR_DWORD)(error->error_decode));
		DpsTrcGetVal(MyBuf,"%x,",      (DPR_DWORD)(error->error_code_1));
		DpsTrcGetVal(MyBuf,"%x) ",     (DPR_DWORD)(error->error_code_2));
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcOpen                                                   */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_open                                     */
/*                                                                           */
/*****************************************************************************/


void DpsTrcOpen                     (DPR_STRING       *cp_name,
									 DPR_DWORD       user_handle,
									 DPR_DWORD       slave_mode,
									 DPR_WORD        station_addr,
									 DPR_WORD        addr_change,
									 DPR_WORD        pno_ident_nr,
									 DPR_WORD        reserved,
									 DPS_INIT_DATA_T *init_data,
									 DPS_MAX_DATA_T  *max_data_lens,
									 DPR_WORD        baud_rate,
									 DP_ERROR_T      *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,"<>DPS_open: ");
        DpsTrcGetErrVal (MyTxt,error);
        DpsTrcGetString (MyTxt,"cp_name: ",cp_name);
        DpsTrcGetVal    (MyTxt,"usr_handle: %x ",user_handle);
        DpsTrcGetVal    (MyTxt,"slave_mode: %x ",slave_mode);
        DpsTrcGetVal    (MyTxt,"station_addr: %x ",station_addr);
        DpsTrcGetVal    (MyTxt,"addr_change: %x ",addr_change);
        DpsTrcGetVal    (MyTxt,"pno_ident_nr: %x ",pno_ident_nr);
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_OPEN,
			SELECT_DPS_OPEN_CLOSE,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcClose                                                  */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_close                                    */
/*                                                                           */
/*****************************************************************************/

void DpsTrcClose                    (DPR_DWORD    user_handle,
									 DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,"<>DPS_close: ");
        DpsTrcGetVal    (MyTxt,"user_handle: %x ",user_handle);
        DpsTrcGetErrVal (MyTxt,error);
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_CLOSE,
			SELECT_DPS_OPEN_CLOSE,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcStart                                                  */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_start                                    */
/*                                                                           */
/*****************************************************************************/

void DpsTrcStart                    (DPR_DWORD    user_handle,
									 DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,"<>DPS_start: ");
        DpsTrcGetVal    (MyTxt,"user_handle: %x ",user_handle);
        DpsTrcGetErrVal (MyTxt,error);
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_START,
			SELECT_DPS_START_STOP,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcStop                                                   */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_stop                                     */
/*                                                                           */
/*****************************************************************************/

void DpsTrcStop                     (DPR_DWORD    user_handle,
									 DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,"<>DPS_stop: ");
        DpsTrcGetVal    (MyTxt,"user_handle: %x ",user_handle);
        DpsTrcGetErrVal (MyTxt,error);
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_STOP,
			SELECT_DPS_START_STOP,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcGetBaudRate                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_get_baud_rate                            */
/*                                                                           */
/*****************************************************************************/

void DpsTrcGetBaudRate              (DPR_DWORD    user_handle,
									 DPR_WORD   state,
									 DPR_WORD   baud_rate,
									 DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,"<>DPS_get_baud_rate: ");
        DpsTrcGetVal    (MyTxt,"user_handle: %x ",user_handle);
        DpsTrcGetVal    (MyTxt,"state: %x ",state);
        DpsTrcGetVal    (MyTxt,"baud_rate: %x ",baud_rate);
        DpsTrcGetErrVal (MyTxt,error);
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_GET_BAUD_RATE,
			SELECT_DPS_GET_BAUD_RATE,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcGetGcCommand                                           */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_get_gc_command                           */
/*                                                                           */
/*****************************************************************************/

void DpsTrcGetGcCommand            (DPR_DWORD    user_handle,
                                    DPR_WORD   gc,
                                    DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,"<>DPS_get_gc_command: ");
        DpsTrcGetVal    (MyTxt,"user_handle: %x ",user_handle);
        DpsTrcGetVal    (MyTxt,"gc_cmd: %x ",gc);
        DpsTrcGetErrVal (MyTxt,error);
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_GET_GC_COMMAND,
			SELECT_DPS_GET_GC_COMMAND,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcGetState                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_get_state                                */
/*                                                                           */
/*****************************************************************************/

void DpsTrcGetState                 (DPR_DWORD    user_handle,
									 DPR_WORD   state,       /* out  */
									 DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,"<>DPS_get_state: ");
        DpsTrcGetVal    (MyTxt,"user_handle: %x ",user_handle);
        DpsTrcGetVal    (MyTxt,"dps_state: %x ",state);
        DpsTrcGetErrVal (MyTxt,error);
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_GET_STATE,
			SELECT_DPS_GET_STATE,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcSetDiag                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_set_diag                                 */
/*                                                                           */
/*****************************************************************************/

void DpsTrcSetDiag                  (DPR_DWORD    user_handle,
									 DPR_BYTE   *user_diag_data,
									 DPR_WORD   user_diag_len,
									 DPR_WORD   diag_state,
									 DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,"<>DPS_set_diag: ");
        DpsTrcGetVal    (MyTxt,"user_handle: %x ",user_handle);
        DpsTrcGetVal    (MyTxt,"user_diag_data: %8.8x ",*((DPR_DWORD*)user_diag_data));
        DpsTrcGetVal    (MyTxt,"user_diag_len: %x ",user_diag_len);
        DpsTrcGetVal    (MyTxt,"diag_state: %x ",diag_state);
        DpsTrcGetErrVal (MyTxt,error);
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_SET_DIAG,
			SELECT_DPS_SET_DIAG,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcCalcIoDataLen                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_calc_io_data_len                         */
/*                                                                           */
/*****************************************************************************/

void DpsTrcCalcIoDataLen            (
									 DPR_WORD   cfg_len,
									 DPR_BYTE   *cfg_data,
									 DPR_WORD   in_data_len,
									 DPR_WORD   out_data_len,
									 DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        DpsTrcGetErrVal (MyTxt,error);
        strcpy(MyTxt,"<>DPS_calc_io_data_len: ");
        DpsTrcGetVal    (MyTxt,"cfg_len: %x ",cfg_len);
        DpsTrcGetVal    (MyTxt,"cfg_data: %8.8x ",*((DPR_DWORD*)cfg_data));
        DpsTrcGetVal    (MyTxt,"in_data_len: %x ",in_data_len);
        DpsTrcGetVal    (MyTxt,"out_data_len: %x ",out_data_len);
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_CALC_IO_DATA_LEN,
			SELECT_DPS_CALC_IO_DATA_LEN,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcGetIndBegin                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_get_ind                                  */
/*                                                                           */
/*****************************************************************************/

void DpsTrcGetIndBegin              (DPR_DWORD    user_handle,
									 DPR_DWORD   timeout,
									 DPR_DWORD   indication,
									 DPR_WORD   data_len,
									 DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,">DPS_get_ind: ");
        DpsTrcGetErrVal (MyTxt,error);
        DpsTrcGetVal    (MyTxt,"user_handle: %x ",user_handle);
        DpsTrcGetVal    (MyTxt,"timeout: %x ",timeout);
        DpsTrcGetVal    (MyTxt,"indication: %x ",indication);
        DpsTrcGetVal    (MyTxt,"data_len: %x ",data_len);
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_GET_IND,
			SELECT_DPS_GET_IND,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcGetIndEnd                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_get_ind                                  */
/*                                                                           */
/*****************************************************************************/

void DpsTrcGetIndEnd                (DPR_DWORD    user_handle,
									 DPR_DWORD   ind_ref,
									 DPR_DWORD   indication,
									 DPR_WORD   data_len,
									 DPR_BYTE   *data_blk,
									 DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,"<DPS_get_ind: ");
        DpsTrcGetErrVal (MyTxt,error);
        DpsTrcGetVal    (MyTxt,"user_handle: %x ",user_handle);
        DpsTrcGetVal    (MyTxt,"ind_ref: %x ",ind_ref);
        DpsTrcGetVal    (MyTxt,"indication: %x ",indication);
        DpsTrcGetVal    (MyTxt,"data_len: %x ",data_len);
        DpsTrcGetVal    (MyTxt,"data_blk: %8.8x ",*((DPR_DWORD*)data_blk));
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_GET_IND,
			SELECT_DPS_GET_IND,
			MyTxt);
    }
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DpsTrcSetResp                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from DPS_set_resp                                 */
/*                                                                           */
/*****************************************************************************/

void DpsTrcSetResp                  (DPR_DWORD    user_handle,
									 DPR_DWORD   ind_ref,
									 DPR_WORD   data_len,
									 DPR_BYTE   *data_blk,
									 DP_ERROR_T   *error)
{
    DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
    if (DpsTrcIndex != 0)
    {
        strcpy(MyTxt,"<>DPS_set_resp: ");
        DpsTrcGetErrVal (MyTxt,error);
        DpsTrcGetVal    (MyTxt,"user_handle: %x ",user_handle);
        DpsTrcGetVal    (MyTxt,"ind_ref: %x ",ind_ref);
        DpsTrcGetVal    (MyTxt,"data_len: %x ",data_len);
        DpsTrcGetVal    (MyTxt,"data_blk: %8.8x ",*((DPR_DWORD*)data_blk));
		
        X_Trc_write_txt (DpsTrcIndex,
			LEVEL_DPS_SET_RESP,
			SELECT_DPS_SET_RESP,
			MyTxt);
    }
}



#ifdef WIN32
#pragma warning( default : 4201 4214 4115 4100)
#endif

