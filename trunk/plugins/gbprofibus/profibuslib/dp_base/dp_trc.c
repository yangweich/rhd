/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Module:     dp_trc.c                                                     */
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

#ifdef linux
#include <stdio.h>
#endif

#include "dp_5613.h"
#include "ci_5613.h"

#include "dp_base.h"
#include "uif_def.h"
#include "fkt_def.h"
#include "trace.h"
#include "xtrace.h"


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcGetString                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Puts strings together.                                                   */
/*                                                                           */
/*****************************************************************************/

void DpTrcGetString (DPR_STRING* MyBuf,DPR_STRING* MyText,DPR_STRING* string_var)
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
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcSetString                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Copies a string.                                                         */
/*                                                                           */
/*****************************************************************************/

void DpTrcSetString (DPR_STRING* MyBuf,DPR_STRING* MyText)
{
	if ( ( (strlen(MyBuf)) + (strlen(MyText)) + 1 ) < TRC_TEXT_BUF_SIZE)
	{
		strcat(MyBuf,MyText);
		strcat(MyBuf,", ");
	}
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcGetVal                                                  */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Creates a formatted string.                                              */
/*                                                                           */
/*****************************************************************************/

void DpTrcGetVal(DPR_STRING* MyBuf,DPR_STRING* MyFormatString,DPR_DWORD MyValue)
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
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcGetValByPtr                                             */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Creates a formatted string.                                              */
/*                                                                           */
/*****************************************************************************/

void DpTrcGetValByPtr(DPR_STRING* MyBuf,DPR_STRING* MyFormatString,DPR_DWORD *MyValuePtr)
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
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcGetErrVal                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets error information from the DP_ERROR_T structure.                    */
/*                                                                           */
/*****************************************************************************/

void DpTrcGetErrVal (DPR_STRING* MyBuf, DP_ERROR_T* error)
{
	if (error)
	{
		DpTrcGetVal(MyBuf,"error(%x,",(DPR_DWORD)(error->error_class));
		DpTrcGetVal(MyBuf,"%x,",      (DPR_DWORD)(error->error_code));
		DpTrcGetVal(MyBuf,"%x,",      (DPR_DWORD)(error->error_decode));
		DpTrcGetVal(MyBuf,"%x,",      (DPR_DWORD)(error->error_code_1));
		DpTrcGetVal(MyBuf,"%x) ",     (DPR_DWORD)(error->error_code_2));
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcStartCp                                                 */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_start_cp request.                     */
/*                                                                           */
/*****************************************************************************/


void  DpTrcStartCp       (DPR_STRING *cp_name,
                          DPR_STRING *database,
                          DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,"<>DP_start_cp: ");
		DpTrcGetErrVal (MyTxt,error);
		DpTrcGetString (MyTxt,"cp_name: ",cp_name);
		DpTrcGetString (MyTxt,"dbase: ",database); 
		X_Trc_write_txt (DpTrcIndex,  
			LEVEL_DP_START_CP,
			SELECT_DP_START_CP,
			MyTxt);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcResetCp                                                 */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_reset_cp request.                     */
/*                                                                           */
/*****************************************************************************/


void  DpTrcResetCp       (DPR_STRING *cp_name,
                          DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,"<>DP_reset_cp: ");
		DpTrcGetErrVal (MyTxt,error);
		DpTrcGetString (MyTxt,"cp_name: ",cp_name);
		X_Trc_write_txt (DpTrcIndex,  
			LEVEL_DP_RESET_CP,
			SELECT_DP_RESET_CP,
			MyTxt);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcOpen                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_open request.                         */
/*                                                                           */
/*****************************************************************************/


void  DpTrcOpen   (DPR_STRING *cp_name,
                   DPR_DWORD  *user_handle,
                   DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,"<>DP_open: ");
		DpTrcGetErrVal (MyTxt,error);
		DpTrcGetString (MyTxt,"cp_name: ",cp_name);
		DpTrcGetValByPtr (MyTxt,"usr_hdl: %x ",user_handle); 
		X_Trc_write_txt (DpTrcIndex, 
			LEVEL_DP_OPEN,
			SELECT_DP_OPEN,
			MyTxt);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcClose                                                   */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_close request.                        */
/*                                                                           */
/*****************************************************************************/


void  DpTrcClose   (DPR_DWORD  user_handle,
                    DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,"<>DP_close: ");
		DpTrcGetErrVal (MyTxt,error);
		DpTrcGetVal    (MyTxt,"usr_hdl: %x ",user_handle); 
		X_Trc_write_txt  (DpTrcIndex, 
			LEVEL_DP_CLOSE,
			SELECT_DP_CLOSE,
			MyTxt);
	}
	
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcGetPointer                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_get_pointer request.                  */
/*                                                                           */
/*****************************************************************************/


void  DpTrcGetPointer  (DPR_DWORD user_handle,
						DPR_DWORD timeout,
						DPR_CP5613_DP_T volatile** dpr,
						DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_get_pointer: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle); 
		DpTrcGetVal      (MyTxt,"timeout: %x ",timeout); 
		DpTrcGetValByPtr (MyTxt,"dpr: %x ",(DPR_DWORD*)dpr); 
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_GET_PTR,
			SELECT_DP_GET_PTR,
			MyTxt);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcRelPointer                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_release_pointer request.              */
/*                                                                           */
/*****************************************************************************/


void  DpTrcRelPointer   (DPR_DWORD  user_handle,
                         DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,"<>DP_release_ptr ");
		DpTrcGetErrVal (MyTxt,error);
		DpTrcGetVal    (MyTxt,"usr_hdl: %x ",user_handle); 
		X_Trc_write_txt  (DpTrcIndex, 
			LEVEL_DP_REL_PTR,
			SELECT_DP_REL_PTR,
			MyTxt);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcSetMode                                                 */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_set_mode request.                     */
/*                                                                           */
/*****************************************************************************/


void  DpTrcSetMode     (DPR_DWORD user_handle,
						DPR_WORD mode,
						DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_set_mode: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle); 
		DpTrcGetVal      (MyTxt,"mode: %x ",mode); 
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_SET_MODE,
			SELECT_DP_SET_MODE,
			MyTxt);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcSlvState                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_slv_state request.                    */
/*                                                                           */
/*****************************************************************************/

void  DpTrcSlvState    (DPR_DWORD  user_handle,
						DPR_WORD   slv_add,
						DPR_WORD   mode,
						DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_slv_state: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetVal      (MyTxt,"slv_add: %x ",slv_add); 
		DpTrcGetVal      (MyTxt,"mode: %x ",mode); 
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_SLV_STATE,
			SELECT_DP_SLV_STATE,
			MyTxt);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcGlobalCtrl                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_global_ctrl request.                  */
/*                                                                           */
/*****************************************************************************/

void  DpTrcGlobalCtrl  (DPR_DWORD  user_handle,
						DPR_WORD   slv_add,
						DPR_BYTE   command,
						DPR_BYTE   group,
						DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_global_control: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetVal      (MyTxt,"slv_add: %x ",slv_add);
		
		DpTrcGetVal (MyTxt,"command: %x ",(DPR_DWORD)command);
		DpTrcGetVal (MyTxt,"group: %x ",(DPR_DWORD)group);
		
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_GLB_CTR,
			SELECT_DP_GLB_CTR,
			MyTxt);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcGetCref                                                 */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_get_cref request.                     */
/*                                                                           */
/*****************************************************************************/

void  DpTrcGetCref    (DPR_DWORD  user_handle,
					   DPR_WORD   slv_add,
					   DPR_DWORD  *c_ref,
					   DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_get_c_ref: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetVal      (MyTxt,"slv_add: %x ",slv_add); 
		DpTrcGetValByPtr (MyTxt,"cref: %x ",c_ref); 
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_GET_CREF,
			SELECT_DP_GET_CREF,
			MyTxt);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcReadSlvPar                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_read_slv_par request.                 */
/*                                                                           */
/*****************************************************************************/

void  DpTrcReadSlvPar (DPR_DWORD  user_handle,
					   DPR_WORD   slv_add,
					   DPR_WORD   type,
					   DPR_WORD*  len,
					   DPR_BYTE*  data,
					   DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_read_slv_par: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetVal      (MyTxt,"slv_add: %x ",slv_add); 
		DpTrcGetVal      (MyTxt,"type: %x ",type); 
		DpTrcGetValByPtr (MyTxt,"length: %2.2x ",(DPR_DWORD*)len); 
		if (data)
		{
			X_Trc_write        (DpTrcIndex, 
				LEVEL_DP_READ_SLV_PAR,
				SELECT_DP_READ_SLV_PAR,
				MyTxt,
				(char*)data,
				*len);
		}
		else
		{
			DpTrcSetString(MyTxt,"data (Null-Pointer)"); 
			X_Trc_write_txt    (DpTrcIndex, 
				LEVEL_DP_READ_SLV_PAR,
				SELECT_DP_READ_SLV_PAR,
				MyTxt);
		}
		
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcReadAlarm                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_read_alarm request.                   */
/*                                                                           */
/*****************************************************************************/

void  DpTrcReadAlarm (DPR_DWORD    user_handle,
					  DPR_WORD    slv_add,
					  DP_ALARM_T* alarm,
					  DP_ERROR_T* error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_read_alarm: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetVal      (MyTxt,"slv_add: %x ",slv_add);
		if (alarm)
		{
			DpTrcGetVal      (MyTxt,"msg: %x ",(DPR_DWORD) alarm->msg); 
			DpTrcGetVal      (MyTxt,"add: %x ",(DPR_DWORD) alarm->slv_add); 
			DpTrcGetVal      (MyTxt,"slot: %x ",(DPR_DWORD)alarm->slot_number); 
			DpTrcGetVal      (MyTxt,"type: %x ",(DPR_DWORD)alarm->alarm_type); 
			DpTrcGetVal      (MyTxt,"spec: %x ",(DPR_DWORD)alarm->specifier); 
			DpTrcGetVal      (MyTxt,"len: %x ",(DPR_DWORD) alarm->length_s);
			
			X_Trc_write        (DpTrcIndex, 
				LEVEL_DP_READ_ALARM,
				SELECT_DP_READ_ALARM,
				MyTxt,
				(char*)alarm->data_s,
				alarm->length_s);
		}
		else
		{
			DpTrcSetString(MyTxt,"alarm (Null-Pointer)"); 
			X_Trc_write_txt    (DpTrcIndex, 
				LEVEL_DP_READ_ALARM,
				SELECT_DP_READ_ALARM,
				MyTxt);
		}
	}
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcGetErrTxt                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_get_err_txt request.                  */
/*                                                                           */
/*****************************************************************************/

void  DpTrcGetErrTxt  (DP_ERROR_T *error,
					   DPR_STRING *language,
					   DPR_STRING text[DP_ERR_TXT_SIZE])
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_get_error_txt: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetString   (MyTxt,"language:",language);
		DpTrcGetString   (MyTxt,"text:",text);
		
		X_Trc_write_txt  (DpTrcIndex,
			LEVEL_DP_GET_ERR_TXT,
			SELECT_DP_GET_ERR_TXT,
			MyTxt);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcInitSemaObj                                             */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_init-sema_object request.             */
/*                                                                           */
/*****************************************************************************/

void  DpTrcInitSemaObj  (DPR_DWORD  user_handle,
						 DPR_DWORD  sema_type,
						 DPR_DWORD  *sema_handle,
						 DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_init_sema_object: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetVal      (MyTxt,"sem_typ: %x ",sema_type);
		DpTrcGetValByPtr (MyTxt,"sema_handle: %x ",sema_handle); 
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_INIT_SEMA_OBJ,
			SELECT_DP_INIT_SEMA_OBJ,
			MyTxt);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcDelSemaObj                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/
void  DpTrcDelSemaObj  (DPR_DWORD  user_handle,
						DPR_DWORD  sema_handle,
						DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_delete_sema_object: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetVal      (MyTxt,"sema_handle: %x ",sema_handle); 
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_DEL_SEMA_OBJ,
			SELECT_DP_DEL_SEMA_OBJ,
			MyTxt);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcDsRead                                                  */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_ds_read request.                      */
/*                                                                           */
/*****************************************************************************/

void  DpTrcDsRead         (DPR_DWORD  user_handle,
						   DPC1_REQ_T *request, 
						   DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   ">DP_ds_read: ");
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetErrVal   (MyTxt,error);
		if(request)
		{
			DpTrcGetVal (MyTxt,"order_id: %x ",(DPR_DWORD)request->order_id); 
			DpTrcGetVal (MyTxt,"cref: %x ",(DPR_DWORD)request->c_ref); 
			
			DpTrcGetVal (MyTxt,"slot: %x ",(DPR_DWORD)request->req.dp_ds_read.slot_number); 
			DpTrcGetVal (MyTxt,"index: %x ",(DPR_DWORD)request->req.dp_ds_read.index); 
			DpTrcGetVal (MyTxt,"length: %x ",(DPR_DWORD)request->req.dp_ds_read.length_s); 
		}
		else
		{
			DpTrcSetString(MyTxt,"request(Null-Pointer)"); 
		}
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_DS_READ,
			SELECT_DP_DS_READ,
			MyTxt);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcDsWrite                                                 */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_ds_write request.                     */
/*                                                                           */
/*****************************************************************************/

void  DpTrcDsWrite        (DPR_DWORD  user_handle,
						   DPC1_REQ_T *request, 
						   DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   ">DP_ds_write: ");
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetErrVal   (MyTxt,error);
		if(request)
		{
			DpTrcGetVal (MyTxt,"o_id: %x ",(DPR_DWORD)request->order_id); 
			DpTrcGetVal (MyTxt,"cref: %x ",(DPR_DWORD)request->c_ref); 
			
			DpTrcGetVal (MyTxt,"slot: %x ",(DPR_DWORD)request->req.dp_ds_write.slot_number); 
			DpTrcGetVal (MyTxt,"idx: %x ",(DPR_DWORD)request->req.dp_ds_write.index); 
			DpTrcGetVal (MyTxt,"lng: %x ",(DPR_DWORD)request->req.dp_ds_write.length_m); 
			
			X_Trc_write        (DpTrcIndex, 
				LEVEL_DP_DS_WRITE,
				SELECT_DP_DS_WRITE,
				MyTxt,
				(char*)request->req.dp_ds_write.data_m,
				request->req.dp_ds_write.length_m);
		}
		else
		{
			DpTrcSetString(MyTxt,"request(Null-Pointer)");
			X_Trc_write_txt    (DpTrcIndex, 
				LEVEL_DP_DS_WRITE,
				SELECT_DP_DS_WRITE,
				MyTxt);
		}
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcAlarmAck                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_alarm_ack request.                    */
/*                                                                           */
/*****************************************************************************/

void  DpTrcAlarmAck       (DPR_DWORD  user_handle,
						   DPC1_REQ_T *request, 
						   DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   ">DP_alarm_ack: ");
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetErrVal   (MyTxt,error);
		if(request)
		{
			DpTrcGetVal (MyTxt,"order_id: %x ",(DPR_DWORD)request->order_id); 
			DpTrcGetVal (MyTxt,"cref: %x ",(DPR_DWORD)request->c_ref); 
			
			DpTrcGetVal (MyTxt,"slot: %x ",(DPR_DWORD)request->req.dp_alarm_ack.slot_number); 
			DpTrcGetVal (MyTxt,"alarm_type: %x ",(DPR_DWORD)request->req.dp_alarm_ack.alarm_type); 
			DpTrcGetVal (MyTxt,"specifier: %x ",(DPR_DWORD)request->req.dp_alarm_ack.specifier); 
			
		}
		else
		{
			DpTrcSetString(MyTxt,"request(Null-Pointer)");
		}
		
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_ALARM_ACK,
			SELECT_DP_ALARM_ACK,
			MyTxt);
		
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcGetActualCfg                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_get_actual_cfg request.               */
/*                                                                           */
/*****************************************************************************/

void  DpTrcGetActualCfg   (DPR_DWORD  user_handle,
						   DPC1_REQ_T *request, 
						   DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   ">DP_get_act_cfg: ");
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetErrVal   (MyTxt,error);
		if(request)
		{
			DpTrcGetVal (MyTxt,"order_id: %x ",(DPR_DWORD)request->order_id); 
			DpTrcGetVal (MyTxt,"cref: %x ",(DPR_DWORD)request->c_ref); 
			DpTrcGetVal (MyTxt,"length: %x ",(DPR_DWORD)request->req.dp_get_cfg.length_s); 
		}
		else
		{
			DpTrcSetString(MyTxt,"request(Null-Pointer)");
		}
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_GET_ACT_CFG,
			SELECT_DP_GET_ACT_CFG,
			MyTxt);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcEnableEvt                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_enable_event request.                 */
/*                                                                           */
/*****************************************************************************/

void  DpTrcEnableEvt            (DPR_DWORD  user_handle,
								 DPC1_REQ_T *request, 
								 DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   ">DP_enable_evt: ");
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetErrVal   (MyTxt,error);
		if(request)
		{
			DpTrcGetVal (MyTxt,"order_id: %x ",(DPR_DWORD)request->order_id); 
			DpTrcGetVal (MyTxt,"selector: %x ",(DPR_DWORD)request->req.dp_enable_evt.selector);
			DpTrcGetVal (MyTxt,"mst_state: %x ",(DPR_DWORD)request->req.dp_enable_evt.mst_state);
			
		}
		else
		{
			DpTrcSetString(MyTxt,"request(Null-Pointer)");
		}
		X_Trc_write_txt  (DpTrcIndex, 
			LEVEL_DP_ENABLE_EVT,
			SELECT_DP_ENABLE_EVT,
			MyTxt);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcDisableEvt                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_disable_event request.                */
/*                                                                           */
/*****************************************************************************/

void  DpTrcDisableEvt           (DPR_DWORD  user_handle,
								 DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_disable_evt: ");
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetErrVal   (MyTxt,error);
		
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_DISABLE_EVT,
			SELECT_DP_DISABLE_EVT,
			MyTxt);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcGetResult                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_get_result request.                   */
/*                                                                           */
/*****************************************************************************/

void  DpTrcGetResult      (DPR_DWORD  user_handle,
						   DPR_DWORD  timeout,
						   DPR_WORD  *req_type, 
						   DPC1_REQ_T *result, 
						   DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	char* HlpPtr = 0;
	byte  HlpLen = 0;
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<DP_get_result ");
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"timeout: %x ",timeout); 
		if(result && req_type)
		{
			DpTrcGetVal (MyTxt,"order_id: %x ",(DPR_DWORD)result->order_id); 
			DpTrcGetVal (MyTxt,"cref: %x ",(DPR_DWORD)result->c_ref); 
			
			switch (*req_type)
			{
			case DP_NO_CNF:
				{
					DpTrcGetString (MyTxt,"req_type: ","DP_NO_CNF");                  
					break;
				}
			case DP_DS_READ:
				{
					DpTrcGetString (MyTxt,"req_type: ","DP_DS_READ");
					DpTrcGetVal (MyTxt,"slot: %x ",(DPR_DWORD)result->req.dp_ds_read.slot_number); 
					DpTrcGetVal (MyTxt,"index: %x ",(DPR_DWORD)result->req.dp_ds_read.index); 
					DpTrcGetVal (MyTxt,"length: %x ",(DPR_DWORD)result->req.dp_ds_read.length_s);           
					HlpPtr = (char*)result->req.dp_ds_read.data_s;
					HlpLen = result->req.dp_ds_read.length_s;
					break;
				}
			case DP_DS_WRITE:
				{
					DpTrcGetString (MyTxt,"req_type: ","DP_DS_WRITE");
					DpTrcGetVal (MyTxt,"slot: %x ",(DPR_DWORD)result->req.dp_ds_write.slot_number); 
					DpTrcGetVal (MyTxt,"index: %x ",(DPR_DWORD)result->req.dp_ds_write.index); 
					DpTrcGetVal (MyTxt,"length: %x ",(DPR_DWORD)result->req.dp_ds_write.length_m);           
					HlpPtr = (char*)result->req.dp_ds_write.data_m;
					HlpLen = result->req.dp_ds_write.length_m;
					break;
				}
			case DP_ALARM_ACK:
				{
					DpTrcGetString (MyTxt,"req_type: ","DP_ALARM_ACK");         
					DpTrcGetVal (MyTxt,"slot: %x ",(DPR_DWORD)result->req.dp_alarm_ack.slot_number); 
					DpTrcGetVal (MyTxt,"alarm_type: %x ",(DPR_DWORD)result->req.dp_alarm_ack.alarm_type); 
					DpTrcGetVal (MyTxt,"specifier: %x ",(DPR_DWORD)result->req.dp_alarm_ack.specifier); 
					
					break;
				}
			case DP_ENABLE_EVENT:
				{
					DpTrcGetString (MyTxt,"req_type: ","DP_ENABLE_EVENT");
					DpTrcGetVal (MyTxt,"selector: %x ",(DPR_DWORD)result->req.dp_enable_evt.selector);
					DpTrcGetVal (MyTxt,"mst_state: %x ",(DPR_DWORD)result->req.dp_enable_evt.mst_state);
					HlpPtr = (char*)result->req.dp_enable_evt.event;
					HlpLen = DPR_MAX_SLAVE_NR;
					break;
				}
			case DP_GET_CFG:
				{
					DpTrcGetString (MyTxt,"req_type: ","DP_GET_CFG");
					HlpLen = result->req.dp_get_cfg.length_s;
					HlpPtr = (char*)result->req.dp_get_cfg.data_s;
					break;
				}
			default:
				{
					break;
				}
			}
			
			if (HlpPtr)
			{
				X_Trc_write (DpTrcIndex, 
					LEVEL_DP_GET_RESULT,
					SELECT_DP_GET_RESULT,
					MyTxt,
					HlpPtr,
					HlpLen);
			}
			else
			{
				X_Trc_write_txt    (DpTrcIndex, 
					LEVEL_DP_GET_RESULT,
					SELECT_DP_GET_RESULT,
					MyTxt);
				
			}
			
		}
		else
		{
			X_Trc_write_txt    (DpTrcIndex, 
				LEVEL_DP_GET_RESULT,
				SELECT_DP_GET_RESULT,
				MyTxt);
		}
	}
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcFastLogicOn                                             */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_fast_logic_on request.                */
/*                                                                           */
/*****************************************************************************/

void  DpTrcFastLogicOn          (DPR_DWORD  user_handle,
								 DPR_WORD fast_logic_id,
								 DP_FAST_LOGIC_T *fast_logic,
								 DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_fast_logic_on: ");
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"fl_id: %x ",(DPR_DWORD)fast_logic_id);
		DpTrcGetVal      (MyTxt,"slv_in: %x ",(DPR_DWORD)fast_logic->slave_addr_in_byte); 
		DpTrcGetVal      (MyTxt,"idx_in: %x ",(DPR_DWORD)fast_logic->index_in_byte); 
		DpTrcGetVal      (MyTxt,"cmp_in: %x ",(DPR_DWORD)fast_logic->cmp_value_in_byte); 
		DpTrcGetVal      (MyTxt,"msk_in: %x ",(DPR_DWORD)fast_logic->mask_in_byte); 
		DpTrcGetVal      (MyTxt,"slv_out: %x ",(DPR_DWORD)fast_logic->slave_addr_out_byte); 
		DpTrcGetVal      (MyTxt,"idx_out: %x ",(DPR_DWORD)fast_logic->index_out_byte); 
		DpTrcGetVal      (MyTxt,"val_out: %x ",(DPR_DWORD)fast_logic->value_out_byte); 
		DpTrcGetVal      (MyTxt,"msk_out: %x ",(DPR_DWORD)fast_logic->mask_out_byte); 
		
		
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_FL_ON_OFF,
			SELECT_DP_FL_ON_OFF,
			MyTxt);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcFastLogicOff                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_fast_logic_off request.               */
/*                                                                           */
/*****************************************************************************/

void  DpTrcFastLogicOff         (DPR_DWORD  user_handle,
								 DPR_WORD   fast_logic_id,
								 DP_ERROR_T *error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_fast_logic_on: ");
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetVal      (MyTxt,"fl_id: %x ",(DPR_DWORD)fast_logic_id);
		DpTrcGetErrVal   (MyTxt,error);
		
		X_Trc_write_txt    (DpTrcIndex, 
			LEVEL_DP_FL_ON_OFF,
			SELECT_DP_FL_ON_OFF,
			MyTxt);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DpTrcWatchdog                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Gets trace information from the DP_watchdog request.                     */
/*                                                                           */
/*****************************************************************************/

void  DpTrcWatchdog         (DPR_DWORD    user_handle,
							 DPR_DWORD    timeout,
							 DPR_WORD     *wd_index,
							 DP_ERROR_T*  error)
{
	
	DPR_STRING MyTxt[TRC_TEXT_BUF_SIZE];
	
	
	if (DpTrcIndex != 0)
	{
		strcpy(MyTxt,   "<>DP_watchdog: ");
		DpTrcGetErrVal   (MyTxt,error);
		DpTrcGetVal      (MyTxt,"usr_hdl: %x ",user_handle);
		DpTrcGetVal      (MyTxt,"timeout: %x ",timeout);
		if (wd_index)
		{
			DpTrcGetVal      (MyTxt,"idx: %x ",(DPR_DWORD) *wd_index); 
		}
		else
		{
			DpTrcSetString(MyTxt,"wd_index (Null-Pointer)");
		}  
		X_Trc_write_txt  (DpTrcIndex, 
			LEVEL_DP_READ_ALARM,
			SELECT_DP_READ_ALARM,
			MyTxt);
	}
}




#ifdef WIN32
#pragma warning( default : 4201 4214 4115 4100)
#endif

