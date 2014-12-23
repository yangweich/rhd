/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Version:    1.0                                                          */
/*  Modul:      dps_uif.c                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains the user interface functions of the DPS_BASE.DLL      */
/*                                                                           */
/*  // in  = Vorgabeparameter                                                */
/*  // out = Rueckgabeparameter                                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*  History of changes:                                                      */
/*  ===================                                                      */
/*                                                                           */
/*  Date     Comment                                                         */
/*----------!----------------------------------------------------------------*/
/*  28.10.98 first release                                                   */
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
#include "5614_ret.h"
#include "ci_5613.h"

#include "dps_base.h"
#include "fkt_def.h"

#ifndef __GNUC__
#pragma warning (disable:4100)
#endif

#define DPS_CFG_IS_BYTE_FORMAT          (DPR_BYTE)0x30
#define DPS_CFG_BF_LENGTH               (DPR_BYTE)0x0f
#define DPS_CFG_LENGTH_IS_WORD_FORMAT   (DPR_BYTE)0x40
#define DPS_CFG_BF_INP_EXIST            (DPR_BYTE)0x10
#define DPS_CFG_BF_OUTP_EXIST           (DPR_BYTE)0x20
#define DPS_CFG_SF_OUTP_EXIST           (DPR_BYTE)0x80
#define DPS_CFG_SF_INP_EXIST            (DPR_BYTE)0x40
#define DPS_CFG_SF_LENGTH               (DPR_BYTE)0x3f



// global definition of pointer to DP-RAM
DPR_CP5613_DP_T volatile *dpr = NULL;
DPR_BYTE            max_user_diag_len;


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_open()                                                   */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Mit dieser Funktion meldet sich eine DPS-Applikation beim Treiber an.    */
/*  Im Erfolgsfall gibt die Funktion ein User-Handle zurueck.                */
/*  Das User-Handle muss bei allen weiteren  Funktionsnsaufrufen verwendet   */
/*  werden.                                                                  */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  DP_CODE_ATTR DPS_open  (DPR_STRING       *cp_name,       /* in  */
								   DPR_DWORD       *user_handle,   /* out */
								   DPR_DWORD       slave_mode,     /* in */
								   DPR_WORD        station_addr,   /* in */
								   DPR_WORD        addr_change,    /* in */
								   DPR_WORD        pno_ident_nr,   /* in */
								   DPR_WORD        user_wd,        /* in */
								   DPS_INIT_DATA_T *init_data,     /* in */
								   DPS_MAX_DATA_T  *max_data_lens, /* in */
								   DPR_WORD        baud_rate,      /* in */
								   DP_ERROR_T      *error)         /* out */
{
	
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	DPR_DWORD       Result;
	DPR_DWORD       SyncUserHandle  = 0;
	DPR_DWORD       AsyncUserHandle = 0;
	CI_DPS_REQUEST_T CiRequest;
	CI_DPS_ORDER_T  *DpsOrderPtr;
	CI_DPS_OPEN_T   *OpenData;
	CI_DPS_SETUP_INDS_T *SetupIndsData;
	
#ifdef WIN32
	EnterCriticalSection(&DpsCritSec);
#endif
	// check, if slavemodule present
	
	if ( (SynCeckResult = dps_open_syntax(cp_name,user_handle,DpsUserCtr,
		slave_mode,station_addr,init_data,max_data_lens,error)) == DP_OK)
	{
		*user_handle          = 0; /* default value */
		
		/* get first user handle */
		CiResult = CI_open (&SyncUserHandle, cp_name);
		if(CiResult == CI_RET_OK)
		{
			/* get second user handle */
			CiResult = CI_open (&AsyncUserHandle, cp_name);
			if (CiResult != CI_RET_OK)
			{
				/* release first handle */
				(void)CI_close (SyncUserHandle);
			}
		}
		
		if (CiResult != CI_RET_OK)
		{
			error -> error_class  = DP_ERROR_CI;
			error -> error_code   = CiResult;
			error -> error_decode = 0;
			error -> error_code_1 = 0;
			error -> error_code_2 = 0;
		}
		else
		{
		/* registration at driver was successfull,
			now registration of the DP-User at the CP 5613 (here: *user_handle) */
			
			build_dps_header (&CiRequest,CI_DPS_OPEN,SyncUserHandle);
			
			/* DP-request block for DP_open */
			DpsOrderPtr = &CiRequest.order;
			
			OpenData = &DpsOrderPtr->OrderData.open;
			OpenData->slave_mode            = slave_mode;
			OpenData->station_addr          = station_addr;
			OpenData->addr_change           = addr_change;
			OpenData->pno_ident_nr          = pno_ident_nr;
			OpenData->user_wd               = user_wd;
			if((slave_mode & DPS_SM_SIMPLE) == DPS_SM_SIMPLE)
			{
				// simple slave
				OpenData->user_prm_data_len         = init_data->simple.user_prm_data_len;
				memcpy(OpenData->user_prm_data, init_data->simple.user_prm_data, init_data->simple.user_prm_data_len);
				OpenData->cfg_data_len          = init_data->simple.cfg_data_len;
				memcpy(OpenData->cfg_data, init_data->simple.cfg_data, init_data->simple.cfg_data_len);
			}
			else
			{
				// dynamischer slave
				OpenData->cfg_data_len          = init_data->dynamic.def_cfg_data_len;
				memcpy(OpenData->cfg_data, init_data->dynamic.def_cfg_data, init_data->dynamic.def_cfg_data_len);
			}
			OpenData->max_input_data_len    = max_data_lens->max_input_data_len;
			OpenData->max_output_data_len   = max_data_lens->max_output_data_len;
			OpenData->max_usr_diag_data_len = max_data_lens->max_user_diag_len;
			max_user_diag_len               = (DPR_BYTE)max_data_lens->max_user_diag_len;
			OpenData->max_usr_prm_data_len  = max_data_lens->max_user_prm_data_len;
			OpenData->max_cfg_data_len      = max_data_lens->max_cfg_data_len;
			if(addr_change)
			{
				OpenData->max_usr_ssa_data_len  = max_data_lens->max_user_ssa_data_len;
			}
			else
			{
				OpenData->max_usr_ssa_data_len  = 0;
			}
			
			/* send DP request and receive confirmation */
			
			CiResult = dps_send_and_receive (&CiRequest);
			
			dps_build_result (CiResult, &CiRequest, error);
			
			if(error -> error_class == DP_OK)
			{
				
				/* store user handles in the internal list of the DLL */
				if ((Result = dps_store_user(SyncUserHandle,AsyncUserHandle,
					user_handle)) != DP_OK)
				{
					error -> error_class  = DP_ERROR_RES;
					error -> error_code   = Result;
					error -> error_decode = 0;
					error -> error_code_1 = 0;
					error -> error_code_2 = 0;
					
					/* DP-Close */
					
					(void)DPS_close (SyncUserHandle,error);
					(void)DPS_close (AsyncUserHandle,error);
				}
				
				// DPR-pointer holen
				CiResult = CI_get_pointer(AsyncUserHandle, &dpr);
				if (CiResult != CI_RET_OK)
				{
					error -> error_class  = DP_ERROR_CI;
					error -> error_code   = CiResult;
					error -> error_decode = 0;
					error -> error_code_1 = 0;
					error -> error_code_2 = 0;
				}
				
				// alle indications freigeben
				build_dps_header (&CiRequest,CI_DPS_SETUP_INDS,SyncUserHandle);
				
				DpsOrderPtr = &CiRequest.order;
				SetupIndsData = &DpsOrderPtr->OrderData.setup_inds;
				
				SetupIndsData->indications = 0xffffffffu;
				
				/* send DP request (receive confirmation with dp_get_result) */
				
				CiResult = dps_send_and_receive (&CiRequest);
				
				dps_build_result (CiResult, &CiRequest, error);
				if(error -> error_class == DP_OK)
				{
					// asynchronen Block runterstellen
					build_dps_header (&CiRequest,CI_DPS_RCV_IND,AsyncUserHandle);
					CiResult = dps_send (&CiRequest);
					dps_build_result_async (CiResult, &CiRequest, error);
					if (error -> error_class != DP_OK_ASYNC)
					{
						error -> error_class  = DP_ERROR_CI;
						error -> error_code   = CiResult;
						error -> error_decode = 0;
						error -> error_code_1 = 0;
						error -> error_code_2 = 0;
					}
					else
					{
						error -> error_class  = DP_OK;
					}
				}
			}
			else
			{
				/* close connection to the driver */
				CiResult = CI_close (SyncUserHandle);
				CiResult = CI_close (AsyncUserHandle);
			}
		}
	}
	*user_handle |= DPS_USR_HDL_ID;
	
#ifdef WIN32
	DpsTrcOpen (cp_name,*user_handle,slave_mode,station_addr,addr_change,
		pno_ident_nr,user_wd,init_data,max_data_lens,baud_rate,
		error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DpsCritSec);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}

/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_close()                                                  */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Mit dieser Funktion meldet sich eine DP-Applikation wieder beim          */
/*  Treiber ab.                                                              */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

/* removing an application off the DP interface */

DPR_DWORD DP_CODE_ATTR DPS_close ( DPR_DWORD    user_handle,  /* in   */
								  DP_ERROR_T   *error )      /* out  */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DPS_REQUEST_T MyCiRequest;
	DPR_DWORD       orig_user_handle;
	
#ifdef WIN32
	EnterCriticalSection(&DpsCritSec);
#endif
	
	orig_user_handle = user_handle;
	user_handle &= ~DPS_USR_HDL_ID;
	
	if ( (SynCeckResult = dps_close_syntax(error,user_handle)) == DP_OK)
	{
		build_dps_header (&MyCiRequest,CI_DPS_CLOSE,
			DpsUserIdTable[user_handle].DpSyncHandle);
		
		/* send DP request and receive confirmation */
		
		CiResult = dps_send_and_receive (&MyCiRequest);
		
		dps_build_result (CiResult, &MyCiRequest, error);
		
		/* release Dp User index  */
		if(error -> error_class == DP_OK)
		{
			
			/* close connection to the driver */
			CiResult = CI_close (DpsUserIdTable[user_handle].DpSyncHandle);
			
			if (CiResult != CI_RET_OK)
			{
				error -> error_class  = DP_ERROR_CI;
				error -> error_code   = CiResult;
				error -> error_decode = 0;
				error -> error_code_1 = 0;
				error -> error_code_2 = 0;
			}
			else
			{
				CiResult = CI_close (DpsUserIdTable[user_handle].DpAsyncHandle);
				if (CiResult != CI_RET_OK)
				{
					error -> error_class  = DP_ERROR_CI;
					error -> error_code   = CiResult;
					error -> error_decode = 0;
					error -> error_code_1 = 0;
					error -> error_code_2 = 0;
				}
				
				/* release user handle in internal array of the DLL */
				(void)dps_release_user(user_handle);
				// reset DP-RAM pointer
				dpr = NULL;
			}
		}
		
	}
	
#ifdef WIN32
	DpsTrcClose (orig_user_handle,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DpsCritSec);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
	
}



/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_start                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Mit dieser Funktion wird der Slave online geschaltet.                    */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/



DPR_DWORD DP_CODE_ATTR DPS_start (DPR_DWORD  user_handle,  /* in  */
								  DP_ERROR_T *error)       /* out */
{
	DPR_DWORD           CiResult;
	DPR_DWORD           SynCeckResult;
	CI_DPS_REQUEST_T    CiRequest;
	DPR_DWORD       orig_user_handle;
	
#ifdef WIN32
	EnterCriticalSection(&DpsCritSec);
#endif
	
	/* check syntax */
	
	orig_user_handle = user_handle;
	user_handle &= ~DPS_USR_HDL_ID;
	
	if ( (SynCeckResult = dps_start_syntax(user_handle,error)) == DP_OK)
	{
		build_dps_header (&CiRequest,CI_DPS_START,
			DpsUserIdTable[user_handle].DpSyncHandle);
		
		/* send DP request and receive confirmation */
		
		CiResult = dps_send_and_receive (&CiRequest);
		
		dps_build_result (CiResult, &CiRequest, error);
		
	}
	
	
#ifdef WIN32
	DpsTrcStart (orig_user_handle,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DpsCritSec);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_stop                                                     */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Mit dieser Funktion wird der Slave offline geschaltet.                   */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DP_CODE_ATTR DPS_stop (DPR_DWORD  user_handle,  /* in  */
								 DP_ERROR_T *error)       /* out */
{
	DPR_DWORD           CiResult;
	DPR_DWORD           SynCeckResult;
	CI_DPS_REQUEST_T    CiRequest;
	DPR_DWORD       orig_user_handle;
	
#ifdef WIN32
	EnterCriticalSection(&DpsCritSec);
#endif
	
	/* check syntax */
	
	orig_user_handle = user_handle;
	user_handle &= ~DPS_USR_HDL_ID;
	if ( (SynCeckResult = dps_stop_syntax(user_handle,error)) == DP_OK)
	{
		build_dps_header (&CiRequest,CI_DPS_STOP,
			DpsUserIdTable[user_handle].DpSyncHandle);
		
		/* send DP request and receive confirmation */
		
		CiResult = dps_send_and_receive (&CiRequest);
		
		dps_build_result (CiResult, &CiRequest, error);
		
	}
	
#ifdef WIN32
	DpsTrcStop (orig_user_handle,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DpsCritSec);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}




/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_get_baud_rate                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion ermittelt die aktuelle Baudrate des Slaves                */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DP_CODE_ATTR DPS_get_baud_rate(DPR_DWORD  user_handle,  /* in  */
										 DPR_WORD   *state,       /* out  */
										 DPR_WORD   *baud_rate,   /* out  */
										 DP_ERROR_T *error)       /* out */
{
	DPR_DWORD           SynCeckResult;
	DPR_DWORD       orig_user_handle;
	
#ifdef WIN32
	EnterCriticalSection(&DpsCritSec);
#endif
	
	/* check syntax */
	
	orig_user_handle = user_handle;
	user_handle &= ~DPS_USR_HDL_ID;
	if ( (SynCeckResult = dps_get_baud_rate_syntax(user_handle,state,baud_rate,error)) == DP_OK)
	{
		if(dpr)
		{
			/* copy result */
			*state = dpr->info_watch.slavemod_data.baud_state;
			*baud_rate = dpr->info_watch.slavemod_data.baud_rate;
		}
		else
		{
			error -> error_class  = DP_ERROR_RES;
			error -> error_code   = DPS_RET_NO_DPR_PTR;
		}
		
	}
	
#ifdef WIN32
	DpsTrcGetBaudRate (orig_user_handle,*state,*baud_rate,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DpsCritSec);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_get_gc_command                                           */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion ermittelt das zuletzt empfangene GC-Kommando              */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DP_CODE_ATTR DPS_get_gc_command(DPR_DWORD  user_handle,  /* in  */
										  DPR_WORD   *gc,       /* out  */
										  DP_ERROR_T *error)       /* out */
{
	DPR_DWORD           SynCeckResult;
	DPR_DWORD       orig_user_handle;
	
#ifdef WIN32
	EnterCriticalSection(&DpsCritSec);
#endif
	
	/* check syntax */
	
	orig_user_handle = user_handle;
	user_handle &= ~DPS_USR_HDL_ID;
	if ( (SynCeckResult = dps_get_gc_command_syntax(user_handle,gc,error)) == DP_OK)
	{
		if (dpr)
		{
			
			/* copy result */
			*gc = dpr->info_watch.slavemod_data.act_gc;
		}
		else
		{
			error -> error_class  = DP_ERROR_RES;
			error -> error_code   = DPS_RET_NO_DPR_PTR;
		}
		
	}
	
#ifdef WIN32
	DpsTrcGetGcCommand (orig_user_handle,*gc,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DpsCritSec);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}



/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_get_state                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion ermittelt den Zustand der DPS State-machine               */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DP_CODE_ATTR DPS_get_state(DPR_DWORD  user_handle,  /* in  */
									 DPR_WORD   *state,       /* out  */
									 DP_ERROR_T *error)       /* out */
{
	DPR_DWORD           CiResult;
	DPR_DWORD           SynCeckResult;
	CI_DPS_REQUEST_T    CiRequest;
	CI_DPS_ORDER_T*     DpsOrderPtr;
	CI_DPS_GET_STATE_T* GetStateData;
	DPR_DWORD       orig_user_handle;
	
#ifdef WIN32
	EnterCriticalSection(&DpsCritSec);
#endif
	
	/* check syntax */
	
	orig_user_handle = user_handle;
	user_handle &= ~DPS_USR_HDL_ID;
	if ( (SynCeckResult = dps_get_state_syntax(user_handle,state,error)) == DP_OK)
	{
		build_dps_header (&CiRequest,CI_DPS_GET_STATE,
			DpsUserIdTable[user_handle].DpSyncHandle);
		
		DpsOrderPtr = &CiRequest.order;
		
		/* send DP request and receive confirmation */
		
		CiResult = dps_send_and_receive (&CiRequest);
		
		dps_build_result (CiResult, &CiRequest, error);
		
		/* copy result */
		GetStateData = &DpsOrderPtr->OrderData.get_state;
		*state = GetStateData->state;
	}
	
#ifdef WIN32
	DpsTrcGetState (orig_user_handle,*state,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DpsCritSec);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}

/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_set_diag                                                 */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Mit dieser Funktion werden neue diagnosedaten fr das Slavemodul         */
/*  bergeben.                                                               */
/*  Die bertragung erfolgt nicht ber den Kommandokanal, sondern direkt     */
/*  ber das DP-Ram                                                                         */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DP_CODE_ATTR DPS_set_diag (DPR_DWORD  user_handle,    /* in  */
									 DPR_BYTE   *user_diag_data, /* in */
									 DPR_WORD   user_diag_len,   /* in  */
									 DPR_WORD   diag_state,      /* in  */
									 DP_ERROR_T *error)          /* out */
{
	DPR_DWORD           SynCeckResult;
	DPR_DWORD       orig_user_handle;
	
#ifdef WIN32
	EnterCriticalSection(&DpsCritSec);
#endif
	
	/* check syntax */
	
	orig_user_handle = user_handle;
	user_handle &= ~DPS_USR_HDL_ID;
	if ( (SynCeckResult = dps_set_diag_syntax(user_handle,user_diag_data,
		user_diag_len, diag_state, error)) == DP_OK)
	{
		if(dpr)
		{
			memcpy((void *)dpr->info_watch.slavemod_data.sm_diag.diag_data, user_diag_data, user_diag_len);
			dpr->info_watch.slavemod_data.sm_diag.len = user_diag_len;
			dpr->info_watch.slavemod_data.sm_diag.state = diag_state;
			dpr->info_watch.slavemod_data.sm_diag.request ++;
		}
		else
		{
			error -> error_class  = DP_ERROR_RES;
			error -> error_code   = DPS_RET_NO_DPR_PTR;
		}
		
	}
	
#ifdef WIN32
	DpsTrcSetDiag (orig_user_handle,user_diag_data,user_diag_len,diag_state,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DpsCritSec);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}

#if 0
/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_trigger_user_wd                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion triggert den User-Watchdog                                */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DP_CODE_ATTR DPS_trigger_user_wd(DPR_DWORD  user_handle,  /* in  */
										   DP_ERROR_T *error)       /* out */
{
	DPR_DWORD           CiResult;
	DPR_DWORD           SynCeckResult;
	CI_DPS_REQUEST_T    CiRequest;
	CI_DPS_ORDER_T*     DpsOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DpsCritSec);
#endif
	
	/* check syntax */
	
	user_handle &= ~DPS_USR_HDL_ID;
	if ( (SynCeckResult = dps_trigger_user_wd_syntax(user_handle,error)) == DP_OK)
	{
		build_dps_header (&CiRequest,CI_DPS_TRIGGER_USER_WD,
			DpUserIdTable[user_handle].DpSyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		DpsOrderPtr = (CI_DPS_ORDER_T*)CiRequest.data;
		
		DpsOrderPtr -> DatLen   = 0;
		
		/* send DP request and receive confirmation */
		
		CiResult = dps_send_and_receive (&CiRequest);
		
		dps_build_result (CiResult, &CiRequest, error);
	}
	
#ifdef WIN32
	LeaveCriticalSection(&DpsCritSec);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}
#endif

/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_calc_io_data_len                                         */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion berechnet die E/A-Datenl„nge des bergebenen config-      */
/*  telegramms.                                                              */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

#ifndef __GNUC__
#pragma warning (disable:4244)
#endif

DPR_DWORD DP_CODE_ATTR DPS_calc_io_data_len(
											DPR_WORD   cfg_len,         /* in  */
											DPR_BYTE   *cfg_data,       /* in  */
											DPR_WORD   *in_data_len,    /* out  */
											DPR_WORD   *out_data_len,   /* out  */
											DP_ERROR_T *error)          /* out */
{
	DPR_DWORD           SynCeckResult;
	DPR_WORD            length;
	DPR_WORD            count;
	DPR_WORD            specific_data_length;
	DPR_WORD            result_ok;
	
	/* check syntax */
	
	if ( (SynCeckResult = dps_calc_io_data_len_syntax(cfg_len, cfg_data, in_data_len, out_data_len, error)) == DP_OK)
	{
		
		result_ok = 1;
		
		*in_data_len  = 0;
		*out_data_len = 0;
		
		for ( ; (cfg_len > 0) && result_ok; cfg_len -= count)
		{
			count = 0;
			
			if (*cfg_data & DPS_CFG_IS_BYTE_FORMAT)
			{
				count++;
				/* cfg_data points to "Kennungsbyte", CFG_BF means "CFG_IS_BYTE_FORMA
				T" */
				length =  (*cfg_data & DPS_CFG_BF_LENGTH) + 1;
				
				if (*cfg_data & DPS_CFG_LENGTH_IS_WORD_FORMAT)
				{
					length *= 2;
				}
				if (*cfg_data & DPS_CFG_BF_OUTP_EXIST)
				{
					*out_data_len += length;
				}
				if (*cfg_data & DPS_CFG_BF_INP_EXIST)
				{
					*in_data_len += length;
				}
				cfg_data++;
			}
			else
			{
			/* cfg_data points to the headerbyte of "spezielles Kennungsformat"
				*/
				/* CFG_SF means "CFG_IS_SPECIAL_FORMAT" */
				if (*cfg_data & DPS_CFG_SF_OUTP_EXIST)
				{
				count++;                /* next byte contains the length of outp_
				data */
				length = (*(cfg_data + count) & DPS_CFG_SF_LENGTH) +1;
				
				if (*(cfg_data + count) & DPS_CFG_LENGTH_IS_WORD_FORMAT)
				{
					*out_data_len += (DPR_WORD)(2*length);
				}
				else
				{
					*out_data_len += length;
				}
				}
				if (*cfg_data & DPS_CFG_SF_INP_EXIST)
				{
				count++;                /* next byte contains the length of inp_d
				ata */
				length = (*(cfg_data + count) & DPS_CFG_SF_LENGTH) +1;
				
				if (*(cfg_data + count) & DPS_CFG_LENGTH_IS_WORD_FORMAT)
				{
					*in_data_len += (DPR_WORD)(2 * length);
				}
				else
				{
					*in_data_len += length;
				}
				}
				specific_data_length = (*cfg_data & DPS_CFG_BF_LENGTH);
				
				if (specific_data_length != 15)
				{
					count += 1 + specific_data_length;
					cfg_data = cfg_data + count;
				}
				else
				{
					result_ok = 0;
				}
			}
		}
		if ( result_ok && (*in_data_len <= 244) && (*out_data_len <= 244) )
		{
			result_ok = 1;
		}
		else
		{
			result_ok = 0;
		}
		
		if (!result_ok)
		{
			*in_data_len = *out_data_len = 0;
		}
	}
	
#ifdef WIN32
	DpsTrcCalcIoDataLen (cfg_len,cfg_data,*in_data_len,*out_data_len,error);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}

#ifndef __GNUC__
#pragma warning (default:4244)
#endif

/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_get_ind()                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Diese Funktion meldet asynchrone ereignisse.                             */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DP_CODE_ATTR DPS_get_ind (DPR_DWORD    user_handle,   /* in  */
									DPR_DWORD   *ind_ref,       /* out  */
									DPR_DWORD   timeout,        /* in  */
									DPR_DWORD   *indication,    /* inout  */
									DPR_WORD   *data_len,       /* inout  */
									DPR_BYTE   *data_blk,       /* out */
									DP_ERROR_T   *error)        /* out */
{
	DPR_DWORD           CiResult;
	DPR_DWORD           SynCeckResult;
	CI_DPS_REQUEST_T    CiRequest;
	CI_DPS_ORDER_T*     DpsOrderPtr;
	CI_DPS_SETUP_INDS_T *SetupIndsData;
	CI_DPS_RCV_IND_T    *RcvIndData;
	DPR_DWORD       orig_user_handle;
	
	
	/* check syntax */
	
	orig_user_handle = user_handle;
	user_handle &= ~DPS_USR_HDL_ID;
	if ( (SynCeckResult = dps_get_ind_syntax(user_handle,ind_ref,timeout,
		indication, data_len, data_blk, error)) == DP_OK)
	{
		// angegebene indications freigeben
		build_dps_header (&CiRequest,CI_DPS_SETUP_INDS,
			DpsUserIdTable[user_handle].DpSyncHandle);
		
		DpsOrderPtr = &CiRequest.order;
		SetupIndsData = &DpsOrderPtr->OrderData.setup_inds;
		
		SetupIndsData->indications = *indication;
		
		/* send DP request (receive confirmation with dp_get_result) */
		
#ifdef WIN32
		EnterCriticalSection(&DpsCritSec);
#endif
		
		CiResult = dps_send_and_receive (&CiRequest);
		
#ifdef WIN32
		LeaveCriticalSection(&DpsCritSec);
#endif
		
		dps_build_result (CiResult, &CiRequest, error);
		
#ifdef WIN32
		DpsTrcGetIndBegin (orig_user_handle,timeout,*indication,*data_len,error);
#endif
		
		if(error -> error_class == DP_OK)
		{
			// auf receive warten mit angegeben timeout
			CiResult = dps_receive(&CiRequest, DpsUserIdTable[user_handle].DpAsyncHandle,
				timeout);
			if((CiResult == CI_RET_OK) ||
				(CiResult == CI_RET_RECEIVE_TIMEOUT_NO_DATA))
			{
				// timeout oder keine Daten, leer zurckkehren
				*ind_ref    = 0;
				*indication = 0;
				*data_len   = 0;
			}
			else
			{
				// sonst Daten an user schicken
				dps_build_result (CiResult, &CiRequest, error);
				if(error -> error_class == DP_OK)
				{
					DpsOrderPtr = &CiRequest.order;
					RcvIndData = &DpsOrderPtr->OrderData.rcv_ind;
					*ind_ref    = RcvIndData->ind_ref;
					*indication = RcvIndData->indication;
					*data_len   = RcvIndData->data_len;
					memcpy(data_blk, RcvIndData->data_blk, RcvIndData->data_len);
				}
				
				// neuen receive-block runterstellen
				build_dps_header (&CiRequest,CI_DPS_RCV_IND,
					DpsUserIdTable[user_handle].DpAsyncHandle);
				CiResult = dps_send (&CiRequest);
				dps_build_result_async (CiResult, &CiRequest, error);
				if (error -> error_class == DP_OK_ASYNC)
				{
					error -> error_class = DP_OK;
				}
			}
		}
	}
#ifdef WIN32
	DpsTrcGetIndEnd (orig_user_handle,*ind_ref,*indication,*data_len,data_blk,error);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
									/*  Function:   DPS_set_resp                                                 */
									/*---------------------------------------------------------------------------*/
									/*  Comment:                                                                 */
									/*  Mit dieser Funktion stellt die HostSW die tats„chliche Konfiguration ein */
									/*                                                                           */
									/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DPS_set_resp (DPR_DWORD  user_handle,  /* in  */
									 DPR_DWORD   ind_ref,    /* in  */
									 DPR_WORD   data_len,    /* in  */
									 DPR_BYTE   *data_blk,   /* in */
									 DP_ERROR_T *error)      /* out */
{
	DPR_DWORD           CiResult;
	DPR_DWORD           SynCeckResult;
	CI_DPS_REQUEST_T    CiRequest;
	CI_DPS_ORDER_T*     DpsOrderPtr;
	CI_DPS_SET_RESP_T   *SetRespData;
	DPR_DWORD       orig_user_handle;
	
	
#ifdef WIN32
	EnterCriticalSection(&DpsCritSec);
#endif
	
	/* check syntax */
	
	orig_user_handle = user_handle;
	user_handle &= ~DPS_USR_HDL_ID;
	if ( (SynCeckResult = dps_set_resp_syntax(user_handle,ind_ref,
		data_len, data_blk, error)) == DP_OK)
	{
		build_dps_header (&CiRequest,CI_DPS_SET_RESP,
			DpsUserIdTable[user_handle].DpSyncHandle);
		
		DpsOrderPtr = &CiRequest.order;
		SetRespData = &DpsOrderPtr->OrderData.set_resp;
		SetRespData->ind_ref = ind_ref;
		SetRespData->resp = *data_blk;
		
		/* send DP request and receive confirmation */
		
		CiResult = dps_send_and_receive (&CiRequest);
		
		dps_build_result (CiResult, &CiRequest, error);
		
	}
	
#ifdef WIN32
	DpsTrcSetResp (orig_user_handle,ind_ref,data_len,data_blk,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DpsCritSec);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   DPS_get_ci_hdl                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Mit dieser Funktion ermittelt DP_BASE die user-handles von DPS_BASE      */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DPS_get_ci_hdl(DPR_DWORD user_handle,
						 DPR_DWORD *sync_ci_hdl,
						 DPR_DWORD *async_ci_hdl,
						 DP_ERROR_T *error)
{
	DPR_DWORD           SynCeckResult;
	//    DPR_DWORD       orig_user_handle;
	
	//    orig_user_handle = user_handle;
	user_handle &= ~DPS_USR_HDL_ID;
	if ( (SynCeckResult = dps_get_ci_hdl_syntax(user_handle,sync_ci_hdl,
		async_ci_hdl, error)) == DP_OK)
	{
		*sync_ci_hdl = DpsUserIdTable[user_handle].DpSyncHandle;
		*async_ci_hdl = DpsUserIdTable[user_handle].DpAsyncHandle;
	}
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}

