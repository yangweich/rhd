/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Module:     dp_uif.c                                                     */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains the synchronous user interface functions of the       */
/*  DP_base.dll                                                              */
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
#endif

#include <string.h>

#include "dp_5613.h"
#include "5613_ret.h"
#include "ci_5613.h"

#include "dp_base.h"
#include "fkt_def.h"


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_start_cp()                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function downloads the firmware and the database to the             */
/*  CP 5613\5614.                                                            */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_start_cp  (DPR_STRING DP_MEM_ATTR *cp_name,  /* in  */
                                     DPR_STRING DP_MEM_ATTR *database, /* in  */
                                     DP_ERROR_T DP_MEM_ATTR *error)    /* out */
{
	
	DPR_DWORD   SynCeckResult;
	DPR_DWORD   CiResult;
	DPR_STRING* FwPathPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	if ( (SynCeckResult = dp_start_cp_syntax(cp_name,error)) == DP_OK)
	{
		
		get_fw_path(&FwPathPtr);
		
		CiResult = CI_start_cp (cp_name,
			FwPathPtr,     /* 0 = default path */
			database,	     /* database         */
			STATUS_INIT);
		
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
			error -> error_class  = DP_OK; 
			error -> error_code   = 0;
			error -> error_decode = 0;
			error -> error_code_1 = 0;
			error -> error_code_2 = 0;
		}
	}
	
#ifdef WIN32
	DpTrcStartCp (cp_name,database,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_reset_cp()                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function resets the CP 5613\5614.                                   */
/*  Following this, the CP is no longer active on the bus 					 */
/*  (token LED is off).                                                      */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/




DPR_DWORD  DP_CODE_ATTR  DP_reset_cp  (DPR_STRING DP_MEM_ATTR *cp_name,    /* in  */
                                       DP_ERROR_T DP_MEM_ATTR *error)      /* out */
{
	DPR_DWORD SynCeckResult;
	DPR_DWORD CiResult;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	if ( (SynCeckResult = dp_reset_cp_syntax(cp_name,error)) == DP_OK)
	{
		CiResult =  CI_reset_cp (cp_name, RESET_MODE_SAFE);
		
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
			error -> error_class  = DP_OK; 
			error -> error_code   = 0;
			error -> error_decode = 0;
			error -> error_code_1 = 0;
			error -> error_code_2 = 0;
		}
	}
#ifdef WIN32
	DpTrcResetCp (cp_name,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_open()                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Using this function, the DP application registers with the driver.       */
/*  If successful, the function returns a user handle.                       */
/*  The user handle must be used in all subsequent function calls            */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  DP_CODE_ATTR DP_open (DPR_STRING DP_MEM_ATTR *cp_name,      /* in  */
                                 DPR_DWORD  DP_MEM_ATTR *user_handle,  /* out */
                                 DP_ERROR_T DP_MEM_ATTR *error)        /* out */
{
	return DP_open_int(cp_name,user_handle,error,0);
}


DPR_DWORD  DP_open_int (DPR_STRING  *cp_name,      /* in  */
                        DPR_DWORD   *user_handle,  /* out */
                        DP_ERROR_T  *error,        /* out */
                        DPR_WORD    Mode)          /* in  */                               
{  
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	DPR_DWORD       Result;
	DPR_DWORD       SyncUserHandle  = 0;
	DPR_DWORD       AsyncUserHandle = 0;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	if ( (SynCeckResult = dp_open_syntax(cp_name,user_handle,DpUserCtr,error)) == DP_OK)
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
			
			build_dp_header (&MyCiRequest,CI_DP_USR_OPEN,SyncUserHandle,0);
			
			/* DP-request block for DP_open */
			MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
			
			MyDpOrderPtr -> UsrId         = *user_handle;
			MyDpOrderPtr -> DatLen        = 0;
			MyDpOrderPtr -> DatTyp        = Mode; /* 0 = dp_base-mode,0xffff=Converter */
			
			
			/* send DP request and receive confirmation */
			
			CiResult = dp_send_and_receive (&MyCiRequest);
			
			build_result (CiResult, &MyCiRequest, error);
			
			if(error -> error_class == DP_OK)
			{
				
				/* store user handles in the internal list of the DLL */
				if ((Result = store_user(SyncUserHandle,AsyncUserHandle,
					MyDpOrderPtr -> UsrIndex,user_handle)) != DP_OK)
				{
					error -> error_class  = DP_ERROR_RES; 
					error -> error_code   = Result;
					error -> error_decode = 0;
					error -> error_code_1 = 0;
					error -> error_code_2 = 0;
					
					/* DP-Close */
					
					(void)DP_close (SyncUserHandle,error);
					(void)DP_close (AsyncUserHandle,error);
				}
			}
			
		}
	}
#ifdef WIN32
	DpTrcOpen (cp_name,user_handle,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
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

DPR_DWORD DP_CODE_ATTR DP_get_pointer (DPR_DWORD user_handle,                        /* in  */
                                       DPR_DWORD timeout,                            /* in  */ 
                                       DPR_CP5613_DP_T volatile  DP_MEM_ATTR  **dpr, /* out */
                                       DP_ERROR_T  DP_MEM_ATTR    *error)            /* out */
{
	return DP_get_pointer_int(user_handle,
		timeout,
		dpr,
		error,
		0);
}

DPR_DWORD DP_get_pointer_int (DPR_DWORD user_handle,             /* in  */
                              DPR_DWORD timeout,                 /* in  */ 
                              DPR_CP5613_DP_T volatile  **dpr,   /* out */
                              DP_ERROR_T  *error,                /* out */
                              DPR_WORD  Mode)                    /* in  */ 
							  
{
	
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	DPR_DWORD       MyTimeout;
	
	DPR_DWORD       SlvResult;
	DPR_DWORD       SlvSnycCiHdl;
	DPR_DWORD       SlvAsnycCiHdl;
	
	SlvResult     = DP_OK;
	SynCeckResult = DP_OK;
	CiResult      = DP_OK;
	SlvSnycCiHdl  = DP_OK;
	
	if (user_handle & 0x80000000)
	{
		/* get slvae ci_handles */
		SlvResult = DPS_get_ci_hdl(user_handle, 
			&SlvSnycCiHdl, 
			&SlvAsnycCiHdl, 
			error);
	}
	
	if ((SlvResult == DP_OK) &&
		((SynCeckResult = dp_get_ptr_syntax(user_handle,dpr,error)) == DP_OK) )
	{
		*dpr = 0; /* default value */
		
		/* set timeout limit */
		if (timeout > DP_TIMEOUT_FOREVER)
		{
			MyTimeout = DP_TIMEOUT_FOREVER;
		}
		else
		{
			MyTimeout = timeout;
		}
		
		if (user_handle & 0x80000000)
		{
			/* Slave */
			/* get access to dpr */
			CiResult = CI_get_dp_access (SlvSnycCiHdl, MyTimeout);
			if(CiResult == CI_RET_OK)
			{
				/* get second user handle */
				CiResult = CI_get_pointer (SlvSnycCiHdl, dpr);
			}
		}
		else
		{
			/* Master */
			/* get access to dpr */
			if (Mode == 0) /* DP_base-Master */
			{
				CiResult = CI_get_dp_access (DpUserIdTable[user_handle].DpSyncHandle, MyTimeout);
			}
			else /* DPLIB-Converter */
			{
				CiResult = CI_RET_OK;
			}
			
			if(CiResult == CI_RET_OK)
			{
				/* get second user handle */
				CiResult = CI_get_pointer (DpUserIdTable[user_handle].DpSyncHandle, dpr);
			}
		}
		
		if (CiResult != CI_RET_OK)
		{
			if (CiResult == CI_RET_RECEIVE_TIMEOUT_NO_DATA)
			{
				error -> error_class  = DP_ERROR_EVENT_NET; 
				error -> error_code   = DP_RET_TIMEOUT;
			}
			else
			{
				error -> error_class  = DP_ERROR_CI; 
				error -> error_code   = CiResult;
			}
			
			error -> error_decode = 0;
			error -> error_code_1 = 0;
			error -> error_code_2 = 0;
		}
		else
		{
			error -> error_class  = DP_OK; 
			error -> error_code   = 0;
			error -> error_decode = 0;
			error -> error_code_1 = 0;
			error -> error_code_2 = 0;
		}
	}
	
#ifdef WIN32
	if(Mode ==0)
	{
		DpTrcGetPointer (user_handle,timeout,dpr,error);
	}
	
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
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

DPR_DWORD DP_CODE_ATTR DP_release_pointer (DPR_DWORD user_handle,           /* in  */
                                           DP_ERROR_T DP_MEM_ATTR *error)   /* out */
{
	return DP_release_pointer_int(user_handle,error,0);
}



DPR_DWORD DP_release_pointer_int (DPR_DWORD  user_handle,  /* in  */
                                  DP_ERROR_T *error,       /* out */
                                  DPR_WORD   Mode)         /* in  */
{
	
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	DPR_DWORD       SlvResult;
	DPR_DWORD       SlvSnycCiHdl;
	DPR_DWORD       SlvAsnycCiHdl;
	
	SlvResult     = DP_OK;
	SynCeckResult = DP_OK;
	SlvSnycCiHdl  = DP_OK;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	if (user_handle & 0x80000000)
	{
		/* get slvae ci_handles */
		SlvResult = DPS_get_ci_hdl(user_handle, 
			&SlvSnycCiHdl, 
			&SlvAsnycCiHdl, 
			error);
	}
	
	if ((SlvResult == DP_OK) &&
		((SynCeckResult = dp_rel_ptr_syntax(user_handle,error)) == DP_OK) )
	{
		if (user_handle & 0x80000000)
		{
			/* Slave */
			/* release access to dpr */
			CiResult = CI_release_dp_access (SlvSnycCiHdl);
		}
		else
		{
			/* Master */
			/* release access to dpr */
			if(Mode == 0)
			{
				CiResult = CI_release_dp_access (DpUserIdTable[user_handle].DpSyncHandle);
			}
			else
			{
				CiResult = CI_RET_OK;
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
			error -> error_class  = DP_OK; 
			error -> error_code   = 0;
			error -> error_decode = 0;
			error -> error_code_1 = 0;
			error -> error_code_2 = 0;
		}
	}
#ifdef WIN32
	if(Mode == 0)
	{
		DpTrcRelPointer (user_handle,error);
	}
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_set_mode()                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function sets the desired DP mode                                   */
/*  (offline/stop/clear/operate).                                            */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_set_mode (DPR_DWORD  user_handle,        /* in  */
                                    DPR_WORD  mst_mode,            /* in  */
                                    DP_ERROR_T DP_MEM_ATTR *error) /* out */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	/* check syntax */
	
	if ( (SynCeckResult = dp_set_mode_syntax(mst_mode,user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_SET_MODE,
			DpUserIdTable[user_handle].DpSyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		
		MyDpOrderPtr -> MstMode  = mst_mode;
		MyDpOrderPtr -> DatLen   = 0;
		
		/* send DP request and receive confirmation */
		
		CiResult = dp_send_and_receive (&MyCiRequest);
		
		build_result (CiResult, &MyCiRequest, error);
		
	}
#ifdef WIN32
	DpTrcSetMode (user_handle,mst_mode,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_close()                                                   */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  With this function, a DP application logs off again at the driver.       */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_close ( DPR_DWORD    user_handle,        /* in   */
								 DP_ERROR_T DP_MEM_ATTR  *error ) /* out  */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	if ( (SynCeckResult = dp_close_syntax(error,user_handle)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_USR_CLOSE,
			DpUserIdTable[user_handle].DpSyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		
		MyDpOrderPtr -> DatLen   = 0;
		
		/* send DP request and receive confirmation */
		
		CiResult = dp_send_and_receive (&MyCiRequest);
		
		build_result (CiResult, &MyCiRequest, error);
		
		/* release Dp User index  */
		if(error -> error_class == DP_OK)
		{
			
			/* close connection to the driver */
			CiResult = CI_close (DpUserIdTable[user_handle].DpSyncHandle);
			
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
				CiResult = CI_close (DpUserIdTable[user_handle].DpAsyncHandle);
				if (CiResult != CI_RET_OK)
				{
					error -> error_class  = DP_ERROR_CI; 
					error -> error_code   = CiResult;
					error -> error_decode = 0;
					error -> error_code_1 = 0;
					error -> error_code_2 = 0;
				}
				
				/* release user handle in internal array of the DLL */
				(void)release_user(user_handle);
			}
		}
		
	}
#ifdef WIN32
	DpTrcClose (user_handle,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_slv_state()                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function can be used to change the state of a DP slave while        */
/*  the DP application is running. The slave can be taken out of             */
/*  processing or activated again.                                           */
/*  Autoclear properties can also be changed.                                */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_slv_state (DPR_DWORD  user_handle,        /* in  */
                                     DPR_WORD   slv_add,            /* in  */
                                     DPR_WORD   slv_mode,           /* in  */
                                     DP_ERROR_T DP_MEM_ATTR *error) /* out */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	/* check syntax */
	
	if ( (SynCeckResult = dp_slv_state_syntax(slv_add, slv_mode,user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_SLV_STATE,
			DpUserIdTable[user_handle].DpSyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		
		MyDpOrderPtr -> SlvAdr   = (DPR_BYTE)slv_add;
		MyDpOrderPtr -> SlvState = slv_mode;
		MyDpOrderPtr -> DatLen   = 0;
		
		/* send DP request and receive confirmation */
		
		CiResult = dp_send_and_receive (&MyCiRequest);
		
		build_result (CiResult, &MyCiRequest, error);
		
	}
	
	
#ifdef WIN32
	DpTrcSlvState (user_handle,slv_add,slv_mode,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
	
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_read_slv_par()                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function reads the parameters of a DP slave from the database.      */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_read_slv_par  (DPR_DWORD  user_handle,          /* in  */
                                         DPR_WORD   slv_add,              /* in  */
                                         DPR_WORD   type,                 /* in  */
                                         DPR_WORD   DP_MEM_ATTR *data_len,/* out */
                                         DPR_BYTE   DP_MEM_ATTR *data,    /* out */
                                         DP_ERROR_T DP_MEM_ATTR *error)   /* out */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	/* check syntax */
	
	if ( (SynCeckResult = dp_read_slv_par_syntax(slv_add,type,data_len,data,user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_READ_SLV_PAR,
			DpUserIdTable[user_handle].DpSyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		
		MyDpOrderPtr -> SlvAdr   = (DPR_BYTE)slv_add;
		MyDpOrderPtr -> DatTyp   = type;
		MyDpOrderPtr -> DatLen   = 0;
		
		/* send DP request and receive confirmation */
		
		CiResult = dp_send_and_receive (&MyCiRequest);
		
		build_result (CiResult, &MyCiRequest, error);
		
		/* copy result */
		if (error -> error_class == DP_OK)
		{
			memcpy (data,MyDpOrderPtr -> DatBuf,MyDpOrderPtr -> DatLen);
			*data_len=MyDpOrderPtr -> DatLen; 
		}
		else
		{
			*data_len=0;
		}
	}
	else
	{
		if(data_len)
		{
			*data_len = 0;
		}
	}
	
#ifdef WIN32
	DpTrcReadSlvPar (user_handle,slv_add,type,data_len,data,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_global_ctrl()                                             */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Using this function, a global control command can be sent to one         */
/*  or more slaves.                                                          */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_global_ctrl   (DPR_DWORD  user_handle,        /* in  */
                                         DPR_WORD   slv_add,            /* in  */
                                         DPR_BYTE   command,            /* in  */
                                         DPR_BYTE   group,              /* in  */
                                         DP_ERROR_T DP_MEM_ATTR *error) /* out */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	/* check syntax */
	
	if ( (SynCeckResult = dp_global_ctrl_syntax(slv_add,command,user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_GLOBAL_CTRL,
			DpUserIdTable[user_handle].DpSyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		
		MyDpOrderPtr -> SlvAdr     = (DPR_BYTE)slv_add;
		MyDpOrderPtr -> GroupCmd   = command;
		MyDpOrderPtr -> Identifier = group;
		
		MyDpOrderPtr -> DatLen   = 0;
		
		/* send DP request and receive confirmation */
		
		CiResult = dp_send_and_receive (&MyCiRequest);
		
		build_result (CiResult, &MyCiRequest, error);
		
	}
	
#ifdef WIN32
	DpTrcGlobalCtrl (user_handle,slv_add,command, group,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_read_alarm()                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function reads out status messages or alarms from slaves.           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  DP_CODE_ATTR DP_read_alarm  (DPR_DWORD  user_handle,        /* in     */
                                        DPR_WORD   slv_add,            /* in out */
                                        DP_ALARM_T DP_MEM_ATTR *alarm, /* out    */
                                        DP_ERROR_T DP_MEM_ATTR *error) /* out    */  
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	/* check syntax */
	
	if ( (SynCeckResult = dp_read_alarm_syntax(slv_add,alarm,user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_READ_ALARM,
			DpUserIdTable[user_handle].DpSyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		
		MyDpOrderPtr -> SlvAdr   = (DPR_BYTE)slv_add;
		MyDpOrderPtr -> DatLen   = 0;
		
		/* send DP request and receive confirmation */
		
		CiResult = dp_send_and_receive (&MyCiRequest);
		
		build_result (CiResult, &MyCiRequest, error);
		
		/* copy result */
		if (error -> error_class == DP_OK)
		{
			memcpy (alarm->data_s,MyDpOrderPtr -> DatBuf,MyDpOrderPtr -> DatLen);
			
			if (MyDpOrderPtr -> DatLen == 0)
			{
				alarm -> msg = DP_MSG_NONE;
			}
			else
			{
				if(MyDpOrderPtr -> AlarmType & 0x80)
				{
					alarm -> msg = DP_MSG_STATUS;
				}
				else
				{
					alarm -> msg = DP_MSG_ALARM;
				}
			}
			alarm->length_s    = (DPR_BYTE)MyDpOrderPtr -> DatLen;
			alarm->alarm_type  = MyDpOrderPtr -> AlarmType;
			alarm->slot_number = MyDpOrderPtr -> SlotNumber;
			alarm->specifier   = MyDpOrderPtr -> AlarmIdentifier;
			alarm->slv_add     = MyDpOrderPtr -> SlvAdr;
		}
		else
		{
			memset (alarm,0,sizeof(DP_ALARM_T));
		}
	}
	
	
#ifdef WIN32
	DpTrcReadAlarm (user_handle,slv_add,alarm,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_disable_event()                                           */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function cancels an active DP_enable_event request.                 */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_disable_event (DPR_DWORD  user_handle,        /* in     */
                                         DP_ERROR_T DP_MEM_ATTR *error) /* out    */ 
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	/* check syntax */
	
	if ( (SynCeckResult = dp_disable_event_syntax(user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_DISABLE_EVENT,
			DpUserIdTable[user_handle].DpSyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		MyDpOrderPtr -> DatLen   = 0;
		
		/* send DP request and receive confirmation */
		
		CiResult = dp_send_and_receive (&MyCiRequest);
		
		build_result (CiResult, &MyCiRequest, error);
		
	}
	
#ifdef WIN32
	DpTrcDisableEvt (user_handle,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_get_err_txt()                                             */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function provides detailed error information in plain language      */
/*  from the DP_ERROR error structure.                                       */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/



DPR_DWORD DP_CODE_ATTR DP_get_err_txt ( DP_ERROR_T DP_MEM_ATTR *error,        /* in     */
									   DPR_STRING DP_MEM_ATTR *language,     /* in     */
									   DPR_STRING    text[DP_ERR_TXT_SIZE] ) /* out    */
{
    DPR_DWORD Result;
    size_t TotalLength   = 0;
	
#ifdef WIN32
    EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
    if (error == 0)
    {
		Result = DP_ERROR_REQ_PAR;
    }
    else if (language == 0)
    {
		Result = DP_ERROR_REQ_PAR;
    }
    else if (text == 0)
    {
		Result = DP_ERROR_REQ_PAR;
    }
    else
    {
		/* get text for error class */
		
		TotalLength = fetch_err_txt(language,text,error);
		Result = DP_OK;
    }
	
#ifdef WIN32
    DpTrcGetErrTxt (error,language,text);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
    return(Result);
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_init_sema_object                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function initializes a semaphore for certain DP events              */
/*  (for example process image has changed.                                  */
/*                                                                           */
/*****************************************************************************/



DPR_DWORD   DP_CODE_ATTR DP_init_sema_object (DPR_DWORD  user_handle,              /* in  */
                                              DPR_DWORD  sema_type,                /* in  */
                                              DPR_DWORD  DP_MEM_ATTR *sema_handle, /* out */
                                              DP_ERROR_T DP_MEM_ATTR *error)       /* out */
{
	DPR_DWORD      SynCeckResult;
	DPR_DWORD      CiResult;
	DPR_DWORD      SlvSnycCiHdl;
	DPR_DWORD      SlvAsnycCiHdl;
	DPR_DWORD      SlvResult;
	
	SynCeckResult = 0;
	CiResult      = 0;
	SlvResult     = DP_OK;
	SlvAsnycCiHdl = DP_OK;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	if (user_handle & 0x80000000)
	{
		/* get slvae ci_handles */
		SlvResult = DPS_get_ci_hdl(user_handle, 
			&SlvSnycCiHdl, 
			&SlvAsnycCiHdl, 
			error);
	}
	
	if ((SlvResult == DP_OK) &&
		((SynCeckResult = dp_init_sema_obj_syntax(user_handle,sema_type,sema_handle,error)) == DP_OK))
	{
		/* use handle for asynchronous calls! */
		if (user_handle & 0x80000000)
		{
			/* Slave */
			CiResult = CI_init_sema ( SlvAsnycCiHdl,
				sema_type,
				sema_handle );
		}
		else
		{
			/* Master */
			CiResult = CI_init_sema ( DpUserIdTable[user_handle].DpAsyncHandle,
				sema_type,
				sema_handle );
		}
		
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0; 
		
		if (CiResult == CI_RET_OK)
		{
			error -> error_class = DP_OK;
			error -> error_code  = 0;
		}
		else
		{
			error -> error_class = DP_ERROR_CI;
			error -> error_code  = CiResult;
		}
	}
	
#ifdef WIN32
	DpTrcInitSemaObj (user_handle, sema_type,sema_handle,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
	
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_delete_sema_object                                        */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function deletes a semaphore, previously initialized with 			 */
/*  DP_init_sema_object.                                                     */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD   DP_CODE_ATTR DP_delete_sema_object (DPR_DWORD  user_handle,       /* in  */
                                                DPR_DWORD  sema_handle,       /* in  */
                                                DP_ERROR_T DP_MEM_ATTR *error)/* out */
{
	DPR_DWORD      SynCeckResult;
	DPR_DWORD      CiResult;
	DPR_DWORD      SlvSnycCiHdl;
	DPR_DWORD      SlvAsnycCiHdl;
	DPR_DWORD      SlvResult;
	
	SynCeckResult = 0;
	CiResult      = 0;
	SlvResult     = DP_OK;
	SlvAsnycCiHdl = DP_OK;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	if (user_handle & 0x80000000)
	{
		/* get slvae ci_handles */
		SlvResult = DPS_get_ci_hdl(user_handle, 
			&SlvSnycCiHdl, 
			&SlvAsnycCiHdl, 
			error);
	}
	
	if ((SlvResult == DP_OK) &&
		((SynCeckResult = dp_delete_sema_obj_syntax(user_handle,sema_handle,error)) == DP_OK))
	{
		/* use handle for asynchronous calls! */
		if (user_handle & 0x80000000)
		{
			/* Slave */
			CiResult = CI_delete_sema ( SlvAsnycCiHdl,
				sema_handle );
		}
		else
		{
			/* Master */
			CiResult = CI_delete_sema ( DpUserIdTable[user_handle].DpAsyncHandle,
				sema_handle );
		}
		
		
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0; 
		
		if (CiResult == CI_RET_OK)
		{
			error -> error_class = DP_OK;
			error -> error_code  = 0;
		}
		else
		{
			error -> error_class = DP_ERROR_CI;
			error -> error_code  = CiResult;
		}
	}
	
#ifdef WIN32
	DpTrcDelSemaObj (user_handle,sema_handle,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
	
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_fast_logic_on()                                           */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function activates the fast logic of the CP 5613\5614.              */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DP_CODE_ATTR  DP_fast_logic_on( DPR_DWORD                       user_handle,   /* in  */
										 DPR_WORD                        fast_logic_id, /* in */
										 DP_FAST_LOGIC_T DP_MEM_ATTR *   fast_logic,    /* in  */
										 DP_ERROR_T DP_MEM_ATTR          *error)        /* out */
{
	
	DPR_DWORD   SynCeckResult;
	DPR_DWORD   CiResult;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	if ( (SynCeckResult = dp_fast_logic_on_syntax(user_handle,fast_logic,error)) == DP_OK)
	{
		
		CiResult =  CI_fast_logic_on (DpUserIdTable[user_handle].DpSyncHandle,
			fast_logic_id,fast_logic);
		
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
			error -> error_class  = DP_OK; 
			error -> error_code   = 0;
			error -> error_decode = 0;
			error -> error_code_1 = 0;
			error -> error_code_2 = 0;
		}
	}
	
#ifdef WIN32
	DpTrcFastLogicOn (user_handle,fast_logic_id,fast_logic,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_fast_logic_off()                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function deactivates the fast logic of the CP 5613\5614.            */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD DP_CODE_ATTR  DP_fast_logic_off( DPR_DWORD               user_handle,   /* in  */
										  DPR_WORD                fast_logic_id, /* in  */                                 
										  DP_ERROR_T DP_MEM_ATTR  *error)        /* out */
{
	
	DPR_DWORD   SynCeckResult;
	DPR_DWORD   CiResult;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	if ( (SynCeckResult = dp_fast_logic_off_syntax(user_handle,error)) == DP_OK)
	{
		
		CiResult =  CI_fast_logic_off(DpUserIdTable[user_handle].DpSyncHandle,fast_logic_id);
		
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
			error -> error_class  = DP_OK; 
			error -> error_code   = 0;
			error -> error_decode = 0;
			error -> error_code_1 = 0;
			error -> error_code_2 = 0;
		}
	}
	
#ifdef WIN32
	DpTrcFastLogicOff (user_handle,fast_logic_id,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_watchdog()                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function activates the watchdog of the CP 5613\5614.                */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_watchdog (DPR_DWORD  user_handle,        /* in  */
                                    DPR_DWORD  timeout,            /* in  */
                                    DPR_WORD   *wd_index,          /* out */
                                    DP_ERROR_T *error)             /* out */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	/* check syntax */
	
	if ( (SynCeckResult = dp_watchdog_syntax(user_handle,timeout,wd_index,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_WATCHDOG,
			DpUserIdTable[user_handle].DpSyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		MyDpOrderPtr -> DatLen   = 0;
		MyDpOrderPtr -> DatTyp   = 0;
		MyDpOrderPtr -> WdTime   = (DPR_WORD)timeout;
		
		/* send DP request and receive confirmation */
		CiResult = dp_send_and_receive (&MyCiRequest);
		
		build_result (CiResult, &MyCiRequest, error);
		
		if (error -> error_class == DP_OK)
		{
			*wd_index = MyDpOrderPtr -> UsrIndex;
		}
		else
		{
			*wd_index = 0;
		}
	}
	
#ifdef WIN32
	DpTrcWatchdog (user_handle,timeout,wd_index,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}


