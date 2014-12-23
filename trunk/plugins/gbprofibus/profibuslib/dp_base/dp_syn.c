/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Module:     dp_syn.c                                                     */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains functions for syntax checking.                        */
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
#include "uif_def.h"
#include "fkt_def.h"



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_start_cp_syntax()                                         */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_start_cp request.              */
/*  Note: The pointer to the database can be null. In this case, the         */
/*        database configured in the registry is used.                       */
/*****************************************************************************/


DPR_DWORD  dp_start_cp_syntax (DPR_STRING *cp_name,
                               DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if(cp_name == 0)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_CP_NAME;
		}
		else
		{
			error -> error_class  = DP_OK; 
			error -> error_code   = 0;
		}
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_reset_cp_syntax()                                         */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_reset_cp request.              */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_reset_cp_syntax (DPR_STRING *cp_name,
                               DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if(cp_name == 0)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_CP_NAME;
		}
		else
		{
			error -> error_class  = DP_OK; 
			error -> error_code   = 0;
		}
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_set_mode_syntax()                                         */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_set_mode request.              */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_set_mode_syntax (DPR_WORD  mst_mode,DPR_DWORD user_handle,DP_ERROR_T *error)
{
	
	if (error)
	{
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		switch (mst_mode)
		{
        case DP_OFFLINE:
        case DP_STOP:
        case DP_CLEAR:
        case DP_OPERATE:
			{
				/* ok! */
				error -> error_class = DP_OK;
				break;
			}
        default:
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_MST_MODE;
				break;
			}
		}
		
		if (error -> error_class == DP_OK)
		{
			if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
			{
				error -> error_class  = DP_ERROR_REQ_PAR;
				error -> error_code   = DP_RET_PAR_USR_HNDL;
			}
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}

/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_open_syntax()                                             */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_open request.                  */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_open_syntax (DPR_STRING *cp_name, DPR_DWORD *user_handle,
                           DPR_WORD usr_count, DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if(cp_name == 0)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_CP_NAME;
		}
		else if (user_handle == 0)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
		else if (usr_count > DP_MAX_USER_INST)
		{
			error -> error_class  = DP_ERROR_RES; 
			error -> error_code   = DP_RET_TOO_MANY_USR;
		}
		else
		{
			error -> error_class  = DP_OK; 
			error -> error_code   = 0;
		}
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_get_ptr_syntax()                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_get_pointer request.           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dp_get_ptr_syntax(DPR_DWORD user_handle,
                             volatile DPR_CP5613_DP_T **dpr,
                             DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK; 
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (!(user_handle & 0x80000000))
		{
			/* Master */
			if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_USR_HNDL;
			}
		}
		
		if (error -> error_class == DP_OK)
		{
			if (dpr == 0)
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_DPR;
			}
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_rel_ptr_syntax()                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_release_pointer request.       */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_rel_ptr_syntax(DPR_DWORD user_handle,
                             DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK; 
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (!(user_handle & 0x80000000))
		{
			/* Master */
			if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_USR_HNDL;
			}
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
}





/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_close_syntax()                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the dp_close request.                 */
/*                                                                           */
/*****************************************************************************/



DPR_DWORD  dp_close_syntax (DP_ERROR_T *error,DPR_DWORD user_handle)
{
	if (error)
	{
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if( (user_handle <= DP_MAX_USER_INST) && (DpUserIdTable[user_handle].Used == 0xff))
		{
			error -> error_class  = DP_OK;
			error -> error_code   = 0;
		}
		else
		{
			error -> error_class  = DP_ERROR_REQ_PAR;
			error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
		return (error -> error_class);
		
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_slv_state_syntax()                                        */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_slv_state request.             */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_slv_state_syntax (DPR_WORD  slv_add, DPR_WORD slv_mode,
                                DPR_DWORD user_handle,DP_ERROR_T *error)
{
	
	if (error)
	{
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		switch (slv_mode)
		{
        case DP_SLV_ACTIVATE:
        case DP_SLV_DEACTIVATE:
        case DP_AUTOCLEAR_ON:
        case DP_AUTOCLEAR_OFF:
        case DP_SLV_ACTIVATE | DP_AUTOCLEAR_ON:
        case DP_SLV_ACTIVATE | DP_AUTOCLEAR_OFF:
        case DP_SLV_DEACTIVATE | DP_AUTOCLEAR_ON:
        case DP_SLV_DEACTIVATE | DP_AUTOCLEAR_OFF:
			{
				/* ok! */
				error -> error_class = DP_OK;
				break;
			}
        default:
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_SLV_MODE;
				break;
			}
		}
		if (error -> error_class == DP_OK)
		{
			if (slv_add >= DPR_MAX_SLAVE_ADDR)
			{
				if (slv_add != 0xff)
				{
					error -> error_class  = DP_ERROR_REQ_PAR; 
					error -> error_code   = DP_RET_PAR_SLV_ADD;
				}
				else
				{
					/* here only DP_AUTOCLEAR_ON/OFF allowed */
					if ((slv_mode != DP_AUTOCLEAR_ON) && (slv_mode != DP_AUTOCLEAR_OFF))
					{
						error -> error_class  = DP_ERROR_REQ_PAR; 
						error -> error_code   = DP_RET_PAR_SLV_ADD;
					}
				}
			}
			else if ((user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
			{
				error -> error_class  = DP_ERROR_REQ_PAR;
				error -> error_code   = DP_RET_PAR_USR_HNDL;
			}
		}
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_read_slv_par_syntax()                                     */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_read_slv_par request.          */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_read_slv_par_syntax (DPR_WORD  slv_add, DPR_WORD type,
                                   DPR_WORD *data_len, DPR_BYTE *data,
                                   DPR_DWORD user_handle,
                                   DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		switch (type)
		{
        case DP_SLV_TYP:
        case DP_SLV_PRM:
        case DP_SLV_CFG:
        case DP_SLV_ADD_TAB:
        case DP_SLV_USR:
			{
				/* ok! */
				error -> error_class = DP_OK;
				break;
			}
        default:
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_TYPE;
				break;
			}
		}
		if (error -> error_class == DP_OK)
		{
			if (slv_add >= DPR_MAX_SLAVE_ADDR)
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_SLV_ADD;
			}
			
			else if (data_len == 0)
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_DATA_LEN;
			}
			else if (data == 0)
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_DATA;
			}
			else if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
			{
				error -> error_class  = DP_ERROR_REQ_PAR;
				error -> error_code   = DP_RET_PAR_USR_HNDL;
			}
		}
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_global_ctrl_syntax()                                      */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_global_ctrl request.           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_global_ctrl_syntax (DPR_WORD  slv_add, DPR_BYTE command,
                                  DPR_DWORD user_handle, DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		
		if ( ((command & 
			(DP_SYNC|DP_UNSYNC|DP_FREEZE|DP_UNFREEZE|0x02)) != command))
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_CTRL_CMD;
		}
		else
		{
			if (error -> error_class == DP_OK)
			{
				if ((slv_add >= DPR_MAX_SLAVE_ADDR) && (slv_add != DP_BROADCAST_ADR))
				{
					error -> error_class  = DP_ERROR_REQ_PAR; 
					error -> error_code   = DP_RET_PAR_SLV_ADD;
				}
				else if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
				{
					error -> error_class  = DP_ERROR_REQ_PAR;
					error -> error_code   = DP_RET_PAR_USR_HNDL;
				}
			}
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_read_alarm_syntax()                                       */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_read_alarm request.            */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_read_alarm_syntax (DPR_WORD  slv_add, DP_ALARM_T* alarm,
                                 DPR_DWORD user_handle, DP_ERROR_T *error)
{
	if (error)
	{   error -> error_class  = DP_OK;
	error -> error_code   = 0;
	error -> error_decode = 0;
	error -> error_code_1 = 0;
	error -> error_code_2 = 0;
	
	if (alarm == 0)
	{
        error -> error_class  = DP_ERROR_REQ_PAR; 
        error -> error_code   = DP_RET_PAR_ALARM;         
	}
	
	if (error -> error_class == DP_OK)
	{
		if ((slv_add >= DPR_MAX_SLAVE_ADDR) && (slv_add != 0xff))
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_SLV_ADD;
		}
		else if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
		{
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
	}
	return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}





/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_ds_read_syntax()                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_ds_read request.               */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dp_ds_read_syntax (DPC1_REQ_T *request,DPR_DWORD user_handle,DP_ERROR_T *error)
{
	DPR_WORD SlvAdd;
	
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (request == 0)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_REQUEST;
		}
		else if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
		{
			error -> error_class  = DP_ERROR_REQ_PAR;
			error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
		else if ((request -> req.dp_ds_read.length_s > DPR_DPC1_MAX_LENGTH) ||
			(request -> req.dp_ds_read.length_s == 0))
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_LENGTH_S;
		}
		else if ((request -> c_ref & 0x0000ffff) != user_handle)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_CREF;
		}
		else 
		{
			SlvAdd = get_slv_add_from_cref(request -> c_ref);
			if(DpUserIdTable[user_handle].DpDsWriteReadActiv[SlvAdd] == 0xff)
			{
				/* DP-C1-Request is still running */
				error -> error_class  = DP_ERROR_EVENT_NET; 
				error -> error_code   = DP_RET_REQ_ACTIV;
			}
		}
		
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_ds_write_syntax()                                         */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_ds_write request.              */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_ds_write_syntax (DPC1_REQ_T *request,DPR_DWORD user_handle,DP_ERROR_T *error)
{
	DPR_WORD SlvAdd;
	
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (request != 0)
		{
			if ((request -> req.dp_ds_write.length_m > DPR_DPC1_MAX_LENGTH) ||
				(request -> req.dp_ds_write.length_m == 0))
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_LENGTH_M;
			}
			else if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
			{
				error -> error_class  = DP_ERROR_REQ_PAR;
				error -> error_code   = DP_RET_PAR_USR_HNDL;
			}
			else if ((request -> c_ref & 0x0000ffff) != user_handle)
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_CREF;
			}
			else 
			{
				SlvAdd = get_slv_add_from_cref(request -> c_ref);
				if(DpUserIdTable[user_handle].DpDsWriteReadActiv[SlvAdd] == 0xff)
				{
					/* DP-C1-Request is still running */
					error -> error_class  = DP_ERROR_EVENT_NET; 
					error -> error_code   = DP_RET_REQ_ACTIV;
				}
			}
		}
		else
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_REQUEST;
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_alarm_ack_syntax()                                        */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_alarm_ack request.             */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_alarm_ack_syntax (DPC1_REQ_T *request,DPR_DWORD user_handle,DP_ERROR_T *error)
{
	
	DPR_WORD SlvAdd;
	
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (request == 0)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_REQUEST;
		}
		else if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
		{
			error -> error_class  = DP_ERROR_REQ_PAR;
			error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
		else if ((request -> c_ref & 0x0000ffff) != user_handle)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_CREF;
		}
		else 
		{
			SlvAdd = get_slv_add_from_cref(request -> c_ref);
			if(DpUserIdTable[user_handle].DpAlarmAckActiv[SlvAdd] == 0xff)
			{
				/* dp_alarm ack request is still running */
				error -> error_class  = DP_ERROR_EVENT_NET; 
				error -> error_code   = DP_RET_REQ_ACTIV;
			}
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_get_actual_cfg_syntax()                                   */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_get_actual_cfg request.        */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_get_actual_cfg_syntax (DPC1_REQ_T *request,DPR_DWORD user_handle,DP_ERROR_T *error)
{
	DPR_WORD  SlvAdd;
	
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (request == 0)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_REQUEST;
		}
		else if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
		{
			error -> error_class  = DP_ERROR_REQ_PAR;
			error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
		else if ((request -> c_ref & 0x0000ffff) != user_handle)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_CREF;
		}
		else 
		{
			SlvAdd = get_slv_add_from_cref(request -> c_ref);
			if(DpUserIdTable[user_handle].DpDsWriteReadActiv[SlvAdd] == 0xff)
			{
				/* DP-C1-Request is still running */
				error -> error_class  = DP_ERROR_EVENT_NET; 
				error -> error_code   = DP_RET_REQ_ACTIV;
			}
		}
		
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_enable_event_syntax()                                     */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_await_event() request.         */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_enable_event_syntax (DPC1_REQ_T *request,DPR_DWORD user_handle,DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (request != 0)
		{
			switch(request->req.dp_enable_evt.selector)
			{
			case DP_DIAG_ALARM:
			case DP_SLAVE_STATE:
			case DP_MST_STATE:
			case (DP_DIAG_ALARM  | DP_SLAVE_STATE):
			case (DP_DIAG_ALARM  | DP_SLAVE_STATE | DP_MST_STATE):
			case (DP_DIAG_ALARM  | DP_MST_STATE):
			case (DP_SLAVE_STATE | DP_MST_STATE):
				{
					/* ok! */
					break;
				}
			default:
				{
					error -> error_class  = DP_ERROR_REQ_PAR; 
					error -> error_code   = DP_RET_PAR_SELECTOR;
					break;
				}
			}
			if (error -> error_class == DP_OK)
			{
				if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
				{
					error -> error_class  = DP_ERROR_REQ_PAR;
					error -> error_code   = DP_RET_PAR_USR_HNDL;
				}
				
			}
			if ((error -> error_class == DP_OK) && 
				(request->req.dp_enable_evt.selector & DP_MST_STATE))
			{
				switch(request->req.dp_enable_evt.mst_state)
				{
				case DP_OFFLINE:
				case DP_STOP:
				case DP_CLEAR:
				case DP_AUTOCLEAR:
				case DP_OPERATE:
					{
						/* ok! */
						break;
					}
				default:
					{
						error -> error_class  = DP_ERROR_REQ_PAR; 
						error -> error_code   = DP_RET_PAR_MST_MODE;
						break;
					}
				}
			}
			if (error -> error_class == DP_OK)
			{
				if( DpUserIdTable[user_handle].DpEnableEvtActiv == 0xff)
				{
					/* dp_await request is running */
					error -> error_class  = DP_ERROR_EVENT_NET; 
					error -> error_code   = DP_RET_REQ_ACTIV;
				}
			}
		}
		else
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_REQUEST;
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_disable_event_syntax_syntax()                             */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_disable_event request.         */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_disable_event_syntax (DPR_DWORD user_handle,DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
		{
			error -> error_class  = DP_ERROR_REQ_PAR;
			error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}


/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_get_result_syntax()                                       */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_get_result() request.          */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_get_result_syntax (DPR_WORD *req_type,DPC1_REQ_T *result,
                                 DPR_DWORD user_handle,DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (req_type != 0)
		{
			if (result == 0)
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_RESULT;
			}
			else if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
			{
				error -> error_class  = DP_ERROR_REQ_PAR;
				error -> error_code   = DP_RET_PAR_USR_HNDL;
			}
			
		}
		else
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_REQ_TYPE;
		}
		
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_get_cref_syntax()                                         */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_get_cref() request.            */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_get_cref_syntax (DPR_WORD slv_add,DPR_DWORD *c_ref,
                               DPR_DWORD user_handle,DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_decode = 0;
		error -> error_code   = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (c_ref == 0)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_CREF;
		}
		else if ((slv_add >= DPR_MAX_SLAVE_ADDR) && (slv_add != 0xff))
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_SLV_ADD;
		}
		else if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
		{
			error -> error_class  = DP_ERROR_REQ_PAR;
			error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_init_sema_obj_syntax()                                    */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_init_sema_object request.      */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_init_sema_obj_syntax (DPR_DWORD user_handle,DPR_DWORD sema_type,void* sema_handle,DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (sema_handle != 0)
		{
			switch(sema_type)
			{
			case DP_OBJECT_TYPE_INPUT_CHANGE:
			case DP_OBJECT_TYPE_DIAG_CHANGE:
			case DP_OBJECT_TYPE_CYCLE_INT:
			case DP_OBJECT_TYPE_FAST_LOGIC:
			case DP_OBJECT_TYPE_ASYNC:
				{
					/* ok */
					break;
				}
			default:
				{
					error -> error_class  = DP_ERROR_REQ_PAR; 
					error -> error_code   = DP_RET_PAR_SEMA_TYPE;
					break;
				}
			}
		}
		else 
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_REQ_TYPE;
		}
		if (!(user_handle & 0x80000000))
		{
			/* Master */
			if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_USR_HNDL;
			}
		}
		
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_delete_sema_obj_syntax()                                  */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_delete_sema_object request.    */
/*                                                                           */
/*****************************************************************************/



DPR_DWORD  dp_delete_sema_obj_syntax (DPR_DWORD user_handle,
                                      DPR_DWORD sema_handle,
                                      DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (sema_handle == 0)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_REQ_TYPE;
		}
		else if (!(user_handle & 0x80000000))
		{
			/* Master */
			if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_USR_HNDL;
			}
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_fast_logic_on_syntax()                                    */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_fast_logic_on  request.        */
/*                                                                           */
/*****************************************************************************/



DPR_DWORD  dp_fast_logic_on_syntax (DPR_DWORD user_handle,
									DP_FAST_LOGIC_T *fast_logic,
									DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (fast_logic == 0)
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_FL;
		}
		else if ( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_fast_logic_off_syntax()                                   */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_fast_logic_off request.        */
/*                                                                           */
/*****************************************************************************/



DPR_DWORD  dp_fast_logic_off_syntax (DPR_DWORD user_handle,
									 DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		
		/* Master */
		if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dp_watchdog_syntax()                                         */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function checks the syntax of the DP_watchdog request.              */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dp_watchdog_syntax (DPR_DWORD  user_handle,
                               DPR_DWORD  timeout,
                               DPR_WORD*  wd_index,
                               DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_code   = 0;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if (wd_index != 0)
		{
			if (timeout > 0xffff)
			{
				error -> error_class  = DP_ERROR_REQ_PAR; 
				error -> error_code   = DP_RET_PAR_TIMEOUT;
			}
		}
		else 
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_WD_INDEX;
		}
		
		/* Master */
		if( (user_handle > DP_MAX_USER_INST) || (DpUserIdTable[user_handle].Used == 0x00))
		{
			error -> error_class  = DP_ERROR_REQ_PAR; 
			error -> error_code   = DP_RET_PAR_USR_HNDL;
		}
		
		
		return (error -> error_class);
	}
	else  /* error == 0 */
	{
		return(DP_ERROR_REQ_PAR);
	}
	
}

