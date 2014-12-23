/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Version:    1.0                                                          */
/*  Modul:      dps_syn.c                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains functions for syntax checking                         */
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

#include "dp_5613.h"
#include "dps_5614.h"
#include "5613_ret.h"
#include "5614_ret.h"
#include "ci_5613.h"

#include "dps_base.h"
#include "uif_def.h"
#include "fkt_def.h"

#ifdef WIN32
#pragma warning (disable:4100)
#endif

extern DPR_BYTE            max_user_diag_len;

/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_open_syntax()                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Mit dieser Funktion wird die Syntax des Requests DP_open ueberprueft.    */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_open_syntax (DPR_STRING *cp_name, DPR_DWORD *user_handle,
							DPR_WORD usr_count,
							DPR_DWORD slave_mode,DPR_WORD station_addr,
							DPS_INIT_DATA_T *init_data,DPS_MAX_DATA_T *max_data_lens,
							DP_ERROR_T *error)
{
	if (error)
	{
		error -> error_class  = DP_OK;
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
		else if (station_addr >=127)
		{
			error -> error_class  = DP_ERROR_REQ_PAR;
			error -> error_code   = DPS_RET_INV_SLAVE_ADDR;
		}
		else if (
			(!init_data) ||
			(
			(slave_mode & DPS_SM_SIMPLE) &&
			(
			(init_data->simple.user_prm_data_len > 237) ||
			(init_data->simple.cfg_data_len > 244)
			)
			) ||
			(
			(!(slave_mode & DPS_SM_SIMPLE)) &&
			(
			(init_data->dynamic.def_cfg_data_len > 244)
			)
			)
			
            )
		{
			error -> error_class  = DP_ERROR_REQ_PAR;
			error -> error_code   = DPS_RET_PAR_INIT_DATA;
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
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   dps_close_syntax()                                           */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  Mit dieser Funktion wird die Syntax des Requests dp_close ueberprueft.   */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD  dps_close_syntax (DP_ERROR_T *error,DPR_DWORD user_handle)
{
	if (error)
	{
		error -> error_class  = DP_OK;
		error -> error_decode = 0;
		error -> error_code_1 = 0;
		error -> error_code_2 = 0;
		
		if( (user_handle <= DP_MAX_USER_INST) && (DpsUserIdTable[user_handle].Used == 0xff))
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
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_start_syntax                                             */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_start_syntax (DPR_DWORD user_handle,DP_ERROR_T *error)
{
    DPR_DWORD   result;
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_code   = 0;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
        if( (user_handle > DP_MAX_USER_INST) || (DpsUserIdTable[user_handle].Used != 0xff))
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_USR_HNDL;
        }
        result = error -> error_class;
    }
    else  /* error == 0 */
    {
        result = DP_ERROR_REQ_PAR;
    }
    return result;
}


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_stop_syntax                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_stop_syntax (DPR_DWORD user_handle,DP_ERROR_T *error)
{
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
        if( (user_handle > DP_MAX_USER_INST) || (DpsUserIdTable[user_handle].Used != 0xff))
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_USR_HNDL;
        }
        return(error -> error_class);
    }
    else  /* error == 0 */
    {
        return(DP_ERROR_REQ_PAR);
    }
}


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_get_state_syntax                                         */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_get_state_syntax(DPR_DWORD user_handle, DPR_WORD *state,      /* in  */
								DP_ERROR_T *error)
{
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
		
        if( (user_handle > DP_MAX_USER_INST) || (DpsUserIdTable[user_handle].Used != 0xff))
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_USR_HNDL;
        }
        else if (!state)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_STATE;
        }
		
        return (error -> error_class);
    }
    else  /* error == 0 */
    {
        return(DP_ERROR_REQ_PAR);
    }
}

/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_get_baud_rate_syntax                                     */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_get_baud_rate_syntax(DPR_DWORD user_handle, DPR_WORD *state,
									DPR_WORD *baud_rate ,DP_ERROR_T *error)
{
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
		
        if( (user_handle > DP_MAX_USER_INST) || (DpsUserIdTable[user_handle].Used != 0xff))
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_USR_HNDL;
        }
        else if (!state)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_STATE;
        }
        else if (!baud_rate)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_BAUD_RATE;
        }
		
        return (error -> error_class);
    }
    else  /* error == 0 */
    {
        return(DP_ERROR_REQ_PAR);
    }
}

/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_get_gc_syntax                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_get_gc_command_syntax(DPR_DWORD user_handle, DPR_WORD *gc,      /* in  */
									 DP_ERROR_T *error)
{
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
		
        if( (user_handle > DP_MAX_USER_INST) || (DpsUserIdTable[user_handle].Used != 0xff))
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_USR_HNDL;
        }
        else if (!gc)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_GC;
        }
		
        return (error -> error_class);
    }
    else  /* error == 0 */
    {
        return(DP_ERROR_REQ_PAR);
    }
}



/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_set_diag_syntax                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_set_diag_syntax(DPR_DWORD user_handle, DPR_BYTE *user_diag_data,
							   DPR_WORD user_diag_len, DPR_WORD diag_state, DP_ERROR_T *error)
{
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
		
        if( (user_handle > DP_MAX_USER_INST) || (DpsUserIdTable[user_handle].Used != 0xff))
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_USR_HNDL;
        }
        else if(user_diag_len > max_user_diag_len)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_DIAG_LEN;
        }
        else if (!user_diag_data)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_DIAG_DATA;
        }
        else if(diag_state & (~0x07))
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_DIAG_STATE;
        }
		
        return (error -> error_class);
    }
    else  /* error == 0 */
    {
        return(DP_ERROR_REQ_PAR);
    }
}


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_trigger_user_wd_syntax                                   */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_trigger_user_wd_syntax (DPR_DWORD user_handle,DP_ERROR_T *error)
{
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
        if( (user_handle > DP_MAX_USER_INST) || (DpsUserIdTable[user_handle].Used != 0xff))
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_USR_HNDL;
        }
        return error -> error_class;
    }
    else  /* error == 0 */
    {
        return(DP_ERROR_REQ_PAR);
    }
}


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_calc_io_data_len_syntax                                  */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_calc_io_data_len_syntax(DPR_WORD cfg_len,
									   DPR_BYTE *cfg_data, DPR_WORD *in_data_len,
									   DPR_WORD *out_data_len, DP_ERROR_T *error)
{
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
		
        if(cfg_len > 244)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_CFG_LEN;
        }
        else if (!cfg_data)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_CFG_DATA;
        }
        else if (!in_data_len)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_IN_DATA_LEN;
        }
        else if (!out_data_len)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_OUT_DATA_LEN;
        }
        return (error -> error_class);
    }
    else  /* error == 0 */
    {
        return(DP_ERROR_REQ_PAR);
    }
}


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_set_resp_syntax                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_set_resp_syntax(DPR_DWORD user_handle, DPR_DWORD ind_ref,
							   DPR_WORD data_len,DPR_BYTE *data_blk,
							   DP_ERROR_T *error)
{
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
		
        if( (user_handle > DP_MAX_USER_INST) || (DpsUserIdTable[user_handle].Used != 0xff))
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_USR_HNDL;
        }
        else if (!data_blk)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_DATA_BLK;
        }
        else if (data_len != 2)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_DATA_LEN;
        }
        return (error -> error_class);
    }
    else  /* error == 0 */
    {
        return(DP_ERROR_REQ_PAR);
    }
}


/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_get_ind_syntax                                           */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_get_ind_syntax(DPR_DWORD user_handle, DPR_DWORD *ind_ref,
							  DPR_DWORD timeout, DPR_DWORD *indication,
							  DPR_WORD *data_len,DPR_BYTE *data_blk,
							  DP_ERROR_T *error)
{
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
		
        if( (user_handle > DP_MAX_USER_INST) || (DpsUserIdTable[user_handle].Used != 0xff))
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_USR_HNDL;
        }
        else if(!data_len)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DP_RET_PAR_DATA_LEN;
        }
        else if (*data_len < DPS_MAX_PDU_LEN)
        {
			error -> error_class = DP_ERROR_REQ_PAR;
			error -> error_code = DPS_RET_BUF_LEN;
        }
        else if (!ind_ref)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_IND_REF;
        }
        else if (!data_blk)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_DATA_BLK;
        }
		
        return (error -> error_class);
    }
    else  /* error == 0 */
    {
        return(DP_ERROR_REQ_PAR);
    }
}



/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
/*  Function:   dps_get_ci_hdl_syntax                                        */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*                                                                           */
/*****************************************************************************/

DPR_DWORD  dps_get_ci_hdl_syntax(DPR_DWORD user_handle, DPR_DWORD *sync_ci_hdl,
								 DPR_DWORD *async_ci_hdl,
								 DP_ERROR_T *error)
{
    if (error)
    {
        error -> error_class  = DP_OK;
        error -> error_decode = 0;
        error -> error_code_1 = 0;
        error -> error_code_2 = 0;
		
        if(!sync_ci_hdl)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_SYNC_HDL;
        }
        else if (!async_ci_hdl)
        {
            error -> error_class  = DP_ERROR_REQ_PAR;
            error -> error_code   = DPS_RET_PAR_ASYNC_HDL;
        }
		
        return (error -> error_class);
    }
    else  /* error == 0 */
    {
        return(DP_ERROR_REQ_PAR);
    }
}
