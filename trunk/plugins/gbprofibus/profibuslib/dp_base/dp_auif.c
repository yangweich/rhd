/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613/5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Module:     dp_auif.c                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains the asynchronous user interface functions of the      */
/*  DP_BASE.DLL                                                              */
/*                                                                           */
/*  // in  = job parameters                                                  */
/*  // out = return parameters                                               */
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
#else
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#endif

#include "dp_5613.h"
#include "ci_5613.h"
#include "5613_ret.h"
#include "dp_base.h"
#include "fkt_def.h"



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_ds_read()                                                 */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function sends a "read data record" job to a DPV1 slave.            */
/*  The DPV1 data is read parallel to cyclic operation.                      */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_ds_read   (DPR_DWORD    user_handle,         /* in  */
									 DPC1_REQ_T DP_MEM_ATTR  *request, /* in  */
									 DP_ERROR_T DP_MEM_ATTR  *error)   /* out */
{
	DPR_DWORD         CiResult;
	DPR_DWORD         SynCeckResult;
	CI_DP_REQUEST_T   MyCiRequest;
	CI_DP_ORDER_T*    MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	if ( (SynCeckResult = dp_ds_read_syntax(request,user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_DS_READ,
			DpUserIdTable[user_handle].DpAsyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		;
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		MyDpOrderPtr -> CRef       = request -> c_ref;
		MyDpOrderPtr -> SlotNumber = request -> req.dp_ds_read.slot_number;
		MyDpOrderPtr -> Index      = request -> req.dp_ds_read.index;
		MyDpOrderPtr -> DatLen     = request -> req.dp_ds_read.length_s;
		MyDpOrderPtr -> DpOrderId  = request -> order_id;
		
		/* send DP request (receive confirmation with dp_get_result) */
		
		CiResult = dp_send (&MyCiRequest);
		build_result_async (CiResult, &MyCiRequest, error);
		
		/* check result, notifiy if successfully */
		if (error -> error_class == DP_OK_ASYNC)
		{
			DpUserIdTable[user_handle].DpDsWriteReadActiv[get_slv_add_from_cref(request -> c_ref)] = 0xff;
		}
		
	}
#ifdef WIN32
	DpTrcDsRead (user_handle,request,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}





/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_ds_write()                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function sends a "write data record" job to a DPV1 slave.           */
/*  The DPV1 data is written parallel to cyclic operation.                   */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_ds_write  (DPR_DWORD    user_handle,         /* in  */
									 DPC1_REQ_T DP_MEM_ATTR  *request, /* in  */
									 DP_ERROR_T DP_MEM_ATTR  *error)   /* out */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	if ( (SynCeckResult = dp_ds_write_syntax(request,user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_DS_WRITE,
			DpUserIdTable[user_handle].DpAsyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		MyDpOrderPtr -> CRef       = request -> c_ref;
		MyDpOrderPtr -> SlotNumber = request -> req.dp_ds_write.slot_number;
		MyDpOrderPtr -> Index      = request -> req.dp_ds_write.index;
		MyDpOrderPtr -> DpOrderId  = request -> order_id;
		MyDpOrderPtr -> DatLen     = request -> req.dp_ds_write.length_m;
		memcpy (MyDpOrderPtr -> DatBuf,
			request -> req.dp_ds_write.data_m,
			request -> req.dp_ds_write.length_m);
		
		/* send DP request (receive confirmation with dp_get_result) */
		
		CiResult = dp_send (&MyCiRequest);
		build_result_async (CiResult, &MyCiRequest, error);
		
		/* check result, notifiy if successfully */
		if (error -> error_class == DP_OK_ASYNC)
		{
			DpUserIdTable[user_handle].DpDsWriteReadActiv[get_slv_add_from_cref(request -> c_ref)] = 0xff;
		}
		
		
	}
#ifdef WIN32
	DpTrcDsWrite (user_handle,request,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_alarm_ack()                                               */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  In certain situations, slaves with DPC1 functionality report alarms.     */
/*  These alarms are received by the master as part of the diagnostic        */
/*  data. Alarms must be acknowledged by the master DP application           */
/*  explicitly using the DP_alarm_ack function.                              */
/*                                                                           */
/*****************************************************************************/



DPR_DWORD DP_CODE_ATTR DP_alarm_ack  (DPR_DWORD    user_handle,        /* in  */
									  DPC1_REQ_T DP_MEM_ATTR  *request,/* in  */
									  DP_ERROR_T DP_MEM_ATTR  *error)  /* out */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	if ( (SynCeckResult = dp_alarm_ack_syntax(request,user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_ALARM_ACK,
			DpUserIdTable[user_handle].DpAsyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		MyDpOrderPtr -> CRef             = request -> c_ref;
		MyDpOrderPtr -> SlotNumber       = request -> req.dp_alarm_ack.slot_number;
		MyDpOrderPtr -> AlarmType        = request -> req.dp_alarm_ack.alarm_type;
		MyDpOrderPtr -> AlarmIdentifier  = request -> req.dp_alarm_ack.specifier;
		
		MyDpOrderPtr -> DpOrderId        = request -> order_id;
		MyDpOrderPtr -> DatLen           = 0;
		
		/* send DP request (receive confirmation with dp_get_result) */
		
		CiResult = dp_send (&MyCiRequest);
		build_result_async (CiResult, &MyCiRequest, error);
		
		/* check result, notifiy if successfully */
		if (error -> error_class == DP_OK_ASYNC)
		{
			DpUserIdTable[user_handle].DpAlarmAckActiv[get_slv_add_from_cref(request -> c_ref)] = 0xff;
		}
		
	}
#ifdef WIN32
	DpTrcAlarmAck (user_handle,request,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}



/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_get_actual_cfg()                                          */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function can be used to read the current configuration information  */
/*  of a slave. The data is requested from the slave using a special         */ 
/*  DP frame.                                                                */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_get_actual_cfg  (DPR_DWORD    user_handle,         /* in  */
										   DPC1_REQ_T DP_MEM_ATTR  *request, /* in  */
										   DP_ERROR_T DP_MEM_ATTR  *error)   /* out */
{
	
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	if ( (SynCeckResult = dp_get_actual_cfg_syntax (request,user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_GET_CFG,
			DpUserIdTable[user_handle].DpAsyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		MyDpOrderPtr -> CRef             = request -> c_ref;
		MyDpOrderPtr -> DpOrderId        = request -> order_id;
		MyDpOrderPtr -> DatLen           = 0;
		
		/* send DP request (receive confirmation with DP_get_result) */
		
		CiResult = dp_send (&MyCiRequest);
		build_result_async (CiResult, &MyCiRequest, error);
		
		/* check result, notifiy if successfully */
		if (error -> error_class == DP_OK_ASYNC)
		{
			/* Note: treated as DP_ds_write/read */
			DpUserIdTable[user_handle].DpDsWriteReadActiv[get_slv_add_from_cref(request -> c_ref)] = 0xff;
		}
		
	}
	
#ifdef WIN32
	DpTrcGetActualCfg (user_handle,request,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_enable_event()                                            */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  By calling this function, important events can be reported to the        */
/*  DP application explicitly.                                               */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_enable_event (DPR_DWORD    user_handle,        /* in  */
										DPC1_REQ_T DP_MEM_ATTR  *request,/* in  */
										DP_ERROR_T DP_MEM_ATTR  *error)  /* out */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	CI_DP_REQUEST_T MyCiRequest;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	if ( (SynCeckResult = dp_enable_event_syntax (request,user_handle,error)) == DP_OK)
	{
		build_dp_header (&MyCiRequest,CI_DP_ENABLE_EVENT,
			DpUserIdTable[user_handle].DpAsyncHandle,
			DpUserIdTable[user_handle].CpIndex);
		
		MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiRequest.data;
		MyDpOrderPtr -> CRef             = request -> c_ref;
		MyDpOrderPtr -> DpOrderId        = request -> order_id;
		MyDpOrderPtr -> Selector         = request -> req.dp_enable_evt.selector;
		MyDpOrderPtr -> MstMode          = request -> req.dp_enable_evt.mst_state;
		MyDpOrderPtr -> DatLen           = DPR_MAX_SLAVE_NR + 2; /* 2=new_mst_state,mst_event */
		
		/* send DP request (receive confirmation with dp_get_result) */
		
		CiResult = dp_send (&MyCiRequest);
		build_result_async (CiResult, &MyCiRequest, error);
		
		/* check result, notifiy if successfully */
		if (error -> error_class == DP_OK_ASYNC)
		{
			DpUserIdTable[user_handle].DpEnableEvtActiv = 0xff;
		}
	}
	
#ifdef WIN32
	DpTrcEnableEvt (user_handle,request,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_get_result()                                              */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function fetches the result of a completed job. With each          */
/*  asynchronous job, the corresponding result must be fetched using         */
/*  this function.                                                           */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_get_result  (DPR_DWORD    user_handle,          /* in  */
									   DPR_DWORD    timeout,              /* in  */
									   DPR_WORD   DP_MEM_ATTR  *req_type, /* out */
									   DPC1_REQ_T DP_MEM_ATTR  *result,   /* out */
									   DP_ERROR_T DP_MEM_ATTR  *error)    /* out */
{
	DPR_DWORD       CiResult;
	DPR_DWORD       SynCeckResult;
	DPR_DWORD       MyTimeout;
	CI_DP_REQUEST_T MyCiResult;
	CI_DP_ORDER_T*  MyDpOrderPtr;
	DPR_WORD        SlvAdd;
	
	
	if ( (SynCeckResult = dp_get_result_syntax (req_type,result,user_handle,error)) == DP_OK)
	{
		/* set timeout limit */
		if (timeout > DP_TIMEOUT_FOREVER)
		{
			MyTimeout = DP_TIMEOUT_FOREVER;
		}
		else
		{
			MyTimeout = timeout;
		}
		
		/* Receive confirmation */
		CiResult = dp_receive (&MyCiResult,DpUserIdTable[user_handle].DpAsyncHandle,MyTimeout);
		build_result_async (CiResult, &MyCiResult, error);
		
		/* copy result, if available */
		switch (error -> error_class)
		{
		case DP_OK:
		case DP_ERROR_EVENT:
		case DP_ERROR_EVENT_NET:
		case DP_ERROR_RES:
		case DP_ERROR_REQ_PAR:
			{
				MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiResult.data;
				
				switch((MyCiResult.header.opcode) & 0x0000ffff)
				{
				case CI_DP_DS_READ:
					{
						/* reset flag */
						SlvAdd = get_slv_add_from_cref(MyDpOrderPtr -> CRef);
						DpUserIdTable[user_handle].DpDsWriteReadActiv[SlvAdd] = 0x00;
						
						*req_type = DP_DS_READ;
						memcpy (result->req.dp_ds_read.data_s,
							MyDpOrderPtr -> DatBuf,
							MyDpOrderPtr -> DatLen);
						
						result->order_id                   = MyDpOrderPtr -> DpOrderId;
						result->c_ref                      = MyDpOrderPtr -> CRef;
						if (error -> error_class == DP_OK)
						{
							result->req.dp_ds_read.length_s    = (unsigned char)MyDpOrderPtr -> DatLen;
						}
						else
						{
							result->req.dp_ds_read.length_s    = (unsigned char)0;
						}
						
						result->req.dp_ds_read.slot_number = MyDpOrderPtr -> SlotNumber;
						result->req.dp_ds_read.index       = MyDpOrderPtr -> Index;
						
						break;
					}
				case CI_DP_DS_WRITE:
					{
						/* reset flag */
						SlvAdd = get_slv_add_from_cref(MyDpOrderPtr -> CRef);
						DpUserIdTable[user_handle].DpDsWriteReadActiv[SlvAdd] = 0x00;
						
						*req_type = DP_DS_WRITE;
						memcpy (result->req.dp_ds_write.data_m,
							MyDpOrderPtr -> DatBuf,
							MyDpOrderPtr -> DatLen);
						
						result->order_id                   = MyDpOrderPtr -> DpOrderId;
						result->c_ref                      = MyDpOrderPtr -> CRef;
						result->req.dp_ds_write.length_m    = (unsigned char)MyDpOrderPtr -> DatLen;
						result->req.dp_ds_write.slot_number = MyDpOrderPtr -> SlotNumber;
						result->req.dp_ds_write.index       = MyDpOrderPtr -> Index;
						
						break;
					}
				case CI_DP_ALARM_ACK:
					{
						/* reset flag */
						SlvAdd = get_slv_add_from_cref(MyDpOrderPtr -> CRef);
						DpUserIdTable[user_handle].DpAlarmAckActiv[SlvAdd] = 0x00;
						
						*req_type = DP_ALARM_ACK;
						result->order_id                     = MyDpOrderPtr -> DpOrderId;
						result->c_ref                        = MyDpOrderPtr -> CRef;
						result->req.dp_alarm_ack.slot_number = MyDpOrderPtr -> SlotNumber;
						result->req.dp_alarm_ack.alarm_type  = MyDpOrderPtr -> AlarmType;
						result->req.dp_alarm_ack.specifier   = MyDpOrderPtr -> AlarmIdentifier;
						
						break;
					}
				case CI_DP_ENABLE_EVENT:
					{
						/* reset flag */
						DpUserIdTable[user_handle].DpEnableEvtActiv = 0x00;
						
						
						*req_type = DP_ENABLE_EVENT;
						result->order_id                     = MyDpOrderPtr -> DpOrderId;
						result->c_ref                        = MyDpOrderPtr -> CRef;
						result->req.dp_enable_evt.mst_state   = (DPR_BYTE)MyDpOrderPtr -> MstMode;
						result->req.dp_enable_evt.selector    = MyDpOrderPtr -> Selector;
						memcpy (result->req.dp_enable_evt.event,
							MyDpOrderPtr -> DatBuf,
							DPR_MAX_SLAVE_ADDR);
						result->req.dp_enable_evt.mst_event     = MyDpOrderPtr -> DatBuf[DPR_MAX_SLAVE_NR];
						result->req.dp_enable_evt.new_mst_state = MyDpOrderPtr -> DatBuf[DPR_MAX_SLAVE_NR+1];
						break;
					}
				case CI_DP_GET_CFG:
					{
						/* reset flag */
						SlvAdd = get_slv_add_from_cref(MyDpOrderPtr -> CRef);
						DpUserIdTable[user_handle].DpDsWriteReadActiv[SlvAdd] = 0x00;
						
						*req_type = DP_GET_CFG;
						result->order_id                     = MyDpOrderPtr -> DpOrderId;
						result->c_ref                        = MyDpOrderPtr -> CRef;
						result->req.dp_get_cfg.length_s      = (unsigned char)MyDpOrderPtr -> DatLen;
						memcpy (result->req.dp_get_cfg.data_s,
							MyDpOrderPtr -> DatBuf,
							MyDpOrderPtr -> DatLen);
						
						break;
					}
					
				default:
					{
						*req_type = DP_NO_CNF;
						break;
					}
		  }
		  break;
		}
		case DP_ERROR_CI:
			{
				/* check if abort by CI.dll */
				if (error -> error_code == CI_RET_RECEIVE_TIMEOUT_CANCEL)
				{
					/* new error class */
					error -> error_class = DP_ERROR_USR_ABORT;
					MyDpOrderPtr = (CI_DP_ORDER_T*)MyCiResult.data;
					
					switch((MyCiResult.header.opcode) & 0x0000ffff)
					{
					case CI_DP_DS_READ:
						{
							*req_type = DP_DS_READ;
							break;
						}
					case CI_DP_DS_WRITE:
						{
							*req_type = DP_DS_WRITE;
							break;
						}
					case CI_DP_ALARM_ACK:
						{
							*req_type = DP_ALARM_ACK;
							break;
						}
					case CI_DP_ENABLE_EVENT:
						{
							/* reset flag */
							DpUserIdTable[user_handle].DpEnableEvtActiv = 0x00;
							
							*req_type = DP_ENABLE_EVENT;
							break;
						}
					case CI_DP_GET_CFG:
						{
							*req_type = DP_GET_CFG;
							break;
						}
					default:
						{
							*req_type = DP_NO_CNF;
							break;
						}
					}
				}
				else
				{
					*req_type = DP_NO_CNF;
				}
				break;
			}
		default:
			{
				*req_type = DP_NO_CNF;
			}
	  }
	  
  }
  else
  {
	  if (req_type != 0)
	  {
		  *req_type = DP_NO_CNF;
	  }
  }
  
#ifdef WIN32
  DpTrcGetResult (user_handle,timeout,req_type,result,error);
#endif
  
  return ((!error) ? (SynCeckResult) : (error -> error_class));
}




/*****************************************************************************/
/*  Project:    CP 5613\5614                                                 */
/*  Component:  DP_BASE.DLL                                                  */
/*  Function:   DP_get_cref()                                                */
/*---------------------------------------------------------------------------*/
/*  Comment:                                                                 */
/*  This function obtains the C_ref for the DPC1 functions from the          */
/*  slave address and the User_ID. The C_ref must be entered in the          */
/*  DPC1_REQ_S structure as a job parameter.                                 */
/*                                                                           */
/*****************************************************************************/


DPR_DWORD DP_CODE_ATTR DP_get_cref  (DPR_DWORD  user_handle,       /* in  */
									 DPR_WORD   slv_add,           /* in  */
									 DPR_DWORD  DP_MEM_ATTR *c_ref,/* out */
									 DP_ERROR_T DP_MEM_ATTR *error)/* out */
{
	DPR_DWORD  SynCeckResult;
	
#ifdef WIN32
	EnterCriticalSection(&DP_CriticalSection);
#endif
	
	
	if ( (SynCeckResult = dp_get_cref_syntax (slv_add,c_ref,user_handle,error)) == DP_OK)
	{
		*c_ref = (((DPR_DWORD) (slv_add << 16)) | (user_handle & 0x0000ffff));
	}
	
#ifdef WIN32
	DpTrcGetCref (user_handle,slv_add,c_ref,error);
#endif
	
#ifdef WIN32
	LeaveCriticalSection(&DP_CriticalSection);
#endif
	
	return ((!error) ? (SynCeckResult) : (error -> error_class));
}




