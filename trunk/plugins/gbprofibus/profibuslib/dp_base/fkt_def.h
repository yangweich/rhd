/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Modul:      fkt_def.h                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains the prototype declarations of internal used           */
/*  functions.                                                               */
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

extern void        build_dp_header          (CI_DP_REQUEST_T*,DPR_DWORD,DPR_DWORD,DPR_WORD);
extern DPR_DWORD   dp_send_and_receive      (CI_DP_REQUEST_T*);
extern DPR_DWORD   dp_send                  (CI_DP_REQUEST_T*);
extern DPR_DWORD   dp_receive               (CI_DP_REQUEST_T*,DPR_DWORD,DPR_DWORD);
extern void        build_result             (DPR_DWORD,CI_DP_REQUEST_T*,DP_ERROR_T *);
extern void        build_result_async       (DPR_DWORD,CI_DP_REQUEST_T*,DP_ERROR_T *);
extern DPR_DWORD   dp_set_mode_syntax       (DPR_WORD,DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_open_syntax           (DPR_STRING *,DPR_DWORD *,DPR_WORD,DP_ERROR_T *);
extern DPR_DWORD   dp_get_ptr_syntax        (DPR_DWORD,volatile DPR_CP5613_DP_T **,
                                             DP_ERROR_T *error);
extern DPR_DWORD   dp_rel_ptr_syntax        (DPR_DWORD,DP_ERROR_T *error);
extern DPR_DWORD   dp_close_syntax          (DP_ERROR_T *,DPR_DWORD);
extern DPR_DWORD   dp_start_cp_syntax       (DPR_STRING*,DP_ERROR_T*);
extern DPR_DWORD   dp_reset_cp_syntax       (DPR_STRING*,DP_ERROR_T*);
extern DPR_DWORD   dp_slv_state_syntax      (DPR_WORD, DPR_WORD,DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_read_slv_par_syntax   (DPR_WORD,DPR_WORD,DPR_WORD*,
                                             DPR_BYTE*,DPR_DWORD,DP_ERROR_T*);
extern DPR_DWORD   dp_global_ctrl_syntax    (DPR_WORD,DPR_BYTE,DPR_DWORD,DP_ERROR_T *);
extern size_t      fetch_err_txt            (DPR_STRING*,DPR_STRING*,DP_ERROR_T*);
extern DPR_DWORD   dp_ds_read_syntax        (DPC1_REQ_T*,DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_ds_write_syntax       (DPC1_REQ_T*,DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_alarm_ack_syntax      (DPC1_REQ_T*,DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_get_actual_cfg_syntax (DPC1_REQ_T*,DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_read_alarm_syntax     (DPR_WORD,DP_ALARM_T*,DPR_DWORD,DP_ERROR_T*); 
extern DPR_DWORD   dp_enable_event_syntax   (DPC1_REQ_T*,DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_disable_event_syntax  (DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_get_result_syntax     (DPR_WORD *,DPC1_REQ_T *,DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_get_cref_syntax       (DPR_WORD,DPR_DWORD*,DPR_DWORD,DP_ERROR_T*);
extern DPR_DWORD   dp_init_sema_obj_syntax  (DPR_DWORD,DPR_DWORD,void*,DP_ERROR_T *);
extern DPR_DWORD   dp_delete_sema_obj_syntax(DPR_DWORD,DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_fast_logic_on_syntax  (DPR_DWORD,DP_FAST_LOGIC_T *,DP_ERROR_T *);
extern DPR_DWORD   dp_fast_logic_off_syntax (DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dp_watchdog_syntax       (DPR_DWORD,DPR_DWORD,DPR_WORD *,DP_ERROR_T *);
extern DPR_DWORD   store_user               (DPR_DWORD,DPR_DWORD,DPR_WORD,DPR_DWORD*);
extern DPR_DWORD   get_next_user            (DPR_DWORD *);
extern DPR_DWORD   release_user             (DPR_DWORD);
extern DPR_WORD    get_slv_add_from_cref    (DPR_DWORD);
extern void        get_fw_path              (DPR_STRING**);
extern DPR_DWORD   DP_open_int              (DPR_STRING *,DPR_DWORD *,DP_ERROR_T *,DPR_WORD);    
extern DPR_DWORD   DP_get_pointer_int       (DPR_DWORD,DPR_DWORD,
                                             DPR_CP5613_DP_T volatile **,
                                             DP_ERROR_T  *,DPR_WORD);
extern DPR_DWORD   DP_release_pointer_int   (DPR_DWORD,DP_ERROR_T *,DPR_WORD);
   

/* common function with dps_base.dll */

extern DPR_DWORD   DPS_get_ci_hdl(DPR_DWORD user_handle, 
						          DPR_DWORD *sync_ci_hdl, 
						          DPR_DWORD *async_ci_hdl, 
						          DP_ERROR_T *error);



/* global variables */
extern DPR_WORD           DpUserCtr;
extern DP_USER_ID_TABLE_T DpUserIdTable[];

#ifdef WIN32
extern CRITICAL_SECTION   DP_CriticalSection;
extern DPR_WORD           DpTrcIndex;
/* Trace functions */
extern void        DpTrcStartCp             (DPR_STRING*,DPR_STRING*,DP_ERROR_T*);
extern void        DpTrcResetCp             (DPR_STRING*,DP_ERROR_T*);
extern void        DpTrcOpen                (DPR_STRING*,DPR_DWORD*,DP_ERROR_T*);
extern void        DpTrcClose               (DPR_DWORD,DP_ERROR_T*);
extern void        DpTrcGetPointer          (DPR_DWORD,DPR_DWORD,DPR_CP5613_DP_T volatile**,
                                             DP_ERROR_T*);
extern void        DpTrcRelPointer          (DPR_DWORD,DP_ERROR_T*);
extern void        DpTrcSetMode             (DPR_DWORD,DPR_WORD,DP_ERROR_T*);

extern void        DpTrcSlvState            (DPR_DWORD,DPR_WORD,DPR_WORD,DP_ERROR_T*);
extern void        DpTrcGlobalCtrl          (DPR_DWORD,DPR_WORD,DPR_BYTE,DPR_BYTE,DP_ERROR_T*);
extern void        DpTrcGetCref             (DPR_DWORD,DPR_WORD,DPR_DWORD*,DP_ERROR_T*);

extern void        DpTrcReadSlvPar          (DPR_DWORD,DPR_WORD,DPR_WORD,
                                             DPR_WORD*,DPR_BYTE*,DP_ERROR_T*);
extern void        DpTrcReadAlarm           (DPR_DWORD,DPR_WORD,DP_ALARM_T*,DP_ERROR_T*);
extern void        DpTrcGetErrTxt           (DP_ERROR_T*,DPR_STRING*,DPR_STRING[]);
extern void        DpTrcInitSemaObj         (DPR_DWORD,DPR_DWORD,DPR_DWORD*,DP_ERROR_T*);
extern void        DpTrcDelSemaObj          (DPR_DWORD,DPR_DWORD,DP_ERROR_T*);
extern void        DpTrcDsRead              (DPR_DWORD,DPC1_REQ_T*,DP_ERROR_T*);
extern void        DpTrcDsWrite             (DPR_DWORD,DPC1_REQ_T*,DP_ERROR_T*);
extern void        DpTrcAlarmAck            (DPR_DWORD,DPC1_REQ_T*,DP_ERROR_T*);
extern void        DpTrcGetActualCfg        (DPR_DWORD,DPC1_REQ_T*,DP_ERROR_T*);
extern void        DpTrcEnableEvt           (DPR_DWORD,DPC1_REQ_T*,DP_ERROR_T*);
extern void        DpTrcDisableEvt          (DPR_DWORD,DP_ERROR_T*);
extern void        DpTrcGetResult           (DPR_DWORD,DPR_DWORD,DPR_WORD*,
                                             DPC1_REQ_T*,DP_ERROR_T*);
extern void        DpTrcFastLogicOn         (DPR_DWORD,DPR_WORD,DP_FAST_LOGIC_T *,DP_ERROR_T *);
extern void        DpTrcFastLogicOff        (DPR_DWORD,DPR_WORD,DP_ERROR_T *);
extern void        DpTrcWatchdog            (DPR_DWORD,DPR_DWORD,DPR_WORD *, DP_ERROR_T*);


#endif 



