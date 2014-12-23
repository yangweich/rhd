/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5614                                                      */
/*  Component:  DPS_BASE.DLL                                                 */
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

extern void        build_dps_header         (CI_DPS_REQUEST_T*,DPR_DWORD,DPR_DWORD);
extern DPR_DWORD   dps_send_and_receive     (CI_DPS_REQUEST_T*);
extern DPR_DWORD   dps_send                 (CI_DPS_REQUEST_T*);
extern DPR_DWORD   dps_receive              (CI_DPS_REQUEST_T*,DPR_DWORD,DPR_DWORD);
extern void        dps_build_result         (DPR_DWORD,CI_DPS_REQUEST_T*,DP_ERROR_T *);
extern void        dps_build_result_async   (DPR_DWORD,CI_DPS_REQUEST_T*,DP_ERROR_T *);
extern DPR_DWORD   dps_store_user           (DPR_DWORD,DPR_DWORD,DPR_DWORD*);
extern DPR_DWORD   dps_get_next_user        (DPR_DWORD *);
extern DPR_DWORD   dps_release_user         (DPR_DWORD);
extern void        dps_get_fw_path          (DPR_STRING**);
extern DPR_DWORD   dps_open_syntax          (DPR_STRING*,DPR_DWORD*,DPR_WORD,DPR_DWORD,DPR_WORD,DPS_INIT_DATA_T*,DPS_MAX_DATA_T*,DP_ERROR_T*);
extern DPR_DWORD   dps_close_syntax         (DP_ERROR_T*,DPR_DWORD);
extern DPR_DWORD   dps_start_syntax         (DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dps_stop_syntax          (DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dps_get_baud_rate_syntax (DPR_DWORD,DPR_WORD *,DPR_WORD *,DP_ERROR_T *);
extern DPR_DWORD   dps_get_gc_command_syntax(DPR_DWORD,DPR_WORD *,DP_ERROR_T *);
extern DPR_DWORD   dps_get_state_syntax     (DPR_DWORD,DPR_WORD *,DP_ERROR_T *);
extern DPR_DWORD   dps_set_diag_syntax      (DPR_DWORD,DPR_BYTE *,DPR_WORD,DPR_WORD,DP_ERROR_T *);
extern DPR_DWORD   dps_trigger_user_wd_syntax (DPR_DWORD,DP_ERROR_T *);
extern DPR_DWORD   dps_calc_io_data_len_syntax(DPR_WORD,DPR_BYTE *,DPR_WORD *,DPR_WORD *,DP_ERROR_T *);
extern DPR_DWORD   dps_set_resp_syntax      (DPR_DWORD,DPR_DWORD,DPR_WORD,DPR_BYTE *,DP_ERROR_T *);
extern DPR_DWORD   dps_get_ind_syntax       (DPR_DWORD,DPR_DWORD*,DPR_DWORD,DPR_DWORD *,DPR_WORD *,DPR_BYTE *,DP_ERROR_T *);
extern DPR_DWORD   dps_get_ci_hdl_syntax    (DPR_DWORD,DPR_DWORD*,DPR_DWORD*,DP_ERROR_T*);

/* global variables */
extern DPR_WORD           DpsUserCtr;
extern DP_USER_ID_TABLE_T DpsUserIdTable[];
#ifdef WIN32
extern CRITICAL_SECTION   DpsCritSec;
#endif
extern DPR_WORD           DpsTrcIndex;

/* trace functions */

void DpsTrcOpen                     (DPR_STRING*,DPR_DWORD,DPR_DWORD,DPR_WORD,DPR_WORD,DPR_WORD,DPR_WORD,DPS_INIT_DATA_T *,DPS_MAX_DATA_T  *,DPR_WORD,DP_ERROR_T*);
void DpsTrcClose                    (DPR_DWORD,DP_ERROR_T*);
void DpsTrcStart                    (DPR_DWORD,DP_ERROR_T*);
void DpsTrcStop                     (DPR_DWORD,DP_ERROR_T*);
void DpsTrcGetBaudRate              (DPR_DWORD,DPR_WORD,DPR_WORD,DP_ERROR_T*);
void DpsTrcGetGcCommand             (DPR_DWORD,DPR_WORD,DP_ERROR_T*);
void DpsTrcGetState                 (DPR_DWORD,DPR_WORD,DP_ERROR_T*);
void DpsTrcSetDiag                  (DPR_DWORD,DPR_BYTE*,DPR_WORD,DPR_WORD,DP_ERROR_T*);
void DpsTrcCalcIoDataLen            (DPR_WORD,DPR_BYTE*,DPR_WORD,DPR_WORD,DP_ERROR_T*);
void DpsTrcGetIndBegin              (DPR_DWORD,DPR_DWORD,DPR_DWORD,DPR_WORD,DP_ERROR_T*);
void DpsTrcGetIndEnd                (DPR_DWORD,DPR_DWORD,DPR_DWORD,DPR_WORD,DPR_BYTE*,DP_ERROR_T*);
void DpsTrcSetResp                  (DPR_DWORD,DPR_DWORD,DPR_WORD,DPR_BYTE*,DP_ERROR_T*);


