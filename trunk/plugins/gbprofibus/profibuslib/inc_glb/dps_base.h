/***************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                       */
/*    All Rights Reserved.                                                 */
/***************************************************************************/
/*                                                                         */
/*    Component            &K: CP 5614                :K&                  */
/*                                                                         */
/*    Name of module       &M: dps_base.h             :M&                  */
/*                                                                         */
/*    Version              &V: V1.0                   :V&                  */
/*                                                                         */
/***************************************************************************/
/*    Description:  This file contains the declarations used by            */
/*                  the DPS_BASE.DLL                                       */
/***************************************************************************/
/*                                                                          */
/* Dieses Programm ist Freeware. Jedem Benutzer steht es frei, dieses       */
/* Programm unentgeltlich zu nutzen, zu kopieren, zu verändern, in andere   */
/* Applikationen zu integrieren und/oder weiterzugeben, vorausgesetzt, dass */
/* die im Programm enthaltenen Urheberrechtsvermerke und Marken unverändert */
/* übernommen werden und jede Änderung des Programms als solche bezeichnet  */
/* wird.                                                                    */
/*                                                                          */
/* JEGLICHE GEWÄHRLEISTUNG FÜR DIE FUNKTIONSTÜCHTIGKEIT ODER KOMPATIBILITÄT */
/* DIESES PROGRAMMS IST AUSGESCHLOSSEN. DIE BENUTZUNG ERFOLGT AUF EIGENE    */
/* VERANTWORTUNG UND GEFAHR.                                                */ 
/*                                                                          */
/*                                                                          */
/* This software is Freeware. You may copy, modify, integrate it into       */
/* another application and use it for free as well as distribute it to      */
/* others, provided however, that all trademarks and copyright notices      */
/* remain unchanged and any modification of the software is marked.         */
/*                                                                          */
/* SINCE THIS SOFTWARE IS DISTRIBUTED FOR FREE, IT IS PROVIDED "AS IS"      */
/* WITHOUT ANY REPRESENTATION OR WARRANTY OF ANY KIND EITHER EXPRESSED OR   */
/* IMPLIED INCLUDING BUT NOT LIMITED TO IMPLIED WARRANTIES FOR              */
/* MERCHANTIBILITY OR FITNESS FOR USE. ANY USE OF THE SOFTWARE IS ON YOUR   */
/* OWN RISK AND RESPONSIBILITY.                                             */
/*                                                                          */
/****************************************************************************/

#ifndef DONT_USE_MS_PACK
 #pragma pack(1)
#endif

#ifdef __GNUC__
#define GNUC__PACK __attribute__ ((packed))
#else
#define GNUC__PACK
#endif



  #define MAX_BUFF_LEN    308
  #define DP_TYP_FALSE      0
  #define DPS_ORDER_HEADER_SIZE 32

  #define DPS_USR_HDL_ID    0x80000000u;

  typedef struct CI_DPS_OPEN_S
  {
    DPR_DWORD           slave_mode;
    DPR_WORD            station_addr;
    DPR_WORD            addr_change;
    DPR_WORD            pno_ident_nr;
    DPR_WORD            user_wd;
    DPR_WORD            baud_rate;
    DPR_WORD            cfg_data_len;
    DPR_BYTE            cfg_data[244];
    DPR_WORD            user_prm_data_len;
    DPR_BYTE            user_prm_data[244];
    DPR_WORD            max_input_data_len;
    DPR_WORD            max_output_data_len;
    DPR_WORD            max_usr_diag_data_len;
    DPR_WORD            max_usr_prm_data_len;
    DPR_WORD            max_cfg_data_len;
    DPR_WORD            max_usr_ssa_data_len;
  } GNUC__PACK CI_DPS_OPEN_T;

#if 0
  typedef struct CI_DPS_GET_IND_S
  {
      DPR_DWORD         ind_ref;
      DPR_DWORD         timeout;
      DPR_DWORD         indication;
      DPR_WORD          data_len;
      DPR_BYTE          data_blk[244];
  } GNUC__PACK CI_DPS_GET_IND_T;
#endif

  typedef struct CI_DPS_RCV_IND_S
  {
      DPR_DWORD         ind_ref;
      DPR_DWORD         timeout;
      DPR_DWORD         indication;
      DPR_WORD          data_len;
      DPR_BYTE          data_blk[244];
  } GNUC__PACK CI_DPS_RCV_IND_T;

  typedef struct CI_DPS_SETUP_INDS_S
  {
      DPR_DWORD         indications;
  } GNUC__PACK CI_DPS_SETUP_INDS_T;

  typedef struct CI_DPS_SET_RESP_S
  {
      DPR_DWORD         ind_ref;
      DPR_WORD          indication;
      DPR_BYTE          resp;
  } GNUC__PACK CI_DPS_SET_RESP_T;

  typedef struct CI_DPS_GET_STATE_S
  {
      DPR_WORD          state;
  } GNUC__PACK CI_DPS_GET_STATE_T;

  typedef union CI_DPS_ORDER_DATA_S
  {
    CI_DPS_OPEN_T       open;
//  CI_DPS_GET_IND_T    get_ind;
    CI_DPS_SET_RESP_T   set_resp;
    CI_DPS_GET_STATE_T  get_state;
    CI_DPS_SETUP_INDS_T setup_inds;
    CI_DPS_RCV_IND_T    rcv_ind;
  } GNUC__PACK CI_DPS_ORDER_DATA_T;


  typedef struct CI_DPS_ORDER_S
  {
    /* error events  */
    DPR_DWORD   ErrClass;               /* Error class                     */
    DPR_DWORD   ErrCode;                /* Error code                      */
    DPR_BYTE    ErrDecode;              /* Error decode                    */
    DPR_BYTE    ErrCode1;               /* Error code 1                    */
    DPR_BYTE    ErrCode2;               /* Error code 2                    */
    DPR_BYTE    fill;
    DPR_WORD    DatLen;
    /* space for extensions */
    DPR_BYTE    reserved[18];
    CI_DPS_ORDER_DATA_T     OrderData;  /* Data buffer                     */
  } GNUC__PACK CI_DPS_ORDER_T;


  typedef struct DP_USER_ID_TABLE_S
  {
    DPR_WORD      Used;
    DPR_WORD      CpIndex;
    DPR_DWORD     DpSyncHandle;
    DPR_DWORD     DpAsyncHandle;
  } GNUC__PACK DP_USER_ID_TABLE_T;


/* ----------------------------------
   Definition of base functions via CI
   ----------------------------------- */
#define CI_DPS_OPEN             (0x00000000U + SUBSYSTEM_DPC31)
#define CI_DPS_CLOSE            (0x00000001U + SUBSYSTEM_DPC31)
#define CI_DPS_START            (0x00000002U + SUBSYSTEM_DPC31)
#define CI_DPS_STOP             (0x00000003U + SUBSYSTEM_DPC31)
#define CI_DPS_GET_BAUD_RATE    (0x00000004U + SUBSYSTEM_DPC31)
#define CI_DPS_GET_GC_COMMAND   (0x00000005U + SUBSYSTEM_DPC31)
#define CI_DPS_GET_STATE        (0x00000006U + SUBSYSTEM_DPC31)
#define CI_DPS_TRIGGER_USER_WD  (0x00000007U + SUBSYSTEM_DPC31)
//#define CI_DPS_GET_IND          (0x00000008U + SUBSYSTEM_DPC31)
#define CI_DPS_SET_RESP         (0x00000009U + SUBSYSTEM_DPC31)
#define CI_DPS_SETUP_INDS       (0x0000000AU + SUBSYSTEM_DPC31)
#define CI_DPS_RCV_IND          (0x0000000BU + SUBSYSTEM_DPC31)

/* ----------------------------------
   structure for accessing CI
   ----------------------------------- */
typedef struct CI_DPS_REQUEST_S
{
  CI_REQUEST_HEADER_T     header;
  CI_DPS_ORDER_T          order;
} GNUC__PACK CI_DPS_REQUEST_T;


#ifndef DONT_USE_MS_PACK
 #pragma pack()
#endif


/***************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                       */
/*    All Rights Reserved.                                                 */
/***************************************************************************/
