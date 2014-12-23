/****************************************************************************/
/*    Copyright (C) SIEMENS AG 1989..1998 All Rights Reserved.Confidential  */
/****************************************************************************/
/*     Projekt       :                                                      */
/*                                                                          */
/*     Modulname     : DPS_5614.H                                           */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
/****************************************************************************/
/* Task / Description:                                                      */
/*  - user interface definition of DPS_BASE.DLL                             */
/*                                                                          */
/****************************************************************************/
/*   Modification                                                           */
/*                                                                          */
/*   Date       Person       Modification                                   */
/*   08.12.98   MM2072       file created                                   */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
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

#ifndef __DPS_5614_H
 #define __DPS_5614_H

/* structure for DPS_open */
/* ====================== */

typedef union DPS_INIT_DATA_S
{
    struct DPS_SIMPLE_S
    {
        DPR_WORD    user_prm_data_len;
        DPR_BYTE    user_prm_data[244];
        DPR_WORD    cfg_data_len;
        DPR_BYTE    cfg_data[244];
    } GNUC__PACK simple;
    struct DPS_DYNAMIC_S
    {
        DPR_WORD    def_cfg_data_len;
        DPR_BYTE    def_cfg_data[244];
    } GNUC__PACK dynamic;
} DPS_INIT_DATA_T;

typedef struct DPS_MAX_DATA_S
{
    DPR_WORD    max_input_data_len;
    DPR_WORD    max_output_data_len;
    DPR_WORD    max_user_diag_len;
    DPR_WORD    max_user_prm_data_len;
    DPR_WORD    max_cfg_data_len;
    DPR_WORD    max_user_ssa_data_len;
} GNUC__PACK DPS_MAX_DATA_T;


/* baudrate constants */
/* ================== */


#define DPS_BD_9K6              0x09
#define DPS_BD_19K2             0x08
#define DPS_BD_45K45            0x07
#define DPS_BD_93K75            0x06
#define DPS_BD_187K5            0x05
#define DPS_BD_500K             0x04
#define DPS_BD_1M5              0x03
#define DPS_BD_3M               0x02
#define DPS_BD_6M               0x01
#define DPS_BD_12M              0x00
#define DPS_BD_AUTO_DETECT      0x7f

/* baud_state constants */
/* ==================== */

#define DPS_BAUD_SEARCH         0x00
#define DPS_BAUD_FOUND          0x01
#define DPS_BAUD_FOUND_WD       0x02

/* gc constants */
/* ============ */

#define DPS_CLEAR               0x02
#define DPS_FREEZE              0x08
#define DPS_UNFREEZE            0x04
#define DPS_SYNC                0x20
#define DPS_UNSYNC              0x10

/* dps_state constants */
/* =================== */

#define DPS_OFFLINE     0
#define DPS_WAIT_PRM    1
#define DPS_WAIT_CFG    2
#define DPS_DATA_EX     3

/* slave_mode constants */
/* =================== */

#define DPS_SM_DYNAMIC          0x00000000ul
#define DPS_SM_SIMPLE           0x00000001ul
#define DPS_SM_FREEZE_SUPP      0x00000002ul
#define DPS_SM_SYNC_SUPP        0x00000004ul
#define DPS_SM_V1_ENABLE        0x00010000ul


/* dps_set_diag constants */
/* ====================== */

#define DPS_EXT_DIAG        0x01
#define DPS_STAT_DIAG       0x02
#define DPS_EXT_DIAG_OV     0x04


/* dps indication constants */
/* ======================== */


#define DPS_NO_IND              0x00000000l
#define DPS_CHK_PRM             0x00000001l
#define DPS_CHK_CFG             0x00000002l
#define DPS_NEW_SSA             0x00000004l
#define DPS_BAUD_CHANGED        0x00000008l
#define DPS_GO_LEAVE_DATA_EX    0x00000010l
#define DPS_NEW_GC              0x00000020l
#define DPS_C2_INITIATE_REQ     0x00000040l
#define DPS_C2_READ_REQ         0x00000080l
#define DPS_C2_WRITE_REQ        0x00000100l
#define DPS_C2_XPORT_REQ        0x00000200l
#define DPS_C2_ABORT_REQ        0x00000400l
#define DPS_C1_READ_REQ         0x00000800l
#define DPS_C1_WRITE_REQ        0x00001000l
#define DPS_C1_ALARM_ACK        0x00002000l


/* dps response constants */
/* ====================== */

#define DPS_PRM_OK      1
#define DPS_PRM_FAULT   0
#define DPS_CFG_OK      1
#define DPS_CFG_FAULT   0
#define DPS_SSA_OK      1

/* other constants */
/* =============== */

#define DPS_MAX_PDU_LEN     244


#ifdef WIN32
 #ifdef __cplusplus
 extern "C" {
 #endif
#endif

/* ======================= */
/* DPS interface functions */
/* ======================= */

/* in  = request parameter */
/* out = return  parameter */

DPR_DWORD  DP_CODE_ATTR DPS_open            ( DPR_STRING        *cp_name,       /* in  */
                                              DPR_DWORD         *user_handle,   /* out */
                                              DPR_DWORD         slave_mode,     /* in  */
                                              DPR_WORD          station_addr,   /* in  */
                                              DPR_WORD          addr_change,    /* in  */
                                              DPR_WORD          pno_ident_nr,   /* in  */
                                              DPR_WORD          user_wd,        /* in  */
                                              DPS_INIT_DATA_T   *init_data,     /* in  */
                                              DPS_MAX_DATA_T    *max_data_lens, /* in  */
                                              DPR_WORD          baud_rate,      /* in  */
                                              DP_ERROR_T        *error );       /* out */

DPR_DWORD DP_CODE_ATTR DPS_close            ( DPR_DWORD         user_handle,    /* in  */
                                              DP_ERROR_T        *error );       /* out */

DPR_DWORD DP_CODE_ATTR DPS_start            ( DPR_DWORD         user_handle,    /* in  */
                                              DP_ERROR_T        *error );       /* out */

DPR_DWORD DP_CODE_ATTR DPS_stop             ( DPR_DWORD         user_handle,    /* in  */
                                              DP_ERROR_T        *error );       /* out */

DPR_DWORD DP_CODE_ATTR DPS_get_baud_rate    ( DPR_DWORD         user_handle,    /* in  */
                                              DPR_WORD          *state,         /* out */
                                              DPR_WORD          *baud_rate,     /* out */
                                              DP_ERROR_T        *error );       /* out */

DPR_DWORD DP_CODE_ATTR DPS_get_gc_command   ( DPR_DWORD         user_handle,    /* in  */
                                              DPR_WORD          *gc,            /* out */
                                              DP_ERROR_T        *error );       /* out */

DPR_DWORD DP_CODE_ATTR DPS_get_state        ( DPR_DWORD         user_handle,    /* in  */
                                              DPR_WORD          *state,         /* out */
                                              DP_ERROR_T        *error );       /* out */

DPR_DWORD DP_CODE_ATTR DPS_set_diag         ( DPR_DWORD         user_handle,    /* in  */
                                              DPR_BYTE          *user_diag_data,/* in  */
                                              DPR_WORD          user_diag_len,  /* in  */
                                              DPR_WORD          diag_state,     /* in  */
                                              DP_ERROR_T        *error );       /* out */

DPR_DWORD DP_CODE_ATTR DPS_calc_io_data_len ( DPR_WORD          cfg_len,        /* in  */
                                              DPR_BYTE          *cfg_data,      /* in  */
                                              DPR_WORD          *in_data_len,   /* out */
                                              DPR_WORD          *out_data_len,  /* out */
                                              DP_ERROR_T        *error );       /* out */

DPR_DWORD DP_CODE_ATTR DPS_set_resp         ( DPR_DWORD         user_handle,    /* in  */
                                              DPR_DWORD         ind_ref,        /* in  */
                                              DPR_WORD          data_len,       /* in  */
                                              DPR_BYTE          *data_blk,      /* in  */
                                              DP_ERROR_T        *error );       /* out */

DPR_DWORD DP_CODE_ATTR DPS_get_ind          ( DPR_DWORD         user_handle,    /* in  */
                                              DPR_DWORD         *ind_ref,       /* out */
                                              DPR_DWORD         timeout,        /* in  */
                                              DPR_DWORD         *indication,    /* inout */
                                              DPR_WORD          *data_len,      /* inout */
                                              DPR_BYTE          *data_blk,      /* out */
                                              DP_ERROR_T        *error);        /* out */

/* getting pointer for access to Dual Port Ram */

DPR_DWORD DP_CODE_ATTR DPS_get_pointer (DPR_DWORD                       user_handle,  /* in  */
                                       DPR_DWORD                       timeout,      /* in  */
                                       DPR_CP5613_DP_T volatile  DP_MEM_ATTR  **dpr, /* out */
                                       DP_ERROR_T      DP_MEM_ATTR    *error    );   /* out */


/* release of Dual Port Ram pointer */

DPR_DWORD DP_CODE_ATTR DPS_release_pointer (DPR_DWORD                  user_handle,  /* in  */
                                           DP_ERROR_T  DP_MEM_ATTR    *error    );  /* out */


#ifdef WIN32
 #ifdef __cplusplus
 }
 #endif
#endif


#endif



#ifndef DONT_USE_MS_PACK
 #pragma pack()
#endif



/****************************************************************************/
/*     END OF FILE:      DPS_5614.H                                         */
/****************************************************************************/
