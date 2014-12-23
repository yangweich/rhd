/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Modul:      serr_txt.h                                                   */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains the error text for different error classes and        */
/*  error numbers.                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*  History of changes:                                                      */
/*  ===================                                                      */
/*                                                                           */
/*  Date     Comment                                                         */
/*----------!----------------------------------------------------------------*/
/*  18.05.98 first release                                                   */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Dieses Programm ist Freeware. Jedem Benutzer steht es frei, dieses        */
/* Programm unentgeltlich zu nutzen, zu kopieren, zu ver‰ndern, in andere    */
/* Applikationen zu integrieren und/oder weiterzugeben, vorausgesetzt, dass  */
/* die im Programm enthaltenen Urheberrechtsvermerke und Marken unver‰ndert  */
/* ¸bernommen werden und jede ƒnderung des Programms als solche bezeichnet   */
/* wird.                                                                     */
/*                                                                           */
/* JEGLICHE GEWƒHRLEISTUNG F‹R DIE FUNKTIONST‹CHTIGKEIT ODER KOMPATIBILITƒT  */
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



#ifndef __SLV_ERR_TXT__
#define __SLV_ERR_TXT__



#ifdef DP_LARGE_TEXT

/* ================================================================= */
/* text for DP error codes at at error classes except DP_ERROR_EVENT */
/* ================================================================= */

const ERR_INF_T ErrCodeSlv[] =
{
    {   DP_OK,                     /* error number always 0 */
        "DP_OK:",
        "Kein Fehler vorhanden\n",
        "",
        "",
        "",
        /* from here english */
        "DP_OK:",
        "No error found\n",
        "",
        "",
        "",
    },
    {   DPS_RET_INV_SLAVE_ADDR,
        "Fehlercode: DPS_RET_INV_SLAVE_ADDR",
        "Parameter slave_addr falsch"
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_INV_SLAVE_ADDR",
        "Bad slave_addr parameter",
        "",
        "",
        "",
    },

    {   DPS_RET_DIN_DOUT_LEN,       
        "Fehlercode: DPS_RET_DIN_DOUT_LEN",
        "Max. Input/Output falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_DIN_DOUT_LEN",
        "Bad max. input/output",
        "",
        "",
        "",
    },

    {   DPS_RET_DIAG_LEN,       
        "Fehlercode: DPS_RET_DIAG_LEN",
        "max_user_diag_len falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_DIAG_LEN",
        "Bad max_user_diag_len",
        "",
        "",
        "",
    },

    {   DPS_RET_PRM_LEN,
        "Fehlercode: DPS_RET_PRM_LEN",
        "max_user_prm_data_len falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PRM_LEN",
        "Bad max_user_prm_data_len",
        "",
        "",
        "",
    },

    {   DPS_RET_CFG_LEN,
        "Fehlercode: DPS_RET_CFG_LEN",
        "max_cfg_data_len falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_CFG_LEN",
        "Bad max_cfg_data_len",
        "",
        "",
        "",
    },

    {   DPS_RET_SSA_LEN,       
        "Fehlercode: DPS_RET_SSA_LEN",
        "max_user_ssa_data_len falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_SSA_LEN",
        "Bad max_user_ssa_data_len",
        "",
        "",
        "",
    },

    {   DPS_RET_INV_CFG,       
        "Fehlercode: DPS_RET_INV_CFG",
        "ung¸ltige Default-Konfiguration",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_INV_CFG",
        "Invalid default configuration",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_IND_REF,       
        "Fehlercode: DPS_RET_PAR_IND_REF",
        "ind_ref falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_IND_REF",
        "Bad ind_ref",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_DATA_BLK,
        "Fehlercode: DPS_RET_PAR_DATA_BLK",
        "data_blk falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_DATA_BLK",
        "Bad data_blk",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_SYNC_HDL,
        "Fehlercode: DPS_RET_PAR_SYNC_HDL",
        "Der Parameter SyncHandle ist ungÅltig",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_SYNC_HDL",
        "The SyncHandle parameter is invalid",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_ASYNC_HDL,
        "Fehlercode: DPS_RET_PAR_ASYNC_HDL",
        "Der Parameter AsyncHandle ist ungÅltig",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_ASYNC_HDL",
        "The AsyncHandle parameter is invalid",
        "",
        "",
        "",
    },

    {   DPS_RET_INV_TIMEOUT,
        "Fehlercode: DPS_RET_INV_TIMEOUT",
        "Der Parameter timeout ist ungÅltig",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_INV_TIMEOUT",
        "The timeout parameter is invalid",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_CFG_LEN,
        "Fehlercode: DPS_RET_PAR_CFG_LEN",
        "Der Parameter cfg_len ist falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_CFG_LEN",
        "Bad cfg_len parameter",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_CFG_DATA,
        "Fehlercode: DPS_RET_PAR_CFG_DATA",
        "cfg_data falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_CFG_DATA",
        "Bad cfg_data",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_IN_DATA_LEN,
        "Fehlercode: DPS_RET_PAR_IN_DATA_LEN",
        "in_data_len falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_IN_DATA_LEN",
        "Bad in_data_len",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_OUT_DATA_LEN,
        "Fehlercode: DPS_RET_PAR_OUT_DATA_LEN",
        "out_data_len falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_OUT_DATA_LEN",
        "Bad out_data_len",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_STATE,
        "Fehlercode: DPS_RET_PAR_STATE",
        "state falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_STATE",
        "Bad state",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_BAUD_RATE,
        "Fehlercode: DPS_RET_PAR_BAUD_RATE",
        "baud_rate falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_BAUD_RATE",
        "Bad baud_rate",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_GC,
        "Fehlercode: DPS_RET_PAR_GC",
        "gc falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_GC",
        "Bad gc",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_DIAG_LEN,
        "Fehlercode: DPS_RET_PAR_DIAG_LEN",
        "diag_len falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_DIAG_LEN",
        "Bad diag_len",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_DIAG_DATA,
        "Fehlercode: DPS_RET_PAR_DIAG_DATA",
        "diag_data falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_DIAG_DATA",
        "Bad diag_data",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_DIAG_STATE,
        "Fehlercode: DPS_RET_PAR_DIAG_STATE",
        "diag_state falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_DIAG_STATE",
        "Bad diag_state",
        "",
        "",
        "",
    },

    {   DPS_RET_PAR_INIT_DATA,
        "Fehlercode: DPS_RET_PAR_INIT_DATA",
        "init_data falsch",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_PAR_INIT_DATA",
        "Bad init_data",
        "",
        "",
        "",
    },

    {   DPS_RET_NOT_IMPLEMENTED,
        "Fehlercode: DPS_RET_NOT_IMPLEMENTED",
        "Der Dienst ist noch nicht implementiert",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_NOT_IMPLEMENTED",
        "The service is not yet implemented.",
        "",
        "",
        "",
    },

    {   DPS_RET_LESS_MEM,
        "Fehlercode: DPS_RET_LESS_MEM",
        "Die angeforderten Puffergrˆﬂen sind zu groﬂ",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_LESS_MEM",
        "The requested buffers are too long",
        "",
        "",
        "",
    },

    {   DPS_RET_NO_DPR_PTR,
        "Fehlercode: DPS_RET_NO_DPR_PTR",
        "kein Zugriff auf das DP-RAM",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_NO_DPR_PTR",
        "No access to the DP-RAM",
        "",
        "",
        "",
    },

    {   DPS_RET_NO_SLAVE_MODULE,
        "Fehlercode: DPS_RET_NO_SLAVE_MODULE",
        "Slavemodul nicht vorhanden",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_NO_SLAVE_MODULE",
        "Slave module does not exist.",
        "",
        "",
        "",
    },

    {   DPS_RET_NOT_OFFLINE,
        "Fehlercode: DPS_RET_NOT_OFFLINE",
        "Das Slavemodul ist nicht offline",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_NOT_OFFLINE",
        "The slave module is not offline",
        "",
        "",
        "",
    },
    {   DPS_RET_UNKNOWN_ERROR,
        "Fehlercode: DPS_RET_UNKNOWN_ERROR",
        "Unbekannter Fehler",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_UNKNOWN_ERROR",
        "Unknown error",
        "",
        "",
        "",
    },
    {   DPS_RET_SEQUENCE_ERROR,
        "Fehlercode: DPS_RET_SEQUENCE_ERROR",
        "Das Kommando ist im momentanen Betriebszustand nicht erlaubt",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_SEQUENCE_ERROR",
        "The command is not permitted in the current operating state.",
        "",
        "",
        "",
    },

    {   DPS_RET_BUF_LEN,
        "Fehlercode: DPS_RET_BUF_LEN",
        "Die PufferlÑnge ist ungÅltig",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_BUF_LEN",
        "The buffer length is invalid",
        "",
        "",
        "",
    },

    {   DPS_RET_DOUBLE_OPEN,
        "Fehlercode: DPS_RET_DOUBLE_OPEN",
        "DPS_open wurde bereits durchgefÅhrt",
        "",
        "",
        "",
        /* from here english */
        "error_code: DPS_RET_DOUBLE_OPEN",
        "DPS_open already done",
        "",
        "",
        "",
    },

    /* This entry terminates the error table */
    {   ERR_TXT_END,
        ERR_DEFAULT_TXT_D,
        "",
        "",
        "",
        "",
        /* from here english */
        ERR_DEFAULT_TXT_E,
        "",
        "",
        "",
        "",
    }
};

#endif  /* DP_LARGE_TEXT */


#endif  /*  __SLV_ERR_TXT__  */
