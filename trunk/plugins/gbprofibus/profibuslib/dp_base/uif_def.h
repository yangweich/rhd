/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Modul:      uif_def.h                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains constants for the user interface functions.           */
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


/* timeout at CI_receive (Milliseconds) */
 
#define DP_WAIT_RCV_TIMEOUT    5000

/* length of path */
#define DP_PATH_LEN          260

/* maximal number of user instances at the dp_base.dll */
#define DP_MAX_USER_INST      32 


/* small or large error text information at function DP_get_err_txt()     */
/* if undefined at error codes only the error number will be returned     */
#define LARGE_TEXT 
#define DP_LARGE_TEXT 


/* Trace - Defines */
                   
#define LEVEL_DP_START_CP        1
#define SELECT_DP_START_CP       0x00000001

#define LEVEL_DP_RESET_CP        1
#define SELECT_DP_RESET_CP       0x00000002

#define LEVEL_DP_OPEN            1
#define SELECT_DP_OPEN           0x00000004

#define LEVEL_DP_CLOSE           1
#define SELECT_DP_CLOSE          0x00000008

#define LEVEL_DP_GET_PTR         1
#define SELECT_DP_GET_PTR        0x00000010

#define LEVEL_DP_REL_PTR         1
#define SELECT_DP_REL_PTR        0x00000020

#define LEVEL_DP_SET_MODE        1
#define SELECT_DP_SET_MODE       0x00000040

#define LEVEL_DP_SLV_STATE       1
#define SELECT_DP_SLV_STATE      0x00000080

#define LEVEL_DP_GLB_CTR         1
#define SELECT_DP_GLB_CTR        0x00000100

#define LEVEL_DP_GET_CREF        1
#define SELECT_DP_GET_CREF       0x00000200

#define LEVEL_DP_READ_SLV_PAR    1
#define SELECT_DP_READ_SLV_PAR   0x00000400


#define LEVEL_DP_READ_ALARM      1
#define SELECT_DP_READ_ALARM     0x00000800


#define LEVEL_DP_GET_ERR_TXT     1
#define SELECT_DP_GET_ERR_TXT    0x00001000

#define LEVEL_DP_INIT_SEMA_OBJ   1
#define SELECT_DP_INIT_SEMA_OBJ  0x00002000

#define LEVEL_DP_DEL_SEMA_OBJ    1
#define SELECT_DP_DEL_SEMA_OBJ   0x00004000

#define LEVEL_DP_DS_READ         1
#define SELECT_DP_DS_READ        0x00008000

#define LEVEL_DP_DS_WRITE        1
#define SELECT_DP_DS_WRITE       0x00010000

#define LEVEL_DP_ALARM_ACK       1
#define SELECT_DP_ALARM_ACK      0x00020000

#define LEVEL_DP_GET_ACT_CFG     1
#define SELECT_DP_GET_ACT_CFG    0x00040000

#define LEVEL_DP_ENABLE_EVT      1
#define SELECT_DP_ENABLE_EVT     0x00080000

#define LEVEL_DP_DISABLE_EVT     1
#define SELECT_DP_DISABLE_EVT    0x00100000

#define LEVEL_DP_GET_RESULT      1
#define SELECT_DP_GET_RESULT     0x00200000

#define LEVEL_DP_FL_ON_OFF       1
#define SELECT_DP_FL_ON_OFF      0x00400000


#define LEVEL_DP_WATCHDOG        1
#define SELECT_DP_WATCHDOGF      0x00800000

