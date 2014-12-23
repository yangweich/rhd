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


/* timeout at CI_receive (seconds) */
 
#define DP_WAIT_RCV_TIMEOUT    5

/* length of path */
#define DP_PATH_LEN          260

/* maximal number of user instances at the dp_base.dll */
#define DP_MAX_USER_INST      32


/* small or large error text information at function DP_get_err_txt()     */
/* if undefined at error codes only the error number will be returned     */
#define DP_LARGE_TEXT

#define LEVEL_DPS_OPEN               1
#define LEVEL_DPS_CLOSE              1
#define SELECT_DPS_OPEN_CLOSE        0x80000000

#define LEVEL_DPS_START              1
#define LEVEL_DPS_STOP               1
#define SELECT_DPS_START_STOP        0x40000000

#define LEVEL_DPS_GET_BAUD_RATE      2
#define SELECT_DPS_GET_BAUD_RATE     0x20000000

#define LEVEL_DPS_GET_GC_COMMAND     2
#define SELECT_DPS_GET_GC_COMMAND    0x10000000

#define LEVEL_DPS_GET_STATE          2
#define SELECT_DPS_GET_STATE         0x08000000

#define LEVEL_DPS_SET_DIAG           1
#define SELECT_DPS_SET_DIAG          0x04000000

#define LEVEL_DPS_CALC_IO_DATA_LEN   1
#define SELECT_DPS_CALC_IO_DATA_LEN  0x02000000

#define LEVEL_DPS_GET_IND            2
#define SELECT_DPS_GET_IND           0x01000000

#define LEVEL_DPS_SET_RESP           1
#define SELECT_DPS_SET_RESP          0x00800000




