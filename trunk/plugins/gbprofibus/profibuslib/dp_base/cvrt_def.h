/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Modul:      cvrt_def.h                                                   */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains the prototype declarations for the DP-Converter       */
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
/*  15.04.99 first release                                                   */
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

  
/* C, C++ */
/* ------ */  


#ifdef WIN32
 #ifdef __cplusplus	
 extern "C" {
 #endif
#endif


DPR_DWORD DP_register_slv_cvrt ( DPR_DWORD    user_handle,  /* in */
                                 DPR_BYTE     slv[127],     /* in  */
                                 DP_ERROR_T   *error    );  /* out */


DPR_DWORD DP_open_cvrt ( DPR_STRING  *cp_name,      /* in  */
                         DPR_DWORD   *user_handle,  /* out */
                         DP_ERROR_T  *error    );   /* out */

 
 
 
DPR_DWORD DP_CODE_ATTR DP_get_pointer_cvrt (DPR_DWORD user_handle,                        /* in  */
                                            DPR_DWORD timeout,                            /* in  */ 
                                            DPR_CP5613_DP_T volatile  DP_MEM_ATTR  **dpr, /* out */
                                            DP_ERROR_T  DP_MEM_ATTR    *error);           /* out */



DPR_DWORD DP_CODE_ATTR DP_release_pointer_cvrt (DPR_DWORD user_handle,           /* in  */
                                                DP_ERROR_T DP_MEM_ATTR *error);  /* out */



#ifdef WIN32
 #ifdef __cplusplus	
  }
 #endif
#endif
