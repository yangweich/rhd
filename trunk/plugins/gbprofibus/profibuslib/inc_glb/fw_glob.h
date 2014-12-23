/****************************************************************************/
/*    Copyright (C) SIEMENS AG 1989..1998 All Rights Reserved.Confidential  */
/****************************************************************************/
/*     Projekt       :                                                      */
/*                                                                          */
/*     Modulname     : FW_GLOB.H                                            */
/*                                                                          */
/*     Ersteller     : Joerg Mensinger     A&D PT 2                         */
/*                                                                          */
/****************************************************************************/
/****************************************************************************/
/* Task / Description:                                                      */
/*  -                                                                       */
/*                                                                          */
/****************************************************************************/
/*   Modification                                                           */
/*                                                                          */
/*   Date       Person       Modification                                   */
/*   07.05.98   Me           file created                                   */
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

#ifndef __FW_GLOB_H
#define __FW_GLOB_H

#include <string.h>

#ifdef FW_MSC

#pragma warning (disable:4032) /* C4032: formal parameter 1 has different type when promoted */
#include <conio.h>
#include <dos.h>
#pragma warning (default:4032) 


#else // CADUL
#pragma noalign 
#endif

#include"inline.h"
#include "dp_5613.h"
#include "5613_ret.h"
#include "ci_5613.h"
#include "mem_5613.h"

#include "xx_trace.h"
#include "adm_cp.h"

#ifdef FW_MSC
#pragma bss_seg( "UNINI_DATA" )
#else

#endif

#endif





/****************************************************************************/
/*     END OF FILE:      FW_GLOB.H                                          */
/****************************************************************************/
