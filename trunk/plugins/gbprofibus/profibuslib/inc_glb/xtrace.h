/****************************************************************************/
/*     Copyright (C) Siemens AG 1994  All Rights Reserved. Confidential     */
/****************************************************************************/
/*                                                                          */
/*  Modul: trace.h                                                          */
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

#ifndef X_TRACE_H
#define X_TRACE_H

#ifdef WIN32

/* Win 32 */
/* ------ */



extern CT W32EXPORT_CX unsigned short CLX X_Trc_init (char* LevelKeyName,
                                                      char* SelectKeyName,
                                                      char* FilePathKeyName, // File-Pfad
                                                      char* TrcNumKeyName,
                                                      // ---------------
                                                      char* TextKeyName,
                                                      char* TextTitleKeyName,
                                                      char* TargetKeyName,
                                                      char* HeaderName,
                                                      char* RegistryPath,
                                                      char* FileName,
                                                      // ---------------
                                                      unsigned short DefaultLevel,
                                                      unsigned long  DefaultSelect,
                                                      unsigned short DefaultTrcNum);

extern CT W32EXPORT_CX void CLX X_Trc_close (unsigned short MyIndex);


extern CT W32EXPORT_CX void CLX X_Trc_write (unsigned short MyIndex,
                                             unsigned short MyLevel,
                                             unsigned long  MySelect,
                                             char *         MyTextBuf,
                                             char *         MyDumpBuf,
                                             unsigned short MyNumOfBytesToRead);

extern CT W32EXPORT_CX void CLX X_Trc_write_txt (unsigned short MyIndex,
                                               unsigned short MyLevel,
                                               unsigned long  MySelect,
                                               char *         MyTextBuf);
 

#endif  // WIN32

#endif  // X_TRACE_H
 
/****************************************************************************/
/*     Copyright (C) Siemens AG 1995  All Rights Reserved. Confidential     */
/****************************************************************************/
