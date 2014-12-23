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

// central build number:
#include "BUILD_NR.h"

// conversion of numbers into string:
#define chSTR(x)  #x
#define chSTR2(x) chSTR(x)

#define FW_MAJOR              1
#define FW_MINOR              2
#define	FW_VERS	              0007

#define FW_DATA_DAY           12
#define FW_DATA_MONTH         04
#define FW_DATA_YEAR          1999

#define FW_FILE_VERSION_STR  \
 "@(#1)FW_CP5613_5614" "  " "V " chSTR2(FW_MAJOR) "." chSTR2(FW_MINOR)\
 "." chSTR2(FW_VERS)"." chSTR2(SINEC_BUILD) " " chSTR2(FW_DATA_DAY)\
 "." chSTR2(FW_DATA_MONTH) "." chSTR2(FW_DATA_YEAR)


