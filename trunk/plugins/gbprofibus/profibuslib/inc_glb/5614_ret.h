/****************************************************************************/
/*    Copyright (C) SIEMENS AG 1989..1998 All Rights Reserved.Confidential  */
/****************************************************************************/
/*                                                                          */
/*     Modulname     : 5614_RET.H                                           */
/*                                                                          */
/****************************************************************************/
/* Task / Description:                                                      */
/*  -                                                                       */
/*                                                                          */
/****************************************************************************/
/*   Modification                                                           */
/*                                                                          */
/*   Date       Person       Modification                                   */
/*   08.01.99   MM           file created                                   */
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


/* DPS error codes */
/* =============== */
#define DPS_ERR_BEGIN                   0x00030000

/* DP_ERROR_REQ_PAR */

#define DPS_RET_INV_SLAVE_ADDR			0x00030000   /* parameter station_addr invalid                */
#define DPS_RET_DIN_DOUT_LEN			0x00030001   /* parameter max_input/output_data_len invalid   */
#define DPS_RET_DIAG_LEN				0x00030002   /* parameter max_user_diag_len invalid           */
#define DPS_RET_PRM_LEN					0x00030003   /* parameter max_user_prm_data_len invalid       */
#define DPS_RET_CFG_LEN					0x00030004   /* parameter max_cfg_data_len invalid            */
#define DPS_RET_SSA_LEN					0x00030005   /* parameter max_user_ssa_data_len invalid       */
#define DPS_RET_INV_CFG					0x00030006   /* parameter (def)_cfg_data invalid              */
#define DPS_RET_PAR_IND_REF    			0x00030007   /* parameter ind_ref invalid                     */
#define DPS_RET_PAR_DATA_BLK   			0x00030008   /* parameter data_blk invalid                    */
#define DPS_RET_PAR_SYNC_HDL   			0x00030009   /* parameter sync_hdl invalid                    */
#define DPS_RET_PAR_ASYNC_HDL  			0x0003000a   /* parameter async_hdl invalid                   */
#define DPS_RET_INV_TIMEOUT    			0x0003000b   /* parameter timeout invalid                     */
#define DPS_RET_PAR_CFG_LEN    			0x0003000c   /* parameter cfg_len invalid                     */
#define DPS_RET_PAR_CFG_DATA   			0x0003000d   /* parameter cfg_data invalid                    */
#define DPS_RET_PAR_IN_DATA_LEN			0x0003000e   /* parameter in_data_len invalid                 */
#define DPS_RET_PAR_OUT_DATA_LEN		0x0003000f   /* parameter out_data_len invalid                */
#define DPS_RET_PAR_STATE				0x00030010   /* parameter state invalid                       */
#define DPS_RET_PAR_BAUD_RATE			0x00030011   /* parameter baud_rate invalid                   */
#define DPS_RET_PAR_GC					0x00030012   /* parameter gc_cmd invalid                      */
#define DPS_RET_PAR_DIAG_LEN			0x00030013   /* parameter user_diag_len invalid               */
#define DPS_RET_PAR_DIAG_DATA			0x00030014   /* parameter user_diag_data invalid              */
#define DPS_RET_PAR_DIAG_STATE			0x00030015   /* parameter diag_state invalid                  */
#define DPS_RET_PAR_INIT_DATA 			0x00030016   /* parameter init_data  invalid                  */

#define DPS_RET_NOT_IMPLEMENTED			0x0003ffff   /* function is not implemented                   */

/* DP_ERROR_RES */

#define DPS_RET_LESS_MEM    			0x00030100   /* out of memory                      */
#define DPS_RET_NO_DPR_PTR  			0x00030101   /* no access to DPRAM                 */
#define DPS_RET_NO_SLAVE_MODULE         0x00030102   /* slave-module not present           */


/* DP_ERROR_EVENT_NET */

#define DPS_RET_NOT_OFFLINE       		0x00030200   /* offline-state required             */
#define DPS_RET_UNKNOWN_ERROR       	0x00030201   /* unknown error                      */
#define DPS_RET_SEQUENCE_ERROR         	0x00030202   /* service not allowed in this state  */
#define DPS_RET_BUF_LEN                	0x00030203   /* invalid buffer length              */
#define DPS_RET_DOUBLE_OPEN            	0x00030204   /* open already done                  */







/****************************************************************************/
/*     END OF FILE:      5614_RET.H                                         */
/****************************************************************************/
