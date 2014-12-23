/***************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                       */
/*    All Rights Reserved.                                                 */
/***************************************************************************/
/*                                                                         */
/*    Component            &K: CP 5613                :K&                  */
/*                                                                         */
/*    Name of module       &M: dp_base.h              :M&                  */
/*                                                                         */
/*    Version              &V: V1.0                   :V&                  */
/*                                                                         */
/***************************************************************************/
/*    Description:  This file contains the declarations used by            */
/*                  the DP_BASE_FUNC.DLL                                   */
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




  #define RES1_LEN         10 /* 8 */
  #define RES2_LEN         14 /* 42 */
  #define RES3_LEN          4

  #define MAX_BUFF_LEN    256
  #define DP_TYP_FALSE      0

  typedef struct CI_DP_ORDER_S
  {

    /* DP requests	*/
	DPR_DWORD	UsrId;			        /* User ID                         */
	DPR_WORD	UsrIndex;		        /* Index of user                   */
	DPR_WORD	MstMode;		        /* DP mode                         */
	DPR_WORD	SlvState;		        /* Slave state                     */
	DPR_WORD	SlvType;		        /* Slave type                      */
	DPR_WORD	DatTyp;			        /* data type                       */
	DPR_WORD	WdTime;		            /* Watchdog time                   */
	DPR_WORD	Event;			        /* Event message                   */
	DPR_BYTE	SlvAdr;			        /* Slave address                   */
	DPR_BYTE	GroupCmd;		        /* Global Control command          */
	DPR_BYTE	Identifier;		        /* Global Control identifier       */
	DPR_BYTE	Reserved1[RES1_LEN];    /* Reserved	for CP 5613            */

	/* DPV1 requests */
	DPR_DWORD	CRef;			        /* DPV1 Connection reference       */
	DPR_DWORD	Selector;               /* Selector for enable event	       */       
	DPR_BYTE	SlotNumber;		        /* DPV1  slot number               */
	DPR_BYTE	Index;			        /* DPV1  Index	                   */
	DPR_BYTE	AlarmType;		        /* Alarm type                      */
	DPR_BYTE	AlarmIdentifier;        /* Identifier                      */
	DPR_BYTE	Reserved2[RES2_LEN];    /* Reserved                        */

	/* error events	 */
	DPR_DWORD	ErrClass;		        /* Error class	                   */
	DPR_DWORD	ErrCode;		        /* Error code                      */
	DPR_BYTE	ErrDecode;		        /* Error decode	                   */
	DPR_BYTE	ErrCode1;		        /* Error code 1                    */
	DPR_BYTE	ErrCode2;		        /* Error code 2	                   */
	DPR_BYTE	Reserved3[RES3_LEN];    /* Reserved	                       */

    /* data length, version infos, other */
	DPR_WORD	DpOrderId;		        /* DPV1 orderid	                   */
	DPR_DWORD	HwVersion;		        /* Hardware version				   */
	DPR_DWORD	FwVersion;		        /* Firmware version				   */
	DPR_WORD	DatLen;			        /* Data length					   */
	DPR_BYTE	DatBuf[MAX_BUFF_LEN];	/* Data buffer					   */


  } GNUC__PACK CI_DP_ORDER_T;               



  typedef struct DP_USER_ID_TABLE_S
  {
    DPR_WORD      Used;
    DPR_WORD      CpIndex;
    DPR_DWORD     DpSyncHandle;
    DPR_DWORD     DpAsyncHandle;
    DPR_WORD      DpEnableEvtActiv;
    DPR_WORD      DpAlarmAckActiv   [DPR_MAX_SLAVE_NR];
    DPR_WORD      DpDsWriteReadActiv[DPR_MAX_SLAVE_NR];
  } GNUC__PACK DP_USER_ID_TABLE_T;


/* ----------------------------------
   Definition of base functions via CI
   ----------------------------------- */
#define CI_DP_SET_MODE          0x0000
#define CI_DP_SLV_STATE         0x0001
#define CI_DP_READ_SLV_PAR      0x0002
#define CI_DP_GLOBAL_CTRL       0x0003
#define CI_DP_DS_READ           0x0004
#define CI_DP_DS_WRITE          0x0005
#define CI_DP_ALARM_ACK         0x0006 
#define CI_DP_ENABLE_EVENT      0x0007
#define CI_DP_GET_CFG           0x0008
#define CI_DP_USR_OPEN          0x0009
#define CI_DP_USR_CLOSE         0x000a
#define CI_DP_READ_ALARM        0x000b
#define CI_DP_DISABLE_EVENT     0x000c
#define CI_DP_WATCHDOG          0x000d

#define CI_DP_REGISTER_SLV      0x000e  // fuer Konverter

/* ----------------------------------
   structure for accessing CI
   ----------------------------------- */
typedef struct CI_DP_REQUEST_S
{
  CI_REQUEST_HEADER_T     header;
  DPR_BYTE                data[CI_BLOCK_SIZE]; 
} GNUC__PACK CI_DP_REQUEST_T;


/* Subkey under HKEY_LOCAL_MACHINE
   =============================== */
#define DP_BASE_TRC_REGISTRY        "SOFTWARE\\SIEMENS\\SINEC\\CP561X"

/* value in registry
   ================= */
#define DP_BASE_TRC_TARGET          "DP_BASE_TRC"
#define DP_BASE_TRC_TEXT            "DP_BASE_TXT"
#define DP_BASE_TRC_TITLE_TEXT      "DP_BASE_TITLE_TXT"


#ifndef DONT_USE_MS_PACK
 #pragma pack()
#endif



/***************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                       */
/*    All Rights Reserved.                                                 */
/***************************************************************************/

