/****************************************************************************/
/*    Copyright (C) SIEMENS AG 1989..1998 All Rights Reserved.Confidential  */
/****************************************************************************/
/*                                                                          */
/*     Modulname     : DP_5613.H                                            */
/*                                                                          */
/****************************************************************************/
/****************************************************************************/
/* Task / Description:                                                      */
/*  - provisional structure of the Dual Ported Ram of the CP5613            */
/*                                                                          */
/****************************************************************************/
/*   Modification                                                           */
/*                                                                          */
/*   Date       Person       Modification                                   */
/*   11.03.98   Me           file created                                   */
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

#ifndef __DP_5613_H
 #define __DP_5613_H

#ifdef __GNUC__
#define GNUC__PACK __attribute__ ((packed))
#else
#define GNUC__PACK
#endif

typedef unsigned short  DPR_WORD;
typedef unsigned long   DPR_DWORD;
typedef long            DPR_LONG;
typedef unsigned char   DPR_BYTE;
typedef signed short    DPR_INT16;
typedef signed long     DPR_INT32;
typedef signed char     DPR_INT08;
typedef char            DPR_STRING;


#ifdef USED_IN_FIRMWARE
 #define DP_CODE_ATTR
 #define DP_MEM_ATTR
#else
 #define DP_CODE_ATTR   __stdcall
 #define DP_MEM_ATTR
#endif

/****************************************************************************/
/******** Structure of the DP specific Dual Ported Ram of the CP5613     ****/
/****************************************************************************/


#define DPR_SLAVE_DATA_SIZE      246U
#define DPR_MAX_SLAVE_ADDR       127U
#define DPR_MAX_SLAVE_NR         (DPR_MAX_SLAVE_ADDR + 1)

/* slave state */
/* ----------- */

#define DPR_SLV_NOT_READY        0x0000U
#define DPR_SLV_READY            0x0001U


/* Fast Logic  */
/* ----------- */
#define DPR_MAX_FASTLOGIC_CNT    0x0004U

/* consistency */
/* ----------- */

#define DPR_DP_UNLOCK                0xFFFF


//--------------------------------------------------------------------------------


/* additional information about a slave */
typedef struct DPR_SLAVE_INFO_S
{
 DPR_WORD    slave_in_database;  /* 0 -> not configured 1 -> configured  */
 DPR_WORD    slave_type;
 DPR_WORD    slave_out_byte;
 DPR_WORD    slave_in_byte;
 DPR_WORD    slave_state;
 DPR_WORD    detail;             /* internal reserved */
 DPR_WORD    alarm_flag;
 DPR_WORD    reserved_w_00;
} GNUC__PACK DPR_SLAVE_INFO_T;

/* additional information about the master  */
typedef struct DPR_MASTER_INFO_S
{
 DPR_WORD    USIF_state;   /* actual state of the master */
 DPR_WORD    ident_number;
 DPR_WORD    hw_version;
 DPR_WORD    fw_version;
 DPR_WORD    reserved_w_00[0x04];
} GNUC__PACK DPR_MASTER_INFO_T;




/* alle elements are counters*/
typedef struct DPR_ASPC2_EVENT_S
 {
  DPR_WORD     reserved_tick_5ms;
  DPR_WORD     on_double_token;
  DPR_WORD     on_timeout;
  DPR_WORD     on_syni_error;
  DPR_WORD     on_hsa_error;
  DPR_WORD     on_response_error;
  DPR_WORD     on_las_useless;
  DPR_WORD     on_rec_frame_overflow;
  DPR_WORD     on_fifo_error;
  DPR_WORD     on_req_length_error;
  DPR_WORD     off_pass_token_error;
  DPR_WORD     off_ts_adr_error;
  DPR_WORD     off_hsa_error;

  DPR_WORD     in_ring;    
  DPR_WORD     out_of_ring;    
  DPR_WORD     bus_control_error;   /* bus short cut */

 } GNUC__PACK DPR_ASPC2_EVENT_T;



#define DP_M_BAUDRATE_9K6    0
#define DP_M_BAUDRATE_19K2   1
#define DP_M_BAUDRATE_93K75  2
#define DP_M_BAUDRATE_187K5  3
#define DP_M_BAUDRATE_500K   4
#define DP_M_BAUDRATE_750K   5
#define DP_M_BAUDRATE_1M5    6
#define DP_M_BAUDRATE_3M     7
#define DP_M_BAUDRATE_6M     8
#define DP_M_BAUDRATE_12M    9
#define DP_M_BAUDRATE_31K25  10
#define DP_M_BAUDRATE_45K45  11

/* watchdog structure */
typedef struct DPR_WD_S
{
  DPR_WORD     wd_state;
  DPR_WORD     wd_start;
  DPR_WORD     wd_counter;
  DPR_WORD     wd_trigger;
  DPR_WORD     reserved[4];
} GNUC__PACK DPR_WD_T;



typedef struct DPR_ASPC2_BUSPARA_S
 {
		DPR_BYTE			ts;
		DPR_BYTE			baud_rate;			   
		DPR_WORD			tsl;
        DPR_WORD            min_tsdr;
        DPR_WORD            max_tsdr;
		DPR_BYTE			tqui;
		DPR_BYTE			tset;
		DPR_DWORD			ttr;
		DPR_BYTE			g;
		DPR_BYTE			hsa;
		DPR_BYTE			max_retry_limit;
		DPR_BYTE			station_type;
		DPR_WORD			trdy;            

        /* Details */
		DPR_WORD			tid1;
		DPR_WORD			tid2;
		DPR_DWORD	    	tmsi;
		DPR_WORD			tmsi_reserve;
		DPR_WORD			tbus_control_in_ring;
		DPR_WORD			tbus_control_out_of_ring;  
		DPR_WORD			acyc_req_ctr;
		DPR_BYTE			mode_clock_sync;	
		DPR_BYTE			delay_time_ctr_clock_sync;
		DPR_BYTE			mode_equ_dis;				
		DPR_BYTE			master_equ_dis;        

        /* DP specific */
		DPR_BYTE			BpFlag;
		DPR_WORD            MinSlaveInterval;
		DPR_WORD            PollTimeout;
		DPR_WORD            DataControlTime;

        DPR_BYTE            reserved_b_00[0x01];
 } GNUC__PACK DPR_ASPC2_BUSPARA_T;

typedef struct DPR_CP5613_PCI_DATA_S
 {
  DPR_DWORD          PhysAddr_LowPart;
  DPR_LONG           PhysAddr_HighPart;
  DPR_DWORD          OriginalIrql;
  DPR_DWORD          OriginalVector;
  DPR_DWORD          InterruptMode;
  DPR_DWORD          SlotNumber;
  DPR_DWORD          BusNumber;
  DPR_DWORD          reserved_dw[5];
 } DPR_CP5613_PCI_DATA_T;



typedef struct DPR_CP5613_CTR_S
{
 /* Mask to enable the DP cycle start interrupt feature !
    This feature is only available if the CP is used in the
    equidistance mode !!
    C_en_dp_cycle_start_int is automatically masked after the interrupt
    has occured.
    To enable the interrupt all the time the user has to enable the mask after
    every interrupt. The user has to do this to avoid corrupting
    his system with to much interrupts per time. */
 DPR_WORD   D_cycle_start_mask; /* 0x00078000 */
                                /* 0x00F0 -> enable , 0x00F1 -> disabled */
 DPR_BYTE   no_ram_here_reserved_b_00[0x7FFE];
 /*---------------------------------------------------------------------------*/
 /* Control DP */
 /* Control area for interrupts and consistency */
 /* Control to manage consistency for the input data of a slave. */
 /* See also =>  PROCESS IMAGE  -> CONSISTENCY MECHANISM         */
 DPR_WORD   D_lock_in_slave_adr;  /* 0x00080000 */
                                  /* slave address for consistent input data */
 DPR_BYTE   no_ram_here_reserved_b_01[0x7FFE];

 /* Control to manage consistency for the output data of a slave.*/
 /* See also =>  PROCESS IMAGE  -> CONSISTENCY MECHANISM         */
 DPR_WORD   D_out_slave_adr;  /* 0x00088000 */
                               /* slave address for consistent output data */
 DPR_BYTE   no_ram_here_reserved_b_02[0x7FFE];

 /* Control to manage consistency for the diagnosis data of a slave. */
 /* See also =>  PROCESS IMAGE  -> CONSISTENCY MECHANISM         */
 DPR_WORD   D_lock_diag_slave_adr; /* 0x00090000 */
                                /* slave address for consistent diagnosis data */
 DPR_BYTE   no_ram_here_reserved_b_03[0x7FFE];

 DPR_BYTE   no_ram_here_reserved_b_04[0x8000];   /* 0x00098000 */


} GNUC__PACK DPR_CP5613_CTR_T;

/* Process image for one slave */
typedef struct DPR_SLAVE_IN_S
{
 DPR_BYTE       data[DPR_SLAVE_DATA_SIZE];
 DPR_BYTE       reserved_b_00[0x0A];
} GNUC__PACK DPR_SLAVE_IN_T;

typedef struct DPR_SLAVE_OUT_S
{
 DPR_BYTE       data[DPR_SLAVE_DATA_SIZE];
 DPR_BYTE       reserved_b_00[0x0A];
} GNUC__PACK DPR_SLAVE_OUT_T;
                  
typedef struct DPR_SLAVE_DIAG_S
{
 DPR_BYTE       data[DPR_SLAVE_DATA_SIZE];
 DPR_WORD       diag_len;
 DPR_WORD       diag_count;         /* every diag event increments diag_count */
 DPR_WORD       reserved_w_00[0x03];
} GNUC__PACK DPR_SLAVE_DIAG_T;



typedef struct DPR_CP5613_PI_S
{
 /*---------------------------------------------------------------------------*/
 /* Process Image   */
 DPR_SLAVE_IN_T         slave_in[DPR_MAX_SLAVE_NR];   /* 0x000C0000  */
 DPR_SLAVE_OUT_T        slave_out[DPR_MAX_SLAVE_NR];  /* 0x000C8000  */
 DPR_SLAVE_DIAG_T       slave_diag[DPR_MAX_SLAVE_NR]; /* 0x000D0000  */

 /* internal trace of the CP 5613 -> read only area */
 DPR_BYTE               ro_trace[0x8000];     /* 0x000D8000 */
} GNUC__PACK DPR_CP5613_PI_T;



/* The user can use this feature to avoid polling the input data of the process image.
   If input_data there is an interrupt triggered if the input data of the slave has
   changed.
   To avoid too much interrupts per time input_data is automatically masked
   by the CP after an interrupt has occured.
   To enable the  data change interrupt all the time the user has to enable
   req_mask after every interrupt for the slave. */

#define DPR_DATA_INT_CLEAR_AND_UNMASK  0x00 /* clear request and enable interrupt  */
#define DPR_DATA_INT_CLEAR_AND_MASK    0x0F /* clear request and do not enable
                                           interrupt        */
#define DPR_DATA_CHANGE                0xFF /* mask to indicate a change */

typedef struct DPR_MASK_DATA_INT_S
{
  DPR_BYTE       req_mask;  /* see -> DATA_INT... defines */
  DPR_BYTE       no_ram_here_dont_touch[0x00FF];
} GNUC__PACK DPR_MASK_DATA_INT_T;



typedef struct DPR_CP5613_EF_S
{
 /*---------------------------------------------------------------------------*/
 /* Events and Filter */
 DPR_MASK_DATA_INT_T          input[DPR_MAX_SLAVE_NR]; /* 0x000A0000  */
 DPR_BYTE                     reserved_b_00[0x8000];
 DPR_MASK_DATA_INT_T          diag[DPR_MAX_SLAVE_NR];  /* 0x000B0000  */
 DPR_BYTE                     reserved_b_01[0x8000];   /* - 0x000BFFFF */

} GNUC__PACK DPR_CP5613_EF_T;



typedef struct DPR_SM_DIAG_S
{
 DPR_WORD			request;
 DPR_WORD			response;
 DPR_WORD			len;
 DPR_WORD			state;
 DPR_BYTE			diag_data[238];
 DPR_BYTE			reserved[10];
} GNUC__PACK DPR_SM_DIAG_T;

typedef struct DPR_SLAVEMOD_DATA_S
{
 DPR_SM_DIAG_T			sm_diag;
 DPR_WORD				act_gc;
 DPR_WORD				gc_ctr;
 DPR_WORD				baud_state;
 DPR_WORD				baud_rate;
 DPR_WORD				data_ex_state;
 DPR_BYTE				reserved[246];
} GNUC__PACK DPR_SLAVEMOD_DATA_T;

typedef struct DPR_CP5613_EQU_ERROR_S
{
 DPR_WORD     count;
 DPR_WORD     last_diff_tbit;
 DPR_BYTE     reserved_b_00[0x0C];
} GNUC__PACK DPR_CP5613_EQU_ERROR_T;


typedef struct DPR_CP5613_INFO_WATCH_S
{
 /*---------------------------------------------------------------------------*/
 /* Info -> not used bei the CP or driver !  */
 DPR_SLAVE_INFO_T       slave_info[DPR_MAX_SLAVE_NR]; /* 0x000E0000 -> 0x800 Byte*/

 DPR_MASTER_INFO_T      master_info;  /* 0x000E0800 -> 0x10 Byte */

 DPR_ASPC2_EVENT_T      aspc2_event;  /* 0x000E0810   -> 0x20 Byte */

 DPR_ASPC2_BUSPARA_T    aspc2_buspara; /* 0x000E0830 -> 0x30 Byte */

 DPR_CP5613_PCI_DATA_T  pci;  /* 0x000E0860 -> 0x30 Byte */
 /*---------------------------------------------------------------------------*/
 /* for future use  */
 DPR_WD_T               user_watchdog[0x08]; /* 0x000E0890 */
 DPR_BYTE               reserved_b_00[0xF0];

 DPR_WORD               activated_fast_logic[DPR_MAX_FASTLOGIC_CNT]; /* 0xFFFF -> triggered */

 DPR_SLAVEMOD_DATA_T    slavemod_data;         /* 0x000E0898 bis 0x000E0BFF*/
 DPR_CP5613_EQU_ERROR_T equ_error;
 DPR_BYTE               reserved_b_10[0x3E8];  /* 0x000E0C00 bis 0x000E0FFF*/

} GNUC__PACK DPR_CP5613_INFO_WATCH_T;




/* DP specific structure of the Dual Ported Ram  */
typedef struct DPR_CP5613_DP_S
{
 DPR_CP5613_CTR_T         ctr; /* 0x00078000 - 0x0009FFFF */

 DPR_CP5613_EF_T          ef;  /* 0x000A0000 - 0x000BFFFF */

 DPR_CP5613_PI_T          pi;  /* 0x000C0000 - 0x000DFFFF */

 DPR_CP5613_INFO_WATCH_T  info_watch; /* 0x000E0000 - 0x000E0FFF */

} GNUC__PACK DPR_CP5613_DP_T;




/*=============================*/
/* DP REQUEST BLOCK INTERFACE  */
/*=============================*/

/* in  = request parameter  */
/* out = return parameter   */


/* error structure of the DP-function calls   */
/* ========================================   */

typedef struct DP_ERROR
{
  DPR_DWORD          error_class;  /* error class (= return value of the functions)*/
  DPR_DWORD          error_code;   /* error detail */
  DPR_BYTE           error_decode; /* DPC1-Decode  */
  DPR_BYTE           error_code_1; /* DPC1-Code_1  */
  DPR_BYTE           error_code_2; /* DPC1-Code_2  */
} GNUC__PACK DP_ERROR_T;



/* max data size for DPC1 requests */
/*-------------------------------- */

#define DPR_DPC1_MAX_LENGTH      240

/* max data size for alarm/status  */
/*-------------------------------  */

#define DP_ALARM_SIZE             64


/* structure for DP_ds_read  */
/* ========================  */

typedef struct DP_DS_READ_S
{
  DPR_BYTE           slot_number;                  /* in */
  DPR_BYTE           index;                        /* in */
  DPR_BYTE           length_s;                     /* in out*/
  DPR_BYTE           data_s[DPR_DPC1_MAX_LENGTH];   /* out*/
} GNUC__PACK DP_DS_READ_T;


/* structure for DP_ds_write */
/* ========================= */

typedef struct DP_DS_WRITE_S
{
  DPR_BYTE           slot_number;                  /* in */
  DPR_BYTE           index;                        /* in */
  DPR_BYTE           length_m;                     /* in */
  DPR_BYTE           data_m[DPR_DPC1_MAX_LENGTH];  /* in */
} GNUC__PACK DP_DS_WRITE_T;





/* structure for DP_alarm_ack */
/* ========================== */

typedef struct DP_ALARM_ACK_S
{
  DPR_BYTE           slot_number;                  /* in  */
  DPR_BYTE           alarm_type;                   /* in  */
  DPR_BYTE           specifier;                    /* in  */
} GNUC__PACK DP_ALARM_ACK_T;


/* structure for DP_enable_event */
/* ============================= */

typedef struct DP_ENABLE_EVT_S
{
  DPR_DWORD          selector;                     /* in   */
  DPR_BYTE           mst_state;                    /* in   */
  DPR_BYTE           event[DPR_MAX_SLAVE_ADDR];      /* out  */
  DPR_BYTE           mst_event;                    /* out  */
  DPR_BYTE           new_mst_state;                /* out  */
} GNUC__PACK DP_ENABLE_EVT_T;


/* structure for DP_get_actual_cfg */
/* =============================== */

typedef struct DP_GET_CFG_S
{
  DPR_BYTE           length_s;                     /* out  */
  DPR_BYTE           data_s [DPR_SLAVE_DATA_SIZE]; /* out  */
} GNUC__PACK DP_GET_CFG_T;


typedef struct DPC1_REQ_S
{
  DPR_WORD         order_id;                       /* in */
  DPR_DWORD        c_ref;                          /* in */
  union
  {
    DP_DS_READ_T    dp_ds_read;
    DP_DS_WRITE_T   dp_ds_write;
    DP_ALARM_ACK_T  dp_alarm_ack;
    DP_ENABLE_EVT_T dp_enable_evt;
    DP_GET_CFG_T    dp_get_cfg;
  }                 req;
} GNUC__PACK DPC1_REQ_T;


typedef struct DP_ALARM_T
{
  DPR_WORD         msg;                            /* out */
  DPR_WORD         slv_add;                        /* out */
  DPR_BYTE         slot_number;                    /* out */
  DPR_BYTE         alarm_type;                     /* out */
  DPR_BYTE         specifier;                      /* out */
  DPR_BYTE         length_s;                       /* out */ 
  DPR_BYTE         data_s[DP_ALARM_SIZE];          /* out */
} GNUC__PACK DP_ALARM_T;


/* ================================== */
/* Defines for DP-interface functions */
/* ================================== */

/* data length */

#define DP_MAX_DATA_LEN         0x100

/* parameter mode in DP_set_mode und DP_get_mode  */

#define DP_OFFLINE              0x0000
#define DP_STOP                 0x0001
#define DP_CLEAR                0x0002
#define DP_AUTOCLEAR            0x0003
#define DP_OPERATE              0x0004


/* parameter type in DP_read_slv_par  */

#define DP_SLV_TYP              0x0000  /* Sl-flag,Typ,Octett-String */
#define DP_SLV_PRM              0x0001  /* parameter  data           */
#define DP_SLV_CFG              0x0002  /* configuration             */
#define DP_SLV_ADD_TAB          0x0003  /* Add-Tab                   */
#define DP_SLV_USR              0x0004  /* User                      */


/* parameter slv_mode in DP_slv_state  */

#define DP_SLV_DEACTIVATE       0x0001
#define DP_SLV_ACTIVATE         0x0002
#define DP_AUTOCLEAR_ON         0x0004
#define DP_AUTOCLEAR_OFF        0x0008

/* group command for global control */

#define DP_SYNC                   0x20
#define DP_UNSYNC                 0x10
#define DP_FREEZE                 0x08
#define DP_UNFREEZE               0x04


/* address for a group of DP-slaves */

#define DP_BROADCAST_ADR           127

/* address for non specific slave at DP_read_alarm */

#define DP_NEXT_ALARM             0xff


/* size of the data buffer in DP_read_slv_par  */

#define DP_PAR_SIZE             0x00ff

/* slave_type */

#define DP_SLV_TYP_EMPTY        0x0000
#define DP_SLV_TYP_DP           0x0001
#define DP_SLV_TYP_DPV1         0x0002

/* parameter req_type in DP_get_result  */

#define DP_NO_CNF               0x0000
#define DP_DS_READ              0x0001
#define DP_DS_WRITE             0x0002
#define DP_ALARM_ACK            0x0003
#define DP_ENABLE_EVENT         0x0004
#define DP_GET_CFG              0x0005

/* parameter selector in DP_enable_event  */

#define DP_DIAG_ALARM           0x0003
#define DP_SLAVE_STATE          0x000c
#define DP_MST_STATE            0x0010

/* return values for event */

#define DP_SLAVE_ENTER          0x0004
#define DP_SLAVE_EXIT           0x0008
#define DP_DIAG                 0x0001
#define DP_ALARM_STATUS         0x0002

/* return values for mst_event */
#define DP_MST_STATE_CHG        0x0010



/* types of alarm */

#define DP_MSG_NONE             0x0000
#define DP_MSG_ALARM            0x0001
#define DP_MSG_STATUS           0x0002

/* parameter text_len in DP_get_err_txt */

#define DP_ERR_TXT_SIZE  1024


/* defines for watchdog */
#define DP_WD_STOPPED           0x0000
#define DP_WD_STARTED           0x0001
#define DP_WD_TIMEOUT           0x0002


#ifdef WIN32
 #ifdef __cplusplus	
 extern "C" {
 #endif
#endif
/* ====================== */
/* DP interface functions */
/* ====================== */

/* in  = request parameter */
/* out = return  parameter */


/* Download of the firmware and the DP-database  */

DPR_DWORD DP_CODE_ATTR DP_start_cp( DPR_STRING   DP_MEM_ATTR  *cp_name,   /* in */
                                    DPR_STRING   DP_MEM_ATTR  *database,  /* in */
                                    DP_ERROR_T   DP_MEM_ATTR  *error );   /* out*/

/* Stop of the CP-firmware  */

DPR_DWORD DP_CODE_ATTR DP_reset_cp ( DPR_STRING  DP_MEM_ATTR   *cp_name,   /* in  */
                                     DP_ERROR_T  DP_MEM_ATTR   *error );   /* out */

/* registration of an application at the DP interface  */

DPR_DWORD DP_CODE_ATTR DP_open ( DPR_STRING      DP_MEM_ATTR    *cp_name,      /* in  */
                                 DPR_DWORD       DP_MEM_ATTR    *user_handle,  /* out */
                                 DP_ERROR_T      DP_MEM_ATTR    *error    );   /* out */


/* getting pointer for access to Dual Port Ram */

DPR_DWORD DP_CODE_ATTR DP_get_pointer (DPR_DWORD                       user_handle,  /* in  */
                                       DPR_DWORD                       timeout,      /* in  */ 
                                       DPR_CP5613_DP_T volatile  DP_MEM_ATTR  **dpr, /* out */
                                       DP_ERROR_T      DP_MEM_ATTR    *error    );   /* out */


/* release of Dual Port Ram pointer */

DPR_DWORD DP_CODE_ATTR DP_release_pointer (DPR_DWORD                  user_handle,  /* in  */
                                           DP_ERROR_T  DP_MEM_ATTR    *error    );  /* out */


/* removing an application off the DP interface */

DPR_DWORD DP_CODE_ATTR DP_close ( DPR_DWORD                 user_handle,  /* in   */
                                  DP_ERROR_T   DP_MEM_ATTR  *error );     /* out  */


/* get error text */

DPR_DWORD DP_CODE_ATTR DP_get_err_txt ( DP_ERROR_T   DP_MEM_ATTR  *error,      /* in     */
                                        DPR_STRING   DP_MEM_ATTR  *language,   /* in     */
                                        DPR_STRING   text[DP_ERR_TXT_SIZE] );  /* out    */

/* set master mode of DP  */

DPR_DWORD DP_CODE_ATTR  DP_set_mode ( DPR_DWORD                 user_handle, /* in  */
                                      DPR_WORD                  mst_mode,    /* in  */
                                      DP_ERROR_T   DP_MEM_ATTR  *error );    /* out */



/* activation/deactivation of a slave */

DPR_DWORD DP_CODE_ATTR  DP_slv_state ( DPR_DWORD                user_handle,  /* in  */
                                       DPR_WORD                 slv_add,      /* in  */
                                       DPR_WORD                 slv_mode,     /* in  */
                                       DP_ERROR_T   DP_MEM_ATTR  *error );    /* out */



/* get slave parameter of the database */

DPR_DWORD DP_CODE_ATTR  DP_read_slv_par ( DPR_DWORD                user_handle, /* in  */
                                          DPR_WORD                 slv_add,     /* in  */
                                          DPR_WORD                 type,        /* in  */
                                          DPR_WORD    DP_MEM_ATTR  *data_len,   /* in out */
                                          DPR_BYTE    DP_MEM_ATTR  *data,       /* out */
                                          DP_ERROR_T  DP_MEM_ATTR  *error  );   /* out */



/* sending of a global control command  */

DPR_DWORD DP_CODE_ATTR DP_global_ctrl  ( DPR_DWORD               user_handle, /* in  */
                                         DPR_WORD                slv_add,     /* in  */
                                         DPR_BYTE                command,     /* in  */
                                         DPR_BYTE                group,
                                         DP_ERROR_T DP_MEM_ATTR  *error  );   /* out */


/* read DPV1 data set  */

DPR_DWORD DP_CODE_ATTR DP_ds_read( DPR_DWORD                 user_handle,  /* in  */
                                   DPC1_REQ_T   DP_MEM_ATTR  *request,     /* in  */
                                   DP_ERROR_T   DP_MEM_ATTR  *error );     /* out */

/* write DPV1 data set */

DPR_DWORD DP_CODE_ATTR  DP_ds_write ( DPR_DWORD                user_handle,  /* in  */
                                      DPC1_REQ_T   DP_MEM_ATTR *request,     /* in  */
                                      DP_ERROR_T   DP_MEM_ATTR *error );     /* out */

/* alarm acknowledge  */

DPR_DWORD DP_CODE_ATTR  DP_alarm_ack ( DPR_DWORD                 user_handle, /* in  */
                                       DPC1_REQ_T    DP_MEM_ATTR *request,    /* in  */
                                       DP_ERROR_T    DP_MEM_ATTR *error );    /* out */


/* wait for event */

DPR_DWORD DP_CODE_ATTR DP_enable_event ( DPR_DWORD                user_handle, /* in   */
                                         DPC1_REQ_T  DP_MEM_ATTR  *request,    /* in   */
                                         DP_ERROR_T  DP_MEM_ATTR  *error );    /* out  */

/* cancels a current "DP_enable_event"-request */

DPR_DWORD DP_CODE_ATTR DP_disable_event( DPR_DWORD                user_handle, /* in   */
                                         DP_ERROR_T  DP_MEM_ATTR  *error );    /* out  */



/* get actual configuration  */

DPR_DWORD DP_CODE_ATTR DP_get_actual_cfg(DPR_DWORD                user_handle, /* in  */
                                         DPC1_REQ_T  DP_MEM_ATTR  *request,    /* in  */
                                         DP_ERROR_T  DP_MEM_ATTR  *error );    /* out */

/* get alarm/status data */

DPR_DWORD  DP_CODE_ATTR DP_read_alarm ( DPR_DWORD                user_handle, /* in  */
                                        DPR_WORD                 slv_add,     /* in  */
                                        DP_ALARM_T  DP_MEM_ATTR  *alarm,      /* out */
                                        DP_ERROR_T  DP_MEM_ATTR  *error);     /* out */

/* get Parameter c_ref  */

DPR_DWORD  DP_CODE_ATTR DP_get_cref   ( DPR_DWORD                user_handle, /* in  */
                                        DPR_WORD                 slv_add,     /* in  */
                                        DPR_DWORD  DP_MEM_ATTR   *c_ref,      /* out */
                                        DP_ERROR_T DP_MEM_ATTR   *error);     /* out */


/* get an asynchronous DPC1-event */
#define DP_TIMEOUT_FOREVER     0x7FFFFFFFUL

DPR_DWORD DP_CODE_ATTR DP_get_result  ( DPR_DWORD                user_handle,  /* in  */
                                        DPR_DWORD                timeout,      /* in  */
                                        DPR_WORD    DP_MEM_ATTR  *req_type,    /* out */
                                        DPC1_REQ_T  DP_MEM_ATTR  *result,      /* out */
                                        DP_ERROR_T  DP_MEM_ATTR  *error );     /* out */



#define DP_OBJECT_TYPE_INPUT_CHANGE   0x00000001UL
#define DP_OBJECT_TYPE_DIAG_CHANGE    0x00000002UL
#define DP_OBJECT_TYPE_CYCLE_INT      0x00000003UL
#define DP_OBJECT_TYPE_FAST_LOGIC     0x00000004UL
#define DP_OBJECT_TYPE_ASYNC          0x00000005UL


/* This function creates an object ( semaphore ). The user application can wait at  */
/* thís semaphore in an own thread using the ADPR calls                             */
/* WaitForSingleObject /WaitForMultipleObjects.                                     */
DPR_DWORD DP_CODE_ATTR  DP_init_sema_object (DPR_DWORD              user_handle,    /* in */
                                             DPR_DWORD              sema_type,      /* in */
                                             DPR_DWORD  DP_MEM_ATTR *sema_handle,   /* out */
                                             DP_ERROR_T DP_MEM_ATTR *error);        /* out */


DPR_DWORD DP_CODE_ATTR  DP_delete_sema_object ( DPR_DWORD              user_handle, /* in */
                                                DPR_DWORD              sema_handle, /* out*/
                                                DP_ERROR_T DP_MEM_ATTR *error);     /* out */


DPR_DWORD DP_CODE_ATTR  DP_watchdog           ( DPR_DWORD              user_handle, /* in  */
                                                DPR_DWORD              timeout,     /* in  */
                                                DPR_WORD DP_MEM_ATTR * wd_index,    /* out */
                                                DP_ERROR_T DP_MEM_ATTR *error);     /* out */


/************** Fast Logic ********************************************************/


#define DP_FAST_LOGIC_ID_0        0UL
#define DP_FAST_LOGIC_ID_1        1UL
#define DP_FAST_LOGIC_ID_2        2UL
#define DP_FAST_LOGIC_ID_3        3UL


typedef struct DP_FAST_LOGIC_S
{
 DPR_BYTE   slave_addr_in_byte;  /* -> Slave address */
 DPR_BYTE   index_in_byte;           /* -> Input Byte */
 DPR_BYTE   cmp_value_in_byte; /* -> Compare value  */
 DPR_BYTE   mask_in_byte;       /* -> Bit = 1 -> masked */

 DPR_BYTE   slave_addr_out_byte; /* -> Slave address */
 DPR_BYTE   index_out_byte;       /* -> Output Byte */
 DPR_BYTE   value_out_byte;    /* -> out value  */
 DPR_BYTE   mask_out_byte;     /* -> Bit = 1 -> Bit masked */

} GNUC__PACK DP_FAST_LOGIC_T;


#define DP_FASTLOGIC_CLEAR       0x0000
#define DP_FASTLOGIC_ACTIVATED   0x8888
#define DP_FASTLOGIC_TRIGGERED   0xFFFF

DPR_DWORD DP_CODE_ATTR  DP_fast_logic_on( DPR_DWORD                    user_handle,
                                          DPR_WORD                     fast_logic_id,
                                          DP_FAST_LOGIC_T DP_MEM_ATTR  *fast_logic,
                                          DP_ERROR_T DP_MEM_ATTR       *error );     /* out */

DPR_DWORD DP_CODE_ATTR  DP_fast_logic_off( DPR_DWORD              user_handle,
                                           DPR_WORD               fast_logic_id,
                                           DP_ERROR_T DP_MEM_ATTR *error );     /* out */
/**********************************************************************************/






#ifdef WIN32
 #ifdef __cplusplus	
  }
 #endif
#endif

/* Defines for error class */
/* ======================= */

#define DP_OK                0x0000   /* Request completed, confirmation available */
#define DP_OK_ASYNC          0x0001   /* Request activated, confirmation not available */
#define DP_ERROR_EVENT       0x0002   /* slave sends in a response telegram  DPC1 error values */
#define DP_ERROR_EVENT_NET   0x0003   /* error at an underlaying driver */
#define DP_ERROR_REQ_PAR     0x0004   /* wrong request parameter   */
#define DP_ERROR_CI          0x0005   /* error at accessing the CP */
                                      /* error_code see ci_ret.h   */
#define DP_ERROR_RES         0x0006   /* not enough ressources     */
#define DP_ERROR_USR_ABORT   0x0007   /* user has finished DP-communication */


/* error codes
   ===========

   see file 5613_ret.h 
*/








#endif



#ifndef DONT_USE_MS_PACK
 #pragma pack()
#endif



/****************************************************************************/
/*     END OF FILE:      DP_5613.H                                          */
/****************************************************************************/
