/****************************************************************************/
/*    Copyright (C) SIEMENS AG 1989..1998 All Rights Reserved.Confidential  */
/****************************************************************************/
/*     Projekt       :                                                      */
/*                                                                          */
/*     Modulname     : CI_5613.H                                            */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
/****************************************************************************/
/* Task / Description:                                                      */
/*  - Communication Interface of the CP5613                                 */
/*                                                                          */
/****************************************************************************/
/*   Modification                                                           */
/*                                                                          */
/*   Date       Person       Modification                                   */
/*   26.02.98   Me           file created                                   */
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

/* 1. Headerfile */
#ifndef DONT_USE_MS_PACK
 #pragma pack(1)
#endif

#ifdef __GNUC__
#define GNUC__PACK __attribute__ ((packed))
#else
#define GNUC__PACK
#endif

#define FW_EXCP_ALL      0xFFFFU
#define FW_EXCP_ASPC2    0x0001U
#define FW_EXCP_HARDWARE 0x0002U






#define CI_MAX_CP                 0x0004U
#define CI_MAX_USER               0x0040U
#define CI_MAX_PATH               260U

#define CI_LOGDEVICE_CP1       "CP5613/5614(1)"
#define CI_LOGDEVICE_CP2       "CP5613/5614(2)"
#define CI_LOGDEVICE_CP3       "CP5613/5614(3)"
#define CI_LOGDEVICE_CP4       "CP5613/5614(4)"



/* Opcodes FASTLOGIC */
#define OPC_FASTLOGIC_ON                0x00090000UL
#define OPC_FASTLOGIC_OFF               0x00090001UL





/* Subsystems  */
#define SUBSYSTEM_CIB                   0x00010000UL
#define SUBSYSTEM_DPT                   0x00020000UL
#define SUBSYSTEM_DPMC                  0x00030000UL
#define SUBSYSTEM_AMPRO                 0x00040000UL
#define SUBSYSTEM_DIAG                  0x00050000UL
#define SUBSYSTEM_DPC31                 0x00060000UL
#define SUBSYSTEM_DRIVER                0x00070000UL
#define SUBSYSTEM_ADMIN                 0x00080000UL
#define SUBSYSTEM_FASTLOGIC             0x00090000UL


/* Opcodes FASTLOGIC */
#define OPC_FASTLOGIC_ON                0x00090000UL
#define OPC_FASTLOGIC_OFF               0x00090001UL



/* Opcodes ADMIN */
#define OPC_ADMIN_OPEN                  0x00080000UL
#define OPC_ADMIN_CLOSE                 0x00080001UL
#define OPC_ADMIN_LED                   0x00080002UL




/* Opcodes Driver */
#define CI_ORDER_ID_EVENT_QUITT      0x00000001UL
#define CI_ORDER_ID_EVENT            0x00000002UL

#define OPC_DRV_EVENT                0x00070001UL
#define OPC_CP0_BUS_EVENT            0x00070002UL
#define OPC_CP1_BUS_EVENT            0x00070003UL
#define OPC_CP2_BUS_EVENT            0x00070004UL
#define OPC_CP3_BUS_EVENT            0x00070005UL
#define OPC_CP0_CP_EVENT             0x00070006UL
#define OPC_CP1_CP_EVENT             0x00070007UL
#define OPC_CP2_CP_EVENT             0x00070008UL
#define OPC_CP3_CP_EVENT             0x00070009UL



/* */
#define RSP_DRIVER_INFO                 0x00000001UL



/* Werte fuer ci_receive_handle */
/* wird nicht mehr unterstuetzt
#define CI_UNSPECIFIC_REQUEST 0xFFFFFFFF
#define CI_ALL_REQUESTS       0xFFFFFFFE   */



/* Anwender Auftragsblock Header */
typedef struct CI_REQUEST_HEADER_S
{

 /* wurde beim open vom Treiber vergebe */
 DPR_DWORD        user_handle;

 DPR_DWORD        reserved_dw_00;

 DPR_DWORD        opcode;
 /* Subsystem in opcode kodiert !! */

 DPR_DWORD        timeout;
 /* nur bei CI_receive notwendig!!   */
 /* Angabe in Sekunden -> 0xFFFF bedeutet unendlich warten.  */
 /*                    -> 0 bedeutet nicht warten.           */

 DPR_DWORD        order_id;
 /* wird unverändert zurückgegeben */

 DPR_DWORD        response;      /* Fehlercode der Firmware */

 DPR_WORD        buf_length;
 /* Länge des Anwenderpuffers  */

 DPR_WORD        buf_fill_length;
 /* Beim CI_send muß der Anwender die Anzahl der gültigen Daten in seinem Puffer */
 /* eintragen.                                                                   */
 /* Beim CI_receive steht hier die Anzahl der empfangenen Daten                  */

 /* ab hier reserviert fuer den Treiber  */
 DPR_DWORD   alloc_index; /* Umlaufzaehler fuer Allokierungshistorie -> nur Treiber
                             schreibt -> wg. Analyse
                             0xFFFFFFFF -> Initialisierungswert
                             0 - 0x7FFFFFFF Wertebereich       */
 DPR_DWORD   stamp_send;     /* 5ms Ticks cp_life_counter */
 DPR_DWORD   stamp_receive;  /* 5ms Ticks cp_life_counter */

 // JM DPR_DWORD   reserved_00[0x02];

} GNUC__PACK CI_REQUEST_HEADER_T;





/* allgemeiner Anwenderrequestblock  */

typedef struct CI_REQUEST_S
{
 CI_REQUEST_HEADER_T     header;
 DPR_BYTE                data[1]; /* auftragsspezifische Puffer mit variabler Laenge*/

} GNUC__PACK CI_REQUEST_T;

/*============================================================================*/
/*============================ DPR ===========================================*/
/*============================================================================*/
#define CI_BLOCK_SIZE    0x0154U  // 340 Byte (80 Byte SCI + 260 Byte Daten für FDL!)
#define CI_BLOCK_COUNT   0x0132U  // 274 Puffer
#define CI_BLOCK_MAX_INDEX   (CI_BLOCK_COUNT - 1UL)
#define CI_BLOCK_RESERVE     0x0004


#define CI_NIL               0xFFFFU

#define CI_SEARCH_PAT_FW   0xABCDU
#define CI_SEARCH_PAT_1    0x4848U
#define CI_SEARCH_PAT_DRV  0xD123U

typedef struct CI_DATA_BUFFER_S
{
 CI_REQUEST_HEADER_T    usr_header;    /* 0x30 Byte         */
 DPR_WORD               next_block;    /* index eines verketteten Blocks     */
 DPR_WORD               next_request;  /* index eines verketteten Requests   */
 DPR_WORD               block_fill_length;  /* Gueltige Daten im Block  */
 DPR_WORD               own_index;
 DPR_WORD               search_pat_0; /* wird von der FW im Anlauf einmal */
 DPR_DWORD              res_for_subsys; /* vom Subsystem frei verwendbar */
 DPR_WORD               reserved[1];         /* 0x02  */
 DPR_BYTE               data[CI_BLOCK_SIZE];
} GNUC__PACK CI_DATA_BUFFER_T;   /* 0x18C      */



#define  ADM_ERR_INFO_LEN    22U
#define  ADM_ERR_BUF_LEN     0x200U
typedef struct CP_ADM_ERR_S
{
 DPR_WORD               code;
 DPR_STRING             info[ADM_ERR_INFO_LEN];
 DPR_DWORD              DWPara1;
 DPR_DWORD              DWPara2;
 DPR_BYTE               buf[ADM_ERR_BUF_LEN];
} GNUC__PACK CP_ADM_ERR_T;


/* ADM   */
#define FW_INT_NO_ACTION    0
#define FW_INT_TRIGGER      1


typedef struct CP_TIME_DATA_S
{
 DPR_DWORD       time;
 DPR_WORD        date;
 DPR_WORD        transmitter_status;
 DPR_WORD        scp_status;
 DPR_BYTE        sync_count;
 DPR_BYTE        send_time;
} GNUC__PACK CP_TIME_DATA_T;


typedef struct CP_DPR_SCI_TIME_S
{
 DPR_WORD         typ;
 DPR_WORD         len;
 DPR_BYTE         host_access;
 DPR_BYTE         board_access;
 CP_TIME_DATA_T   td;
} GNUC__PACK CP_DPR_SCI_TIME_T;

#define SAP_SIZE					66


typedef struct CP_DPR_SAP_S
{
 DPR_BYTE	 prot;        // protocol
 DPR_BYTE	 state;       // state
 DPR_BYTE	 conflict;    // 0 -> no conflict
 DPR_BYTE	 reserved;    // reserved

} GNUC__PACK CP_DPR_SAP_T;




typedef struct ADM_CP_S    /* 1 K     */
{
 DPR_BYTE           fw_version[0x20];

 CP_ADM_ERR_T       fw_error;  // 0x220

 DPR_WORD           to_send_index; // -> 0x240
 DPR_WORD           cp_ready;  /* 1 -> Host tosend_index darf eingetragen werden
                                  0 -> CP bearbeitet gerade den Auftrag -> Auftrag
                                  in die tosendlist und mit der folgenden Aktion
                                  ( neuer Auftrag oder CP Interrupt) abschicken */

 DPR_WORD           host_ready; /* 1 -> es darf eingekettet werden  */
                                /* 0 -> CP MUSS warten bis Host fertig ist */
 DPR_WORD           block_return_head; /* Puffer, die nur zurueckgegeben werden */
 DPR_WORD           data_return_head; /* 0x248 Daten kommen zurueck -> in der User Kette */
                                       /* speichern und warten bis abgeholt wird */
 DPR_WORD           status;
 DPR_WORD           in_ice;
 DPR_WORD           cpu_id;
 DPR_WORD           aspc2_id;
 DPR_DWORD          cp_life_counter; /* wird alle 5ms vom CP inkrementiert */
 DPR_WORD           fw_int_status;
 DPR_WORD           pc_int_status; /* hier steht zu Beginn der PC Interrupt!*/
 DPR_DWORD          cp_perf_counter;

 struct ADM_TEST_S
 {
  DPR_DWORD        trace_adr;
  DPR_DWORD        dest_adr;
  DPR_DWORD        src_adr;
  DPR_DWORD        action;
  DPR_WORD         byte_count;
  DPR_DWORD        time_in_us;
  DPR_BYTE         pattern;
  DPR_BYTE         res_b_00;
  DPR_DWORD        loop;
  DPR_DWORD        res_dw[1];
 } GNUC__PACK test;


 DPR_WORD          fastlogic_req;
 DPR_WORD          dp_access;
 // Uhrzeit
 CP_DPR_SCI_TIME_T uhr;


 CP_DPR_SAP_T      sap[SAP_SIZE];

 DPR_BYTE          reserved_3[0x064];

} GNUC__PACK ADM_CP_T;




/* Testmodus */
/*
 1.Host -> CI_start_cp ( "CP5613", 0, 0, STATUS_INIT_TEST   );
 2.CP   -> dpr->admin_cp.status = STATUS_RUNNING_TEST
 3.Host -> CI_connect_cp( &ptr1, &ptr2 )
 4.Host -> dpr->admin_cp.test.trace_adr = <Adresse des Firmwaretrace>
           dpr->admin_cp.test.adr       = <Startadresse des Speicherbereichs>
           ....
           .....
 5.Host -> dpr->admin_cp.status = STATUS_TEST_START
 6.CP   -> dpr->admin_cp.status = STATUS_TEST_DONE_xxx
 ...
 ...
 n.Host -> CI_disconnect_cp
*/
/* action test mode */
#define ACTION_MEMSET_PATTERN       0x0001
#define ACTION_SHIFT_BIT            0x0002
#define ACTION_START_TIMER_1        0x0003
#define ACTION_START_TIMER_2        0x0004
#define ACTION_STOP_TIMER_1         0x0005
#define ACTION_STOP_TIMER_2         0x0006
#define ACTION_MEMCPY               0x0007
#define ACTION_LED_01_ON            0x0008
#define ACTION_LED_01_OFF           0x0009
#define ACTION_LED_02_ON            0x000A
#define ACTION_LED_02_OFF           0x000B

/* dpr->admin_cp.status -> Test ok, wenn dpr->admin_cp.test.loop > 0 !!*/
#define STATUS_INIT                     0xABCD
#define STATUS_INIT_TEST                0x1234
#define STATUS_INIT_VERSION             0x4455
#define STATUS_INIT_SWITCH              0xEF98
#define STATUS_RUNNING                  0xEEEE
#define STATUS_TEST_START               0x0001
#define STATUS_TEST_STOP                0x0002
#define STATUS_TEST_DONE_ERROR          0x0003
#define STATUS_TEST_DONE_WRONG_ACTION   0x0004
#define STATUS_TEST_DONE_WRONG_STATUS   0x0005


#define ADM_HC_DATA_TAKEN    0
#define ADM_HC_DATA_HERE     1
#define ADM_HC_CIB_EXCP      2

#define ADM_EXCP_CP      0xFFFF
#define ADM_EXCP_CIB     0xEEEE






typedef struct DPR_CP5613_CTR_CPU_S
{
 DPR_WORD   C_res_en;       /* 0x00000000 :  03 -> Reset all
                                             02 -> go 386EX, Reset ASPC2
                                             01 -> Reset 386EX, go ASPC2
                                             00 -> go all                */

 DPR_BYTE   no_ram_here_reserved_b_00[0x3FFE];

 DPR_WORD   C_int_cp_to_host;    /* 0x00004000 -> Datum beliebig */

 DPR_BYTE   no_ram_here_reserved_b_01[0x3FFE];

 DPR_WORD   C_info_cp_int;  /* 0x00008000 -> Host liest Interruptursache vom CP */
                            /* Bit 0: 0 -> DP Zyklusstart Interrupt */
                            /* Bit 1: 0 -> PAE Data Change Interrupt */
                            /* Bit 2: 0 -> PAD Data Change Interrupt */
                            /* Bit 3: 0 -> CP -> Host Interrupt      */


 DPR_BYTE   no_ram_here_reserved_b_02[0x3FFE];

 DPR_WORD   C_ack_dp_cycle_int;  /* 0x0000C000 -> Host quittiert Cycle Int */
                                 /* schreiben beliebiges Datum */

 DPR_BYTE   no_ram_here_reserved_b_03[0x1FFE];

 DPR_WORD   C_ack_pae_int;  /* 0x0000E000 -> Host quittiert PAE Input change */
                            /* schreiben beliebiges Datum */

 DPR_BYTE   no_ram_here_reserved_b_04[0x1FFE];

 DPR_WORD   C_ack_pad_int;  /* 0x00010000 -> Host quittiert PAD Diagnose change */
                            /* schreiben beliebiges Datum */

 DPR_BYTE   no_ram_here_reserved_b_05[0x1FFE];

 DPR_WORD   C_ack_cp_to_host_int;  /* 0x00012000 -> Host quittiert CP Interrupt */
                                   /* schreiben beliebiges Datum */

 DPR_BYTE   no_ram_here_reserved_b_06[0x1FFE];


 DPR_WORD   C_reserved_xx;    /* 0x00014000  */
                             
 DPR_BYTE   no_ram_here_reserved_b_07[0x3FFE];

 DPR_WORD   C_reserved_yy;   /* 0x00018000  */

 DPR_BYTE   no_ram_here_reserved_b_08[0x7FFE];

 DPR_WORD   C_reserved_zz; /* 0x00020000  -> 0 enabled, 1 disabled */

 DPR_BYTE   no_ram_here_reserved_b_09[0x7FFE];

 DPR_WORD   C_res_00; /* 0x00028000   */

 DPR_BYTE   no_ram_here_reserved_b_0a[0x7FFE];

 DPR_WORD   C_res_01; /* 0x00030000  */

 DPR_BYTE   no_ram_here_reserved_b_0b[0x7FFE];

 /* Maske 0 */
 DPR_WORD   C_res_02; /* 0x00038000  */

 DPR_BYTE   no_ram_here_reserved_b_0c[0x7FFE];

 DPR_WORD   C_res_03; /* 0x00040000  */

 DPR_BYTE   no_ram_here_reserved_b_0d[0x7FFE];

 /* Maske 1 */
 DPR_WORD   C_res_04; /* 0x00048000  */

 DPR_BYTE   no_ram_here_reserved_b_0e[0x7FFE];

 DPR_WORD   C_res_05; /* 0x00050000  */

 DPR_BYTE   no_ram_here_reserved_b_0f[0x7FFE];

 /* Maske 2 */
 DPR_WORD   C_res_06; /* 0x00058000  */

 DPR_BYTE   no_ram_here_reserved_b_10[0x7FFE];

 DPR_WORD   C_res_07; /* 0x00060000 */

 DPR_BYTE   no_ram_here_reserved_b_11[0x7FFE];

 /* Maske 3 */
 DPR_WORD   C_res_08; /* 0x00068000  */

 DPR_BYTE   no_ram_here_reserved_b_12[0x7FFE];

 DPR_WORD   C_res_09; /* 0x00070000  */

 DPR_BYTE   no_ram_here_reserved_b_13[0x7FFE];
} GNUC__PACK DPR_CP5613_CTR_CPU_T;




/* Size = 1 MByte */
typedef struct DPR_CP5613_CC_S
{
 ADM_CP_T         adm_cp;             /* 0x000E1000 - 0x000E13FF ->  1 KByte   */

 CI_DATA_BUFFER_T block[CI_BLOCK_COUNT];  /* 0x000E1400 - 0x000FA3FF -> ca.100 KByte */
                                          /* -> 0x0F6 Puffer mit 0x1B8 Byte    */

 DPR_BYTE         reserved_b_01[0x00A8];    /* 0x000FC400 - 0x000FCFFF -> 0x180 Byte */

 DPR_BYTE         trace_driver[0x1200];     /* 0x000FEE00 - 0x000FFFFF -> 12k */

} GNUC__PACK DPR_CP5613_CC_T;




/* Size = 1 MByte */
typedef struct DPR_CP5613_ALL_S
{
 DPR_CP5613_CTR_CPU_T   ctr_cpu; /* 0x00000000 - 0x00077FFF -> Steuerbits */

 /* 0x00078000 -> Hier beginnt die Anwendersicht !!!!  */
 /*=================================================== */
 DPR_CP5613_DP_T     dp;

 /* 0x000E1000 -> Hier endet die Anwendersicht !!!! */
 /*=================================================*/
 DPR_CP5613_CC_T     cc;  /* 0x000E1000 - 0x000FFFFF */

} GNUC__PACK DPR_CP5613_ALL_T;


typedef struct CP5613_PLX_S
{
 DPR_WORD            LAS0_Range_lsw;
 DPR_WORD            LAS0_Range_msw;
 DPR_WORD            LAS1_Range_lsw;
 DPR_WORD            LAS1_Range_msw;
 DPR_WORD            res_w_00[0x06];
 DPR_WORD            LAS0_Base_lsw;
 DPR_WORD            LAS0_Base_msw;
 DPR_WORD            LAS1_Base_lsw;
 DPR_WORD            LAS1_Base_msw;
 DPR_WORD            res_w_01[0x06];
 DPR_WORD            LAS0_Bus_lsw;
 DPR_WORD            LAS0_Bus_msw;
 DPR_WORD            LAS1_Bus_lsw;
 DPR_WORD            LAS1_Bus_msw;
 DPR_WORD            res_w_02[0x06];
 DPR_WORD            CS0_lsw;
 DPR_WORD            CS0_msw;
 DPR_WORD            CS1_lsw;
 DPR_WORD            CS1_msw;
 DPR_WORD            res_w_03[0x04];
 DPR_BYTE            LB_INT_CONTROL_lsw;
 DPR_BYTE            HB_INT_CONTROL_lsw;
 DPR_WORD            INT_CONTROL_msw;
 DPR_BYTE            LB_MULTI_CONTROL_lsw;
 DPR_BYTE            HB_MULTI_CONTROL_lsw;
 DPR_WORD            MULTI_CONTROL_msw;

} GNUC__PACK CP5613_PLX_T;




typedef struct CP5613_EQU_S
{
 DPR_DWORD      mode_equ_dis;
 DPR_DWORD      gc_data_0;
 DPR_DWORD      gc_data_1;
 DPR_DWORD      gc_group;
 DPR_DWORD      tmsi;
 DPR_DWORD      tmsi_reserve;
 DPR_DWORD      ttr_div_256;
 DPR_DWORD      tth_div_256_equ_dis;
 DPR_BYTE       reserved[0x0E0];
} GNUC__PACK CP5613_EQU_T;

#define CLOCK_NONE     0
typedef struct CP5613_ADD_INFO_S
{
 DPR_WORD       soft_entry;
 DPR_WORD       clock;
 DPR_BYTE       reserved[0x00C];
} GNUC__PACK CP5613_ADD_INFO_T;

#define DOWN_DATABASE_SIZE     (0x00010000 - sizeof(CP5613_EQU_T) - sizeof(CP5613_ADD_INFO_T))
#define DOWN_RESET_AREA_SIZE   0x0000000C
#define DOWN_FIRMWARE_SIZE     (0x00060000 - DOWN_RESET_AREA_SIZE)
#define DOWN_MEM_ASPC2_SIZE      0x00080000
#define DOWN_MEM_SCB_SIZE        0x00010000

typedef struct CP5613_DOWNLOAD_S
{
 DPR_BYTE        ampro_scb[DOWN_MEM_SCB_SIZE]; /* 0x00100000 - 0x0010FFFF -> SCB Ampro */
 DPR_BYTE        mem_aspc2[DOWN_MEM_ASPC2_SIZE]; /* 0x00100000 - 0x0019FFFF -> Memory Verwaltung*/
 DPR_BYTE        database_area[DOWN_DATABASE_SIZE];
 CP5613_EQU_T    equ;
 CP5613_ADD_INFO_T add_info;
 DPR_BYTE        firmware[DOWN_FIRMWARE_SIZE];
 DPR_BYTE        reset_area[DOWN_RESET_AREA_SIZE];
} GNUC__PACK CP5613_DOWNLOAD_T;


#ifdef __cplusplus
 extern "C" {
#endif


/* Auftragsfunktionen in CI_BASE_FUNC.DLL -> SCI Light+  */
/*=====================================================  */

/* =========================================  */
/* CI ( Communication Interface ) Funktionen  */
/* =========================================  */

/* Auftragsfunktionen in CI_BASE_FUNC.DLL -> SCI Light+   */
/*=====================================================   */


/* Die Rueckgabewerte dieser Funktionen stehen im File ret_5613.h */




/**********************************************************************************/

/* Beschreibung:
   ============
   - Diese Funktion lädt die Firmware und die Datenbasis auf den CP5613.
   - Wird diese Funktion erfolgreich ausgeführt ist der CP betriebsbereit, d. h
     der CP hat seine Busparameter und ist Profibus Netzteilnehmer.

   Parameter:
   ----------
   cp_name  : Name des CP5613, analog zum PG / PC Panel
   firmware : Name der zu benutzenden Firmware. Wird an dieser Stelle ein NULL
              Pointer übergeben, benutzt die Funktion die konfigurierte Firmware.
   database : Name der zu benutzenden Datenbasis. Wird an dieser Stelle ein NULL
              Pointer übergeben, benutzt die Funktion die konfigurierte Datenbasis.
   mode     : STATUS_INIT        -> Normalanlauf
              STATUS_INIT_TEST   -> Speichertestmodus
              STATUS_INIT_SWITCH -> Normalanlauf mit vertauschten Schnittstellen
*/
DPR_DWORD DP_CODE_ATTR  CI_start_cp ( const DPR_STRING   DP_MEM_ATTR     *cp_name,
                                      const DPR_STRING   DP_MEM_ATTR     *firmware,
                                      const DPR_STRING   DP_MEM_ATTR     *database,
                                      DPR_WORD                     mode     );



/**********************************************************************************/

/* Beschreibung:
   ============
   - Diese Funktion setzt den entsprechenden CP zurück. Nach diesem Aufruf ist der
     CP5613 deaktiviert, d. h. er sieht kein Busteilnehmer mehr.

   Parameter:
   ----------
   cp_name : Name des CP5613, analog zum PG / PC Panel
   mode    : Ein Reset des CP ist nur rückwirkungsfrei möglich, sofern keine Applikation
             mehr den CP gerade benutzt ( -> alle Applikationen haben sich mit CI_close
             abgemeldet. Daher sollte die Funktion im Normalfall mit dem Wert
             RESET_MODE_SAFE ausgeführt werden. Nur so kein ein in allen Fällen ein
             einwandfreies Systemverhalten garantiert werden.
             In Ausnahmefällen ( z. B. eine Applikation ist abgestürzt ohne ein
             CI_close abzusetzen ) kann es sinnvoll sein den CP absolut zurückzusetzen
             (RESET_MODE_ALWAYS).   */


#define RESET_MODE_SAFE      0x00000000
#define RESET_MODE_ALWAYS    0x00000001
DPR_DWORD DP_CODE_ATTR  CI_reset_cp ( const DPR_STRING   DP_MEM_ATTR  *cp_name,
                                      DPR_DWORD                 reset_mode );




/**********************************************************************************/

/* Beschreibung:
   ============
   - Mit dieser Funktion meldet sich eine Applikation beim Treiber der CP5613
     Kommunikationsprozessoren an. Als Rückgabewert erhält er unter anderem
     einen Zeiger auf das DP spezifische Dual Ported Ram der Baugruppe mit dem er
     direkt auf das DP - Prozessabbild zugreifen kann. Vorraussetzung für eine sinnvolle
     Nutzung dieses Pointers ist, daß DP vorher ordnungsgemäß hochgefahren wurde.
     ( -> siehe die DP spezifischen Funktionen im File dp_5613.h. )


   Parameter:
   ----------
   user_handle: Hier liefert die Funktion ein Handle zurueck, welches die Applikation
                 für alle folgenden Aufrufe verwenden muß.
   cp_name    : Name des CP5613, analog zum PG / PC Panel
*/

DPR_DWORD DP_CODE_ATTR  CI_open  ( DPR_DWORD        DP_MEM_ATTR  *user_handle,
                                   const DPR_STRING       DP_MEM_ATTR  *cp_name   );



/**********************************************************************************/

/* Beschreibung:
   ============
   - Mit dieser Funktion holt sich eine Applikation den Pointer auf das DPR  


   Parameter:
   ----------
   user_handle: Hier liefert die Funktion ein Handle zurueck, welches die Applikation
                 für alle folgenden Aufrufe verwenden muß.
   dpr        : Hier gibt die Funktion der Applikation einen Pointer auf das Dual Ported
                Ram der Baugruppe zurück. Dieser Pointer ist bis zum entsprechenden
                CI_close gültig. Ein Verwenden dieses Pointers nach einem CI_close
                oder die Verletzung der Strukturgrenzen, bewikt eine Schutzverletzung
                in der Anwenderapplikation.  */


DPR_DWORD DP_CODE_ATTR  CI_get_pointer ( DPR_DWORD           user_handle,
                               DPR_CP5613_DP_T  volatile DP_MEM_ATTR  **dpr ); 



/**********************************************************************************/

/* Beschreibung:
   ============
   - Mit dieser Funktion meldet sich eine Applikation für den Direktzugriff an


   Parameter:
   ----------
   user_handle: Handle aus CI_open
   timeout    : Timeout in ms 


   Returnwerte:
  =============

   CI_RET_OK: Ok !
   CI_RET_GET_TIMEOUT: andere DP-Applikation hat schon Zugriff

  */


DPR_DWORD DP_CODE_ATTR  CI_get_dp_access  ( DPR_DWORD       user_handle,
                                            DPR_DWORD       timeout );


/**********************************************************************************/

/* Beschreibung:
   ============
   - Mit dieser Funktion meldet sich eine Applikation für den Direktzugriff ab
   - wird mit CI_close implizit mit durch geführt

   Parameter:
   ----------
   user_handle: Handle aus CI_open  */

DPR_DWORD DP_CODE_ATTR  CI_release_dp_access ( DPR_DWORD      user_handle );





/**********************************************************************************/

/* Beschreibung:
   ============
   - Mit dieser Funktion meldet sich eine Applikation beim Treiber ab.
     Vor dieser Funktion müssen die protokolspezifischen Aufräumarbeiten durchgeführt
     worden sein ( -> alles mit CI_receive abgeholt, Resourcen auf dem CP freigegeben,
     .... ).


   Parameter:
   ----------
   user_handle: Nach dem Aufruf dieser Funktion ist der Pointer des entsprechenden
                CI_open nicht mehr gültig.   */


DPR_DWORD DP_CODE_ATTR  CI_close ( DPR_DWORD     user_handle);

/**********************************************************************************/

/* Beschreibung:
   ============
   - Mit dieser Funktion wird der Produktivbetrieb über die Kommunikationsschnittstelle
     abgewickelt. Die Antwort auf den Auftrag erhält der Anwender entweder sofort
     oder muß sie mit einem CI_receive anschließend abholen ( -> siehe Returnwerte ).

   Parameter:
   ----------
   ci_request : Parameter siehe Strukturbeschreibung   */


DPR_DWORD DP_CODE_ATTR  CI_send  ( CI_REQUEST_T   DP_MEM_ATTR    *ci_request  );

/**********************************************************************************/

/* Beschreibung:
   ============
   - Diese Funktion funktioniert analog CI_send. Der Aufruf dieser Funktion
     sollte auf die Notfälle beschränkt werden, falls CI_send mit
     CI_RET_SEND_NO_BUFFER_AVAILABLE  zurückkommt, es aber unbedingt notwendig ist,
     noch einen Auftrag an den CP zu senden.

   Parameter:
   ----------
   ci_request : Parameter siehe Strukturbeschreibung   */


DPR_DWORD DP_CODE_ATTR  CI_reserve_send  ( CI_REQUEST_T  DP_MEM_ATTR    *ci_request );

/**********************************************************************************/

/* Beschreibung:
   ============
   - Diese Funktion holt alle CI_receive mit Timeout vorzeitig zurück.
     Wichtig für Aufräumarbeiten.

   Parameter:
   ----------
   user_handle: Handle aus CI_open   */


DPR_DWORD DP_CODE_ATTR  CI_cancel_receive ( DPR_DWORD     user_handle    );

/**********************************************************************************/

/* Beschreibung:
   ============
   - Mit dieser Funktion werden Antworten des CPs abgeholt, die nicht sofort mit
     CI_send zurückgekommenn sind.


   Parameter:
   ----------
   ci_request : Parameter siehe Strukturbeschreibung    */



DPR_DWORD DP_CODE_ATTR  CI_receive  ( CI_REQUEST_T  DP_MEM_ATTR  *ci_request );


/**********************************************************************************/

/* Beschreibung:
   ============

   Parameter:
   ----------
   ci_request : Parameter siehe Strukturbeschreibung    */

#define CI_EVENT_MAX_INFO    0x40U

typedef struct CI_EVENT_HEADER_S
{
 DPR_DWORD        id;
 DPR_DWORD        code;
 DPR_DWORD        dw_para_1;
 DPR_DWORD        dw_para_2;
 DPR_STRING       info[CI_EVENT_MAX_INFO];
 DPR_STRING       originator[CI_EVENT_MAX_INFO];
 DPR_STRING       log_name[CI_EVENT_MAX_INFO];
} GNUC__PACK CI_EVENT_HEADER_T;


#define CI_EVENT_INFO         0x00000000UL
#define CI_EVENT_PCI          0xCCCC0000UL
#define CI_EVENT_BUS_CP       0xDDDD0000UL
#define CI_EVENT_EXCP_CP      0xEEEE0000UL
#define CI_EVENT_EXCP_DRIVER  0xFFFF0000UL



DPR_DWORD DP_CODE_ATTR  CI_receive_event( CI_REQUEST_T  DP_MEM_ATTR  *ci_request );

/**********************************************************************************/

/* Beschreibung:
   ============
   - Diese Funktion erzeugt für die Applikations eine Semaphore ( siehe API Aufruf ->
     CreateSemaphore ). An dieser Semaphore  kann die Applikation in einem Thread auf
     das entsprechende Ereignis mit den API Aufrufen ->
     WaitForSingleObject /WaitForMultipleObjects warten / MsgWaitForMultipleObjects

   - INPUT_CHANGE, DIAG_CHANGE, CYCLE_INT koennen nur einmal pro CP angemeldet werden


   Parameter:
   ----------
   user_handle: Handle aus CI_open
   sema_type:   Typ des Ereignisses bei dem der Treiber den Semaphoren Zähler
                erhöhen soll.
   sema_handle: Handle an dem die Applikation mit den WaitFor.. API Funktionen
                warten kann. */


DPR_DWORD DP_CODE_ATTR  CI_init_sema ( DPR_DWORD               user_handle,
                                       DPR_DWORD               sema_type,
                                       void       DP_MEM_ATTR  *sema_handle );

/**********************************************************************************/

/* Beschreibung:
   ============
   - Diese Funktion erzeugt für die Applikations eine Semaphore ( siehe API Aufruf ->
     CreateSemaphore ). An dieser Semaphore  kann die Applikation in einem Thread auf
     das entsprechende Ereignis mit den API Aufrufen ->
     WaitForSingleObject /WaitForMultipleObjects warten / MsgWaitForMultipleObjects

   - INPUT_CHANGE, DIAG_CHANGE, CYCLE_INT koennen nur einmal pro CP angemeldet werden


   Parameter:
   ----------
   user_handle: Handle aus CI_open
   sema_handle: Handle an dem die Applikation mit den WaitFor.. API Funktionen
                warten kann. */

DPR_DWORD DP_CODE_ATTR  CI_delete_sema ( DPR_DWORD               user_handle,
                                         DPR_DWORD               sema_handle );

/**********************************************************************************/

/* Beschreibung:
   ============
   - Die Applikation teilt der CI_BASE_FUNC.DLL mit, daß eine Windows Message verschickt
     werden soll, wenn ein Ereignis eintritt

   Parameter:
   ----------
   user_handle : Handle aus CI_open
   hAppHandle  : Windows Handle der Applikation
   msg         : Windows Message, die verschickt werden soll     */

DPR_DWORD DP_CODE_ATTR CI_SetHWndMsg( DPR_DWORD          user_handle,
                                      DPR_DWORD          hAppHandle,
                                      DPR_DWORD          msg           );


/**********************************************************************************/


/************** Fast Logic ********************************************************/

/* Beschreibung:
   ============
   - 

   Parameter:
   ----------
   user_handle: Handle aus CI_open

*/
DPR_DWORD DP_CODE_ATTR  CI_fast_logic_on( DPR_DWORD        user_handle,
                                          DPR_WORD         fast_logic_id,
                                          DP_FAST_LOGIC_T  *fast_logic );

DPR_DWORD DP_CODE_ATTR  CI_fast_logic_off( DPR_DWORD        user_handle,
                                           DPR_WORD         fast_logic_id );

/**********************************************************************************/


/**********************************************************************************/

/* Beschreibung:
   ============
   - Diese Funktion schreibt den Inhalt des dual ported RAMs des CPs in eine Datei.

   Parameter:
   ----------
   cp_name  : Name des CP5613, analog zum PG / PC Panel
   filename : Name der Datei. Falls die Datei noch nicht existiert, so wird sie
              angelegt. Falls sie existiert, wird sie überschreiben. */

DPR_DWORD DP_CODE_ATTR  CI_dpr_dump( const DPR_STRING   DP_MEM_ATTR     *cp_name,
                                     const DPR_STRING   DP_MEM_ATTR     *filename );


/**********************************************************************************/



/**********************************************************************************/

/* Beschreibung:
   ============
   - 

   Parameter:         
   ----------      */

DPR_DWORD DP_CODE_ATTR CI_connect_cp(const DPR_STRING DP_MEM_ATTR *cp_name,
                                     DPR_CP5613_ALL_T volatile DP_MEM_ATTR **dpr,
                                     CP5613_PLX_T volatile DP_MEM_ATTR     **plx,
                                     CP5613_DOWNLOAD_T volatile DP_MEM_ATTR  **down);

/**********************************************************************************/

/* Beschreibung:
   ============

   Parameter:       
   ----------   */

DPR_DWORD DP_CODE_ATTR CI_disconnect_cp(const DPR_STRING DP_MEM_ATTR *cp_name);


/**********************************************************************************/

/* Beschreibung:
   ============
   - 

   Parameter:
   ----------
              */

void DP_CODE_ATTR CI_MBox(const char *info, ...);  /*lint !e1916 */

/**********************************************************************************/



/**********************************************************************************/

/* Beschreibung:
   ============
   - 

   Parameter:
   ----------

              */


typedef struct CI_CP_CFG_S
{
 DPR_STRING      cp_name[CI_MAX_PATH];  // IN
 DPR_STRING      log_device[CI_MAX_PATH];  // OUT
 DPR_DWORD       here; // 0 -> not here
 DPR_STRING      database[CI_MAX_PATH]; // OUT when here == 1
                                        // 0 when CP not started
 DPR_DWORD       bus_nr;  // OUT when here == 1
 DPR_DWORD       slot_nr; // OUT when here == 1
} GNUC__PACK CI_CP_CFG_T;     



DPR_DWORD DP_CODE_ATTR CI_get_cp_cfg(CI_CP_CFG_T    *cp_cfg);  

/**********************************************************************************/


/**********************************************************************************/

/* Beschreibung:
   ============
   - 

   Parameter:
   ----------
   Parameter:
   ----------
   user_handle: Handle aus CI_open
   led1_ms    : halbe Periodendauer -> 50ms - 5000ms , 0 Led immer aus, 1 Led an
   led2_ms    : halbe Periodendauer -> 50ms - 5000ms , 0 Led immer aus, 1 Led an
   mode       : 0 -> Led blinken asynchron 
                1 -> Led blinken synchron,
                2 -> Led auf ASIC Kontrolle
           */


typedef struct  CI_LED_BLINK_S
{
 DPR_DWORD     led1_ms;
 DPR_DWORD     led2_ms;
 DPR_DWORD     mode;
} GNUC__PACK CI_LED_BLINK_T;

DPR_DWORD DP_CODE_ATTR  CI_led_blink  ( DPR_DWORD      user_handle,
                                        CI_LED_BLINK_T *blink       ); 

/**********************************************************************************/


/**********************************************************************************/

/* Beschreibung:
   ============
   - Diese Funktion liefert einen Fehlertext fuer einen CI-Fehler

   Parameter:
   ----------
   error    : CI-Fehlernummer
   language : Spracheinstellung ("momentan German" oder "English", falls Sprache nicht 
              wird englischer Text ausgegeben
   text     : Array fuer Fehlertext
 */


DPR_DWORD DP_CODE_ATTR CI_get_err_txt ( DPR_DWORD  error,                               /* in     */
                                    const  DPR_STRING DP_MEM_ATTR *language,               /* in     */
                      DPR_STRING DP_MEM_ATTR text[DP_ERR_TXT_SIZE] ); /* out    */



/**********************************************************************************/



#ifdef __cplusplus	
 }
#endif


#ifndef DONT_USE_MS_PACK
 #pragma pack()
#endif




/****************************************************************************/
/*     END OF FILE:      CI_5613.H                                          */
/****************************************************************************/
