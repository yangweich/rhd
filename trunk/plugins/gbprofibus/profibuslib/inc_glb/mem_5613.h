/****************************************************************************/
/*    Copyright (C) SIEMENS AG 1989..1998 All Rights Reserved.Confidential  */
/****************************************************************************/
/*     Projekt       :                                                      */
/*                                                                          */
/*     Modulname     : FW_5613.H                                            */
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

#ifndef DONT_USE_MS_PACK
#pragma pack(1)
#endif

#ifdef __GNUC__
#define GNUC__PACK __attribute__ ((packed))
#else
#define GNUC__PACK
#endif


typedef struct CP5613_LEN_SLAVE_S
{
	
	DPR_WORD     length_in_dw;    /* Laenge der Kopierdaten in DWORD */
	DPR_BYTE     reserved_b_00[0xFE];
} GNUC__PACK CP5613_LEN_SLAVE_T;



typedef struct CP5613_FASTLOGIC_S
{
	DPR_BYTE    ack; // 0x00500000 Bit 7..4 Hardwareversion
	// Bit 0..3 : -> fl0..fl3
	DPR_BYTE    res00[0x1FFF];
	DPR_BYTE    cmp_value_byte; /* 0x00520000 -> Wert auf den verglichen wird */
	DPR_BYTE    res01[0x1FFF];
	DPR_BYTE    mask_byte;    /* 0x00540000 -> Bit = 1 -> maskiert */
	DPR_BYTE    res02[0x1FFF];
	DPR_WORD    slave_addr_index;  /* 0x00560000 -> Slavenummer : Index */
	DPR_BYTE    res03[0x1FFE];
	
} GNUC__PACK CP5613_FASTLOGIC_T;



typedef struct CP5613_MEM_S
{
	DPR_BYTE    system_area[0x00004100]; /* 0x00000000 - 0x000040FF -> Finger weg */
	
	DPR_BYTE    reserved_xx[0x0000BF00]; /* 0x00004100 - 0x0000FFFF -> Reserve*/
	
	DPR_BYTE    mem_first_mb[0x000F0000]; /* 0x00010000 - 0x000FFFFF -> Memory */
	
	
	CP5613_DOWNLOAD_T   download;	/* 0x00100000 - 0x001FFFFF -> 2.MByte DRAM */
									/* hier ist die Firmware -> siehe 8. MByte */
	
	DPR_BYTE    sh_ampro_scb[0x00010000]; /* 0x00200000 - 0x0020FFFF -> SCB Ampro */
	
	DPR_BYTE    sh_mem_aspc2[0x00080000]; /* 0x00210000 - 0x0028FFFF -> Spiegel  */
	
	DPR_BYTE        sh_database_area[DOWN_DATABASE_SIZE]; /* 0x00290000 - 0x0029FFFF -> Spiegel  */
	CP5613_EQU_T    sh_equ;
	
	/*----------- Fifo ---------------------------------------------*/
	DPR_CP5613_PI_T     fifo_pi; /* 0x002A0000 - 0x002BFFFF -> Fifo  */
	
	/*----------- PA ---------------------------------------------*/
	DPR_CP5613_PI_T     pi; /* 0x002C0000 - 0x002DFFFF -> PA  */
	
	/*----------- KK ---------------------------------------------*/
	DPR_CP5613_INFO_WATCH_T  info_watch;  /* 0x002E0000 - 0x002E0FFF */
	DPR_CP5613_CC_T          cc;  /* 0x002E1000 - 0x002FFFFF */
	
	
	/*----------- Info fuer den Hardwarekopierer ueber die Slavelaengen ---------*/
	CP5613_LEN_SLAVE_T   slv_in[DPR_MAX_SLAVE_NR];  /* 0x00300000 - 0x00307FFF */
	CP5613_LEN_SLAVE_T   slv_out[DPR_MAX_SLAVE_NR];  /* 0x00308000 - 0x0030fFFF */
	CP5613_LEN_SLAVE_T   slv_diag[DPR_MAX_SLAVE_NR];  /* 0x00310000 - 0x00317FFF */
	DPR_BYTE             slv_reserved[0x00008000];
	
	/*----------- Events und Filter ---------------------------------------------*/
	DPR_CP5613_EF_T      ef;  /* 0x00320000 - 0x0033FFFF */
	DPR_CP5613_EF_T      ef_shadow_1;  /* 0x00340000 - 0x0035FFFF */
	DPR_CP5613_EF_T      ef_shadow_2;  /* 0x00360000 - 0x0037FFFF */
	
	/*----------- Adressraum Slavemodul -------------------------------------*/
	DPR_BYTE    sdar[0x00080000]; /* 0x00380000 - 0x003FFFFF -> Slave Modul */
	
	
	/*----------- Steuerbits    ---------------------------------------------*/
	DPR_CP5613_CTR_CPU_T     ctr_cpu; /* 0x00400000 - 0x0047FFFF -> Steuerbits */
	DPR_BYTE                 reserved_ctr_cpu[0x8000];
	DPR_CP5613_CTR_CPU_T     ctr_cpu_shadow; /* 0x00480000 - 0x004fFFFF -> Steuerbits */
	DPR_BYTE                 reserved_ctr_cpu_shadow[0x8000];
	
	
	CP5613_FASTLOGIC_T       fl[0x04]; /* 0x00500000 - 0x0051FFFF -> FastLogic */
	CP5613_FASTLOGIC_T       res_fl[0x1C]; /* 0x00520000 - 0x005FFFFF  */
	
	
	DPR_BYTE    mem_1_mb[0x00100000]; /* 0x00600000 - 0x006FFFFF -> 1. MByte */
	
									  CP5613_DOWNLOAD_T       down_s2;  /* 0x00700000 - 0x007FFFFF -> 2. MByte
									  384 kB Firmware mit Reset Jump  */
} GNUC__PACK CP5613_MEM_T;



#ifndef DONT_USE_MS_PACK
#pragma pack()
#endif



/****************************************************************************/
/*     END OF FILE:      FW_5613.H                                          */
/****************************************************************************/
