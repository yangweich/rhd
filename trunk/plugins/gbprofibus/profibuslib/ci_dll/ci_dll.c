/****************************************************************************/
/*     Copyright (C) Siemens AG 1998  All Rights Reserved.                  */
/****************************************************************************/
/*                                                                          */
/*  version: @(#1)CP5613.C     V 1.00                                       */
/*                                                                          */
/*  This source file contains all functions to initialize CP5613.           */
/*                                                                          */
/****************************************************************************/
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

/*
+-------------------------------------------------------------------+
| includes                                                          |
+-------------------------------------------------------------------+
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "linux.h"

#include <dp_5613.h>
#include <ci_5613.h>
#include <5613_ret.h>
#include <dp_base.h>

/*
+-------------------------------------------------------------------+
| defines                                                           |
+-------------------------------------------------------------------+
*/

/* DPMI */
#ifdef CI_DPMI
#define CI_FAR far
#define CI_MEMCPY _fmemcpy
#define CI_MEMSET _fmemset
#else
#define CI_FAR
#define CI_MEMCPY memcpy
#define CI_MEMSET memset
#endif

/* define for length of device name */
#define DEVICE_NAME_LENGTH    80

/* define for delay (waiting for CP5613) */
#define CP5613_BASE_TIMEOUT_SEC  50 /* 50msec           */
#define CP5613_TIMEOUT_LOOP      20 /* 20*50msec=1sec */
static void delay(int ms) {
  struct timespec t;
  t.tv_sec = 0;
  t.tv_nsec = ms * 1000000;
  nanosleep(&t, 0);
}

#ifndef linux
/* defines for PCI interrupt */
#define PCI_INT                 0x1A /* BIOS-Interrupt                   */
#define PCI_FUNCTION_ID         0xB1 /* AH: ID for PCI-Functions         */
#define PCI_BIOS_PRESENT        0x01 /* AL: check if BIOS is present     */
#define PCI_FIND_DEVICE         0x02 /* AL: find device                  */
#define PCI_READ_CONFIG_LONG    0x0a /* AL: read configuration (long)    */

/* defines for DPMI interrupt */
#define DPMI_INT                0x31 /* DPMI-Interrupt                   */
#define DPMI_ALLOC_SELECTOR     0x00 /* AX: allocate a selector          */
#define DPMI_FREE_SELECTOR      0x01 /* AX: free selector                */
#define DPMI_SET_SELECTOR_BASE  0x07 /* AX: set selector base            */
#define DPMI_SET_SELECTOR_LIMIT 0x08 /* AX: set selector limit           */
#endif

/* defines for vendorID and deviceID of CP5613 */
#define VENDOR_ID         0x110a
#define DEVICE_ID         0x3142

/* defines for configuration registers for CP5613 */
#define CP5613_REGISTER_PLX             0x10
#define CP5613_REGISTER_DOWNLOAD_AREA   0x18
#define CP5613_REGISTER_DPRAM           0x1c

/* defines for enabling and disabling CPU of CP5613 */
#define CP5613_DISABLE  0x03
#define CP5613_ENABLE   0x00

/* For test purpose, output on screen is available.               */
#ifndef OUTPUT_DESIRED
#define MYPRINTF0(a)
#define MYPRINTF1(a,b)
#define MYPRINTF2(a,b,c)
#define MYPRINTF3(a,b,c,d)
#define MYPRINTF4(a,b,c,d,e)
#define MYDISPLAY(a,b)
#define MYDUMP(a,b,c)
#else
#define MYPRINTF0(a)         printf(a)
#define MYPRINTF1(a,b)       printf(a,b)
#define MYPRINTF2(a,b,c)     printf(a,b,c)
#define MYPRINTF3(a,b,c,d)   printf(a,b,c,d)
#define MYPRINTF4(a,b,c,d,e) printf(a,b,c,d,e)
#define MYDISPLAY(a,b)       display_pointer(a,b)
#ifdef DUMP_DESIRED
#define MYDUMP(a,b,c)        display_dump(a,b,c)
#else
#define MYDUMP(a,b,c)
#endif
#endif

/* show informatin in adm_cp */
#define SHOW_ADM_CP()  MYPRINTF1("ADM_CP_T: to_send_index    =%4x\n",ptr_dpram->cc.adm_cp.to_send_index);     \
	MYPRINTF1("ADM_CP_T: cp_ready         =%4x\n",ptr_dpram->cc.adm_cp.cp_ready);          \
	MYPRINTF1("ADM_CP_T: host_ready       =%4x\n",ptr_dpram->cc.adm_cp.host_ready);        \
	MYPRINTF1("ADM_CP_T: block_return_head=%4x\n",ptr_dpram->cc.adm_cp.block_return_head); \
MYPRINTF1("ADM_CP_T: data_return_head =%4x\n",ptr_dpram->cc.adm_cp.data_return_head)

/* show information in send/receive block */
#define SHOW_BLOCK_DATA(a) MYPRINTF1(                                     \
	"BLOCK: user_handle=%8lx\n",                 \
	((CI_DATA_BUFFER_T CI_FAR *)&               \
	(ptr_dpram->cc.block[a]))                  \
	->usr_header.user_handle);                  \
	MYPRINTF1(                                     \
	"BLOCK: response   =%8lx\n",                 \
	((CI_DATA_BUFFER_T CI_FAR *)&               \
	(ptr_dpram->cc.block[a]))                  \
->usr_header.response)

#ifndef linux
/* defines, depending on the operating system */
#define REG_AL    al
#define REG_AH    ah
#define REG_AX    eax
#define REG_BL    bl
#define REG_BH    bh
#define REG_BX    ebx
#define REG_CL    cl
#define REG_CX    ecx
#define REG_DX    edx
#define REG_SI    esi
#define REG_DI    edi
#define REG_CFLAG cflag
#endif

/* defines for accessing low and high word of long integer */
#define LOWORD(a)    ((unsigned short int)(a&0x0000FFFFl))  
#define HIWORD(a)    ((unsigned short int)((a>>16)&0x0000FFFFl))  

/* defines for buffer free or buffer occupied in DPRAM */
#define DPRAM_BUFFER_FREE      0x00
#define DPRAM_BUFFER_OCCUPIED  0x01

/*
+-------------------------------------------------------------------+
| structures                                                        |
+-------------------------------------------------------------------+
*/

#ifdef CI_DPMI
/* structure for far pointer */
struct CP5613_FP
{
	unsigned long  offset;
	unsigned short selector;
};
#else
/* structure for far pointer */
struct CP5613_FP
{
	unsigned long  offset;
};
#endif

/*
+-------------------------------------------------------------------+
| Global data                                                       |
+-------------------------------------------------------------------+
*/

#ifndef linux
static unsigned char      pci_device_and_function;
static unsigned char      pci_bus;
#endif
static unsigned long      physical_address_plx;
static unsigned long      physical_address_download_area;
static unsigned long      physical_address_dpram;
static CP5613_PLX_T       CI_FAR *ptr_plx;
static CP5613_DOWNLOAD_T  CI_FAR *ptr_download_area;
static DPR_CP5613_ALL_T   CI_FAR *ptr_dpram;
static char               *buffer_ptr=NULL;
#ifndef linux
static union REGS         pregs;
#endif
static char               my_dummy_device_name[DEVICE_NAME_LENGTH];
static DPR_DWORD          my_open_user_handle=0x00000000l;
static DPR_DWORD          my_dummy_user_handle=0x00000000l;
static DPR_DWORD          my_dummy_reset_mode;
static DPR_DWORD          my_dummy_sema_type;
static void               *my_dummy_init_sema_handle;
static DPR_DWORD          my_dummy_delete_sema_handle;
static DPR_WORD           my_dummy_mode;
static DPR_DWORD          my_dummy_timeout;
static DPR_WORD     	  my_dummy_fast_logic_id;
static DP_FAST_LOGIC_T   *my_dummy_fast_logic;
static DPR_DWORD          my_dummy_error;
static DPR_STRING DP_MEM_ATTR  *my_dummy_language;
static DPR_STRING DP_MEM_ATTR  *my_dummy_text;



static DPR_CP5613_DP_T    CI_FAR *ptr_to_dppart_of_dpram;
#ifdef CI_DPMI
static unsigned short     selector_plx;
static unsigned short     selector_download_area;
static unsigned short     selector_dpram;
#endif
static unsigned char      dpram_buffer[CI_BLOCK_COUNT];

/*
+-------------------------------------------------------------------+
| Prototypings                                                      |
+-------------------------------------------------------------------+
*/

static unsigned long copy_file(const char *filename,
			                                   char CI_FAR *destination,
							   unsigned long dest_len);
static unsigned long dpram_buffer_get_index(void);
static void          dpram_buffer_release_index(unsigned long index);
#ifndef linux
static DPR_DWORD     pci_bios_available(void);
#endif
static void          cp5613_disable(void);
static DPR_DWORD     cp5613_enable(void);
static DPR_DWORD     cp5613_load_fw_ldb(
										const char *fw_name,const char *ldb_name);
static void          cp5613_make_interrupt(unsigned long index);
static void          cp5613_shut(void);
static DPR_DWORD     cp5613_wait_for_response(DPR_DWORD timeout,
											  unsigned long *index_ptr);
static unsigned long get_file_size(const char *filename);
#ifndef linux
static DPR_DWORD     make_pci_interrupt(union REGS *pregs);
#endif

#ifdef OUTPUT_DESIRED
#ifdef DUMP_DESIRED
static void          display_dump(char *msg,char CI_FAR *ptr,int length);
#endif
static void          display_pointer(
									 char *msg,
									 struct CP5613_FP CI_FAR *ptr);
#endif

#ifdef CI_DPMI
static DPR_DWORD     dpmi_alloc_selector(unsigned short *selector_ptr);
static DPR_DWORD     dpmi_map_physical_address_to_ptr(
													  unsigned long physical_address,
													  unsigned short *selector_ptr,
													  unsigned long limit,
													  struct CP5613_FP CI_FAR *ptr);
static DPR_DWORD     dpmi_set_selector_base(unsigned short selector,
											unsigned long base);
static DPR_DWORD     dpmi_set_selector_limit(unsigned short selector,
											 unsigned long limit);
static void          dpmi_free_selector(unsigned short selector);
static DPR_DWORD     make_dpmi_interrupt(union REGS *pregs);
#endif

/*
+-------------------------------------------------------------------+
| copy_file                                                         |
| (return number of bytes read from file and copied to destination) |
+-------------------------------------------------------------------+
*/

static unsigned long copy_file(const char *filename,
							   char CI_FAR *destination,
							   unsigned long dest_len)
{  FILE *fp;
unsigned long length=0;

MYPRINTF0("copy_file\n");

/* copy contents of file to buffer_ptr        */
/* (only near pointer for fread available)    */
/* remember: buffer has been allocated before */
/* and must be freed after copying all data   */
/* to the download area                       */
fp=fopen(filename,"rb");
if(fp!=(FILE *)NULL)
{
	length=fread(buffer_ptr,1,dest_len,fp);
	fclose(fp);
}

/* now copy data from near buffer_ptr to far? destination */
if(length<=dest_len)
{
	CI_MEMCPY((char CI_FAR *)destination,(char CI_FAR *)buffer_ptr,length);
}
return(length);
}

/*
+-------------------------------------------------------------------+
| dpram_buffer_get_index                                            |
| (returns index of buffer in DPRAM if free, CI_BLOCK_COUNT else)   |
+-------------------------------------------------------------------+
*/

static unsigned long dpram_buffer_get_index(void)
{  static unsigned long last_index=0;
unsigned long index;
unsigned short found;
CI_DATA_BUFFER_T CI_FAR *ci_ptr;

found=0;

/* start searching with last used index */
for(index=last_index;index<CI_BLOCK_COUNT;index++)
{
	if(dpram_buffer[index]==DPRAM_BUFFER_FREE)
	{
		last_index=index;
		found=1;
		break;
	}
}
if(!found)
{
	for(index=0;index<last_index;index++)
	{
		if(dpram_buffer[index]==DPRAM_BUFFER_FREE)
		{
			last_index=index;
			found=1;
			break;
		}
	}
}
if(found)
{
	ci_ptr=(CI_DATA_BUFFER_T CI_FAR *)&
		(ptr_dpram->cc.block[index]);
	
	/* mark block as sent to firmware */
	ci_ptr->search_pat_0=CI_SEARCH_PAT_FW;
	return index;
}
else
{
	return(CI_BLOCK_COUNT);
}
}

/*
+-------------------------------------------------------------------+
| dpram_buffer_release_index                                        |
| (release index of buffer in DPRAM)                                |
+-------------------------------------------------------------------+
*/

static void dpram_buffer_release_index(unsigned long index)
{
	CI_DATA_BUFFER_T CI_FAR *ci_ptr;
	dpram_buffer[index]=DPRAM_BUFFER_FREE;
	ci_ptr=(CI_DATA_BUFFER_T CI_FAR *)&(ptr_dpram->cc.block[index]);
	ci_ptr->search_pat_0 = CI_SEARCH_PAT_DRV;
}

/*
+-------------------------------------------------------------------+
| cp5613_available                                                  |
| (check if CP5613 is available)                                    |
+-------------------------------------------------------------------+
moved to linux.c
*/

/*
+-------------------------------------------------------------------+
| cp5613_disable                                                    |
| (disable CP5613)                                                  |
+-------------------------------------------------------------------+
*/

static void cp5613_disable(void)
{  
	MYPRINTF0("cp5613_disable\n");
	ptr_dpram->ctr_cpu.C_res_en=CP5613_DISABLE;
}

/*
+-------------------------------------------------------------------+
| cp5613_enable                                                     |
| (enable CP5613)                                                   |
+-------------------------------------------------------------------+
*/

static DPR_DWORD cp5613_enable(void)
{  DPR_DWORD delay_cnt;

MYPRINTF0("cp5613_enable\n");

/* start CP */
ptr_download_area->equ.mode_equ_dis=0;
CI_MEMSET(&(ptr_download_area->add_info), 0, sizeof(ptr_download_area->add_info));
ptr_download_area->add_info.clock=CLOCK_NONE;
ptr_dpram->cc.adm_cp.status=STATUS_INIT;
ptr_dpram->ctr_cpu.C_res_en=CP5613_ENABLE;

/* check, if CP5613 is running (timeout=1sec) */
for(delay_cnt=0;
delay_cnt<(CP5613_TIMEOUT_LOOP);
delay_cnt++)
{
	/* CP5613 running? */
	if(ptr_dpram->cc.adm_cp.status==STATUS_RUNNING)
	{
		break;
	}
	
	/* delay for CP5613_BASE_TIMEOUT_SEC */
	delay(CP5613_BASE_TIMEOUT_SEC);
}

/* timeout or CP5613 running */
if(ptr_dpram->cc.adm_cp.status!=STATUS_RUNNING)
{  /* CP5613 not running */
	return(CI_RET_START_CP_NO_REACTION);
}
return(CI_RET_OK);
}

/*
+-------------------------------------------------------------------+
| cp5613_load_fw_ldb                                                |
| (load firmware and ldb to download area)                          |
+-------------------------------------------------------------------+
*/

static DPR_DWORD cp5613_load_fw_ldb(const char *fw_name,const char *ldb_name)
{  unsigned long file_length_fw,file_length_ldb,length;
char CI_FAR *tmp_ptr;

MYPRINTF2("cp5613_load_fw_and_ldb: fw %s and ldb %s\n",
		  fw_name,ldb_name);
MYDISPLAY("download_area",
		  (struct CP5613_FP CI_FAR *)&ptr_download_area);

/* first step: get size of firmware file and size of ldb file */
file_length_fw=get_file_size(fw_name);
file_length_ldb=get_file_size(ldb_name);
MYPRINTF2("size of firmware file = %ld, size of ldb file = %ld\n",
		  file_length_fw,file_length_ldb);
if(file_length_fw==0)
{
	MYPRINTF0("Firmware file not found\n");
	return(CI_RET_START_OPEN_FIRMWARE);
}
if(file_length_ldb==0)
{
	MYPRINTF0("Database file not found\n");
	return(CI_RET_START_OPEN_DATABASE);
}

/* second step: malloc temporary buffer to read files              */
/* attention: buffer_ptr will be used in function copy_file        */
/*            at the end of the sequence, the buffer must be freed */
length=file_length_fw;
if(file_length_ldb>length)
{
	length=file_length_ldb;
}
buffer_ptr=malloc(length);
if(buffer_ptr==NULL)
{  /* no ressources */
	MYPRINTF0("No resources\n");
	return(CI_RET_START_CP_RESOURCES_INT);
}

/* third step: copy ldb to download area */
length=copy_file(ldb_name,
				 (char CI_FAR *)
				 (ptr_download_area->database_area),
				 sizeof(ptr_download_area->database_area));
if(length!=file_length_ldb)
{
	/* free temporary buffer */
	free(buffer_ptr);
	buffer_ptr=NULL;
	MYPRINTF0("Database file too big\n");
	return(CI_RET_START_ALREADY_DATABASE);
}

/* fourth and last step: copy firmware to download area           */
/* attention please: do not copy the file to the beginning        */
/*                   of the firmware area                         */
/*                   the file must be copied from top to          */
/*                   down -> the last byte of the file to         */
/*                   the last byte of the firmware area and so on */
if(file_length_fw>sizeof(ptr_download_area->firmware))
{
	/* free temporary buffer */
	free(buffer_ptr);
	buffer_ptr=NULL;
	MYPRINTF0("Firmware file too big\n");
	return(CI_RET_START_OPEN_FIRMWARE);
}
tmp_ptr=(char CI_FAR *)(ptr_download_area->firmware);
tmp_ptr+=sizeof(ptr_download_area->firmware);
tmp_ptr-=file_length_fw;
length=copy_file(fw_name,
				 (char CI_FAR *)tmp_ptr,
				 file_length_fw);
if(length!=file_length_fw)
{
	/* free temporary buffer */
	free(buffer_ptr);
	buffer_ptr=NULL;
	MYPRINTF0("Firmware file too big\n");
	return(CI_RET_START_OPEN_FIRMWARE);
}

/* free temporary buffer */
free(buffer_ptr);
buffer_ptr=NULL;
return(CI_RET_OK);
}

/*
+-------------------------------------------------------------------+
| cp5613_make_interrupt                                             |
| (call interrupt on board of CP5613)                               |
+-------------------------------------------------------------------+
*/

static void cp5613_make_interrupt(unsigned long index)
{  DPR_WORD host_ready;

MYPRINTF0("cp5613_make_interrupt\n");

/* wait til last request has been executed by the firmware */
do
{  host_ready=ptr_dpram->cc.adm_cp.host_ready;
}  while(host_ready!=ADM_HC_DATA_TAKEN);

/* initialize administrator area and call interrupt to CP5613 */
ptr_dpram->cc.adm_cp.to_send_index=index;
ptr_dpram->cc.adm_cp.host_ready=ADM_HC_DATA_HERE;

/* show data */
SHOW_ADM_CP();
SHOW_BLOCK_DATA(index);

/* now really make interrupt */
ptr_plx->HB_MULTI_CONTROL_lsw=0x15;
MYPRINTF1("  ::-->> 0x%x\n", ptr_plx->HB_MULTI_CONTROL_lsw);
ptr_plx->HB_MULTI_CONTROL_lsw=0x1D;
MYPRINTF1("  ::-->> 0x%x\n", ptr_plx->HB_MULTI_CONTROL_lsw);
}

/*
+-------------------------------------------------------------------+
| cp5613_shut                                                       |
| (reset CP5613)                                                    |
+-------------------------------------------------------------------+
*/

void cp5613_shut(void)
{
	MYPRINTF0("\ncp5613_shut\n");
	
	/* disable CP5613 */
	cp5613_disable();
	
#ifdef CI_DPMI
	/* free all selectors */
	dpmi_free_selector(selector_dpram);
	dpmi_free_selector(selector_download_area);
	dpmi_free_selector(selector_plx);
#endif
}

/*
+-------------------------------------------------------------------+
| cp5613_wait_for_response                                          |
| (wait, til CP5613 has executed the outstanding request)           |
+-------------------------------------------------------------------+
*/

static DPR_DWORD cp5613_wait_for_response(DPR_DWORD timeout,
										  unsigned long *index_ptr)
{  DPR_DWORD delay_cnt,max_delay;

MYPRINTF0("cp5613_wait_for_response\n");

/* check, if CP5613 has sent data (timeout!!!) */
if(timeout>(DP_TIMEOUT_FOREVER/CP5613_TIMEOUT_LOOP))
{  /* wait forever */
	max_delay=0xffffffffl;  /* that's enough */
}
else
{
	max_delay=timeout*CP5613_TIMEOUT_LOOP;
}
for(delay_cnt=0;
delay_cnt<max_delay;
delay_cnt++)
{
	/* check if request has been executed */
	if(ptr_dpram->cc.adm_cp.cp_ready==ADM_HC_DATA_HERE)
	{
		break;
	}
	
	/* delay for CP5613_BASE_TIMEOUT_SEC */
	delay(CP5613_BASE_TIMEOUT_SEC);
}
/* show data */
SHOW_ADM_CP();

/* check if request has been executed */
if(ptr_dpram->cc.adm_cp.cp_ready!=ADM_HC_DATA_HERE)
{
	return(CI_RET_RECEIVE_TIMEOUT_NO_DATA);
}

/* show data */
*index_ptr=ptr_dpram->cc.adm_cp.data_return_head;
SHOW_BLOCK_DATA(*index_ptr);

/* initialize administrator area                                 */
ptr_dpram->cc.adm_cp.data_return_head=CI_NIL;
ptr_dpram->cc.adm_cp.cp_ready=ADM_HC_DATA_TAKEN;

return(CI_RET_OK);
}
#ifdef OUTPUT_DESIRED
#ifdef DUMP_DESIRED
/*
+-------------------------------------------------------------------+
| display_dump                                                      |
| (display dump)                                                    |
+-------------------------------------------------------------------+
*/

static void display_dump(char *msg,char CI_FAR *ptr,int length)
{  int i;
  static char buffer1[80];
  static char buffer2[10];

  MYPRINTF1("display dump of %s\n",msg);
  strcpy(buffer1,"");
  for(i=0;i<length;i++)
  {  sprintf(buffer2,"%02x ",ptr[i]);
    strcat(buffer1,buffer2);
    if((!((i+1)%16))||(i==(length-1)))
    {
      MYPRINTF1("%s\n",buffer1);
      strcpy(buffer1,"");
    }
  }
}
#endif

/*
+-------------------------------------------------------------------+
| display_pointer                                                   |
| (display pointer (selector and offset in case of CI_DPMI))        |
+-------------------------------------------------------------------+
*/

static void display_pointer(char *msg,
							struct CP5613_FP CI_FAR *ptr)
{
#ifdef CI_DPMI
	MYPRINTF3("%s: %04x%08lx\n",msg,ptr->selector,ptr->offset);
#else
	MYPRINTF2("%s: %08lx\n",msg,ptr->offset);
#endif
}
#endif

#ifdef CI_DPMI
/*
+-------------------------------------------------------------------+
| dpmi_alloc_selector                                               |
| (alloc new selector)                                              |
+-------------------------------------------------------------------+
*/

static DPR_DWORD dpmi_alloc_selector(unsigned short *selector_ptr)
{  DPR_DWORD ret;
  MYPRINTF0("dpmi_alloc_selector\n");
  pregs.x.REG_CX = 1;   /* number of selectors */
  pregs.x.REG_AX = DPMI_ALLOC_SELECTOR;
  ret=make_dpmi_interrupt(&pregs);
  if(ret!=CI_RET_OK)
  {  /* error */
    MYPRINTF0("Error allocating selector\n");
  }
  else
  {
    MYPRINTF1("Value of selector: %04lxh\n",pregs.x.REG_AX);
    *selector_ptr=pregs.x.REG_AX;
  }
  return(ret);
}

/*
+-------------------------------------------------------------------+
| dpmi_free_selector                                                |
| (free selector)                                                   |
+-------------------------------------------------------------------+
*/

static void dpmi_free_selector(unsigned short selector)
{  short int ret;

MYPRINTF1("dpmi_free_selector %04lxh\n",selector);
pregs.x.REG_BX = selector;
pregs.x.REG_AX = DPMI_FREE_SELECTOR;
ret=make_dpmi_interrupt(&pregs);
if(ret!=CI_RET_OK)
{  /* error */
	MYPRINTF0("Error free selector\n");
}
}

/*
+-------------------------------------------------------------------+
| dpmi_map_physical_address_to_ptr                                  |
| (map physical address to far pointer)                             |
+-------------------------------------------------------------------+
*/

static DPR_DWORD dpmi_map_physical_address_to_ptr(
												  unsigned long physical_address,
												  unsigned short *selector_ptr,
												  unsigned long limit,
												  struct CP5613_FP CI_FAR *ptr)
{  DPR_DWORD ret;

MYPRINTF0("dpmi_map_physical_address_to_ptr\n");

/* get selector */
ret=dpmi_alloc_selector(selector_ptr);
if(ret!=CI_RET_OK)
{  /* error allocating selector */
	return(ret);
}

/* set base */
ret=dpmi_set_selector_base(*selector_ptr,physical_address);
if(ret!=CI_RET_OK)
{  /* error setting base */
	dpmi_free_selector(*selector_ptr);
	return(ret);
}

/* set limit */
ret=dpmi_set_selector_limit(*selector_ptr,limit);
if(ret!=CI_RET_OK)
{  /* error setting limit */
	dpmi_free_selector(*selector_ptr);
	return(ret);
}

/* make far pointer */
ptr->selector=*selector_ptr;
ptr->offset=0;
return(CI_RET_OK);
}

/*
+-------------------------------------------------------------------+
| dpmi_set_selector_base                                            |
| (set base of selector)                                            |
+-------------------------------------------------------------------+
*/

static DPR_DWORD dpmi_set_selector_base(unsigned short selector,
										unsigned long base)
{  DPR_DWORD ret;

MYPRINTF2("dpmi_set_selector_base: selector %04lxh, base %04lxh\n",selector,base);
pregs.x.REG_DX = LOWORD(base);
pregs.x.REG_CX = HIWORD(base);
pregs.x.REG_BX = selector;
pregs.x.REG_AX = DPMI_SET_SELECTOR_BASE;
ret=make_dpmi_interrupt(&pregs);
if(ret!=CI_RET_OK)
{  /* error */
	MYPRINTF0("Error setting selector base\n");
}
return(ret);
}

/*
+-------------------------------------------------------------------+
| dpmi_set_selector_limit                                           |
| (set limit of selector)                                           |
+-------------------------------------------------------------------+
*/

static DPR_DWORD dpmi_set_selector_limit(unsigned short selector,
										 unsigned long limit)
{  DPR_DWORD ret;

MYPRINTF0("dpmi_set_selector_limit\n");
MYPRINTF2("Setting selector %04lxh, limit %04lxh\n",selector,limit);
pregs.x.REG_DX = LOWORD(limit);
pregs.x.REG_CX = HIWORD(limit);
pregs.x.REG_BX = selector;
pregs.x.REG_AX = DPMI_SET_SELECTOR_LIMIT;
ret=make_dpmi_interrupt(&pregs);
if(ret!=CI_RET_OK)
{  /* error */
	MYPRINTF0("Error setting selector limit\n");
}
return(ret);
}

/*
+-------------------------------------------------------------------+
| make_dpmi_interrupt                                               |
| (call dpmi interrupt)                                             |
+-------------------------------------------------------------------+
*/

static DPR_DWORD make_dpmi_interrupt(union REGS *pregs)
{  
	MYPRINTF0("make_dpmi_interrupt\n");
	
	/* call interrupt */
	int386( DPMI_INT, pregs, pregs);
	if( pregs->x.REG_CFLAG )
	{
		MYPRINTF1("DPMI: cflag: %2xh\n",pregs->x.REG_CFLAG);
		return(CI_RET_START_CP_RESOURCES_INT);
	}
	return(CI_RET_OK);
} 
#endif

/*
+-------------------------------------------------------------------+
| get_file_size                                                     |
| (return size of specified file)                                   |
+-------------------------------------------------------------------+
*/

static unsigned long get_file_size(const char *filename)
{  FILE *fp;
unsigned long length=0;

MYPRINTF0("get_file_size\n");

fp=fopen(filename,"rb");
if(fp!=(FILE *)NULL)
{
	fseek(fp,0,2);
	length=ftell(fp);
	fclose(fp);
}
return(length);
}

#ifndef linux
/*
+-------------------------------------------------------------------+
| make_pci_interrupt                                                |
| (call pci interrupt)                                              |
+-------------------------------------------------------------------+
*/

static DPR_DWORD make_pci_interrupt(union REGS *pregs)
{  
	MYPRINTF0("make_pci_interrupt\n");
	
	/* call interrupt */
	int386( PCI_INT, pregs, pregs);
	if( (pregs->x.REG_CFLAG) | (pregs->h.REG_AH) )
	{
		MYPRINTF2("PCI: cflag: %2xh;  ah: %2xh\n",pregs->x.REG_CFLAG,pregs->h.REG_AH);
		return(CI_RET_START_CP_NOT_FOUND);
	}
	return(CI_RET_OK);
} 

/*
+-------------------------------------------------------------------+
| pci_bios_available                                                |
| (check if BIOS is available)                                      |
+-------------------------------------------------------------------+
*/

static DPR_DWORD pci_bios_available(void)
{  DPR_DWORD ret;

MYPRINTF0("pci_bios_available\n");
pregs.h.REG_AH=PCI_FUNCTION_ID;  /* BIOS: function ID for PCI calls  */
pregs.h.REG_AL=PCI_BIOS_PRESENT; /* function: PCI BIOS present?      */
ret=make_pci_interrupt(&pregs);
if( (ret!=CI_RET_OK) | (((pregs.x.REG_DX)&0x0000ffffl) != 0x4350))
{  /* 'CP' part of 'PCI' */
	MYPRINTF0("PCI-BIOS not found\n");
	return(CI_RET_START_CP_NOT_FOUND);
}
else
{
	MYPRINTF0("PCI-BIOS found\n");
	MYPRINTF2("PCI-BIOS Version:    %2x.%02x\n",pregs.h.REG_BH,
		pregs.h.REG_BL);
	MYPRINTF1("PCI-BIOS Bus number: %2xh\n",pregs.h.REG_CL);
	MYPRINTF1("PCI-BIOS HW Mech.:   %2xh\n",pregs.h.REG_AL);
	return(CI_RET_OK);
}
}
#endif

/*
+-------------------------------------------------------------------+
| now following all calls of the CI interface                       |
| (attention: I've only implemented what's really necessary for     |
|             the DP interface)                                     |
+-------------------------------------------------------------------+
*/

DPR_DWORD DP_CODE_ATTR CI_open(DPR_DWORD DP_MEM_ATTR *user_handle,
			       const DPR_STRING DP_MEM_ATTR  *cp_name)
{  DPR_DWORD               ret;
CI_DATA_BUFFER_T CI_FAR *cc_send_ptr;
unsigned long           index;

MYPRINTF0("CI_open\n");

/* 1. I don't care about cp_name      */
/* 2. to avoid compiler warnings      */
/* -> copy cp_name to internal buffer */
my_dummy_device_name[DEVICE_NAME_LENGTH-1]=0x00;
memcpy(my_dummy_device_name,cp_name,DEVICE_NAME_LENGTH-1);

/* get free buffer within DPRAM */
index=dpram_buffer_get_index();
if(index==CI_BLOCK_COUNT)
{  /* all buffers in DPRAM occupied */
	return(CI_RET_SEND_NO_BUFFER_AVAILABLE);
}
cc_send_ptr=(CI_DATA_BUFFER_T CI_FAR *)&
(ptr_dpram->cc.block[index]);

/* initialize block (for DP, only one block for one request necessary) */
cc_send_ptr->next_block=CI_NIL;
cc_send_ptr->next_request=CI_NIL;
cc_send_ptr->usr_header.opcode=OPC_ADMIN_OPEN;
cc_send_ptr->usr_header.user_handle=my_open_user_handle;

/* now call interrupt */
cp5613_make_interrupt(index);

/* next step: wait for response from CP5613 */
ret=cp5613_wait_for_response(1,&index);   /* wait for max. one second */
if(ret!=CI_RET_OK)
{  /* no response from CP5613 */
	MYPRINTF1("CI_open=%ld\n",ret);
	dpram_buffer_release_index(index);
	return(ret);
}
else
{
	dpram_buffer_release_index(index);
}

/* return address of dpram */
ptr_to_dppart_of_dpram=&(ptr_dpram->dp);
MYDISPLAY("dpram user",(struct CP5613_FP CI_FAR *)&ptr_to_dppart_of_dpram);
MYPRINTF1("ptr_to_dppart_of_dpram = %x\n",(unsigned)&ptr_to_dppart_of_dpram);

/* return user handle */
*user_handle=my_open_user_handle;
my_open_user_handle++;

MYPRINTF0("CI_open=CI_RET_OK\n");
return(CI_RET_OK);
}

DPR_DWORD DP_CODE_ATTR CI_close(DPR_DWORD user_handle)
{
	MYPRINTF0("CI_close\n");
	
	/* 1. I don't care about user_handle      */
	/* 2. to avoid compiler warnings          */
	/* -> copy user_handle to internal buffer */
	my_dummy_user_handle=user_handle;
	
	MYPRINTF0("CI_close=CI_RET_OK\n");
	return(CI_RET_OK);
}

DPR_DWORD DP_CODE_ATTR CI_send(CI_REQUEST_T DP_MEM_ATTR *ci_request)
{
	CI_DATA_BUFFER_T CI_FAR *cc_send_ptr;
	unsigned long           index;
	
	MYPRINTF0("CI_send\n");
	
	/* get free buffer within DPRAM */
	index=dpram_buffer_get_index();
	if(index==CI_BLOCK_COUNT)
	{  /* all buffers in DPRAM occupied */
		return(CI_RET_SEND_NO_BUFFER_AVAILABLE);
	}
	cc_send_ptr=(CI_DATA_BUFFER_T CI_FAR *)&
		(ptr_dpram->cc.block[index]);
	
	/* initialize block (for DP, only one block for one request necessary) */
	cc_send_ptr->usr_header.user_handle=ci_request->header.user_handle;
	cc_send_ptr->usr_header.opcode=ci_request->header.opcode;
	cc_send_ptr->usr_header.timeout=ci_request->header.timeout;
	cc_send_ptr->usr_header.order_id=ci_request->header.order_id;
	cc_send_ptr->usr_header.buf_length=ci_request->header.buf_length;
	cc_send_ptr->usr_header.buf_fill_length=ci_request->header.buf_fill_length;
	CI_MEMCPY((char CI_FAR *)cc_send_ptr->data,(char CI_FAR *)ci_request->data,
		ci_request->header.buf_fill_length);
	cc_send_ptr->next_block=CI_NIL;
	cc_send_ptr->next_request=CI_NIL;
	
	/* now call interrupt */
	cp5613_make_interrupt(index);
	
	MYPRINTF0("CI_send=CI_RET_OK\n");
	return(CI_RET_OK);
}

DPR_DWORD DP_CODE_ATTR CI_receive(CI_REQUEST_T DP_MEM_ATTR *ci_request)
{  DPR_DWORD               ret;
CI_DATA_BUFFER_T CI_FAR *cc_receive_ptr;
unsigned long           index;

MYPRINTF0("CI_receive\n");

/* next step: wait for response from CP5613 */
ret=cp5613_wait_for_response(ci_request->header.timeout,&index);
if(ret!=CI_RET_OK)
{  /* no response from CP5613 */
	MYPRINTF1("CI_receive=%ld\n",ret);
	return(ret);
}

/* get index of buffer within DPRAM, where data is stored in */
cc_receive_ptr=(CI_DATA_BUFFER_T CI_FAR *)&
(ptr_dpram->cc.block[index]);

/* return user data */
ci_request->header.order_id=cc_receive_ptr->usr_header.order_id;
ci_request->header.response=cc_receive_ptr->usr_header.response;
ci_request->header.opcode=cc_receive_ptr->usr_header.opcode;
CI_MEMCPY((char CI_FAR *)ci_request->data,(char CI_FAR *)cc_receive_ptr->data,
		  cc_receive_ptr->usr_header.buf_fill_length);

/* now release buffer in DPRAM */
dpram_buffer_release_index(index);

MYPRINTF0("CI_receive=CI_RET_OK_DATA\n");
return(CI_RET_OK_DATA);
}

DPR_DWORD DP_CODE_ATTR CI_init_sema(DPR_DWORD user_handle,
									DPR_DWORD sema_type,
									void DP_MEM_ATTR *sema_handle)
{
	MYPRINTF0("CI_init_sema\n");
	
	/* 1. I don't care about user_handle, sema_type and sema_handle */
	/* 2. to avoid compiler warnings                                */
	/* -> copy all parameters to internal buffer                    */
	my_dummy_user_handle=user_handle;
	my_dummy_sema_type=sema_type;
	my_dummy_init_sema_handle=sema_handle;
	
	return(CI_RET_NOT_IMPLEMENTED);
}

DPR_DWORD DP_CODE_ATTR CI_delete_sema(DPR_DWORD user_handle,
									  DPR_DWORD sema_handle )
{
	MYPRINTF0("CI_delete_sema\n");
	
	/* 1. I don't care about user_handle and sema_handle */
	/* 2. to avoid compiler warnings                     */
	/* -> copy all parameters to internal buffer         */
	my_dummy_user_handle=user_handle;
	my_dummy_delete_sema_handle=sema_handle;
	
	return(CI_RET_NOT_IMPLEMENTED);
}

DPR_DWORD DP_CODE_ATTR CI_start_cp(const DPR_STRING DP_MEM_ATTR *cp_name,
				   const DPR_STRING DP_MEM_ATTR *firmware,
				   const DPR_STRING DP_MEM_ATTR *database,
				   DPR_WORD mode)
{  DPR_DWORD       ret;

MYPRINTF0("CI_start_cp\n");

/* 1. I don't care about cp_name and mode      */
/* 2. to avoid compiler warnings               */
/* -> copy cp_name and mode to internal buffer */
my_dummy_device_name[DEVICE_NAME_LENGTH-1]=0x00;
memcpy(my_dummy_device_name,cp_name,DEVICE_NAME_LENGTH-1);
my_dummy_mode=mode;

#ifndef linux
/* check if PCI BIOS is available */
ret=pci_bios_available();
if(ret!=CI_RET_OK)
{  /* PCI BIOS not available */
	return(ret);
}
#endif

#ifdef linux

/* check if CP5613 is available */
ret=cp5613_available(&physical_address_plx,
		     &physical_address_download_area,
		     &physical_address_dpram);
if(ret!=CI_RET_OK)
{  /* CP5613 not available */
	return(ret);
}

#else
/* check if CP5613 is available */
ret=cp5613_available();
if(ret!=CI_RET_OK)
{  /* CP5613 not available */
	return(ret);
}

/* read physical address of plx */
ret=cp5613_read_config_long(CP5613_REGISTER_PLX,
							&physical_address_plx);
if(ret!=CI_RET_OK)
{  /* error reading address of plx */
	return(ret);
}

/* read physical address of download area */
ret=cp5613_read_config_long(CP5613_REGISTER_DOWNLOAD_AREA,
							&physical_address_download_area);
if(ret!=CI_RET_OK)
{  /* error reading address of download area */
	return(ret);
}

/* read physical address of DPRAM */
ret=cp5613_read_config_long(CP5613_REGISTER_DPRAM,
							&physical_address_dpram);
if(ret!=CI_RET_OK)
{  /* error reading address of DPRAM */
	return(ret);
}
#endif

#ifdef CI_DPMI
/* map physical address of plx to pointer */
ret=dpmi_map_physical_address_to_ptr(physical_address_plx,
									 &selector_plx,
									 sizeof(CP5613_PLX_T),
									 (struct CP5613_FP CI_FAR *)
									 &ptr_plx);
if(ret!=CI_RET_OK)
{  /* error mapping physical address of plx to pointer */
	return(ret);
}

/* map physical address of download area to pointer */
ret=dpmi_map_physical_address_to_ptr(physical_address_download_area,
									 &selector_download_area,
									 sizeof(CP5613_DOWNLOAD_T),
									 (struct CP5613_FP CI_FAR *)
									 &ptr_download_area);
if(ret!=CI_RET_OK)
{  /* error mapping physical address of download area to pointer */
	dpmi_free_selector(selector_plx);
	return(ret);
}

/* map physical address of DPRAM to pointer */
ret=dpmi_map_physical_address_to_ptr(physical_address_dpram,
									 &selector_dpram,
									 sizeof(DPR_CP5613_ALL_T),
									 (struct CP5613_FP CI_FAR *)
									 &ptr_dpram);
if(ret!=CI_RET_OK)
{  /* error mapping physical address of DPRAM to pointer */
	dpmi_free_selector(selector_plx);
	dpmi_free_selector(selector_download_area);
	return(ret);
}
#else
ptr_plx=(CP5613_PLX_T *)physical_address_plx;
ptr_download_area=(CP5613_DOWNLOAD_T *)physical_address_download_area;
ptr_dpram=(DPR_CP5613_ALL_T *)physical_address_dpram;
#endif

MYDISPLAY("dpram",(struct CP5613_FP CI_FAR *)&ptr_dpram);
MYDISPLAY("download_area",(struct CP5613_FP CI_FAR *)&ptr_download_area);
MYDISPLAY("plx",(struct CP5613_FP CI_FAR *)&ptr_plx);

/* next step: copy firmware and ldb to download area */
if((firmware==NULL)&&(database==NULL))
{
	ret=cp5613_load_fw_ldb("fw_5613.bin","dp_demo.ldb");
}
else
{
	if(firmware==NULL)
	{
		ret=cp5613_load_fw_ldb("fw_5613.bin",database);
	}
	else
	{
		ret=cp5613_load_fw_ldb(firmware,database);
	}
}
if(ret!=CI_RET_OK)
{  /* error loading firmware or database */
	cp5613_shut();
	return(ret);
}

/* last step: enable CP5613 */
ret=cp5613_enable();
if(ret!=CI_RET_OK)
{  /* error starting CP5613 */
	cp5613_shut();
	return(ret);
}

/* now initialize internal array that shows, if buffer in */
/* DPRAM is free or occupied                              */
for(ret=0;ret<CI_BLOCK_COUNT;ret++)
{
	dpram_buffer[ret]=DPRAM_BUFFER_OCCUPIED;
}
dpram_buffer[0]=DPRAM_BUFFER_FREE;

return(CI_RET_OK);
}

DPR_DWORD DP_CODE_ATTR CI_reset_cp(const DPR_STRING DP_MEM_ATTR *cp_name,
				   DPR_DWORD reset_mode )
{
	MYPRINTF0("CI_reset_cp\n");
	
	/* 1. I don't care about cp_name and reset_mode */
	/* 2. to avoid compiler warnings                */
	/* -> copy all parameters to internal buffer    */
	my_dummy_device_name[DEVICE_NAME_LENGTH-1]=0x00;
	memcpy(my_dummy_device_name,cp_name,DEVICE_NAME_LENGTH-1);
	my_dummy_reset_mode=reset_mode;
	
	cp5613_shut();
	return(CI_RET_OK);
}

DPR_DWORD DP_CODE_ATTR CI_get_dp_access(DPR_DWORD user_handle,
										DPR_DWORD timeout)
{
	MYPRINTF0("CI_get_dp_access\n");
	
	/* 1. I don't care about user_handle and timeout */
	/* 2. to avoid compiler warnings                 */
	/* -> copy all parameters to internal buffer     */
	my_dummy_user_handle=user_handle;
	my_dummy_timeout=timeout;
	
	return(CI_RET_OK);
}

DPR_DWORD DP_CODE_ATTR CI_release_dp_access(DPR_DWORD user_handle)
{
	MYPRINTF0("CI_release_dp_access\n");
	
	/* 1. I don't care about user_handle         */
	/* 2. to avoid compiler warnings             */
	/* -> copy all parameters to internal buffer */
	my_dummy_user_handle=user_handle;
	
	return(CI_RET_OK);
}

DPR_DWORD DP_CODE_ATTR CI_get_pointer(DPR_DWORD user_handle,
									  DPR_CP5613_DP_T volatile DP_MEM_ATTR **dpr)
{
	MYPRINTF0("CI_get_pointer\n");
	
	/* 1. I don't care about user_handle         */
	/* 2. to avoid compiler warnings             */
	/* -> copy all parameters to internal buffer */
	my_dummy_user_handle=user_handle;
	
	/* return address of dpram */
#ifdef CI_DPMI
	*(DPR_CP5613_DP_T volatile DP_MEM_ATTR **)dpr=
		(DPR_CP5613_DP_T volatile DP_MEM_ATTR *)&ptr_to_dppart_of_dpram;
#else
	*dpr=ptr_to_dppart_of_dpram;
#endif
	return(CI_RET_OK);
}

DPR_DWORD DP_CODE_ATTR  CI_fast_logic_on( DPR_DWORD        user_handle,
										 DPR_WORD         fast_logic_id,
										 DP_FAST_LOGIC_T  *fast_logic )
{
	my_dummy_user_handle=user_handle;
	my_dummy_fast_logic_id=fast_logic_id;
	my_dummy_fast_logic=fast_logic;
	return(CI_RET_NOT_IMPLEMENTED);
}

DPR_DWORD DP_CODE_ATTR  CI_fast_logic_off( DPR_DWORD        user_handle,
										  DPR_WORD         fast_logic_id )
{
	my_dummy_user_handle=user_handle;
	my_dummy_fast_logic_id=fast_logic_id;
	return(CI_RET_NOT_IMPLEMENTED);
}

DPR_DWORD DP_CODE_ATTR CI_get_err_txt ( DPR_DWORD  error,                              /* in  */
									   const  DPR_STRING DP_MEM_ATTR *language,              /* in  */
									   DPR_STRING DP_MEM_ATTR text[DP_ERR_TXT_SIZE] ) /* out */
{
    my_dummy_error=error;
    my_dummy_language=(DPR_STRING DP_MEM_ATTR *)language;
    my_dummy_text=text;
	return(CI_RET_NOT_IMPLEMENTED);
}


/****************************************************************************/
/*     Copyright (C) Siemens AG 1998  All Rights Reserved.                  */
/****************************************************************************/
