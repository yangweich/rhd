/****************************************************************************/
/*    Copyright (C) SIEMENS AG 1989..1998 All Rights Reserved.Confidential  */
/****************************************************************************/
/*     Projekt       : CP5613                                               */
/*                                                                          */
/*     Modulname     : CI_DRIV.H                                            */
/*                                                                          */
/*     Ersteller     : Gerhard Marquardt   A&D PT 2                         */
/*                                                                          */
/****************************************************************************/
/****************************************************************************/
/* Task / Description:                                                      */
/*                                                                          */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
/*   Modification                                                           */
/*                                                                          */
/*   Date       Person       Modification                                   */
/*   05.04.98   GM           file created                                   */
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



#define CP_NAME             "cp5613"

#define FILE_DEVICE_CI      0x880BUL

/*#define CTL_CODE( DeviceType, Function, Method, Access ) (                 \
    ((DeviceType) << 16) | ((Access) << 14) | ((Function) << 2) | (Method) \
)*/



#define CTL_GET_FUNC( code )  ( (code >> 2) & 0x00000FFFUL )

#define IOCTL_A_START \
    CTL_CODE(FILE_DEVICE_CI, 0x805, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_RESET \
    CTL_CODE(FILE_DEVICE_CI, 0x806, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_OPEN \
    CTL_CODE(FILE_DEVICE_CI, 0x807, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_CLOSE \
    CTL_CODE(FILE_DEVICE_CI, 0x808, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_INIT_SEMA \
    CTL_CODE(FILE_DEVICE_CI, 0x809, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_SEND \
    CTL_CODE(FILE_DEVICE_CI, 0x80a, METHOD_BUFFERED, FILE_ANY_ACCESS)


#define IOCTL_A_SEND_RESERVED \
    CTL_CODE(FILE_DEVICE_CI, 0x80b, METHOD_BUFFERED, FILE_ANY_ACCESS)

//#define IOCTL_A_SEND_RESERVED  CTL_CODE(FILE_DEVICE_CI, 0x80b, METHOD_IN_DIRECT, FILE_ANY_ACCESS)

#define IOCTL_A_RECEIVE \
    CTL_CODE(FILE_DEVICE_CI, 0x80c, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_CANCEL \
    CTL_CODE(FILE_DEVICE_CI, 0x80d, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_INIT_EVENT \
    CTL_CODE(FILE_DEVICE_CI, 0x80e, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_DEL_SEMA \
    CTL_CODE(FILE_DEVICE_CI, 0x80f, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_RECEIVE_EVENT \
    CTL_CODE(FILE_DEVICE_CI, 0x810, METHOD_BUFFERED, FILE_ANY_ACCESS)


#define IOCTL_A_GET_DP_ACCESS \
    CTL_CODE(FILE_DEVICE_CI, 0x811, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_RELEASE_DP_ACCESS \
    CTL_CODE(FILE_DEVICE_CI, 0x812, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_GET_POINTER \
    CTL_CODE(FILE_DEVICE_CI, 0x813, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_A_GET_CP_CFG \
    CTL_CODE(FILE_DEVICE_CI, 0x814, METHOD_BUFFERED, FILE_ANY_ACCESS)


#ifndef __GNUC__
#pragma pack(push, 1)
#define GNUC__PACK
#else
#define GNUC__PACK __attribute__ ((packed))
#endif



typedef struct  cp_para_event_s
{
 HANDLE            ObjectHandle; /* valid for the calling process */
} GNUC__PACK cp_para_event_t;

typedef struct  cp_para_sema_s
{
 DPR_DWORD         type;
 HANDLE            ObjectHandle; /* valid for the calling process */
} GNUC__PACK cp_para_sema_t;


typedef struct  cp_para_open_s
{
 DPR_STRING         log_device[CI_MAX_PATH];
 DPR_DWORD          user_handle;
} GNUC__PACK cp_para_open_t;

typedef struct  cp_para_reset_s
{
 DPR_STRING         log_device[CI_MAX_PATH];
 DPR_DWORD          mode;
} GNUC__PACK cp_para_reset_t;

typedef struct  cp_para_access_s
{
 DPR_CP5613_DP_T    *dp_dpr;
} GNUC__PACK cp_para_access_t;







#define ACTION_START_CP        0x00000000U
#define ACTION_CP_START_OK     0x00000001U
#define ACTION_CP_START_ERROR  0x00000002U
#define ACTION_CP_CONNECT      0x00000003U
#define ACTION_CP_DISCONNECT   0x00000004U

typedef struct  cp_para_start_s
{
 DPR_DWORD          action;
// DPR_DWORD          cp_id;
 DPR_STRING         firmware[CI_MAX_PATH];
 DPR_STRING         database[CI_MAX_PATH];
 DPR_STRING         log_device[CI_MAX_PATH];
 DPR_STRING         log_name[CI_MAX_PATH];
 DPR_CP5613_ALL_T   *dpr;
 CP5613_PLX_T       *plx;
 CP5613_DOWNLOAD_T  *down;
} GNUC__PACK cp_para_start_t;


typedef struct ci_dll_request_s
{
 CI_REQUEST_HEADER_T     header;
 union 
 {
  cp_para_sema_t      sema;
  cp_para_event_t     event;
  cp_para_access_t    access;
 } u;
} GNUC__PACK ci_dll_request_t;


#ifndef __GNUC__
#pragma pack(pop)
#endif

/* Windows NT status codes */
#define CUSTOMER_STATUS 0x20000000UL
#define CUSTOMER_WARNING ((DPR_LONG)(CUSTOMER_STATUS | 0x80000000UL))

/* codes from ci_ret.h ored with CUSTOMER_WARNING   */
/* except for CI_RET_OK and CI_RET_OK_DATA          */
/* which will be ored with CUSTOMER_STATUS          */

#define CI_STATUS_OK(x) ((DPR_LONG)(x | CUSTOMER_STATUS))
#define CI_STATUS(x)    ((DPR_LONG)(x | CUSTOMER_WARNING))



  




/****************************************************************************/
/*     END OF FILE:      CI_DRV.H                                           */
/****************************************************************************/
