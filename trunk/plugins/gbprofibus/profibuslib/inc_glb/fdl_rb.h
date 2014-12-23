/****************************************************************************/
/*                                                                          */
/*   Project           : SCI                     ,###.   ,###.   #####      */
/*   Filename          : fdl_rb.h                #   #   #   #     #        */
/*   Version           : 1.4                      #      #         #        */
/*   System            : MSDOS/WINDOWS/WinNT       #     #         #        */
/*                                                  #    #         #        */
/*                                               #   #   #   #     #        */
/*                                               `###'   `###'   #####      */
/*                                                                          */
/****************************************************************************/
/*   Function         : Definitions concerning INA960 commands and request  */
/*                      blocks                                              */
/****************************************************************************/
/*                                                                          */
/*         Copyright (C) Siemens AG 1991, 1997. All Rights Reserved         */
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

#ifdef __GNUC__
#define GNUC__PACK __attribute__ ((packed))
#else
#pragma pack(1)
#define GNUC__PACK
#endif

#ifndef UBYTE
#define UBYTE unsigned char
#endif
#ifndef UWORD
#define UWORD unsigned short
#endif
#ifndef ULONG
#define ULONG unsigned long
#endif


/* ------------------- */
/* definitions for FLC */
/* ------------------- */

#define   FLC_Subsys    0x22

#define   DEFAULT_SAP   0xff
#define   EVENT_SAP     64
#define   NO_SEGMENT    0xff
#define   BROADCAST     127             /* broadcast-addr.: rem_add.station, dsap = 63  */
#define   MULTICAST     127             /* multicast-addr.: rem_add.station, dsap != 63 */

#define   LEN_MAX_RECEIVE_BUFFER  255    /* max. buffer-length for a receive telegram    */
#define   LEN_MIN_RECEIVE_BUFFER  32     /* min. receive_l_sdu.length                    */

#define   ALL                     0x7f
#define   SEGMENT_VALID           0x80
#define   SEGMENT_INVALID         0x00
#define   SEGMENT_TYP             0x40

/* example */
/*   access_sap = ALL or sap_number;                                                */
/*   access_station = (ALL or station_number) + (SEGMENT_VALID or SEGMENT_INVALID); */
/*   if SEGMENT_VALID: access_segment = SEGMENT_TYP + segment_number (0 ... 63);    */

/* SERVICE */
#define   SDA_RESERVED            0x00
#define   SDN_RESERVED            0x01
#define   SRD_RESERVED            0x03
#define   CSRD_RESERVED           0x05

/* ROLE */
#define   INITIATOR               0x00            /* only possible by "sap_activate"                                      */
#define   RESPONDER               0x10            /* possible by "sap_activate" and mandatory by "rsap_activate"          */
#define   BOTH_ROLES              0x20            /* only possible by "sap_activate"                                      */
#define   SERVICE_NOT_ACTIVATED   0x30            /* service not activated                                                */


#define   STATION_PASSIVE         0x00
#define   STATION_NON_EXISTANT    0x10
#define   STATION_NON_EXISTENT    0x10
#define   STATION_ACTIVE_READY    0x20
#define   STATION_ACTIVE          0x30

typedef enum { flc_false, flc_true } flc_boolean;

enum  com_class
{
    request                 = 0x00,
		confirm                 = 0x01,
		indication              = 0x02
};

enum  service_code
{
    sda                             = 0x00,       /* Send Data with Acknowledge                                           */
		sdn                             = 0x01,       /* Send Data with no Acknowledge                                        */
		sdn_broadcast                   = 0x7f,       /* only for FDL-indication !!! (signs received broadcast-telegram)      */
		srd                             = 0x03,       /* Send and Request Data                                                */
		csrd                            = 0x05,       /* Cyclic Send and Request Data                                         */
		reply_update_single             = 0x06,       /* Reply Update Single Mode                                             */
		reply_update_multiple           = 0x07,       /* Reply Update Multiple Mode                                           */
		fdl_read_value                  = 0x0b,       /* read busparameter                                                    */
		fdl_set_value                   = 0x0c,       /* set busparameter                                                     */
		sap_activate                    = 0x0e,       /* activate local SAP                                                   */
		rsap_activate                   = 0x11,       /* activate local Responder-SAP                                         */
		sap_deactivate                  = 0x12,       /* deactivate local (R)SAP                                              */
		fdl_reset                       = 0x13,       /* reset PHY and FDL; all FDL-information is lost, exc. last busparam.  */
		mac_reset                       = 0x15,       /* reset for MAC; a part of last valid busparameter will be updated     */
		fdl_event                       = 0x18,       /* only for indication, list of events                                  */
		lsap_status                     = 0x19,       /* requests information of remote-SAP or local-SAP                      */
		fdl_life_list_create_remote     = 0x1a,       /* requests list of intact stations                                     */
		fdl_life_list_create_local      = 0x1b,       /* requests quick-list of intact stations (LAS and GAP will be actual)  */
		fdl_ident                       = 0x1c,       /* requests data of software- and hardware-release                      */
		fdl_read_statistic_ctr          = 0x1d,       /* reads counter values of statistic and resets counter                 */
		fdl_read_las_statistic_ctr      = 0x1e,       /* reads LAS and las_cycle_ctr and resets las_cycle_ctr                 */
		await_indication                = 0x1f,       /* provides resources for indication (sap-dependent)                    */
		withdraw_indication             = 0x20,       /* returnes indication-resources                                        */
		load_routing_table              = 0x21,       /* only for network-connection !!!                                      */
		deactivate_routing_table        = 0x22,       /* only for network-connection !!!                                      */
		get_direct_conn                 = 0x23,       /* gets adress of next station                                          */
};

enum  service_class
{
    low                             = 0x00,
		high                            = 0x01
};

enum  link_status
{
		ok                      = 0x00,               /* ACK. positive                                                        */
		ue                      = 0x01,               /* ACK. negative:   remote-USER/FDL interface error                     */
		rr                      = 0x02,               /* ACK. negative:   no remote resource available                        */
		rs                      = 0x03,               /* ACK. negative:   service or rem_add at remote-lsap not activated     */
		dl                      = 0x08,               /* response-data (l_sdu) low available                                  */
		nr                      = 0x09,               /* ACK. negative:   no response-data at remote-FDL available            */
		dh                      = 0x0a,               /* response-data (l_sdu) high available                                 */
		rdl                     = 0x0c,               /* response-data (l_sdu) low available, but negative-ACK for send-data  */
		rdh                     = 0x0d,               /* response-data (l_sdu) high available, but negative-ACK for send-data */
		ls                      = 0x10,               /* service not activated at local sap                                   */
		na                      = 0x11,               /* no reaction (ACK/RES) from remote-station                            */
		ds                      = 0x12,               /* local FDL/PHY not in token-ring                                      */
		no                      = 0x13,               /* ACK. negative:   not ok (different meanings dependant on service)    */
		lr                      = 0x14,               /* resource of local FDL not available                                  */
		iv                      = 0x15,               /* invalid parameter in request                                         */
		lo                      = 0x20,               /* LOw-prior response-data are sent at this srd                         */
		hi                      = 0x21,               /* HIgh-prior response-data are sent at this srd                        */
		no_data                 = 0x22                /* NO-DATA are sent at this srd                                         */
};

enum    poll_element_entry
{
	    unlocked                = 0x00,               /* polling enabled for this element                                     */
		locked                  = 0x01,               /* element locked for this poll-cycle                                   */
		not_changed             = 0x02                /* the same meaning as "unlocked", but layer 2 doesn't plausible this   */
		/* service; use only, if this element has already been sent and no      */
		/* parameter changed, but actualize receive_l_sdu.length !!!!!!!!!      */
};

enum    station_type
{
	    passive                 = 0x00,
		active                  = 0x01,
		active_fast             = 0x02,
		passive_fast            = 0x03,
		sm_active               = 0x04,               /* possible returned by read_value (PROFIBUS-PA)                        */
		sm_passive              = 0x05
};

enum    baud_rate
{
	    kbaud_9_6               = 0x00,
		kbaud_19_2              = 0x01,
		kbaud_93_75             = 0x02,
		kbaud_187_5             = 0x03,
		kbaud_500               = 0x04,
		kbaud_375               = 0x05,
		kbaud_750               = 0x06,
		/*CT_CHANGE*/
		mbaud_1_5               = 0x07,
		mbaud_3                 = 0x08,
		mbaud_6                 = 0x09,
		mbaud_12                = 0x0a,
		kbaud_45_45             = 0x0c                /* 09-06-1997: a new baud rate */
};

enum    redundancy
{
	    no_redundancy           = 0x00,
		bus_a_highprior         = 0x01,
		bus_b_highprior         = 0x02,
		redundancy_on           = 0x03
};

enum    physical_layer
{
	    rs485                   = 0x00,
		modem                   = 0x01
};

enum    false_bus_parameter             /* "fdl_set_value" : if FDL-User sets false bus_parameter,              */
/* application_block -> l_status = false_bus_parameter + iv;            */
/* that means, high-byte corresponds with number of false parameter     */
{
	    false_hsa                       = 0x100,
		false_ts                        = 0x200,
		false_station_type              = 0x300,
		false_baud_rate                 = 0x400,
		false_medium_red                = 0x500,
		false_retry_ctr                 = 0x600,
		false_default_sap               = 0x700,
		false_network_connection_sap    = 0x800,
		false_tsl                       = 0x900,
		false_tqui                      = 0xa00,
		false_tset                      = 0xb00,
		false_min_tsdr                  = 0xc00,
		false_max_tsdr                  = 0xd00,
		false_ttr                       = 0xe00,
		false_g                         = 0xf00,
		false_in_ring_desired           = 0x1000,
		false_physical_layer            = 0x1100,
		false_ident                     = 0x1200
};



struct  service
{
#ifdef M_DOS
	enum  service_code      code;
#else
	short                   code;
#endif
} GNUC__PACK;


struct  remote_address
{
    UBYTE                    station;
    UBYTE                    segment;              /* if no segment is used, set "NO_SEGMENT"                              */
} GNUC__PACK;

struct  link_service_data_unit
{
    void                    *buffer_ptr;
    UBYTE                    length;
} GNUC__PACK;


struct  application_block
{
    UBYTE                          opcode;         /* class of communication                                               */
    UBYTE                          subsystem;      /* number of source-task (only necessary for MTK-user !!!!!)            */
    UWORD                          id;             /* identification of FDL-USER                                           */
    struct  service                service;        /* identification of service                                            */
    struct  remote_address         loc_add;        /* only for network-connection !!!                                      */
    UBYTE                          ssap;           /* source-service-access-point                                          */
    UBYTE                          dsap;           /* destination-service-access-point                                     */
    struct  remote_address         rem_add;        /* address of the remote-station                                        */
#ifdef M_DOS
    enum    service_class          serv_class;     /* priority of service                                                  */
#else	
    short                          serv_class;     /* priority of service                                                  */
#endif
    struct  link_service_data_unit receive_l_sdu;  /* address and length of received netto-data, exception:                */
    UBYTE                          reserved1;      /* (reserved for FDL !!!!!!!!!!)                                        */
    UBYTE                          reserved2;      /* (reserved for FDL !!!!!!!!!!)                                        */
    struct  link_service_data_unit send_l_sdu;     /* address and length of send-netto-data, exception:                    */
	/* 1. csrd                 : length means number of POLL-elements       */
	/* 2. await_indication     : concatenation of application-blocks and    */
	/*    withdraw_indication  : number of application-blocks               */
#ifdef M_DOS
    enum    link_status            l_status;       /* link-status of service or update_state for srd-indication            */
#else
    short                          l_status;       /* link-status of service or update_state for srd-indication            */
#endif
    UBYTE                          reserved3[4];   /* for concatenated lists       (reserved for FDL !!!!!!!!!!)           */
} GNUC__PACK;

typedef struct
{
    UWORD                           reserved [2];
    UBYTE                           length;
    UWORD                           user;
    UBYTE                           rb_type;
    UBYTE                           priority;
    UBYTE                           reserved_1;
    UWORD                           reserved_2;
    UBYTE                           subsystem;
    UBYTE                           opcode;
    UWORD                           response;
    UWORD                           fill_length_1;
    UBYTE                           reserved_3;
    UWORD                           seg_length_1;
    UWORD                           offset_1;
    UWORD                           reserved_4;
    UWORD                           fill_length_2;
    UBYTE                           reserved_5;
    UWORD                           seg_length_2;
    UWORD                           offset_2;
    UWORD                           reserved_6;
	
} GNUC__PACK rb2_header_type;

typedef struct
{
    rb2_header_type                 rb2_header;
    struct application_block        application_block;
    UBYTE                           reserved[12];
    UBYTE                           reference[2];
    UBYTE                           user_data_1[260];
    UBYTE                           user_data_2[260];
} GNUC__PACK fdl_rb;

typedef fdl_rb flc_rb;

/* Moegliche Strukturen der Datenpuffer */

struct  user_poll_element
{
    UBYTE                           dsap;          /* destination-service-access-point                                     */
    struct  remote_address          rem_add;       /* address of the remote-station                                        */
#ifdef M_DOS
    enum    service_class           serv_class;    /* priority of send-telegram                                            */
#else
    short                           serv_class;    /* priority of send-telegram                                            */
#endif
    struct  link_service_data_unit  receive_l_sdu; /* request: length means buffer-length in byte                          */
	/* confirm: length means length of received netto-data                  */
    struct  link_service_data_unit  send_l_sdu;    /* address and length of send-netto-data                                */
    UBYTE                           reserved;      /* reserved for FDL !!!!!                                               */
#ifdef M_DOS
    enum    link_status             l_status;      /* link-status of poll-element-service                                  */
    enum    poll_element_entry      entry;         /* locks or unlocks a poll-element                                      */
#else
    short                           l_status;      /* link-status of poll-element-service                                  */
    short                           entry;         /* locks or unlocks a poll-element                                      */
#endif
    UBYTE                           reserved_2;    /* reserved for FDL !!!!!                                               */
} GNUC__PACK;


struct  ident
{
    UBYTE   reserved_header[8];                    /* reserved for FDL !!!!!                                               */
#ifdef M_DOS
    UBYTE   ident[202];
#else
    UBYTE   ident_info[202];
#endif
    UBYTE   response_telegram_length;              /* reserved for FDL !!!!!                                               */
} GNUC__PACK;


struct  bus_parameter_block
{
    UBYTE                    hsa;                  /* highest station-address                                              */
	/* range of values:  2 ... 126                                          */
    UBYTE                    ts;                   /* FDL-address of this station                                          */
	/* range of values:  0 ... 126                                          */
#ifdef M_DOS
    enum    station_type    station_type;          /* active, passive                                                      */
    enum    baud_rate       baud_rate;             /* transmission rate                                                    */
    enum    redundancy      medium_red;            /* availability of redundant media                                      */
#else
    short                   station_type;          /* active, passive                                                      */
    short                   baud_rate;             /* transmission rate                                                    */
    short                   medium_red;            /* availability of redundant media                                      */
#endif
    UWORD                    retry_ctr;            /* retry-number of requestor, if no reaction of responder               */
	/* range of values:  1 ... 8                                            */
    UBYTE                    default_sap;          /* Default SAP if no address-extension is used                          */
	/* range of values:  2 ... 62                                           */
    UBYTE                    network_connection_sap;/* number of sap for network-connection (only for network-connections) */
	/* range of values:  2 ... 62                                           */
    UWORD                    tsl;                  /* SLOT-time:                                                           */
	/* range of values:  2 exp 0 ... (2 exp 16) - 1   BIT-times             */
    UWORD                    tqui;                 /* Transmitter-Fall-Time / Repeater-Switch-Time:                        */
	/* range of values:  0 ... (2 exp 8) - 1 BIT-times                      */
    UWORD                    tset;                 /* setup-time                                                           */
	/* range of values:  0 ... (2 exp 8) - 1 BIT-times                      */
    UWORD                    min_tsdr;             /* smallest STATION-DELAY-time:                                         */
	/* range of values:  2 exp 0 ... (2 exp 16) - 1   BIT-times             */
    UWORD                    max_tsdr;             /* largest STATION-DELAY-time:                                          */
	/* range of values:  2 exp 0 ... (2 exp 16) - 1   BIT-times             */
    ULONG                    ttr;                  /* TARGET-ROTATION-time:                                                */
	/* range of values:  2 exp 0 ... (2 exp 24) - 1   BIT-times             */
    UBYTE                    g;                    /* GAP-UPDATE-factor: in multiples of ttr                               */
	/* range_of_values:  1 ... 100                                          */
#ifdef M_DOS
    flc_boolean             in_ring_desired;       /* request entrance into the token-ring                                 */
    enum    physical_layer  physical_layer;        /* RS485, modem                                                         */
#else
	short		            in_ring_desired;       /* request entrance into the token-ring                                 */
	short                   physical_layer;        /* RS485, modem                                                         */
#endif
    struct  ident           ident;                 /* vendor-name, controller_type, version of hardware and software       */
} GNUC__PACK;

struct  fdl_sap
{
    UWORD                  user_id;                /* identification for user                                              */
    UBYTE                  max_l_sdu_length;       /* maximal length of netto_data for this sap                            */
    UBYTE                  access_sap;             /* reserved destination_sap                                             */
    UBYTE                  access_station;         /* reserved destination_address                                         */
    UBYTE                  access_segment;         /* reserved destination_segment                                         */
    UBYTE                  sda;                    /* ... .sda = "ROLE"; sda unused by service "rsap_activate"             */
    UBYTE                  sdn;                    /* look sda; sdn unused by service "rsap_activate"                      */
    UBYTE                  srd;                    /* look sda;                                                            */
    UBYTE                  csrd;                   /* if you want to activate csrd for this SAP, type: ... .csrd = "ROLE"; */
	/* (only INITIATOR or SERVICE_NOT_ACTIVATED possible)                   */
	/* csrd unused by service "rsap_activate"                               */
    void                  *rup_l_sdu_ptr_low;      /* reserved for "sap_deactivate"; USER gets rup-buffer returned         */
    void                  *rup_l_sdu_ptr_high;     /* reserved for "sap_deactivate"; USER gets rup-buffer returned         */
    UBYTE                  reserved[22];           /* reserved for FDL !!!!!                                               */
} GNUC__PACK;


/**********************************************************************/
/****************   fdl_event (returned by fma_indication) ************/
/****************   await_indication (for DSAP = EVENT_SAP)************/
/**********************************************************************/


struct  event_ctr
{
    UWORD                    threshold;                      /* if counter achieves threshold, a fma_indication is initiated         */
    UWORD                    counter;
} GNUC__PACK;

struct  event_indication
{
    struct  event_ctr       time_out;
    struct  event_ctr       not_syn;
    struct  event_ctr       uart_error;
    struct  event_ctr       out_of_ring;
    struct  event_ctr       sdn_not_indicated;
    struct  event_ctr       duplicate_address;
    struct  event_ctr       hardware_error;
    struct  event_ctr       mac_error;
} GNUC__PACK;


/**********************************************************************/
/****************   fdl_read_statistic_ctr  ***************************/
/**********************************************************************/

struct  statistic_ctr_list
{
	/* error-counter */
    UWORD                    invalid_start_delimiter_ctr;
    UWORD                    invalid_fcb_fcv_ctr;
    UWORD                    invalid_token_ctr;
    UWORD                    collision_ctr;
    UWORD                    wrong_fcs_or_ed_ctr;
    UWORD                    frame_error_ctr;
    UWORD                    char_error_ctr;
    UWORD                    retry_ctr;
	/* reference-counter */
    ULONG                    start_delimiter_ctr;
    ULONG                    stop_receive_ctr;
    ULONG                    send_confirmed_ctr;
    ULONG                    send_sdn_ctr;
} GNUC__PACK;


/*
*	Prototypings
*	************
*/

/* old prototypes for MSDOS and Windows 3.x */

#ifdef M_DOS

#ifndef MM
#define MM far
#endif

int MM ihi_get_dev_list(short MM *,unsigned short MM *,char MM *);
int MM ihi_open(short);
int MM ihi_open_dev(unsigned short,char MM *);
int MM ihi_write(int,void MM *);
int MM ihi_read(int,short,void MM * MM *);
int MM ihi_close(int);


#ifdef M_WINDOWS
#define WM_SINEC                 500
int MM SetSinecHWnd (int, HWND);
int MM SetSinecHWndMsg(int, HWND, unsigned int);
#endif

#else /* M_DOS */
/* prototypes for Win32 */

#ifdef __cplusplus	
extern "C" {
#endif
	
	INT WINAPI SCP_open ( CHAR *);
	INT WINAPI SCP_close ( INT );
	INT WINAPI SCP_send ( INT, USHORT, char *);
	INT WINAPI SCP_receive ( INT, SHORT, SHORT *, SHORT, CHAR *);
	INT WINAPI SetSinecHWndMsg( INT, HANDLE, ULONG );
	INT WINAPI SetSinecHWnd( INT,HANDLE);
	INT WINAPI SCP_get_errno( VOID );
	
#ifdef __cplusplus
}
#endif

#define WM_SINEC                 (WM_USER+500)

#endif /* M_DOS */

#pragma pack()


/****************************************************************************/
/*                                                                          */
/*         Copyright (C) Siemens AG 1991, 1997. All Rights Reserved         */
/*                                                                          */
/****************************************************************************/
