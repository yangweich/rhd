/****************************************************************************/
/*                                                                          */
/*   Project           : SCI                     ,###.   ,###.   #####      */
/*   Filename          : fdl_fw.h                #   #   #   #     #        */
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
#ifndef DONT_USE_MS_PACK
#pragma pack(1)
#endif

#ifdef __GNUC__
#define GNUC__PACK __attribute__ ((packed))
#else
#define GNUC__PACK
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

/* com_class */
#define  request     0x0000u
#define  confirm     0x0001u
#define  indication  0x0002u

/* service_code */
#define sda                               0x00u       /* Send Data with Acknowledge                                           */
#define sdn                               0x01u       /* Send Data with no Acknowledge                                        */
#define sdn_broadcast                     0x7fu       /* only for FDL-indication !!! (signs received broadcast-telegram)      */
#define srd                               0x03u       /* Send and Request Data                                                */
#define csrd                              0x05u       /* Cyclic Send and Request Data                                         */
#define reply_update_single               0x06u       /* Reply Update Single Mode                                             */
#define reply_update_multiple             0x07u       /* Reply Update Multiple Mode                                           */
#define fdl_read_value                    0x0bu       /* read busparameter                                                    */
#define fdl_set_value                     0x0cu       /* set busparameter                                                     */
#define sap_activate                      0x0eu       /* activate local SAP                                                   */
#define rsap_activate                     0x11u       /* activate local Responder-SAP                                         */
#define sap_deactivate                    0x12u       /* deactivate local (R)SAP                                              */
#define fdl_reset                         0x13u       /* reset PHY and FDL; all FDL-information is lost, exc. last busparam.  */
#define mac_reset                         0x15u       /* reset for MAC; a part of last valid busparameter will be updated     */
#define fdl_event                         0x18u       /* only for indication, list of events                                  */
#define lsap_status                       0x19u       /* requests information of remote-SAP or local-SAP                      */
#define fdl_life_list_create_remote       0x1au       /* requests list of intact stations                                     */
#define fdl_life_list_create_local        0x1bu       /* requests quick-list of intact stations (LAS and GAP will be actual)  */
#define fdl_ident                         0x1cu       /* requests data of software- and hardware-release                      */
#define fdl_read_statistic_ctr            0x1du       /* reads counter values of statistic and resets counter                 */
#define fdl_read_las_statistic_ctr        0x1eu       /* reads LAS and las_cycle_ctr and resets las_cycle_ctr                 */
#define await_indication                  0x1fu       /* provides resources for indication (sap-dependent)                    */
#define withdraw_indication               0x20u       /* returnes indication-resources                                        */
#define load_routing_table                0x21u       /* only for network-connection !!!                                      */
#define deactivate_routing_table          0x22u       /* only for network-connection !!!                                      */
#define get_direct_conn                   0x23u       /* gets adress of next station                                          */

// service_class
#define low                               0x00u
#define high                              0x01u

// link_status
#define ok                        0x00u               /* ACK. positive                                                        */
#define ue                        0x01u               /* ACK. negative:   remote-USER/FDL interface error                     */
#define rr                        0x02u               /* ACK. negative:   no remote resource available                        */
#define rs                        0x03u               /* ACK. negative:   service or rem_add at remote-lsap not activated     */
#define dl                        0x08u               /* response-data (l_sdu) low available                                  */
#define nr                        0x09u               /* ACK. negative:   no response-data at remote-FDL available            */
#define dh                        0x0au               /* response-data (l_sdu) high available                                 */
#define rdl                       0x0cu               /* response-data (l_sdu) low availableu but negative-ACK for send-data  */
#define rdh                       0x0du               /* response-data (l_sdu) high available, but negative-ACK for send-data */
#define ls                        0x10u               /* service not activated at local sap                                   */
#define na                        0x11u               /* no reaction (ACK/RES) from remote-station                            */
#define ds                        0x12u               /* local FDL/PHY not in token-ring                                      */
#define no                        0x13u               /* ACK. negative:   not ok (different meanings dependant on service)    */
#define lr                        0x14u               /* resource of local FDL not available                                  */
#define iv                        0x15u               /* invalid parameter in request                                         */
#define lo                        0x20u               /* LOw-prior response-data are sent at this srd                         */
#define hi                        0x21u               /* HIgh-prior response-data are sent at this srd                        */
#define no_data                   0x22u               /* NO-DATA are sent at this srd                                         */

//   poll_element_entry
#define unlocked                  0x00u               /* polling enabled for this element                                     */
#define locked                    0x01u               /* element locked for this poll-cycle                                   */
#define not_changed               0x02u               /* the same meaning as "unlocked", but layer 2 doesn't plausible this   */

//   station_type
#define passive                   0x00u
#define active                    0x01u
#define active_fast               0x02u
#define passive_fast              0x03u
#define sm_active                 0x04u               /* possible returned by read_value (PROFIBUS-PA)                        */
#define sm_passive                0x05u

//   baud_rate
/*
#define kbaud_9_6                 0x00u
#define kbaud_19_2                0x01u
#define kbaud_93_75               0x02u
#define kbaud_187_5               0x03u
#define kbaud_500                 0x04u
#define kbaud_375                 0x05u
#define kbaud_750                 0x06u
#define mbaud_1_5                 0x07u
#define mbaud_3                   0x08u
#define mbaud_6                   0x09u
#define mbaud_12                  0x0au
#define kbaud_45_45               0x0cu  */

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
   short                   code;
} GNUC__PACK;


struct  remote_address
{
    DPR_BYTE                    station;
    DPR_BYTE                    segment;     /* if no segment is used, set "NO_SEGMENT"                              */
} GNUC__PACK;

struct  link_service_data_unit
{
    void                    *buffer_ptr;
    DPR_BYTE                    length;
} GNUC__PACK;


struct  application_block
{
    DPR_BYTE                          opcode;         /* class of communication                                               */
    DPR_BYTE                          subsystem;      /* number of source-task (only necessary for MTK-user !!!!!)            */
    DPR_WORD                          id;             /* identification of FDL-USER                                           */
    struct  service                service;        /* identification of service                                            */
    struct  remote_address         loc_add;        /* only for network-connection !!!                                      */
    DPR_BYTE                          ssap;           /* source-service-access-point                                          */
    DPR_BYTE                          dsap;           /* destination-service-access-point                                     */
    struct  remote_address         rem_add;        /* address of the remote-station                                        */
    DPR_WORD                          serv_class;     /* priority of service                                                  */
    struct  link_service_data_unit receive_l_sdu;  /* address and length of received netto-data, exception:                */
    DPR_BYTE                          reserved1;      /* (reserved for FDL !!!!!!!!!!)                                        */
    DPR_BYTE                          reserved2;      /* (reserved for FDL !!!!!!!!!!)                                        */
    struct  link_service_data_unit send_l_sdu;     /* address and length of send-netto-data, exception:                    */
                                                   /* 1. csrd                 : length means number of POLL-elements       */
                                                   /* 2. await_indication     : concatenation of application-blocks and    */
                                                   /*    withdraw_indication  : number of application-blocks               */
    DPR_WORD                          l_status;       /* link-status of service or update_state for srd-indication            */
    DPR_BYTE                          reserved3[4];   /* for concatenated lists       (reserved for FDL !!!!!!!!!!)           */
} GNUC__PACK;

typedef struct
{
    DPR_WORD                           reserved [2];
    DPR_BYTE                           length;
    DPR_WORD                           user;
    DPR_BYTE                           rb_type;
    DPR_BYTE                           priority;
    DPR_BYTE                           reserved_1;
    DPR_WORD                           reserved_2;
    DPR_BYTE                           subsystem;
    DPR_BYTE                           opcode;
    DPR_WORD                           response;
    DPR_WORD                           fill_length_1;
    DPR_BYTE                           reserved_3;
    DPR_WORD                           seg_length_1;
    DPR_WORD                           offset_1;
    DPR_WORD                           reserved_4;
    DPR_WORD                           fill_length_2;
    DPR_BYTE                           reserved_5;
    DPR_WORD                           seg_length_2;
    DPR_WORD                           offset_2;
    DPR_WORD                           reserved_6;

} GNUC__PACK rb2_header_type;

typedef struct
{
    rb2_header_type                 rb2_header;
    struct application_block        application_block;
    DPR_BYTE                           reserved[12];
    DPR_BYTE                           reference[2];
    DPR_BYTE                           user_data_1[260];
    DPR_BYTE                           user_data_2[260];
} GNUC__PACK fdl_rb;

typedef fdl_rb flc_rb;

/* Moegliche Strukturen der Datenpuffer */

struct  user_poll_element
{
    DPR_BYTE                           dsap;          /* destination-service-access-point                                     */
    struct  remote_address          rem_add;       /* address of the remote-station                                        */
    short                           serv_class;    /* priority of send-telegram                                            */
    struct  link_service_data_unit  receive_l_sdu; /* request: length means buffer-length in byte                          */
                                                   /* confirm: length means length of received netto-data                  */
    struct  link_service_data_unit  send_l_sdu;    /* address and length of send-netto-data                                */
    DPR_BYTE                           reserved;      /* reserved for FDL !!!!!                                               */
    short                           l_status;      /* link-status of poll-element-service                                  */
    short                           entry;         /* locks or unlocks a poll-element                                      */
    DPR_BYTE                           reserved_2;    /* reserved for FDL !!!!!                                               */
} GNUC__PACK;


struct  ident
{
    DPR_BYTE   reserved_header[8];                    /* reserved for FDL !!!!!                                               */
    DPR_BYTE   ident_info[202];
    DPR_BYTE   response_telegram_length;              /* reserved for FDL !!!!!                                               */
} GNUC__PACK;


struct  bus_parameter_block
{
    DPR_BYTE                    hsa;                  /* highest station-address                                              */
                                                   /* range of values:  2 ... 126                                          */
    DPR_BYTE                    ts;                   /* FDL-address of this station                                          */
                                                   /* range of values:  0 ... 126                                          */
    DPR_WORD                    station_type;          /* active, passive                                                      */
    DPR_WORD                    baud_rate;             /* transmission rate                                                    */
    DPR_WORD                    medium_red;            /* availability of redundant media                                      */
    DPR_WORD                    retry_ctr;            /* retry-number of requestor, if no reaction of responder               */
                                                   /* range of values:  1 ... 8                                            */
    DPR_BYTE                    default_sap;          /* Default SAP if no address-extension is used                          */
                                                   /* range of values:  2 ... 62                                           */
    DPR_BYTE                    network_connection_sap;/* number of sap for network-connection (only for network-connections) */
                                                   /* range of values:  2 ... 62                                           */
    DPR_WORD                    tsl;                  /* SLOT-time:                                                           */
                                                   /* range of values:  2 exp 0 ... (2 exp 16) - 1   BIT-times             */
    DPR_WORD                    tqui;                 /* Transmitter-Fall-Time / Repeater-Switch-Time:                        */
                                                   /* range of values:  0 ... (2 exp 8) - 1 BIT-times                      */
    DPR_WORD                    tset;                 /* setup-time                                                           */
                                                   /* range of values:  0 ... (2 exp 8) - 1 BIT-times                      */
    DPR_WORD                    min_tsdr;             /* smallest STATION-DELAY-time:                                         */
                                                   /* range of values:  2 exp 0 ... (2 exp 16) - 1   BIT-times             */
    DPR_WORD                    max_tsdr;             /* largest STATION-DELAY-time:                                          */
                                                   /* range of values:  2 exp 0 ... (2 exp 16) - 1   BIT-times             */
    DPR_DWORD                    ttr;                  /* TARGET-ROTATION-time:                                                */
                                                   /* range of values:  2 exp 0 ... (2 exp 24) - 1   BIT-times             */
    DPR_BYTE                    g;                    /* GAP-UPDATE-factor: in multiples of ttr                               */
                                                   /* range_of_values:  1 ... 100                                          */
	   DPR_WORD                in_ring_desired;       /* request entrance into the token-ring                                 */
	   DPR_WORD                physical_layer;        /* RS485, modem                                                         */
    struct  ident           ident;                 /* vendor-name, controller_type, version of hardware and software       */
} GNUC__PACK;

struct  fdl_sap
{
    DPR_WORD                  user_id;                /* identification for user                                              */
    DPR_BYTE                  max_l_sdu_length;       /* maximal length of netto_data for this sap                            */
    DPR_BYTE                  access_sap;             /* reserved destination_sap                                             */
    DPR_BYTE                  access_station;         /* reserved destination_address                                         */
    DPR_BYTE                  access_segment;         /* reserved destination_segment                                         */
    DPR_BYTE                  s_sda;                    /* ... .sda = "ROLE"; sda unused by service "rsap_activate"             */
    DPR_BYTE                  s_sdn;                    /* look sda; sdn unused by service "rsap_activate"                      */
    DPR_BYTE                  s_srd;                    /* look sda;                                                            */
    DPR_BYTE                  s_csrd;                   /* if you want to activate csrd for this SAP, type: ... .csrd = "ROLE"; */
                                                   /* (only INITIATOR or SERVICE_NOT_ACTIVATED possible)                   */
                                                   /* csrd unused by service "rsap_activate"                               */
    void                  *rup_l_sdu_ptr_low;      /* reserved for "sap_deactivate"; USER gets rup-buffer returned         */
    void                  *rup_l_sdu_ptr_high;     /* reserved for "sap_deactivate"; USER gets rup-buffer returned         */
    DPR_BYTE                  reserved[22];           /* reserved for FDL !!!!!                                               */
} GNUC__PACK;


/**********************************************************************/
/****************   fdl_event (returned by fma_indication) ************/
/****************   await_indication (for DSAP = EVENT_SAP)************/
/**********************************************************************/


struct  event_ctr
{
    DPR_WORD                    threshold;                      /* if counter achieves threshold, a fma_indication is initiated         */
    DPR_WORD                    counter;
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
    DPR_WORD                    invalid_start_delimiter_ctr;
    DPR_WORD                    invalid_fcb_fcv_ctr;
    DPR_WORD                    invalid_token_ctr;
    DPR_WORD                    collision_ctr;
    DPR_WORD                    wrong_fcs_or_ed_ctr;
    DPR_WORD                    frame_error_ctr;
    DPR_WORD                    char_error_ctr;
    DPR_WORD                    retry_ctr;
            /* reference-counter */
    DPR_DWORD                    start_delimiter_ctr;
    DPR_DWORD                    stop_receive_ctr;
    DPR_DWORD                    send_confirmed_ctr;
    DPR_DWORD                    send_sdn_ctr;
} GNUC__PACK;


#ifndef DONT_USE_MS_PACK
#pragma pack()
#endif

/****************************************************************************/
/*                                                                          */
/*         Copyright (C) Siemens AG 1991, 1997. All Rights Reserved         */
/*                                                                          */
/****************************************************************************/
