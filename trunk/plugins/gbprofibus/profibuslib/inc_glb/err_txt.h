/*****************************************************************************/
/*    Copyright (c) SIEMENS AG, 1998                                         */
/*    All Rights reserved                                                    */
/*****************************************************************************/
/*  Project:    CP 5613                                                      */
/*  Component:  DP_BASE.DLL                                                  */
/*  Version:    1.0                                                          */
/*  Modul:      err_txt.h                                                    */
/*---------------------------------------------------------------------------*/
/*  Comment;                                                                 */
/*  This file contains the error text for different error classes and        */
/*  error numbers.                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*  History of changes:                                                      */
/*  ===================                                                      */
/*                                                                           */
/*  Date     Comment                                                         */
/*----------!----------------------------------------------------------------*/
/*  18.05.98 first release                                                   */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/*                                                                          */
/* Dieses Programm ist Freeware. Jedem Benutzer steht es frei, dieses       */
/* Programm unentgeltlich zu nutzen, zu kopieren, zu ver�ndern, in andere   */
/* Applikationen zu integrieren und/oder weiterzugeben, vorausgesetzt, dass */
/* die im Programm enthaltenen Urheberrechtsvermerke und Marken unver�ndert */
/* �bernommen werden und jede �nderung des Programms als solche bezeichnet  */
/* wird.                                                                    */
/*                                                                          */
/* JEGLICHE GEW�HRLEISTUNG F�R DIE FUNKTIONST�CHTIGKEIT ODER KOMPATIBILIT�T */
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
 


#ifndef __ERR_TXT__
#define __ERR_TXT__


/* constants */

#define ERR_TXT_NOT_FOUND            0x0000
#define ERR_TXT_FOUND                0x0001
#define ERR_TXT_END                  0xffffffff

/* default text */

#define ERR_DEFAULT_TXT_D         "Unbekannter Errorcode"
#define ERR_DEFAULT_TXT_E         "Unknown error code"
#define ERR_DEFAULT_CLS_TXT_D     "Unbekannte Errorclass"
#define ERR_DEFAULT_CLS_TXT_E     "Unknown error class"
#define ERR_DEFAULT_MINIMAL_TXT_D "Errorcode: "
#define ERR_DEFAULT_MINIMAL_TXT_E "Errorcode: "



/* internal used error classes(info) */

#define DP_ERROR_EVENT_INFO       0x000a   /* slave sends in a response telegram  DPC1 error values */
#define DP_ERROR_EVENT_NET_INFO   0x000b   /* error at an underlaying driver */
#define DP_ERROR_REQ_PAR_INFO     0x000c   /* wrong request parameter   */
#define DP_ERROR_CI_INFO          0x000d   /* error at accessing the CP */
                                           /* error_code see ci_ret.h   */
#define DP_ERROR_RES_INFO         0x000e   /* not enough ressources     */
#define DP_ERROR_USR_ABORT_INFO   0x000f   /* user has finished DP-communication */




/* The error table can be extended according to the error values  */

typedef struct ERR_INF_S
{
    DPR_DWORD errClassOrNum;
    char      *header_d;
    char      *errTxt_d[4];
    char      *header_e;
    char      *errTxt_e[4];
} ERR_INF_T;


/* ====================== */
/* text for error classes */
/* ====================== */

static const ERR_INF_T ErrClass[] =
{ 
    {   DP_OK,
        "Fehlerklasse",
	{
	  "Der Auftrag wurde erfolgreich durchgefuehrt. Die Quittung ist vorhanden\n",
	  "und kann ausgewertet werden.\n",
	  "",
	  ""
	},
        /* from here english */
        "error_class",
	{
	  "The job was executed successfully. The response was received\n",
	  "and can be evaluated.\n",
	  "",
	  ""
	}
    },
	{  
		DP_OK_ASYNC,
        "Fehlerklasse",
		{"Der Auftrag wurde an den CP zur weiteren Bearbeitung uebergeben.\n",
        "Die Quittung erfolgt asynchron und muss durch einen DP_get_result-Aufruf abgeholt werden.\n",
        "",
        ""},
        /* from here english */
		  "error_class",
		  {"The job was transferred to the CP for execution.\n",
        "The response is asynchronous and must be fetched with a DP_get_result.\n",
        "",
        ""}
    },
	{  
		DP_ERROR_EVENT,
        "Fehlerklasse",
		{"Der Slave hat nach einem DPC1-Request(ds_write/ds_read_alarm_acknowledge)\n",
        "im Responsetelegramm eine detaillierte Fehlercodierung zurueckgesendet.\n",
        "",
        ""},
        /* from here english */
		  "error_class",
		  {"The slave has returned a detailed error coding in the response\n",
        "after a DPC1 request(ds_write/ds_read_alarm_acknowledge).\n",
        "",
        ""}
    },
	{  
		DP_ERROR_EVENT_NET,
        "Fehlerklasse",
		{"Bei der Auftragsbearbeitung oder bei der Profibus-Kommunikation ist ein Fehler\n",
        "aufgetreten.",
        "",
        ""},
        /* from here english */
		  "error_class",
        {"An error occurred processing the job or in the Profibus communication.\n",
        "",
        "",
        ""}
    },
	{  
		DP_ERROR_REQ_PAR,
        "Fehlerklasse",
		{"Fehlerhafter Parameter beim Aufruf der Funktion.\n",
        "",
        "",
        ""},
        /* from here english */
        "error_class",
		  {"Bad parameter in the function call.\n",
        "",
        "",
        ""}
    },
	{  
		DP_ERROR_CI,
        "Fehlerklasse",
		{"Fehlermeldung des Treibers.\n",
        "",
        "",
        ""},
        /* from here english */
        "error_class",
		  {"Error message of the driver.\n",
        "",
        "",
        ""}
    },
	{  
		DP_ERROR_RES,
        "Fehlerklasse",
		{"Ressourcen-Problem.\n",
        "",
        "",
        ""},
        /* from here english */
		  "error_class",
        {"Resource problem.\n",
        "",
        "",
        ""}
    },
	{  
		DP_ERROR_USR_ABORT,
        "Fehlerklasse",
		{"Abbruch in Bearbeitung befindlicher Auftraege nach Abmelden der DP-Applikation.\n",
        "",
        "",
        ""},
        /* from here english */
		  "error_class",
        {"Active jobs aborted after the DP application logged off.\n",
        "",
        "",
        ""}
    },
    /* info text */
 	{  
		DP_ERROR_EVENT_INFO,
        "Fehlerklasse",
		{"DP Meldung.\n",
        "",
        "",
        ""},
        /* from here english */
        "error_class",
		  {"DP message.\n",
        "",
        "",
        ""}
    },
	{  
		DP_ERROR_EVENT_NET_INFO,
        "Fehlerklasse",
		{"DP Meldung.\n",
        "",
        "",
        ""},
        /* from here english */
		  "error_class",
        {"DP message.\n",
        "",
        "",
        ""}
    },
	{  
		DP_ERROR_REQ_PAR_INFO,
        "Fehlerklasse",
		{"DP Meldung.\n",
        "",
        "",
        ""},
        /* from here english */
		  "error_class",
        {"DP message.\n",
        "",
        "",
        ""}
    },
	{  
		DP_ERROR_RES_INFO,
        "Fehlerklasse",
		{"DP Meldung.\n",
        "",
        "",
        ""},
        /* from here english */
		  "error_class",
        {"DP message.\n",
        "",
        "",
        ""}
    },
	{  
		DP_ERROR_USR_ABORT_INFO,
        "Fehlerklasse",
		{"DP Meldung.\n",
        "",
        "",
        ""},
        /* from here english */
		  "error_class",
        {"DP message.\n",
        "",
        "",
        ""}
    },
	{  
		DP_ERROR_CI_INFO,
        "Fehlerklasse",
		{"Meldung des Treibers.\n",
        "",
        "",
        ""},
        /* from here english */
        "error_class",
        {"Message of the driver.\n",
        "",
        "",
        ""}
    },

    /* This entry terminates the error table */
    {   ERR_TXT_END, 
        ERR_DEFAULT_CLS_TXT_D, 
	 {"",
        "",
        "",
        "\n"},
        /* from here english */
		  ERR_DEFAULT_CLS_TXT_E,
        {"",
        "",
        "",
        "\n"}
    }
}; 


#ifdef LARGE_TEXT

/* ================================================================= */
/* text for DP error codes at at error classes except DP_ERROR_EVENT */
/* ================================================================= */

static const ERR_INF_T ErrCodeDp[] =
{ 
    {   DP_OK,                     /* error number always 0 */
        "Hinweis",
	 {"Kein Fehler vorhanden\n",
        "",
        "",
        ""},
        /* from here english */
        "Note",
		  {"No error found\n",
        "",
        "",
        ""}
    },
    {   DP_RET_CP_REQ_NOT_ALLOWED,       
        "Fehler",
	 {"Der Dienst ist nicht erlaubt",
        "",
        "",
        ""},
        /* from here english */
        "Error",
		  {"The service is not permitted",
        "",
        "",
        ""}
    },
    {   DP_RET_TIMEOUT,       
        "Hinweis",
	 {"Die vorgegebene Timeout-Zeit ist abgelaufen\n",
        "Bedeutung bei DP_get_result: Es wurde keine Confirmation empfangen\n",
        "Bedeutung bei DP_get_pointer:Der Zeiger konnte nicht erhalten werden, Moegliche Ursache:\n ",
        "weitere Applikationen haben den Zeiger in Benutzung"},
        /* from here english */
        "Note",
		  {"The timeout monitoring time has elapsed\n",
        "Meaning for DP_get_result: No confirmation was received\n",
        "Meaning for DP_get_pointer: The pointer could not be obtained. Possible cause:\n ",
        "Other applications are using the pointer."}
    },
    {   DP_RET_CP_INIT_INSTANCE,       
        "Fehler",
	 {"Fehler beim DP Setup",
        "",
        "",
        ""},
        /* from here english */
        "Error",
		  {"Error in DP setup.",
        "",
        "",
        ""}
    },

    {   DP_RET_CP_CONTROL_COMMAND,       
        "Fehler",
	 {"Control Command ist ungueltig",
        "",
        "",
        ""},
        /* from here english */
        "Error",
		  {"Control command is invalid",
        "",
        "",
        ""}
    },
    
    {   DP_RET_CP_TOO_MANY_CTRL_CMD,       
        "Fehler",
	 {"Zu viele Global Control Commands sind in Bearbeitung",
        "",
        "",
        ""},
        /* from here english */
        "Error",
		  {"Too many global control commands are processed!",
        "",
        "",
        ""}
    },

    {   DP_RET_CP_WRONG_INSTANCE,       
        "Fehler",
        "Zugriff durch ungueltige User Instanz",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Access by invalid user instance",
        "",
        "",
        "",
    },

    {   DP_RET_CP_RESET_INSTANCE,       
        "Fehler",
        "Fehler beim DP-Reset",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Error in DP reset",
        "",
        "",
        "",
    },

    {   DP_RET_CP_RESET_RUNNING,       
        "Fehler",
        "Reset ist bereits aktiviert",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Reset already activated",
        "",
        "",
        "",
    },

    {   DP_RET_CP_UNKNOWN_SLV_TYPE,       
        "Fehler",
        "Slave Typ unbekannt",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Slave type unknown",
        "",
        "",
        "",
    },

    {   DP_RET_CP_WRONG_MODE_OFL,       
        "Fehler",
        "Ungueltige Betriebsart beim Set Mode \n",
        "Aktuelle Betriebsart: DP_OFFLINE \n",
        "Erlaubte Betriebsart: DP_STOP",
        "",
        /* from here english */
        "Error",
        "Invalid mode for Set Mode \n",
        "Actual mode: DP_OFFLINE \n",
        "Allowed mode: DP_STOP",
        "",
    },

    {   DP_RET_CP_WRONG_MODE_STP,       
        "Fehler",
        "Ungueltige Betriebsart beim Set Mode \n",
        "Aktuelle Betriebsart: DP_STOP \n",
        "Erlaubte Betriebsart: DP_OFFLINE, DP_CLEAR",
        "",
        /* from here english */
        "Error",
        "Invalid mode for Set Mode \n",
        "Actual mode: DP_STOP \n",
        "Allowed mode: DP_OFFLINE, DP_CLEAR",
        "",
    },

    {   DP_RET_CP_WRONG_MODE_CLR,       
        "Fehler",
        "Ungueltige Betriebsart beim Set Mode \n",
        "Aktuelle Betriebsart: DP_CLEAR \n",
        "Erlaubte Betriebsart: DP_STOP, DP_OPERATE",
        "",
        /* from here english */
        "Error",
        "Invalid mode for Set Mode \n",
        "Actual mode: DP_CLEAR \n",
        "Allowed mode: DP_STOP, DP_OPERATE",
        "",
    },

    {   DP_RET_CP_WRONG_MODE_OPR,       
        "Fehler",
        "Ungueltige Betriebsart beim Set Mode \n",
        "Aktuelle Betriebsart: DP_OPERATE \n",
        "Erlaubte Betriebsart: DP_CLEAR",
        "",
        /* from here english */
        "Error",
        "Invalid mode for Set Mode \n",
        "Actual mode: DP_OPERATE \n",
        "Allowed mode: DP_CLEAR",
        "",
    },

    {   DP_RET_CP_SET_MODE_FAIL,       
        "Fehler",
        "Fehler beim Set Mode",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Error in Set Mode",
        "",
        "",
        "",
    },

    {   DP_RET_CP_SLV_NOT_ACTIV,       
        "Fehler",
        "Slave ist inaktiviert",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Slave not activated",
        "",
        "",
        "",
    },
    {   DP_RET_CP_SLV_NOT_IN_DATA,       
        "Fehler",
        "Der Slave ist (momentan) nicht bereit fuer den Datenaustausch",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The slave is not currently ready for data exchange",
        "",
        "",
        "",
    },
    {   DP_RET_CP_REQ_ACTIV,
        "Fehler",
        "DP request schon in Bearbeitung\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "DP request being processed\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_SET_MODE_FAIL,
        "Fehler",
        "Fehler waehrend der DP_set_mode-Bearbeitung\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Error processing DP_set_mode\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_CLOSED,
        "Fehler",
        "Die Bearbeitungsinstanz in der Firmware wurde zuvor beendet und besteht nicht mehr\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The processing instance in the firmware was terminated earlier and no longer exists\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_STOPPED,
        "Fehler",
        "Die Bearbeitungsinstanz ist schon gestoppt worden\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The processing instance has already been stopped\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_STARTED,
        "Fehler",
        "Die Bearbeitungsinstanz wurde zuvor schon gestartet\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The processing instance has already been started\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_STATE_UNKNOWN,
        "Fehler",
        "Die Bearbeitungsinstanz ist in einem undefinierten Zustand\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The processing instance is in an unknown state\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_REQ_WITHDRAW,
        "Fehler",
        "Der Auftrag wurde zurueckgezogen und kann daher nicht bearbeitet werden\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The job was withdrawn and cannot be executed\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_REQ_NOT_FOUND,
        "Fehler",
        "Der zugehoerige Auftragsblock des Requests wurde nicht gefunden\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The job field for the request was not found\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_REQ_NEG,
        "Fehler",
        "Negativquittung beim Senden des Auftrags ueber den Profibus.\n",
        "Moegliche Ursachen:Slave antwortet nicht oder Dienstzugangspunkt beim Slave nicht\n",
        "aktiviert",
        "",
        /* from here english */
        "Error",
        "Negative acknowledgment for job sent on Profibus.\n",
        "Possible causes: Slave not responding or the service access point on the\n",
        "slave is not activated",
        "",
    },
    {   DP_RET_CP_L2_REQ,
        "Fehler",
        "Unbekannter Opcode in der Confirmation\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Unknown opcode in the confirmation\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_TIMEOUT,
        "Hinweis",
        "Der Auftrag wurde durch Timeout beendet\n",
        "",
        "",
        "",
        /* from here english */
        "Note",
        "The job was terminated due to a timeout\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_REQ_RE,
        "Fehler",
        "Format-Error in einem Response-Telegramm\n",
        "Quelle: lokale DP-Instanz (Direct Data Link Mapper)\n",
        "",
        "",
        /* from here english */
        "Error",
        "Format error in a response frame\n",
        "Source: local DP instance (direct data link mapper)\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_FE,
        "Fehler",
        "Format-Error in einem Request-Frame\n",
        "Quelle: remote DP-Instanz (Direct Data Link Mapper)\n",
        "",
        "",
        /* from here english */
        "Error",
        "Format error in a request frame\n",
        "Source: remote DP instance (direct data link mapper)\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_NI,
        "Fehler",
        "Funktion nicht implementiert\n",
        "Quelle: remote User\n",
        "",
        "",
        /* from here english */
        "Error",
        "Function not implemented\n",
        "Source: remote user\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_AD,
        "Fehler",
        "Zugang abgelehnt\n",
        "Quelle: remote User\n",
        "",
        "",
        /* from here english */
        "Error",
        "Access denied\n",
        "Source: remote user\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_EA,
        "Fehler",
        "Bereich zu gross (Up-/Download)\n",
        "Quelle: remote User\n",
        "",
        "",
        /* from here english */
        "Error",
        "Area too large (up/download)\n",
        "Source: remote user\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_LE,
        "Fehler",
        "Datenblock-Laenge zu gross (Up-/Download)\n",
        "Quelle: remote User\n",
        "",
        "",
        /* from here english */
        "Error",
        "Data block too long (Up/download)\n",
        "Source: remote user\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_RE,
        "Fehler",
        "Format-Fehler in einem Response-Frame\n",
        "Quelle: lokale DP-Instanz (Direct Data Link Mapper)\n",
        "",
        "",
        /* from here english */
        "Error",
        "Format error in a response frame\n",
        "Source: local DP instance (direct data link mapper)\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_IP,
        "Fehler",
        "Ungueltiger Parameter\n",
        "Quelle: remote User\n",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid parameter\n",
        "Source: remote user\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_SC,
        "Fehler",
        "Sequenz-Konflikt\n",
        "Quelle: remote User\n",
        "",
        "",
        /* from here english */
        "Error",
        "Sequence conflict\n",
        "Source: remote user\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_SE,
        "Fehler",
        "Sequenz-Fehler\n",
        "Quelle: remote DP-Instanz (Direct Data Link Mapper)\n",
        "",
        "",
        /* from here english */
        "Error",
        "Sequence error\n",
        "Source: remote DP instance (direct data link mapper)\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_NE,
        "Fehler",
        "Bereich exisitert nicht\n",
        "Quelle: remote User\n",
        "",
        "",
        /* from here english */
        "Error",
        "Area does not exist\n",
        "Source: remote user\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_DI,
        "Fehler",
        "Daten unvollstaendig\n",
        "Quelle: remote User\n",
        "",
        "",
        /* from here english */
        "Error",
        "Data incomplete\n",
        "Source: remote user\n",
        "",
        "",
    },
    {   DP_RET_CP_MM_NC,
        "Fehler",
        "Master Parametersatz nicht kompatibel\n",
        "Quelle: remote User\n",
        "",
        "",
        /* from here english */
        "Error",
        "Master parameter set not compatible\n",
        "Source: remote user\n",
        "",
        "",
    },
    {   DP_RET_CP_REQ_INVALID_PAR,       
        "Fehler",
        "Der Auftrag wurde mit Fehler beendet wegen ungueltiger Parameter",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The job was terminated with an error due an invalid parameter",
        "",
        "",
        "",
    },
    {   DP_RET_REQ_ACTIV,       
        "Fehler",
        "Ein entsprechender Request ist schon in Bearbeitung. Erst nach Abholen der Quittung durch \n",
        "DP_get_result ist ein weiterer Request dieser Art moeglich: Bei DP_ds_write,DP_ds_read und \n",
        "DP_get_actual_cfg ist einer dieser Requests sowie zusaetzlich ein DP_alarm_ack pro Slave moeglich.\n",
        "Bei DP_enable_event koennen nicht mehrere Requests parallel bearbeitet werden.",
        /* from here english */
        "Error",
        "A request similar to this one is already being processed. The result must first be fetched \n",
        "with DP_get_result before a second request of this type is possible: With DP_ds_write,DP_ds_read \n",
        "and DP_get_actual_cfg one of these requests and one DP_alarm_ack is possible per slave.\n",
        "With DP_enable_event, multiple requests cannot be handled at the same time.",
    },
    {   DP_RET_CP_SET_MODE_OFFL_ACT,       
        "Fehler",
        "Der DP_set_mode-Aufruf konnte nicht durchgefuehrt werden, weil ein voriger \n",
        "DP_set_mode-Aufruf (Wechsel in den Status Offline) noch in Bearbeitung ist.\n",
        "",
        "",
        /* from here english */
        "Error",
        "The DP_set_mode call could not be executed because a previous \n",
        "DP_set_mode call (change to offline status) is still active.\n",
        "",
        "",
    },
    {   DP_RET_CP_SET_MODE_STOP_ACT,       
        "Fehler",
        "Der DP_set_mode-Aufruf konnte nicht durchgefuehrt werden, weil ein voriger \n",
        "DP_set_mode-Aufruf (Wechsel in den Status Stop) noch in Bearbeitung ist.\n",
        "",
        "",
        /* from here english */
        "Error",
        "The DP_set_mode call could not be executed because a previous \n",
        "DP_set_mode call (change to stop status) is still active.\n",
        "",
        "",
    },
    {   DP_RET_CP_SET_MODE_CLR_ACT,       
        "Fehler",
        "Der DP_set_mode-Aufruf konnte nicht durchgefuehrt werden, weil ein voriger \n",
        "DP_set_mode-Aufruf (Wechsel in den Status Clear) noch in Bearbeitung ist.\n",
        "",
        "",
        /* from here english */
        "Error",
        "The DP_set_mode call could not be executed because a previous \n",
        "DP_set_mode call (change to clear status) is still active.\n",
        "",
        "",
    },
    {   DP_RET_CP_SET_MODE_OPR_ACT,       
        "Fehler",
        "Der DP_set_mode-Aufruf konnte nicht durchgefuehrt werden, weil ein voriger \n",
        "DP_set_mode-Aufruf (Wechsel in den Status Operate) noch in Bearbeitung ist.\n",
        "",
        "",
        /* from here english */
        "Error",
        "The DP_set_mode call could not be executed because a previous \n",
        "DP_set_mode call (change to operate status) is still active.\n",
        "",
        "",
    },
    {   DP_RET_PAR_MST_MODE, 
        "Fehler",
        "Der Parameter mst_mode ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The mst_mode parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_CP_NAME, 
        "Fehler",
        "Der Parameter cp_name ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The cp_name parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_USR_HNDL, 
        "Fehler",
        "Der Parameter user_handle ist ungueltig oder undefiniert.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The user_handle parameter is invalid or not defined.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_DPR, 
        "Fehler",
        "Der Parameter dpr (dual port ram) ist ungueltig\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The dpr (dual-port RAM) parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_SLV_MODE, 
        "Fehler",
        "Der Parameter slv_mode ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The slv_mode parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_SLV_ADD, 
        "Fehler",
        "Der Parameter slv_add ist ungueltig\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The slv_add parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_TYPE, 
        "Fehler",
        "Der Parameter type ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The type parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_DATA_LEN, 
        "Fehler",
        "Der Parameter data_len ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The data_len parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_DATA, 
        "Fehler",
        "Der Parameter data ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The data parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_CTRL_CMD, 
        "Fehler",
        "Der Parameter control_command ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The control_command parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_REQUEST, 
        "Fehler",
        "Der Parameter request ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The request parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_LENGTH_M, 
        "Fehler",
        "Der Parameter length_m ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The length_m parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_LENGTH_S, 
        "Fehler",
        "Der Parameter length_s ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The length_s parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_ALARM, 
        "Fehler",
        "Der Parameter alarm ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The alarm parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_SELECTOR, 
        "Fehler",
        "Der Parameter selector ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The selector parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_REQ_TYPE, 
        "Fehler",
        "Der Parameter req_type ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The req_type parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_RESULT, 
        "Fehler",
        "Der Parameter result ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The result parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_SEMA_TYPE, 
        "Fehler",
        "Der Parameter sema_type ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The sema_type parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_CREF, 
        "Fehler",
        "Der Parameter c_ref ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The c_ref parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_FL, 
        "Fehler",
        "Der Parameter fast_logic ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The fast_logic parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },

    {   DP_RET_PAR_TIMEOUT, 
        "Fehler",
        "Der Parameter timeout ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The timeout parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },
    {   DP_RET_PAR_WD_INDEX, 
        "Fehler",
        "Der Parameter wd_index ist ungueltig.\n",
        "Der Auftrag wurde von der DP_BASE-Library abgelehnt.",
        "",
        "",
        /* from here english */
        "Error",
        "The wd_index parameter is invalid.\n",
        "The job was rejected by the DP_BASE library.",
        "",
        "",
    },


    {   DP_RET_CP_REQ_INVALID_LEN,       
        "Fehler",
        "Der Auftrag wurde mit Fehler beendet (ungueltige Datenlaenge)",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The job was terminated with an error (invalid data length)",
        "",
        "",
        "",
    },
    {   DP_RET_CP_WRONG_FREEZE_GRP,       
        "Fehler",
        "Der Global Control-Aufruf (Freeze/Unfreeze) wurde mit Fehler beendet, weil der adressierte DP-Slave \n",
        "keiner der angegebenen Gruppe(n) zugehoert.",
        "",
        "",
        /* from here english */
        "Error",
        "The global control call (Freeze/Unfreeze) was terminated with an error, because the addressed \n",
        "DP slave does not belong to the specified group or any of the specified groups.",
        "",
        "",
    },
    {   DP_RET_CP_WRONG_SYNC_GRP,       
        "Fehler",
        "Der Global Control-Aufruf (Sync/Unsync) wurde mit Fehler beendet, weil der adressierte DP-Slave \n",
        "keiner der angegebenen Gruppe(n) zugehoert.",
        "",
        "",
        /* from here english */
        "Error",
        "The global control call (Sync/Unsync) was terminated with an error, because the addressed \n",
        "DP slave does not belong to the specified group or any of the specified groups.",
        "",
        "",
    },
    {   DP_RET_CP_WRONG_GC_CMD,       
        "Fehler",
        "Der Auftrag wurde vom CP mit Fehler beendet (ungueltiges Global Control Command)",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The job was terminated by the CP with an error(invalid global control command)",
        "",
        "",
        "",
    },
    {   DP_RET_CP_WRONG_GC_GRP,       
        "Fehler",
        "Der Auftrag wurde vom CP mit Fehler beendet (ungueltige Global Control Gruppe)",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The job was terminated by the CP with an error(invalid global control group)",
        "",
        "",
        "",
    },
    {   DP_RET_MEMORY,
        "Fehler",
        "interner Speicher-Fehler\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal memory error\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_NO_DATABASE,
        "Fehler",
        "Es wurde keine (gueltige) Datenbasis in den  CP geladen\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "No (valid) database was downloaded to the CP\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_DATABASE_ADR,
        "Fehler",
        "Es befindet sich eine ungueltige Slaveadresse in der Datenbasis des CP.\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "There is an invalid slave address in the database of the CP.\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_ADR_NOT_IN_DB,
        "Fehler",
        "Die Slaveadresse ist nicht in der Datenbasis des CP enthalten\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The slave address is not in the database of the CP\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_NO_BUS_PAR,
        "Fehler",
        "Es sind keine Busparameter in der Datenbasis des CP enthalten\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "There are no bus parameters in the database of the CP\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_NO_DP_PAR,
        "Fehler",
        "Es sind keine DP-Parameter in der Datenbasis des CP enthalten\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "There are no DP parameters in the database of the CP\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_MEMORY_DPMC,
        "Fehler",
        "interner Speicherfehler im CP\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal memory error on the CP\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_TIMER,
        "Fehler",
        "interner Timerfehler im CP\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal timer error on the CP\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_TOO_MANY_SLV,
        "Fehler",
        "Es sind zu viele Slaves projektiert\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "There are too many slaves configured\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_TOO_MANY_USR,
        "Fehler",
        "Es koennen sich keine weiteren DP-Instanzen beim CP anmelden\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "No more DP instances can log on at the CP\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_USR_NOT_COMPATIBLE,
        "Fehler",
        "Die dplib.dll und die dp_base.dll koennen sich nicht beim gleichen CP anmelden\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The dplib.dll and the dp_base.dll cannot log on at the same CP\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_MEMORY,
        "Fehler",
        "interner Speicherfehler\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal memory error\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_MEMORY_1,
        "Fehler",
        "interner Speicherfehler(1)\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal memory error(1)\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_MEMORY_2,
        "Fehler",
        "interner Speicherfehler(2)\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal memory error(2)\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_MEMORY_3,
        "Fehler",
        "interner Speicherfehler(3)\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal memory error(3)\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_MEMORY_4,
        "Fehler",
        "interner Speicherfehler(4)\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal memory error(4)\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_MEMORY_5,
        "Fehler",
        "interner Speicherfehler(5)\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal memory error(5)\n",
        "",
        "",
        "",
    },

    {   DP_RET_CP_MEMORY_6,
        "Fehler",
        "interner Speicherfehler(6)\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal memory error(6)\n",
        "",
        "",
        "",
    },
    {   DP_RET_CP_MEMORY_7,
        "Fehler",
        "interner Speicherfehler(7)\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal memory error(7)\n",
        "",
        "",
        "",
    },
    {   DP_RET_TOO_MANY_USR,
        "Fehler",
        "Es koennen sich keine weiteren User Instanzen an der DP_BASE-Library anmelden.\n",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "No more user instances can log on at the DP_Base library.\n",
        "",
        "",
        "",
    },
    /* This entry terminates the error table */
    {   ERR_TXT_END, 
        ERR_DEFAULT_TXT_D, 
        "",
        "",
        "",
        "",
        /* from here english */
        ERR_DEFAULT_TXT_E,
        "",
        "",
        "",
        "",
    }
}; 


/* ================================================== */
/* text for error codes at error class DP_ERROR_EVENT */
/* ================================================== */

static const ERR_INF_T ErrCodeDpEvent[] =
{ 
    {   DP_OK,                                               /* error number 0 */
        "Negativquittung im DPC1-Responsetelegramm:",        /* header */
	 {"Der Slave sendet nach einem DPC1-Request(DP_ds_write,DP_ds_read,DP_alarm_ack) im\n ",
        "zugehoerigen Responsetelegramm eine detaillierte Fehlercodierung zurueck.\n",
        "Es muessen die Rueckgabewerte in error_decode, error_code_1 und error_code_2 \n",
        "ausgewertet werden."},
        /* from here english */
        "Negative acknowledgment in the DPC1 response frame:",        /* header */
		  {"The slave has returned a detailed error coding in the response\n",
        "after a DPC1 request(DP_ds_write,DP_ds_read,DP_alarm_ack).\n",
        "The return values in error_decode, error_code_1 and error_code_2 \n",
        "must be evaluated."}
    },

    /* This entry terminates the error table */
    {   ERR_TXT_END, 
        ERR_DEFAULT_TXT_D, 
        "",
        "",
        "",
        "",
        /* from here english */
        ERR_DEFAULT_TXT_E,
        "",
        "",
        "",
        "",
    }
};


#if 0
/* =============================================== */
/* text for error codes at error class DP_ERROR_CI */
/* =============================================== */

static const ERR_INF_T ErrCodeDpErrorCi[] =
{ 
    {   CI_RET_OK,                     /* t.b.d. */
        "Hinweis",
        "OK !\n",
        "",
        "",
        "",
        /* from here english */
        "Note",
        "OK !\n",
        "",
        "",
        "",
    },

    {   CI_RET_OK_DATA,                     /* t.b.d. */
        "Hinweis",
        "OK, Antwortdaten da !",
        "",
        "",
        "",
        /* from here english */
        "Hinweis",
        "OK, response data there !",
        "",
        "",
        "",
    },

    {   CI_RET_SET_HWND_MSG,                     /* t.b.d. */
        "Fehler",
        "Fehler bei der Funktion SetSinecHWndMsg !",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Error in the function SetSinecHWndMsg !",
        "",
        "",
        "",
    },

    {   CI_RET_INV_USR_OPCODE,                     /* t.b.d. */
        "Fehler",
        "Ungueltiges Subsystem !",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid subsystem !",
        "",
        "",
        "",
    },

    {   CI_RET_INV_USR_BUF_LENGTH,                     /* t.b.d. */
        "Fehler",
        "Puffer zu lang",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Buffer too long",
        "",
        "",
        "",
    },

    {   CI_RET_INV_USR_BUF_FILL_LENGTH,                     /* t.b.d. */
        "Fehler",
        "Fehler: -> fill_length > buf_length",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Error: -> fill_length > buf_length",
        "",
        "",
        "",
    },

    {   CI_RET_BUF_NOT_VALID,                     /* t.b.d. */
        "Fehler",
        "ung�ltiger Userbuffer",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid user buffer",
        "",
        "",
        "",
    },

    {   CI_RET_NOT_IMPLEMENTED,                     /* t.b.d. */
        "Fehler",
        "Funktion nicht implementiert",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Function not implemented",
        "",
        "",
        "",
    },

    {   CI_RET_NO_DRIVER,                     /* t.b.d. */
        "Fehler",
        "Treiber nicht geladen",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Driver not loaded",
        "",
        "",
        "",
    },

    {   CI_RET_CP_NOT_HERE,                     /* t.b.d. */
        "Fehler",
        "CP nicht vorhanden !",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "No CP found !",
        "",
        "",
        "",
    },

    {   CI_RET_HANDLE_INVALID,                     /* t.b.d. */
        "Fehler",
        "ungueltiger Wert fuer User Handle",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid value for user handle",
        "",
        "",
        "",
    },

    {   CI_RET_HANDLE_NOT_OPEN,                     /* t.b.d. */
        "Fehler",
        "Handle nicht bekannt",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Handle unknown",
        "",
        "",
        "",
    },

    {   CI_RET_HANDLE_CLOSING,                     /* t.b.d. */
        "Fehler",
        "Handle wird gerade geschlossen",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Handle being closed",
        "",
        "",
        "",
    },

    {   CI_RET_HANDLE_CP_RESET,                     /* t.b.d. */
        "Fehler",
        "Der CP wurde zurueckgesetzt -> Beenden Sie die Applikation.",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The CP has been resetted -> Terminate the application !",
        "",
        "",
        "",
    },

    {   CI_RET_UNMAP,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_MAP_DPR_0,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_MAP_DPR_1,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
    },

    {   CI_RET_MAP_PLX_0,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_MAP_PLX_1,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_MAP_DOWN_0,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_MAP_DOWN_1,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_HalAssignSlotResources,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_CmResourceTypePort,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_InterruptMode,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_mem_def,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_CmResourceTypeDma,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_CmResourceTypeDeviceSpecific,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_CmResourceTypeDefault,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_MAP_00,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_MAP_01,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_MAP_02,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_MAP_03,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_mem_res_count,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_plx_length,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_IoReportResourceUsage,                     /* t.b.d. */
        "Fehler",
        "interne Fehler beim Identifizieren des CP ueber das Plug&Play Bios \n",
        "Abhilfe -> Kontakt CP/Rechner ueberpruefen | CP tauschen | Rechner tauschen ",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error identifying the CP with the Plug&Play Bios \n",
        "Remedy -> Contact CP/check computer | replace CP | replace computer ",
        "",
        "",
    },

    {   CI_RET_EXCP_OPEN,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_CLOSE_0,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_CLOSE_1,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_APPEND_0,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_APPEND_1,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_RETURN_0,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_RETURN_1,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_RETURN_2,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_CYCLE_0,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_CYCLE_1,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_CIB,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_ALERTED,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_USER_APC,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_KE_WAIT,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_E_LIST_0,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_E_LIST_1,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_LIFE_COUNTER,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },


    {   CI_RET_EXCP_CRITICAL_0,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_GET_DP_ACCESS,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_REL_DP_ACCESS_0,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_REL_DP_ACCESS_1,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_QUEUE_DPA_0,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_EXCP_QUEUE_DPA_1,                     /* t.b.d. */
        "Fehler",
        "Exceptions -> Interne Fehler des Treibers oder der Firmware \n",
        "Abhilfe -> CP zur�cksetzen oder den PC neu booten", 
        "",
        "",
        /* from here english */
        "Error",
        "Exceptions -> Internal error in driver or firmware \n",
        "Remedy -> Reset the CP or reboot the PC",
        "",
        "",
    },

    {   CI_RET_OPEN_MAX,                     /* t.b.d. */
        "Fehler",
        "maximale CI_open Anzahl �berschritten",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Maximum number of CI_open exceeded",
        "",
        "",
        "",
    },

    {   CI_RET_OPEN_CP_NOT_STARTED,                     /* t.b.d. */
        "Fehler",
        "CP nicht gestarted",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "CP not started",
        "",
        "",
        "",
    },

    {   CI_RET_OPEN_TEMP_LOCKED,                     /* t.b.d. */
        "Fehler",
        "Dienst momentan nicht moeglich",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Service not currently possible",
        "",
        "",
        "",
    },

    {   CI_RET_RESET_CP_USED,                     /* t.b.d. */
        "Fehler",
        "Reset momentan nicht m�glich",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Reset not currently possible",
        "",
        "",
        "",
    },

    {   CI_RET_RESET_INVALID_CP_NAME,                     /* t.b.d. */
        "Fehler",
        "ung�ltiger CP Name ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid CP name ",
        "",
        "",
        "",
    },

    {   CI_RET_RESET_ALREADY_DONE,                     /* t.b.d. */
        "Hinweis",
        "CP ist schon im Zustand Reset",
        "",
        "",
        "",
        /* from here english */
        "Note",
        "CP is already in the reset state",
        "",
        "",
        "",
    },

    {   CI_RET_RESET_INVALID_MODE,                     /* t.b.d. */
        "Fehler",
        "ung�ltiger Reset Mode",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid reset mode",
        "",
        "",
        "",
    },

    {   CI_RET_RESET_CP_NOT_FOUND,                     /* t.b.d. */
        "Fehler",
        "CP wurde nicht gefunden",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "CP not found",
        "",
        "",
        "",
    },

    {   CI_RET_START_INVALID_CP_NAME,                     /* t.b.d. */
        "Fehler",
        "ung�ltiger CP Name ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid CP name",
        "",
        "",
        "",
    },

    {   CI_RET_START_ALREADY_DONE,                     /* t.b.d. */
        "Hinweis",
        "CP schon gestartet",
        "",
        "",
        "",
        /* from here english */
        "Note",
        "CP already started",
        "",
        "",
        "",
    },

    {   CI_RET_START_CP_NOT_FOUND,                     /* t.b.d. */
        "Fehler",
        "CP wurde nicht gefunden",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "CP not found",
        "",
        "",
        "",
    },

    {   CI_RET_START_CP_RESOURCES_INT,                     /* t.b.d. */
        "Fehler",
        "interner Fehler -> Resourcenproblem",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error -> resource problem",
        "",
        "",
        "",
    },

    {   CI_RET_START_ACTION_ERR,                     /* t.b.d. */
        "Fehler",
        "interner Fehler !",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error !",
        "",
        "",
        "",
    },

    {   CI_RET_START_MAP,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error !",
        "",
        "",
        "",
    },

    {   CI_RET_START_MAX_CP ,                     /* t.b.d. */
        "Fehler",
        "maximale Anzahl von CPs schon erreicht",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Maximum number of CPs already reached",
        "",
        "",
        "",
    },

    {   CI_RET_START_TEMP_LOCKED,                     /* t.b.d. */
        "Fehler",
        "Dienst momentan nicht moeglich",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Service not currently possible",
        "",
        "",
        "",
    },

    {   CI_RET_START_ERROR_ERR,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_START_CP_NO_REACTION,                     /* t.b.d. */
        "Fehler",
        "interner Fehler -> CP reagiert nicht",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error -> CP not reacting",
        "",
        "",
        "",
    },

    {   CI_RET_START_OK_ERR,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_START_INVALID_MODE,                     /* t.b.d. */
        "Fehler",
        "ung�ltiger Start Mode",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid start mode",
        "",
        "",
        "",
    },

    {   CI_RET_START_TRANSLATE,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_START_REG_NO_SWITCH_OUTPUT,                     /* t.b.d. */
        "Fehler",
        "Fehlerhafter Registryeintrag",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Bad registry entry",
        "",
        "",
        "",
    },

    {   CI_RET_START_ALREADY_DATABASE,                     /* t.b.d. */
        "Fehler",
        "zweiter Start mit einer unterschiedlichen Datenbasis",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Second start with a different database",
        "",
        "",
        "",
    },

    {   CI_RET_NO_CP_NAME,                     /* t.b.d. */
        "Fehler",
        "bei der Funktion muss der CP Name angegeben werden ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The CP name must be specified for the function",
        "",
        "",
        "",
    },

    {   CI_RET_CP_NAME_00,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_CP_NAME_NOT_FOUND,                     /* t.b.d. */
        "Fehler",
        "ungueltiger CP Name",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid CP name",
        "",
        "",
        "",
    },

    {   CI_RET_OPEN_REG,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_START_REG_DATABASE,                     /* t.b.d. */
        "Fehler",
        "interner Fehler keine Datenbasis eingetragen ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error no database entered",
        "",
        "",
        "",
    },

    {   CI_RET_START_REG_DOWNLOAD,                     /* t.b.d. */
        "Fehler",
        "interner Fehler keine Firmware eingetragen ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error no firmware entered",
        "",
        "",
        "",
    },

    {   CI_RET_START_NO_VIEW,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_START_OPEN_FIRMWARE,                     /* t.b.d. */
        "Fehler",
        "Firmware File konnte nicht ge�ffnet werden",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Could not open firmware file",
        "",
        "",
        "",
    },

    {   CI_RET_START_OPEN_DATABASE,                     /* t.b.d. */
        "Fehler",
        "Datenbasis File konnte nicht ge�ffnet werden",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Could not open database file",
        "",
        "",
        "",
    },

    {   CI_RET_START_DOWN_1,                     /* t.b.d. */
        "Fehler",
        "interner Fehler ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error ",
        "",
        "",
        "",
    },

    {   CI_RET_START_FW_INIT_TIMEOUT,                     /* t.b.d. */
        "Fehler",
        "interner Fehler -> Firmware antwortet nicht",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error -> firmware not responding",
        "",
        "",
        "",
    },

    {   CI_RET_START_LEN_FIRMWARE,                     /* t.b.d. */
        "Fehler",
        "Firmware File ist zu lang",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Firmware file is too long",
        "",
        "",
        "",
    },

    {   CI_RET_START_LEN_DATABASE,                     /* t.b.d. */
        "Fehler",
        "Datenbasis File ist zu lang",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Database file is too long",
        "",
        "",
        "",
    },

    {   CI_RET_START_FW_INIT_EXCP,                     /* t.b.d. */
        "Fehler",
        "interner Fehler -> Firmware meldet Exception ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error -> firmware reporting exception",
        "",
        "",
        "",
    },

    {   CI_RET_START_FN_DB_TOO_LONG,                     /* t.b.d. */
        "Fehler",
        "Pfad Datenbasis zu lang ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Database path too long",
        "",
        "",
        "",
    },

    {   CI_RET_START_FN_FW_TOO_LONG,                     /* t.b.d. */
        "Fehler",
        "Pfad Firmware zu lang",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Firmware path too long",
        "",
        "",
        "",
    },

    {   CI_RET_CP561X_ROOT,                     /* t.b.d. */
        "Fehler",
        "Eintrag fehlt in der Registry -> Fehler bei der Installation ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Entry missing in the registry -> installation error ",
        "",
        "",
        "",
    },

    {   CI_RET_CP_NAME_TOO_LONG,                     /* t.b.d. */
        "Fehler",
        "CP Name ist zu lang",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "CP name is too long",
        "",
        "",
        "",
    },

    {   CI_RET_START_REG_gc_data_0,                     /* t.b.d. */
        "Fehler",
        "Eintrag fehlt in der Registry -> Fehler bei der Installation ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Entry missing in the registry -> installation error ",
        "",
        "",
        "",
    },

    {   CI_RET_START_REG_gc_data_1,                     /* t.b.d. */
        "Fehler",
        "Eintrag fehlt in der Registry -> Fehler bei der Installation ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Entry missing in the registry -> installation error ",
        "",
        "",
        "",
    },

    {   CI_RET_START_REG_gc_group,                     /* t.b.d. */
        "Fehler",
        "Eintrag fehlt in der Registry -> Fehler bei der Installation ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Entry missing in the registry -> installation error ",
        "",
        "",
        "",
    },

    {   CI_RET_START_REG_tmsi,                     /* t.b.d. */
        "Fehler",
        "Eintrag fehlt in der Registry -> Fehler bei der Installation ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Entry missing in the registry -> installation error ",
        "",
        "",
        "",
    },

    {   CI_RET_START_REG_tmsi_reserve,                     /* t.b.d. */
        "Fehler",
        "Eintrag fehlt in der Registry -> Fehler bei der Installation ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Entry missing in the registry -> installation error ",
        "",
        "",
        "",
    },

    {   CI_RET_START_REG_ttr_div_256,                     /* t.b.d. */
        "Fehler",
        "Eintrag fehlt in der Registry -> Fehler bei der Installation ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Entry missing in the registry -> installation error ",
        "",
        "",
        "",
    },

    {   CI_RET_START_REG_tth_div_256_equ_dis,                     /* t.b.d. */
        "Fehler",
        "Eintrag fehlt in der Registry -> Fehler bei der Installation ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Entry missing in the registry -> installation error ",
        "",
        "",
        "",
    },

    {   CI_RET_SEND_NO_BUFFER_AVAILABLE,                     /* t.b.d. */
        "Fehler",
        "Resourcen Engpass im DPR",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Lack of resources in DPR",
        "",
        "",
        "",
    },

    {   CI_RET_SEND_BUF_TOO_SMALL ,                     /* t.b.d. */
        "Fehler",
        "User Buffer f�r die  Antwort ist zu klein ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "User buffer for the response is too short",
        "",
        "",
        "",
    },

    {   CI_RET_RECEIVE_TIMEOUT_NO_DATA,                     /* t.b.d. */
        "Fehler",
        "Timeout abgelaufen",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Timeout elapsed",
        "",
        "",
        "",
    },

    {   CI_RET_RECEIVE_BUF_TOO_SMALL_0,                     /* t.b.d. */
        "Fehler",
        "User Buffer ist zu klein ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "User buffer is too short",
        "",
        "",
        "",
    },

    {   CI_RET_RECEIVE_BUF_TOO_SMALL_1,                     /* t.b.d. */
        "Fehler",
        "",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "",
        "",
        "",
        "",
    },

    {   CI_RET_RECEIVE_TIMEOUT_CANCEL,                     /* t.b.d. */
        "Hinweis",
        "Receive wurde gecancelt ",
        "",
        "",
        "",
        /* from here english */
        "Note",
        "Receive canceled",
        "",
        "",
        "",
    },

    {   CI_RET_RECEIVE_MAX_PENDING,                     /* t.b.d. */
        "Fehler",
        "zuviele Receive mit Timeout pro CI_open ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Too many receives with timeout per CI_open",
        "",
        "",
        "",
    },

    {   CI_RET_RECEIVE_TIMEOUT_USER_APC,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_CANCEL_NO_RECEIVE,                     /* t.b.d. */
        "Hinweis",
        "nichts da zum canceln",
        "",
        "",
        "",
        /* from here english */
        "Note",
        "Nothing to cancel",
        "",
        "",
        "",
    },

    {   CI_RET_FL_INV_ACTION,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_FL_INV_ID,                     /* t.b.d. */
        "Fehler",
        "ungueltiger Parameter fast_logic_id -> 0..3",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "invalid parameter fast_logic_id -> 0..3 ",
        "",
        "",
        "",
    },

    {   CI_RET_FL_INV_ADDR_IN_BYTE,                     /* t.b.d. */
        "Fehler",
        "Ungueltige Adresse des Eingabeslaves",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "invalid address of the input slave",
        "",
        "",
        "",
    },

    {   CI_RET_FL_INV_ADDR_OUT_BYTE,                     /* t.b.d. */
        "Fehler",
        "Ungueltige Adresse des Ausgabeslaves",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "invalid address of the output slave",
        "",
        "",
        "",
    },

    {   CI_RET_FL_SLAVE_IN_NOT_IN_DB,                    /* t.b.d. */
        "Fehler",
        "Eingabeslave nicht in der Datenbasis",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Input slave not in database",
        "",
        "",
        "",
    },

    {   CI_RET_FL_SLAVE_OUT_NOT_IN_DB,                    /* t.b.d. */
        "Fehler",
        "Ausgabeslave nicht in der Datenbasis",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Output slave not in database",
        "",
        "",
        "",
    },

    {   CI_RET_FL_INV_INDEX_IN_BYTE,                    /* t.b.d. */
        "Fehler",
        "Ungueltiges Eingabebyte",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid input byte",
        "",
        "",
        "",
    },

    {   CI_RET_FL_INV_INDEX_OUT_BYTE,                    /* t.b.d. */
        "Fehler",
        "Ungueltiges Ausgabebyte",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid output byte",
        "",
        "",
        "",
    },

    {   CI_RET_FL_ALREADY_ON,                    /* t.b.d. */
        "Fehler",
        "Fast Logic schon aktiviert",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Fast Logic already activated",
        "",
        "",
        "",
    },

    {   CI_RET_FL_ALREADY_OFF,                    /* t.b.d. */
        "Fehler",
        "Fast Logic schon deaktiviert",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Fast Logic already deactivated",
        "",
        "",
        "",
    },

    {   CI_RET_FL_INV_IN_MASK,                    /* t.b.d. */
        "Fehler",
        "Ungueltige Maske f�r das Eingabe Byte",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid mask for the input byte",
        "",
        "",
        "",
    },

    {   CI_RET_FL_INV_OUT_MASK,                    /* t.b.d. */
        "Fehler",
        "Ungueltige Maske f�r das Ausgabe Byte",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid mask for the output byte",
        "",
        "",
        "",
    },

    {   CI_RET_FL_DOUBLE_USER,                    /* t.b.d. */
        "Fehler",
        "Ein zweiter User ist nicht erlaubt",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "A second user is not allowed",
        "",
        "",
        "",
    },

    {   CI_RET_FL_NOT_CLEAR,                   /* t.b.d. */
        "Fehler",
        "Feld activated_fast_logic[id] ist nicht Null",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "field activated_fast_logic[id] is not zero",
        "",
        "",
        "",
    },

    {   CI_RET_INV_SEMA_TYPE,                     /* t.b.d. */
        "Fehler",
        "ung�ltiger sema type ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid sema type",
        "",
        "",
        "",
    },

    {   CI_RET_SEMA_TWICE,                     /* t.b.d. */
        "Fehler",
        "fuer das User Handle existiert schon ein Object von diesem Typ ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "An object of this type already exists for the user handle",
        "",
        "",
        "",
    },

    {   CI_RET_SEMA_NOT_INITIALIZED,                     /* t.b.d. */
        "Fehler",
        "Semaphore nicht initialisiert",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Semaphore not initialized",
        "",
        "",
        "",
    },

    {   CI_RET_FDL_PARAM,                     /* t.b.d. */
        "Fehler",
        "ung�ltiger Parameter",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid parameter",
        "",
        "",
        "",
    },

    {   CI_RET_FDL_WRONG_SUBSYSTEM,                     /* t.b.d. */
        "Fehler",
        "Subsystem ist nicht 0x22 ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Subsystem is not 0x22 ",
        "",
        "",
        "",
    },

    {   CI_RET_FDL_SEG_LENGTH_1_TOO_BIG,                     /* t.b.d. */
        "Fehler",
        "Seg_length_1 > 260",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Seg_length_1 > 260",
        "",
        "",
        "",
    },

    {   CI_RET_FDL_FILL_LENGTH_1_TOO_BIG,                     /* t.b.d. */
        "Fehler",
        "Fill_length_1 > seg_length_1",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Fill_length_1 > seg_length_1",
        "",
        "",
        "",
    },

    {   CI_RET_FDL_OFFSET_1_INVALID,                     /* t.b.d. */
        "Fehler",
        "Offset_1 != 80 ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Offset_1 != 80 ",
        "",
        "",
        "",
    },

    {   CI_RET_FDL_SEG_LENGTH_2_TOO_BIG,                     /* t.b.d. */
        "Fehler",
        "Seg_length_2 > 260",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Seg_length_2 > 260",
        "",
        "",
        "",
    },

    {   CI_RET_FDL_FILL_LENGTH_2_TOO_BIG,                     /* t.b.d. */
        "Fehler",
        "Fill_length_2 > seg_length",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Fill_length_2 > seg_length",
        "",
        "",
        "",
    },

    {   CI_RET_FDL_OFFSET_2_INVALID,                     /* t.b.d. */
        "Fehler",
        "Offset_2 out of range",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Offset_2 out of range",
        "",
        "",
        "",
    },

    {   CI_RET_FDL_BUF_FILL_LENGTH_INV,                     /* t.b.d. */
        "Fehler",
        "Buf_fill_length invalid",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Buf_fill_length invalid",
        "",
        "",
        "",
    },

    {   CI_RET_FDL_OPEN_MAX,                     /* t.b.d. */
        "Fehler",
        "maximale Anzahl der SCP_open erreicht",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Maximum number of SCP_open reached",
        "",
        "",
        "",
    },

    {   CI_RET_DUMP_OPEN_FILE,                     /* t.b.d. */
        "Fehler",
        "Das File fuer den DPR Abzug kann nicht geoeffnet werden ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "The file for the DPR dump cannot be opened ",
        "",
        "",
        "",
    },

    {   CI_RET_DUMP_FILENAME_TOO_LONG,                     /* t.b.d. */
        "Fehler",
        "File Name zu lang",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "File name too long",
        "",
        "",
        "",
    },

    {   CI_RET_CONNECT_MAX ,                     /* t.b.d. */
        "Fehler",
        "maximale CI_connect_cp Anzahl ueberschritten ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Maximum number of CI_connect_cp exceeded",
        "",
        "",
        "",
    },

    {   CI_RET_DISCONNECT_NOT_CONNECTED_0,                     /* t.b.d. */
        "Fehler",
        "kein mapping angelegt -> interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "No mapping created -> internal error",
        "",
        "",
        "",
    },

    {   CI_RET_DISCONNECT_NOT_CONNECTED_1,                     /* t.b.d. */
        "Fehler",
        "kein mapping angelegt -> interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "No mapping created -> internal error",
        "",
        "",
        "",
    },

    {   CI_RET_DISCONNECT_NOT_CONNECTED_2,                     /* t.b.d. */
        "Fehler",
        "kein mapping angelegt -> interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "No mapping created -> internal error",
        "",
        "",
        "",
    },

    {   CI_RET_CONNECT_NO_CP ,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_CONNECT_DPR,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_CONNECT_PLX,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_CONNECT_DOWN,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_CONNECT_TWICE,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_RCV_EVENT_INV_OPC,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_RELEASE_NO_ACCESS,                     /* t.b.d. */
        "Fehler",
        "User hat keinen Zugriff gehabt",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "User did not have access",
        "",
        "",
        "",
    },
 
    {   CI_RET_GET_MAX_PENDING,                     /* t.b.d. */
        "Fehler",
        "maximale Anzahl pro CI_open �berschritten",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Maximum number per CI_open exceeded",
        "",
        "",
        "",
    },

    {   CI_RET_GET_TIMEOUT,                     /* t.b.d. */
        "Hinweis",
        "Timeout abgelaufen",
        "",
        "",
        "",
        /* from here english */
        "Note",
        "Timeout elapsed",
        "",
        "",
        "",
    },

    {   CI_RET_ALREADY_CONNECTED,                     /* t.b.d. */
        "Fehler",
        "User hat schon Zugriff !",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "User has already access !",
        "",
        "",
        "",
    },


    {   CI_RET_BLINK_INV_MODE,                     /* t.b.d. */
        "Fehler",
        "ungueltiger Mode",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid mode",
        "",
        "",
        "",
    },

    {   CI_RET_BLINK_INV_LED1_MS,                     /* t.b.d. */
        "Fehler",
        "ungueltige Zeit fuer Led1 ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid time for LED1",
        "",
        "",
        "",
    },

    {   CI_RET_BLINK_INV_LED2_MS,                     /* t.b.d. */
        "Fehler",
        "ungueltige Zeit fuer Led2 ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid time for LED2",
        "",
        "",
        "",
    },

    {   CI_RET_GET_CFG_INV_NAME,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_GET_CFG_WRONG_NAME,                     /* t.b.d. */
        "Fehler",
        "interner Fehler",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Internal error",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_HOST_READY,                     /* t.b.d. */
        "Fehler",
        "Datensemaphore hat keinen plausiblen Zustand",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Data semaphore is not in a plausible state",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_MAX_INDEX,                     /* t.b.d. */
        "Fehler",
        "index ung�ltig",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Index invalid",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_SUBSYSTEM,                     /* t.b.d. */
        "Fehler",
        "Subsystem ist ung�ltig ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Subsystem is invalid ",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_INV_FILL_LENGTH_1,                     /* t.b.d. */
        "Fehler",
        "fill_length ist ung�ltig",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "fill_length is invalid",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_INV_NEXT_REQUEST,                     /* t.b.d. */
        "Fehler",
        "next_request ist ung�ltig",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "next_request is invalid",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_INV_NEXT_BLOCK,                     /* t.b.d. */
        "Fehler",
        "next_block ist ung�ltig",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "next_block is invalid",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_NEXT_INDEX,                     /* t.b.d. */
        "Fehler",
        "Requestverkettung fehlerhaft",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Request chaining is incorrect",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_NEXT_BLOCK ,                     /* t.b.d. */
        "Fehler",
        "Requestverkettung fehlerhaft",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Request chaining is incorrect",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_SUB_NOT_IMP,                     /* t.b.d. */
        "Fehler",
        "Subsystem nicht implementiert",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Subsystem not implemented",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_OPEN_HANDLE,                     /* t.b.d. */
        "Fehler",
        "ungueltiges Handle ",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Invalid handle",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_OPEN_ALREADY,                     /* t.b.d. */
        "Fehler",
        "doppelter Open",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Second open",
        "",
        "",
        "",
    },

    {   CI_RET_CIB_CLOSE_ALREADY,                     /* t.b.d. */
        "Fehler",
        "doppelter Close",
        "",
        "",
        "",
        /* from here english */
        "Error",
        "Second close",
        "",
        "",
        "",
    },

    /* This entry terminates the error table */
    {   ERR_TXT_END, 
        ERR_DEFAULT_TXT_D, 
        "",
        "",
        "",
        "",
        /* from here english */
        ERR_DEFAULT_TXT_E,
        "",
        "",
        "",
        "",
    }
};
#endif


#else /* not LARGE_TEXT */



static const ERR_INF_T ErrCodeMinimal[] =
{ 
    /* This entry terminates the error table */
    {   ERR_TXT_END, 
        ERR_DEFAULT_MINIMAL_TXT_D,
        "",
        "",
        "",
        "",
        /* from here english */
        ERR_DEFAULT_MINIMAL_TXT_E,
        "",
        "",
        "",
        "",
    }
}; 




#endif  /* LARGE_TEXT */


#endif  /*  __ERR_TXT__  */
