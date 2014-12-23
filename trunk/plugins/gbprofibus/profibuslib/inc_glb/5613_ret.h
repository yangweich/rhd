/****************************************************************************/
/*    Copyright (C) SIEMENS AG 1989..1998 All Rights Reserved.Confidential  */
/****************************************************************************/
/*                                                                          */
/*     Modulname     : 5613_RET.h                                           */
/*                                                                          */
/****************************************************************************/
/*   Modification                                                           */
/*                                                                          */
/*   Date       Person       Modification                                   */
/*   26.02.98   Me           file created                                   */
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






/* Return values of the Communication Interface(CI) */

#define INFO_MASK  0x00800000L

/* ALLE */
#define CI_RET_OK                        0x00000000L
#define CI_RET_OK_DATA                   0x00000001L /* OK, response data there !*/
#define CI_RET_SET_HWND_MSG              0x00000002L /* Error in the function SetSinecHWndMsg*/

#define CI_RET_INV_USR_OPCODE            0x00000004L /* Invalid subsystem */
#define CI_RET_INV_USR_BUF_LENGTH        0x00000005L /* Buffer too long     */
#define CI_RET_INV_USR_BUF_FILL_LENGTH   0x00000006L /* Error: -> fill_length > buf_length */
#define CI_RET_BUF_NOT_VALID             0x00000007L /* Invalid user buffer */

#define CI_RET_NOT_IMPLEMENTED           0x00000009L /* Function not implemented */
#define CI_RET_NO_DRIVER                 0x0000000AL /* Driver not loaded */
#define CI_RET_CP_NOT_HERE               0x0000000BL /* No CP found !  */

#define CI_RET_HANDLE_INVALID       0x0000000DL /* Invalid value for user handle */
#define CI_RET_HANDLE_NOT_OPEN      0x0000000EL /* Handle unknown */
#define CI_RET_HANDLE_CLOSING       0x00000010L /* Handle being closed */
#define CI_RET_HANDLE_CP_RESET      0x00000011L /* The CP has been resetted -> Terminate the application ! */

/* Internal error identifying the CP with the Plug&Play Bios ,
Remedy -> Contact CP/check computer | replace CP | replace computer */
#define CI_RET_UNMAP                              0x00000020L
#define CI_RET_MAP_DPR_0                          0x00000021L
#define CI_RET_MAP_DPR_1                          0x00000022L
#define CI_RET_MAP_PLX_0                          0x00000023L
#define CI_RET_MAP_PLX_1                          0x00000024L
#define CI_RET_MAP_DOWN_0                         0x00000025L
#define CI_RET_MAP_DOWN_1                         0x00000026L
#define CI_RET_HalAssignSlotResources             0x00000027L
#define CI_RET_CmResourceTypePort                 0x00000028L
#define CI_RET_InterruptMode                      0x00000029L
#define CI_RET_mem_def                            0x0000002AL
#define CI_RET_CmResourceTypeDma                  0x0000002BL
#define CI_RET_CmResourceTypeDeviceSpecific       0x0000002DL
#define CI_RET_CmResourceTypeDefault              0x0000002EL
#define CI_RET_MAP_00                             0x0000002FL
#define CI_RET_MAP_01                             0x00000030L
#define CI_RET_MAP_02                             0x00000031L
#define CI_RET_MAP_03                             0x00000032L
#define CI_RET_mem_res_count                      0x00000033L
#define CI_RET_plx_length                         0x00000034L
#define CI_RET_IoReportResourceUsage              0x00000035L

/* Exceptions -> Internal error in driver or firmware,
Remedy -> Reset the CP or reboot the PC"    */
#define CI_RET_EXCP_OPEN                 0x00000100L
#define CI_RET_EXCP_CLOSE_0              0x00000101L
#define CI_RET_EXCP_CLOSE_1              0x00000102L
#define CI_RET_EXCP_APPEND_0             0x00000103L
#define CI_RET_EXCP_APPEND_1             0x00000104L
#define CI_RET_EXCP_RETURN_0             0x00000105L
#define CI_RET_EXCP_RETURN_1             0x00000106L
#define CI_RET_EXCP_RETURN_2             0x00000107L
#define CI_RET_EXCP_CYCLE_0              0x00000108L
#define CI_RET_EXCP_CYCLE_1              0x00000109L
#define CI_RET_EXCP_CIB                  0x0000010AL
#define CI_RET_EXCP_ALERTED              0x0000010BL
#define CI_RET_EXCP_USER_APC             0x0000010CL
#define CI_RET_EXCP_KE_WAIT              0x0000010DL
#define CI_RET_EXCP_E_LIST_0             0x0000010EL
#define CI_RET_EXCP_E_LIST_1             0x0000010FL
#define CI_RET_EXCP_LIFE_COUNTER         0x00000110L
#define CI_RET_EXCP_CRITICAL_0           0x00000111L
#define CI_RET_EXCP_GET_DP_ACCESS        0x00000112L
#define CI_RET_EXCP_REL_DP_ACCESS_0      0x00000113L
#define CI_RET_EXCP_REL_DP_ACCESS_1      0x00000114L
#define CI_RET_EXCP_QUEUE_DPA_0          0x00000115L
#define CI_RET_EXCP_QUEUE_DPA_1          0x00000116L
#define CI_RET_EXCP_K_LIST_0             0x00000117L
#define CI_RET_EXCP_REL_DP_ACCESS_2      0x00000118L
#define CI_RET_EXCP_TIMEOUT              0x00000119L
#define CI_RET_EXCP_CLOSE_2              0x0000011AL
#define CI_RET_EXCP_REMOVE_DEAD_0        0x0000011BL
#define CI_RET_EXCP_OPEN_1               0x0000011CL



/* OPEN */
#define CI_RET_OPEN_MAX                  0x00001000L /* Maximum number of CI_open exceeded */
#define CI_RET_OPEN_CP_NOT_STARTED       0x00001001L /* CP not started */
#define CI_RET_OPEN_TEMP_LOCKED          0x00001002L /* Service not currently possible */

/* RESET */
#define CI_RET_RESET_CP_USED             (0x00002000L | INFO_MASK) /* Reset not currently possible*/
#define CI_RET_RESET_INVALID_CP_NAME     0x00002001L /* Invalid CP name*/
#define CI_RET_RESET_ALREADY_DONE        (0x00002002L  | INFO_MASK)/* CP is already in the reset state */
#define CI_RET_RESET_INVALID_MODE        0x00002003L /* Invalid reset mode */
#define CI_RET_RESET_CP_NOT_FOUND        0x00002004L /* CP not found */


/* START */
#define CI_RET_START_INVALID_CP_NAME     0x00003000L /* Invalid CP name */
#define CI_RET_START_ALREADY_DONE        (0x00003001L  | INFO_MASK)/* CP already started */
#define CI_RET_START_CP_NOT_FOUND        0x00003002L /* CP not found */
#define CI_RET_START_CP_RESOURCES_INT    0x00003004L /* Internal error -> resource problem */
#define CI_RET_START_ACTION_ERR          0x00003005L /* Internal error */


#define CI_RET_START_MAP                 0x00003009L /* Internal error */ 

#define CI_RET_START_MAX_CP              0x0000300BL /* Maximum number of CPs already reached */
#define CI_RET_START_TEMP_LOCKED         0x0000300CL /* Service not currently possible */
#define CI_RET_START_ERROR_ERR           0x0000300DL /* Internal error */
#define CI_RET_START_CP_NO_REACTION      0x0000300FL /* Internal error -> CP not reacting */
#define CI_RET_START_OK_ERR              0x00003010L /* Internal error */
#define CI_RET_START_INVALID_MODE        0x00003011L /* Invalid start mode */
#define CI_RET_START_TRANSLATE           0x00003012L /* Internal error */

#define CI_RET_START_REG_NO_SWITCH_OUTPUT 0x00003015L /* Internal error */
#define CI_RET_START_ALREADY_DATABASE    0x00003016L /* Second start with a different database */


// LIB
#define CI_RET_NO_CP_NAME                0x00003017L /* The CP name must be specified for the function */
#define CI_RET_CP_NAME_00                0x00003018L /* Internal error */
#define CI_RET_CP_NAME_NOT_FOUND         0x00003019L /* Invalid CP name */
#define CI_RET_OPEN_REG                  0x0000301AL /* Internal error */
#define CI_RET_START_REG_DATABASE        0x0000301BL /* Internal error no database entered */
#define CI_RET_START_REG_DOWNLOAD        0x0000301CL /* Internal error no firmware entered */
#define CI_RET_START_NO_VIEW             0x0000301DL /* Internal error */
#define CI_RET_START_OPEN_FIRMWARE       0x0000301EL /* Could not open firmware file */
#define CI_RET_START_OPEN_DATABASE       0x0000301FL /* Could not open firmware file */
#define CI_RET_START_DOWN_1              0x00003020L /* Internal error */
#define CI_RET_START_FW_INIT_TIMEOUT     0x00003021L /* Internal error -> firmware not responding */
#define CI_RET_START_LEN_FIRMWARE        0x00003022L /* Firmware file is too long */
#define CI_RET_START_LEN_DATABASE        0x00003023L /* Database file is too long */

#define CI_RET_START_FW_INIT_EXCP        0x00003025L /* Internal error -> firmware reporting exception */
#define CI_RET_START_FN_DB_TOO_LONG      0x00003026L /* Database path too long */
#define CI_RET_START_FN_FW_TOO_LONG      0x00003027L /* Firmware path too long */
#define CI_RET_CP561X_ROOT               0x00003028L /* Entry missing in the registry -> installation error */
#define CI_RET_CP_NAME_TOO_LONG          0x00003029L /* CP name is too long */
#define CI_RET_START_REG_gc_data_0       0x0000302AL /* Entry missing in the registry -> installation error */
#define CI_RET_START_REG_gc_data_1       0x0000302BL /* Entry missing in the registry -> installation error */
#define CI_RET_START_REG_gc_group        0x0000302CL /* Entry missing in the registry -> installation error */
#define CI_RET_START_REG_tmsi            0x0000302DL /* Entry missing in the registry -> installation error */
#define CI_RET_START_REG_tmsi_reserve    0x0000302EL /* Entry missing in the registry -> installation error */
#define CI_RET_START_REG_ttr_div_256     0x0000302FL /* Entry missing in the registry -> installation error */
#define CI_RET_START_REG_tth_div_256_equ_dis   0x00003031L /* Entry missing in the registry -> installation error */


/* SEND */
#define CI_RET_SEND_NO_BUFFER_AVAILABLE  0x00004000L /* Lack of resources in DPR */
#define CI_RET_SEND_BUF_TOO_SMALL        0x00004001L /* User buffer for the response is too short */


/* RECEIVE */
#define CI_RET_RECEIVE_TIMEOUT_NO_DATA       0x00005000L /* Timeout elapsed */
#define CI_RET_RECEIVE_BUF_TOO_SMALL_0       0x00005001L /* User buffer is too short */
#define CI_RET_RECEIVE_BUF_TOO_SMALL_1       0x00005002L /* User buffer is too short */
#define CI_RET_RECEIVE_TIMEOUT_CANCEL        0x00005003L /* Receive canceled */
#define CI_RET_RECEIVE_MAX_PENDING           0x00005004L /* Too many receives with timeout per CI_open */
#define CI_RET_RECEIVE_TIMEOUT_USER_APC      0x00005005L /* Internal error */



/* CANCEL */
#define CI_RET_CANCEL_NO_RECEIVE         0x00006000L /* Nothing to cancel */


/* FAST LOGIC */
#define CI_RET_FL_INV_ACTION            0x00007000L /* Internal error */
#define CI_RET_FL_INV_ID                0x00007001L /* invalid parameter fast_logic_id -> 0..3 */
#define CI_RET_FL_INV_ADDR_IN_BYTE      0x00007002L /* invalid address of the input slave*/
#define CI_RET_FL_INV_ADDR_OUT_BYTE     0x00007003L /* invalid address of the output slave */
#define CI_RET_FL_SLAVE_IN_NOT_IN_DB    0x00007004L /* input slave not in database */
#define CI_RET_FL_SLAVE_OUT_NOT_IN_DB   0x00007005L /* output slave not in database*/
#define CI_RET_FL_INV_INDEX_IN_BYTE     0x00007006L /* invalid input byte */
#define CI_RET_FL_INV_INDEX_OUT_BYTE    0x00007007L /* invalid output byte */
#define CI_RET_FL_ALREADY_ON            0x00007008L /* Fast Logic already activated */
#define CI_RET_FL_ALREADY_OFF           0x00007009L /* Fast Logic already deactivated */
#define CI_RET_FL_INV_IN_MASK           0x0000700AL /* invalid mask for input byte */
#define CI_RET_FL_INV_OUT_MASK          0x0000700BL /* invalid mask for output byte */
#define CI_RET_FL_DOUBLE_USER           0x0000700CL /* a second user is not allowed */
#define CI_RET_FL_NOT_CLEAR             0x0000700DL /* field activated_fast_logic[id] is not clear */



/* CI_init_sema */
#define CI_RET_INV_SEMA_TYPE             0x00008000L /* Invalid sema type */
#define CI_RET_SEMA_TWICE                (0x00008001L  | INFO_MASK)/* An object of this type already exists for the user handle */
#define CI_RET_SEMA_NOT_INITIALIZED      0x00008002L /* Semaphore not initialized */




/* FDL errors */
#define CI_RET_FDL_PARAM                 0x00009000L /* Invalid parameter */
#define CI_RET_FDL_WRONG_SUBSYSTEM       0x00009001L /* Subsystem is not 0x22 */
#define CI_RET_FDL_SEG_LENGTH_1_TOO_BIG  0x00009002L /* Seg_length_1 > 260 */
#define CI_RET_FDL_FILL_LENGTH_1_TOO_BIG 0x00009003L /* Fill_length_1 > seg_length_1 */
#define CI_RET_FDL_OFFSET_1_INVALID      0x00009004L /* Offset_1 != 80           */
#define CI_RET_FDL_SEG_LENGTH_2_TOO_BIG  0x00009005L /* Seg_length_2 > 260       */
#define CI_RET_FDL_FILL_LENGTH_2_TOO_BIG 0x00009006L /* Fill_length_2 > seg_length_2 */
#define CI_RET_FDL_OFFSET_2_INVALID      0x00009007L /* Offset_2 out of range    */

#define CI_RET_FDL_BUF_FILL_LENGTH_INV   0x00009008L /* Buf_fill_length invalid  */
#define CI_RET_FDL_OPEN_MAX              0x00009009L /* Maximum number of SCP_open reached */

/* CI_dpr_dump */
#define CI_RET_DUMP_OPEN_FILE            0x0000A000L /* The file for the DPR dump cannot be opened */
#define CI_RET_DUMP_FILENAME_TOO_LONG    0x0000A001L /* File name too long */

/* CONNECT/DISCONNECT */
#define CI_RET_CONNECT_MAX                0x0000B000L /* Maximum number of CI_connect_cp exceeded */
#define CI_RET_DISCONNECT_NOT_CONNECTED_0 0x0000B005L /* No mapping created -> internal error */
#define CI_RET_DISCONNECT_NOT_CONNECTED_1 0x0000B006L /* No mapping created -> internal error */
#define CI_RET_DISCONNECT_NOT_CONNECTED_2 0x0000B007L /* No mapping created -> internal error */
#define CI_RET_CONNECT_NO_CP              0x0000B008L /* Internal error */
#define CI_RET_CONNECT_DPR                0x0000B009L /* Internal error */
#define CI_RET_CONNECT_PLX                0x0000B00AL /* Internal error */
#define CI_RET_CONNECT_DOWN               0x0000B00CL /* Internal error */
#define CI_RET_CONNECT_TWICE              0x0000B00DL /* Internal error */

/* RECEIVE_EVENT */
#define CI_RET_RCV_EVENT_INV_OPC          0x0000C000L  /* Internal error */


/* RELEASE / GET */
#define CI_RET_RELEASE_NO_ACCESS        0x0000D000L /* User did not have access */
#define CI_RET_GET_MAX_PENDING          0x0000D002L /* Maximum number per CI_open exceeded */
#define CI_RET_GET_TIMEOUT              0x0000D003L /* Timeout elapsed */
#define CI_RET_ALREADY_CONNECTED        0x0000D004L /* User has already access */



/* CI_blink */
#define CI_RET_BLINK_INV_MODE            0x0000E000L /* Invalid mode */
#define CI_RET_BLINK_INV_LED1_MS         0x0000E001L /* Invalid time for LED1 */
#define CI_RET_BLINK_INV_LED2_MS         0x0000E002L /* Invalid time for LED2 */


/* CFG */
#define CI_RET_GET_CFG_INV_NAME          0x0000F000L  /* Internal error */
#define CI_RET_GET_CFG_WRONG_NAME        0x0000F001L  /* Internal error */



/* CIB error -> errors can only occur, if not the original driver is used
-> Firmware checks the request a second time*/
#define  CI_RET_CIB_HOST_READY         0x00010000L  /* Data semaphore is not in a plausible state */                  
#define  CI_RET_CIB_MAX_INDEX          0x00010001L  /* Index invalid */
#define  CI_RET_CIB_SUBSYSTEM          0x00010002L  /* Subsystem is invalid */
#define  CI_RET_CIB_INV_FILL_LENGTH_1  0x00010003L  /* fill_length is invalid */
#define  CI_RET_CIB_INV_NEXT_REQUEST   0x00010004L  /* next_request is invalid */
#define  CI_RET_CIB_INV_NEXT_BLOCK     0x00010005L  /* next_block is invalid  */
#define  CI_RET_CIB_NEXT_INDEX         0x00010006L  /* Request chaining is incorrect */
#define  CI_RET_CIB_NEXT_BLOCK         0x00010007L  /* Request chaining is incorrect */
#define  CI_RET_CIB_SUB_NOT_IMP        0x00010008L  /* Subsystem not implemented */
#define  CI_RET_CIB_OPEN_HANDLE        0x00010009L  /* Invalid handle */
#define  CI_RET_CIB_OPEN_ALREADY       0x0001000AL  /* Second open */
#define  CI_RET_CIB_CLOSE_ALREADY      0x0001000BL  /* Second close */

/* DP error codes */
/* ============== */


/* DP_ERROR_REQ_PAR */

#define DP_RET_PAR_MST_MODE          0x00020000   /* parameter mst_mode invalid         */
#define DP_RET_PAR_CP_NAME           0x00020001   /* parameter cp_name invalid          */
#define DP_RET_PAR_USR_HNDL          0x00020002   /* parameter user_handle invalid      */
#define DP_RET_PAR_DPR               0x00020003   /* parameter dual port ram invalid    */
#define DP_RET_PAR_RESET_MODE        0x00020004   /* parameter reset_mode invalid       */
#define DP_RET_PAR_SLV_MODE          0x00020005   /* parameter slv_mode invalid         */
#define DP_RET_PAR_SLV_ADD           0x00020006   /* parameter slv_add invalid          */
#define DP_RET_PAR_TYPE              0x00020007   /* parameter type invalid             */
#define DP_RET_PAR_DATA_LEN          0x00020008   /* parameter data_len invalid         */
#define DP_RET_PAR_DATA              0x00020009   /* parameter data invalid             */
#define DP_RET_PAR_CTRL_CMD          0x0002000A   /* parameter control command invalid  */
#define DP_RET_PAR_REQUEST           0x0002000B   /* parameter request invalid          */
#define DP_RET_PAR_LENGTH_M          0x0002000C   /* parameter length_m invalid         */
#define DP_RET_PAR_SELECTOR          0x0002000D   /* parameter selector invalid         */
#define DP_RET_PAR_REQ_TYPE          0x0002000E   /* parameter req_type invalid         */
#define DP_RET_PAR_RESULT            0x0002000F   /* parameter result invalid           */
#define DP_RET_PAR_SEMA_TYPE         0x00020010   /* parameter sema_type invalid        */
#define DP_RET_PAR_CREF              0x00020011   /* parameter c_ref invalid            */
#define DP_RET_PAR_LENGTH_S          0x00020012   /* parameter length_s invalid         */
#define DP_RET_PAR_ALARM             0x00020013   /* parameter alarm invalid            */
#define DP_RET_CP_REQ_INVALID_LEN    0x00020014   /* request failed (invalid length)    */
#define DP_RET_CP_WRONG_FREEZE_GRP   0x00020015   /* global ctrl failed,wrong freeze group */
#define DP_RET_CP_WRONG_SYNC_GRP     0x00020016   /* global ctrl failed,wrong sync group   */
#define DP_RET_CP_WRONG_GC_CMD       0x00020017   /* global ctrl failed,wrong command      */
#define DP_RET_CP_WRONG_GC_GRP       0x00020018   /* wrong global ctrl group            */
#define DP_RET_PAR_FL                0x00020019   /* parameter fast logic invalid       */
#define DP_RET_PAR_TIMEOUT           0x0002001a   /* parameter timeout invalid          */
#define DP_RET_PAR_WD_INDEX          0x0002001b   /* parameter wd_index invalid         */

/* DP_ERROR_RES */

#define DP_RET_SVR_NOT_SUPPORTED     0x00020100   /* service not supported              */
#define DP_RET_MEMORY                0x00020101   /* internal memory failure            */
#define DP_RET_CP_NO_DATABASE        0x00020102   /* no (valid) database available      */
#define DP_RET_CP_DATABASE_ADR       0x00020103   /* wrong slave address in db          */
#define DP_RET_CP_ADR_NOT_IN_DB      0x00020104   /* slave address not in database      */
#define DP_RET_CP_NO_BUS_PAR         0x00020105   /* no busparameter in database        */
#define DP_RET_CP_NO_DP_PAR          0x00020106   /* no DP parameter in database        */
#define DP_RET_CP_MEMORY_DPMC        0x00020107   /* internal memory failure            */
#define DP_RET_CP_TIMER              0x00020108   /* internal timer failure             */
#define DP_RET_CP_TOO_MANY_SLV       0x00020109   /* too many slaves                    */
#define DP_RET_CP_TOO_MANY_USR       0x0002010A   /* too many user                      */
#define DP_RET_CP_MEMORY             0x0002010B   /* internal memory failure            */
#define DP_RET_CP_MEMORY_1           0x0002010C   /* internal memory failure            */
#define DP_RET_CP_MEMORY_2           0x0002010D   /* internal memory failure            */
#define DP_RET_CP_MEMORY_3           0x0002010E   /* internal memory failure            */
#define DP_RET_CP_MEMORY_4           0x0002010F   /* internal memory failure            */
#define DP_RET_CP_MEMORY_5           0x00020110   /* internal memory failure            */
#define DP_RET_CP_MEMORY_6           0x00020111   /* internal memory failure            */
#define DP_RET_CP_MEMORY_7           0x00020112   /* internal memory failure            */

#define DP_RET_TOO_MANY_USR          0x00020113   /* too many user (dp_base.dll)        */


/* DP_ERROR_EVENT_NET */

#define DP_RET_TIMEOUT              (0x00020200 | INFO_MASK)  /* timeout at request(DP_Base)           */
#define DP_RET_CP_TIMEOUT            0x00020201   /* The job was terminated due to a timeout of the CP */
#define DP_RET_CP_REQ_NOT_ALLOWED    0x00020202   /* Service not allowed                */
#define DP_RET_CP_INIT_INSTANCE      0x00020203   /* DP setup failed                    */
#define DP_RET_CP_CONTROL_COMMAND    0x00020204   /* Control Command not allowed        */
#define DP_RET_CP_WRONG_INSTANCE     0x00020205   /* wrong user instance has accessed   */
#define DP_RET_CP_RESET_INSTANCE     0x00020206   /* DP reset failed                    */
#define DP_RET_CP_RESET_RUNNING      0x00020207   /* reset already activated            */
#define DP_RET_CP_UNKNOWN_SLV_TYPE   0x00020208   /* slave type unknown                 */
#define DP_RET_CP_WRONG_MODE_OFL     0x00020209   /* wrong mode at set_mode request(act.state=Offline) */
#define DP_RET_CP_WRONG_MODE_STP     0x0002020A   /* wrong mode at set_mode request(act.state=Stop)    */
#define DP_RET_CP_WRONG_MODE_CLR     0x0002020B   /* wrong mode at set_mode request(act.state=Clear)   */
#define DP_RET_CP_WRONG_MODE_OPR     0x0002020C   /* wrong mode at set_mode request(act.state=Operate) */

#define DP_RET_CP_SLV_NOT_ACTIV      0x0002020D   /* slave inactivated                  */
#define DP_RET_CP_SLV_NOT_IN_DATA    0x0002020E   /* slave not in ready state           */            
#define DP_RET_CP_REQ_ACTIV          0x0002020F   /* DP request already activ           */
#define DP_RET_CP_SET_MODE_FAIL      0x00020210   /* error during set mode              */
#define DP_RET_CP_CLOSED             0x00020211   /* The processing instance in the firmware was terminated earlier and no longer exists*/
#define DP_RET_CP_STOPPED            0x00020212   /* The processing instance has already been stopped*/
#define DP_RET_CP_STARTED            0x00020213   /* The processing instance has already been started*/
#define DP_RET_CP_STATE_UNKNOWN      0x00020214   /* The processing instance is in an unknown state*/
#define DP_RET_CP_REQ_WITHDRAW       0x00020215   /* The job was withdrawn and cannot be executed*/
#define DP_RET_CP_REQ_NOT_FOUND      0x00020216   /* The job field for the request was not found*/
#define DP_RET_CP_REQ_NEG            0x00020217   /* Negative acknowledgment for job sent on Profibus,
Possible causes: Slave not responding or the service access point on the   
slave is not activated */
#define DP_RET_CP_L2_REQ             0x00020218   /* error opcode in the confirmation */
#define DP_RET_CP_REQ_RE             0x00020219   /* Format error in a response frame, Source: local DP instance (direct data link mapper)*/
#define DP_RET_CP_MM_FE              0x0002021A   /* Format error in a request frame, Source: remote DP instance (direct data link mapper)*/
#define DP_RET_CP_MM_NI              0x0002021B   /* Function not implemented, Source: remote user*/
#define DP_RET_CP_MM_AD              0x0002021C   /* Access denied, Source: remote user*/
#define DP_RET_CP_MM_EA              0x0002021D   /* Area too large (up/download), Source: remote user*/
#define DP_RET_CP_MM_LE              0x0002021E   /* Data block too long (Up/download), Source: remote user*/
#define DP_RET_CP_MM_RE              0x0002021F   /* Format error in a response frame, Source: local DP instance (direct data link mapper)*/
#define DP_RET_CP_MM_IP              0x00020220   /* Invalid parameter, Source: remote user*/
#define DP_RET_CP_MM_SC              0x00020221   /* Sequence conflict, Source: remote user*/
#define DP_RET_CP_MM_SE              0x00020222   /* Sequence error, Source: remote DP instance (direct data link mapper)*/
#define DP_RET_CP_MM_NE              0x00020223   /* Area does not exist, Source: remote user*/
#define DP_RET_CP_MM_DI              0x00020224   /* Data incomplete, Source: remote user*/
#define DP_RET_CP_MM_NC              0x00020225   /* Master parameter set not compatible, Source: remote user*/
#define DP_RET_CP_REQ_INVALID_PAR    0x00020226   /* The job was terminated with an error due an invalid parameter */
#define DP_RET_REQ_ACTIV             0x00020227   /* request still activ                */
#define DP_RET_CP_SET_MODE_OFFL_ACT  0x00020228   /* The DP_set_mode call could not be executed because a previous, DP_set_mode call (change to offline status) is still active*/
#define DP_RET_CP_SET_MODE_STOP_ACT  0x00020229   /* The DP_set_mode call could not be executed because a previous, DP_set_mode call (change to stop status) is still active.*/
#define DP_RET_CP_SET_MODE_CLR_ACT   0x0002022A   /* The DP_set_mode call could not be executed because a previous, DP_set_mode call (change to clear status) is still active.*/
#define DP_RET_CP_SET_MODE_OPR_ACT   0x0002022B   /* The DP_set_mode call could not be executed because a previous, DP_set_mode call (change to operate status) is still active.*/
/* V1.2 */                              
#define DP_RET_CP_USR_NOT_COMPATIBLE 0x0002022C   /* DP_BASE together with DP_LIB are incompatible */
#define DP_RET_CP_TOO_MANY_CTRL_CMD  0x0002022D   /* Too many global control commands are activ    */


#define CI_RET_BESY_ERROR           0xF0000000L   /* Internal error  */

#define CI_RET_RESERVED_CP_STATE    0x0F000000L   /* Internal error */

/****************************************************************************/
/*     END OF FILE:      5613_RET.H                                         */
/****************************************************************************/
