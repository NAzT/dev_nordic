#ifndef _BG96_H
#define _BG96_H

#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "app_error.h"
#include "board_basic.h"

#define  GSM_GENER_CMD_LEN                    (128)
#define  GSM_GENER_CMD_TIMEOUT                (500)  //ms
#define  GSM_OPENSOCKET_CMD_TIMEOUT           (15000)  //ms
#define  GSM_GETDNSIP_CMD_TIMEOUT             (30*1000)  //ms

#define  GSM_FIX_BAUD           							(57600)  //baud
#define  GSM_CHECKSIM_RETRY_NUM    						(5)//  60S
#define  GSM_CHECKSIM_RETRY_TIME    					(1000)//  1000ms

#define  GSM_AUTO_CMD_NUM                     (10)

#define  GSM_TCP_STR        									"\"TCP\""
#define  GSM_UDP_STR        									"\"UDP\""
#define  GSM_TCP_TYPE        									(0)
#define  GSM_UDP_TYPE        									(1)

#define  GSM_SOCKET_CONNECT_ERR        				(-2)

#define  GSM_OPENSOCKET_OK_STR          "CONNECT OK\r\n"
#define  GSM_OPENSOCKET_FAIL_STR        "CONNECT FAIL\r\n"
#define  FIX_BAUD_URC            				"RDY\r\n"
#define  GSM_CMD_CRLF            				"\r\n"
#define  GSM_CMD_RSP_OK_RF        			"OK\r\n"
#define  GSM_CMD_RSP_OK           	 		"OK"
#define  GSM_CHECKSIM_RSP_OK           	"+CPIN: READY"

#define  GSM_CHECKNETWORK_RSP_OK       	"+CREG: 0,1"
#define  GSM_CHECKNETWORK_RSP_OK_5      "+CREG: 0,5"
#define  GSM_CHECKGPRS_RSP_OK          	"+CGATT: 1"
#define  GSM_GETRSSI_RSP_OK       			"+CSQ: "

#define  GSM_IPSTATUS_CONNECTING        "TCP CONNECTING"
#define  GSM_AUTO_CMD_STR               "AT"
#define  GSM_FIXBAUD_CMD_STR            "AT+IPR="
#define  GSM_SETECHO_CMD_STR            "ATE"
#define  GSM_CHECKSIM_CMD_STR           "AT+CPIN?"

#define  GSM_CHECKNETWORK_CMD_STR       "AT+CREG?"

#define  GSM_CHECKGPRS_CMD_STR          "AT+CGATT?"
#define  GSM_SETCONTEXT_CMD_STR         "AT+QIFGCNT=0"
#define  GSM_SETDNSMODE_CMD_STR         "AT+QIDNSIP=1"
#define  GSM_SETSENDECHO_CMD_STR        "AT+QISDE=0"
#define  GSM_ATS_ENABLE_CMD_STR         "ATS0=1"

#define  GSM_OPENSOCKET_CMD_STR         "AT+QIOPEN="
#define  GSM_IPSTATIC_CMD_STR        		"AT+QISTAT"
#define  GSM_RSSI_CMD_STR        				"AT+CSQ"
#define  GSM_SENDDATA_CMD_STR           "AT+QISEND="
#define  GSM_CLOSESOCKET_CMD_STR        "AT+QICLOSE"

#define  GSM_DNS_GETIP_CMD_STR         "AT+QIDNSGIP="
#define  GSM_HTTP_SETURL_CMD_STR         "AT+QHTTPURL="
#define  GSM_HTTP_GET_CMD_STR         "AT+QHTTPGET="
#define  GSM_HTTP_READ_CMD_STR         "AT+QHTTPREAD="
#define  GSM_HTTP_POST_CMD_STR         "AT+QHTTPPOST="
#define  GSM_HTTP_DL_CMD_STR         "AT+QHTTPDL="


#define TRANSFE_EVENT_POSTDATA  (0x01)

/*
*********************************************************************************************************
*                                         EXTERN FUNCTION
*********************************************************************************************************
*/

void Gsm_CheckAutoBaud(void);
//cmd list
int Gsm_AutoBaud(void);
int Gsm_FixBaudCmd(int baud);
int Gsm_SetEchoCmd(int flag);
int Gsm_CheckSimCmd(void);
int Gsm_CheckNetworkCmd(void);
void Gsm_print(uint8_t *at_cmd);
void Gsm_nb_iot_config(void);
void gsm_send_test(void);
void gps_data_get(uint8_t *data, uint8_t len);
void gps_config(void);
void gps_parse(uint8_t *data);
int Gsm_WaitRspOK(char *rsp_value, uint16_t timeout_ms, uint8_t is_rf);
int Gsm_Init(void);
void Gsm_RingBuf(uint8_t in_data);
void Gps_data_update(uint8_t data);
#endif

