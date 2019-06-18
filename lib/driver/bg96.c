/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#include "bg96.h"
#include "gps.h"
#include "nrf_drv_rtc.h"
#include "nrf_rtc.h"
#include "nrf_drv_gpiote.h"
#include "hal_uart.h"

#include <stdio.h>
#include <stdlib.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define GSM_RXBUF_MAXSIZE 1600

static uint16_t rxReadIndex = 0;
static uint16_t rxWriteIndex = 0;
static uint16_t rxCount = 0;
static uint8_t Gsm_RxBuf[GSM_RXBUF_MAXSIZE];

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

extern GSM_RECEIVE_TYPE g_type;

char GSM_RSP[1600] = {0};

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

uint32_t get_stamp(void)
{
    uint32_t ticks = nrf_drv_rtc_counter_get(&rtc);
    return (ticks / RTC_DEFAULT_CONFIG_FREQUENCY);
}

int GSM_UART_TxBuf(uint8_t *buffer, int nbytes)
{
    uint32_t err_code;
    for (uint32_t i = 0; i < nbytes; i++)
    {
        do
        {
            err_code = app_uart_put(buffer[i]);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_BUSY);
    }
    return err_code;
}

void Gsm_RingBuf(uint8_t in_data)
{
    Gsm_RxBuf[rxWriteIndex] = in_data;
    rxWriteIndex++;
    rxCount++;

    if (rxWriteIndex == GSM_RXBUF_MAXSIZE)
    {
        rxWriteIndex = 0;
    }

    /* Check for overflow */
    if (rxCount == GSM_RXBUF_MAXSIZE)
    {
        rxWriteIndex = 0;
        rxCount = 0;
        rxReadIndex = 0;
    }
}

void Gsm_Gpio_Init(void)
{
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_write(LED_PIN, 1);

    nrf_gpio_cfg_output(GSM_PWR_ON_PIN);
    nrf_gpio_pin_write(GSM_PWR_ON_PIN, 0);
    nrf_gpio_cfg_output(GSM_RESET_PIN);
    nrf_gpio_pin_write(GSM_RESET_PIN, 0);
    nrf_gpio_cfg_output(GSM_PWRKEY_PIN);
}

void Gsm_PowerUp(void)
{
    NRF_LOG_INFO("GMS_PowerUp");
    nrf_gpio_pin_write(GSM_PWR_ON_PIN, 1);
    nrf_delay_ms(2000);
    nrf_gpio_pin_write(GSM_PWRKEY_PIN, 1);
    nrf_delay_ms(500);
    nrf_gpio_pin_write(GSM_PWRKEY_PIN, 0);
    nrf_delay_ms(2000);
    nrf_gpio_pin_write(GSM_RESET_PIN, 1);
    nrf_delay_ms(100);
    nrf_gpio_pin_write(GSM_RESET_PIN, 0);
    nrf_delay_ms(500);

    NRF_LOG_INFO("GMS_PowerUp OK");
}

int Gsm_RxByte(void)
{
    int c = -1;

    //__disable_irq();

    if (rxCount > 0)
    {
        c = Gsm_RxBuf[rxReadIndex];

        rxReadIndex++;
        if (rxReadIndex == GSM_RXBUF_MAXSIZE)
        {
            rxReadIndex = 0;
        }
        rxCount--;
    }
    //__enable_irq();

    return c;
}

int Gsm_WaitRspOK(char *rsp_value, uint16_t timeout_ms, uint8_t is_rf)
{
    int ret = -1, wait_len = 0;
    char len[10] = {0};
    uint16_t time_count = timeout_ms;
    uint32_t i = 0;
    int c;
    char *cmp_p = NULL;

    wait_len = is_rf ? strlen(GSM_CMD_RSP_OK_RF) : strlen(GSM_CMD_RSP_OK);
//    NRF_LOG_INFO("WAIT LEN=");
//    NRF_LOG_INFO(wait_len
    SEGGER_RTT_printf(0,"[Gsm_WaitRspOK]\r\n");
    if (g_type == GSM_TYPE_FILE)
    {
          NRF_LOG_INFO("GSM_TYPE_FILE");
        //        do
        //        {
        //            c = Gsm_RxByte();
        //            if(c < 0)
        //            {
        //                time_count--;
        //                nrf_delay_ms(1);
        //                continue;
        //            }

        //            rsp_value[i++] = (char)c;
        //            //NRF_LOG_INFO("%02X", rsp_value[i - 1]);
        //            time_count--;
        //        }
        //        while(time_count > 0);
    }
    else
    {
//        NRF_LOG_INFO("<<<<");
        memset(GSM_RSP, 0, 1600);
        do
        {
            int c;
            c = Gsm_RxByte();
            if (c < 0)
            {
                time_count--;
                nrf_delay_ms(1);
                continue;
            }

            GSM_RSP[i++] = (char)c;

            if (i >= 0 && rsp_value != NULL)
            {
                if (is_rf)
                    cmp_p = strstr(GSM_RSP, GSM_CMD_RSP_OK_RF);
                else
                    cmp_p = strstr(GSM_RSP, GSM_CMD_RSP_OK);
                if (cmp_p)
                {
                    if (i > wait_len && rsp_value != NULL)
                    {
                        //NRF_LOG_INFO("SHOULD PRINT");
                        SEGGER_RTT_printf(0,"[rsp_value] --%s  len=%d", rsp_value, i);
                        memcpy(rsp_value, GSM_RSP, i);
                    }
                    ret = 0;
                    break;
                }
            }
        } while (time_count > 0);
    }

    return ret;
}

//The shut off api is emergency cmd. For safe, please use "AT+QPOWD=1" and it will cost less than 60s.
void Gsm_PowerDown(void)
{
    int ret = -1;
    Gsm_print("AT+QPOWD=0");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
}

int Gsm_WaitSendAck(uint16_t timeout_ms)
{
    int ret = -1;
    uint16_t time_count = timeout_ms;
    do
    {
        int c;
        c = Gsm_RxByte();
        if (c < 0)
        {
            time_count--;
            vTaskDelay(1);
            continue;
        }
        //R485_UART_TxBuf((uint8_t *)&c,1);
        if ((char)c == '>')
        {
            ret = 0;
            break;
        }
    } while (time_count > 0);

    //DPRINTF(LOG_DEBUG,"");
    return ret;
}

//int Gsm_AutoBaud(void)
//{
//    int ret = -1, retry_num = GSM_AUTO_CMD_NUM;
//    //
//    char *cmd;

//    cmd = (char *)malloc(GSM_GENER_CMD_LEN);
//    if(cmd)
//    {
//        uint8_t cmd_len;
//        memset(cmd, 0, GSM_GENER_CMD_LEN);
//        cmd_len = sprintf(cmd, "%s", GSM_AUTO_CMD_STR);
//        do
//        {
//            NRF_LOG_INFO(" auto baud retry");
//            GSM_UART_TxBuf((uint8_t *)cmd, cmd_len);

//            ret = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT, NULL);
//            vTaskDelay(500);
//            retry_num--;
//        }
//        while(ret != 0 && retry_num > 0);

//        free(cmd);
//    }
//    NRF_LOG_INFO("Gsm_AutoBaud ret= %d", ret);
//    return ret;
//}

//int Gsm_FixBaudCmd(int baud)
//{
//    int ret = -1;
//    char *cmd;

//    cmd = (char*)malloc(GSM_GENER_CMD_LEN);
//    if(cmd)
//    {
//        uint8_t cmd_len;
//        memset(cmd, 0, GSM_GENER_CMD_LEN);
//        cmd_len = sprintf(cmd, "%s%d%s\r\n", GSM_FIXBAUD_CMD_STR, baud, ";&W");
//        GSM_UART_TxBuf((uint8_t *)cmd, cmd_len);

//        ret = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT, true);

//        free(cmd);
//    }
//    NRF_LOG_INFO("Gsm_FixBaudCmd ret= %d", ret);
//    return ret;
//}

//close cmd echo
//int Gsm_SetEchoCmd(int flag)
//{
//    int ret = -1;
//    char *cmd;

//    cmd = (char *)malloc(GSM_GENER_CMD_LEN);
//    if(cmd)
//    {
//        uint8_t cmd_len;
//        memset(cmd, 0, GSM_GENER_CMD_LEN);
//        cmd_len = sprintf(cmd, "%s%d\r\n", GSM_SETECHO_CMD_STR, flag);
//        GSM_UART_TxBuf((uint8_t *)cmd, cmd_len);

//        ret = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT, true);

//        free(cmd);
//    }
//    NRF_LOG_INFO("Gsm_SetEchoCmd ret= %d", ret);
//    return ret;
//}
//Check SIM Card Status

//int Gsm_CheckSimCmd(void)
//{
//    int ret = -1;
//    //
//    char *cmd;

//    cmd = (char *)malloc(GSM_GENER_CMD_LEN);
//    if(cmd)
//    {
//        uint8_t cmd_len;
//        memset(cmd, 0, GSM_GENER_CMD_LEN);
//        cmd_len = sprintf(cmd, "%s\r\n", GSM_CHECKSIM_CMD_STR);
//        GSM_UART_TxBuf((uint8_t *)cmd, cmd_len);

//        memset(cmd, 0, GSM_GENER_CMD_LEN);
//        ret = Gsm_WaitRspOK(cmd, GSM_GENER_CMD_TIMEOUT, true);
//        NRF_LOG_INFO("Gsm_CheckSimCmd cmd= %s", cmd);
//        if(ret >= 0)
//        {
//            if(NULL != strstr(cmd, GSM_CHECKSIM_RSP_OK))
//            {
//                ret = 0;
//            }
//            else
//            {
//                ret = -1;
//            }
//        }

//        free(cmd);
//    }
//    NRF_LOG_INFO("Gsm_CheckSimCmd ret= %d", ret);
//    return ret;
//}

void Gsm_print(uint8_t *at_cmd)
{
    uint8_t cmd_len;
    uint8_t CMD[512] = {0};
    if (at_cmd == NULL)
        return;
    memset(CMD, 0, 512);
    cmd_len = sprintf((char *)CMD, "%s\r\n", at_cmd);
    SEGGER_RTT_printf(0, "\r\n[Gsm_print] cmd=%s\r\n", CMD);
    GSM_UART_TxBuf(CMD, cmd_len);
}

void Gsm_nb_iot_config(void)
{
    int ret = -1;
#if 0
    //query the info of BG96 GSM
    Gsm_print("ATI");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("ATI ret= %d", ret);
    vTaskDelay(1000);
    //Set Phone Functionality
    Gsm_print("AT+CFUN?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+CFUN? ret= %d", ret);
    vTaskDelay(1000);
    //Query Network Information
    Gsm_print("AT+QNWINFO");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QNWINFO ret= %d", ret);
    vTaskDelay(1000);
    //Network Search Mode Configuration:0->Automatic,1->3->LTE only ;1->Take effect immediately
    Gsm_print("AT+QCFG=\"nwscanmode\",3,1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QCFG=\"nwscanmode\" ret= %d", ret);
    vTaskDelay(1000);
    //LTE Network Search Mode
    Gsm_print("AT+QCFG=\"IOTOPMODE\"");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QCFG=\"IOTOPMODE\" ret= %d", ret);
    vTaskDelay(1000);
    //Network Searching Sequence Configuration
    Gsm_print("AT+QCFG=\"NWSCANSEQ\"");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QCFG=\"NWSCANSEQ\" ret= %d", ret);
    vTaskDelay(1000);
    //Band Configuration
    Gsm_print("AT+QCFG=\"BAND\"");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 10, true);
    NRF_LOG_INFO("AT+QCFG=\"BAND\" ret= %d", ret);
    vTaskDelay(8000);
    //(wait reply of this command for several time)Operator Selection
    Gsm_print("AT+COPS=?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 200, true);
    NRF_LOG_INFO("AT+COPS=? ret= %d", ret);
    vTaskDelay(1000);
    //Switch on/off Engineering Mode
    Gsm_print("AT+QENG=\"SERVINGCELL\"");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QENG=\"SERVINGCELL\" ret= %d", ret);
    vTaskDelay(1000);
    //Activate or Deactivate PDP Contexts
    Gsm_print("AT+CGACT?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+CGACT? ret= %d", ret);
    vTaskDelay(1000);
    //Show PDP Address
    Gsm_print("AT+CGPADDR=1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+CGPADDR=1 ret= %d", ret);
    vTaskDelay(1000);
    //show signal strenth
    Gsm_print("AT+CSQ");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+CSQ ret= %d", ret);
    vTaskDelay(1000);
    //show net register status
    Gsm_print("AT+CEREG?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+CEREG? ret= %d", ret);
    vTaskDelay(1000);

    Gsm_print("AT+QIOPEN=1,0,\"TCP\",\"192.168.0.106\",60000,0,2");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+QIOPEN=1,0,\"TCP\",\"192.168.0.106\",60000,0,2 ret= %d", ret);
    vTaskDelay(1000);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 200, true);
    vTaskDelay(1000);
    Gsm_print("AT+QISTATE");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+QISTATE GSM_RSP = %s", GSM_RSP);
    vTaskDelay(1000);
    //open a socket of tcp as a client
//  Gsm_print("AT+QIOPEN=1,1,\"TCP LISTENER\",\"127.0.0.1\",0,2020,0");
//  memset(GSM_RSP,0,GSM_GENER_CMD_LEN);
//  ret=Gsm_WaitRspOK(GSM_RSP,GSM_GENER_CMD_TIMEOUT * 40,true);
//  DPRINTF(LOG_DEBUG,"AT+QIOPEN=1,1,\"TCP LISTENER\",\"127.0.0.1\",0,2020,0 ret= %d\r\n",ret);
//  vTaskDelay(1000);
//	ret=Gsm_WaitRspOK(GSM_RSP,GSM_GENER_CMD_TIMEOUT * 200,true);
//	vTaskDelay(1000);
//	Gsm_print("AT+QISTATE");
//  memset(GSM_RSP,0,GSM_GENER_CMD_LEN);
//  ret=Gsm_WaitRspOK(GSM_RSP,GSM_GENER_CMD_TIMEOUT * 40,true);
//	DPRINTF(LOG_DEBUG,"AT+QISTATE GSM_RSP = %s\r\n",GSM_RSP);
//	vTaskDelay(1000);

    //open a socket of tcp as a server listener because only listener can recieve update file
#endif
#if 1
    Gsm_print("AT+COPS=?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 120, true);
    NRF_LOG_INFO("AT+COPS=? %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+COPS=1,0,\"CHINA MOBILE\",0");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+COPS=1 %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QNWINFO");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QNWINFO %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QICSGP=1,1,\"CMCC\","
              ","
              ",1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+QICSGP=1 %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QIACT=1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+QIACT=1 %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QIACT?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+QIACT? %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QPING=1,\"www.baidu.com\"");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+QPING? %s", GSM_RSP);
    vTaskDelay(1000);

#endif
}

void gsm_send_test(void)
{
    int ret = -1;
    int len = 0;
    NRF_LOG_INFO("+++++send gps data++++");
    Gsm_print("AT+QISEND=1,75");
    Gsm_print("$GPGGA,134303.00,3418.040101,N,10855.904676,E,1,07,1.0,418.5,M,-28.0,M,,*4A");
    ret = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO(" gps_data send ret= %d", ret);
    NRF_LOG_INFO("+++++send sensor data++++");
    Gsm_print("AT+QISEND=1,170");
    ret = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT * 40, true);
}

int Gsm_test_hologram(void)
{
    int ret = -1;
    NRF_LOG_INFO("Gsm_test_hologram begin");
    Gsm_print("ATI");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 120, true);
    NRF_LOG_INFO("ATI %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+COPS=?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 120, true);
    NRF_LOG_INFO("AT+COPS=? %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+COPS=1,0,\"CHINA MOBILE\",0");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+COPS=1,0,\"CHINA MOBILE\",0 %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+CREG?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+CREG? %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QNWINFO");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QNWINFO %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+COPS?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+COPS? %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QICSGP=1,1,\"hologram\",\"\",\"\",1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 10, true);
    NRF_LOG_INFO("AT+QICSGP= %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QIACT=1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 120, true);
    NRF_LOG_INFO("AT+QIACT=1 %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QIACT?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+QIACT? %s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QIOPEN=1,0,\"TCP\",\"cloudsocket.hologram.io\",9999,0,1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 60, true);
    NRF_LOG_INFO("AT+QIOPEN=1,0,\"TCP\",\"cloudsocket.hologram.io\",9999,0,1 %s", GSM_RSP);
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 20, true);
    NRF_LOG_INFO("%s", GSM_RSP);
    vTaskDelay(1000);
    Gsm_print("AT+QISEND=0,48");
    vTaskDelay(1000);
    Gsm_print("{\"k\":\"+C7pOb8=\",\"d\":\"Hello,World!\",\"t\":\"TOPIC1\"}");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 60, true);
    NRF_LOG_INFO("%s", GSM_RSP);
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    ret = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 80, true);
    NRF_LOG_INFO("%s", GSM_RSP);
    Gsm_print("AT+QICLOSE=0");
    vTaskDelay(5000);
    return ret;
}

//Check Network register Status
int Gsm_CheckNetworkCmd(void)
{
    int ret = -1;
    //
    //    char *cmd;

    //    cmd = (char *)malloc(GSM_GENER_CMD_LEN);
    //    if(cmd)
    //    {
    //        uint8_t cmd_len;
    //        memset(cmd, 0, GSM_GENER_CMD_LEN);
    //        cmd_len = sprintf(cmd, "%s\r\n", GSM_CHECKNETWORK_CMD_STR);
    //        //NRF_LOG_INFO( "%s", cmd);
    //        GSM_UART_TxBuf((uint8_t *)cmd, cmd_len);
    //        memset(cmd, 0, GSM_GENER_CMD_LEN);
    //        ret = Gsm_WaitRspOK(cmd, GSM_GENER_CMD_TIMEOUT, true);

    //        if(ret >= 0)
    //        {

    //            if (strstr(cmd, GSM_CHECKNETWORK_RSP_OK))
    //            {
    //                ret = 0;
    //            }
    //            else if (strstr(cmd, GSM_CHECKNETWORK_RSP_OK_5))
    //            {
    //                ret = 0;
    //            }
    //            else
    //            {
    //                ret = -1;
    //            }
    //        }

    //        free(cmd);
    //    }
    return ret;
}

//void Gsm_CheckAutoBaud(void)
//{
//    uint8_t  is_auto = true, i = 0;
//    uint16_t time_count = 0;
//    uint8_t  str_tmp[64];

//    nrf_delay_ms(800);
//    //check is AutoBaud
//    memset(str_tmp, 0, 64);

//    do
//    {
//        int       c;
//        c = Gsm_RxByte();
//        if(c <= 0)
//        {
//            time_count++;
//            nrf_delay_ms(2);
//            continue;
//        }

//        //R485_UART_TxBuf((uint8_t *)&c,1);
//        if(i < 64)
//        {
//            str_tmp[i++] = (char)c;
//        }

//        if (i > 3 && is_auto == true)
//        {
//            if(strstr((const char*)str_tmp, FIX_BAUD_URC))
//            {
//                is_auto = false;
//                time_count = 800;  //Delay 400ms
//            }
//        }
//    }
//    while(time_count < 1000);   //time out 2000ms

//    if(is_auto == true)
//    {
//        Gsm_AutoBaud();

//        NRF_LOG_INFO("  Fix baud");
//        Gsm_FixBaudCmd(GSM_FIX_BAUD);
//    }
//}

int Gsm_Init(void)
{
    //int  ret;
    NRF_LOG_INFO("Gsm_Init");
    int time_count;

    Gsm_Gpio_Init();

    rak_uart_init(GSM_USE_UART, GSM_RXD_PIN, GSM_TXD_PIN, UARTE_BAUDRATE_BAUDRATE_Baud57600);


    Gsm_PowerDown();
    Gsm_PowerUp();
    

    char str_tmp[64];
    memset(str_tmp, 0, 64);
    Gsm_print("AT");
    Gsm_WaitRspOK(str_tmp, 1000, true);
    nrf_delay_ms(2);
    Gsm_print("AT+CGNSPWR=1");
    Gsm_WaitRspOK(str_tmp, 1000, true);
    nrf_delay_ms(2);
    Gsm_print("AT+CGNSPWR?");
    Gsm_WaitRspOK(str_tmp, 1000, true);
    nrf_delay_ms(2);
	
    Gsm_print("AT+CPIN?");//Check SIM card							
    Gsm_WaitRspOK(str_tmp, 1000, true);
    nrf_delay_ms(100);
		
    Gsm_print("AT+CNMP=38");//Set net mod NB-IOT
    Gsm_WaitRspOK(str_tmp, 1000, true);

    nrf_delay_ms(2);
    Gsm_print("AT+CMNB=2");//Set net mod NB-IOT
    Gsm_WaitRspOK(str_tmp, 1000, true);
		
    //gps_config();
//    //Segger_RTT_printf(0, "Waiting 1000ms\r\n");
    nrf_delay_ms(1000);
    return 0;
}
/**
* @}
*/

void Gps_data_update(uint8_t data)
{
}
