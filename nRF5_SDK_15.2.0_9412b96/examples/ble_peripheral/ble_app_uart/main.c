/** @file

   @defgroup ble_sdk_uart_over_ble_main main.c
   @{
   @ingroup  ble_sdk_app_nus_eval
   @brief    UART over BLE application main file.

   This file contains the source code for a sample application that uses the Nordic UART service.
   This application uses the @ref srvlib_conn_params module.
*/

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

#include "semphr.h"
#include "timers.h"

#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_rtc.h"

#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "ble_nus.h"
#include "board_basic.h"
#include "bsp_btn_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "pin_define.h"

#define serverIP "iot.dfrobot.com.cn"
#define IOT_CLIENT " CLIENT NAME "
#define IOT_USERNAME " USER   NAME "
#define IOT_KEY " PASSWORD    "
#define IOT_TOPIC " TOPIC       "

static uint8_t unlock_command = 0;
static uint8_t lock_status = 0;
static uint8_t ble_status = 0;
GSM_RECEIVE_TYPE g_type = GSM_TYPE_CHAR;
uint8_t cmd[128] = {0};
extern char GSM_RSP[1600];
char latitude[8];
char longitude[8];

#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME "Mobike_DEV"                         /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL 64 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION 18000 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)    /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)    /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY 0                                      /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                         /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                           /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);               /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;               /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[] =                                      /**< Universally unique service identifier. */
    {
        {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};

/**@brief Function for assert macro callback.

   @details This function will be called in case of an assert in the SoftDevice.

   @warning This handler is an example only and does not fit a final product. You need to analyse
            how your product is supposed to react in case of Assert.
   @warning On assert from the SoftDevice, the system can only recover on reset.

   @param[in] line_num    Line number of the failing ASSERT call.
   @param[in] p_file_name File name of the failing ASSERT call.
*/

void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
*/
static void timers_init(void) {
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.

   @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
            the device. It also sets the permissions and appearance.
*/
static void gap_params_init(void) {
  uint32_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)DEVICE_NAME,
      strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.

   @details A pointer to this function will be passed to each service which may need to inform the
            application about an error.

   @param[in]   nrf_error   Error code containing information about what went wrong.
*/
static void nrf_qwr_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Nordic UART Service.

   @details This function will process the data received from the Nordic UART BLE Service and send
            it to the UART module.

   @param[in] p_evt       Nordic UART Service event.
*/
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t *p_evt) {

  if (p_evt->type == BLE_NUS_EVT_RX_DATA) {

    NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
    NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

    for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++) {

      if (p_evt->params.rx_data.p_data[i] == '1') {
        nrf_gpio_pin_write(LED_PIN, 0);
        unlock_command = 1;
        NRF_LOG_INFO("Received UNLOCK command");
      }
      if (p_evt->params.rx_data.p_data[i] == '0') {
        nrf_gpio_pin_write(LED_PIN, 1);
      }
    }
    //        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
    //        {
    //            while (app_uart_put('\n') == NRF_ERROR_BUSY);
    //        }
  }
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
*/
static void services_init(void) {
  uint32_t err_code;
  ble_nus_init_t nus_init;
  nrf_ble_qwr_init_t qwr_init = {0};

  // Initialize Queued Write Module.
  qwr_init.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

  // Initialize NUS.
  memset(&nus_init, 0, sizeof(nus_init));

  nus_init.data_handler = nus_data_handler;

  err_code = ble_nus_init(&m_nus, &nus_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.

   @details This function will be called for all events in the Connection Parameters Module
            which are passed to the application.

   @note All this function does is to disconnect. This could have been done by simply setting
         the disconnect_on_fail config parameter, but instead we use the event handler
         mechanism to demonstrate its use.

   @param[in] p_evt  Event received from the Connection Parameters Module.
*/
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
  uint32_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling errors from the Connection Parameters module.

   @param[in] nrf_error  Error code containing information about what went wrong.
*/
static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
*/
static void conn_params_init(void) {
  uint32_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.

   @note This function will not return.
*/
static void sleep_mode_enter(void) {
  //    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  //    APP_ERROR_CHECK(err_code);

  // Prepare wakeup buttons.
  uint32_t err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.

   @details This function will be called for advertising events which are passed to the application.

   @param[in] ble_adv_evt  Advertising event.
*/
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {

  switch (ble_adv_evt) {
  case BLE_ADV_EVT_FAST:
    //uint32_t err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    //APP_ERROR_CHECK(err_code);
    break;
  case BLE_ADV_EVT_IDLE:
    sleep_mode_enter();
    break;
  default:
    break;
  }
}

/**@brief Function for handling BLE events.

   @param[in]   p_ble_evt   Bluetooth stack event.
   @param[in]   p_context   Unused.
*/
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  uint32_t err_code;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    NRF_LOG_INFO("Connected");
    ble_status = 1;
    nrf_gpio_pin_write(LED_PIN, 0);
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    NRF_LOG_INFO("Disconnected");
    ble_status = 0;
    nrf_gpio_pin_write(LED_PIN, 1);
    // LED indication will be changed when advertising starts.
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  } break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    // Pairing not supported
    err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_SYS_ATTR_MISSING:
    // No system attributes have been stored.
    err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  default:
    // No implementation needed.
    break;
  }
}

/**@brief Function for the SoftDevice initialization.

   @details This function initializes the SoftDevice and the BLE event interrupt.
*/
static void ble_stack_init(void) {
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
    m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
  }
  NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
      p_gatt->att_mtu_desired_central,
      p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void) {
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.

   @param[in]   event   Event generated by button press.
*/
void bsp_event_handler(bsp_event_t event) {
  uint32_t err_code;
  switch (event) {
  case BSP_EVENT_SLEEP:
    sleep_mode_enter();
    break;

  case BSP_EVENT_DISCONNECT:
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_ERROR_INVALID_STATE) {
      APP_ERROR_CHECK(err_code);
    }
    break;

  case BSP_EVENT_WHITELIST_OFF:
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
      err_code = ble_advertising_restart_without_whitelist(&m_advertising);
      if (err_code != NRF_ERROR_INVALID_STATE) {
        APP_ERROR_CHECK(err_code);
      }
    }
    break;

  default:
    break;
  }
}

///**@brief   Function for handling app_uart events.
// *
// * @details This function will receive a single character from the app_uart module and append it to
// *          a string. The string will be be sent over BLE when the last character received was a
// *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
// */
///**@snippet [Handling the data received over UART] */
extern void uart_event_handle(app_uart_evt_t *p_event);
//void uart_event_handle(app_uart_evt_t * p_event)
//{
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//    static uint8_t index = 0;
//    uint32_t       err_code;

//    switch (p_event->evt_type)
//    {
//        case APP_UART_DATA_READY:
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;

//            if ((data_array[index - 1] == '\n') ||
//                (data_array[index - 1] == '\r') ||
//                (index >= m_ble_nus_max_data_len))
//            {
//                if (index > 1)
//                {

//                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
//                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

//                    do
//                    {
//                        uint16_t length = (uint16_t)index;
//                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
//                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
//                            (err_code != NRF_ERROR_RESOURCES) &&
//                            (err_code != NRF_ERROR_NOT_FOUND))
//                        {
//                            APP_ERROR_CHECK(err_code);
//                        }
//                    } while (err_code == NRF_ERROR_RESOURCES);
//                }

//                index = 0;
//            }
//            break;

//        case APP_UART_COMMUNICATION_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_communication);
//            break;

//        case APP_UART_FIFO_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_code);
//            break;

//        default:
//            break;
//    }
//}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
*/
/**@snippet [UART Initialization] */
static void uart_init(void) {
  //    uint32_t                     err_code;
  //    app_uart_comm_params_t const comm_params =
  //    {
  //        .rx_pin_no    = GSM_RXD_PIN, //RX_PIN_NUMBER,
  //        .tx_pin_no    = GSM_TXD_PIN, //TX_PIN_NUMBER,
  //        .rts_pin_no   = GSM_RTS_PIN, //RTS_PIN_NUMBER,
  //        .cts_pin_no   = GSM_CTS_PIN, //CTS_PIN_NUMBER,
  //        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
  //        .use_parity   = false,
  //#if defined (UART_PRESENT)
  //        .baud_rate    = NRF_UART_BAUDRATE_57600
  //#else
  //        .baud_rate    = NRF_UARTE_BAUDRATE_115200
  //#endif
  //    };

  //    APP_UART_FIFO_INIT(&comm_params,
  //                       UART_RX_BUF_SIZE,
  //                       UART_TX_BUF_SIZE,
  //                       uart_event_handle,
  //                       APP_IRQ_PRIORITY_LOWEST,
  //                       err_code);
  //    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

/**@brief Function for initializing the Advertising functionality.
*/
static void advertising_init(void) {
  uint32_t err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = false;
  init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing buttons and leds.

   @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
*/
static void buttons_leds_init(bool *p_erase_bonds) {
  bsp_event_t startup_event;

  uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for initializing the nrf log module.
*/
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
*/
static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).

   @details If there is no pending log operation, then sleep until next the next event occurs.
*/
static void idle_state_handle(void) {
  UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
  nrf_pwr_mgmt_run();
}

/**@brief Function for starting advertising.
*/
static void advertising_start(void) {
  uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);
}

void vApplicationIdleHook(void) {
  //while (1)
  {
    idle_state_handle();
  }
}

uint32_t gps_data_get_bus(uint8_t *data, uint32_t len) {
  uint32_t ret = 0;
  uint8_t i = 0;
  char *position;
  if (data == NULL || len < 0) {
    return 1;
  }

  gps_data_get(data, len);

  if (NULL != strstr((char *)data, "+CGNSINF: 1,1")) {

    position = strstr((char *)data, ".000");

    memcpy(latitude, position + 5, 7);
    memcpy(longitude, position + 15, 7);

    ret = 1;
  } else {
    ret = 0;
  }

  gps_parse(data);

  return ret;
}

void Gsm_wait_response(uint8_t *rsp, uint32_t len, uint32_t timeout, GSM_RECEIVE_TYPE type) {
  if (rsp == NULL || len < 0) {
    return;
  }
  g_type = type;
  memset(GSM_RSP, 0, 1600);
  Gsm_WaitRspOK(GSM_RSP, timeout, true);
  NRF_LOG_INFO("%s\r\n", GSM_RSP);
}

uint8_t TCPopenNetwork(const char *host, int port) {
  char str_tmp[64] = {0};
  uint8_t len[20] = {0};

  sprintf((char *)len, "AT+CIPSTART=\"TCP\",\"%s\",%d", host, port);
  Gsm_print(len);
  NRF_LOG("TCPopenNETWORK");

  return Gsm_WaitRspOK(str_tmp, 1000, true);
}

//uint8_t   mqttPublish(char* iot_topic, String iot_data)
//{
//              Gsm_print("AT+CIPSEND");
//              vTaskDelay(200);
//        char     MQTTdata[2]={0x00,0x04};
//        char     MQTTbuff[50]={0};
//        MQTTbuff[0] = 0x32;
//        send_buff(MQTTbuff,1);
//        MQTTbuff[0] = strlen(iot_topic)+iot_data.length()+4;
//        send_buff(MQTTbuff,2);
//        MQTTbuff[0] = strlen(iot_topic);
//        send_buff(MQTTbuff,1);
//        send_cmd(iot_topic);
//        send_buff(MQTTdata,2);
//        iot_data.toCharArray(MQTTbuff,iot_data.length());
//        send_String(iot_data);
//        if(check_send_cmd(" ","CLOSED")){
//            return false;
//        }else{
//            return true;
//        }

//}

void gsm_task(void *pvParameter) {
  
  uint8_t rsp[500] = {0};
  uint8_t device_key[9] = {0};
  uint8_t test_data[256] = {0};
  uint8_t gps_data[128] = {0};
  uint8_t len[20] = {0};
  uint8_t sensor_len = 0;
  double temp = 0;
  double humidity = 0;
  double pressure = 0;
  int x = 0;
  int y = 0;
  int z = 0;
  float magnetic_x = 0;
  float magnetic_y = 0;
  float magnetic_z = 0;
  float light = 0;
  double lat = 0;
  double lon = 0;
  uint8_t i = 0;
  uint8_t j = 0;

  NRF_LOG_INFO("[gsp task] Gsm_Inited.");
  while (1) {
    // while(ble_status == 1)
    //       {
    NRF_LOG_INFO("doing gsm_task");
    vTaskDelay(100);

    char str_tmp[64];
    memset(str_tmp, 0, 64);
    Gsm_print("AT+CGNSPWR=1");
    Gsm_WaitRspOK(str_tmp, 1000, true);
    vTaskDelay(100);

    memset(str_tmp, 0, 64);
    Gsm_print("AT+CSTT=\"ctnb\"");
    Gsm_WaitRspOK(str_tmp, 1000, true);
    vTaskDelay(100);

    memset(str_tmp, 0, 64);
    Gsm_print("AT+CIICR");
    Gsm_WaitRspOK(str_tmp, 1000, true);
    vTaskDelay(100);

    Gsm_print("AT+CIICR");
    vTaskDelay(100);

    //        if(TCPopenNetwork(serverIP,1883)){                  //Connect to server
    //
    //            vTaskDelay(200);
    //
    ////                        if(mqttPublish(IOT_TOPIC,sendData)){                      //Close connection
    ////
    ////                            vTaskDelay(200);
    ////                          closeNetwork();

    ////                        }
    //
    //        }

    gps_data_get_bus(gps_data, 128);
    vTaskDelay(100);
    SEGGER_RTT_printf(0, "GPS = %s\r\n", gps_data);
    //              memset(test_data,0,256);
    //              sensor_len = sprintf((char *)test_data,"Acc:%d,%d,%d;Tem:%d;Hum:%d;Pre:%d;Mag:%d,%d,%d;Lig:%d;Gps:%s;",x,y,z,(int)temp,(int)humidity,(int)pressure,(int)magnetic_x,(int)magnetic_y,(int)magnetic_z,(int)light,gps_data);
    //              memset(cmd,0,128);
    //              hologram_cmd_packet(device_key,test_data);
    //              NRF_LOG_INFO("device_key = %s\r\n",device_key);
    //              NRF_LOG_INFO("test_data = %s\r\n",test_data);
    //              NRF_LOG_INFO("test_data len = %d\r\n",sensor_len);
    //              NRF_LOG_INFO("send packet = %s\r\n",cmd);
    //              Gsm_print("AT+QIOPEN=1,0,\"TCP\",\"cloudsocket.hologram.io\",9999,0,1");
    //              memset(rsp, 0, 500);
    //              Gsm_wait_response(rsp, 500, 500 * 60, GSM_TYPE_CHAR);
    //              memset(rsp, 0, 500);
    //              Gsm_wait_response(rsp, 500, 500 * 20, GSM_TYPE_CHAR);
    //              vTaskDelay(500);
    //              memset(len,0,20);
    //              sprintf(len,"AT+QISEND=0,%d",36+sensor_len+1);
    //              Gsm_print(len);
    //              vTaskDelay(500);
    //              Gsm_print(cmd);
    //              memset(rsp, 0, 500);
    //              Gsm_wait_response(rsp, 500, 500 * 60, GSM_TYPE_CHAR);
    //              memset(rsp, 0, 500);
    //              Gsm_wait_response(rsp, 500, 500 * 80, GSM_TYPE_CHAR);
    //              Gsm_print("AT+QICLOSE=0,30000");
    //              memset(rsp, 0, 500);
    //              Gsm_wait_response(rsp, 500, 500 * 60, GSM_TYPE_CHAR);

    //              Gsm_print(cmd);
    //              memset(rsp, 0, 500);
    //              Gsm_wait_response(rsp, 500, 500 * 60, GSM_TYPE_CHAR);

    //          memset(cmd,0,128);
    //          memset(device_key,0,9);
    //          memset(test_data,0,256);
    //          memset(gps_data,0,256);
    //          memset(len,0,20);
    //          sensor_len = 0;
    //          i = 0;
    //          j = 0;
    //          vTaskDelay(10000);
    //      }
    //      //Gsm_PowerDown();

    //idle_state_handle(); // sleep
  }
}

static void main_task_function(void *pvParameter) {
  nrf_gpio_cfg_output(LED_PIN);
  nrf_gpio_pin_write(LED_PIN, 1);

  UNUSED_PARAMETER(pvParameter);
  while (1) {
    NRF_LOG_FLUSH();
//    NRF_LOG_INFO("MAIN TASK");
//    NRF_LOG_FLUSH();
    nrf_gpio_pin_write(LED_PIN, 1);
    NRF_LOG_FLUSH();
    vTaskDelay(100);
//    NRF_LOG_FLUSH();s
    nrf_gpio_pin_write(LED_PIN, 0);
    vTaskDelay(100);
  }

  /* Tasks must be implemented to never return... */
}

static void motor_task_function(void *pvParameter) {
  SEGGER_RTT_printf(0, "MOTOR_TASK START\r\n");
  UNUSED_PARAMETER(pvParameter);
  nrf_gpio_cfg_output(DRV88_IN1_PIN); // ccw
  nrf_gpio_cfg_output(DRV88_IN2_PIN); // cw

  nrf_gpio_cfg_input(LIMIT_SW1_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(LIMIT_SW2_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(LIMIT_SW3_PIN, NRF_GPIO_PIN_NOPULL);

  while (true) {
//    NRF_LOG_INFO("MOTOR_TASK POOLING..");
    
    if (nrf_gpio_pin_read(LIMIT_SW1_PIN) == 1) {
      lock_status = 0;
    } else {
      lock_status = 1;
    }
    SEGGER_RTT_printf(0, "[MOTOR TASK] LOCK STATUS=%d\r\n", lock_status);
    vTaskDelay(500);
    if (unlock_command == 1) {

      SEGGER_RTT_printf(0, "Unlocking \n");

      nrf_gpio_pin_write(DRV88_IN1_PIN, 1);
      nrf_gpio_pin_write(DRV88_IN2_PIN, 1);
      vTaskDelay(20);
      nrf_gpio_pin_write(DRV88_IN2_PIN, 0);
      while (nrf_gpio_pin_read(LIMIT_SW2_PIN) == 0) {
        SEGGER_RTT_printf(0, "LIMIT_SW2 == 0");
        vTaskDelay(100);
      }
      while (nrf_gpio_pin_read(LIMIT_SW3_PIN) == 1) {
         SEGGER_RTT_printf(0, "LIMIT_SW3 == 1");
        vTaskDelay(100);
      }
      nrf_gpio_pin_write(DRV88_IN2_PIN, 1);
      vTaskDelay(200);
      nrf_gpio_pin_write(DRV88_IN1_PIN, 0);

      while (nrf_gpio_pin_read(LIMIT_SW3_PIN) == 0) {
         SEGGER_RTT_printf(0, "LIMIT_SW3_PIN == 0");
        vTaskDelay(100);
      }

      while (nrf_gpio_pin_read(LIMIT_SW3_PIN) == 1) {
        vTaskDelay(1);
         SEGGER_RTT_printf(0, "LIMIT_SW3_PIN == 1");
      }
      nrf_gpio_pin_write(DRV88_IN1_PIN, 1);
      vTaskDelay(50);
      nrf_gpio_pin_write(DRV88_IN1_PIN, 0);
      nrf_gpio_pin_write(DRV88_IN2_PIN, 0);

      unlock_command = 0;
    }

    vTaskDelay(10);
  }

  /* Tasks must be implemented to never return... */
}

/**@brief Application main function.
*/
int main(void) {
  // Initialize.

  //uart_init();

  //rak_uart_init(GSM_USE_UART, GSM_RXD_PIN, GSM_TXD_PIN, UARTE_BAUDRATE_BAUDRATE_Baud57600);

  log_init();
  nrf_drv_clock_init();

  timers_init();

  ble_stack_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();
  advertising_start();

  // Start execution.
  NRF_LOG_INFO("[Booting...]");
  NRF_LOG_INFO("[Booting...]");
  NRF_LOG_INFO("[Booting...]");
  NRF_LOG_INFO("[Booting...]");
  NRF_LOG_INFO("Debug logging for UART over RTT started.");
  
Gsm_Init();

  xTaskCreate(gsm_task, "gsm_task", 2048, NULL, 2, NULL);
  xTaskCreate(motor_task_function, "idle_state", 64, NULL, 2, NULL);
  xTaskCreate(main_task_function, "main_task", 128, NULL, 2, NULL);

  NRF_LOG_INFO("vTaskStartScheduler started.");
  vTaskStartScheduler();

  // Enter main loop.
  for (;;) {
    idle_state_handle();
  }
}

/**
   @}
*/