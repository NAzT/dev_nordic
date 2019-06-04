
#ifndef __PIN_DEFINE_ITRACKER_52832_H__
#define __PIN_DEFINE_ITRACKER_52832_H__

/**
 * Pin definitions
 */
#define RADIO_DIO_0 P7
#define RADIO_DIO_1 P8
#define RADIO_DIO_2 P9
#define RADIO_DIO_3 P10

#define RADIO_NSS P14
#define RADIO_MOSI P13
#define RADIO_MISO P12
#define RADIO_SCK P11

#define RADIO_RESET P6
#define RADIO_TCXO P5
#define RADIO_RF_CTX P23
#define RADIO_RF_CPS P22

#define ASSERT_ERROR 0xA55EA55E

#define USE_FULL_ASSERT
#ifdef USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
#define assert_param(expr) ((expr) ? (void)0U : app_error_handler(ASSERT_ERROR, __LINE__, (const uint8_t *)__FILE__))
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#define BATTERY_ADC_PIN (3)

#define GSM_PWR_ON_PIN (16)
#define GSM_RESET_PIN (18)
#define GSM_PWRKEY_PIN (19)

#define GSM_TXD_PIN (6)
#define GSM_RXD_PIN (8)
#define GSM_RTS_PIN (5)
#define GSM_CTS_PIN (7)

#define LED_PIN (12)

#define LIMIT_SW1_PIN (13)
#define LIMIT_SW2_PIN (14)
#define LIMIT_SW3_PIN (15)

#define DRV88_IN1_PIN (22)
#define DRV88_IN2_PIN (23)


#define GSM_PWR_ON nrf_gpio_pin_write(GSM_PWR_ON_PIN, 1)
#define GSM_PWR_OFF nrf_gpio_pin_write(GSM_PWR_ON_PIN, 0)

#define GSM_PWRKEY_HIGH nrf_gpio_pin_write(GSM_PWRKEY_PIN, 0)
#define GSM_PWRKEY_LOW nrf_gpio_pin_write(GSM_PWRKEY_PIN, 1)

#define GSM_RESET_HIGH nrf_gpio_pin_write(GSM_RESET_PIN, 0)
#define GSM_RESET_LOW nrf_gpio_pin_write(GSM_RESET_PIN, 1)

#define POWER_ON   \
	GSM_PWR_OFF;     \
	vTaskDelay(200); \
	GSM_PWR_ON;      \
	GSM_RESET_HIGH;  \
	vTaskDelay(60);  \
	GSM_PWRKEY_LOW;  \
	vTaskDelay(500); \
	GSM_PWRKEY_HIGH; \
	vTaskDelay(500)


#endif // __PIN_DEFINE_ITRACKER_H__
