#ifndef _BOARD_H
#define _BOARD_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_drv_rtc.h"
#include "nrf_delay.h"
#include "app_uart.h"
#include "utilities.h"
#include "gps.h"
#include "SEGGER_RTT.h"
#include "hal_spi.h"
#include "hal_gpio.h"
#include "hal_uart.h"
#include "pin_define.h"
#include "nrf_drv_saadc.h"
#include "bg96.h"



typedef enum GSM_RECEIVE_TYPE
{
	GSM_TYPE_CHAR,
	GSM_TYPE_FILE,
}GSM_RECEIVE_TYPE;


typedef struct {
        uint8_t sof;
        uint8_t dev_eui[8];
        uint8_t app_eui[8];
        uint8_t app_key[16];
        uint32_t dev_addr;
        uint8_t nwkskey[16];
        uint8_t appskey[16];
} lora_cfg_t;


#define LORA_CONFIG_MAGIC  	0xAB


/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS		1
#endif

#ifndef FAIL
#define FAIL		0
#endif



/*
*********************************************************************************************************
*                                             LOG 
*********************************************************************************************************
*/
#define     LOG_NONE     (0x00UL)
#define     LOG_ERROR    (0x01UL)
#define     LOG_WARN     (0x02UL)
#define     LOG_INFO     (0x04UL)
#define     LOG_DEBUG    (0x08UL)
#define     LOG_TRACE    (0x10UL)

#define     G_DEBUG  (LOG_NONE | LOG_ERROR | LOG_WARN | LOG_INFO | LOG_DEBUG )     
//#define     G_DEBUG  (LOG_NONE)     
#define     LOG_LEVEL_CHECK(level)      (G_DEBUG & level)


//extern OS_MUTEX   pfMutex;

//static inline void p_lock_mutex(OS_MUTEX *mutex)
//{
//  OS_ERR oserr;  
//  OSMutexPend(mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &oserr);
//}


//static inline void p_unlock_mutex(OS_MUTEX *mutex)
//{
//  OS_ERR oserr;  
//  OSMutexPost(mutex, OS_OPT_POST_NONE, &oserr);
//}

static inline char* log_level_str(uint8_t level)
{
	  char* string;
    switch(level) 
		{
			case LOG_ERROR:
				string ="ERROR";
			 break;
			case LOG_WARN:
				string ="WARN";
			 break;			
			case LOG_INFO:
				string ="INFO";
			 break;		
			case LOG_DEBUG:
				string ="DEBUG";
			 break;			
			case LOG_TRACE:
				string ="TRACE";
			 break;		
      default:
         break;				
		}
    return string;
}


#ifdef DEBUG
static const char* clean_filename(const char* path)
{
  const char* filename = path + strlen(path); 
  while(filename > path)
  {
    if(*filename == '/' || *filename == '\\')
    {
      return filename + 1;
    }
    filename--;
  }
  return path;
}
#endif

#ifdef DEBUG
#define DPRINTF(level, fmt, args...)\
	NRF_LOG_INFO(fmt, ##args)
#else
#define DPRINTF(fmt, args...)
#endif
	
/*!
 * Possible power sources
 */
enum BoardPowerSources
{
	USB_POWER = 0,
	BATTERY_POWER,
};

/*!
 * \brief Measure the Battery voltage
 *
 * \retval value	battery voltage in volts
 */
uint16_t BoardGetBatteryVoltage( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value	battery level [	0: USB,
 *								 1: Min level,
 *								 x: level
 *								254: fully charged,
 *								255: Error]
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Processing board events
 */
void BoardProcess( void );


/*!
 * Select the edge of the PPS signal which is used to start the
 * reception of data on the UART. Depending of the GPS, the PPS
 * signal may go low or high to indicate the presence of data
 */
typedef enum PpsTrigger_s
{
    PpsTriggerIsRising = 0,
    PpsTriggerIsFalling,
}PpsTrigger_t;

/*!
 * \brief Low level handling of the PPS signal from the GPS receiver
 */
void GpsMcuOnPpsSignal( void );

/*!
 * \brief Invert the IRQ trigger edge on the PPS signal
 */
void GpsMcuInvertPpsTrigger( void );

/*!
 * \brief Low level Initialisation of the UART and IRQ for the GPS
 */
void GpsMcuInit( void );

/*!
 * \brief Switch ON the GPS
 */
void GpsMcuStart( void );

/*!
 * \brief Switch OFF the GPS
 */
void GpsMcuStop( void );

/*!
 * Updates the GPS status
 */
void GpsMcuProcess( void );

/*!
 * \brief IRQ handler for the UART receiver
 */
void GpsMcuIrqNotify( void );

/*!
 * \brief Timer object description
 */
//typedef	struct {
//	app_timer_id_t	    id;
//	app_timer_t		timer;
//	uint32_t		timeout;
//} TimerEvent_t;
typedef	struct {
	TimerHandle_t 	  id;
	uint32_t		timeout;
}TimerEvent_t;

typedef uint32_t TimerTime_t;

/*!
 * \brief Initializes the timer object
 *
 * \remark TimerSetValue function must be called before starting the timer.
 *         this function initializes timestamp and reload value at 0.
 *
 * \param [IN] obj          Structure containing the timer object parameters
 * \param [IN] callback     Function callback called at the end of the timeout
 */
void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) );

/*!
 * \brief Starts and adds the timer object to the list of timer events
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerStart( TimerEvent_t *obj );

/*!
 * \brief Stops and removes the timer object from the list of timer events
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerStop( TimerEvent_t *obj );

/*!
 * \brief Set timer new timeout value
 *
 * \param [IN] obj   Structure containing the timer object parameters
 * \param [IN] value New timer timeout value
 */
void TimerSetValue( TimerEvent_t *obj, uint32_t value );

/*!
 * \brief Read the current time
 *
 * \retval time returns current time
 */
TimerTime_t TimerGetCurrentTime( void );

/*!
 * \brief Return the Time elapsed since a fix moment in Time
 *
 * \param [IN] savedTime    fix moment in Time
 * \retval time             returns elapsed time
*/
TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime );

/*!
 * \brief Manages the entry into ARM cortex deep-sleep mode
 */
void TimerLowPowerHandler( void );

/*!
 * \brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * \param [IN] period Time period to compensate
 * \param [IN] temperature Current temperature
 *
 * \retval Compensated time period
 */
TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature );

void GpioDeinit( Gpio_t *obj );
void delay_ms(uint32_t ms);


//void lora_init();
void saadc_init(void);
void power_manage(void);
#endif
