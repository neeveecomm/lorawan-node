/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech - STMicroelectronics
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    MCD Application Team (C)( STMicroelectronics International )
 */
#include <math.h>
#include <time.h>
#include "utilities.h"
#include "delay.h"
#include "board.h"
#include "boardapi.h"
#include "timer.h"
#include "systime.h"
#include "gpio.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "fsl_rtc.h"
#include "fsl_pit.h"

/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static volatile bool RtcTimeoutPendingInterrupt = false;
static volatile bool RtcTimeoutPendingPolling = false;

typedef enum AlarmStates_e
{
    ALARM_STOPPED = 0,
    ALARM_RUNNING = !ALARM_STOPPED
} AlarmStates_t;


/*!
 * RTC timer context
 */
typedef struct
{
    uint32_t Time;  // Reference time
    uint32_t Delay; // Reference Timeout duration
    uint32_t AlarmState;
}RtcTimerContext_t;

/*!
 * Used to store the Seconds and SubSeconds.
 *
 * WARNING: Temporary fix fix. Should use MCU NVM internal
 *          registers
 */
uint32_t RtcBkupRegisters[] = { 0, 0 };

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

// MCU Wake Up Time
#define MIN_ALARM_DELAY                             100 // in ticks

// sub-second number of bits
#define N_PREDIV_S                                  10

// Synchronous prediv
#define PREDIV_S                                    ( ( 1 << N_PREDIV_S ) - 1 )

// Asynchronous prediv
#define PREDIV_A                                    ( 1 << ( 15 - N_PREDIV_S ) ) - 1

// Sub-second mask definition
//#define ALARM_SUBSECOND_MASK                        ( N_PREDIV_S << RTC_ALRMASSR_MASKSS_Pos )

// RTC Time base in us
#define USEC_NUMBER                                 1000000
#define MSEC_NUMBER                                 ( USEC_NUMBER / 1000 )

#define COMMON_FACTOR                               3
#define CONV_NUMER                                  ( MSEC_NUMBER >> COMMON_FACTOR )
#define CONV_DENOM                                  ( 1 << ( N_PREDIV_S - COMMON_FACTOR ) )

/*!
 * \brief Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR                           ( ( uint32_t )  366U )
#define DAYS_IN_YEAR                                ( ( uint32_t )  365U )
#define SECONDS_IN_1DAY                             ( ( uint32_t )86400U )
#define SECONDS_IN_1HOUR                            ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE                          ( ( uint32_t )   60U )
#define MINUTES_IN_1HOUR                            ( ( uint32_t )   60U )
#define HOURS_IN_1DAY                               ( ( uint32_t )   24U )

/*!
 * \brief Correction factors
 */
#define  DAYS_IN_MONTH_CORRECTION_NORM              ( ( uint32_t )0x99AAA0 )
#define  DAYS_IN_MONTH_CORRECTION_LEAP              ( ( uint32_t )0x445550 )

/*!
 * \brief Calculates ceiling( X / N )
 */
#define DIVC( X, N )                                ( ( ( X ) + ( N ) -1 ) / ( N ) )

/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitialized = false;

/*!
 * \brief Indicates if the RTC Wake Up Time is calibrated or not
 */
static bool McuWakeUpTimeInitialized = false;

/*!
 * \brief Compensates MCU wakeup time
 */
static int16_t McuWakeUpTimeCal = 0;

/*!
 * Number of days in each month on a normal year
 */
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Number of days in each month on a leap year
 */
static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

uint32_t RTC_ConvertDatetimeToSeconds(const rtc_datetime_t *datetime);

#define CALC_COUNT(us, clockFreqInHz) (uint64_t)(((uint64_t)us * clockFreqInHz) / 10000U)
#define PIT_IRQ_ID PIT_IRQn
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

void RtcInit( void )
{
	rtc_config_t rtcCfg = {0};
	rtc_datetime_t rtcDate = {0};

	if(RtcInitialized == false) {
	    /* Structure of initialize PIT */
	    pit_config_t pitConfig;

	    /*
	     * pitConfig.enableRunInDebug = false;
	     */
	    PIT_GetDefaultConfig(&pitConfig);

	    /* Init pit module */
	    PIT_Init(PIT, &pitConfig);

	    /* Set timer period for channel 0 */
	    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(100U, PIT_SOURCE_CLOCK));

	    /* Enable timer interrupts for channel 0 */
	    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	    /* Enable at the NVIC */
	    EnableIRQ(PIT_IRQ_ID);

	    /* Start channel 0 */
	    PIT_StartTimer(PIT, kPIT_Chnl_0);

        RtcTimerContext.AlarmState = ALARM_STOPPED;
		RtcSetTimerContext( );
		RtcInitialized = true;
	}
}

static uint32_t rtcTimer;
static uint8_t startAlarm;
static uint32_t pitTimerCnt, alarmTimeout;

/*!
 * \brief Sets the RTC timer reference, sets also the RTC_DateStruct and RTC_TimeStruct
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcSetTimerContext( void )
{
    RtcTimerContext.Time = ( uint32_t )pitTimerCnt;
    return ( uint32_t )RtcTimerContext.Time;
}

/*!
 * \brief Gets the RTC timer reference
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcGetTimerContext( void )
{
    return RtcTimerContext.Time;
}

/*!
 * \brief returns the wake up time in ticks
 *
 * \retval wake up time in ticks
 */
uint32_t RtcGetMinimumTimeout( void )
{
    return( MIN_ALARM_DELAY );
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( uint32_t milliseconds )
{
    return ( uint32_t )milliseconds * 10;
}

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
uint32_t RtcTick2Ms( uint32_t tick )
{
	return tick / 10;
}

/*!
 * \brief a delay of delay ms by polling RTC
 *
 * \param[IN] delay in ms
 */
void RtcDelayMs( uint32_t delay )
{
    uint64_t delayTicks = 0;
    uint64_t refTicks = RtcGetTimerValue( );

    delayTicks = RtcMs2Tick( delay );

    // Wait delay ms
    while( ( ( RtcGetTimerValue( ) - refTicks ) ) < delayTicks )
    {
        __NOP( );
    }
}

/*void DelayMsMcu( uint32_t ms )
{
	RtcDelayMs(ms);
}*/

/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this function) + timeout
 *
 * \param timeout Duration of the Timer ticks
 */
void RtcSetAlarm( uint32_t timeout )
{
    RtcStartAlarm( timeout );
}

void RtcStopAlarm( void )
{
	RtcTimerContext.AlarmState = ALARM_STOPPED;
}

void RtcStartAlarm( uint32_t timeout )
{
    CRITICAL_SECTION_BEGIN( );

    RtcStopAlarm( );

    RtcTimerContext.Delay = timeout;

    alarmTimeout = (RtcTimerContext.Time + RtcTimerContext.Delay);

    RtcTimeoutPendingInterrupt = true;
    RtcTimeoutPendingPolling = false;

    RtcTimerContext.AlarmState = ALARM_RUNNING;
    if( pitTimerCnt > (RtcTimerContext.Time + RtcTimerContext.Delay))
    {
        // If timer already passed
        if( RtcTimeoutPendingInterrupt == true )
        {
            // And interrupt not handled, mark as polling
            RtcTimeoutPendingPolling = true;
            RtcTimeoutPendingInterrupt = false;
        }
    }
    CRITICAL_SECTION_END( );
}

uint32_t RtcGetTimerValue( void )
{
	return pitTimerCnt;
}

uint32_t RtcGetTimerElapsedTime( void )
{
	return ( uint32_t)( pitTimerCnt - RtcTimerContext.Time );;
}

void RtcSetMcuWakeUpTime( void )
{
}

int16_t RtcGetMcuWakeUpTime( void )
{
    return McuWakeUpTimeCal;
}

void SX1276OnDio1Irq( void* context );
void SX1276OnDio2Irq( void* context );

void PIT_IRQHandler(void)
{
	pitTimerCnt++;

	if((alarmTimeout != 0) && (alarmTimeout < pitTimerCnt)) {
		TimerIrqHandler();
	    RtcTimerContext.AlarmState = ALARM_STOPPED;
	    // Because of one shot the task will be removed after the callback
	    RtcTimeoutPendingInterrupt = false;
	    //alarmTimeout = 0;
	}
#if 1
	if((pitTimerCnt % 100) == 0) {
		if(GPIO_ReadPinInput(GPIOC, 8) == 1) {
			SX1276OnDio1Irq(NULL);
		}

		if(GPIO_ReadPinInput(GPIOC, 9) == 1) {
			SX1276OnDio2Irq(NULL);
		}
	}
#endif
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
}

#if 0
static uint64_t RtcGetCalendarValue( RTC_DateTypeDef* date, RTC_TimeTypeDef* time )
{
    uint64_t calendarValue = 0;
    uint32_t firstRead;
    uint32_t correction;
    uint32_t seconds;

    // Get Time and Date
    HAL_RTC_GetTime( &RtcHandle, time, RTC_FORMAT_BIN );

    // Make sure it is correct due to asynchronus nature of RTC
    do
    {
        firstRead = time->SubSeconds;
        HAL_RTC_GetDate( &RtcHandle, date, RTC_FORMAT_BIN );
        HAL_RTC_GetTime( &RtcHandle, time, RTC_FORMAT_BIN );
    }while( firstRead != time->SubSeconds );

    // Calculte amount of elapsed days since 01/01/2000
    seconds = DIVC( ( DAYS_IN_YEAR * 3 + DAYS_IN_LEAP_YEAR ) * date->Year , 4 );

    correction = ( ( date->Year % 4 ) == 0 ) ? DAYS_IN_MONTH_CORRECTION_LEAP : DAYS_IN_MONTH_CORRECTION_NORM;

    seconds += ( DIVC( ( date->Month-1 ) * ( 30 + 31 ), 2 ) - ( ( ( correction >> ( ( date->Month - 1 ) * 2 ) ) & 0x03 ) ) );

    seconds += ( date->Date -1 );

    // Convert from days to seconds
    seconds *= SECONDS_IN_1DAY;

    seconds += ( ( uint32_t )time->Seconds + 
                 ( ( uint32_t )time->Minutes * SECONDS_IN_1MINUTE ) +
                 ( ( uint32_t )time->Hours * SECONDS_IN_1HOUR ) ) ;

    calendarValue = ( ( ( uint64_t )seconds ) << N_PREDIV_S ) + ( PREDIV_S - time->SubSeconds );

    return( calendarValue );
}
#endif

uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
    uint32_t seconds = ( uint32_t )pitTimerCnt / 10000;

    *milliseconds = RtcTick2Ms(pitTimerCnt);

    return seconds;
}

void RtcProcess( void )
{
    CRITICAL_SECTION_BEGIN( );

    if( (  RtcTimerContext.AlarmState == ALARM_RUNNING ) && ( RtcTimeoutPendingPolling == true ) )
    {
        if( RtcGetTimerElapsedTime( ) >= RtcTimerContext.Delay )
        {
            RtcTimerContext.AlarmState = ALARM_STOPPED;

            // Because of one shot the task will be removed after the callback
            RtcTimeoutPendingPolling = false;
            // NOTE: The handler should take less then 1 ms otherwise the clock shifts
            TimerIrqHandler();
        }
    }
    CRITICAL_SECTION_END( );
}

TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
	return period;
}
