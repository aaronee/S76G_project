
#ifndef __TIMER_BOARD_H__
#define __TIMER_BOARD_H__

extern TIM_HandleTypeDef htim2;

/*!
 * \brief Timer time variable definition
 */
#ifndef TimerTime_t
typedef uint32_t TimerTime_t;
#endif

/*!
 * \brief Initializes the timer
 *
 * \remark The timer is based on TIM2 with a 10uS time basis
 */
void TimerHwInit( void );

/*!
 * \brief DeInitializes the timer
 */
void TimerHwDeInit( void );

/*!
 * \brief Start the Standard Timer counter
 *
 * \param [IN] rtcCounter Timer duration
 */
void TimerHwStart( uint32_t val );

/*!
 * \brief Perfoms a standard blocking delay in the code execution
 *
 * \param [IN] delay Delay value in ms
 */
void TimerHwDelayMs( uint32_t delay );

/*!
 * \brief Stop the the Standard Timer counter
 */
void TimerHwStop( void );

/*!
 * \brief Return the value of the timer counter
 */
TimerTime_t TimerHwGetTimerValue( void );

/*!
 * \brief Return the value of the current time in us
 */
TimerTime_t TimerHwGetTime( void );

/*!
 * \brief Return the value on the timer Tick counter
 */
TimerTime_t TimerHwGetElapsedTime( void );

/*!
 * \brief Set the ARM core in Wait For Interrupt mode (only working if Debug mode is not used)
 */
void TimerHwEnterLowPowerStopMode( void );

/*!
 * \brief Compute the elapsed time since a fix event in time
 *
 * \param[IN] eventInTime Value in time
 * \retval elapsed Time since the eventInTime
 */
TimerTime_t TimerHwComputeElapsedTime( TimerTime_t eventInTime );

/*!
 * \brief Compute the timeout time of a future event in time
 *
 * \param[IN] futureEventInTime Value in time
 * \retval time Time between now and the futureEventInTime
 */
TimerTime_t TimerHwComputeFutureEventTime( TimerTime_t futureEventInTime );

#endif // __RTC_BOARD_H__
