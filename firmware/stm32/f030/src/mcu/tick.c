/////////////////////////////////////////////////////////////////////////
/// \file tick.c
/// \brief implements mili-second tick counter.
///
/// Author: Ronald Sousa (Opticalworm)
/////////////////////////////////////////////////////////////////////////
#include "MCU/tick.h"

/////////////////////////////////////////////////////////////////////////
/// \brief defines the frequency we want the system tick to trigger.
/// for 1ms = 1/1000hz
/////////////////////////////////////////////////////////////////////////
#define TIMER_FREQUENCY_HZ 1000

/////////////////////////////////////////////////////////////////////////
/// \brief Current system tick count since boot-up.
/// \note tick is expected to overflow.
/////////////////////////////////////////////////////////////////////////
static volatile uint32_t TickCounter;

/////////////////////////////////////////////////////////////////////////
/// \brief setup the ARM M0 tick counter to trigger every 1ms
/////////////////////////////////////////////////////////////////////////
void Tick_init(void)
{
    // configure the system tick so that it trigger every one ms
  SysTick_Config(SystemCoreClock / TIMER_FREQUENCY_HZ);
}

/////////////////////////////////////////////////////////////////////////
/// \brief return the number of mili-seconds since power up.
///
/// \return number of mili-seconds.
///
/// \note the tick counter is expected to overflow and therefore code
/// using the tick value should take this into account.
/////////////////////////////////////////////////////////////////////////
uint32_t Tick_GetMs(void)
{
    return TickCounter;
}

/////////////////////////////////////////////////////////////////////////
/// \brief this is a blocking delay.
///
/// \code
/// #include "common.h"
/// #include "MCU/led.h"
/// #include "MCU/tick.h"
///
/// void main(void) {
///
///     Led_Init();
///     Tick_init();
///
///     for( ;;) {
///       Tick_DelayMs(1000); // delay 1s;
///       Led_Toggle();
///     }
///}
/// \endcode
///
/// \param delayMs how long to delay for.
/////////////////////////////////////////////////////////////////////////
void Tick_DelayMs(uint32_t delayMs)
{
  uint32_t StartTickValue;

  StartTickValue = TickCounter;

  // sit in the while loop until the difference between the start tick
  // and current tick is greater than or equal to
  // the delayMs.
  while((TickCounter - StartTickValue) < delayMs);

}


/////////////////////////////////////////////////////////////////////////
/// \brief Non-blocking delay in ms.
///
/// \code
/// #include "common.h"
/// #include "MCU/led.h"
/// #include "MCU/tick.h"
///
/// void main(void) {
///     TickType Delay;
///     Delay.DelayMs = 1000; //set to 1s
///
///     Led_Init();
///     Tick_init();
///
///     // reset the counter
///     Tick_DelayMs_NonBlocking(TRUE, &Delay);
///
///     for( ;;) {
///
///         if(Tick_DelayMs_NonBlocking(TRUE, &Delay)) {
///             // Delay has been reached
///
///             Tick_DelayMs_NonBlocking(TRUE, &Delay);
///             Led_Toggle();
///             }
///         else {
///             // User code when the code delay hasn't passed
///             }
///
///     }
///}
/// \endcode
///
/// \param reset true = reset timer start value/ false = Check if time has lapsed
/// \param config delay setting
///
/// \return  1 = the desire delay has been reached.
///          0 = not reached the desire delay time.
///         -1 = config point is null
/////////////////////////////////////////////////////////////////////////
int_fast8_t Tick_DelayMs_NonBlocking(uint_fast8_t reset, TickType * config)
{
    uint32_t LapsedTick = 0;

    if( !config )
    {
        return -1; // error can't use a null pointer
    }

    if ( reset )
    {
        config->StartMs = Tick_GetMs();
        return FALSE;
    }

    LapsedTick = (Tick_GetMs() - config->StartMs);

    if( LapsedTick < config->DelayMs )
    {
        return FALSE;
    }

    return TRUE; // time has lapsed

}

/////////////////////////////////////////////////////////////////////////
/// \brief ARM M0 hardware interrupt. This should trigger every
/// 1 ms and update TickCounter.
///
/// \sa TickCounter
/////////////////////////////////////////////////////////////////////////
void SysTick_Handler(void)
{
    TickCounter++;
}
