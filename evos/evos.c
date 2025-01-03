/**
 * @file    evos.c
 * @brief   EVOS (Event-based Operating System) core implementation file
 *
 * This file contains the core functions for EVOS: event registration,
 * scheduling, and handling. It offers a simple event-based concurrency
 * mechanism, allowing multiple software components to schedule tasks
 * (events) at various times or intervals without requiring a dedicated
 * RTOS thread for each task. The event loop is cooperatively scheduled:
 * tasks run to completion and no preemption occurs inside an event’s
 * handler, simplifying thread-safety.
 */

#include <string.h>
#include <stdio.h>

#include "evos/evos.h"


/*******************************************************************************
 *    Type definitions
 ******************************************************************************/

/**
 * @brief Represents a single EVOS event.
 *
 * An EVOS event consists of:
 * - A flag indicating whether it is active (`on`).
 * - The absolute time at which it should fire (`time`).
 * - A reload time (`reloadTime`), if the event is periodic.
 * - A parameter (`param`) passed to the event’s callback function.
 * - A function pointer (`func`) to the event’s callback handler.
 * - A descriptive name (`name`) for tracing/debugging.
 */
typedef struct {
  bool                on;            /**< true if the event is scheduled to run */
  PlfTime_t           time;          /**< Time at which the event will occur */
  PlfTime_t           reloadTime;    /**< Interval for recurring events */
  EvosEventParam_t    param;         /**< Parameter associated with the event */
  EvosEventFunc_t     func;          /**< Pointer to event handler function */
  const char          *name;         /**< Event name (used for tracing) */
} EvosEvent_t;


/*******************************************************************************
 *    Local variables
 ******************************************************************************/

/** 
 * @brief Global pool of events. 
 *
 * EVOS maintains a static pool of events. Each slot may contain an event
 * or be unused. This approach simplifies memory management and improves
 * predictability, essential for embedded and real-time systems.
 */
static EvosEvent_t      evosEventPool[EVOS_POOL_SIZE];

/** 
 * @brief Holds the current system time (in ticks) as obtained from the HAL.
 *
 * The EVOS scheduler compares the scheduled time of each event
 * to the current time to determine if an event should fire.
 */
PlfTime_t               evosCurrentTime;

/**
 * @brief Holds the scheduled time of the currently executing event.
 *
 * This variable is set right before the event is executed and can be
 * used for logging or diagnostic purposes.
 */
PlfTime_t               evosScheduledTime;

/**
 * @brief Tracks the index of the event currently being processed.
 *
 * Used internally during scheduling to identify and manipulate
 * the current event (especially relevant when deleting or re-registering
 * an event during its own handler).
 */
static EvosEventHandle_t currentEvent;


/******************************************************************************/ 
#ifdef CLI_ENABLE
/******************************************************************************/ 

/**
 * @brief Provides logging capabilities for EVOS events when CLI is enabled.
 *
 * Each event gets a log entry to track:
 * - The number of times (`count`) it has fired.
 * - The total time spent inside its handler (`time`).
 */
typedef struct {
  uint32_t              count;  /**< Number of times an event has executed */
  PlfTime_t             time;   /**< Cumulative execution time of the event's handler */
} EvosLog_t;

/**
 * @brief Event log array for collecting statistics about each event.
 *
 * Useful for performance analysis and debugging.
 */
static EvosLog_t        evosEventLog[EVOS_POOL_SIZE];

static PlfTime_t        measureStartTime; 
static PlfTime_t        startTime, stopTime;

#endif /* CLI_ENABLE */
/******************************************************************************/


/*******************************************************************************
 * @brief Registers a new event in the EVOS system.
 *
 * @param[in] pFunc  Pointer to the function to execute when the event triggers.
 * @param[in] pName  String name for the event (used for tracing and debugging).
 *
 * @return Handle (index) to the registered event within the pool.
 *
 * @details 
 * - Scans the event pool to find an unused slot (where `func` is NULL).
 * - Assigns the provided handler and name.
 * - If the pool is full (no available slot), the system halts execution (`for(;;);`).
 *
 * @note 
 *  In an event-based system like EVOS, the ability to register events dynamically
 *  allows for flexible program flow. Components can register recurring or
 *  one-shot tasks without blocking other parts of the system.
 ******************************************************************************/
EvosEventHandle_t EvosEventRegister(
  const EvosEventFunc_t  pFunc,
  const char * const pName)
{
  EvosEventHandle_t i;
  i=0;
  while ((i<EVOS_POOL_SIZE) && (evosEventPool[i].func != NULL)) {
    i++;
  }
  if (i==EVOS_POOL_SIZE) {
    /* No available slot. Halt. */
    for (;;) ;
  } else {
    /* Initialize the event slot with the provided handler and name. */
    evosEventPool[i].func = pFunc;
    evosEventPool[i].name = pName;
  }
  return i;
}


/*******************************************************************************
 * @brief Clears (disables) an event from the EVOS system.
 *
 * @param[in] event  Handle to the event that should be cleared.
 *
 * @details 
 * - Disables the specified event by setting `on` to false.
 * - Resets its timing information.
 * - This function is atomic: interrupts are disabled before manipulation
 *   of shared resources.
 *
 * @note 
 *  Clearing an event prevents its handler from being called until it is
 *  scheduled again.
 ******************************************************************************/
void EvosEventClear(
  const EvosEventHandle_t  event)
{
  if (event < EVOS_POOL_SIZE) {
    HalIrqStat_t istat;
    HalInterruptDisable(&istat);
    evosEventPool[event].on = false;
    evosEventPool[event].time  = 0;
    evosEventPool[event].reloadTime = 0;
    HalInterruptRestore(istat);
  }
}


/*******************************************************************************
 * @brief Schedules a recurring event with a specific time interval.
 *
 * @param[in] event       Handle to the event.
 * @param[in] firstTime   Absolute time for the first occurrence of the event.
 * @param[in] reloadTime  Time interval (in ticks) for subsequent occurrences.
 * @param[in] param       Parameter to pass to the event handler.
 *
 * @details 
 * - Sets up the event so that it triggers first at `firstTime`.
 * - After it fires, it automatically reloads and will trigger again
 *   after `reloadTime` ticks, and so on.
 * - This function is atomic to ensure thread safety.
 *
 * @note 
 *  Recurring events are useful for tasks that must execute periodically 
 *  (e.g., sensor polling, regular status updates).
 ******************************************************************************/
void EvosEventSetAndReload(
  const EvosEventHandle_t event,
  const PlfTime_t firstTime, 
  const PlfTime_t reloadTime, 
  EvosEventParam_t param)
{
  if (event < EVOS_POOL_SIZE) {
    HalIrqStat_t istat;
    HalInterruptDisable(&istat);
    evosEventPool[event].time       = firstTime;
    evosEventPool[event].reloadTime = reloadTime;
    evosEventPool[event].param      = param;
    evosEventPool[event].on         = true;
    HalInterruptRestore(istat);
  }
}


/*******************************************************************************
 * @brief Schedules a one-time event at an absolute time.
 *
 * @param[in] event  Handle to the event.
 * @param[in] time   Absolute time (in ticks) at which the event should trigger.
 * @param[in] param  Parameter to pass to the event handler.
 *
 * @details 
 * - The event will fire once at the specified `time` and then automatically
 *   be disabled (unless otherwise re-scheduled).
 * - This function is atomic to ensure thread safety.
 *
 * @note 
 *  One-time events are perfect for delayed operations or timeouts in 
 *  asynchronous systems.
 ******************************************************************************/
void EvosEventSet(
  const EvosEventHandle_t event,
  const PlfTime_t time, 
  EvosEventParam_t param)
{
  if (event < EVOS_POOL_SIZE) {
    HalIrqStat_t istat;
    HalInterruptDisable(&istat);
    evosEventPool[event].time  = time;
    evosEventPool[event].param = param;
    evosEventPool[event].on    = true;
    HalInterruptRestore(istat);
  }
}


/*******************************************************************************
 * @brief Schedules a one-time event relative to the current time.
 *
 * @param[in] event      Handle to the event.
 * @param[in] deltaTime  Delay in ticks from the current time.
 * @param[in] param      Parameter to pass to the event handler.
 *
 * @details 
 * - Adds `deltaTime` to the current system time and schedules the event.
 * - Use this variant when you already have an up-to-date `evosCurrentTime`.
 *
 * @note 
 *  Useful when you want to ensure an event fires after a known delay, 
 *  without performing manual calculations.
 ******************************************************************************/
void EvosEventSetDelta(
  const EvosEventHandle_t event,
  const PlfTime_t         deltaTime, 
  EvosEventParam_t        param )
{
  EvosEventSet(event, evosCurrentTime + deltaTime, param);
}


/*******************************************************************************
 * @brief Schedules a one-time event relative to a freshly sampled current time.
 *
 * @param[in] event      Handle to the event.
 * @param[in] deltaTime  Delay in ticks from the (freshly polled) current time.
 * @param[in] param      Parameter to pass to the event handler.
 *
 * @details 
 * - Explicitly polls the current time (`PlfTimeMsGet`) before scheduling,
 *   ensuring minimal uncertainty in scheduling.
 * - Adds `deltaTime` to the newly retrieved time and schedules the event.
 *
 * @note 
 *  Use this function when you need to reduce jitter from potentially
 *  outdated `evosCurrentTime`.
 ******************************************************************************/
void EvosEventSetDelta2(
  const EvosEventHandle_t event,
  const PlfTime_t         deltaTime, 
  EvosEventParam_t        param )
{
  PlfTimeMsGet(&evosCurrentTime);
  EvosEventSet(event, evosCurrentTime + deltaTime, param);
}


/*******************************************************************************
 * @brief Immediately schedules an event to run "now".
 *
 * @param[in] event  Handle to the event.
 * @param[in] param  Parameter to pass to the event handler.
 *
 * @details 
 * - Sets the event to fire at the current system time (`evosCurrentTime`).
 * - The event will fire in the next scheduler cycle.
 *
 * @note 
 *  Useful if you need to trigger an event’s handler as soon as possible
 *  while still respecting the EVOS cooperative scheduling model.
 ******************************************************************************/
void EvosEventSetNow(
  const EvosEventHandle_t event, 
  EvosEventParam_t param)
{
  EvosEventSet(event, evosCurrentTime, param);
}


/*******************************************************************************
 * @brief Re-registers (updates the handler of) an existing event.
 *
 * @param[in] event  Handle to the event.
 * @param[in] pFunc  New function pointer to the event handler.
 *
 * @details 
 * - Overwrites the existing event’s function pointer with `pFunc`.
 * - Disables the event (`on = false`) until it is next scheduled.
 *
 * @note 
 *  Can be used to change the behavior of an event without needing to 
 *  find a new slot in the event pool.
 ******************************************************************************/
void EvosEventReregister(
  const EvosEventHandle_t  event,
  const EvosEventFunc_t  pFunc)
{
  evosEventPool[event].func  = pFunc;
  evosEventPool[event].on    = false;
}


/*******************************************************************************
 * @brief Deletes an event from the EVOS system.
 *
 * @param[in] event  Handle to the event, or EVOS_CURRENT_HANDLE to delete the 
 *                   event currently being processed.
 *
 * @details 
 * - Resets the event structure to zero, effectively removing it from
 *   the scheduling system.
 * - If the event is the one currently running (`EVOS_CURRENT_HANDLE`),
 *   it cleans that event slot.
 *
 * @warning 
 *  Deleting the currently running event will prevent any further references
 *  to its handle. Be cautious when using this during event handler execution.
 ******************************************************************************/
void EvosEventDelete(
  const EvosEventHandle_t  event)
{
  if (event == EVOS_CURRENT_HANDLE) {
    memset(&evosEventPool[currentEvent], 0, sizeof(EvosEvent_t));
  } else {
    memset(&evosEventPool[event], 0, sizeof(EvosEvent_t));
  }
}


/*******************************************************************************
 * @brief Retrieves the name of a registered event.
 *
 * @param[in] i  Handle (index) of the event.
 * 
 * @return Null-terminated string containing the event name.
 *
 * @details 
 * - Helps with debugging, logging, or GUI displays showing event states.
 ******************************************************************************/
const char * EvosEventNameGet(EvosEventHandle_t i)
{
  return evosEventPool[i].name;
}


/*******************************************************************************
 * @brief Gets the scheduled time for an event if it is active.
 *
 * @param[out] time   Pointer to store the event’s scheduled time.
 * @param[in]  event  Handle to the event.
 *
 * @return `true` if the event is active and `time` was set, `false` otherwise.
 *
 * @details 
 * - Allows introspection into future firing times of an event.
 ******************************************************************************/
bool EvosEventTimerGet(PlfTime_t * time, EvosEventHandle_t event)
{
  if (evosEventPool[event].on) {
    *time = evosEventPool[event].time;
    return true;
  }
  return false;
}


/*******************************************************************************
 * @brief Atomic check to see if the time is up for the current event.
 *
 * @return `true` if the current event’s scheduled time has passed, `false` otherwise.
 *
 * @details 
 * - Disables the event (`on = false`) if the time has passed.
 * - Wrapped in an atomic section to ensure concurrency safety, 
 *   preventing race conditions with time updates.
 ******************************************************************************/
static bool _TimeUpAndClearAtomic(void)
{
  bool run = false;
  HAL_START_ATOMIC();
  if (evosEventPool[currentEvent].on) {
    if (TIME_UP(evosCurrentTime, evosEventPool[currentEvent].time)) {
      run = true;
      evosEventPool[currentEvent].on = false;
    }
  }
  HAL_END_ATOMIC();
  return run;
}


/*******************************************************************************
 * @brief Runs the EVOS scheduler check loop once.
 *
 * @details 
 * - Updates `evosCurrentTime` by calling `PlfTimeMsGet`.
 * - Iterates over each event in the pool, checking if it is due to run
 *   by invoking `_TimeUpAndClearAtomic`.
 * - If an event is due, increments stats (if CLI logging is enabled),
 *   runs the event handler, and re-schedules it if `reloadTime` is non-zero.
 *
 * @note 
 *  This function must be periodically called (e.g., from a main loop or RTOS 
 *  task) for EVOS to dispatch events. EVOS does not spawn its own threads, 
 *  so it relies on cooperative calls.
 ******************************************************************************/
void EvosSchedulerCheck(void)
{
  PlfTimeMsGet(&evosCurrentTime);
  HalWdtFeed();  
  
  for (currentEvent=0; currentEvent<EVOS_POOL_SIZE; currentEvent++) {
    if (_TimeUpAndClearAtomic()) {
      #if defined(CLI_ENABLE) 
        evosEventLog[currentEvent].count++;
        PlfTimeMsGet(&startTime);
      #endif
      
      /* Store scheduled time for diagnostic/log purposes */
      evosScheduledTime = evosEventPool[currentEvent].time;
      
      /* Reschedule if recurring */
      if (evosEventPool[currentEvent].reloadTime > 0) {
        evosEventPool[currentEvent].time += evosEventPool[currentEvent].reloadTime;
        evosEventPool[currentEvent].on = true;
      }
      
      /* Execute the event handler function */
      evosEventPool[currentEvent].func(
        #ifdef EVOS_PARAM_USED
          evosEventPool[currentEvent].param
        #endif
        );
      
      #if defined(CLI_ENABLE) 
        PlfTimeMsGet(&stopTime);
        evosEventLog[currentEvent].time += stopTime - startTime;
      #endif
    }
  }
}


/*******************************************************************************
 * @brief Starts an infinite scheduling loop for EVOS.
 *
 * @details 
 * - Repeatedly calls `EvosSchedulerCheck()` in a never-ending loop.
 * - If compiled on an RTOS, it performs a small delay (`RTOS_TASK_DELAY(1)`) 
 *   to give other tasks CPU time.
 *
 * @note 
 *  This is an optional convenience function if you want EVOS to run in its 
 *  own dedicated loop instead of being called manually.
 ******************************************************************************/
void EvosSchedulerStart(void)
{
  for (;;) {
#if (PLF_OS == PLF_OS_RTOS)
    RTOS_TASK_DELAY(1);
#endif
    EvosSchedulerCheck();
  }
}


/*******************************************************************************
 * @brief Initializes the EVOS system (event pool).
 *
 * @details 
 * - Clears out the event pool on first call.
 * - Sets a static `init` flag to prevent re-initialization.
 * - Must be called before using any other EVOS functions.
 *
 * @note 
 *  Allows multiple modules to rely on EVOS without worrying about 
 *  double-initialization, since it checks the `init` flag.
 ******************************************************************************/
void EvosInit(void)
{
  static bool init = false;
  if (!init) {
    memset(&evosEventPool, 0, sizeof(evosEventPool));
    init = true;
  }
}
