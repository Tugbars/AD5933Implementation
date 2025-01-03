/**
 * @file    evos.h
 * @brief   EVOS (Event-based Operating System) - Public API
 *
 * The EVOS (Event-based Operating System) framework provides a lightweight,
 * cooperative event scheduler for embedded systems. It is designed for
 * use-cases where a full RTOS might be too large, complex, or unnecessary.
 * EVOS allows multiple tasks to be scheduled and run asynchronously in a
 * cooperative manner without the overhead of multiple threads or context
 * switching. 
 *
 * ## Overview
 *
 * - **Event-Driven**: EVOS works by registering "events," each associated 
 *   with a function (handler) to be executed when certain timing conditions 
 *   are met (e.g., at a specific time or at a fixed interval).
 * - **Cooperative Scheduling**: Events run to completion within a single 
 *   scheduler loop (`EvosSchedulerCheck()` or `EvosSchedulerStart()`), 
 *   meaning no preemption occurs during the event handler. This simplifies 
 *   concurrency concerns because shared resources can be protected easily 
 *   with simple atomic operations.
 * - **Minimal Footprint**: Memory usage is controlled by a compile-time 
 *   configuration (`EVOS_POOL_SIZE`), where a static array of event slots 
 *   is allocated. This makes EVOS suitable for resource-constrained 
 *   environments.
 * - **Thread Safety**: Critical sections (e.g., enabling/disabling events) 
 *   are protected by atomic operations or interrupt disables. This ensures 
 *   safe updates even if the scheduler or interrupts are active.
 *
 * ## What EVOS Solves
 *
 * 1. **Simplifies Concurrency**: By running event handlers one-at-a-time in 
 *    a cooperative loop, you avoid many complexities of multithreaded 
 *    programming (race conditions, complex synchronization primitives, etc.).
 * 2. **Scheduling Flexibility**: Events can be scheduled once (`EvosEventSet()`)
 *    or configured to reoccur at a fixed interval (`EvosEventSetAndReload()`).
 * 3. **Deterministic Memory Usage**: With a fixed pool of event slots, 
 *    there is no dynamic memory allocation at runtime, helping to ensure 
 *    predictable behavior.
 * 4. **Low Overhead**: EVOS offers much of the functionality of a basic RTOS 
 *    (timers, scheduling) but at a fraction of the complexity and code size.
 *
 * ## Typical Use Cases
 * - Periodic tasks like sensor polling or data transmission (using 
 *   `EvosEventSetAndReload()`).
 * - One-shot delayed actions or timeouts (using `EvosEventSet()` or 
 *   `EvosEventSetDelta()`).
 * - Immediate scheduling from an ISR (using `EvosEventSetNow()`).
 *
 * ## How It Works
 * 1. **Initialization**: Call `EvosInit()` once at system startup to clear 
 *    and prepare the event pool.
 * 2. **Register Events**: Use `EvosEventRegister()` to allocate a slot in 
 *    the pool and retrieve a handle to your new event.
 * 3. **Scheduling**: Set your event to run at a specific time with 
 *    `EvosEventSet()`, at a fixed interval with `EvosEventSetAndReload()`, 
 *    or as soon as possible with `EvosEventSetNow()`.
 * 4. **Execution**: Repeatedly call `EvosSchedulerCheck()` (or run 
 *    `EvosSchedulerStart()` in a dedicated loop). EVOS will check all 
 *    active events, dispatch their handlers if their scheduled time has 
 *    arrived, and reschedule periodic events accordingly.
 * 5. **Modification**: Events can be cleared, re-registered with a new 
 *    handler, or deleted at any time, allowing you to dynamically adapt 
 *    your system’s behavior.
 *
 * EVOS thus provides a robust yet simple framework for event-based 
 * concurrency in embedded systems, striking a balance between ease-of-use, 
 * real-time responsiveness, and memory efficiency.
 */

#ifndef EVOS_H
#define EVOS_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
// Configuration of EvOS ...
#include "cfg/evos_cfg.h"

/******************************************************************************/
#define EVOS_CURRENT_HANDLE         (EvosEventHandle_t)(UINT8_MAX)
#define EVOS_UNINITIALIZED_HANDLE   (EvosEventHandle_t)(UINT8_MAX)

/******************************************************************************/
/** 
 * @brief Current system time in milliseconds, as obtained from the hardware
 * abstraction layer (HAL).
 */
extern PlfTime_t        evosCurrentTime;

/** 
 * @brief The scheduled time for the event currently being executed.
 */
extern PlfTime_t        evosScheduledTime;

/******************************************************************************/
/**
 * @brief Registers a new event with a specified handler function and name.
 *
 * @param[in] pFunc  Pointer to the event handling function.
 * @param[in] pName  String name of the event (for debugging or logging).
 * 
 * @return Handle (index) to the newly registered event.
 */
extern EvosEventHandle_t EvosEventRegister(
  const EvosEventFunc_t   pFunc,
  const char * const      pName);

/**
 * @brief Schedules a recurring event.
 *
 * @param[in] event       The handle to the event (from EvosEventRegister).
 * @param[in] firstTime   Absolute time (ms) at which to fire the first event.
 * @param[in] reloadTime  Interval (ms) after which the event will reload.
 * @param[in] param       User-defined parameter passed to the event handler.
 */
extern void EvosEventSetAndReload(
  const EvosEventHandle_t event,
  const PlfTime_t firstTime,
  const PlfTime_t reloadTime,
  EvosEventParam_t param);

/**
 * @brief Schedules a one-shot event at an absolute time.
 *
 * @param[in] event  The event handle.
 * @param[in] time   Absolute time (ms) at which the event fires.
 * @param[in] param  User-defined parameter passed to the event handler.
 */
extern void EvosEventSet(
  const EvosEventHandle_t event,
  const PlfTime_t         time,
  EvosEventParam_t        param);

/**
 * @brief Schedules a one-shot event after a specified delay (relative to now).
 *
 * @param[in] event     The event handle.
 * @param[in] deltaTime Delay in milliseconds from the current time.
 * @param[in] param     User-defined parameter passed to the event handler.
 */
extern void EvosEventSetDelta(
  const EvosEventHandle_t event,
  const PlfTime_t  deltaTime,
  EvosEventParam_t param);

/**
 * @brief Schedules a one-shot event after a specified delay, refreshing the 
 * current time first.
 *
 * @param[in] event     The event handle.
 * @param[in] deltaTime Delay in milliseconds from the newly polled current time.
 * @param[in] param     User-defined parameter passed to the event handler.
 */
extern void EvosEventSetDelta2(
  const EvosEventHandle_t event,
  const PlfTime_t         deltaTime, 
  EvosEventParam_t        param );

/**
 * @brief Immediately schedules an event to run at the current time.
 *
 * @param[in] event  The event handle.
 * @param[in] param  User-defined parameter passed to the event handler.
 */
extern void EvosEventSetNow(
  const EvosEventHandle_t event,
  EvosEventParam_t param);

/**
 * @brief Clears (disables) the specified event so it will no longer fire.
 *
 * @param[in] event  The event handle.
 */
extern void EvosEventClear(
  const EvosEventHandle_t event);

/**
 * @brief Re-registers (updates) an event with a new handler function.
 *
 * @param[in] event  The event handle.
 * @param[in] pFunc  New function pointer for handling this event.
 */
extern void EvosEventReregister(
  const EvosEventHandle_t event,
  const EvosEventFunc_t   pFunc);

/**
 * @brief Deletes an event entirely, freeing its slot in the event pool.
 *
 * @param[in] event  The event handle, or \ref EVOS_CURRENT_HANDLE for the event 
 *                   currently in execution.
 */
extern void EvosEventDelete(
  const EvosEventHandle_t event);

/**
 * @brief Retrieves the scheduled time (ms) for an event if it is active.
 *
 * @param[out] time   Pointer to store the event’s scheduled time.
 * @param[in]  event  The event handle.
 * 
 * @return True if the event is active and `time` was filled, false otherwise.
 */
extern bool EvosEventTimerGet(PlfTime_t * time, EvosEventHandle_t event);

/******************************************************************************/
#ifdef CLI_ENABLE
/**
 * @brief CLI command that prints EVOS event statistics (if available).
 *
 * @param[in] param1  Reserved (implementation-specific).
 * @param[in] param2  Reserved (implementation-specific).
 * @param[in] param3  Reserved (implementation-specific).
 *
 * @return Implementation-specific status code.
 */
extern int_fast16_t CliEvosStat(CliParam_t param1, CliParam_t param2, CliParam_t param3);
#endif /* CLI_ENABLE */

/**
 * @brief Retrieves the name of an event given its handle.
 *
 * @param[in] i  The event handle.
 * 
 * @return Null-terminated string of the event name, or NULL if not available.
 */
extern const char * EvosEventNameGet(EvosEventHandle_t i);

/******************************************************************************/
/**
 * @brief Checks all EVOS events and dispatches any that are due.
 *
 * @details Should be called periodically (e.g., in the main loop or from an 
 * RTOS task). This function updates `evosCurrentTime`, checks each event’s 
 * schedule, and executes the handler if its time has come. Recurring events 
 * are automatically reloaded.
 */
extern void EvosSchedulerCheck(void);

/**
 * @brief Starts a dedicated, infinite scheduling loop for EVOS.
 *
 * @details This function repeatedly calls `EvosSchedulerCheck()` (and may 
 * also include a short delay on RTOS-based systems). It is an optional 
 * convenience method if you wish to run EVOS in its own standalone loop.
 */
extern void EvosSchedulerStart(void);

/**
 * @brief Initializes the EVOS system (event pool, time tracking, etc.).
 *
 * @details Must be called once at system startup before registering events.
 */
extern void EvosInit(void);

/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif //EVOS_H
