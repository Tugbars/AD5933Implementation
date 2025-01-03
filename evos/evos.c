#include <string.h>
#include <stdio.h>

#include "evos/evos.h"


/*******************************************************************************
 *    Type definitions
 ******************************************************************************/

typedef struct {
  bool                on;    /**< true if the event is signaled. */
  PlfTime_t             time;  /**< Time at which the event will occur. */
  PlfTime_t             reloadTime;
  EvosEventParam_t      param; /**< Parameter value associated with an occurrence of the event. */
  EvosEventFunc_t       func;  /**< Pointer to event handler function. */
  const char            *name; /**< Name of event in string form. Used only for trace purposes. */
}                       EvosEvent_t;

/*******************************************************************************
 *    Local variables
 ******************************************************************************/

static EvosEvent_t      evosEventPool[EVOS_POOL_SIZE];  /**< Global pool of events. */
PlfTime_t               evosCurrentTime;   /**< Current timer tick value as obtained from HAL. */
PlfTime_t               evosScheduledTime;   /**< Scheduled timer tick value for current event. */
static EvosEventHandle_t currentEvent;


/******************************************************************************/
#ifdef CLI_ENABLE

/******************************************************************************/

typedef struct {
  uint32_t              count;
  PlfTime_t             time;
}                       EvosLog_t;

static EvosLog_t        evosEventLog[EVOS_POOL_SIZE];
static PlfTime_t        measureStartTime; //, measureStopTime;
static PlfTime_t        startTime, stopTime;


/******************************************************************************/
#endif /*  */
/******************************************************************************/

/*******************************************************************************
 * @brief Register a new event in the system.
 * 
 * @param[in] pFunc Function to execute when the event triggers.
 * @param[in] pName Name of the event for tracing purposes.
 * @return Handle to the registered event.
 * 
 * @details This function allocates an available slot in the event pool for
 * the new event. If the pool is full, the system halts execution.
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
    for (;;) ;
  } else {
    //memset(&evosEventPool[i], 0, sizeof(EvosEvent_t));
    evosEventPool[i].func = pFunc;
    evosEventPool[i].name = pName;
  }
  return i;
}

/*******************************************************************************
 * @brief Clear an event from the system.
 * 
 * @param[in] event Handle to the event to clear.
 * 
 * @details Disables the specified event and clears its timing and parameters.
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
 * @brief Schedule a recurring event with a specific time interval.
 * 
 * @param[in] event Handle to the event.
 * @param[in] firstTime Time for the first occurrence of the event.
 * @param[in] reloadTime Time interval for subsequent occurrences.
 * @param[in] param Parameter to pass to the event handler.
 * 
 * @details Sets the event to trigger repeatedly at the specified interval.
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
    evosEventPool[event].time  = firstTime;
    evosEventPool[event].reloadTime = reloadTime;
    evosEventPool[event].param = param;
    evosEventPool[event].on = true;
    HalInterruptRestore(istat);
  }
}

/*******************************************************************************
 * @brief Schedule a one-time event.
 * 
 * @param[in] event Handle to the event.
 * @param[in] time Absolute time when the event should trigger.
 * @param[in] param Parameter to pass to the event handler.
 * 
 * @details Configures the event to trigger once at the specified time.
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
    //evosEventPool[event].reloadTime = 0;
    evosEventPool[event].param = param;
    evosEventPool[event].on = true;
    HalInterruptRestore(istat);
  }
}

/*******************************************************************************
 * @brief Schedule a one-time event with a delay from the current time.
 * 
 * @param[in] event Handle to the event.
 * @param[in] deltaTime Delay in ticks from the current time.
 * @param[in] param Parameter to pass to the event handler.
 ******************************************************************************/
void EvosEventSetDelta(
  const EvosEventHandle_t event,
  const PlfTime_t         deltaTime, 
  EvosEventParam_t        param )
{
  EvosEventSet(event, evosCurrentTime+deltaTime, param);
}


void EvosEventSetDelta2(
  const EvosEventHandle_t event,
  const PlfTime_t         deltaTime, 
  EvosEventParam_t        param )
{
  PlfTimeMsGet(&evosCurrentTime);
  EvosEventSet(event, evosCurrentTime+deltaTime, param);
}

/*******************************************************************************
 * @brief Schedule an event to trigger immediately.
 * 
 * @param[in] event Handle to the event.
 * @param[in] param Parameter to pass to the event handler.
 ******************************************************************************/
void EvosEventSetNow(
  const EvosEventHandle_t event, 
  EvosEventParam_t param)
{
  EvosEventSet(event, evosCurrentTime, param);
}

/******************************************************************************/

/* Re-Register event */
void EvosEventReregister(
  const EvosEventHandle_t  event,
  const EvosEventFunc_t  pFunc)
{
  //memset(&evosEventPool[event], 0, sizeof(EvosEvent_t));
  evosEventPool[event].func  = pFunc;
  evosEventPool[event].on    = false;
}

/* Re-Register event */
void EvosEventDelete(
  const EvosEventHandle_t  event)
{
  if (event == EVOS_CURRENT_HANDLE) {
    memset(&evosEventPool[currentEvent], 0, sizeof(EvosEvent_t));
  } else {
    memset(&evosEventPool[event], 0, sizeof(EvosEvent_t));
  }
}


/******************************************************************************/

const char * EvosEventNameGet(EvosEventHandle_t i)
{
  return evosEventPool[i].name;
}

/******************************************************************************/

bool EvosEventTimerGet(PlfTime_t * time, EvosEventHandle_t event)
{
  if (evosEventPool[event].on) {
    *time = evosEventPool[event].time;
    return true;
  }
  return false;
}


/******************************************************************************/

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
      evosScheduledTime = evosEventPool[currentEvent].time;
      if (evosEventPool[currentEvent].reloadTime > 0) {
        evosEventPool[currentEvent].time += evosEventPool[currentEvent].reloadTime;
        evosEventPool[currentEvent].on = true;
      }
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

void EvosSchedulerStart(void)
{
  for (;;) {
#if (PLF_OS == PLF_OS_RTOS)
		RTOS_TASK_DELAY(1);
#endif
	  EvosSchedulerCheck();

  }
}

/******************************************************************************/

static bool init = false;;
void EvosInit(void)
{
  if (!init) {
    memset(&evosEventPool, 0, sizeof(evosEventPool));
    init = true;
  }
}


