/**
 * @file    ics.c
 * @brief   Implementation of the AD5933 impedance converter / network analyzer.
 *
 * This module provides an interface to the AD5933 chip using:
 * - **Protothreads** for lightweight, stackless concurrency.
 * - **EVOS (Event-based OS)** for event scheduling and timing.
 *
 * @section integration Overall Integration of EVOS and Protothreads
 *
 * The design pairs EVOS events with Protothread "steps" to achieve non-blocking,
 * cooperative multitasking on a single-threaded MCU (STM environment):
 * - **EVOS Scheduling**: 
 *   - An event (see @ref drvHandle) is registered with EVOS via `EvosEventRegister`.
 *   - EVOS periodically invokes the event’s callback (`DriverOnEvent`), which runs
 *     the main protothread function (`DriverUpdate`) one step at a time.
 * - **Protothreads Execution**:
 *   - Protothreads (`DriverUpdate`, `ReadSamples`) use `PT_BEGIN` and `PT_YIELD` 
 *     macros to yield control back to EVOS. 
 *   - This allows other events and threads to execute between yields, 
 *     preventing blocking of the main loop.
 *
 * Combining these two abstractions allows:
 * - **Non-Blocking I2C** reads/writes, where each step yields if the hardware
 *   needs more time, avoiding a busy-wait or blocking delay on the STM MCU.
 * - **Dynamic Scheduling** of tasks (like frequency sweeps) without manual 
 *   polling. Events are triggered based on time (`EvosEventSetDelta2`) or 
 *   immediate triggers (`EvosEventSetNow`).
 * - **Modular Task Execution**, where each protothread encapsulates a 
 *   distinct flow (e.g., reading samples) while EVOS manages the timing 
 *   and sequence of when that flow runs.
 *
 * @section benefits Why Protothreads and Event-Based OS?
 *
 * - **State Management**: Protothreads remove the need for explicit state 
 *   machines in the application logic, simplifying transitions between 
 *   different stages of a frequency sweep.
 * - **Non-Blocking Operation**: Both I2C operations and lengthy sweeps 
 *   are run cooperatively, allowing the rest of the system to proceed 
 *   without stalling.
 * - **Dynamic Scheduling**: EVOS triggers tasks based on time or conditions, 
 *   avoiding tight loops or rigid cycles in the main STM main loop or an RTOS task.
 *
 * @subsection usage_flow Typical Usage Flow
 * 1. **Initialization**: 
 *    - `HalIcsInit()` registers an EVOS event (`drvHandle`), initializes 
 *      the protothread, and schedules the first run after a power-up wait.
 * 2. **Detection & Configuration**: 
 *    - The `DriverUpdate` protothread runs in steps, detecting the AD5933,
 *      configuring clocks/gains, and then transitions to an idle state.
 * 3. **Starting a Frequency Sweep**:
 *    - An external call to `HalIcsStartFreqSweep()` sets up a sweep callback 
 *      and flags the system to enter the sweep routine in the next protothread 
 *      iteration.
 * 4. **Reading Samples**:
 *    - The nested protothread `ReadSamples` handles I2C polling of real and 
 *      imaginary data, yielding periodically to allow other events to execute.
 * 5. **Completion & Power Down**:
 *    - Once the sweep is complete (or aborted), the protothread resets the 
 *      AD5933 and returns to idle.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <i2c.h>
#include <stdio.h>   // For printf
#include <stdarg.h>  // For variadic functions

#include "ics.h"
#include "pt/pt.h"
#include "evos/evos.h"
#include "ad5933.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/// @brief AD5933 device 7-bit address.
#define AD5933_7BIT_ADDRESS       0x0D

/// @brief AD5933 register addresses.
#define AD5933_REG_CTRL_HIGH      0x80
#define AD5933_REG_CTRL_LOW       0x81
#define AD5933_REG_FREQ_START     0x82
#define AD5933_REG_FREQ_INC       0x85
#define AD5933_REG_INC_NUM        0x88
#define AD5933_REG_SETTLING_CYCLES 0x8A
#define AD5933_REG_STATUS         0x8F
#define AD5933_REG_TEMP_DATA      0x92
#define AD5933_REG_REAL_DATA      0x94
#define AD5933_REG_IMAG_DATA      0x96

/// @brief AD5933 I2C block commands (not used directly here).
#define AD5933_I2C_BLOCK_WRITE    0xA0
#define AD5933_I2C_BLOCK_READ     0xA1
#define AD5933_I2C_ADDR_POINTER   0xB0

/*******************************************************************************
 * Type definitions
 ******************************************************************************/

/**
 * @brief Bitfield representing the AD5933 Control High register.
 */
typedef struct
{
  uint8_t pgaGain : 1;              /**< PGA gain setting bit. */
  uint8_t outputVoltageRange : 2;    /**< Output voltage range bits. */
  uint8_t noOperation : 1;          /**< Reserved / no-op bit. */
  uint8_t func : 4;                 /**< Main function bits. */
} Ad5933CtrlHigh_t;

/**
 * @brief Defines possible PGA gain settings.
 */
typedef enum
{
  AD5933_PGA_GAIN_X5,
  AD5933_PGA_GAIN_X1
} Ad5933CtrlHighPgaGain_t;

/**
 * @brief Defines possible cycle multiplication factors for settling cycles.
 */
typedef enum
{
  Ad5933_NO_OF_CYCLES_DEFAULT = 0,
  Ad5933_NO_OF_CYCLES_X2 = 1,
  Ad5933_NO_OF_CYCLES_X4 = 3,
} Ad5933NoOfCycles;

/**
 * @brief Encapsulates settling time info (multiplication and count).
 */
typedef struct
{
  Ad5933NoOfCycles NoOfCycles; /**< Multiplication factor for settling cycles. */
  uint16_t SettligCycles;      /**< Actual number of settling cycles. */
} Ad5933SettlingTime_t;

/**
 * @brief Identifiers for the main functional modes of AD5933.
 */
typedef enum
{
  AD5933_FUNC_NO_OPERATION = 0x00,
  AD5933_FUNC_INIT_WITH_START_FREQ,
  AD5933_FUNC_START_FREQ_SWEEP,
  AD5933_FUNC_INCREMENT_FREQ,
  AD5933_FUNC_REPEAT_FREQ,
  AD5933_FUNC_MEASURE_TEMP = 0x09,
  AD5933_FUNC_POWER_DOWN_MODE,
  AD5933_FUNC_STANDBY_MODE,
} Ad5933CtrlHighFunc_t;

/**
 * @brief Bitfield representing the AD5933 Control Low register.
 */
typedef struct
{
  uint8_t reserved1 : 3;        /**< Reserved bits. */
  uint8_t externalClock : 1;    /**< External clock enable bit. */
  uint8_t reset : 1;            /**< Reset bit. */
  uint8_t reserved2 : 3;        /**< Reserved bits. */
} Ad5933CtrlLow_t;

/**
 * @brief Bitfield representing the AD5933 Status register.
 */
typedef struct
{
  uint8_t validTempMeasurement : 1;      /**< Temperature measurement ready. */
  uint8_t validRealImaginaryData : 1;    /**< Real/Imag data valid. */
  uint8_t freqSweepComplete : 1;         /**< Frequency sweep completion flag. */
  uint8_t reserved : 5;                  /**< Reserved bits. */
} Ad5933Status_t;

/**
 * @brief Union for reading/writing any AD5933 register as byte or structured bitfield.
 */
typedef union
{
  uint8_t byte; 
  Ad5933CtrlHigh_t ctrlHigh;
  Ad5933CtrlLow_t  ctrlLow;
  Ad5933Status_t   status;
} Ad5933Reg_t;

/**
 * @brief Start Frequency or Frequency Increment register structure (3 bytes).
 */
typedef struct __attribute__((packed))
{
  uint8_t high; 
  uint8_t mid;  
  uint8_t low;  
} Ad5933StartFreq_t, Ad5933FreqInc_t;

/**
 * @brief Increment Number or generic 2-byte structure.
 */
typedef struct __attribute__((packed))
{
  uint8_t high; 
  uint8_t low;  
} Ad5933IncNum_t, Ad5933Number_t;

/**
 * @brief Composite structure for writing start freq, freq increment, and increment number in one go.
 */
typedef union __attribute__((packed))
{
  uint8_t bytes[sizeof(Ad5933StartFreq_t) + sizeof(Ad5933FreqInc_t) + sizeof(Ad5933IncNum_t)];
  struct
  {
    Ad5933StartFreq_t freqStart; /**< Starting frequency bytes. */
    Ad5933FreqInc_t   freqInc;   /**< Increment frequency bytes. */
    Ad5933IncNum_t    incNum;    /**< Number of increments. */
  };
} Ad5933Setup_t;

/**
 * @brief Real & imaginary data container for reading out samples (2 x 2 bytes).
 */
typedef union __attribute__((packed))
{
  uint8_t bytes[sizeof(Ad5933Number_t) * 2];
  struct
  {
    Ad5933Number_t real;       /**< Real part of the sample. */
    Ad5933Number_t imaginary;  /**< Imag part of the sample. */
  };
} Ad5933Sample_t;

/*******************************************************************************
 * Local constant variables
 ******************************************************************************/

#if defined(EVK)
static I2C_HandleTypeDef *const I2C = &hi2c3;   /**< Using I2C3 on the EVK board. */
#else
static I2C_HandleTypeDef *const I2C = &hi2c1;   /**< Default to I2C1 otherwise. */
#endif

static const uint8_t ADDRESS = AD5933_7BIT_ADDRESS << 1; /**< 7-bit address shifted left for STM HAL. */

static const uint32_t I2C_TIMEOUT_MSEC = 1;             /**< Timeout in ms for I2C operations. */
static const float_t INTERNAL_CLOCK = 16776000.0f;       /**< Approx. 16.776 MHz internal clock of AD5933. */
static const float_t FREQ_START = 10000.0f;              /**< Default start frequency (Hz). */
static const float_t FREQ_INC = 1.0f;                    /**< Default frequency increment (Hz). */
static const uint16_t INC_NUM = 501;                     /**< Default number of increments. */
static const PlfTime_t POWER_UP_WAIT_MSEC = 20;          /**< Time to wait after power-up for device stability. */

/*******************************************************************************
 * Tracing (Debug) definitions
 ******************************************************************************/
typedef enum {
    TRACE_LEVEL_NONE = 0,  
    TRACE_LEVEL_ERROR = 1, 
    TRACE_LEVEL_INFO = 2,  
    TRACE_LEVEL_DEBUG = 3, 
    TRACE_LEVEL_VERBOSE = 4
} TraceLevelEnum_t;

static const TraceLevel_t CURRENT_TRACE_LEVEL = TRACE_LEVEL_DEBUG; /**< Current trace verbosity. */

typedef enum {
    TRC_TA_HAL = 1,
} TraceCategory_t;

/**
 * @brief Prints debug messages if the specified level is <= the current trace level.
 * 
 * @param category     Category identifier.
 * @param level        Trace level of the message.
 * @param format       printf-style format string.
 * @param ...          Additional arguments for the format string.
 *
 * Internally uses `vprintf` to format the string and outputs a newline at the end.
 */
void Trace_Print(TraceCategory_t category, TraceLevel_t level, const char *format, ...) {
    if (level <= CURRENT_TRACE_LEVEL) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        printf("\n");
    }
}

/// @brief Simplifies calling Trace_Print() without manually creating va_list.
#define TRACE(category, level, ...) \
    Trace_Print(category, level, __VA_ARGS__)

/**
 * @brief Variant macro for trace output that allows custom format strings with arguments.
 */
#define TRACE_VA(category, level, format, ...) \
    Trace_Print(category, level, format, __VA_ARGS__)

// Additional trace levels
static const TraceLevel_t TL_ICS     = TRACE_LEVEL_DEBUG; 
static const TraceLevel_t TL_ICS_I2C = TRACE_LEVEL_DEBUG; 

/*******************************************************************************
 * Local data variables
 ******************************************************************************/

/** 
 * @brief EVOS event handle for this driver. 
 *
 * All protothread steps are triggered through `drvHandle` events in the EVOS.
 */
static EvosEventHandle_t drvHandle = EVOS_UNINITIALIZED_HANDLE; 

/** 
 * @brief Main protothread structure for driver flow. 
 */
static PThread_t drvThread;

static bool isDetected = false;        /**< Flag indicating if AD5933 was detected. */
static bool isIdle = false;           /**< Flag indicating if driver is idle (no sweep in progress). */
static bool isSweepAborted = false;   /**< Sweep-abort request flag. */

static HalIcsSweepSampleCb_t sampleCb = NULL;  /**< Callback for handling each frequency sweep sample. */

static float_t freqStart;             /**< Cached start frequency in Hz. */
static float_t freqInc;               /**< Cached frequency increment in Hz. */
static uint16_t freqIncNum;           /**< Cached number of increments. */

/*******************************************************************************
 * Macros
 ******************************************************************************/

/**
 * @brief Yield the current protothread and schedule an immediate resume via EVOS.
 *
 * This macro yields from the current protothread and calls `EvosEventSetNow` 
 * to request that EVOS triggers this event again as soon as possible 
 * (next scheduler cycle).
 */
#define PT_YIELD_AND_EVOS_SET_NOW(pt) \
  EvosEventSetNow(drvHandle, 0);      \
  PT_YIELD((pt))

/**
 * @brief Yield the current protothread and request a delayed resume.
 *
 * This macro adds a delay (`msec`) in the EVOS scheduler before the next 
 * step of the protothread is invoked, preventing busy-wait loops.
 */
#define PT_YIELD_AND_EVOS_SET_DELTA(pt, msec) \
  EvosEventSetDelta2(drvHandle, (msec), 0);   \
  PT_YIELD((pt))

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

/** @brief Callback invoked by EVOS to drive the protothread steps. */
static void DriverOnEvent(EvosEventParam_t param);
static PT_THREAD(DriverUpdate(void));
static PT_THREAD(ReadSamples(PThread_t *pt));


static bool Detect(void);
static void SetupFreq(void);
static void Reset(void);
static void SetFunction(Ad5933CtrlHighFunc_t func);
static void SetPgaGainX1(void);
static void SetInternalClock(void);
static Ad5933Status_t GetStatus(void);
static void ReadRealAndImaginary(int16_t *real, int16_t *imaginary);
static uint32_t FreqToRegisterValue(float_t hz);
static int16_t RegisterValueToNumber(const Ad5933Number_t *number);
static bool SettlingTimeToRegisterValues(const Ad5933SettlingTime_t *settlingTime, uint8_t *buffer, uint8_t length);
static void SetupSettlingTime(void);
static bool RegisterWriteOneByte(uint8_t reg, uint8_t *data);
static bool RegisterReadOneByte(uint8_t reg, uint8_t *data);
static bool RegisterWrite(uint8_t reg, uint8_t *data, uint8_t size);
static bool RegisterRead(uint8_t reg, uint8_t *data, uint8_t size);

/*******************************************************************************
 * Public functions
 ******************************************************************************/

/**
 * @brief Initializes the ICS (Impedance Converter) driver for AD5933.
 *
 * - Sets default frequency values (start, increment, # of increments).
 * - Initializes the primary driver protothread.
 * - Registers an EVOS event (`drvHandle`) that will run `DriverOnEvent`.
 * - Schedules the first driver invocation after a short power-up wait (20 ms).
 *
 * @note Must be called once during system startup, typically in main or 
 *       another init function. Uses STM HAL I2C internally.
 */
void HalIcsInit(void)
{
  TRACE(TRC_TA_HAL, TRACE_LEVEL_INFO, "HalIcsInit()");

  freqStart = FREQ_START;
  freqInc = FREQ_INC;
  freqIncNum = INC_NUM;

  PT_INIT(&drvThread);
  drvHandle = EvosEventRegister(DriverOnEvent, "hal ics");
  
  // Wait 20 ms for device to stabilize, then call DriverOnEvent
  EvosEventSetDelta2(drvHandle, POWER_UP_WAIT_MSEC, 0);
}

/**
 * @brief Indicates whether the AD5933 chip was successfully detected.
 *
 * @return true if the AD5933 was found, false otherwise.
 */
bool HalIcsIsDetected(void)
{
  return isDetected;
}

/**
 * @brief Checks if the driver is currently idle (no sweep in progress).
 *
 * @return true if idle, false if an active sweep or setup is ongoing.
 */
bool HalIcsIsIdle(void)
{
  return isIdle;
}

/**
 * @brief Sets the start frequency (in Hz) for the next sweep.
 *
 * @param hz The start frequency in Hz.
 * 
 * Used by external code to configure AD5933 sweep parameters before starting.
 */
void HalIcsSetStartFreq(const float_t hz)
{
  freqStart = hz;
}

/**
 * @brief Retrieves the driver’s current start frequency setting.
 *
 * @return Start frequency in Hz.
 */
float_t HalIcsGetStartFreq(void)
{
  return freqStart;
}

/**
 * @brief Sets the frequency increment for the AD5933.
 *
 * @param hz The frequency increment in Hz.
 */
void HalIcsSetFreqInc(const float_t hz)
{
  freqInc = hz;
}

/**
 * @brief Gets the current frequency increment in Hz.
 */
float_t HalIcsGetFreqInc(void)
{
  return freqInc;
}

/**
 * @brief Sets the number of increments to be performed in a sweep.
 *
 * @param number The number of frequency increments (thus total sweep points = number + 1).
 */
void HalIcsSetNoOfSamples(const uint16_t number)
{
  freqIncNum = number;
}

/**
 * @brief Gets the current configured number of increments for sweeps.
 *
 * @return The number of increments.
 */
uint16_t HalIcsGetNoOfSamples(void)
{
  return freqIncNum;
}

/**
 * @brief Initiates a frequency sweep.
 *
 * @param cb Callback function invoked for each real/imag sample read during the sweep.
 * @return true if the sweep was successfully started, false if the driver is busy.
 *
 * Resets `isIdle` to false, sets up callback pointers, and immediately schedules 
 * the driver’s protothread for execution via EVOS (`EvosEventSetNow`).
 */
bool HalIcsStartFreqSweep(HalIcsSweepSampleCb_t cb)
{
  if (!isIdle)
  {
    return false;
  }

  EvosEventSetNow(drvHandle, 0);
  isIdle = false;
  isSweepAborted = false;
  sampleCb = cb;
  return true;
}

/**
 * @brief Aborts an ongoing frequency sweep.
 *
 * If idle, does nothing. Otherwise, flags `isSweepAborted` to stop the loop 
 * in `ReadSamples()`.
 */
void HalIcsAbortFreqSweep(void)
{
  if (isIdle)
  {
    return;
  }
  isSweepAborted = true;
}

/*******************************************************************************
 * Local functions, driver flow
 ******************************************************************************/

/**
 * @brief Callback triggered by EVOS to run the main driver protothread step.
 *
 * @param param Unused parameter in this implementation.
 *
 * EVOS calls this when the scheduled time arrives, ensuring cooperative 
 * execution with other events.
 */
static void DriverOnEvent(const EvosEventParam_t param)
{
  DriverUpdate();
}

/**
 * @brief Main driver protothread that controls device detection, sweep logic, and power states.
 *
 * @return Standard Protothread status (PT_ENDED or PT_WAITING).
 *
 * - First detects AD5933.  
 * - Configures internal clock and PGA gain.  
 * - Waits idle until a sweep is started, then sets up frequencies, runs `ReadSamples`, 
 *   and powers down afterwards.
 */
static PT_THREAD(DriverUpdate(void))
{
  static PThread_t child;

  PT_BEGIN(&drvThread);

  // Attempt detection
  if (Detect())
  {
    isDetected = true;
  }
  else
  {
    TRACE(TRC_TA_HAL, TRACE_LEVEL_ERROR, "ics not detected");
  }
  // Wait until the device is detected before proceeding
  PT_WAIT_UNTIL(&drvThread, isDetected);

  // Basic AD5933 config
  SetInternalClock();
  SetPgaGainX1();
  SetFunction(AD5933_FUNC_POWER_DOWN_MODE);
  Reset();
  isIdle = true;

  // Main loop
  while (true)
  {
    // Only run sweep logic when isIdle is cleared by external calls to HalIcsStartFreqSweep
    if (!isIdle)
    {
      TRACE(TRC_TA_HAL, TL_ICS, "ics prepare freq sweep");

      SetupFreq();
      SetupSettlingTime();
      SetFunction(AD5933_FUNC_STANDBY_MODE);
      Reset();

      // Initialize sweep
      SetFunction(AD5933_FUNC_INIT_WITH_START_FREQ);
      PT_YIELD_AND_EVOS_SET_NOW(&drvThread);

      TRACE(TRC_TA_HAL, TL_ICS, "ics start freq sweep");
      SetFunction(AD5933_FUNC_START_FREQ_SWEEP);

      // Nested Protothread for reading samples
      PT_SPAWN(&drvThread, &child, ReadSamples(&child));

      // Sweep complete or aborted
      SetFunction(AD5933_FUNC_POWER_DOWN_MODE);
      Reset();
      isIdle = true;
    }

    // Yield to allow other events to run
    PT_YIELD(&drvThread);
  }

  PT_END(&drvThread);
}

/**
 * @brief Nested protothread to read real/imag data from AD5933 during a sweep.
 *
 * @param[in] pt Pointer to this protothread's context structure.
 *
 * @return Standard Protothread status (PT_ENDED or PT_WAITING).
 *
 * The loop checks `freqSweepComplete` or `isSweepAborted` to decide when to end.
 * Each iteration waits until valid data is available, reads it, calls `sampleCb`, 
 * and increments the frequency until the sweep completes.
 */
static PT_THREAD(ReadSamples(PThread_t *const pt))
{
  static Ad5933Status_t status;
  static int16_t real;
  static int16_t imaginary;

  PT_BEGIN(pt);

  status = GetStatus();

  while (!status.freqSweepComplete && !isSweepAborted)
  {
    // Wait until real/imag data is valid
    status = GetStatus();
    while (!status.validRealImaginaryData)
    {
      PT_YIELD_AND_EVOS_SET_DELTA(pt, 10); // Poll every 10 ms for data readiness
      status = GetStatus();
    }

    // Read out data
    ReadRealAndImaginary(&real, &imaginary);
    TRACE_VA(TRC_TA_HAL, TRACE_LEVEL_DEBUG, "ics real %i, imag %i", real, imaginary);

    status = GetStatus();
    if (sampleCb != NULL)
    {
      sampleCb(real, imaginary, status.freqSweepComplete);
    }

    // If sweep not complete, increment frequency to next point
    if (!status.freqSweepComplete)
    {
      SetFunction(AD5933_FUNC_INCREMENT_FREQ);
    }

    // Yield and resume immediately for next sample or exit condition
    PT_YIELD_AND_EVOS_SET_NOW(pt);
  }

  TRACE(TRC_TA_HAL, TRACE_LEVEL_INFO, "ics sweep complete");

  PT_END(pt);
}

/*******************************************************************************
 * Local functions, converter functions
 ******************************************************************************/

/**
 * @brief Attempts to read the AD5933 status register to confirm presence.
 *
 * @return true if read succeeds, false if I2C read fails (implying not detected).
 */
static bool Detect(void)
{
  Ad5933Reg_t reg;
  return RegisterReadOneByte(AD5933_REG_STATUS, &reg.byte);
}

/**
 * @brief Prepares the AD5933 with start frequency, frequency increment, and increment count.
 *
 * - Converts `freqStart` and `freqInc` to register values via `FreqToRegisterValue()`.
 * - Writes them sequentially into the AD5933 start freq, increment freq, and increment num regs.
 */
static void SetupFreq(void)
{
  Ad5933Setup_t setup;

  // Convert start frequency to register format
  const uint32_t start = FreqToRegisterValue(freqStart);
  setup.freqStart.low  = (uint8_t)start;
  setup.freqStart.mid  = (uint8_t)(start >> 8);
  setup.freqStart.high = (uint8_t)(start >> 16);

  // Convert increment frequency to register format
  const uint32_t inc   = FreqToRegisterValue(freqInc);
  setup.freqInc.low  = (uint8_t)inc;
  setup.freqInc.mid  = (uint8_t)(inc >> 8);
  setup.freqInc.high = (uint8_t)(inc >> 16);

  // (freqIncNum - 1) is how AD5933 encodes the number of increments
  setup.incNum.low  = (uint8_t)(freqIncNum - 1);
  setup.incNum.high = (uint8_t)((freqIncNum - 1) >> 8);

  RegisterWrite(AD5933_REG_FREQ_START, setup.bytes, sizeof(Ad5933Setup_t));
}

/**
 * @brief Writes settling time data to AD5933 registers, based on a configured struct.
 *
 * Uses a small helper (`SettlingTimeToRegisterValues`) to encode data into 2 bytes 
 * then writes them to the appropriate AD5933 register area.
 */
static void SetupSettlingTime(void)
{
  Ad5933SettlingTime_t settlingTime;
  uint8_t settlingTimeBuffer[2];

  settlingTime.NoOfCycles = Ad5933_NO_OF_CYCLES_X2;
  settlingTime.SettligCycles = 487; // Example, can adjust based on system

  if (SettlingTimeToRegisterValues(&settlingTime, settlingTimeBuffer, sizeof(settlingTimeBuffer)))
  {
    RegisterWrite(AD5933_REG_SETTLING_CYCLES, settlingTimeBuffer, sizeof(settlingTimeBuffer));
  }
}

/**
 * @brief Sends a software reset command to AD5933 by setting the reset bit in the Control Low register.
 *
 * Reads the current register value, sets the reset bit, and writes it back via I2C.
 */
static void Reset(void)
{
  Ad5933Reg_t reg;
  RegisterRead(AD5933_REG_CTRL_LOW, &reg.byte, 1);
  reg.ctrlLow.reset = true;
  RegisterWrite(AD5933_REG_CTRL_LOW, &reg.byte, 1);
}

/**
 * @brief Sets the AD5933 function bits (in Control High register) to the specified mode.
 *
 * @param func Desired functional mode, e.g., start sweep, increment frequency, etc.
 */
static void SetFunction(const Ad5933CtrlHighFunc_t func)
{
  Ad5933Reg_t reg;
  RegisterRead(AD5933_REG_CTRL_HIGH, &reg.byte, 1);
  reg.ctrlHigh.func = func;
  RegisterWrite(AD5933_REG_CTRL_HIGH, &reg.byte, 1);
}

/**
 * @brief Sets the AD5933 to PGA Gain x1 by modifying the Control High register.
 */
static void SetPgaGainX1(void)
{
  Ad5933Reg_t reg;
  RegisterRead(AD5933_REG_CTRL_HIGH, &reg.byte, 1);
  reg.ctrlHigh.pgaGain = AD5933_PGA_GAIN_X1;
  RegisterWrite(AD5933_REG_CTRL_HIGH, &reg.byte, 1);
}

/**
 * @brief Configures the AD5933 to use its internal clock source.
 */
static void SetInternalClock(void)
{
  Ad5933Reg_t reg;
  RegisterRead(AD5933_REG_CTRL_LOW, &reg.byte, 1);
  reg.ctrlLow.externalClock = false;
  RegisterWrite(AD5933_REG_CTRL_LOW, &reg.byte, 1);
}

/**
 * @brief Reads the AD5933 status register and returns a status bitfield union.
 *
 * @return Current status (temp ready, real/imag ready, sweep complete, etc.).
 */
static Ad5933Status_t GetStatus(void)
{
  Ad5933Reg_t reg;
  RegisterRead(AD5933_REG_STATUS, &reg.byte, 1);
  return reg.status;
}

/**
 * @brief Reads the real and imaginary data registers from AD5933 and converts them to signed 16-bit.
 *
 * @param[out] real      Pointer to store the real value.
 * @param[out] imaginary Pointer to store the imaginary value.
 *
 * If the read fails, both values are set to zero.
 */
static void ReadRealAndImaginary(int16_t *const real, int16_t *const imaginary)
{
  Ad5933Sample_t sample;

  if (RegisterRead(AD5933_REG_REAL_DATA, sample.bytes, sizeof(Ad5933Sample_t)))
  {
    *real = RegisterValueToNumber(&sample.real);
    *imaginary = RegisterValueToNumber(&sample.imaginary);
  }
  else
  {
    *real = 0;
    *imaginary = 0;
  }
}

/*******************************************************************************
 * Local functions, helper functions
 ******************************************************************************/

/**
 * @brief Converts frequency in Hz to the AD5933 register representation using the formula from the datasheet.
 *
 * @param hz The frequency in Hz.
 * @return 32-bit integer representing the frequency for AD5933’s register.
 *
 * Formula: 
 *    Register = ( Hz / (INTERNAL_CLOCK/4) ) * 2^27
 */
static uint32_t FreqToRegisterValue(const float_t hz)
{
  float_t calc = (hz / (INTERNAL_CLOCK / 4.0f)) * powf(2.0f, 27.0f);
  return (uint32_t)calc;
}

/**
 * @brief Converts a two-byte AD5933Number_t into a signed 16-bit integer.
 *
 * @param[in] number Pointer to the high/low byte structure.
 * @return Signed 16-bit number, where the topmost bit (bit 15) can indicate negative.
 */
static int16_t RegisterValueToNumber(const Ad5933Number_t *const number)
{
  const uint16_t value = (number->high << 8) | number->low;
  return (int16_t)value;
}

/**
 * @brief Encodes the settling cycles and multiplication factor into 2 bytes for AD5933.
 *
 * @param[in]  settlingTime  Pointer to the desired settings.
 * @param[out] buffer        2-byte buffer to hold the encoded values.
 * @param[in]  length        Must be 2 to store both bytes.
 *
 * @return true if encoding succeeded, false if the buffer size was incorrect.
 *
 * - Byte0 contains multiplication factor (shifted) and the MSB of `SettligCycles`.
 * - Byte1 contains the lower 8 bits of `SettligCycles`.
 */
static bool SettlingTimeToRegisterValues(const Ad5933SettlingTime_t *settlingTime, uint8_t *buffer, uint8_t length)
{
  if (length != 2)
  {
    return false;
  }

  buffer[0] = (settlingTime->NoOfCycles << 1);
  buffer[0] |= (settlingTime->SettligCycles >> 8 & 0x1);
  buffer[1] = settlingTime->SettligCycles & 0xFF;

  return true;
}

/*******************************************************************************
 * Local functions, I2C read/write (blocking)
 ******************************************************************************/

/**
 * @brief Writes exactly one byte of data to the specified AD5933 register via STM HAL I2C.
 *
 * @param reg  The register address to write to (e.g., 0x80).
 * @param data Pointer to the data byte.
 * @return true on success, false if I2C transaction fails.
 */
static bool RegisterWriteOneByte(const uint8_t reg, uint8_t *const data)
{
  if (HAL_OK != HAL_I2C_Mem_Write(I2C, ADDRESS, reg, 1, data, 1, I2C_TIMEOUT_MSEC))
  {
    return false;
  }
  return true;
}

/**
 * @brief Reads exactly one byte from the specified AD5933 register via STM HAL I2C.
 *
 * @param reg  The register address to read from.
 * @param data Pointer to store the read byte.
 * @return true on success, false if I2C transaction fails.
 */
static bool RegisterReadOneByte(const uint8_t reg, uint8_t *const data)
{
  if (HAL_OK != HAL_I2C_Mem_Read(I2C, ADDRESS, reg, 1, data, 1, I2C_TIMEOUT_MSEC))
  {
    return false;
  }
  return true;
}

/**
 * @brief Writes a block of bytes (one at a time) to consecutive registers on the AD5933.
 *
 * @param reg   Starting register address.
 * @param data  Pointer to the data buffer to write.
 * @param size  Number of bytes to write consecutively.
 *
 * @return true on success, false if any individual write fails.
 *
 * The AD5933 typically expects register-pointer-based writes, but we handle each byte individually for clarity.
 */
static bool RegisterWrite(const uint8_t reg, uint8_t *const data, const uint8_t size)
{
  for (uint8_t index = 0; index < size; index++)
  {
    if (!RegisterWriteOneByte(reg + index, &data[index]))
    {
      TRACE(TRC_TA_HAL, TRACE_LEVEL_ERROR, "ics wr fail");
      return false;
    }
  }
  return true;
}

/**
 * @brief Reads a block of bytes from consecutive AD5933 registers.
 *
 * @param reg   Starting register address.
 * @param data  Pointer to the buffer where read bytes will be stored.
 * @param size  Number of bytes to read consecutively.
 *
 * @return true on success, false if the I2C transaction fails at any step.
 *
 * If only 1 byte is requested, uses @ref RegisterReadOneByte for simplicity. Otherwise:
 * - Writes the address pointer to AD5933 (AD5933_I2C_ADDR_POINTER).
 * - Repeatedly receives 1 byte at a time to fill the output buffer.
 */
static bool RegisterRead(uint8_t reg, uint8_t *const data, const uint8_t size)
{
  if (size == 1)
  {
    if (!RegisterReadOneByte(reg, data))
    {
      TRACE(TRC_TA_HAL, TRACE_LEVEL_ERROR, "ics rd fail");
      return false;
    }
    else
    {
      return true;
    }
  }

  // Write the register pointer
  if (HAL_OK != HAL_I2C_Mem_Write(I2C, ADDRESS, AD5933_I2C_ADDR_POINTER, 1, &reg, 1, I2C_TIMEOUT_MSEC))
  {
    TRACE(TRC_TA_HAL, TRACE_LEVEL_ERROR, "ics ptr fail");
    return false;
  }

  // Read each byte individually
  for (uint8_t index = 0; index < size; index++)
  {
    if (HAL_OK != HAL_I2C_Master_Receive(I2C, ADDRESS, &data[index], 1, I2C_TIMEOUT_MSEC))
    {
      TRACE(TRC_TA_HAL, TRACE_LEVEL_ERROR, "ics rd fail");
      return false;
    }
  }
  return true;
}

/******************************************************************************/
