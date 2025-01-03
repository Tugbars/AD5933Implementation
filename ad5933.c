/*******************************************************************************
 * @file    ics.c
 * @brief   Implementation of the AD5933 impedance converter / network analyzer.
 *
 * This module provides an interface to the AD5933 chip. It initializes the chip,
 * configures frequency sweep parameters, starts a sweep, reads back samples, and
 * allows aborting any ongoing sweep. It also contains helper functions to
 * perform I2C read/write operations in a blocking manner. 
 *
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/


#include <i2c.h>
#include <stdio.h>   // Added for printf
#include <stdarg.h>  // Added for variadic functions

#include "ics.h"
#include "pt.h"
#include "evos.h"

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

/// @brief AD5933 I2C block commands (not used directly in this implementation).
#define AD5933_I2C_BLOCK_WRITE    0xA0
#define AD5933_I2C_BLOCK_READ     0xA1
#define AD5933_I2C_ADDR_POINTER   0xB0

/*******************************************************************************
 * Type definitions
 ******************************************************************************/

typedef struct
{
  uint8_t pgaGain : 1;
  uint8_t outputVoltageRange : 2;
  uint8_t noOperation : 1;
  uint8_t func : 4;
} Ad5933CtrlHigh_t;

typedef enum
{
  AD5933_PGA_GAIN_X5,
  AD5933_PGA_GAIN_X1
} Ad5933CtrlHighPgaGain_t;

typedef enum
{
  Ad5933_NO_OF_CYCLES_DEFAULT = 0,
  Ad5933_NO_OF_CYCLES_X2 = 1,
  Ad5933_NO_OF_CYCLES_X4 = 3,
} Ad5933NoOfCycles;

typedef struct
{
  Ad5933NoOfCycles NoOfCycles;
  uint16_t SettligCycles;
} Ad5933SettlingTime_t;

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

typedef struct
{
  uint8_t reserved1 : 3;
  uint8_t externalClock : 1;
  uint8_t reset : 1;
  uint8_t reserved2 : 3;
} Ad5933CtrlLow_t;

typedef struct
{
  uint8_t validTempMeasurement : 1;
  uint8_t validRealImaginaryData : 1;
  uint8_t freqSweepComplete : 1;
  uint8_t reserved : 5;
} Ad5933Status_t;

typedef union
{
  uint8_t byte;
  Ad5933CtrlHigh_t ctrlHigh;
  Ad5933CtrlLow_t ctrlLow;
  Ad5933Status_t status;
} Ad5933Reg_t;

typedef struct __attribute__((packed))
{
  uint8_t high;
  uint8_t mid;
  uint8_t low;
} Ad5933StartFreq_t, Ad5933FreqInc_t;

typedef struct __attribute__((packed))
{
  uint8_t high;
  uint8_t low;
} Ad5933IncNum_t, Ad5933Number_t;

typedef union __attribute__((packed))
{
  uint8_t bytes[sizeof(Ad5933StartFreq_t) + sizeof(Ad5933FreqInc_t) + sizeof(Ad5933IncNum_t)];
  struct
  {
    Ad5933StartFreq_t freqStart;
    Ad5933FreqInc_t freqInc;
    Ad5933IncNum_t incNum;
  };
} Ad5933Setup_t;

typedef union __attribute__((packed))
{
  uint8_t bytes[sizeof(Ad5933Number_t) * 2];
  struct
  {
    Ad5933Number_t real;
    Ad5933Number_t imaginary;
  };
} Ad5933Sample_t;

/*******************************************************************************
 * Local constant variables
 ******************************************************************************/

#if defined(EVK)
static I2C_HandleTypeDef *const I2C = &hi2c3;
#else
static I2C_HandleTypeDef *const I2C = &hi2c1;
#endif

// I2C driver interface requires 7-bit device address shifted one left
static const uint8_t ADDRESS = AD5933_7BIT_ADDRESS << 1;

static const uint32_t I2C_TIMEOUT_MSEC = 1;

static const float_t INTERNAL_CLOCK = 16776000.0f;
static const float_t FREQ_START = 10000.0f;
static const float_t FREQ_INC = 1.0f;
static const uint16_t INC_NUM = 501;

// Nothing about power up wait timing found in the datasheet. Wait is added just
// to be sure.
static const PlfTime_t POWER_UP_WAIT_MSEC = 20;

// Define trace levels
typedef enum {
    TRACE_LEVEL_NONE = 0,  // No tracing
    TRACE_LEVEL_ERROR = 1, // Critical errors
    TRACE_LEVEL_INFO = 2,  // General information
    TRACE_LEVEL_DEBUG = 3, // Debugging information
    TRACE_LEVEL_VERBOSE = 4 // Detailed verbose output
} TraceLevelEnum_t;

// Set the current trace level
static const TraceLevel_t CURRENT_TRACE_LEVEL = TRACE_LEVEL_DEBUG; // Adjust as needed

// Define trace categories if necessary
typedef enum {
    TRC_TA_HAL = 1,
    // Add other categories as needed
} TraceCategory_t;

// Function to handle formatted printing based on trace level
void Trace_Print(TraceCategory_t category, TraceLevel_t level, const char *format, ...) {
    if (level <= CURRENT_TRACE_LEVEL) {
        // Optionally, include category information
        // printf("[Category %d] ", category);
        
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        
        printf("\n"); // Add newline for readability
    }
}

// Redefine TRACE and TRACE_VA macros
#define TRACE(category, level, ...) \
    Trace_Print(category, level, __VA_ARGS__)

#define TRACE_VA(category, level, format, ...) \
    Trace_Print(category, level, format, __VA_ARGS__)

// Example trace level definitions
static const TraceLevel_t TL_ICS = TRACE_LEVEL_DEBUG;       // Originally TRC_TL_4
static const TraceLevel_t TL_ICS_I2C = TRACE_LEVEL_DEBUG;  // Originally TRC_TL_4

/*******************************************************************************
 * Local data variables
 ******************************************************************************/

static EvosEventHandle_t drvHandle = EVOS_UNINITIALIZED_HANDLE;
static PThread_t drvThread;

static bool isDetected = false;
static bool isIdle = false;
static bool isSweepAborted = false;

static HalIcsSweepSampleCb_t sampleCb = NULL;

static float_t freqStart;
static float_t freqInc;
static uint16_t freqIncNum;

/*******************************************************************************
 * Macros
 ******************************************************************************/

#define PT_YIELD_AND_EVOS_SET_NOW(pt) \
  EvosEventSetNow(drvHandle, 0);      \
  PT_YIELD((pt))

#define PT_YIELD_AND_EVOS_SET_DELTA(pt, msec) \
  EvosEventSetDelta2(drvHandle, (msec), 0);   \
  PT_YIELD((pt))

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

// Driver flow
static void DriverOnEvent(EvosEventParam_t param);
static PT_THREAD(DriverUpdate(void));
static PT_THREAD(ReadSamples(PThread_t *pt));

// Converter functions
static bool Detect(void);
static void SetupFreq(void);
static void Reset(void);
static void SetFunction(Ad5933CtrlHighFunc_t func);
static void SetPgaGainX1(void);
static void SetInternalClock(void);
static Ad5933Status_t GetStatus(void);
static void ReadRealAndImaginary(int16_t *real, int16_t *imaginary);

// Helper functions
static uint32_t FreqToRegisterValue(float_t hz);
static int16_t RegisterValueToNumber(const Ad5933Number_t *number);
static bool SettlingTimeToRegisterValues(const Ad5933SettlingTime_t *settlingTime, uint8_t *buffer, uint8_t length);
static void SetupSettlingTime(void);

// I2C read/write (blocking)
static bool RegisterWriteOneByte(uint8_t reg, uint8_t *data);
static bool RegisterReadOneByte(uint8_t reg, uint8_t *data);
static bool RegisterWrite(uint8_t reg, uint8_t *data, uint8_t size);
static bool RegisterRead(uint8_t reg, uint8_t *data, uint8_t size);

/*******************************************************************************
 * Public functions
 ******************************************************************************/

void HalIcsInit(void)
{
  TRACE(TRC_TA_HAL, TRACE_LEVEL_INFO, "HalIcsInit()");

  freqStart = FREQ_START;
  freqInc = FREQ_INC;
  freqIncNum = INC_NUM;

  PT_INIT(&drvThread);
  drvHandle = EvosEventRegister(DriverOnEvent, "hal ics");
  EvosEventSetDelta2(drvHandle, POWER_UP_WAIT_MSEC, 0);
}

bool HalIcsIsDetected(void)
{
  return isDetected;
}

bool HalIcsIsIdle(void)
{
  return isIdle;
}

void HalIcsSetStartFreq(const float_t hz)
{
  freqStart = hz;
}

float_t HalIcsGetStartFreq(void)
{
  return freqStart;
}

void HalIcsSetFreqInc(const float_t hz)
{
  freqInc = hz;
}

float_t HalIcsGetFreqInc(void)
{
  return freqInc;
}

void HalIcsSetNoOfSamples(const uint16_t number)
{
  freqIncNum = number;
}

uint16_t HalIcsGetNoOfSamples(void)
{
  return freqIncNum;
}

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

static void DriverOnEvent(const EvosEventParam_t param)
{
  DriverUpdate();
}

static PT_THREAD(DriverUpdate(void))
{
  static PThread_t child;

  PT_BEGIN(&drvThread);

  if (Detect())
  {
    isDetected = true;
  }
  else
  {
    TRACE(TRC_TA_HAL, TRACE_LEVEL_ERROR, "ics not detected");
  }
  PT_WAIT_UNTIL(&drvThread, isDetected);

  SetInternalClock();
  SetPgaGainX1();

  SetFunction(AD5933_FUNC_POWER_DOWN_MODE);
  Reset();
  isIdle = true;

  while (true)
  {

    // Idle is cleared when a sweep is requested by external software
    if (!isIdle)
    {

      TRACE(TRC_TA_HAL, TL_ICS, "ics prepare freq sweep");

      SetupFreq();
      SetupSettlingTime();
      SetFunction(AD5933_FUNC_STANDBY_MODE);
      Reset();

      SetFunction(AD5933_FUNC_INIT_WITH_START_FREQ);
      PT_YIELD_AND_EVOS_SET_NOW(&drvThread);

      TRACE(TRC_TA_HAL, TL_ICS, "ics start freq sweep");

      SetFunction(AD5933_FUNC_START_FREQ_SWEEP);

      PT_SPAWN(&drvThread, &child, ReadSamples(&child));

      SetFunction(AD5933_FUNC_POWER_DOWN_MODE);
      Reset();
      isIdle = true;
    }

    PT_YIELD(&drvThread);
  }

  PT_END(&drvThread);
}

static PT_THREAD(ReadSamples(PThread_t *const pt))
{
  static Ad5933Status_t status;
  static int16_t real;
  static int16_t imaginary;

  PT_BEGIN(pt);

  status = GetStatus();

  while (!status.freqSweepComplete && !isSweepAborted)
  {

    status = GetStatus();
    while (!status.validRealImaginaryData)
    {
      PT_YIELD_AND_EVOS_SET_DELTA(pt, 10);
      status = GetStatus();
    }

    ReadRealAndImaginary(&real, &imaginary);
    TRACE_VA(TRC_TA_HAL, TRACE_LEVEL_DEBUG, "ics real %i, imag %i", real, imaginary);

    status = GetStatus();
    if (sampleCb != NULL)
    {
      sampleCb(real, imaginary, status.freqSweepComplete);
    }
    if (!status.freqSweepComplete)
    {
      SetFunction(AD5933_FUNC_INCREMENT_FREQ);
    }

    PT_YIELD_AND_EVOS_SET_NOW(pt);
  }

  TRACE(TRC_TA_HAL, TRACE_LEVEL_INFO, "ics sweep complete");

  PT_END(pt);
}

/*******************************************************************************
 * Local functions, converter functions
 ******************************************************************************/

static bool Detect(void)
{
  Ad5933Reg_t reg;
  return RegisterReadOneByte(AD5933_REG_STATUS, &reg.byte);
}

static void SetupFreq(void)
{
  Ad5933Setup_t setup;

  // Convert start frequency to register value
  const uint32_t start = FreqToRegisterValue(freqStart);
  setup.freqStart.low  = (uint8_t)start;
  setup.freqStart.mid  = (uint8_t)(start >> 8);
  setup.freqStart.high = (uint8_t)(start >> 16);

  // Convert frequency increment to register value
  const uint32_t inc   = FreqToRegisterValue(freqInc);
  setup.freqInc.low  = (uint8_t)inc;
  setup.freqInc.mid  = (uint8_t)(inc >> 8);
  setup.freqInc.high = (uint8_t)(inc >> 16);

  // Number of increments is stored as (freqIncNum - 1) for the AD5933
  setup.incNum.low  = (uint8_t)(freqIncNum - 1);
  setup.incNum.high = (uint8_t)((freqIncNum - 1) >> 8);

  // Write the entire structure in consecutive registers
  RegisterWrite(AD5933_REG_FREQ_START, setup.bytes, sizeof(Ad5933Setup_t));
}

static void SetupSettlingTime(void)
{
  Ad5933SettlingTime_t settlingTime;
  uint8_t settlingTimeBuffer[2];

  settlingTime.NoOfCycles = Ad5933_NO_OF_CYCLES_X2; // Ad5933_NO_OF_CYCLES_X4
  settlingTime.SettligCycles = 487;                 // 200

  if (SettlingTimeToRegisterValues(&settlingTime, settlingTimeBuffer, sizeof(settlingTimeBuffer)))
  {
    RegisterWrite(AD5933_REG_SETTLING_CYCLES, settlingTimeBuffer, sizeof(settlingTimeBuffer));
  }
}

static void Reset(void)
{
  Ad5933Reg_t reg;
  RegisterRead(AD5933_REG_CTRL_LOW, &reg.byte, 1);
  reg.ctrlLow.reset = true;
  RegisterWrite(AD5933_REG_CTRL_LOW, &reg.byte, 1);
}

static void SetFunction(const Ad5933CtrlHighFunc_t func)
{
  Ad5933Reg_t reg;
  RegisterRead(AD5933_REG_CTRL_HIGH, &reg.byte, 1);
  reg.ctrlHigh.func = func;
  RegisterWrite(AD5933_REG_CTRL_HIGH, &reg.byte, 1);
}

static void SetPgaGainX1(void)
{
  Ad5933Reg_t reg;
  RegisterRead(AD5933_REG_CTRL_HIGH, &reg.byte, 1);
  reg.ctrlHigh.pgaGain = AD5933_PGA_GAIN_X1;
  RegisterWrite(AD5933_REG_CTRL_HIGH, &reg.byte, 1);
}

static void SetInternalClock(void)
{
  Ad5933Reg_t reg;
  RegisterRead(AD5933_REG_CTRL_LOW, &reg.byte, 1);
  reg.ctrlLow.externalClock = false;
  RegisterWrite(AD5933_REG_CTRL_LOW, &reg.byte, 1);
}

static Ad5933Status_t GetStatus(void)
{
  Ad5933Reg_t reg;
  RegisterRead(AD5933_REG_STATUS, &reg.byte, 1);
  return reg.status;
}

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

static uint32_t FreqToRegisterValue(const float_t hz)
{
  // Formula from datasheet. Used for both the start frequency and the
  // incremental frequency.
  //
  //                      FREQUENCY
  // Register value = ------------------ * 2^27
  //                   SYSTEM_CLOCK / 4

  float_t calc = (hz / (INTERNAL_CLOCK / 4.0f)) * powf(2.0f, 27.0f);
  return (uint32_t)calc;
}

static int16_t RegisterValueToNumber(const Ad5933Number_t *const number)
{
  const uint16_t value = (number->high << 8) | number->low;
  return (int16_t)value;
}

static bool SettlingTimeToRegisterValues(const Ad5933SettlingTime_t *settlingTime, uint8_t *buffer, uint8_t length)
{
  if (length != 2)
  {
    return false;
  }

  buffer[0] = settlingTime->NoOfCycles << 1;

  // Get the MSB of the 9 bit SettligCycles number
  buffer[0] |= (settlingTime->SettligCycles >> 8 & 0x1);
  // Get the last 8 bit of the 9 bit SettligCycles number.
  buffer[1] = settlingTime->SettligCycles & 0xFF;

  return true;
}

/*******************************************************************************
 * Local functions, I2C read/write (blocking)
 ******************************************************************************/

// I2C only works writing or reading one register byte and one data byte at a
// time. The AD5933 describes setting a register pointer and writing/reading a
// data block. All these I2C reads/writes failed because more than 1 byte data
// size.

static bool RegisterWriteOneByte(const uint8_t reg, uint8_t *const data)
{
  if (HAL_OK != HAL_I2C_Mem_Write(I2C, ADDRESS, reg, 1, data, 1, I2C_TIMEOUT_MSEC))
  {
    return false;
  }
  return true;
}

static bool RegisterReadOneByte(const uint8_t reg, uint8_t *const data)
{
  if (HAL_OK != HAL_I2C_Mem_Read(I2C, ADDRESS, reg, 1, data, 1, I2C_TIMEOUT_MSEC))
  {
    return false;
  }
  return true;
}

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

  if (HAL_OK != HAL_I2C_Mem_Write(I2C, ADDRESS, AD5933_I2C_ADDR_POINTER, 1, &reg, 1, I2C_TIMEOUT_MSEC))
  {
    TRACE(TRC_TA_HAL, TRACE_LEVEL_ERROR, "ics ptr fail");
    return false;
  }

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
