#ifndef HAL_ICS_H
#define HAL_ICS_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define AD5933_IMPEDANCE_RANGE_MIN  1000
#define AD5933_IMPEDANCE_RANGE_MAX  10000000
#define AD5933_FREQUENCY_RANGE_MIN  1000
#define AD5933_FREQUENCY_RANGE_MAX  100000

/*******************************************************************************
 * Type definitions
 ******************************************************************************/

typedef void (*HalIcsSweepSampleCb_t)(int16_t real, int16_t imaginary, bool isLast);

/*******************************************************************************
 * Functions
 ******************************************************************************/

// AD5933 Impedance Converter System

void HalIcsInit(void);
bool HalIcsIsDetected(void);
bool HalIcsIsIdle(void);

void HalIcsSetStartFreq(float_t hz);
float_t HalIcsGetStartFreq(void);
void HalIcsSetFreqInc(float_t hz);
float_t HalIcsGetFreqInc(void);
void HalIcsSetNoOfSamples(uint16_t number);
uint16_t HalIcsGetNoOfSamples(void);

// The callback returns a sample with the frequency, real and imaginary numbers.
// Returns true if more samples are in bound and false if last sample of the sweep.
bool HalIcsStartFreqSweep(HalIcsSweepSampleCb_t cb);
void HalIcsAbortFreqSweep(void);

/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
