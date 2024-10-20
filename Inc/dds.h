#ifndef __DDS__H__
#define __DDS__H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

#define MCLK 8000000.0f

void DDS_Init(void);
void DDS_SignalOn(uint8_t state);
void DDS_SetSignalType(uint8_t index);
void DDS_SetFrequency(int freq);
void DDS_SetPhase(int phase);

static void DDS_Reset(uint8_t state);
static void DDS_WriteRegister(uint16_t reg);

#endif
