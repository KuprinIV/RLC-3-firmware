#ifndef __DDS__H__
#define __DDS__H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

#define MCLK 8000000.0f

void DDS_Init(void);
void signalOn(uint8_t state);
void setSignalType(uint8_t index);
void setDDSFrequency(int freq);
void setPhase(int phase);

static void reset(uint8_t state);
static void writeRegister(uint16_t reg);

#endif
