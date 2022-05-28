#ifndef __RLC__H
#define __RLC__H

#include "stm32f1xx_hal.h"
#include "font.h"
#include "dds.h"
#include "complex_numbers.h"

#define M_PI 3.14159265359f
//#define USE_INTERNAL_ADC 1
#define NUM_SAMPLES 4096
#define CALIBRATION_DATA_ADDR 0x0800FC00

#define USB_ON() (GPIOA->IDR&GPIO_PIN_3) // detecting USB
#define IS_LEFT_BUTTON_PRESSED (GPIOA->IDR&GPIO_PIN_6)
#define IS_CENTER_BUTTON_PRESSED (GPIOA->IDR&GPIO_PIN_9)
#define IS_RIGHT_BUTTON_PRESSED (GPIOA->IDR&GPIO_PIN_7)
#define IS_PWR_BUTTON_PRESSED (GPIOA->IDR & GPIO_PIN_9)

typedef struct
{
	uint8_t testSignalFreq; // 0 - 120 Hz, 1 - 1 kHz, 2 - 8 kHz, 3 - 62,5 kHz
	uint8_t R_sense; // 0 - 100 Ohm, 1 - 1 kOhm, 2 - 10 kOhm, 3 - 100 kOhm, 4 - 10 Ohm
	uint8_t uGain; // 0 - 1, 1 - 5, 2 - 13.2, 3 - 34
	uint8_t isParamsUpdated; // 0 - not updated, 0x01 - freq updated, 0x02 - amp updated, 0x04 - R sense updated, 0x08 - uGain updated, 0x10 - send ADC data throw USB 
	uint8_t isAutoSet; //0 - manual/usb set, 1 - autoset
	uint8_t measureType; // 0 - autoset, 1 - R, 2 - L, 3 - C
}measureParams, *pMeasureParams;

typedef enum
{
	VoltageData,
	CurrentData
}MeasureValue;

typedef struct
{
	uint8_t onDataReceived;
	uint8_t onDisplayRedraw;
}RLC_Events;

typedef struct
{
	uint8_t isStable;
	uint8_t stabCount;
	uint8_t stabCountLimit;
}Stabilization;

typedef struct 
{
	ComplexNumber Zo[4];
	ComplexNumber Zc[4];
	uint8_t isCalibrated;	
}CalibrationVals;

uint8_t getFrequency(void);
uint8_t getRSense(void);
uint8_t getUGain(void);
uint8_t getMeasureType(void);

void setAutoSetParams(uint8_t);
void setFrequency(uint8_t);
void setRsense(uint8_t);
void setUGain(uint8_t);
void setMeasureType(uint8_t);
static void setRs(float Z);
void setParameters(float, float, float);

void chargerCtrl(uint8_t);
void enableUSB_PullUp(uint8_t);
void powerCtrl(uint8_t);

void startADCRegularConv(void);
void stopADCRegularConv(void);
void startADCInjectedConv(void);

void updateMeasureParams(void);

void WriteCalibrationDataToFlash(void);
void ReadCalibrationDataFromFlash(void);
void ReadDisplaySettings(void);

uint16_t AD7980_Conversion(void);

void LightEnable(void);
void LightDisable(void);
#endif
