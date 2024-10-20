#ifndef __RLC__H
#define __RLC__H

#include "stm32f1xx_hal.h"
#include "font.h"
#include "dds.h"
#include "complex_numbers.h"

#define M_PI 3.14159265359f
#define NUM_SAMPLES 4096
#define CALIBRATION_DATA_ADDR 0x0800FC00
#define ADC_OVERSCALE					55000
#define ADC_MAX_AMPLITUDE			50000

#define USB_ON() (GPIOA->IDR&GPIO_PIN_3) // detecting USB
#define IS_LEFT_BUTTON_PRESSED() (GPIOA->IDR&GPIO_PIN_6)
#define IS_CENTER_BUTTON_PRESSED() (GPIOA->IDR&GPIO_PIN_9)
#define IS_RIGHT_BUTTON_PRESSED() (GPIOA->IDR&GPIO_PIN_7)
#define IS_PWR_BUTTON_PRESSED() (GPIOA->IDR & GPIO_PIN_9)

typedef struct
{
	uint8_t testSignalFreq; // 0 - 120 Hz, 1 - 1 kHz, 2 - 8 kHz, 3 - 62,5 kHz
	uint8_t R_sense; // 0 - 10 Ohm, 1 - 100 Ohm, 2 - 1 kOhm, 3 - 10 kOhm, 4 - 100 kOhm
	uint8_t uGain; // 0 - 1, 1 - 5, 2 - 13.2, 3 - 34
	uint8_t isParamsUpdated; // 0 - not updated, 0x01 - freq updated, 0x02 - amp updated, 0x04 - R sense updated, 0x08 - uGain updated, 0x10 - send ADC data through USB 
	uint8_t isAutoSet; //0 - manual/usb set, 1 - autoset
	uint8_t measureType; // 0 - autoset, 1 - R, 2 - L, 3 - C, 4 - resettable R
}measureParams, *pMeasureParams;

typedef struct
{
    pMeasureParams param_vals;
    uint8_t display_vals[3];
    float batADC_data[3];
    float R;
    float X;
    float Z;
		float Ur;
		float Ux;
		float fi;
	  uint8_t current_item;
		uint8_t rsrp;
		uint8_t is_calibration_started;
}Data,*pData;

typedef enum
{
	VoltageData,
	CurrentData
}MeasureValue;

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

void RLC_Init(void);
void RLC_SetAutoSetParams(uint8_t);
void RLC_SetFrequency(uint8_t);
void RLC_SetRsense(uint8_t);
void RLC_SetUGain(uint8_t);
void RLC_SetMeasureType(uint8_t);
void RLC_SetParameters(uint16_t, uint16_t, float);
void RLC_AnalyzeInputData(uint16_t* data, uint16_t len, uint8_t freq_index, uint16_t* adc_amplitude, ComplexNumber* cn);

uint8_t RLC_GetFrequencyIndex(void);
float 	RLC_GetFrequencyValue(void);
uint8_t RLC_GetRSenseIndex(void);
float 	RLC_GetRSenseValue(void);
uint8_t RLC_GetMeasureType(void);

void RLC_WriteCalibrationDataToFlash(void);
void RLC_ReadCalibrationDataFromFlash(void);
#endif
