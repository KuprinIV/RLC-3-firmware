#ifndef __RLC_DEVICE_H
#define __RLC_DEVICE_H

#include "rlc.h"

//#define USE_INTERNAL_ADC 1

#define FIRMWARE_VERSION_STR			"1.1"
#define RELEASE_DATE_STR					"16.11.24"
#define DFU_SIGNATURE							0x3DC23DC2
#define DFU_SIGNATURE_ADDRESS			0x0800F800

typedef struct
{
	uint8_t onDataReceived;
	uint8_t onDisplayRedraw;
	uint8_t onBatteryParamDataConverted;
	uint8_t onUsbPlugEvent;
}RLC_Events;

#define DC  						(GPIO_PIN_4)
#define CE  						(GPIO_PIN_15)
#define RST  						(GPIO_PIN_6)
#define Display_Port 		GPIOB

void RLCDEV_ChargerCtrl(uint8_t);
void RLCDEV_EnableUSB_PullUp(uint8_t);
void RLCDEV_PowerCtrl(uint8_t);
void RLCDEV_BacklightCtrl(uint8_t is_enabled);
void RLCDEV_GetBatteryParameters(pData, uint8_t*);
void RLCDEV_ResetDfuSignature(void);

void RLCDEV_StartADCRegularConv(void);
void RLCDEV_StopADCRegularConv(void);
void RLCDEV_StartADCInjectedConv(void);

void RLCDEV_AD7980_GetData(void);

#endif
