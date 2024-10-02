#ifndef __NOKIA_5110_LIB_H
#define __NOKIA_5110_LIB_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "font.h"
#include "rlc.h"

#define DC  (GPIO_PIN_4)
#define CE  (GPIO_PIN_15)
#define RST  (GPIO_PIN_6)
#define Display_Port GPIOB

typedef enum
{
	AlignLeft,
	AlignCenter,
	AlignRight,
}Align;

typedef enum
{
	Inverted,
	NotInverted,
}IsInverted;

typedef enum {Prev,Next,NoAction} Action;

//typedef int (*WindowCallback)(void* ,void* ,Action , Action );

typedef struct 
{
   uint8_t x_pos;
   uint8_t y_pos;
   Align align;
   FontInfo font;
   const char* Text;
   IsInverted inverted;
}String,*pString;

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

typedef struct Window
{
  String strings[7];
  uint8_t StringsQuantity;
  int (*callback)(struct Window* ,pData , Action ,Action );
  struct Window* next;
  struct Window* prev;
}Window,*pWindow;

void Write_Cmd(uint8_t cmd); //?????? ???????
void Write_Data(uint8_t data); //?????? ??????
void Display_Init(void); //????????????? ???????
void SetPos(uint8_t x, uint8_t y); //????????? ??????? ??? ?????? ??????
void Display_Clear(void); //??????? ???????
void Write_Buffer(void);
void Clear_Buffer(void);
void PaintBatteryIndicator(uint8_t percentage);
void PaintUSBIndicator(void);

void SetStringInBuffer(String* string);
void SetWindow(pWindow wnd);
void InvertRegion(uint8_t xn, uint8_t yn, uint8_t xk, uint8_t yk);

void DrawPixel(uint8_t x, uint8_t y);
void DrawLine(int8_t xn, int8_t yn, int8_t xk, int8_t yk);
void DrawCircle(uint8_t x, uint8_t y, uint8_t R);
void DrawEllipse(uint8_t x_pos, uint8_t y_pos, uint8_t rad_x, uint8_t rad_y);
void DrawRect(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height);
void FillRect(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height);
void FillCircle(uint8_t Xpos, uint8_t Ypos, uint8_t Radius);
void FillEllipse(uint8_t Xpos, uint8_t Ypos, uint8_t XRadius, uint8_t YRadius);
#endif 
