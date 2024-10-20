#ifndef __NOKIA_5110_LIB_H
#define __NOKIA_5110_LIB_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "font.h"
#include "rlc.h"

#define DISPLAY_WIDTH			84
#define DISPLAY_HEIGHT		48

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

typedef struct 
{
   uint8_t x_pos;
   uint8_t y_pos;
   Align align;
   FontInfo font;
   const char* Text;
   IsInverted inverted;
}String,*pString;

typedef struct Window
{
  String strings[7];
  uint8_t StringsQuantity;
  int (*callback)(struct Window* ,pData , Action ,Action );
  struct Window* next;
  struct Window* prev;
}Window,*pWindow;

// LCD control
void Display_Init(void);
void Display_Clear(void);
void Display_Write_Buffer(void);
void Display_Clear_Buffer(void);
void Display_SetContrast(uint8_t contrast);

// LCD primitives drawing
void SetStringInBuffer(String* string);
void SetWindow(pWindow wnd);
void InvertRegion(uint8_t xn, uint8_t yn, uint8_t xk, uint8_t yk);

void PaintBatteryIndicator(uint8_t percentage);
void PaintUSBIndicator(void);
void DrawPixel(uint8_t x, uint8_t y);
void DrawLine(int8_t xn, int8_t yn, int8_t xk, int8_t yk);
void DrawCircle(uint8_t x, uint8_t y, uint8_t R);
void DrawEllipse(uint8_t x_pos, uint8_t y_pos, uint8_t rad_x, uint8_t rad_y);
void DrawRect(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height);
void FillRect(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height);
void FillCircle(uint8_t Xpos, uint8_t Ypos, uint8_t Radius);
void FillEllipse(uint8_t Xpos, uint8_t Ypos, uint8_t XRadius, uint8_t YRadius);
#endif 
