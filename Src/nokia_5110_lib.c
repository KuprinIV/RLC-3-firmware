#include "nokia_5110_lib.h"
#include "stm32f1xx.h"
#include "rlc_device.h"
#include <math.h>
#include <string.h>

#define ABS(x) (x) >= 0 ? (x):(-x)

static uint8_t DisplayBuffer[DISPLAY_WIDTH*DISPLAY_HEIGHT/8] = {0};
extern FontInfo font6x8;

/**
  * @brief  Write command to display
	* @param  cmd - command byte
  * @retval None
  */
static void Write_Cmd(uint8_t cmd)
{
	GPIOB->BRR = DC;

	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = cmd;
	while(!(SPI1->SR & SPI_SR_TXE));
	while(SPI1->SR & SPI_SR_BSY);
}

/**
  * @brief  Write data to display
	* @param  data - data byte
  * @retval None
  */
static void Write_Data(uint8_t data)
{
	GPIOB->BSRR = DC;

	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = data;
	while(!(SPI1->SR & SPI_SR_TXE));
	while(SPI1->SR & SPI_SR_BSY);
}

/**
  * @brief  Set start position to display data write
	* @param  x - x-coordinate
	* @param  y - y-coordinate
  * @retval None
  */
static void SetPos(uint8_t x, uint8_t y)
{
  Write_Cmd(0x40 | (y & 7));
  Write_Cmd(0x80 | x);
}

/**
  * @brief  Write bitmap data to framebuffer
  * @param  bmp - bitmap data array
  * @param  x - x-coordinate of top-left corner
  * @param  y - y-coordinate of top-left corner
  * @param  width - bitmap width in pixels
  * @param  height - bitmap height in pixels
  * @retval none
  */
static void Display_DrawBitmap(uint8_t* bmp, uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
	uint8_t bytes_in_col = ((height%8) == 0)?(height>>3):((height>>3) + 1);

	if(x + width > DISPLAY_WIDTH - 1) x = DISPLAY_WIDTH - 1 - width;
	if(y + height > DISPLAY_HEIGHT - 1) y = DISPLAY_HEIGHT - 1 - height;

	for(uint8_t i = 0; i < width; i++)
	{
		for(uint8_t j = 0; j < bytes_in_col; j++)
		{
			for(uint8_t k = 0; k < 8; k++)
			{
			if((bmp[j*width+i]>>k) & 0x01)
				DrawPixel(x+i, y+j*8+k);
			}
		}
	}
}

/**
  * @brief  Display init
	* @param  None
  * @retval None
  */
void Display_Init(void)
{	
	Display_Port->BSRR = RST<<16;
	HAL_Delay(5);
	Display_Port->BSRR = RST;

	GPIOA->BRR = CE;
	Write_Cmd(0x21); // extended command set
	Write_Cmd(0xB8); // LCD voltage offset
	Write_Cmd(0x04); // temperature correction 0
	Write_Cmd(0x14); // contrast offset 1:24
	Write_Cmd(0x20); // common command set
	Write_Cmd(0x0C); // normal mode
	GPIOA->BSRR = CE;
}

/**
  * @brief  Display clear
	* @param  None
  * @retval None
  */
void Display_Clear(void) 
{
	uint8_t x=0, y=0;
	GPIOA->BRR = CE;
  SetPos(0, 0);
  for (y = 0; y < DISPLAY_HEIGHT/8; y++) 
	{
    for (x = 0; x < DISPLAY_WIDTH; x++) 
		{
      Write_Data(0);
    }    
  }
  SetPos(0, 0);
	GPIOA->BSRR = CE;
}

/**
  * @brief  Write framebuffer to display controller
	* @param  None
  * @retval None
  */
void Display_Write_Buffer()
{
	GPIOA->BRR = CE;
  SetPos(0,0);
  for(uint8_t y = 0;y < DISPLAY_HEIGHT/8;y++)
	{
	  for(uint8_t x = 0;x < DISPLAY_WIDTH;x++)
	  {
	    	Write_Data(*(DisplayBuffer+x+DISPLAY_WIDTH*y));
	  }
	}
	GPIOA->BSRR = CE;
}

/**
  * @brief  Clear framebuffer
	* @param  None
  * @retval None
  */
void Display_Clear_Buffer()
{
	memset(DisplayBuffer,0,sizeof(DisplayBuffer));
}	

/**
  * @brief  Set display contrast
	* @param  contrast - contrast value from 0 to 7
  * @retval None
  */
void Display_SetContrast(uint8_t contrast)
{
	GPIOA->BRR = CE;
	Write_Cmd(0x21);
	Write_Cmd(0x10+contrast);
	Write_Cmd(0x20);
	Write_Cmd(0x0C);
	GPIOA->BSRR = CE;
}

/**
  * @brief  Put string into framebuffer
	* @param  string - string data object pointer
  * @retval None
  */
void SetStringInBuffer(String* string)
{
	uint8_t x = string->x_pos, y = string->y_pos, x_inv = 0;
	const char* pStr = string->Text;
	int Length = strlen(string->Text),LengthInv = Length, widthInPixels = 0;
	uint8_t currentCharWidth = 0;
	//calculate width of text in pixels
	const char* lStr = string->Text;
	while(LengthInv-- > 0)
	{
		 widthInPixels += string->font.descriptor[*(lStr)-' '].width + 1;
		 lStr++;
	}

	if(string->align == AlignLeft) x = x;
	if(string->align == AlignCenter) x = (DISPLAY_WIDTH - widthInPixels)/2;
	if(string->align == AlignRight) x = DISPLAY_WIDTH - 1 - widthInPixels;
	x_inv = x;

	while(Length-- > 0)
	{
		currentCharWidth = string->font.descriptor[*(pStr)-' '].width;
		Display_DrawBitmap((uint8_t*)(string->font.pFont+string->font.descriptor[*(pStr)-' '].offset), x, y+1, currentCharWidth, string->font.Height);
		x += currentCharWidth+1;
		pStr++;
	}

	if(string->inverted == Inverted)
	{
			InvertRegion(x_inv-1, y, x_inv+widthInPixels + 1, y+string->font.Height+1);
	}
}

/**
  * @brief  Put window strings into framebuffer
	* @param  wnd - window data object pointer
  * @retval None
  */
void SetWindow(pWindow wnd)
{
    for(int i = 0; i < wnd->StringsQuantity; i++)
    {
        SetStringInBuffer(&wnd->strings[i]);
    }
}

/**
  * @brief  Invert display region into framebuffer
	* @param  xn - start x-coordinate of region
	* @param  yn - start y-coordinate of region
	* @param  xk - stop x-coordinate of region
	* @param  yk - stop y-coordinate of region
  * @retval None
  */
void InvertRegion(uint8_t xn, uint8_t yn, uint8_t xk, uint8_t yk)
{
	uint8_t mask = 0;
	for(uint8_t j = yn/8;j <= yk/8; j++)
	{
		for(uint8_t i = xn; i < xk;i++)
		{
			if(j == yn/8)
			{
			  mask = 0xFF<<(yn%8);
			}
			else
		  {
			  if(j == yk/8)
			  {
				  mask = 0xFF>>(8-yk%8);
			  }
        else 
			  {
			  	mask = 0xFF;
			  }
		 }
			DisplayBuffer[DISPLAY_WIDTH*j + i] ^= mask;
		}
	}
}

/**
  * @brief  Put battery indicator image into framebuffer
	* @param  percentage - battery charge state from 0 (empty) to 10 (full)
  * @retval None 
  */
void PaintBatteryIndicator(uint8_t percentage)
{
	uint8_t BatteryBorder[12] = {0x3C,0x66,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x7E};
	// fill indicator by percentage
	for(uint8_t i = 0; i < percentage; i++)
	{
		if(i < 9)
		{
			BatteryBorder[10-i] |= 0x3C;
		}
		else
		{
			BatteryBorder[10-i] |= 0x18;
		}
	}		
	Display_DrawBitmap(BatteryBorder, 68, 0, 12, 6);
}

/**
  * @brief  Put USB indicator image into framebuffer
	* @param  None
  * @retval None 
  */
void PaintUSBIndicator()
{
	uint8_t USBIndicator[15] = {0x08,0x08,0x08,0x1C,0x3E,0x3E,0x3E,0x3E,0x3E,0x3E,0x14,0x14,0x14,0x14,0x1C};
	Display_DrawBitmap(USBIndicator, 3, 0, 15, 6);
}

/**
  * @brief  Put pixel data into framebuffer
	* @param  x - x-coordinate of pixel
	* @param  y - y-coordinate of pixel
  * @retval None 
  */
void DrawPixel(uint8_t x, uint8_t y)
{
    DisplayBuffer[DISPLAY_WIDTH*(y>>3)+x] |= 1<<(y%8);
}

/**
  * @brief  Put line data into framebuffer
	* @param  xn - start x-coordinate of line
	* @param  yn - start y-coordinate of line
	* @param  xk - stop x-coordinate of line
	* @param  yk - stop y-coordinate of line
  * @retval None 
  */
void DrawLine(int8_t xn, int8_t yn, int8_t xk, int8_t yk)
{
  int8_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(yk-yn);        /* The difference between the x's */
  deltay = ABS(xk-xn);        /* The difference between the y's */
  x = xn;                       /* Start x off at the first pixel */
  y = yn;                       /* Start y off at the first pixel */

  if (xk >= xn)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (yk >= yn)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc2 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc1 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc1 = 0;                  /* Don't change the x for every iteration */
    yinc2 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    DrawPixel(x,y); /* Draw the current pixel */
    num += numadd;                            /* Increase the numerator by the top of the fraction */
    if (num >= den)                           /* Check if numerator >= denominator */
    {
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
    }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
  }
}

/**
  * @brief  Put circle data into framebuffer
	* @param  x - x-coordinate of circle center
	* @param  y - y-coordinate of circle center
	* @param  R - circle radius in pixels
  * @retval None 
  */
void DrawCircle(uint8_t x, uint8_t y, uint8_t R)
{
     int8_t  decision;       /* Decision Variable */
     uint8_t  curx;   /* Current X Value */
     uint8_t  cury;   /* Current Y Value */

     decision = 3 - (R << 1);
     curx = 0;
     cury = R;

     while (curx <= cury)
     {
       DrawPixel((x - cury),(y + curx));
       DrawPixel((x - cury),(y - curx));
       DrawPixel((x - curx),(y + cury));
       DrawPixel((x - curx),(y - cury));
       DrawPixel((x + cury),(y + curx));
       DrawPixel((x + cury),(y - curx));
       DrawPixel((x + curx),(y + cury));
       DrawPixel((x + curx),(y - cury));

       if (decision < 0)
       {
         decision += (curx << 2) + 6;
       }
       else
       {
         decision += ((curx - cury) << 2) + 10;
         cury--;
       }
       curx++;
     }
}

/**
  * @brief  Put ellipse data into framebuffer
	* @param  x_pos - x-coordinate of ellipse center
	* @param  y_pos - y-coordinate of ellipse center
	* @param  rad_x - ellipse radius in pixels by x
	* @param  rad_y - ellipse radius in pixels by y
  * @retval None 
  */
void DrawEllipse(uint8_t x_pos, uint8_t y_pos, uint8_t rad_x, uint8_t rad_y)
{
    char x = 0, y = -rad_x, err = 2-2*rad_y, e2;
    float k = 0, rad1 = 0, rad2 = 0;

    rad1 = rad_y;
    rad2 = rad_x;

    k = (float)(rad2/rad1);

    do
    {
       DrawPixel((x_pos+y),(y_pos -(uint8_t)(x/k)));
       DrawPixel((x_pos+y),(y_pos +(uint8_t)(x/k)));
       DrawPixel((x_pos-y), (y_pos +(uint8_t)(x/k)));
       DrawPixel((x_pos-y), (y_pos -(uint8_t)(x/k)));

        e2 = err;
        if (e2 <= x) {
            err += ++x*2+1;
            if (-y == x && e2 <= y) e2 = 0;
        }
        if (e2 > y) err += ++y*2+1;
    }
    while (y <= 0);
}

/**
  * @brief  Put rectangular data into framebuffer
	* @param  x_pos - start x-coordinate of rectangular
	* @param  y_pos - start y-coordinate of rectangular
	* @param  width - rectangular width in pixels
	* @param  height - rectangular height in pixels
  * @retval None 
  */
void DrawRect(uint8_t x_pos, uint8_t y_pos, uint8_t width, uint8_t height)
{
    DrawLine(x_pos,y_pos,x_pos+width,y_pos);
    DrawLine(x_pos+width,y_pos,x_pos+width,y_pos+height);
    DrawLine(x_pos+width,y_pos+height,x_pos,y_pos+height);
    DrawLine(x_pos,y_pos+height,x_pos,y_pos);
}

/**
  * @brief  Put filled rectangular data into framebuffer
	* @param  x_pos - start x-coordinate of rectangular
	* @param  y_pos - start y-coordinate of rectangular
	* @param  width - rectangular width in pixels
	* @param  height - rectangular height in pixels
  * @retval None 
  */
void FillRect(uint8_t x_pos, uint8_t y_pos, uint8_t width, uint8_t height)
{
    for(;height>0;height--)
    {
       DrawLine(x_pos,y_pos,x_pos+width-1,y_pos);
       y_pos++;
    }
}

/**
  * @brief  Put filled circle data into framebuffer
	* @param  Xpos - x-coordinate of circle center
	* @param  Ypos - y-coordinate of circle center
	* @param  Radius - circle radius in pixels
  * @retval None 
  */
void FillCircle(uint8_t Xpos, uint8_t Ypos, uint8_t Radius)
{
  int8_t  decision;        /* Decision Variable */
  uint8_t  curx;    /* Current X Value */
  uint8_t  cury;    /* Current Y Value */

  decision = 3 - (Radius << 1);

  curx = 0;
  cury = Radius;

  while (curx <= cury)
  {
    if(cury > 0)
    {
      DrawLine(Xpos + curx, Ypos - cury,Xpos + curx, Ypos + cury);
      DrawLine(Xpos - curx, Ypos - cury,Xpos - curx, Ypos + cury);
    }

    if(curx > 0)
    {
      DrawLine(Xpos - cury, Ypos - curx,Xpos - cury, Ypos + curx);
      DrawLine(Xpos + cury, Ypos - curx,Xpos + cury, Ypos + curx);
    }
    if (decision < 0)
    {
      decision += (curx << 2) + 6;
    }
    else
    {
      decision += ((curx - cury) << 2) + 10;
      cury--;
    }
    curx++;
  }

  DrawCircle(Xpos, Ypos, Radius);
}

/**
  * @brief  Put filled ellipse data into framebuffer
	* @param  Xpos - x-coordinate of ellipse center
	* @param  Ypos - y-coordinate of ellipse center
	* @param  XRadius - ellipse radius in pixels by x
	* @param  YRadius - ellipse radius in pixels by y
  * @retval None 
  */
void FillEllipse(uint8_t Xpos, uint8_t Ypos, uint8_t XRadius, uint8_t YRadius)
{
  char x = 0, y = -XRadius, err = 2-2*YRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;

  rad1 = YRadius;
  rad2 = XRadius;

  k = (float)(rad2/rad1);

  do
  {
    DrawLine((Xpos+y), (Ypos-(uint8_t)(x/k)),(Xpos+y), (Ypos + (uint8_t)(x/k) + 1));
    DrawLine((Xpos-y), (Ypos-(uint8_t)(x/k)),(Xpos-y), (Ypos + (uint8_t)(x/k) + 1));

    e2 = err;
    if (e2 <= x)
    {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}
