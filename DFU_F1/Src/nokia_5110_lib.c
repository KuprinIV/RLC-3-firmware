#include "nokia_5110_lib.h"
#include "stm32f1xx.h"
#include <math.h>
#include <string.h>

#define ABS(x) (x) >= 0 ? (x):(-x)

static unsigned char DisplayBuffer[84*6] = {0};
uint8_t character = 0;
extern FontInfo font6x8;

void Write_Cmd(uint8_t cmd)
{
	GPIOB->BRR = DC;
	//GPIOA->BRR = CE;
		while(!(SPI1->SR & SPI_SR_TXE));
		SPI1->DR = cmd;
		while(!(SPI1->SR & SPI_SR_TXE));
		while(SPI1->SR & SPI_SR_BSY);
	//GPIOA->BSRR = CE;
}

void Write_Data(uint8_t data)
{
	GPIOB->BSRR = DC;
	//GPIOA->BRR = CE;
		while(!(SPI1->SR & SPI_SR_TXE));
		SPI1->DR = data;
		while(!(SPI1->SR & SPI_SR_TXE));
		while(SPI1->SR & SPI_SR_BSY);
	//GPIOA->BSRR = CE;
}

void Display_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	 
	/* SPI SCK, MOSI and DC, RST, CS0 GPIO pin configuration  */
	AFIO->MAPR |= AFIO_MAPR_SPI1_REMAP;
	
	GPIOA->CRH &= 0x0FFFFFFF;
	GPIOA->CRH |= 0x20000000;

	GPIOB->CRL &= 0xF0000FFF;
	GPIOB->CRL |= 0x02B2B000;
	
	GPIOA->BSRR = CE;
	
	//SPI init 
	SPI1->CR1 |= SPI_CR1_BR_2|SPI_CR1_BR_1; // fpclk/4
	SPI1->CR1 |= SPI_CR1_BIDIMODE|SPI_CR1_BIDIOE|SPI_CR1_SSM; // 8-bit
	SPI1->CR1 |= SPI_CR1_SSI;
	SPI1->CR1 |= SPI_CR1_MSTR; // spi master 
	SPI1->CR1 |= SPI_CR1_SPE;
	
	Display_Port->BSRR = RST<<16;
	HAL_Delay(5);
	Display_Port->BSRR = RST;

	GPIOA->BRR = CE;
	Write_Cmd(0x21);//??????????? ????? ??????
	Write_Cmd(0xB8);//?????????? ????????
	Write_Cmd(0x04); //????? ????????????? ????????? 
	Write_Cmd(0x14); //???????? 1:24
	Write_Cmd(0x20); //??????? ????? ??????
	Write_Cmd(0x0C); //?????????? ???????????
	GPIOA->BSRR = CE;
}

void SetPos(uint8_t x, uint8_t y)
{
  Write_Cmd(0x40 | (y & 7));
  Write_Cmd(0x80 | x);
}

void Write_Pixel(uint8_t x,uint8_t y, uint8_t state)
{
	SetPos(x,y/8);
	Write_Data(state<<y%8);
}
/* ??????? ?????, ????????????? ?????? ? ????? ??????? ???? */
void Display_Clear(void) 
{
	uint8_t x=0,y=0;
	GPIOA->BRR = CE;
  SetPos(0, 0);
  for (y = 0; y < 6; y++) 
	{
    for (x = 0; x < 84; x++) 
		{
      Write_Data(0);
    }    
  }
  SetPos(0, 0);
	GPIOA->BSRR = CE;
}

void SetStringInBuffer(String* string)
{
   uint8_t x = string->x_pos, y = string->y_pos, x_inv = 0;
    const char* pStr = string->Text;
    uint64_t temp = 0;
    int Length = strlen(string->Text),LengthInv = Length, widthInPixels = 0;
    //calculate width of text in pixels
    const char* lStr = string->Text;
    while(LengthInv-- > 0)
    {
       widthInPixels += string->font.descriptor[*(lStr)-' '].width + 1;
       lStr++;
    }

    if(string->align == AlignLeft) x = x;
    if(string->align == AlignCenter) x = 42 - widthInPixels/2;
    if(string->align == AlignRight) x = 83 - widthInPixels;
    x_inv = x;

    while(Length-- > 0)
    {
        uint8_t i,j;
        uint8_t currentCharWidth = string->font.descriptor[*(pStr)-' '].width;

        for(i = 0; i < currentCharWidth;i++)
        {
            uint8_t j_lim = (string->font.Height%8 == 0)?(string->font.Height/8):(string->font.Height/8+1);
            for(j = 0; j < j_lim; j++)
            {
                temp |= ((uint8_t)(string->font.pFont[(string->font.descriptor[*(pStr)-' '].offset+j*currentCharWidth) + i])<<(8*j+1));
            }
            temp <<= y%8;

            for(j = 0; j <= j_lim; j++)
            {
                DisplayBuffer[84*(y/8+j)+x+i] |= (uint8_t)((temp>>8*j)&0x000000FF);
            }
            temp = 0;
        }
        x += currentCharWidth+1;
        pStr++;
    }

    if(string->inverted == Inverted)
    {
        InvertRegion(x_inv-1, y, x_inv+widthInPixels + 1, y+string->font.Height+1);
    }
}

void SetWindow(pWindow wnd)
{
    for(int i = 0; i < wnd->StringsQuantity; i++)
    {
        SetStringInBuffer(&wnd->strings[i]);
    }
}

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
			DisplayBuffer[84*j + i] ^= mask;
		}
	}
}

void Clear_Buffer()
{
	memset(DisplayBuffer,0,sizeof(DisplayBuffer));
}	

void Write_Buffer()
{
	GPIOA->BRR = CE;
  SetPos(0,0);
  for(uint8_t y = 0;y < 6;y++)
	  for(uint8_t x = 0;x < 84;x++)
	  {
	    	Write_Data(*(DisplayBuffer+x+84*y));
	  }
	GPIOA->BSRR = CE;
//	for(uint16_t i = 0;i < sizeof(DisplayBuffer);i+=2)
//	{
//		Write_Data(*((volatile uint16_t*)(DisplayBuffer+i)));
//	}
}

void PaintBatteryIndicator(uint8_t percentage)
{
	  uint8_t BatteryBorder[12] = {0x3C,0x66,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x7E};
		for(uint8_t i = 68; i < 80;i++)
		{
			if(i < 79 - percentage)
		    DisplayBuffer[i] |= BatteryBorder[i-68];
			else
				DisplayBuffer[i] |= (BatteryBorder[i-68]|0x3C);
	  }		
}

void PaintUSBIndicator()
{
	uint8_t USBIndicator[15] = {0x08,0x08,0x08,0x1C,0x3E,0x3E,0x3E,0x3E,0x3E,0x3E,0x14,0x14,0x14,0x14,0x1C};
		for(uint8_t i = 3; i < 18;i++)
		{
		    DisplayBuffer[i] |= USBIndicator[i-3];
	  }	
}

void DrawPixel(uint8_t x, uint8_t y)
{
    DisplayBuffer[84*(y>>3)+x] |= 1<<(y%8);
}

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

void DrawRect(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height)
{
    DrawLine(x_pos,y_pos,x_pos+width,y_pos);
    DrawLine(x_pos+width,y_pos,x_pos+width,y_pos+height);
    DrawLine(x_pos+width,y_pos+height,x_pos,y_pos+height);
    DrawLine(x_pos,y_pos+height,x_pos,y_pos);
}

void FillRect(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height)
{
    for(;height>0;height--)
    {
       DrawLine(x_pos,y_pos,x_pos+width-1,y_pos);
       y_pos++;
    }
}

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
