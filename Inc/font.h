#ifndef __FONT_H
#define __FONT_H

//#define _NUM_CHARS 224
//#define _NUM 4*10

typedef struct
{
    unsigned char width;
    unsigned short offset;
}charDescriptor;

typedef struct 
{
	const char* pFont;
	unsigned char Height;
	const charDescriptor* descriptor;
}FontInfo;

//extern const char font_1[];
//extern const char tight_numeric_font[_NUM];
//extern const char week_days_string[64];
//extern const char MSSanSerif14[];

#endif 
