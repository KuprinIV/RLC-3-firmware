#include "font.h"

const char font_1[] =
{
//Punctuation
// " "
   0, 0, 0, 0, 0,
// "!"
   0, 0, 0x4F, 0, 0,
// """
   0, 0x07, 0, 0x07, 0,
// "#"
   0x14,
   0x7F,
   0x14,
   0x7F,
   0x14,
// "$"
   0x24,
   0x2A,
   0x7F,
   0x2A,
   0x12,
// "%"
   0x23,
   0x13,
   0x08,
   0x64,
   0x62,
// "&"
   0x36,
   0x49,
   0x55,
   0x22,
   0x50,
// "'"
   0,
   0x5,
   0x3,
   0,
   0,
// "("
   0,
   0x1C,
   0x22,
   0x41,
   0,
// ")"
   0,
   0x41,
   0x22,
   0x1C,
   0,
// "*"
   0x14,
   0x08,
   0x3E,
   0x08,
   0x14,
// "+"
   0x8,
   0x8,
   0x3E,
   0x8,
   0x8,
// ","
   00,
   0x50,
   0x30,
   0,
   0,
// "-"
   0x8,
   0x8,
   0x8,
   0x8,
   0x8,
// "."
   0,
   0x60,
   0x60,
   0,
   0,
// "/"
   0x20,
   0x10,
   0x8,
   0x4,
   0x2,

//Numerals
// "0"
   0x3E,
   0x51,
   0x49,
   0x45,
   0x3E,
// "1"
   0x00,
   0x42,
   0x7F,
   0x40,
   0x00,
// "2"
   0x42,
   0x61,
   0x51,
   0x49,
   0x46,
// "3"
   0x21,
   0x41,
   0x45,
   0x4B,
   0x31,
// "4"
   0x18,
   0x14,
   0x12,
   0x7F,
   0x10,
// "5"
   0x27,
   0x45,
   0x45,
   0x45,
   0x39,
// "6"
   0x3C,
   0x4A,
   0x49,
   0x49,
   0x30,
// "7"
   0x01,
   0x71,
   0x09,
   0x05,
   0x03,
// "8"
   0x36,
   0x49,
   0x49,
   0x49,
   0x36,
// "9"
   0x06,
   0x49,
   0x49,
   0x29,
   0x1E,
// ":"
   0x00,
   0x36,
   0x36,
   0x00,
   0x00,
// ";"
   0x00,
   0x56,
   0x36,
   0x00,
   0x00,
// "<"
   0x08,
   0x14,
   0x22,
   0x41,
   0x00,
// "="
   0x14,
   0x14,
   0x14,
   0x14,
   0x14,
// ">"
   0x00,
   0x41,
   0x22,
   0x14,
   0x08,
// "?"
   0x02,
   0x01,
   0x51,
   0x09,
   0x06,
// "@"
   0x32,
   0x49,
   0x79,
   0x41,
   0x3E,
// "A"
   0x7E,    
   0x11,    
   0x11,    
   0x11,    
   0x7E,    
//"B"
   0x7F,
   0x49,
   0x49,
   0x49,
   0x36,
//"C"
   0x3E,
   0x41,
   0x41,
   0x41,
   0x22,
//"D"
   0x7F,
   0x41,
   0x41,
   0x22,
   0x1C,
//"E"
   0x7F,
   0x49,
   0x49,
   0x49,
   0x41,
//"F"
   0x7F,
   0x9,
   0x9,
   0x9,
   0x1,
//"G"
   0x3E,
   0x41,
   0x49,
   0x49,
   0x7A,
//"H"
   0x7F,
   0x8,
   0x8,
   0x8,
   0x7F,
//"I"
   0,
   0x41,
   0x7F,
   0x41,
   0,
//"J"
   0x20,
   0x40,
   0x41,
   0x3F,
   0x1,
//"K"
   0x7F,
   0x8,
   0x14,
   0x22,
   0x41,
//"L"
   0x7F,
   0x40,
   0x40,
   0x40,
   0x40,
//"M"
   0x7F,
   0x2,
   0xC,
   0x2,
   0x7F,
//"N"
   0x7F,
   0x4,
   0x8,
   0x10,
   0x7F,
//"O"
   0x3E,
   0x41,
   0x41,
   0x41,
   0x3E,
//"P"
   0x7F,
   0x11,
   0x11,
   0x11,
   0xE,
//"Q"
   0x3E,
   0x41,
   0x51,
   0x21,
   0x5E,
//"R"
   0x7F,
   0x9,
   0x19,
   0x29,
   0x46,
//"S"
   0x46,
   0x49,
   0x49,
   0x49,
   0x31,
//"T"
   0x1,
   0x1,
   0x7F,
   0x1,
   0x1,
//"U"
   0x3F,
   0x40,
   0x40,
   0x40,
   0x3F,
//"V"
   0x1F,
   0x20,
   0x40,
   0x20,
   0x1F,
//"W"
   0x3F,
   0x40,
   0x38,
   0x40,
   0x3F,
//"X"
   0x63,
   0x14,
   0x8,
   0x14,
   0x63,
//"Y"
   0x7,
   0x8,
   0x70,
   0x8,
   0x7,
//"Z"
   0x61,
   0x51,
   0x49,
   0x45,
   0x43,
//"["
   0,
   0x7F,
   0x41,
   0x41,
   0,
//"\"
   0x2,
   0x4,
   0x8,
   0x10,
   0x20,
//"]"
   0,
   0x41,
   0x41,
   0x7F,
   0,
//"^"
   0x4,
   0x2,
   0x1,
   0x2,
   0x4,
//"_"
   0x40,
   0x40,
   0x40,
   0x40,
   0x40,
//"`"
   0,
   0x1,
   0x2,
   0,
   0,
//"a"
   0x20,
   0x54,
   0x54,
   0x54,
   0x78,
//"b"
   0x7F,
   0x48,
   0x44,
   0x44,
   0x38,
//"c"
   0x38,
   0x44,
   0x44,
   0x44,
   0x20,
//"d"
   0x38,
   0x44,
   0x44,
   0x48,
   0x7F,
//"e"
   0x38,
   0x54,
   0x54,
   0x54,
   0x18,
//"f"
   0x8,
   0x7E,
   0x9,
   0x1,
   0x2,
//"g"
   0x8,
   0x54,
   0x54,
   0x54,
   0x3C,
//"h"
   0x7F,
   0x8,
   0x4,
   0x4,
   0x78,
//"i"
   0,
   0x44,
   0x7D,
   0x40,
   0,
//"j"
   0x20,
   0x40,
   0x44,
   0x3D,
   0,
//"k"
   0x7F,
   0x10,
   0x28,
   0x44,
   0,
//"l"
   0,
   0x41,
   0x7F,
   0x40,
   0,
//"m"
   0x7C,
   0x4,
   0x18,
   0x4,
   0x78,
//"n"
   0x7C,
   0x8,
   0x4,
   0x4,
   0x78,
//"o"
   0x38,
   0x44,
   0x44,
   0x44,
   0x38,
//"p"
   0x7C,
   0x14,
   0x14,
   0x14,
   0x8,
//"q"
   0x8,
   0x14,
   0x14,
   0x14,
   0x7C,
//"r"
   0x7C,
   0x8,
   0x4,
   0x4,
   0x8,
//"s"
   0x48,
   0x54,
   0x54,
   0x54,
   0x24,
//"t"
   0x4,
   0x3F,
   0x44,
   0x40,
   0x20,
//"u"
   0x3C,
   0x40,
   0x40,
   0x20,
   0x7C,
//"v"
   0x1C,
   0x20,
   0x40,
   0x20,
   0x1C,
//"w"
   0x3C,
   0x40,
   0x3C,
   0x40,
   0x3C,
//"x"
   0x44,
   0x28,
   0x10,
   0x28,
   0x44,
//"y"
   0xC,
   0x50,
   0x50,
   0x50,
   0x3C,
//"z"
   0x44,
   0x64,
   0x54,
   0x4C,
   0x44,
   
   0x00, 
   0x08, 
   0x36, 
   0x41, 
   0x00,   // { 0x7B 123
   
   0x00, 
   0x00, 
   0x7F, 
   0x00, 
   0x00,   // | 0x7C 124
   
   0x00, 
   0x41, 
   0x36, 
   0x08, 
   0x00,   // } 0x7D 125
   
   0x08, 
   0x04, 
   0x08, 
   0x10, 
   0x08,   // ~ 0x7E 126
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   //  0x7F 127
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // Ђ 0x80 128
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // Ѓ 0x81 129
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // ‚ 0x82 130
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // ѓ 0x83 131
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // „ 0x84 132
   
   0x40, 
   0x00, 
   0x40, 
   0x00, 
   0x40,   // … 0x85 133
   
   0x04, 
   0x04, 
   0xFF, 
   0x04, 
   0x04,   // † 0x86 134
   
   0x24, 
   0x24, 
   0xFF, 
   0x24, 
   0x24,   // ‡ 0x87 135
   
   0x28, 
   0x7C, 
   0xAA, 
   0xAA, 
   0x82,   // € 0x88 136
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // ‰ 0x89 137
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // Љ 0x8A 138
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // ‹ 0x8B 139
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // Њ 0x8C 140
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // Ќ 0x8D 141
   
   0x00, 
   0x00, 
   0x00, 
   0x00, 
   0x00,   // Ћ 0x8E 142

   0x00, 0x00, 0x00, 0x00, 0x00,   // Џ 0x8F 143

   0x00, 0x00, 0x00, 0x00, 0x00,   // ђ 0x90 144

   0x00, 0x06, 0x05, 0x00, 0x00,   // ‘ 0x91 145
   
   0x00, 0x00, 0x05, 0x03, 0x00,   // ’ 0x92 146
   
   0x06, 0x05, 0x00, 0x06, 0x05,   // “ 0x93 147
   
   0x05, 0x03, 0x00, 0x05, 0x03,   // ” 0x94 148
   
   0x18, 0x3C, 0x3C, 0x3C, 0x18,   // • 0x95 149
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // – 0x96 150
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // — 0x97 151
   
   0x78, 0x48, 0x48, 0x78, 0x00,   //  0x98 152
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // ™ 0x99 153
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // љ 0x9A 154
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // › 0x9B 155
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // њ 0x9C 156
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // ќ 0x9D 157
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // ћ 0x9E 158
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // џ 0x9F 159
   
   0x00, 0x00, 0x00, 0x00, 0x00,   //   0xA0 160
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // Ў 0xA1 161
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // ў 0xA2 162
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // Ј 0xA3 163
   
   0x22, 0x1C, 0x14, 0x1C, 0x22,   // ¤ 0xA4 164
   
   0x7E, 0x02, 0x02, 0x02, 0x03,   // Ґ 0xA5 165
   
   0x00, 0x00, 0xE7, 0x00, 0x00,   // ¦ 0xA6 166
   
   0x4A, 0x95, 0xA5, 0xA9, 0x52,   // § 0xA7 167
   
   0x7C, 0x55, 0x54, 0x45, 0x44,   // Ё 0xA8 168
   
   0x00, 0x18, 0x24, 0x24, 0x00,   // © 0xA9 169
   
   0x3E, 0x49, 0x49, 0x41, 0x22,   // Є 0xAA 170
   
   0x08, 0x14, 0x2A, 0x14, 0x22,   // « 0xAB 171
   
   0x04, 0x04, 0x04, 0x04, 0x0C,   // ¬ 0xAC 172
   
   0x00, 0x08, 0x08, 0x08, 0x00,   // ­ 0xAD 173
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // ® 0xAE 174
   
   0x00, 0x45, 0x7C, 0x45, 0x00,   // Ї 0xAF 175
   
   0x00, 0x06, 0x09, 0x09, 0x06,   // ° 0xB0 176
   
   0x44, 0x44, 0x5F, 0x44, 0x44,   // ± 0xB1 177
   
   0x00, 0x41, 0x7F, 0x41, 0x00,   // І 0xB2 178
   
   0x00, 0x00, 0x7A, 0x00, 0x00,   // і 0xB3 179
   
   0x00, 0x78, 0x08, 0x0C, 0x00,   // ґ 0xB4 180
   
   0x00, 0xFC, 0x20, 0x3C, 0x20,   // µ 0xB5 181
   
   0x0C, 0x1E, 0xFE, 0x02, 0xFE,   // ¶ 0xB6 182
   
   0x00, 0x18, 0x18, 0x00, 0x00,   // · 0xB7 183
   
   0x39, 0x54, 0x54, 0x49, 0x00,   // ё 0xB8 184
   
   0x78, 0x10, 0x20, 0x7B, 0x03,   // № 0xB9 185
   
   0x38, 0x54, 0x54, 0x44, 0x00,   // є 0xBA 186
   
   0x22, 0x14, 0x2A, 0x14, 0x08,   // » 0xBB 187
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // ј 0xBC 188
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // Ѕ 0xBD 189
   
   0x00, 0x00, 0x00, 0x00, 0x00,   // ѕ 0xBE 190
   
   0x00, 0x54, 0x70, 0x44, 0x00,   // ї 0xBF 191
   
   0x7E, 0x11, 0x11, 0x11, 0x7E,   // А 0xC0 192
   
   0x7F, 0x49, 0x49, 0x49, 0x31,   // Б 0xC1 193
   
   0x7F, 0x49, 0x49, 0x49, 0x36,   // В 0xC2 194
   
   0x7F, 0x01, 0x01, 0x01, 0x01,   // Г 0xC3 195
   
   0x60, 0x3F, 0x21, 0x3F, 0x60,   // Д 0xC4 196
   
   0x7F, 0x49, 0x49, 0x49, 0x41,   // Е 0xC5 197
   
   0x77, 0x08, 0x7F, 0x08, 0x77,   // Ж 0xC6 198
   
   0x22, 0x41, 0x49, 0x49, 0x36,   // З 0xC7 199
   
   0x7F, 0x10, 0x08, 0x04, 0x7F,   // И 0xC8 200
   
   0x7E, 0x10, 0x09, 0x04, 0x7E,   // Й 0xC9 201
   
   0x7F, 0x08, 0x14, 0x22, 0x41,   // К 0xCA 202
   
   0x40, 0x3E, 0x01, 0x01, 0x7F,   // Л 0xCB 203
   
   0x7F, 0x02, 0x0C, 0x02, 0x7F,   // М 0xCC 204
   
   0x7F, 0x08, 0x08, 0x08, 0x7F,   // Н 0xCD 205
   
   0x3E, 0x41, 0x41, 0x41, 0x3E,   // О 0xCE 206
   
   0x7F, 0x01, 0x01, 0x01, 0x7F,   // П 0xCF 207
   
   0x7F, 0x09, 0x09, 0x09, 0x06,   // Р 0xD0 208
   
   0x3E, 0x41, 0x41, 0x41, 0x22,   // С 0xD1 209
   
   0x01, 0x01, 0x7F, 0x01, 0x01,   // Т 0xD2 210
   
   0x07, 0x48, 0x48, 0x48, 0x3F,   // У 0xD3 211
   
   0x0E, 0x11, 0x7F, 0x11, 0x0E,   // Ф 0xD4 212
   
   0x63, 0x14, 0x08, 0x14, 0x63,   // Х 0xD5 213
   
   0x3F, 0x20, 0x20, 0x3F, 0x60,   // Ц 0xD6 214
   
   0x07, 0x08, 0x08, 0x08, 0x7F,   // Ч 0xD7 215
   
   0x7F, 0x40, 0x7E, 0x40, 0x7F,   // Ш 0xD8 216
   
   0x3F, 0x20, 0x3F, 0x20, 0x7F,   // Щ 0xD9 217
   
   0x01, 0x7F, 0x48, 0x48, 0x38,   // Ъ 0xDA 218
   
   0x7F, 0x48, 0x38, 0x00, 0x7F,   // Ы 0xDB 219
   
   0x00, 0x7F, 0x48, 0x48, 0x38,   // Ь 0xDC 220
   
   0x22, 0x41, 0x49, 0x49, 0x3E,   // Э 0xDD 221
   
   0x7F, 0x08, 0x3E, 0x41, 0x3E,   // Ю 0xDE 222
   
   0x46, 0x29, 0x19, 0x09, 0x7F,   // Я 0xDF 223
   
   0x20, 0x54, 0x54, 0x54, 0x78,   // а 0xE0 224
   
   0x7C, 0x54, 0x54, 0x54, 0x24,   // б 0xE1 225
   
   0x7C, 0x54, 0x54, 0x54, 0x28,   // в 0xE2 226
   
   0x7C, 0x04, 0x04, 0x04, 0x00,   // г 0xE3 227
   
   0x60, 0x3C, 0x24, 0x3C, 0x60,   // д 0xE4 228
   
   0x38, 0x54, 0x54, 0x54, 0x18,   // е 0xE5 229
   
   0x6C, 0x10, 0x7C, 0x10, 0x6C,   // ж 0xE6 230
   
   0x00, 0x44, 0x54, 0x54, 0x28,   // з 0xE7 231
   
   0x7C, 0x20, 0x10, 0x08, 0x7C,   // и 0xE8 232
   
   0x7C, 0x21, 0x12, 0x09, 0x7C,   // й 0xE9 233
   
   0x7C, 0x10, 0x10, 0x28, 0x44,   // к 0xEA 234
   
   0x40, 0x38, 0x04, 0x04, 0x7C,   // л 0xEB 235
   
   0x7C, 0x08, 0x10, 0x08, 0x7C,   // м 0xEC 236
   
   0x7C, 0x10, 0x10, 0x10, 0x7C,   // н 0xED 237
   
   0x38, 0x44, 0x44, 0x44, 0x38,   // о 0xEE 238
   
   0x7C, 0x04, 0x04, 0x04, 0x7C,   // п 0xEF 239
   
   0x7C, 0x14, 0x14, 0x14, 0x08,   // р 0xF0 240
   
   0x38, 0x44, 0x44, 0x44, 0x28,   // с 0xF1 241
   
   0x04, 0x04, 0x7C, 0x04, 0x04,   // т 0xF2 242
   
   0x0C, 0x50, 0x50, 0x50, 0x3C,   // у 0xF3 243
   
   0x08, 0x14, 0x7C, 0x14, 0x08,   // ф 0xF4 244
   
   0x44, 0x28, 0x10, 0x28, 0x44,   // х 0xF5 245
   
   0x3C, 0x20, 0x20, 0x3C, 0x60,   // ц 0xF6 246
   
   0x0C, 0x10, 0x10, 0x10, 0x7C,   // ч 0xF7 247
   
   0x7C, 0x40, 0x7C, 0x40, 0x7C,   // ш 0xF8 248
   
   0x3C, 0x20, 0x3C, 0x20, 0x7C,   // щ 0xF9 249
   
   0x04, 0x7C, 0x50, 0x50, 0x20,   // ъ 0xFA 250
   
   0x7C, 0x50, 0x20, 0x00, 0x7C,   // ы 0xFB 251
   
   0x7C, 0x50, 0x50, 0x50, 0x20,   // ь 0xFC 252
   
   0x28, 0x44, 0x54, 0x54, 0x38,
      
   0x7C, 0x10, 0x38, 0x44, 0x38,   
   
   0x48, 0x54, 0x34, 0x14, 0x7C,   
};

const charDescriptor font_1_Descriptors[] =
{
//Punctuation
// " "
   {5, 0},
// "!"
   {5, 5},
// """
   {5, 10},
// "#"
   {5, 15},
// "$"
   {5, 20},
// "%"
   {5, 25},
// "&"
   {5, 30},
// "'"
   {5, 35},
// "("
   {5, 40},
// ")"
   {5, 45},
// "*"
   {5, 50},
// "+"
   {5, 55},
// ","
   {5, 60},
// "-"
   {5, 65},
// "."
   {5, 70},
// "/"
   {5, 75},

//Numerals
// "0"
   {5, 80},
// "1"
   {5, 85},
// "2"
   {5, 90},
// "3"
   {5, 95},
// "4"
   {5, 100},
// "5"
   {5, 105},
// "6"
   {5, 110},
// "7"
   {5, 115},
// "8"
   {5, 120},
// "9"
   {5, 125},
// ":"
   {5, 130},
// ";"
   {5, 135},
// "<"
   {5, 140},
// "="
   {5, 145},
// ">"
   {5, 150},
// "?"
   {5, 155},
// "@"
   {5, 160},
// "A"
   {5, 165},
//"B"
   {5, 170},
//"C"
   {5, 175},
//"D"
   {5, 180},
//"E"
   {5, 185},
//"F"
   {5, 190},
//"G"
   {5, 195},
//"H"
   {5, 200},
//"I"
   {5, 205},
//"J"
   {5, 210},
//"K"
   {5, 215},
//"L"
   {5, 220},
//"M"
   {5, 225},
//"N"
   {5, 230},
//"O"
   {5, 235},
//"P"
   {5, 240},
//"Q"
   {5, 245},
//"R"
   {5, 250},
//"S"
   {5, 255},
//"T"
   {5, 260},
//"U"
   {5, 265},
//"V"
   {5, 270},
//"W"
   {5, 275},
//"X"
   {5, 280},
//"Y"
   {5, 285},
//"Z"
   {5, 290},
//"["
   {5, 295},
//"\"
   {5, 300},
//"]"
   {5, 305},
//"^"
   {5, 310},
//"_"
   {5, 315},
//"`"
   {5, 320},
//"a"
   {5, 325},
//"b"
   {5, 330},
//"c"
   {5, 335},
//"d"
   {5, 340},
//"e"
   {5, 345},
//"f"
   {5, 350},
//"g"
   {5, 355},
//"h"
   {5, 360},
//"i"
   {5, 365},
//"j"
   {5, 370},
//"k"
   {5, 375},
//"l"
   {5, 380},
//"m"
   {5, 385},
//"n"
   {5, 390},
//"o"
   {5, 395},
//"p"
   {5, 400},
//"q"
   {5, 405},
//"r"
   {5, 410},
//"s"
   {5, 415},
//"t"
   {5, 420},
//"u"
   {5, 425},
//"v"
   {5, 430},
//"w"
   {5, 435},
//"x"
   {5, 440},
//"y"
   {5, 445},
//"z"
   {5, 450},

   {5, 455},   // { 0x7B 123

   {5, 460},   // | 0x7C 124

   {5, 465},  // } 0x7D 125

   {5, 470},   // ~ 0x7E 126

   {5, 475},  //  0x7F 127

   {5, 480},   // Ђ 0x80 128

   {5, 485},   // Ѓ 0x81 129

   {5, 490},   // ‚ 0x82 130

   {5, 495},   // ѓ 0x83 131

   {5, 500},   // „ 0x84 132

   {5, 505},   // … 0x85 133

   {5, 510},   // † 0x86 134

   {5, 515},   // ‡ 0x87 135

   {5, 520},   // € 0x88 136

   {5, 525},   // ‰ 0x89 137

   {5, 530},   // Љ 0x8A 138

   {5, 535},   // ‹ 0x8B 139

   {5, 540},   // Њ 0x8C 140

   {5, 545},   // Ќ 0x8D 141

   {5, 550},   // Ћ 0x8E 142

   {5, 555},   // Џ 0x8F 143

   {5, 560},   // ђ 0x90 144

   {5, 565},   // ‘ 0x91 145

   {5, 570},  // ’ 0x92 146

   {5, 575},   // “ 0x93 147

   {5, 580},   // ” 0x94 148

   {5, 585},   // • 0x95 149

   {5, 590},   // – 0x96 150

   {5, 595},   // — 0x97 151

   {5, 600},   //  0x98 152

   {5, 605},   // ™ 0x99 153

   {5, 610},   // љ 0x9A 154

   {5, 615},   // › 0x9B 155

   {5, 620},   // њ 0x9C 156

   {5, 625},   // ќ 0x9D 157

   {5, 630},   // ћ 0x9E 158

   {5, 635},   // џ 0x9F 159

   {5, 640},  //   0xA0 160

   {5, 645},   // Ў 0xA1 161

   {5, 650},   // ў 0xA2 162

   {5, 655},   // Ј 0xA3 163

   {5, 660},   // ¤ 0xA4 164

   {5, 665},  // Ґ 0xA5 165

   {5, 670},   // ¦ 0xA6 166

   {5, 675},   // § 0xA7 167

   {5, 680},   // Ё 0xA8 168

   {5, 685},   // © 0xA9 169

   {5, 690},   // Є 0xAA 170

   {5, 695},   // « 0xAB 171

   {5, 700},   // ¬ 0xAC 172

   {5, 705},   // ­ 0xAD 173

   {5, 710},   // ® 0xAE 174

   {5, 715},   // Ї 0xAF 175

   {5, 720},   // ° 0xB0 176

   {5, 725},   // ± 0xB1 177

   {5, 730},   // І 0xB2 178

   {5, 735},   // і 0xB3 179

   {5, 740},   // ґ 0xB4 180

   {5, 745},   // µ 0xB5 181

   {5, 750},   // ¶ 0xB6 182

   {5, 755},   // · 0xB7 183

   {5, 760},   // ё 0xB8 184

   {5, 765},   // № 0xB9 185

   {5, 770},   // є 0xBA 186

   {5, 775},  // » 0xBB 187

   {5, 780},   // ј 0xBC 188

   {5, 785},   // Ѕ 0xBD 189

   {5, 790},   // ѕ 0xBE 190

   {5, 795},   // ї 0xBF 191

   {5, 800},   // А 0xC0 192

   {5, 805},   // Б 0xC1 193

   {5, 810},   // В 0xC2 194

   {5, 815},   // Г 0xC3 195

   {5, 820},   // Д 0xC4 196

   {5, 825},   // Е 0xC5 197

   {5, 830},   // Ж 0xC6 198

   {5, 835},   // З 0xC7 199

   {5, 840},   // И 0xC8 200

   {5, 845},   // Й 0xC9 201

   {5, 850},   // К 0xCA 202

   {5, 855},  // Л 0xCB 203

   {5, 860},  // М 0xCC 204

   {5, 865},   // Н 0xCD 205

   {5, 870},   // О 0xCE 206

   {5, 875},   // П 0xCF 207

   {5, 880},   // Р 0xD0 208

   {5, 885},   // С 0xD1 209

   {5, 890},   // Т 0xD2 210

   {5, 895},  // У 0xD3 211

   {5, 900},   // Ф 0xD4 212

   {5, 905},   // Х 0xD5 213

   {5, 910},   // Ц 0xD6 214

   {5, 915},   // Ч 0xD7 215

   {5, 920},   // Ш 0xD8 216

   {5, 925},   // Щ 0xD9 217

   {5, 930},   // Ъ 0xDA 218

   {5, 935},   // Ы 0xDB 219

   {5, 940},   // Ь 0xDC 220

   {5, 945},   // Э 0xDD 221

   {5, 950},   // Ю 0xDE 222

   {5, 955},   // Я 0xDF 223

   {5, 960},   // а 0xE0 224

   {5, 965},   // б 0xE1 225

   {5, 970},   // в 0xE2 226

   {5, 975},  // г 0xE3 227

   {5, 980},   // д 0xE4 228

   {5, 985},  // е 0xE5 229

   {5, 990},  // ж 0xE6 230

   {5, 995},   // з 0xE7 231

   {5, 1000},  // и 0xE8 232

   {5, 1005},   // й 0xE9 233

   {5, 1010},   // к 0xEA 234

   {5, 1015},   // л 0xEB 235

   {5, 1020},   // м 0xEC 236

   {5, 1025},   // н 0xED 237

   {5, 1030},   // о 0xEE 238

   {5, 1035},   // п 0xEF 239

   {5, 1040},   // р 0xF0 240

   {5, 1045},   // с 0xF1 241

   {5, 1050},   // т 0xF2 242

   {5, 1055},   // у 0xF3 243

   {5, 1060},  // ф 0xF4 244

   {5, 1065},   // х 0xF5 245

   {5, 1070},   // ц 0xF6 246

   {5, 1075},   // ч 0xF7 247

   {5, 1080},   // ш 0xF8 248

   {5, 1085},   // щ 0xF9 249

   {5, 1090},   // ъ 0xFA 250

   {5, 1095},   // ы 0xFB 251

   {5, 1100},   // ь 0xFC 252

   {5, 1105},

   {5, 1110},

   {5, 1115}
};

FontInfo font6x8 = 
{
	font_1,
  8,
  font_1_Descriptors
};

const char MSSansSerif_6pt[] = 
{
	/* @0 ' ' (2 pixels wide) */
	//   
	//   
	//   
	//   
	//   
	//   
	//   
	//   
	0x00, 0x00, 
	
	/* @0 '!' (1 pixels wide) */
	// #
	// #
	// #
	// #
	// #
	// #
	//  
	//  
	0x3F, 

	/* @1 '"' (3 pixels wide) */
	// # #
	// # #
	//    
	//    
	//    
	//    
	//    
	//    
	0x03, 0x00, 0x03, 

	/* @4 '#' (4 pixels wide) */
	//  # #
	//  # #
	// ####
	//  # #
	// ### 
	// # # 
	//     
	//     
	0x34, 0x1F, 0x34, 0x0F, 

	/* @8 '$' (4 pixels wide) */
	//  ## 
	// # ##
	//  #  
	//   ##
	// # ##
	//  ###
	//   # 
	//     
	0x12, 0x25, 0x7B, 0x3A, 

	/* @12 '%' (7 pixels wide) */
	//  ## #  
	// # # #  
	// ####   
	//    ### 
	//   # # #
	//   # ###
	//        
	//        
	0x06, 0x05, 0x37, 0x0C, 0x3B, 0x28, 0x30, 

	/* @19 '&' (5 pixels wide) */
	//   #  
	//  # # 
	//   #  
	//  ## #
	// #  ##
	//  ####
	//      
	//      
	0x10, 0x2A, 0x2D, 0x32, 0x38, 

	/* @24 ''' (1 pixels wide) */
	// #
	// #
	//  
	//  
	//  
	//  
	//  
	//  
	0x03, 

	/* @25 '(' (2 pixels wide) */
	//  #
	//  #
	//  #
	// # 
	// # 
	//  #
	//  #
	//  #
	0x18, 0xE7, 

	/* @27 ')' (2 pixels wide) */
	// # 
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	// # 
	0x81, 0x7E, 

	/* @29 '*' (3 pixels wide) */
	//  # 
	// ###
	// # #
	//    
	//    
	//    
	//    
	//    
	0x06, 0x03, 0x06, 

	/* @32 '+' (4 pixels wide) */
	//     
	//   # 
	//   # 
	// ####
	//   # 
	//     
	//     
	//     
	0x08, 0x08, 0x1E, 0x08, 

	/* @36 ',' (1 pixels wide) */
	//  
	//  
	//  
	//  
	//  
	// #
	// #
	//  
	0x60, 

	/* @37 '-' (2 pixels wide) */
	//   
	//   
	//   
	// ##
	//   
	//   
	//   
	//   
	0x08, 0x08, 

	/* @39 '.' (1 pixels wide) */
	//  
	//  
	//  
	//  
	//  
	// #
	//  
	//  
	0x20, 

	/* @40 '/' (2 pixels wide) */
	//  #
	//  #
	//  #
	// # 
	// # 
	// # 
	//   
	//   
	0x38, 0x07, 

	/* @42 '0' (4 pixels wide) */
	//  ## 
	// #  #
	// #  #
	// #  #
	// #  #
	//  ##
	//     
	//     
	0x1E, 0x21, 0x21, 0x1E, 

	/* @46 '1' (2 pixels wide) */
	//  #
	// ##
	//  #
	//  #
	//  #
	//  #
	//   
	//   
	0x02, 0x3F, 

	/* @48 '2' (4 pixels wide) */
	//  ## 
	// #  #
	//    #
	//   # 
	//  #  
	// ####
	//     
	//     
	0x22, 0x31, 0x29, 0x26, 

	/* @52 '3' (4 pixels wide) */
	//  ## 
	// #  #
	//   ##
	//    #
	// #  #
	//  ###
	//     
	//     
	0x12, 0x21, 0x25, 0x3E, 

	/* @56 '4' (4 pixels wide) */
	//   # 
	//   # 
	//  ## 
	// # # 
	// ####
	//   # 
	//     
	//     
	0x18, 0x14, 0x3F, 0x10, 

	/* @60 '5' (4 pixels wide) */
	//  ###
	//  #  
	//  ## 
	//    #
	// #  #
	//  ###
	//     
	//     
	0x10, 0x27, 0x25, 0x39, 

	/* @64 '6' (4 pixels wide) */
	//  ## 
	// #  #
	// ### 
	// #  #
	// #  #
	//  ###
	//     
	//     
	0x1E, 0x25, 0x25, 0x3A, 

	/* @68 '7' (4 pixels wide) */
	// ####
	//    #
	//    #
	//   # 
	//   # 
	//  #  
	//     
	//     
	0x01, 0x21, 0x19, 0x07, 

	/* @72 '8' (4 pixels wide) */
	//  ## 
	// #  #
	//  # #
	//  ###
	// #  #
	//  ###
	//     
	//     
	0x12, 0x2D, 0x29, 0x3E, 

	/* @76 '9' (4 pixels wide) */
	//  ## 
	// #  #
	// #  #
	//  ###
	// #  #
	//  ## 
	//     
	//     
	0x16, 0x29, 0x29, 0x1E, 

	/* @80 ':' (1 pixels wide) */
	//  
	//  
	// #
	//  
	//  
	// #
	//  
	//  
	0x24, 

	/* @81 ';' (1 pixels wide) */
	//  
	//  
	// #
	//  
	//  
	// #
	// #
	//  
	0x64, 

	/* @82 '<' (4 pixels wide) */
	//     
	//    #
	//  ## 
	// ##  
	//   ##
	//     
	//     
	//     
	0x08, 0x0C, 0x14, 0x12, 

	/* @86 '=' (4 pixels wide) */
	//     
	//     
	// ####
	// ####
	//     
	//     
	//     
	//     
	0x0C, 0x0C, 0x0C, 0x0C, 

	/* @90 '>' (4 pixels wide) */
	//     
	// #   
	//  ## 
	//   ##
	// ##  
	//     
	//     
	//     
	0x12, 0x14, 0x0C, 0x08, 

	/* @94 '?' (4 pixels wide) */
	//  ## 
	// #  #
	//    #
	//   # 
	//   # 
	//   # 
	//     
	//     
	0x02, 0x01, 0x39, 0x06, 

	/* @98 '@' (8 pixels wide) */
	//   ####  
	//  #    # 
	//  # ### #
	// # #  # #
	// # #  # #
	// # ##### 
	//  #      
	//   ####  
	0x38, 0x46, 0xB9, 0xA5, 0xA5, 0xBD, 0x22, 0x1C, 

	/* @106 'A' (5 pixels wide) */
	//   #  
	//   ## 
	//  # # 
	//  ### 
	// #   #
	// #   #
	//      
	//      
	0x30, 0x0C, 0x0B, 0x0E, 0x30, 

	/* @111 'B' (4 pixels wide) */
	// ### 
	// #  #
	// # # 
	// ## #
	// #  #
	// ### 
	//     
	//     
	0x3F, 0x29, 0x25, 0x1A, 

	/* @115 'C' (5 pixels wide) */
	//  ### 
	// #   #
	// #    
	// #    
	// #   #
	//  ###
	//      
	//      
	0x1E, 0x21, 0x21, 0x21, 0x12, 

	/* @120 'D' (4 pixels wide) */
	// ### 
	// #  #
	// #  #
	// #  #
	// #  #
	// ### 
	//     
	//     
	0x3F, 0x21, 0x21, 0x1E, 

	/* @124 'E' (4 pixels wide) */
	// ####
	// #   
	// #   
	// ####
	// #   
	// ####
	//     
	//     
	0x3F, 0x29, 0x29, 0x29, 

	/* @128 'F' (4 pixels wide) */
	// ####
	// #   
	// #   
	// ### 
	// #   
	// #   
	//     
	//     
	0x3F, 0x09, 0x09, 0x01, 

	/* @132 'G' (6 pixels wide) */
	//   ### 
	//  #   #
	// #     
	// #  ###
	//  #   #
	//  #####
	//       
	//       
	0x0C, 0x32, 0x21, 0x29, 0x29, 0x3A, 

	/* @138 'H' (4 pixels wide) */
	// #  #
	// #  #
	// ####
	// #  #
	// #  #
	// #  #
	//     
	//     
	0x3F, 0x04, 0x04, 0x3F, 

	/* @142 'I' (1 pixels wide) */
	// #
	// #
	// #
	// #
	// #
	// #
	//  
	//  
	0x3F, 

	/* @143 'J' (3 pixels wide) */
	//   #
	//   #
	//   #
	//   #
	// # #
	//  ##
	//    
	//    
	0x10, 0x20, 0x3F, 

	/* @146 'K' (4 pixels wide) */
	// #  #
	// # # 
	// ##  
	// # # 
	// # # 
	// #  #
	//     
	//     
	0x3F, 0x04, 0x1A, 0x21, 

	/* @150 'L' (3 pixels wide) */
	// #  
	// #  
	// #  
	// #  
	// #  
	// ###
	//    
	//    
	0x3F, 0x20, 0x20, 

	/* @153 'M' (5 pixels wide) */
	// #   #
	// ## ##
	// ## ##
	// ## ##
	// # # #
	// # # #
	//      
	//      
	0x3F, 0x0E, 0x30, 0x0E, 0x3F, 

	/* @158 'N' (4 pixels wide) */
	// #  #
	// #  #
	// ## #
	// # ##
	// # ##
	// #  #
	//     
	//     
	0x3F, 0x04, 0x18, 0x3F, 

	/* @162 'O' (6 pixels wide) */
	//   ### 
	//  #   #
	// #    #
	// #    #
	//  #   #
	//  #### 
	//       
	//       
	0x0C, 0x32, 0x21, 0x21, 0x21, 0x1E, 

	/* @168 'P' (4 pixels wide) */
	// ### 
	// #  #
	// #  #
	// ### 
	// #   
	// #   
	//     
	//     
	0x3F, 0x09, 0x09, 0x06, 

	/* @172 'Q' (6 pixels wide) */
	//   ### 
	//  #   #
	// #    #
	// #    #
	//  # # #
	//  #####
	//       
	//       
	0x0C, 0x32, 0x21, 0x31, 0x21, 0x3E, 

	/* @178 'R' (4 pixels wide) */
	// ####
	// #  #
	// #  #
	// ### 
	// #  #
	// #  #
	//     
	//     
	0x3F, 0x09, 0x09, 0x37, 

	/* @182 'S' (5 pixels wide) */
	//  ### 
	// #   #
	//  ##  
	//    ##
	// #   #
	//  ### 
	//      
	//      
	0x12, 0x25, 0x25, 0x29, 0x1A, 

	/* @187 'T' (5 pixels wide) */
	// #####
	//   #  
	//   #  
	//   #  
	//   #  
	//   #  
	//      
	//      
	0x01, 0x01, 0x3F, 0x01, 0x01, 

	/* @192 'U' (4 pixels wide) */
	// #  #
	// #  #
	// #  #
	// #  #
	// #  #
	// ####
	//     
	//     
	0x3F, 0x20, 0x20, 0x3F, 

	/* @196 'V' (5 pixels wide) */
	// #   #
	// #   #
	//  #  #
	//  # # 
	//   ## 
	//   #  
	//      
	//      
	0x03, 0x0C, 0x30, 0x18, 0x07, 

	/* @201 'W' (7 pixels wide) */
	// #  #  #
	// #  ## #
	//  # ## #
	//  ## # #
	//  ##  # 
	//   #  # 
	//        
	//        
	0x03, 0x1C, 0x38, 0x07, 0x0E, 0x30, 0x0F, 

	/* @208 'X' (5 pixels wide) */
	// #   #
	//  # # 
	//   #  
	//   #  
	//  # # 
	// #   #
	//      
	//      
	0x21, 0x12, 0x0C, 0x12, 0x21, 

	/* @213 'Y' (5 pixels wide) */
	// #   #
	//  #  #
	//  # # 
	//   #  
	//   #  
	//   #  
	//      
	//      
	0x01, 0x06, 0x38, 0x04, 0x03, 

	/* @218 'Z' (5 pixels wide) */
	// #####
	//    # 
	//   #  
	//   #  
	//  #   
	// #####
	//      
	//      
	0x21, 0x31, 0x2D, 0x23, 0x21, 

	/* @223 '[' (1 pixels wide) */
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	0xFF, 

	/* @224 '\' (2 pixels wide) */
	// # 
	// # 
	// # 
	//  #
	//  #
	//  #
	//   
	//   
	0x07, 0x38, 

	/* @226 ']' (2 pixels wide) */
	// ##
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	// ##
	0x81, 0xFF, 

	/* @228 '^' (3 pixels wide) */
	//  # 
	//  ##
	// # #
	//    
	//    
	//    
	//    
	//    
	0x04, 0x03, 0x06, 

	/* @231 '_' (4 pixels wide) */
	//     
	//     
	//     
	//     
	//     
	//     
	// ####
	//     
	0x40, 0x40, 0x40, 0x40, 

	/* @235 '`' (2 pixels wide) */
	// ##
	//   
	//   
	//   
	//   
	//   
	//   
	//   
	0x01, 0x01, 

	/* @237 'a' (4 pixels wide) */
	//     
	//     
	// ####
	//  ###
	// #  #
	// ####
	//     
	//     
	0x34, 0x2C, 0x2C, 0x3C, 

	/* @241 'b' (3 pixels wide) */
	// #  
	// #  
	// ###
	// # #
	// # #
	// ###
	//    
	//    
	0x3F, 0x24, 0x3C, 

	/* @244 'c' (4 pixels wide) */
	//     
	//     
	//  ###
	// #   
	// #   
	//  ###
	//     
	//     
	0x18, 0x24, 0x24, 0x24, 

	/* @248 'd' (4 pixels wide) */
	//    #
	//    #
	//  ###
	// #  #
	// #  #
	//  ###
	//     
	//     
	0x18, 0x24, 0x24, 0x3F, 

	/* @252 'e' (4 pixels wide) */
	//     
	//     
	//  ###
	// ####
	// #   
	//  ###
	//     
	//     
	0x18, 0x2C, 0x2C, 0x2C, 

	/* @256 'f' (2 pixels wide) */
	//  #
	//  #
	// ##
	//  #
	//  #
	//  #
	//   
	//   
	0x04, 0x3F, 

	/* @258 'g' (4 pixels wide) */
	//     
	//     
	//  ###
	// #  #
	// #  #
	//  ###
	// #  #
	//  ## 
	0x58, 0xA4, 0xA4, 0x7C, 

	/* @262 'h' (3 pixels wide) */
	// #  
	// #  
	// ###
	// # #
	// # #
	// # #
	//    
	//    
	0x3F, 0x04, 0x3C, 

	/* @265 'i' (1 pixels wide) */
	// #
	//  
	// #
	// #
	// #
	// #
	//  
	//  
	0x3D, 

	/* @266 'j' (1 pixels wide) */
	// #
	//  
	// #
	// #
	// #
	// #
	// #
	// #
	0xFD, 

	/* @267 'k' (3 pixels wide) */
	// #  
	// #  
	// ###
	// #  
	// ## 
	// # #
	//    
	//    
	0x3F, 0x14, 0x24, 

	/* @270 'l' (1 pixels wide) */
	// #
	// #
	// #
	// #
	// #
	// #
	//  
	//  
	0x3F, 

	/* @271 'm' (5 pixels wide) */
	//      
	//      
	// #####
	// # # #
	// # # #
	// # # #
	//      
	//      
	0x3C, 0x04, 0x3C, 0x04, 0x3C, 

	/* @276 'n' (3 pixels wide) */
	//    
	//    
	// ###
	// # #
	// # #
	// # #
	//    
	//    
	0x3C, 0x04, 0x3C, 

	/* @279 'o' (4 pixels wide) */
	//     
	//     
	//  ##
	// #  #
	// #  #
	//  ##
	//     
	//     
	0x18, 0x24, 0x24, 0x18, 

	/* @283 'p' (3 pixels wide) */
	//    
	//    
	// ###
	// # #
	// # #
	// ###
	// #  
	// #  
	0xFC, 0x24, 0x3C, 

	/* @286 'q' (4 pixels wide) */
	//     
	//     
	//  ###
	// #  #
	// #  #
	//  ###
	//    #
	//    #
	0x18, 0x24, 0x24, 0xFC, 

	/* @290 'r' (2 pixels wide) */
	//   
	//   
	// ##
	// # 
	// # 
	// # 
	//   
	//   
	0x3C, 0x04, 

	/* @292 's' (4 pixels wide) */
	//     
	//     
	// ####
	// ##  
	//   ##
	// ### 
	//     
	//     
	0x2C, 0x2C, 0x34, 0x14, 

	/* @296 't' (2 pixels wide) */
	//   
	//  #
	// ##
	//  #
	//  #
	//  #
	//   
	//   
	0x04, 0x3E, 

	/* @298 'u' (3 pixels wide) */
	//    
	//    
	// # #
	// # #
	// # #
	//  # 
	//    
	//    
	0x1C, 0x20, 0x1C, 

	/* @301 'v' (4 pixels wide) */
	//     
	//     
	// #  #
	//  ## 
	//  ## 
	//  ## 
	//     
	//     
	0x04, 0x38, 0x38, 0x04, 

	/* @305 'w' (6 pixels wide) */
	//       
	//       
	// # #  #
	// # ### 
	//  # ## 
	//  #  # 
	//       
	//       
	0x0C, 0x30, 0x0C, 0x18, 0x38, 0x04, 

	/* @311 'x' (4 pixels wide) */
	//     
	//     
	// ####
	//  ## 
	//  ## 
	// #  #
	//     
	//     
	0x24, 0x1C, 0x1C, 0x24, 

	/* @315 'y' (4 pixels wide) */
	//     
	//     
	// #  #
	//  ## 
	//  ## 
	//  ## 
	//  #  
	// #   
	0x84, 0x78, 0x38, 0x04, 

	/* @319 'z' (4 pixels wide) */
	//     
	//     
	// ####
	//   # 
	//  #  
	// ####
	//     
	//     
	0x24, 0x34, 0x2C, 0x24, 

	/* @323 '{' (2 pixels wide) */
	//  #
	//  #
	//  #
	// ##
	//  #
	//  #
	//  #
	//  #
	0x08, 0xFF, 

	/* @325 '|' (1 pixels wide) */
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	0xFF, 

	/* @326 '}' (2 pixels wide) */
	// # 
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	// # 
	0x81, 0x7E, 

	/* @328 '~' (4 pixels wide) */
	//  ###
	// #  #
	//     
	//     
	//     
	//     
	//     
	//     
	0x02, 0x01, 0x01, 0x03, 
};

/* Character descriptors for Microsoft Sans Serif 6pt */
/* { [Char width in bits], [Offset into microsoftSansSerif_6ptCharBitmaps in bytes] } */
const charDescriptor MSSansSerif_6ptDescriptors[] = 
{
	{2, 0}, 		/*   */ 
	{1, 2}, 		/* ! */ 
	{3, 3}, 		/* " */ 
	{4, 6}, 		/* # */ 
	{4, 10}, 		/* $ */ 
	{7, 14}, 		/* % */ 
	{5, 21}, 		/* & */ 
	{1, 26}, 		/* ' */ 
	{2, 27}, 		/* ( */ 
	{2, 29}, 		/* ) */ 
	{3, 31}, 		/* * */ 
	{4, 34}, 		/* + */ 
	{1, 38}, 		/* , */ 
	{2, 39}, 		/* - */ 
	{1, 41}, 		/* . */ 
	{2, 42}, 		/* / */ 
	{4, 44}, 		/* 0 */ 
	{2, 48}, 		/* 1 */ 
	{4, 50}, 		/* 2 */ 
	{4, 54}, 		/* 3 */ 
	{4, 58}, 		/* 4 */ 
	{4, 62}, 		/* 5 */ 
	{4, 66}, 		/* 6 */ 
	{4, 70}, 		/* 7 */ 
	{4, 74}, 		/* 8 */ 
	{4, 78}, 		/* 9 */ 
	{1, 82}, 		/* : */ 
	{1, 83}, 		/* ; */ 
	{4, 84}, 		/* < */ 
	{4, 88}, 		/* = */ 
	{4, 92}, 		/* > */ 
	{4, 96}, 		/* ? */ 
	{8, 100}, 		/* @ */ 
	{5, 108}, 		/* A */ 
	{4, 113}, 		/* B */ 
	{5, 117}, 		/* C */ 
	{4, 122}, 		/* D */ 
	{4, 126}, 		/* E */ 
	{4, 130}, 		/* F */ 
	{6, 134}, 		/* G */ 
	{4, 140}, 		/* H */ 
	{1, 144}, 		/* I */ 
	{3, 145}, 		/* J */ 
	{4, 148}, 		/* K */ 
	{3, 152}, 		/* L */ 
	{5, 155}, 		/* M */ 
	{4, 160}, 		/* N */ 
	{6, 164}, 		/* O */ 
	{4, 170}, 		/* P */ 
	{6, 174}, 		/* Q */ 
	{4, 180}, 		/* R */ 
	{5, 184}, 		/* S */ 
	{5, 189}, 		/* T */ 
	{4, 194}, 		/* U */ 
	{5, 198}, 		/* V */ 
	{7, 203}, 		/* W */ 
	{5, 210}, 		/* X */ 
	{5, 215}, 		/* Y */ 
	{5, 220}, 		/* Z */ 
	{1, 225}, 		/* [ */ 
	{2, 226}, 		/* \ */ 
	{2, 228}, 		/* ] */ 
	{3, 230}, 		/* ^ */ 
	{4, 233}, 		/* _ */ 
	{2, 237}, 		/* ` */ 
	{4, 239}, 		/* a */ 
	{3, 243}, 		/* b */ 
	{4, 246}, 		/* c */ 
	{4, 250}, 		/* d */ 
	{4, 254}, 		/* e */ 
	{2, 258}, 		/* f */ 
	{4, 260}, 		/* g */ 
	{3, 264}, 		/* h */ 
	{1, 267}, 		/* i */ 
	{1, 268}, 		/* j */ 
	{3, 269}, 		/* k */ 
	{1, 272}, 		/* l */ 
	{5, 273}, 		/* m */ 
	{3, 278}, 		/* n */ 
	{4, 281}, 		/* o */ 
	{3, 285}, 		/* p */ 
	{4, 288}, 		/* q */ 
	{2, 292}, 		/* r */ 
	{4, 294}, 		/* s */ 
	{2, 298}, 		/* t */ 
	{3, 300}, 		/* u */ 
	{4, 303}, 		/* v */ 
	{6, 307}, 		/* w */ 
	{4, 313}, 		/* x */ 
	{4, 317}, 		/* y */ 
	{4, 321}, 		/* z */ 
	{2, 325}, 		/* { */ 
	{1, 327}, 		/* | */ 
	{2, 328}, 		/* } */ 
	{4, 330}, 		/* ~ */  
};

FontInfo MSSanSerif_6pt = 
{
	MSSansSerif_6pt,
  6,
  MSSansSerif_6ptDescriptors
};


