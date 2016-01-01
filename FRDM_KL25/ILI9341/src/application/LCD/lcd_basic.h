#ifndef LCD_BASIC_H
#define LCD_BASIC_H

#include "lcd.h"
/*------------------------------------------------------------------------------
/
/   This file is part of Electronics4you FM Transmitter.
/
/   Copyright (C) 2012-2013:
/       Roman Gassmann, rog@gassmann-engineering.ch
/       Nicola Ramagnano, nicola.ramagnano@electronics4you.cc
/
/   This program is free software: you can redistribute it and/or modify
/   it under the terms of the GNU General Public License as published by
/   the Free Software Foundation, either version 3 of the License, or
/   (at your option) any later version.
/
/   This program is distributed in the hope that it will be useful,
/   but WITHOUT ANY WARRANTY; without even the implied warranty of
/   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
/   GNU General Public License for more details.
/
/   You should have received a copy of the GNU General Public License
/   along with this program.  If not, see http://www.gnu.org/licenses/gpl-3.0
/
/-----------------------------------------------------------------------------*/

/*! \file */

//============================================================= Dependencies ===

//========================================================= Build parameters ===


//==============================================================================
#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_MADCTL_MY   0x80
#define ILI9341_MADCTL_MX   0x40
#define ILI9341_MADCTL_MV   0x20
#define ILI9341_MADCTL_ML   0x10
#define ILI9341_MADCTL_BGR  0x08
#define ILI9341_MADCTL_MH   0x04
#define ILI9341_PIXFMT  0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7


#define ILI9341_PWCTRA  0xCB
#define ILI9341_PWCTRB  0xCF

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1

#define ILI9341_DRVTCTRA  0xE8
#define ILI9341_DRVTCTRB  0xEA
#define ILI9341_PWRONCTR  0xED
#define ILI9341_PMPRTCTR  0xE7


/*
#define ILI9341_PWCTR6  0xFC
*/

/* some RGB color definitions                                                 */
#define ILI9341_Black           0x0000      /*   0,   0,   0 */
#define ILI9341_Navy            0x000F      /*   0,   0, 128 */
#define ILI9341_DarkGreen       0x03E0      /*   0, 128,   0 */
#define ILI9341_DarkCyan        0x03EF      /*   0, 128, 128 */
#define ILI9341_Maroon          0x7800      /* 128,   0,   0 */
#define ILI9341_Purple          0x780F      /* 128,   0, 128 */
#define ILI9341_Olive           0x7BE0      /* 128, 128,   0 */
#define ILI9341_LightGrey       0xC618      /* 192, 192, 192 */
#define ILI9341_DarkGrey        0x7BEF      /* 128, 128, 128 */
#define ILI9341_Blue            0x001F      /*   0,   0, 255 */
#define ILI9341_Green           0x07E0      /*   0, 255,   0 */
#define ILI9341_Cyan            0x07FF      /*   0, 255, 255 */
#define ILI9341_Red             0xF800      /* 255,   0,   0 */
#define ILI9341_Magenta         0xF81F      /* 255,   0, 255 */
#define ILI9341_Yellow          0xFFE0      /* 255, 255,   0 */
#define ILI9341_White           0xFFFF      /* 255, 255, 255 */
#define ILI9341_Orange          0xFD20      /* 255, 165,   0 */
#define ILI9341_GreenYellow     0xAFE5      /* 173, 255,  47 */

//Other Colors
#define ILI9341_LightGray       0xBCEF
#define ILI9341_GRAY1    0x8410
#define ILI9341_GRAY2    0x4208

#define DEGREE_0    0
#define DEGREE_90    1
#define DEGREE_180    2
#define DEGREE_270    3


//================================================================= SETTINGS ===
/**
 * @defgroup LCD_SETTINGS LCD settings
 *
 * @{
 */
#define LCD_WIDTH_X                 240uL       /**< @brief horizontal width in pixels */
#define LCD_WIDTH_Y                 320uL       /**< @brief vertical width in pixels */
#define LCD_DEFAULT_FONT_COLOR      ILI9341_Black   /**< @brief default font color */
#define LCD_DEFAULT_BKG_COLOR       ILI9341_White   /**< @brief default background color */
#define LCD_MAX_CONTRAST_LEVEL      5           /**< @brief maximum contrast level */
/** @} */

//================================================================ Datatypes ===

typedef enum{
    rot0,
    rot90,
    rot180,
    rot270
} ORIENTATION;

/*! @brief Convenient data type for the handling of pixel color data
 */

typedef union {
    uint8 Byte[2];
    struct{
        uint8  Hi;
        uint8  Lo;
    }Bytes;
    uint16 Word;
} PIXEL_COLOR;

extern volatile uint16  d_width_x;
extern volatile uint16  d_width_y;

//=========================================== Prototypes of global functions ===
void lcd_set_font_color(uint16 color);
uint16 lcd_get_font_color(void);
void lcd_set_background_color(uint16 color);
uint16 lcd_get_background_color(void);
void lcd_init(void);
void lcd_send_cmd(uint8 cmd);
void lcd_send_data(uint8 data);
void lcd_send_mdata(uint16 data);
void lcd_clear(uint16 color);
void lcd_shutdown(void);
void lcd_set_address(uint16 x1,uint16 x2,uint16 y1,uint16 y2);
void lcd_set_pixel(uint16 x, uint16 y, uint16 color);
void lcd_draw_line(uint16 x1, uint16 x2, uint16 y1, uint16 y2, uint16 color);
void lcd_draw_rect(uint16 x1, uint16 x2, uint16 y1, uint16 y2, uint16 color, uint8 fill);
void lcd_draw_rect_full_height(uint16 x, uint16 width, uint16 color, uint8 fill);
void lcd_draw_rect_full_width(uint16 y, uint16 height, uint16 color, uint8 fill);
void lcd_draw_circle(uint16 x, uint16 y, uint16 r, uint16 color);
void lcd_set_contrast(uint8 contrast);
uint8 lcd_get_contrast(void);
void lcd_set_Orientation(uint8 orientation);
#endif // NOKIA6100_BASIC_H
