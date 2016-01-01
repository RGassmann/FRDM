#ifndef ILI9341_ADVANCED_H
#define ILI9341_ADVANCED_H

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
#include "lcd_basic.h"
#include "fonts.h"

//================================================= String alignment options ===
#define LCD_LEFT                    -1  /**< @brief string is left justified */
#define LCD_CENTER                  -2  /**< @brief string is centered */
#define LCD_RIGHT                   -3  /**< @brief string is right justified */


typedef struct {
    const uint8_t width;
    const uint8_t height;
    const uint8_t *data;
} LCD_BITMAP;

extern const LCD_BITMAP IPOD_SYMBOL;
extern const LCD_BITMAP IR_SYMBOL;
extern const LCD_BITMAP FM_SYMBOL;

void lcd_set_font(const LCD_FONT *font, uint8 use_monospace);
uint16 lcd_write_string(uint16 x, uint16 y, char *str);
uint16 lcd_get_string_width(char *str);
uint16 lcd_write_center_string(uint16 y, char *str);
uint16 lcd_write_char(uint16 x, uint16 y, char c);
uint16 lcd_write_digit(uint16 x, uint16 y, uint16 digit, uint16 scale);
void lcd_draw_battery(int16 x, int16 y, uint8 percentage);
void lcd_draw_bitmap(int16 x, int16 y, const LCD_BITMAP *bitmap);

#endif // ILI9341_ADVANCED_H
