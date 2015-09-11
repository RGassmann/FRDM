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


//============================================================= Dependencies ===
#include "lcd.h"

//================================================ External global variables ===
extern struct {
    PIXEL_COLOR BG_Color;
    PIXEL_COLOR Font_Color;
} d_theme;  // Theme colors

//======================================================== Private variables ===
volatile LCD_FONT const *d_font = &Font8x11;
volatile uint8_t d_use_monospace = 0;

//================================================================== Bitmaps ===
/*const LCD_BITMAP IPOD_SYMBOL = {
    8,  // width
    11, // height
    (uint8_t*)"\xFF\xF0\xFE\x19\xC2\xD8\x5B\x0C\xE1\xFF\xFF"
};

const LCD_BITMAP IR_SYMBOL = {
    8, // width
    11, // height
    (uint8_t*)"\x00\x00\x00\x8F\xA4\x05\x3E\x95\x08\xD8\x00"
};

const LCD_BITMAP FM_SYMBOL = {
    9, // width
    11, // height
    (uint8_t*)"\x03\xE0\x50\x8A\x24\x05\x3E\x94\x08\xF8\x10\x03\xE0"
};
*/
//==============================================================================
/*! \brief Set font type.
 *
 *  This function sets the desired font for the next strings to write.
 *
 *  @param font Pointer to the desired font structure.
 *              The following fonts are now available:
 *              \ref Font8x11, \ref Font13x16.
 *  @param use_monospace Defines if the font has to be written in monospaced or
 *                      proportional style. 0: proportional, 1: monospaced.
 */
void lcd_set_font(const LCD_FONT *font, uint8 use_monospace)
{
    d_font = font;
    d_use_monospace = use_monospace ? 1 : 0;
}

//==============================================================================
/*! \brief Write a character with small font.
 *
 *  This function writes a single character at the given position.
 *
 *  @param x Offset to the left.
 *  @param y Offset to the top.
 *  @param c ASCII character.
 *  @return The function returns the width of the character written.
 */
uint16 lcd_write_char(uint16 x, uint16 y, char c)
{
    uint16_t pixels_used=0;
    uint16_t pixels_to_use;
    uint16_t index;
    uint16_t font_byte;
    uint8_t font_mask;


    // if not a printable character
    if((c < d_font->first_char) || (c > d_font->last_char)) {
        return 0;                       // width=0, abort
    }
    index = c-d_font->first_char;       // get character index in font
    if(d_use_monospace) {
        lcd_set_address(x,x+d_font->width-1,y,y+d_font->height-1);
    }
    else {
        lcd_set_address(x,x+d_font->char_width[index],y,y+d_font->height-1);
    }
    // add half of the space columns in front of the character to balance the spacing
    if(d_use_monospace) {
        // one or more space columns at the beginning
        pixels_to_use = d_font->height*((d_font->width - d_font->char_width[index])/2);
        for(pixels_used=0; pixels_used < pixels_to_use; pixels_used++) {
            lcd_send_mdata(d_theme.BG_Color.Word);
        }
    }

    // add number of pixels occupied by the current character
    pixels_to_use = pixels_used
                    + d_font->pixel_offset[index+1]
                    - d_font->pixel_offset[index];

    // byte offset in data array
    font_byte = d_font->pixel_offset[index]>>3;
    // bit offset in current data byte
    font_mask = 1u << (d_font->pixel_offset[index]&0x07);

    // loop on all pixels to draw
    for( ; pixels_used<pixels_to_use; pixels_used++) {
        if(d_font->data[font_byte] & font_mask) { // if pixel value is one
            lcd_send_mdata(d_theme.Font_Color.Word);
        }
        else {                      // or pixel value is zero
            lcd_send_mdata(d_theme.BG_Color.Word);
        }
        // increment font counters
        if(font_mask<128) {
            font_mask <<= 1;
        }
        else {
            font_mask = 1;
            font_byte++;
        }
    }

    if(d_use_monospace) {
        pixels_to_use = d_font->height*d_font->width; // fixed width
    }
    else {
        pixels_to_use += d_font->height; // add one space column
    }

    // one or more space columns at the end
    for(; pixels_used<pixels_to_use; pixels_used++) {
        lcd_send_mdata(d_theme.BG_Color.Word);
    }

    if(d_use_monospace) {
        return d_font->width;
    }
    else {
        return d_font->char_width[index] + 1;
    }
}

uint16 lcd_get_string_width(char *str)
{
    int width = 0;
    while(*str) {                       // go through all characters in string
        if((*str < d_font->first_char) || (*str > d_font->last_char)) {  // if not printable character
            break;                      // abort
        }
        // else add current character width + 1 space column
        if(d_use_monospace) {
            width += d_font->width;
        }
        else {
            width += d_font->char_width[*str - d_font->first_char] + 1;
        }
        str++;
    }
    return width;
}

uint16 lcd_write_string(uint16 x, uint16 y, char *str)
{
    if(*str==0) {                       // if string has zero length
        return 0;                       // abort
    }

   /* if ( x<0 ) {                           // if special alignment required
        if(x==LCD_LEFT) {               // if aligned left
            x = 0;
        }
        else if(x==LCD_CENTER) {        // if centered
            x = (LCD_WIDTH_X-lcd_get_string_width(str))/2;
        }
        else if(x==LCD_RIGHT) {         // if aligned right
            x = LCD_WIDTH_X-lcd_get_string_width(str);
        }
        else {
            return -3;                  // illegal value, abort
        }
    }*/

    while(*str >= d_font->first_char) {  // while end of string not reached and printable char
        x += lcd_write_char(x,y,*str++); // write next character
    }
    return 0;
}

uint16 lcd_write_center_string(uint16 y, char *str)
{
    return lcd_write_string((d_width_x-lcd_get_string_width(str))>>1,y,str);
}

/*
void lcd_draw_bitmap(int16 x, int16 y, const LCD_BITMAP *bitmap)
{
    uint16_t used;
    uint8_t index, mask;
    PIXEL_COLOR *pixel_odd = &d_theme.BG_Color;
    PIXEL_COLOR *pixel_even = &d_theme.BG_Color;

    lcd_send_cmd(LCD_PASET);            // page address set
    lcd_send_data(x);
    lcd_send_data(x+bitmap->width-1);
    lcd_send_cmd(LCD_CASET);            // column address set
    lcd_send_data(y);
    lcd_send_data(y+bitmap->height-1);
    lcd_send_cmd(LCD_MEMWRITE);         // memory write

    index = 0;
    mask = BIT7;
    for(used=0; used<(bitmap->width*bitmap->height); used++) {
        if(used&0x01) {          // if current pixel number is odd
            if(bitmap->data[index] & mask) { // if pixel value is one
                pixel_odd = &d_theme.Font_Color;
            }
            else {                      // or pixel value is zero
                pixel_odd = &d_theme.BG_Color;
            }
            // write the preceding even and current odd pixel
            lcd_send_data(pixel_even->even_upper);
            lcd_send_data(pixel_even->even_lower | pixel_odd->odd_upper);
            lcd_send_data(pixel_odd->odd_lower);
        }
        else {                          // or if current pixel number is even
            if(bitmap->data[index] & mask) { // if pixel value is one
                pixel_even = &d_theme.Font_Color;
            }
            else {                      // or pixel value is zero
                pixel_even = &d_theme.BG_Color;
            }
        }

        // increment counters
        if(mask>1) {
            mask >>= 1;
        }
        else {
            mask = BIT7;
            index++;
        }
    }
    if(used&0x01) {              // if the last pixel is odd
        lcd_send_data(pixel_even->even_upper);
        lcd_send_data(pixel_even->even_lower);
        lcd_send_cmd(LCD_NOPER);
    }
}

void lcd_draw_battery(int16 x, int16 y, int8 percentage)
{
    if(percentage==100) {
        lcd_draw_rect(x,x+12,y,y+8, LCD_GREEN, 1);      // cap
        lcd_draw_rect(x+13,x+14,y+2,y+6, LCD_GREEN, 1); // area
    }
    else {
        lcd_draw_rect(x+(percentage>>3),x+12,y,y+8, LCD_GRAY, 1);   // void
        lcd_draw_rect(x+13,x+14,y+2,y+6, LCD_GRAY, 1);              // cap
        lcd_draw_rect(x,x+(percentage>>3),y,y+8, LCD_GREEN, 1);     // area
    }

    if(percentage==100) {
        lcd_write_digit(x+15, y+1, 1, 1);       // 1
        lcd_write_digit(x+15+5, y+1, 0, 1);     // 0
        lcd_write_digit(x+15+10, y+1, 0, 1);    // 0
    }
    else if(percentage<10) {
        // clear first two digits
        lcd_draw_rect(x+15,x+25,y,y+8, lcd_get_background_color(),1);
        // third digit
        lcd_write_digit(x+15+10, y+1, percentage, 1);
    }
    else {
        // clear first digit
        lcd_draw_rect(x+15,x+20,y,y+8, lcd_get_background_color(),1);
        // second digit
        lcd_write_digit(x+15+5, y+1, (percentage/10)%10, 1);
        // third digit
        lcd_write_digit(x+15+10, y+1, percentage%10, 1);
    }
    lcd_write_digit(x+15+15, y+1, 11, 1); // %
}
*/
/*
int16 lcd_write_digit(int16 x, int16 y, int16 digit, int16 scale)
{
    uint8_t column, row;
    uint8_t data;
    uint16_t font_color;
    uint8_t x_, y_;

    if(scale==0) {                       // if scaling is too small
        scale = 1;                      // limit scale to 1
    }

    lcd_draw_rect(                      // first, clear the area under the digit
        x,
        x+scale*(FontDigital.char_width[digit]+1)-1,
        y,
        y+scale*(FontDigital.height)-(scale==1?1:2),
        lcd_get_background_color(),
        1
    );

    font_color = lcd_get_font_color();      // determine current font color

    x_ = x;
    // for each column in digit
    for(column=0; column<FontDigital.char_width[digit]; column++) {
        // read column data
        data = FontDigital.data[FontDigital.column_offset[digit]+column];
        y_ = y;                             // reset vertical position
        for(row=0; row<FontDigital.height; row++) { // for each row in digit
            if(data&0x80) {                 // if element is available
                lcd_draw_rect(x_, x_+scale-(scale==1?1:2), y_, y_+scale-(scale==1?1:2), font_color, 1);
            }
            data <<= 1;                     // shift to next element
            y_ += scale;                    // move to next vertical position
        }
        x_ += scale;                        // move to next horizontal position
    }
    return x_-x+scale;                      // return digit width occupied
}*/
