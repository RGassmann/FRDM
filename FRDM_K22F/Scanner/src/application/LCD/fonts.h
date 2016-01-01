#ifndef FONTS_T
#define FONTS_T

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
#include <stdint.h>

//================================================================ Datatypes ===

/*! \brief Universal font structure for ordinary strings
 */
typedef struct {
    const uint8_t width;        /**< @brief max. character width, for monospaced font */
    const uint8_t height;       /**< @brief character height */
    const uint8_t first_char;   /**< @brief first printable character in data */
    const uint8_t last_char;    /**< @brief last printable character in data */
    const uint8_t *char_width;  /**< @brief list of character widths, proportional font */
    //~ const uint8_t char_width[0x7E - 0x20 + 1]; /**< @brief character width, proportional font */
    const uint16_t *pixel_offset;   /**< @brief list of pixel offsets in data */
    const uint8_t *data;        /**< @brief pixel data */
} LCD_FONT;

/*! \brief Special font structure for big digits
 */
typedef struct {
    const uint8_t width;          /**< @brief character width */
    const uint8_t height;         /**< @brief character height */
    const uint8_t *char_width;
    const uint8_t *column_offset;
    const uint8_t *data;          /**< @brief pixel data */
} LCD_FONT_DIGITAL;

//========================================================== Fonts available ===
extern const LCD_FONT Font8x11; /**< @brief Font containing characters of size 8x11 pixels */
extern const LCD_FONT Font13x16;/**< @brief Font containing characters of size 13x16 pixels */
extern const LCD_FONT_DIGITAL FontDigital; /**< @brief Font containing big digits */
#endif // FONTS_T
