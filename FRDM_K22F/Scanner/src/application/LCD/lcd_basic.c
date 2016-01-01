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
#include "lcd_basic.h"
#include <MK22F51212_IBG.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define LCD_RESET       PIN(PC,11)
//#define LCD_CMD_DATA    PIN(PD,4)

//======================================================== Private variables ===
struct {
    PIXEL_COLOR BG_Color;
    PIXEL_COLOR Font_Color;
} d_theme = {.BG_Color.Word = ILI9341_White, .Font_Color.Word = ILI9341_Black};

volatile uint8 d_contrast = LCD_MAX_CONTRAST_LEVEL;
volatile uint16  d_width_x = LCD_WIDTH_Y;
volatile uint16  d_width_y = LCD_WIDTH_X;
volatile uint8 pcs_command = 0x03;
volatile uint8 pcs_data = 0x01;

volatile ORIENTATION d_orientation = rot90;

//============================================ Prototypes of local functions ===
void lcd_reset_on(void);
void lcd_reset_off(void);
void lcd_chipselect_on(void);
void lcd_chipselect_off(void);
void lcd_cmddata_on(void);
void lcd_cmddata_off(void);
void lcd_spi_init(void);

//======================================================== Private functions ===

// enable LCD reset
void lcd_reset_on(void)
{
	//PTD->PSOR = (1U << 3U);
	OUTPUT_SET(LCD_RESET,POSITION_ON);
}

// disable LCD reset
void lcd_reset_off(void)
{
	//PTD->PCOR = (1U << 3U);
	OUTPUT_SET(LCD_RESET,POSITION_OFF);
}

// enable LCD chipselect
void lcd_chipselect_on(void)
{
   // PTA->PCOR = (1U << 5U);
}

// disable LCD chipselect
void lcd_chipselect_off(void)
{
    //PTA->PSOR = (1U << 5U);
}

// enable LCD cmddata
void lcd_cmddata_on(void)
{
//    OUTPUT_SET(LCD_CMD_DATA,POSITION_ON);
   // PTD->PSOR = (1U << 0U);
}

// disable LCD cmddata
void lcd_cmddata_off(void)
{
   // OUTPUT_SET(LCD_CMD_DATA,POSITION_OFF);
    //PTD->PCOR = (1U << 0U);
}

// initialize LCD SPI pins or hardware
void lcd_spi_init(void)
{
    INIT_OUTPUT(LCD_RESET);

    //INIT SPI
    SIM->SCGC6 |= SIM_SCGC6_SPI0_MASK;
	SPI0_MCR = SPI_MCR_MDIS_MASK | SPI_MCR_HALT_MASK | SPI_MCR_PCSIS(0x1F);
	SPI0_CTAR0 = SPI_CTAR_FMSZ(7) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR_MASK | SPI_CTAR_CSSCK(0);
	SPI0_CTAR1 = SPI_CTAR_FMSZ(15) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR_MASK | SPI_CTAR_CSSCK(0);
	SPI0_MCR = SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(0x1F);
	//SPCR.enable_pins();
     //SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK;       //Enable SPI1 clock

    PORTD_PCR0 = PORT_PCR_MUX(0x2);    //Set PTD0 to mux 2 [SPI0_PCS0]
    PORTD_PCR1 = PORT_PCR_MUX(0x2);    //Set PTD1 to mux 2 [SPI0_SCK]
    PORTD_PCR2 = PORT_PCR_MUX(0x2);    //Set PTD2 to mux 2 [SPI0_MOSI]
    PORTD_PCR3 = PORT_PCR_MUX(0x2);    //Set PTD3 to mux 2 [SPI0_MISO]
    PORTD_PCR4 = PORT_PCR_MUX(0x2);    //Set PTD4 to mux 2 [SPI0_PCS1]

    //PORTC_PCR11 = PORT_PCR_MUX(0x1);   //Set PTC11 to mux 1 [GPIO]


//    while ( !(SPI0->S & SPI_S_SPTEF_MASK) )__asm__ __volatile__("nop");
}


	void waitFifoNotFull(void) {
		uint32_t sr;
		uint32_t tmp __attribute__((unused));
		do {
			sr = SPI0_SR;
			if (sr & 0xF0) tmp = SPI0_POPR;  // drain RX FIFO
		} while ((sr & (15 << 12)) > (3 << 12));
	}
	void waitFifoEmpty(void) {
		uint32_t sr;
		uint32_t tmp __attribute__((unused));
		do {
			sr = SPI0_SR;
			if (sr & 0xF0) tmp = SPI0_POPR;  // drain RX FIFO
		} while ((sr & 0xF0F0) > 0);             // wait both RX & TX empty
	}
	void  waitTransmitComplete(void){
		uint32_t tmp __attribute__((unused));
		while (!(SPI0_SR & SPI_SR_TCF_MASK)) ; // wait until final output done
		tmp = SPI0_POPR;                  // drain the final RX FIFO word
	}
	void waitTransmitComplete_last(uint32_t mcr) {
		uint32_t tmp __attribute__((unused));
		while (1) {
			uint32_t sr = SPI0_SR;
			if (sr & SPI_SR_EOQF_MASK) break;  // wait for last transmit
			if (sr &  0xF0) tmp = SPI0_POPR;
		}
		SPI0_SR = SPI_SR_EOQF_MASK;
		SPI0_MCR = mcr;
		while (SPI0_SR & 0xF0) {
			tmp = SPI0_POPR;
		}
	}
	void writecommand_cont(uint8_t c)  {
		SPI0_PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT_MASK;
		waitFifoNotFull();
	}
	void writedata8_cont(uint8_t c)  {
		SPI0_PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT_MASK;
		waitFifoNotFull();
	}
	void writedata16_cont(uint16_t d) {
		SPI0_PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_CONT_MASK;
		waitFifoNotFull();
	}
	void writecommand_last(uint8_t c){
		uint32_t mcr = SPI0_MCR;
		SPI0_PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ_MASK;
		waitTransmitComplete_last(mcr);
	}
	void writedata8_last(uint8_t c)  {
		uint32_t mcr = SPI0_MCR;
		SPI0_PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ_MASK;
		waitTransmitComplete_last(mcr);
	}
	void writedata16_last(uint16_t d){
		uint32_t mcr = SPI0_MCR;
		SPI0_PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_EOQ_MASK;
		waitTransmitComplete_last(mcr);
	}

/*// prepare to send a command byte to LCD
void lcd_send_cmd(uint8 cmd)
{
    while ( !(SPI1->S & SPI_S_SPTEF_MASK) );
    lcd_cmddata_off(); //CMD
    lcd_send(cmd);                    // disable CS
}

// prepare to send a data byte to LCD
void lcd_send_data(uint8 data)
{
    while ( !(SPI1->S & SPI_S_SPTEF_MASK) );
    lcd_cmddata_on(); //CMD
    lcd_send(data);
}

// prepare to send two data byte to LCD
void lcd_send_mdata(uint16 data)
{
    while ( !(SPI1->S & SPI_S_SPTEF_MASK) );
    lcd_cmddata_on(); //CMD
    lcd_send((data>>8)& 0xFF);
    lcd_send(data & 0xFF);
}
*/
//========================================================= Public functions ===


void lcd_set_font_color(uint16 color)
{
    d_theme.Font_Color.Word = color;
}

uint16 lcd_get_font_color(void)
{
    return d_theme.Font_Color.Word;
}

void lcd_set_background_color(uint16 color)
{
    d_theme.BG_Color.Word = color;
}

uint16 lcd_get_background_color(void)
{
    return d_theme.BG_Color.Word;
}

void delay_ms(uint16 val)
{
    volatile unsigned int i,j;

    for (i = 0U; i < val; i++) {
        for (j = 0U; j < 1000U; j++) {
            __asm__("nop");
        }
    }
}

//==============================================================================
/*! \brief Initialize the LCD.
 *
 *  The LCD is reset and initialized with all predefined settings.
 */
void lcd_init(void)
{

lcd_spi_init();

    lcd_reset_on();
	//R_ESET=1;
	//for(i=120; i > 0; i--);
	delay_ms(1);
	lcd_reset_off();
	delay_ms(10);
	//for(i=1200; i > 0; i--);
	lcd_reset_on();
	delay_ms(1000);
	//for(i=12000; i > 0; i--);

static const uint8_t init_commands[] = {
	4, 0xEF, 0x03, 0x80, 0x02,
	4, 0xCF, 0x00, 0XC1, 0X30,
	5, 0xED, 0x64, 0x03, 0X12, 0X81,
	4, 0xE8, 0x85, 0x00, 0x78,
	6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
	2, 0xF7, 0x20,
	3, 0xEA, 0x00, 0x00,
	2, ILI9341_PWCTR1, 0x23, // Power control
	2, ILI9341_PWCTR2, 0x10, // Power control
	3, ILI9341_VMCTR1, 0x3e, 0x28, // VCM control
	2, ILI9341_VMCTR2, 0x86, // VCM control2
	2, ILI9341_MADCTL, 0x48, // Memory Access Control
	2, ILI9341_PIXFMT, 0x55,
	3, ILI9341_FRMCTR1, 0x00, 0x18,
	4, ILI9341_DFUNCTR, 0x08, 0x82, 0x27, // Display Function Control
	2, 0xF2, 0x00, // Gamma Function Disable
	2, ILI9341_GAMMASET, 0x01, // Gamma curve selected
	16, ILI9341_GMCTRP1, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
		0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
	16, ILI9341_GMCTRN1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
		0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
	0
};


const uint8_t *addr = init_commands;
	while (1) {
		uint8_t count = *addr++;
		if (count-- == 0) break;
		writecommand_cont(*addr++);
		while (count-- > 0) {
			writedata8_cont(*addr++);
		}
	}
	writecommand_last(ILI9341_SLPOUT);    // Exit Sleep

	writecommand_last(ILI9341_DISPON); // Display on


    /*lcd_send_cmd(ILI9341_PWCTRA);
        lcd_send_data(0x39);
        lcd_send_data(0x2C);
        lcd_send_data(0x00);
        lcd_send_data(0x34);
        lcd_send_data(0x02);

    lcd_send_cmd(ILI9341_PWCTRB);
        lcd_send_data(0x00);
        lcd_send_data(0X81);
        lcd_send_data(0X30);

    lcd_send_cmd(ILI9341_DRVTCTRA);
        lcd_send_data(0x85);
        lcd_send_data(0x00);
        lcd_send_data(0x78);

    lcd_send_cmd(ILI9341_DRVTCTRB);
        lcd_send_data(0x66);
        lcd_send_data(0x00);

    lcd_send_cmd(ILI9341_PWRONCTR);
        lcd_send_data(0x55);
        lcd_send_data(0x01);
        lcd_send_data(0X23);
        lcd_send_data(0X01);

    lcd_send_cmd(ILI9341_PMPRTCTR);
        lcd_send_data(0x10);

    lcd_send_cmd(ILI9341_PWCTR1);    //Power control
        lcd_send_data(0x21);   //VRH[5:0]

    lcd_send_cmd(ILI9341_PWCTR2);    //Power control
        lcd_send_data(0x10);   //SAP[2:0];BT[3:0]

    lcd_send_cmd(ILI9341_VMCTR1);    //VCM control
        lcd_send_data(0x31);
        lcd_send_data(0x3C);

    lcd_send_cmd(ILI9341_VMCTR2);    //VCM control2
        lcd_send_data(0xC0);  //--0x86

    lcd_set_Orientation(rot0);


    lcd_send_cmd(ILI9341_PIXFMT);
        lcd_send_data(0x55);

    lcd_send_cmd(ILI9341_FRMCTR1);
        lcd_send_data(0x00);
        lcd_send_data(0x11);

    lcd_send_cmd(ILI9341_FRMCTR2);
        lcd_send_data(0x00);
        lcd_send_data(0x11);

    lcd_send_cmd(ILI9341_FRMCTR3);
        lcd_send_data(0x00);
        lcd_send_data(0x11);

    lcd_send_cmd(ILI9341_DFUNCTR);    // Display Function Control
        lcd_send_data(0x08);
        lcd_send_data(0x82);
        lcd_send_data(0x27);

    lcd_send_cmd(0xF2);    // 3Gamma Function Disable
        lcd_send_data(0x02);

    lcd_send_cmd(ILI9341_GAMMASET);    //Gamma curve selected
        lcd_send_data(0x01);

    lcd_send_cmd(ILI9341_SLPOUT);    //Exit Sleep*/

        delay_ms(100);

   // lcd_send_cmd(ILI9341_DISPON);    //Display on
  //  lcd_send_cmd(ILI9341_RAMWR);
}

void lcd_set_address(uint16 x1,uint16 x2,uint16 y1,uint16 y2)
{
    uint16 temp;

    if(x2<x1) {                         // if x2 is below x1
        temp = x2;                      // swap
        x2 = x1;                        // both x
        x1 = temp;                      // variables
    }
    if(y2<y1) {                         // if y2 is below y1
        temp = y2;                      // swap
        y2 = y1;                        // both y
        y1 = temp;                      // variables
    }
    x1 = MIN(d_width_x,x1);
    x2 = MIN(d_width_x,x2);
    y1 = MIN(d_width_y,y1);
    y2 = MIN(d_width_y,y2);
    writecommand_last(ILI9341_PASET);
    //lcd_send_cmd(ILI9341_PASET);
    writedata16_cont(x1);
    writedata16_last(x2);
    //lcd_send_mdata(x1);
    //lcd_send_mdata(x2);
    writecommand_last(ILI9341_CASET);
    writedata16_cont(y1);
    writedata16_last(y2);
    //lcd_send_cmd(ILI9341_CASET);
    //lcd_send_mdata(y1);
    //lcd_send_mdata(y2);

    //lcd_send_cmd(ILI9341_RAMWR);
    writecommand_last(ILI9341_RAMWR);
}

void lcd_set_pixel(uint16 x, uint16 y, uint16 color)
{
    lcd_set_address(x,x,y,y);
    writedata16_last(color);
}

void lcd_draw_line(uint16 x1, uint16 x2, uint16 y1, uint16 y2, uint16 color)
{
    if((x1 == x2) || (y1 == y2)){
        lcd_draw_rect(x1,x2,y1,y2,color,1);
    }else{
        uint16 t;
        int xerr=0,yerr=0,delta_x,delta_y,distance;
        int incx,incy,uRow,uCol;

        delta_x=x2-x1;
        delta_y=y2-y1;
        uRow=x1;
        uCol=y1;
        if(delta_x>0)incx=1;
        else if(delta_x==0)incx=0;
        else {incx=-1;delta_x=-delta_x;}
        if(delta_y>0)incy=1;
        else if(delta_y==0)incy=0;
        else{incy=-1;delta_y=-delta_y;}
        if( delta_x>delta_y)distance=delta_x;
        else distance=delta_y;
        for(t=0;t<=distance+1;t++ )
        {
            lcd_set_pixel(uRow,uCol, color);
            xerr+=delta_x ;
            yerr+=delta_y ;
            if(xerr>distance)
            {
                xerr-=distance;
                uRow+=incx;
            }
            if(yerr>distance)
            {
                yerr-=distance;
                uCol+=incy;
            }
        }
    }
}

void lcd_draw_circle(uint16 x, uint16 y, uint16 r, uint16 color)
{
    int a,b;
	int di;
	a=0;b=r;
	di=3-(r<<1);
	while(a<=b)
	{
		lcd_set_pixel(x-b,y-a,color);             //3
		lcd_set_pixel(x+b,y-a,color);             //0
		lcd_set_pixel(x-a,y+b,color);             //1
		lcd_set_pixel(x-b,y-a,color);             //7
		lcd_set_pixel(x-a,y-b,color);             //2
		lcd_set_pixel(x+b,y+a,color);             //4
		lcd_set_pixel(x+a,y-b,color);             //5
		lcd_set_pixel(x+a,y+b,color);             //6
		lcd_set_pixel(x-b,y+a,color);
		a++;
		if(di<0)di +=4*a+6;
		else
		{
			di+=10+4*(a-b);
			b--;
		}
		lcd_set_pixel(x+a,y+b,color);
	}
}

void lcd_draw_rect(uint16 x1, uint16 x2, uint16 y1, uint16 y2, uint16 color, uint8 fill)
{
    uint16 i;

    if(fill) {                          // if the rectangle has to be filled
        lcd_set_address(x1,x2,y1,y2);
        for(i=0; i<(((x2-x1+1)*(y2-y1+1)+1))-1; i++) {
            writedata16_cont(color);
        }
        writedata16_last(color);
    }
    else {                              // if not to be filled
        lcd_draw_rect(x1,x1,y1,y2,color,1); // draw left line
        lcd_draw_rect(x2,x2,y1,y2,color,1); // draw right line
        lcd_draw_rect(x1,x2,y1,y1,color,1); // draw bottom line
        lcd_draw_rect(x1,x2,y2,y2,color,1); // draw top line
    }
}

void lcd_draw_rect_full_height(uint16 x, uint16 width, uint16 color, uint8 fill)
{
    lcd_draw_rect(x,x+width-1,0,d_width_y,color,fill);
}
void lcd_draw_rect_full_width(uint16 y, uint16 height, uint16 color, uint8 fill)
{
    lcd_draw_rect(0,d_width_x,y,y+height-1,color,fill);
}

//==============================================================================
/*! \brief Clears the LCD screen.
 *
 *  The LCD screen is completely cleared by writing the given color to each
 *  pixel.
 */
void lcd_clear(uint16 color)
{
    int i,j;
    lcd_set_address(0,d_width_x-1,0,d_width_y-1);

    for (i=1;i<d_width_y;i++){
        for (j=0;j<d_width_x;j++){
            writedata16_cont(color);
        }
    }
    for (j=1;j<d_width_x;j++){
        writedata16_cont(color);
    }
    writedata16_last(color);
}

//==============================================================================
/*! \brief Power off the LCD.
 *
 *  Puts LCD in Sleep-Mode
 */
void lcd_sleep(void)
{
    writecommand_last(ILI9341_DISPOFF);
    delay_ms(20);
    writecommand_last(ILI9341_SLPIN);
}

/*! \brief Power off the LCD.
 *
 *  Wakes up the LCD if its in Sleep-Mode
 */
void lcd_wakeup(void)
{
    writecommand_last(ILI9341_SLPOUT);
    delay_ms(120);
    writecommand_last(ILI9341_DISPON);
}

//==============================================================================
/*! \brief Set LCD contrast.
 *
 *  It sets the LCD contrast to the desired value.
 *  @param contrast Value between 0 and \ref LCD_MAX_CONTRAST_LEVEL
 */
void lcd_set_contrast(uint8 contrast)
{
   /* if(contrast>LCD_MAX_CONTRAST_LEVEL) {
        d_contrast = LCD_MAX_CONTRAST_LEVEL;
    }
    else {
        d_contrast = contrast;
    }
    lcd_send_cmd(LCD_CONTRAST);         // write contrast
    lcd_send_data((-43*d_contrast)/LCD_MAX_CONTRAST_LEVEL-20);*/
}

//==============================================================================
/*! \brief Get current LCD contrast.
 *
 *  It returns the current LCD contrast.
 *  @return Value between 0 and \ref LCD_MAX_CONTRAST_LEVEL
 */
uint8_t lcd_get_contrast(void)
{
    return d_contrast;
}

//==============================================================================
/*! \brief Set LCD orientation.
 *
 *  It sets the orientation of  the LCD.
 *  @param contrast Value between 0 and \ref DEGREE_270
 */
void lcd_set_Orientation(ORIENTATION orientation)
{
    if(d_orientation == orientation)
        return;
    d_orientation = orientation;
    writecommand_last(ILI9341_MADCTL);
    switch(d_orientation){
        case rot90: {
            d_width_x = LCD_WIDTH_Y;
            d_width_y = LCD_WIDTH_X;
            writedata8_last(ILI9341_MADCTL_BGR | ILI9341_MADCTL_MX | ILI9341_MADCTL_MY);
        }
        break;
        case rot180: {
            d_width_x = LCD_WIDTH_X;
            d_width_y = LCD_WIDTH_Y;
            writedata8_last(ILI9341_MADCTL_BGR | ILI9341_MADCTL_MX | ILI9341_MADCTL_MV);
        }
        break;
        case rot270: {
            d_width_x = LCD_WIDTH_Y;
            d_width_y = LCD_WIDTH_X;
            writedata8_last(ILI9341_MADCTL_BGR);
        }
        break;
        default: {
            d_width_x = LCD_WIDTH_X;
            d_width_y = LCD_WIDTH_Y;
            writedata8_last(ILI9341_MADCTL_BGR | ILI9341_MADCTL_MV | ILI9341_MADCTL_MY);
        }
        break;
    }

}



