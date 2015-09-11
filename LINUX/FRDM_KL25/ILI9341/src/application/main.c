/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.h"
#include <MKL25Z4.h>
#include <MKL25Z4_IBG.h>

#include "LCD/lcd.h"

#define GREEN_LED    PIN(PB,19)
#define BLUE_LED    PIN(PD,1)   // Blue Led is used in SPI0
#define RED_LED    PIN(PB,18)

void gpio_init(void);


void spi_init(void);
void spi_send(char spiMsg);

int main(void)
{
    char ch = 0x01;

    gpio_init();    //Init GPIO's


    //spi_init();   //Init SPI0

    lcd_init();


    while (1) {
        OUTPUT_TOGGLE(GREEN_LED);
        delay();
        OUTPUT_TOGGLE(RED_LED);
        //delay();
        //OUTPUT_TOGGLE(BLUE_LED);
        delay();
        spi_send(ch);    //Send char over SPI
    }
}

void gpio_init(void)
{
    SIM->SCGC5 =    SIM_SCGC5_PORTA_MASK |
                    SIM_SCGC5_PORTB_MASK |
                    SIM_SCGC5_PORTC_MASK |
                    SIM_SCGC5_PORTD_MASK |
                    SIM_SCGC5_PORTE_MASK;

    INIT_OUTPUT(GREEN_LED);
    //INIT_OUTPUT(BLUE_LED);
    INIT_OUTPUT(RED_LED);
}

void delay(void)
{
    volatile unsigned int i,j;

    for (i = 0U; i < 1000U; i++) {
        for (j = 0U; j < 500U; j++) {
            __asm__("nop");
        }
    }
}

void spi_init(void)
{
    //SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;      //Turn on clock to D module
    SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;       //Enable SPI0 clock

   // PORTD_PCR0 = PORT_PCR_MUX(0x2);    //Set PTD0 to mux 2 [SPI0_PCS0]
    PORTD_PCR1 = PORT_PCR_MUX(0x2);    //Set PTD1 to mux 2 [SPI0_SCK]
    PORTD_PCR2 = PORT_PCR_MUX(0x2);    //Set PTD2 to mux 2 [SPI0_MOSI]
    //PORTD_PCR3 = PORT_PCR_MUX(0x2);    //Set PTD3 to mux 2 [SPIO_MISO]

    SPI0_C1 = SPI_C1_MSTR_MASK;// | SPI_C1_SSOE_MASK;   //Set SPI0 to Master & SS pin to auto SS

    SPI0_C2 = SPI_C2_MODFEN_MASK;   //Master SS pin acts as slave select output

    //SPI0_BR = (SPI_BR_SPPR(0x02) | SPI_BR_SPR(0x08));     //Set baud rate prescale divisor to 3 & set baud rate divisor to 64 for baud rate of 15625 hz
    SPI0_BR = 0;

    SPI0_C1 |= SPI_C1_SPE_MASK;    //Enable SPI0
}

void spi_send(char spiMsg)
{

  while(!(SPI_S_SPTEF_MASK & SPI0_S))
  {
    __asm__("nop");  //While buffer is not empty do nothing
  }

  SPI0_D = spiMsg;    //Write char to SPI

}
