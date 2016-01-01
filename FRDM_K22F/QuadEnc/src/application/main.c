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
#include <MK22F51212.h>
#include <MK22F51212_IBG.h>

#define BLUE_LED    PIN_INV(PD,5)
#define GREEN_LED   PIN_INV(PA,2)
#define RED_LED     PIN_INV(PA,1)
void gpio_init(void);

int main(void)
{
    unsigned long cnt = 0;
    gpio_init();


    while (1) {

        OUTPUT_TOGGLE(BLUE_LED);
        if(cnt!= FTM1_CNT)
            OUTPUT_SET(RED_LED, POSITION_ON);
        cnt = FTM1_CNT;
        delay();
    }
}

void gpio_init(void)
{
    SIM->SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK| SIM_SCGC5_PORTB_MASK;


    INIT_OUTPUT(BLUE_LED);
    INIT_OUTPUT(RED_LED);
    INIT_OUTPUT(GREEN_LED);
    OUTPUT_SET(BLUE_LED, POSITION_OFF);
    OUTPUT_SET(RED_LED, POSITION_OFF);
    OUTPUT_SET(GREEN_LED, POSITION_OFF);

    //enable the clock for FTM1

    SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;

    //enable the counter
    FTM1_SC = 0;

    FTM1_C0V = 0;
    FTM1_C1V = 0;        //Clear these while we can!
   // FTM1_COMBINE = FTM_COMBINE_DECAPEN0_MASK;     //Prep for dual-edge capture


    FTM1_MODE |= FTM_MODE_FTMEN_MASK;

    //enable the counter to run in the BDM mode

    FTM1_CONF |= FTM_CONF_BDMMODE(3);

    //load the Modulo register and counter initial value

    FTM1_MOD = 0xFFFF;

    FTM1_CNTIN = 0x8000;
    FTM1_CNT = 0x8000;
FTM1_CNTIN = 0x0;
    //configuring FTM for quadrature mode

    FTM1_QDCTRL = FTM_QDCTRL_QUADEN_MASK;;// <---------------(Note)

    // start the timer clock, source is the external clock

    FTM1_SC |= FTM_SC_CLKS(3); //<--------------------------------------(Note)

   // FTM1_COMBINE |= FTM_COMBINE_DECAP0_MASK;  //Make dual-cap GO!

    //configuring the input pins:

    PORTB_PCR0 = PORT_PCR_MUX(6); // FTM1 CH0

    PORTB_PCR1 = PORT_PCR_MUX(6); // FTM1 CH1

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
