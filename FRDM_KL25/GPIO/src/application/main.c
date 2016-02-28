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

#define LED_GREEN   PIN_INV(PB,19)
#define LED_BLUE    PIN_INV(PD,1)
#define LED_RED     PIN_INV(PB,18)

void gpio_init(void);

int main(void)
{
    gpio_init();

    while (1) {
        /* GPIO Pin Toggeln (Ein falls Aus, Aus falls Ein)
         * gemäss RM 41.2.4 Port Toggle Output Register */
        GPIOB->PTOR |= 1 << 18;
        delay(); // Quick and Dirty Verzögerungsschleife
        GPIOB->PTOR |= 1 << 19;
        delay(); // Quick and Dirty Verzögerungsschleife
        GPIOD->PTOR |= 1 << 1;
        delay(); // Quick and Dirty Verzögerungsschleife
    }
}

void gpio_init(void)
{
    /* Ports mit Clock aktivieren gemäss Reference Manual 12.2.9*/
    SIM->SCGC5 =    SIM_SCGC5_PORTA_MASK |
                    SIM_SCGC5_PORTB_MASK |
                    SIM_SCGC5_PORTC_MASK |
                    SIM_SCGC5_PORTD_MASK |
                    SIM_SCGC5_PORTE_MASK;

    /* Gewünschte Pins auf GPIO setzten
     * gemäss RM 11.5.1 Pin Control Register */
    PORTB->PCR[18] = PORT_PCR_MUX(1); //Red
    PORTB->PCR[19] = PORT_PCR_MUX(1); //Green
    PORTD->PCR[1]  = PORT_PCR_MUX(1); //Blue

    /* GPIO Pins als Outputs definieren
     * gemäss RM 41.2.6 Port Data Direction Register */
    GPIOB->PDDR |= 1 << 18;
    GPIOB->PDDR |= 1 << 19;
    GPIOD->PDDR |= 1 << 1;

    /* GPIO Pins einschalten (HI) => Led leuchten nicht
     * gemäss RM 41.2.2 Port Set Output Register */
    GPIOB->PSOR |= 1 << 18;
    GPIOB->PSOR |= 1 << 19;
    GPIOD->PSOR |= 1 << 1;
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
