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

 #define MAX(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#include "main.h"
#include <MKL25Z4.h>
#include <MKL25Z4_IBG.h>

#define LED_GREEN   PIN_INV(PB,19)
#define LED_BLUE    PIN_INV(PD,1)
#define LED_RED     PIN_INV(PB,18)

void gpio_init(void);
void init_Counter(void);

int main(void)
{


    gpio_init();
    init_Counter();

    while (1) {
       // OUTPUT_TOGGLE(LED_GREEN);
        delay();
    }
}

void gpio_init(void)
{
    SIM->SCGC5 =    SIM_SCGC5_PORTA_MASK |
                    SIM_SCGC5_PORTB_MASK |
                    SIM_SCGC5_PORTC_MASK |
                    SIM_SCGC5_PORTD_MASK |
                    SIM_SCGC5_PORTE_MASK;

    INIT_OUTPUT(LED_GREEN);
    INIT_OUTPUT(LED_BLUE);
    INIT_OUTPUT(LED_RED);

    OUTPUT_SET(LED_RED,POSITION_OFF);
    OUTPUT_SET(LED_BLUE,POSITION_OFF);
    OUTPUT_SET(LED_GREEN,POSITION_OFF);

    PORTC_PCR5 = PORT_PCR_MUX(3); // PORTE0 = CMP0_OUT => Pulse counting input
}

void init_Counter(void){

    SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK; // Enable LowPowerTimer Clock
    LPTMR0->CSR = 0; //Disable Timer
    LPTMR0->PSR = LPTMR_PSR_PCS(0) //3
                | LPTMR_PSR_PRESCALE(0x2); //No Prescaler

    LPTMR0->CMR = 50;

    LPTMR0->CSR |= LPTMR_CSR_TPS(2) | LPTMR_CSR_TMS_MASK; // USE alt2 as input and select pulse counting mode

    LPTMR0->CSR |= LPTMR_CSR_TIE_MASK | LPTMR_CSR_TCF_MASK; //enable Interrupt
    LPTMR0->CSR |= LPTMR_CSR_TEN_MASK; //Enable Timer

    //Let LPTimer_IRQ be available
    NVIC->ICPR[0] |= 1 << LPTimer_IRQn;
    NVIC->ISER[0] |= 1 << LPTimer_IRQn;
}

/**
 * @brief Timer Interrupt
 *
 *  Interrupt for the timer
 *
 */
void LPTimer_IRQHandler(void) {
    OUTPUT_TOGGLE(LED_RED);
    LPTMR0->CSR |= LPTMR_CSR_TCF_MASK; // Clear Interrupt Flag
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
