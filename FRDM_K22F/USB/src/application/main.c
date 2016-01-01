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
#include "USB.h"
#include "uart.h"
#include "CDC.h"
#include <MK22F51212.h>
#include <MK22F51212_IBG.h>


#define BLUE_LED    PIN_INV(PD,5)
#define GREEN_LED   PIN_INV(PA,2)
#define RED_LED     PIN_INV(PA,1)
#define SW2         PIN_INV(PC,1)
#define SW3         PIN_INV(PB,17)


void gpio_init(void);

int main(void)
{
static uint8_t commandToggle = 0;
uint8_t  commandBuffer[255];
uint8_t bufsize;
char  cdctxbuf[6] = "Roman";
char cdcrxbuf[256];
    __enable_irq();
    gpio_init();
    uart_initialise(115200);

OUTPUT_SET(BLUE_LED,POSITION_ON);

initUSB();

OUTPUT_SET(BLUE_LED,POSITION_OFF);
OUTPUT_SET(GREEN_LED,POSITION_OFF);
   //for(;;) {
      (void)receiveUSBCommand( 254, commandBuffer );
      OUTPUT_SET(GREEN_LED,POSITION_ON);
      /*commandToggle = commandBuffer[1] & 0x80;
      commandBuffer[1] &= 0x7F;
      //commandExec();
      commandBuffer[0] |= commandToggle;
      sendUSBResponse( 1, commandBuffer );*/
  // }
   /*for(;;) {
      enableInterrupts();
      (void)receiveUSBCommand( MAX_COMMAND_SIZE, commandBuffer );
      size = commandExec();
      sendUSBResponse( size, commandBuffer );
   }*/

    while (1) {
        OUTPUT_TOGGLE(BLUE_LED);
        delay();

        bufsize = cdc_setRxBuffer(cdcrxbuf);
        if(bufsize){
            cdc_putTxBuffer(cdcrxbuf,bufsize);
            OUTPUT_TOGGLE(GREEN_LED);
        }

        if ( INPUT_GET(SW2) ) {

            cdc_putTxBuffer(cdctxbuf,6);

        }



       //for(;;) {
         // (void)receiveUSBCommand( 255, commandBuffer );
         /* commandToggle = commandBuffer[1] & 0x80;
          commandBuffer[1] &= 0x7F;
          //commandExec();
          commandBuffer[0] |= commandToggle;
          sendUSBResponse( 255, commandBuffer );*/
      // }
    }
}

void gpio_init(void)
{
   SIM_SCGC5 |=   SIM_SCGC5_PORTA_MASK
                | SIM_SCGC5_PORTB_MASK
                | SIM_SCGC5_PORTC_MASK
                | SIM_SCGC5_PORTD_MASK
                | SIM_SCGC5_PORTE_MASK;

    INIT_OUTPUT(GREEN_LED);
    INIT_OUTPUT(RED_LED);

    INIT_OUTPUT(BLUE_LED);
    OUTPUT_SET(GREEN_LED, POSITION_OFF);
    OUTPUT_SET(RED_LED, POSITION_OFF);

    INIT_INPUT(SW2);
    INIT_INPUT(SW3);

}

void delay(void)
{
    volatile unsigned int i,j;

    for (i = 0U; i < 8000U; i++) {
        for (j = 0U; j < 500U; j++) {
            __asm__("nop");
        }
    }
}

void greenLedOff() {
    OUTPUT_SET(GREEN_LED, POSITION_OFF);
}
void greenLedOn(){
    //OUTPUT_SET(GREEN_LED, POSITION_ON);
}

void greenLedToggle(){
    OUTPUT_TOGGLE(GREEN_LED);
}

void redLedOff() {
    OUTPUT_SET(RED_LED, POSITION_OFF);
}
void redLedOn(){
    OUTPUT_SET(RED_LED, POSITION_ON);
}
