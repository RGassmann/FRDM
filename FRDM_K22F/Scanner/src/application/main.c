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

#include "LCD/lcd.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define GREEN_LED   PIN_INV(PA,2)
#define RED_LED     PIN_INV(PA,1)
#define BLUE_LED    PIN_INV(PD,5)
#define DIRECTION   PIN(PC,6)
#define PWM_MOTOR   PIN(PC,3)
#define END_Front   PIN(PD,7)
#define END_Back    PIN(PD,6)
#define SW_2        PIN_INV(PC,1)
#define SW_1        PIN_INV(PB,17)
#define NEAR_END    PIN_INV(PE,1)

#define MAX_PWM     65535

#define MM_STEP     1645
 typedef enum {stop, error, forwards, backwards} Rstate;
    struct {
        int32 current;
        int32 desired;
        Rstate state;
        Rstate state_old;
        uint8 speed;
    } ReaderPos;

void gpio_init(void);

void motor_Init(void);

int main(void) {
    //uint16 i =0;
    uint16 cnt =  0/*, cnt_old =0*/, OFCnt =0;
  //  uint32 pos = 0;

//bool dir = false;
//    char ch = 0x01;
    gpio_init();    //Init GPIO's


    //spi_init();   //Init SPI0

    lcd_init();

    lcd_clear(ILI9341_White);

    lcd_set_Orientation(rot0);
    lcd_draw_rect(0,240,120,160,ILI9341_Navy,true);
    lcd_set_font(&Font13x16, true);
    //lcd_write_center_string(10,"Test");

    lcd_set_font_color(ILI9341_White);
    lcd_set_background_color(ILI9341_Navy);
    lcd_set_font(&Font8x11, true);
    lcd_write_center_string(145,"G A S S M A N N");

    lcd_set_font(&Font13x16, true);

    lcd_write_center_string(125,"PCS - 2020");

    lcd_draw_rect(0,240,0,20,ILI9341_GRAY1,true);
    lcd_draw_line(0,240,20,20,ILI9341_Black);

    lcd_draw_circle(120, 60,20,ILI9341_Red);
    lcd_set_font_color(ILI9341_Black);
    //lcd_set_background_color(ILI9341_White);
lcd_set_background_color(ILI9341_GRAY1);
    lcd_draw_bitmap(60,5,&IPOD_SYMBOL);

    lcd_draw_bitmap(70,5,&FM_SYMBOL);
    lcd_draw_bitmap(80,5,&IR_SYMBOL);

lcd_set_background_color(ILI9341_GRAY1);
    lcd_draw_battery(200, 5, 50);
lcd_set_background_color(ILI9341_White);




   // void motor_Init();

    //lcd_write_digit(5,200, 3, 9);
//lcd_send_cmd(ILI9341_DISPON);    //Display on
FTM1_CNT = 0x00;


FTM0->CONTROLS[2].CnV = (MAX_PWM*5)/10;

    OUTPUT_SET(DIRECTION, POSITION_OFF);
    while ( INPUT_GET(END_Front) ){
        if ( INPUT_GET(NEAR_END) ) {
            OUTPUT_SET(RED_LED,POSITION_OFF);
            FTM0->CONTROLS[2].CnV = (MAX_PWM*10)/10;
        } else {
            OUTPUT_SET(RED_LED,POSITION_ON);
            FTM0->CONTROLS[2].CnV = (MAX_PWM*4)/10;
        }

        OUTPUT_SET(BLUE_LED, POSITION_ON); //Wait for endpos
    };
    FTM0->CONTROLS[2].CnV = 0;
    OUTPUT_SET(DIRECTION, POSITION_ON);
    delay();
    FTM0->CONTROLS[2].CnV = (MAX_PWM*1)/10;
    while ( !INPUT_GET(END_Front) ) OUTPUT_SET(BLUE_LED, POSITION_OFF);; //Wait for endpos
    FTM0->CONTROLS[2].CnV = 0;
    OUTPUT_SET(DIRECTION, POSITION_ON);
    delay();
            if ( FTM1_SC & FTM_SC_TOF_MASK ) {
                FTM1_SC &=~FTM_SC_TOF_MASK;
            }
    FTM1_CNT = 0x00;
    OFCnt = 0;
//    pos = 0;
    ReaderPos.current = 0;
    ReaderPos.desired = 0;
    ReaderPos.state = stop;
    ReaderPos.state_old = error;
    INIT_OUTPUT(RED_LED);
    while (1) {



        if ( INPUT_GET(NEAR_END) ) {
            OUTPUT_SET(RED_LED,POSITION_OFF);
        } else {
            OUTPUT_SET(RED_LED,POSITION_ON);
        }


        if ( INPUT_GET(SW_1) ) {
            ReaderPos.desired = 120*MM_STEP+100;;//164500*3+100;//164500+100;
        }

        if ( !INPUT_GET(END_Front) ){
            ReaderPos.state = error;
            FTM0->CONTROLS[2].CnV = 0;
        }

        cnt = FTM1_CNT;

        if ( FTM1_SC & FTM_SC_TOF_MASK ) {
            if ( FTM1_QDCTRL & FTM_QDCTRL_TOFDIR_MASK )
                OFCnt++;
            else
                OFCnt--;
            FTM1_SC &=~FTM_SC_TOF_MASK;
        }
        ReaderPos.current = OFCnt * 0xFFFF + cnt;



        lcd_write_digit(10,180,(ReaderPos.current/1000000)%10,1);
        lcd_write_digit(20,180,(ReaderPos.current/100000)%10,1);
        lcd_write_digit(30,180,(ReaderPos.current/10000)%10,1);
        lcd_write_digit(40,180,(ReaderPos.current/1000)%10,1);
        lcd_write_digit(50,180,(ReaderPos.current/100)%10,1);
        lcd_write_digit(60,180,(ReaderPos.current/10)%10,1);
        lcd_write_digit(70,180,(ReaderPos.current)%10,1);



        /*FTM0->CONTROLS[6].CnV = pos < 65535 ? cnt : 65535 ;       // Setting the PWM value

        FTM0->CONTROLS[7].CnV = (old_cnt-cnt)*100;*/


        switch ( ReaderPos.state ) {
            case stop://lcd_write_string(10, 90, "Stop    ");
                FTM0->CONTROLS[2].CnV = 0;
                if ( (ReaderPos.desired - 20) > ReaderPos.current ) {
                    ReaderPos.state = forwards;
                }
                lcd_write_digit(10,170,(ReaderPos.desired/1000000)%10,1);
                lcd_write_digit(20,170,(ReaderPos.desired/100000)%10,1);
                lcd_write_digit(30,170,(ReaderPos.desired/10000)%10,1);
                lcd_write_digit(40,170,(ReaderPos.desired/1000)%10,1);
                lcd_write_digit(50,170,(ReaderPos.desired/100)%10,1);
                lcd_write_digit(60,170,(ReaderPos.desired/10)%10,1);
                lcd_write_digit(70,170,(ReaderPos.desired)%10,1);
            break;

            case error:
            //lcd_write_string(10, 90, "Error    ");
                FTM0->CONTROLS[2].CnV = 0;

            break;

            case forwards://if (ReaderPos.state != ReaderPos.state_old){
            //lcd_write_string(10, 90, "Forwards");
            //}
            OUTPUT_SET(RED_LED, POSITION_ON);
                OUTPUT_SET(DIRECTION, POSITION_ON);
                uint32 diff = ReaderPos.desired - ReaderPos.current;

                if ( diff < 3 || ReaderPos.desired < ReaderPos.current) {
                    ReaderPos.state = stop;
                } else {
                    FTM0->CONTROLS[2].CnV =MIN((((MAX_PWM)*MIN(diff,20000)/20000))+1000,MAX_PWM);
                }

                /*else if ( diff < 100 ){FTM0->CONTROLS[2].CnV = (MAX_PWM*1)/100;}
                else if ( diff < 1000 ){ FTM0->CONTROLS[2].CnV = (MAX_PWM*5)/100;}
                else if ( diff < 40000 ){ FTM0->CONTROLS[2].CnV = (MAX_PWM*20)/100;}
                else if ( diff < 100000 ){FTM0->CONTROLS[2].CnV = (MAX_PWM*30)/100;}
                else if ( diff < 1000000 ){FTM0->CONTROLS[2].CnV = (MAX_PWM*5)/10;}
                else{FTM0->CONTROLS[2].CnV = (MAX_PWM*5)/10;}*/

            break;

            case backwards:
                OUTPUT_SET(DIRECTION, POSITION_OFF);
                FTM0->CONTROLS[2].CnV = 0;
            break;


        }

       /* if( cnt != cnt_old){
            FTM0->CONTROLS[2].CnV = 45000;
            cnt_old = cnt;
            if(pos < 200000) {
                OUTPUT_SET(DIRECTION, POSITION_ON);
                // OUTPUT_SET(RED_LED, POSITION_ON);
            } else {
                OUTPUT_SET(DIRECTION, POSITION_OFF);
                // OUTPUT_SET(RED_LED, POSITION_OFF);
            }
        } else {
            FTM0->CONTROLS[7].CnV = 0;
        }*/

        //OUTPUT_TOGGLE(GREEN_LED);
        //delay();


        /*lcd_set_background_color(ILI9341_GRAY1);
        lcd_draw_battery(200, 5, i%100);

        //lcd_set_background_color(i);
        lcd_set_background_color(ILI9341_White);
        //lcd_set_font_color(i);
        lcd_write_digit(5,200, i++%10, 9);*/









        //OUTPUT_TOGGLE(RED_LED);
        //delay();
        //OUTPUT_TOGGLE(BLUE_LED);
        //delay();
        //delay();
        //spi_send(ch);    //Send char over SPI
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
    INIT_OUTPUT(RED_LED);
    INIT_OUTPUT(BLUE_LED);
    INIT_OUTPUT(DIRECTION);
    INIT_OUTPUT(PWM_MOTOR);
    INIT_INPUT_PULLUP(END_Front);
    INIT_INPUT_PULLUP(END_Back);
    INIT_INPUT(SW_1);
    INIT_INPUT(SW_2);
    INIT_INPUT(NEAR_END);


    OUTPUT_SET(GREEN_LED, POSITION_OFF);
    OUTPUT_SET(RED_LED, POSITION_OFF);
    OUTPUT_SET(BLUE_LED, POSITION_OFF);
    OUTPUT_SET(DIRECTION, POSITION_ON);
    OUTPUT_SET(PWM_MOTOR, POSITION_OFF);


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

        //PWM
    SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;          // Controls the clock gate to the FTM0 module
          FTM0->SC |= FTM_SC_CLKS(1);                    // Selecting External Clock for Source clock

    FTM0->SC |= FTM_SC_PS(4);                       // Selecting presaler as 0

    FTM0->MOD = MAX_PWM;                                       // Setting Modulo value

    FTM0->CONTROLS[6].CnSC |= ( FTM_CnSC_MSB_MASK | FTM_CnSC_ELSA_MASK); // Rising edge counter

    FTM0->CONTROLS[6].CnV = 0;                   // Starting PWM with initial 0 duty cycle
    FTM0->CONTROLS[7].CnSC |= ( FTM_CnSC_MSB_MASK | FTM_CnSC_ELSA_MASK); // Rising edge counter

    FTM0->CONTROLS[7].CnV = 0;                   // Starting PWM with initial 0 duty cycle

    FTM0->CONTROLS[2].CnSC |= ( FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK); // Rising edge counter

    FTM0->CONTROLS[2].CnV = 0;                   // Starting PWM with initial 0 duty cycle
    FTM0->CNTIN = 0;                                       // Initial Counter value to 0

    PORTA_PCR1 = PORT_PCR_MUX(3);
    PORTA_PCR2 = PORT_PCR_MUX(3);

    FTM0->CONTROLS[2].CnV = 0;

    PORTC_PCR3 = PORT_PCR_MUX(4); //PWM OUTPUT for motor
}


void motor_Init(void) {

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


