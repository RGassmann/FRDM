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
#include "MMA8451Q.h"
#include <MKL25Z4.h>
#include <MKL25Z4_IBG.h>

#define TPM_Cn_MODE  (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK)
#define TPM_MODULE    1000
#define TPM_INIT_VAL	25
#define SET_LED_GREEN(x)   TPM2->CONTROLS[1].CnV = (x)
#define SET_LED_RED(x)	   TPM2->CONTROLS[0].CnV = (x)
#define SET_LED_BLUE(x)	   TPM0->CONTROLS[1].CnV = (x)

#define GREEN_LED    PIN(PB,19)
#define BLUE_LED    PIN(PD,1)
#define RED_LED    PIN(PB,18)

void gpio_init(void);

int main(void)
{
    DataReady = 0;
    gpio_init();
	MCU_Init();
  	Accelerometer_Init();
  	Calibrate();

  SET_LED_RED(0);
  SET_LED_GREEN(0);
  SET_LED_BLUE(0);

    while (1) {

        if (DataReady)		// Is a new set of data ready?
		{
			DataReady = 0;

			I2C_ReadMultiRegisters(MMA845x_I2C_ADDRESS, OUT_X_MSB_REG, 6, AccData);		// Read data output registers 0x01-0x06

			Xout_14_bit = ((short) (AccData[0]<<8 | AccData[1])) >> 2;		// Compute 14-bit X-axis output value
			Yout_14_bit = ((short) (AccData[2]<<8 | AccData[3])) >> 2;		// Compute 14-bit Y-axis output value
			Zout_14_bit = ((short) (AccData[4]<<8 | AccData[5])) >> 2;		// Compute 14-bit Z-axis output value

			Xout_g = ((float) Xout_14_bit) / SENSITIVITY_2G;		// Compute X-axis output value in g's
			Yout_g = ((float) Yout_14_bit) / SENSITIVITY_2G;		// Compute Y-axis output value in g's
			Zout_g = ((float) Zout_14_bit) / SENSITIVITY_2G;		// Compute Z-axis output value in g's

			SET_LED_GREEN( 500 +  (long)(Xout_g*500) );
			SET_LED_RED( (long)(Yout_g*500));
			SET_LED_BLUE( 500 +  (long)(Zout_g*500));
		}
		__asm__("nop");
    }
}

void gpio_init(void)
{
    SIM->SCGC5 =    SIM_SCGC5_PORTA_MASK |
                    SIM_SCGC5_PORTB_MASK |
                    SIM_SCGC5_PORTC_MASK |
                    SIM_SCGC5_PORTD_MASK |
                    SIM_SCGC5_PORTE_MASK;

    SIM->SCGC6|=( SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM2_MASK);
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //

    PORTB->PCR[18] = (0|PORT_PCR_MUX(3)); /* TPM2_CH0 enable on PTB18 */
    PORTB->PCR[19] = (0|PORT_PCR_MUX(3)); /* TPM2_CH1 enable on PTB19 */
    PORTD->PCR[1]  = (0|PORT_PCR_MUX(4)); /* TPM0_CH1 enable on PTD1 */

    TPM0->MOD  = TPM_MODULE;  /* 0x0063 / 25MHz = 4uS PWM period */
    TPM0->CONTROLS[1].CnSC = TPM_Cn_MODE;   /* No Interrupts; High True pulses on Edge Aligned PWM */
    TPM0->CONTROLS[1].CnV  = TPM_INIT_VAL;  /* 90% pulse width */

    TPM2->MOD  = TPM_MODULE;  /* 0x0063 / 25MHz = 4uS PWM period */
    TPM2->CONTROLS[0].CnSC = TPM_Cn_MODE;   /* No Interrupts; Low True pulses on Edge Aligned PWM */
    TPM2->CONTROLS[0].CnV  = TPM_INIT_VAL;  /* 90% pulse width */
    TPM2->CONTROLS[1].CnSC = TPM_Cn_MODE;   /* No Interrupts; Low True pulses on Edge Aligned PWM */
    TPM2->CONTROLS[1].CnV  = TPM_INIT_VAL;  /* 90% pulse width */

    TPM2->SC   = TPM_SC_CMOD(1) | TPM_SC_PS(0);	    /* Edge Aligned PWM running from BUSCLK / 1 */
    TPM0->SC   = TPM_SC_CMOD(1) | TPM_SC_PS(0);	    /* Edge Aligned PWM running from BUSCLK / 1 */
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
