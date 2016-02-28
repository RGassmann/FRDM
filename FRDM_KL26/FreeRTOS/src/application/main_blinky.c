/*
 * Filename:	main_blinky.c
 * Description:	Demo of FreeRTOS on KL25Z board
 *              Blinks the three LEDs at different rates
 *              TPM initialization and much of the rest of the code
 *              can be attributed to FreeRTOS and/or Rowley and/or
 *              Freescale example code.
 */

 //-Wextra -Wall -Wno-unused-parameter -Wno-unused-function -Wno-unused-label -Wpointer-arith -Wformat -Wreturn-type -Wsign-compare -Wmultichar -Wformat-nonliteral -Winit-self -Wuninitialized -Wno-deprecated -Wformat-security -Werror


#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <sysinit.h>
#include <math.h>
//#include <cross_studio_io.h>  // for debug_printf
#define NDEBUG 1

#define TPM_Cn_MODE  (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK)
#define TPM_MODULE    1000
#define TPM_INIT_VAL	25
#define SET_LED_GREEN(x)   TPM0->CONTROLS[4].CnV = (x)
#define SET_LED_RED(x)	   TPM0->CONTROLS[2].CnV = (x)
#define SET_LED_BLUE(x)	   TPM0->CONTROLS[5].CnV = (x)

#define RED_MASK   0x01
#define GREEN_MASK 0x02
#define BLUE_MASK  0x04

#define MAX(a,b) (a > b ? a : b)

#define PI 3.1415926

//static char led_state_mask = 0;

void vPrintString( const char * );

/******************************************************************************/
/**   TPM_init
 * \brief    Initialize LPTPM for RGB led control
 * \brief    FTM2 and FTM  in PWM edge aligned mode
 * \author   b01252
 * \param    none
 * \return   none
 */
void TPM_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
    SIM->SCGC6|=( SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM2_MASK);
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //

    PORTD->PCR[5] = (0|PORT_PCR_MUX(4)); /* TPM2_CH0 enable on PTB18 */
    PORTE->PCR[29] = (0|PORT_PCR_MUX(3)); /* TPM2_CH1 enable on PTB19 */
    PORTE->PCR[31]  = (0|PORT_PCR_MUX(3)); /* TPM0_CH1 enable on PTD1 */

    TPM0->MOD  = TPM_MODULE;  /* 0x0063 / 25MHz = 4uS PWM period */
    TPM0->CONTROLS[2].CnSC = TPM_Cn_MODE;   /* No Interrupts; High True pulses on Edge Aligned PWM */
    TPM0->CONTROLS[2].CnV  = TPM_INIT_VAL;  /* 90% pulse width */
    TPM0->CONTROLS[4].CnSC = TPM_Cn_MODE;   /* No Interrupts; High True pulses on Edge Aligned PWM */
    TPM0->CONTROLS[4].CnV  = TPM_INIT_VAL;  /* 90% pulse width */
    TPM0->CONTROLS[5].CnSC = TPM_Cn_MODE;   /* No Interrupts; High True pulses on Edge Aligned PWM */
    TPM0->CONTROLS[5].CnV  = TPM_INIT_VAL;  /* 90% pulse width */

    TPM0->SC   = TPM_SC_CMOD(1) | TPM_SC_PS(0);	    /* Edge Aligned PWM running from BUSCLK / 1 */
}

void vPrintString( const char *pcString )
{
	/* Print the string, suspending the scheduler as method of mutual
	   exclusion. */
	vTaskSuspendAll();
	{
#ifndef NDEBUG
		debug_printf( "%s\n", pcString );
#endif
	}
	xTaskResumeAll();
}

/*-----------------------------------------------------------*/

short getVal(short delay, short time, short maxTime){
    short localTime = (time + delay) % maxTime;
    if( localTime < maxTime / 3){
        return (((float)(time % (maxTime/3))) / (maxTime/3) * 1000);
    } else if ( localTime < maxTime * 2 / 3){
        //return 0;
        return 1000 - (((float)(time % (maxTime/3))) / (maxTime/3) * 1000);
    } else {
        return 0;
    }

}

void vTask1( void *pvParameters )
{
    const portTickType xDelay = 50; // ms
    const char *pcTaskName = "Task 1 is running";
   // short valr= 0,valb= 0,valg = 0;
    static short time = 0;
    vPrintString( pcTaskName );
    for (;;)
    {
        vPrintString( pcTaskName );

        SET_LED_RED(getVal(0,time,360));
        SET_LED_BLUE(getVal(120,time,360));
        SET_LED_GREEN(getVal(240,time,360));
        time+=1;
        if(time == 360) time=0;

        //toggleLED(RED_MASK);
        vTaskDelay(xDelay / portTICK_RATE_MS);
    }
}

/*-----------------------------------------------------------*/

void vTask2( void *pvParameters )
{
    const portTickType xDelay = 200; //ms
    const char *pcTaskName = "Task 2 is running";
    vPrintString( pcTaskName );
    for (;;)
    {
       vPrintString( pcTaskName );
      // toggleLED(GREEN_MASK);
       vTaskDelay(xDelay/ portTICK_RATE_MS);
    }
}

/*-----------------------------------------------------------*/

void vTask3( void *pvParameters )
{
    const portTickType xDelay = 300; //ms
    const char *pcTaskName = "Task 3 is running";
    vPrintString( pcTaskName );
    for (;;)
    {
       vPrintString( pcTaskName );
//       toggleLED(BLUE_MASK);
       vTaskDelay(xDelay/ portTICK_RATE_MS);
    }
}

/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
  SET_LED_RED(0);
  SET_LED_GREEN(0);
  SET_LED_BLUE(0);
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
}

/*-----------------------------------------------------------*/

void vApplicationTickHook( )
{
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( )
{
}

/*-----------------------------------------------------------*/

void vMainConfigureTimerForRunTimeStats( void )
{
}


/*-----------------------------------------------------------*/

int main (void)
{
    // Initialize
    sys_init ();
    TPM_init();
   // toggleLED(RED_MASK);
   // toggleLED(BLUE_MASK);
   // toggleLED(GREEN_MASK);

    // Print to console
    vPrintString("Starting");

    // Set the LEDs to a known state
    prvSetupHardware();

    // Create the blink tasks
    xTaskCreate( vTask1, "Red   - task 1", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
    //xTaskCreate( vTask2, "Green - task 2", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
    //xTaskCreate( vTask3, "Blue  - task 3", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach this line
    while(1)
    {
    }
}
/********************************************************************/
