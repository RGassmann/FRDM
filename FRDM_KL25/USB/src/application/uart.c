/*
 * uart-mk.c
 *
 *  Created on: 14/04/2013
 *      Author: pgo
 */

#include "derivative.h"
//#include "system.h"
//#include "clock_configure.h"
#include "uart.h"

//#define USE_IRQ

#if defined(MCU_MK22F51212)
//=================================================================================
// UART to use
//
#define UART  UART1
#define UART_CLOCK SystemCoreClock //SYSTEM_UART1_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTE->PCR[0] = PORT_PCR_MUX(3);
   PORTE->PCR[1] = PORT_PCR_MUX(3);
}
#elif defined(MCU_MK21F12) || defined(MCU_MK22F12) || defined(MCU_MK20D5)
//=================================================================================
// UART to use
//
#define UART  UART0
#define UART_CLOCK SYSTEM_UART0_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTB->PCR[16] = PORT_PCR_MUX(3);
   PORTB->PCR[17] = PORT_PCR_MUX(3);
}
#elif defined(MCU_MK20D10) || defined(MCU_MK20DZ10) || defined(MCU_MK40D10) || defined(MCU_MK40DZ10) || defined(MCU_MK20D7)
//=================================================================================
// UART to use
//
#define UART  UART0
#define UART_CLOCK SYSTEM_UART0_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTD->PCR[6] = PORT_PCR_MUX(3);
   PORTD->PCR[7] = PORT_PCR_MUX(3);
}
#elif defined(MCU_MK60DZ10) || defined(MCU_MK60D10) || defined(MCU_MK60F12) || defined(MCU_MK10DZ10) || defined(MCU_MK10D10) || defined(MCU_MK10F12)
//=================================================================================
// UART to use
//
#define UART  UART5
#define UART_CLOCK SYSTEM_UART5_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC1 |= SIM_SCGC1_UART5_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

   // Set Tx & Rx pins in use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTE->PCR[8] = PORT_PCR_MUX(3);
   PORTE->PCR[9] = PORT_PCR_MUX(3);
}
#elif defined(MCU_MK64F12)
//=================================================================================
// UART to use
//
#define UART  UART0
#define UART_CLOCK SYSTEM_UART0_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTB->PCR[16] = PORT_PCR_MUX(3);
   PORTB->PCR[17] = PORT_PCR_MUX(3);

#ifdef USE_IRQ
   // Enable IRQs in NVIC
   NVIC_EnableIRQ(UART0_RxTx_IRQn);
   NVIC_EnableIRQ(UART0_Error_IRQn);
#endif
}
#elif defined( MCU_MKL25Z4 )
//=================================================================================
// UART to use
//
#define UART  UART0
#define UART_CLOCK SystemCoreClock

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTA->PCR[1] = PORT_PCR_MUX(2);
   PORTA->PCR[2] = PORT_PCR_MUX(2);

#ifdef USE_IRQ
   // Enable IRQs in NVIC
   NVIC_EnableIRQ(UART0_RxTx_IRQn);
   NVIC_EnableIRQ(UART0_Error_IRQn);
#endif
}
#else
#error "Please modify before use"
//=================================================================================
// UART to use
//
#define UART  UART0
#define UART_CLOCK SYSTEM_UART0_CLOCK

//=================================================================================
// UART Port pin setup
//
__attribute__((always_inline))
inline static void initDefaultUart()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTD->PCR[6] = PORT_PCR_MUX(3);
   PORTD->PCR[7] = PORT_PCR_MUX(3);
}
#endif

#if !defined(UART_CLOCK)
#error "UART_CLOCK not defined"
#endif

/*
 * Initialises the UART
 *
 * @param baudrate - the baud rate to use e.g. DEFAULT_BAUD_RATE
 */
void uart_initialise(int baudrate) {
   initDefaultUart();

   // Disable UART before changing registers
   UART->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

   // Calculate UART clock setting (5-bit fraction at right)
   int scaledBaudValue = (2*UART_CLOCK)/(baudrate);

#ifdef UART_C4_BRFA_MASK
   // Set Baud rate register
   UART->BDH = (UART->BDH&~UART_BDH_SBR_MASK) | UART_BDH_SBR((scaledBaudValue>>(8+5)));
   UART->BDL = UART_BDL_SBR(scaledBaudValue>>5);
   // Fractional divider to get closer to the baud rate
   UART->C4 = (UART->C4&~UART_C4_BRFA_MASK) | UART_C4_BRFA(scaledBaudValue);
#else
	scaledBaudValue += 1<<4; // Round value
   // Set Baud rate register
   UART->BDH = (UART->BDH&~UART_BDH_SBR_MASK) | UART_BDH_SBR((scaledBaudValue>>(8+5)));
   UART->BDL = UART_BDL_SBR(scaledBaudValue>>5);
#endif

   UART->C1 = 0;

#ifdef USE_IRQ
   // Enable UART Tx & Rx - with Rx IRQ
   UART->C2 = UART_C2_TE_MASK|UART_C2_RE_MASK|UART_C2_RIE_MASK;
#else
   // Enable UART Tx & Rx
   UART->C2 = UART_C2_TE_MASK|UART_C2_RE_MASK;
#endif
}

/*
 * Transmits a single character over the UART (blocking)
 *
 * @param ch - character to send
 */
void uart_txChar(int ch) {
   while ((UART->S1 & UART_S1_TDRE_MASK) == 0) {
      // Wait for Tx buffer empty
      __asm__("nop");
   }
   UART->D = ch;
}

#ifdef USE_IRQ
static uint8_t rxBuffer[100];
static uint8_t *rxPutPtr = rxBuffer;
static uint8_t *rxGetPtr = rxBuffer;

void UART0_RxTx_IRQHandler() {
   // Ignores overflow
   (void)UART->S1;
   int ch = UART->D;
   *rxPutPtr++ = ch;
   if (rxPutPtr == rxBuffer+sizeof(rxBuffer)) {
      rxPutPtr = rxBuffer;
   }
}

void UART0_Error_IRQHandler() {
   // Clear & ignore any pending errors
   if ((UART->S1 & (UART_S1_FE_MASK|UART_S1_OR_MASK|UART_S1_PF_MASK|UART_S1_NF_MASK)) != 0) {
      // Discard data (& clear status)
      (void)UART->D;
   }
}

/*
 * Receives a single character over the UART (blocking)
 *
 * @return - character received
 */
int uart_rxChar(void) {

   // Wait for character
   while (rxGetPtr==rxPutPtr) {
   }
   // Get char from buffer
   __disable_irq();
   int ch = *rxGetPtr++;
   if (rxGetPtr==rxBuffer+sizeof(rxBuffer)) {
      rxGetPtr = rxBuffer;
   }
   __enable_irq();
   if (ch == '\r') {
      ch = '\n';
   }
   return ch;
}
#else
/*
 * Receives a single character over the UART (blocking)
 *
 * @return - character received
 */
int uart_rxChar(void) {
   uint8_t status;
   // Wait for Rx buffer full
   do {
      status = UART->S1;
      // Clear & ignore pending errors
      if ((status & (UART_S1_FE_MASK|UART_S1_OR_MASK|UART_S1_PF_MASK|UART_S1_NF_MASK)) != 0) {
         (void)UART->D;
      }
   }  while ((status & UART_S1_RDRF_MASK) == 0);
   int ch = UART->D;
//   uart_txChar(ch);
   if (ch == '\r') {
      ch = '\n';
   }
   return ch;
}
#endif
