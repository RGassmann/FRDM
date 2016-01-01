/*! @file
    @brief This file contains hardware specific information and configuration.


*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

#define __LITTLE_ENDIAN__

// Used to create port register names
//--------------------------------------------------------
#define CONCAT2_(x,y) x ## y
#define CONCAT3_(x,y,z) x ## y ## z
#define CONCAT4_(w,x,y,z) w ## x ## y ## z

#define PCR(reg,num)   CONCAT4_(PORT,reg,_PCR,num)
#define PDOR(reg)      CONCAT3_(GPIO,reg,_PDOR)
#define PSOR(reg)      CONCAT3_(GPIO,reg,_PSOR)
#define PCOR(reg)      CONCAT3_(GPIO,reg,_PCOR)
#define PTOR(reg)      CONCAT3_(GPIO,reg,_PTOR)
#define PDIR(reg)      CONCAT3_(GPIO,reg,_PDIR)
#define PDDR(reg)      CONCAT3_(GPIO,reg,_PDDR)

//==========================================================================================
// USB Serial Number
#ifdef UNIQUE_ID
    #define SERIAL_NO "PCS-MASTER-%lu"
#else
    #define SERIAL_NO "PCS-MASTER-0001"
#endif

#define ProductDescription "PCS2020 MASTER"

#define CRYSTAL 4000000UL

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY       (CAP_RST_IO|CAP_RST_IN|CAP_SWD_HW|CAP_CDC|CAP_CORE_REGS)
#define TARGET_CAPABILITY   (CAP_RST   |CAP_ARM_SWD|CAP_CDC)

#ifndef PLATFORM
#define PLATFORM USBDM   //! Choose BDM emulation
#endif

#define CPU  MKL25Z4

// Define for automatic WINUSB Driver loading
//#define MS_COMPATIBLE_ID_FEATURE (1)

//=================================================================================
// Port Pin assignments
// Please note: some pin assignments cannot be changed freely
// RX AND TX ROUTINES ARE DEPENDENT ON THIS SPECIFIC ASSIGNMENTS
//

//=================================================================================
// Serial Port Bit Masks
//
#if (HW_CAPABILITY&CAP_CDC)
#define CTS_IN
#define DTR_OUT
#define DTR_OUT_DDR

#define CTS_IS_HIGH()      (1)
#define DTR_LOW()          ;
#define DTR_HIGH()         ;
#define DTR_ACTIVE()       ;

// UART Tx Pin = B17 ALT3
#define TX_ALT_FN             (3)
#define TX_OUT_EN_NUM         0
#define TX_OUT_EN_REG         E
#define TX_OUT_EN_MASK        (1<<TX_OUT_EN_NUM)
#define TX_OUT_EN_PCR         PCR(TX_OUT_EN_REG,TX_OUT_EN_NUM)
#define TX_OUT_EN_PDOR        PDOR(TX_OUT_EN_REG)
#define TX_OUT_EN_PSOR        PSOR(TX_OUT_EN_REG)  // Data set
#define TX_OUT_EN_PCOR        PCOR(TX_OUT_EN_REG)  // Data clear
#define TX_OUT_EN_PTOR        PTOR(TX_OUT_EN_REG)  // Data toggle
#define TX_OUT_EN_PDIR        PDIR(TX_OUT_EN_REG)  // Data input
#define TX_OUT_EN_PDDR        PDDR(TX_OUT_EN_REG)  // Data direction

// UART Rx Pin = B16 ALT3
#define RX_ALT_FN             (3)
#define RX_OUT_EN_NUM         1
#define RX_OUT_EN_REG         E
#define RX_OUT_EN_MASK        (1<<RX_OUT_EN_NUM)
#define RX_OUT_EN_PCR         PCR(RX_OUT_EN_REG,RX_OUT_EN_NUM)
#define RX_OUT_EN_PDOR        PDOR(RX_OUT_EN_REG)
#define RX_OUT_EN_PSOR        PSOR(RX_OUT_EN_REG)  // Data set
#define RX_OUT_EN_PCOR        PCOR(RX_OUT_EN_REG)  // Data clear
#define RX_OUT_EN_PTOR        PTOR(RX_OUT_EN_REG)  // Data toggle
#define RX_OUT_EN_PDIR        PDIR(RX_OUT_EN_REG)  // Data input
#define RX_OUT_EN_PDDR        PDDR(RX_OUT_EN_REG)  // Data direction

#define UART_NUM              1

#endif

//=================================================================================
// Use of 1k5 resistor on USB D+ line
//
//  The JMxx has a programmable 1k5 pull-up resistor on the USB D+ line.
//
#define USBPUP_ON    (0) // Turn on internal PUP
#define USBPUP_OFF   (1) // Turn off internal PUP

#define USBPUP USBPUP_ON // Internal 1k5 PUP present on D+

#ifndef USBPUP
#error "Please define USBPUP in Configure.h"
#define USBPUP USBPUP_OFF
#endif

#if ((USBPUP != USBPUP_ON) && (USBPUP != USBPUP_OFF))
#error "Please correctly define USBPUP in Configure.h"
#endif

#endif // _CONFIGURE_H_

