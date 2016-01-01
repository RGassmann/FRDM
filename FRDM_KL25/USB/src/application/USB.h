#ifndef _USB_H_
#define _USB_H_

#include "Common.h"

extern void initUSB(void);

extern void receiveUSBCommand( uint8_t size, uint8_t *buffer);
extern void sendUSBResponse( uint8_t size, const uint8_t *buffer);

void usbPutChar(char ch);
void setBDMBusy(void);

void USB0_IRQHandler( void );
#endif  // _USB_H_
