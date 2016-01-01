
/**********************************************************************/
/** Headerfile for ARM - Kinetis Processors                           */
/** Ingenieurbüro Gassmann, 8707 Uetikon                              */
/**********************************************************************/
/** Version   Autor       Info                                        */
/** 150719    RoG         Grundprogramm                               */
/**                                                                   */
/**********************************************************************/

/**
 * @file MKL_EXT.h
 * @version 1.0
 * @date 2015-07-19
 * @brief Gassmann Extensions of Headerfile for ARM - Kinetis Processors
 */


#if !defined(MKL_EXT_H_)
#define MKL_EXT_H_                               /**< Symbol preventing repeated inclusion */

#define PA  0
#define PB  1
#define PC  2
#define PD  3
#define PE  4


#define PIN(Port, PinNr)              Port, PinNr, 0
#define PIN_INV(Port, PinNr)          Port, PinNr, -1

#define __INIT_OUTPUT(Port, PinNr, Inv)            {((PORT_Type *)(PORTA_BASE + Port*(PORTB_BASE-PORTA_BASE)))->PCR[PinNr] = PORT_PCR_MUX(1); ((GPIO_Type *)(PTA_BASE+Port*(PTB_BASE-PTA_BASE)))->PDDR |= 1 << PinNr;}
#define INIT_OUTPUT(Port_Name)  __INIT_OUTPUT(Port_Name)

#define __INIT_OUTPUT_POWER(Port, PinNr, InvMask)       {((PORT_Type *)(PORTA_BASE+Port*0x00001000))->PCR[PinNr] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;\
                                                         ((GPIO_Type *)(PTA_BASE+Port*0x040))->PDDR |= 1 << PinNr;}
#define INIT_OUTPUT_POWER(Port_Name) \
            __INIT_OUTPUT_POWER(Port_Name)

#define INIT_OUTPUT_POWER_FAST(Port, PinNr, InvMask)  {((PORT_Type *)(PORTA_BASE+Port*0x00001000))->PCR[PinNr] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;\
                                                         ((GPIO_Type *)(PTA_BASE+Port*0x040))->PDDR |= 1 << PinNr;}


#define __INIT_INPUT(Port, PinNr, InvMask)                {((PORT_Type *)(PORTA_BASE+Port*0x00001000))->PCR[PinNr] = PORT_PCR_MUX(1);\
                                                         ((GPIO_Type *)(PTA_BASE+Port*0x040))->PDDR &= ~(1 << PinNr);}
#define INIT_INPUT(Port_Name)   __INIT_INPUT(Port_Name)

#define __INIT_INPUT_PULLUP(Port, PinNr, InvMask)         {((PORT_Type *)(PORTA_BASE+Port*0x00001000))->PCR[PinNr] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;\
                                                         ((GPIO_Type *)(PTA_BASE+Port*0x040))->PDDR &= ~(1 << PinNr);}
#define INIT_INPUT_PULLUP(Port_Name)   __INIT_INPUT_PULLUP(Port_Name)

#define __INIT_INPUT_PULLDOWN(Port, PinNr, InvMask)       {((PORT_Type *)(PORTA_BASE+Port*0x00001000))->PCR[PinNr] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK;\
                                                         ((GPIO_Type *)(PTA_BASE+Port*0x040))->PDDR &= ~(1 << PinNr);}
#define INIT_INPUT_PULLDOWN(Port_Name)   __INIT_INPUT_PULLDOWN(Port_Name)






#define POSITION_ON      -1
#define POSITION_OFF     0

#define     __OUTPUT_TOGGLE(Port, PinNr, Inv) \
            {((GPIO_Type *)(PTA_BASE+Port*0x040))->PTOR = 1 << PinNr;}
#define     OUTPUT_TOGGLE(Port_Name) \
            __OUTPUT_TOGGLE(Port_Name)

#define     __OUTPUT_SET(Port, PinNr, Inv, OnOff) \
            {((GPIO_Type *)(PTA_BASE+Port*0x040))->PCOR = ((!(OnOff ^ Inv)) & 1) << PinNr;\
                ((GPIO_Type *)(PTA_BASE+Port*0x040))->PSOR = ((OnOff ^ Inv) & 1) << PinNr;}
#define     OUTPUT_SET(Port_Name, OnOff) \
            __OUTPUT_SET(Port_Name, OnOff)

#define     __INPUT_GET(Port, PinNr, Inv) \
            ((((GPIO_Type *)(PTA_BASE+Port*0x040))->PDIR) >> PinNr  ^Inv) & 1
#define     INPUT_GET(Port_Name) \
            __INPUT_GET(Port_Name)

#endif  /* #if !defined(MKL_EXT_H_) */

/* MKL_EXT.h, eof. */
