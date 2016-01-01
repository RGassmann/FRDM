/*
 * File:        k60_i2c.h
 * Purpose:     I2C header
 *
 * Notes:
 *
 */
#ifndef I2C_H
#define I2C_H


#define ACCEL_I2C_ADDRESS                         0x1C

#define i2c_DisableAck()       I2C1_C1 |= I2C_C1_TXAK_MASK

#define i2c_RepeatedStart()    I2C1_C1 |= I2C_C1_RSTA_MASK;

#define i2c_Start()            {I2C1_C1 |= I2C_C1_TX_MASK;\
                               I2C1_C1 |= I2C_C1_MST_MASK | I2C_C1_IICEN_MASK;}

#define i2c_Stop()             {I2C1_C1 &= ~I2C_C1_MST_MASK;}
/*\
                               I2C1_C1 &= ~I2C_C1_TX_MASK*/

#define i2c_EnterRxMode()      I2C1_C1 &= ~I2C_C1_TX_MASK;\
                               I2C1_C1 &= ~I2C_C1_TXAK_MASK

#define i2c_Wait()               while((I2C1_S & I2C_S_IICIF_MASK)==0) {} \
                                  I2C1_S |= I2C_S_IICIF_MASK;

#define i2c_write_byte(data)   I2C1_D = data

#define MWSR                   0x00  /* Master write  */
#define MRSW                   0x01  /* Master read */

void I2C_Write(unsigned char SlaveID, unsigned char* u8Data, unsigned char len);
void I2C_Init(void);
void I2C_StartTransmission (unsigned char SlaveID, unsigned char Mode);
void I2C_WriteRegister(unsigned char u8RegisterAddress, unsigned char u8Data);
void I2C_WriteCtrl(unsigned char SlaveID, unsigned char CrtlByte, unsigned char* u8Data, unsigned char len);
unsigned char I2C_ReadRegister(unsigned char u8RegisterAddress);
unsigned char I2C_ReadMultiRegisters(unsigned char u8RegisterAddress, unsigned char bytes);

#endif //I2C_H
