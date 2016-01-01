#include "i2c.h"
#include <MK22F51212.h>
#include <MK22F51212_IBG.h>


signed char result[20];
unsigned char MasterTransmission;
//unsigned char SlaveID;


/*******************************************************************/
/*!
 * I2C Initialization
 * Set Baud Rate and turn on I2C1
 */
void I2C_Init(void)
{
  SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK; // Enable I2C1`clock

  /* configure GPIO for I2C1 function */

  PORTE_PCR0 = PORT_PCR_MUX(6);     // Set PTE0 to mux 6 [I2C1_SDA]
  PORTE_PCR1 = PORT_PCR_MUX(6);     // Set PTE1 to mux 6 [I2C1_SCL]

  I2C1_F  = 0x14;       /* set MULT and ICR */

  //I2C1_C1 = I2C_C1_MST_MASK | I2C_C1_TXAK_MASK | I2C_C1_TX_MASK;       /* enable IIC */
  I2C1_C1 |= I2C_C1_IICEN_MASK;       /* enable IIC */
}

void Pause(void);
/*******************************************************************/
/*!
 * Write a byte of Data to specified register on MMA7660
 * @param u8RegisterAddress is Register Address
 * @param u8Data is Data to write
 */
void I2C_Write(unsigned char SlaveID, unsigned char* u8Data, unsigned char len)
{
    int i = 0;
  /* send data to slave */
  I2C_StartTransmission(SlaveID,0);
  i2c_Wait();

  for ( i = 0 ; i < len ; i++) {
    I2C1_D = u8Data[i];
    i2c_Wait();
  }

  i2c_Stop();

  Pause();
}

/*******************************************************************/
/*!
 * Write a byte of Data to specified register on MMA7660
 * @param u8RegisterAddress is Register Address
 * @param u8Data is Data to write
 */
void I2C_WriteCtrl(unsigned char SlaveID, unsigned char CrtlByte, unsigned char* u8Data, unsigned char len)
{
    int i = 0;
  /* send data to slave */
  I2C_StartTransmission(SlaveID,0);
  i2c_Wait();

    I2C1_D = CrtlByte;
    i2c_Wait();

  for ( i = 0 ; i < len ; i++) {
    I2C1_D = u8Data[i];
    i2c_Wait();
  }

  i2c_Stop();

  Pause();
}


/*******************************************************************/
/*!
 * Start I2C Transmision
 * @param SlaveID is the 7 bit Slave Address
 * @param Mode sets Read or Write Mode
 */
void I2C_StartTransmission (unsigned char SlaveID, unsigned char write)
{
  /* shift ID in right possition */
  //SlaveID = (unsigned char) ACCEL_I2C_ADDRESS << 1;
  SlaveID = (unsigned char) SlaveID << 1;
  /* Set R/W bit at end of Slave Address */
  SlaveID |= (unsigned char) write ? 0x01 : 0x00;

  /* send start signal */
  i2c_Start();

  /* send ID with W/R bit */
  i2c_write_byte(SlaveID);
}

/*******************************************************************/
/*!
 * Pause Routine
 */
void Pause(void){
    int n;
    for(n=1;n<50;n++) {
      __asm__("nop");
    }
}
//
///*******************************************************************/
///*!
// * Read a register from the MMA7660
// * @param u8RegisterAddress is Register Address
// * @return Data stored in Register
// */
//unsigned char I2C_ReadRegister(unsigned char u8RegisterAddress)
//{
//  unsigned char result;
//
//  /* Send Slave Address */
//  I2C_StartTransmission(SlaveID,MWSR);
//  i2c_Wait();
//
//  /* Write Register Address */
//  I2C1_D = u8RegisterAddress;
//  i2c_Wait();
//
//  /* Do a repeated start */
//  I2C1_C1 |= I2C_C1_RSTA_MASK;
//
//  /* Send Slave Address */
//  I2C1_D = (ACCEL_I2C_ADDRESS << 1) | 0x01; //read address
//  i2c_Wait();
//
//  /* Put in Rx Mode */
//  I2C1_C1 &= (~I2C_C1_TX_MASK);
//
//  /* Turn off ACK since this is second to last byte being read*/
//  I2C1_C1 |= I2C_C1_TXAK_MASK;
//
//  /* Dummy read */
//  result = I2C1_D ;
//  i2c_Wait();
//
//  /* Send stop since about to read last byte */
//  i2c_Stop();
//
//  /* Read byte */
//  result = I2C1_D ;
//
//  return result;
//}
//
//
//
///*******************************************************************/
///*!
// * Write a byte of Data to specified register on MMA7660
// * @param u8RegisterAddress is Register Address
// * @param u8Data is Data to write
// */
//void I2C_WriteRegister(unsigned char u8RegisterAddress, unsigned char u8Data)
//{
//  /* send data to slave */
//  I2C_StartTransmission(SlaveID,MWSR);
//  i2c_Wait();
//
//  I2C1_D = u8RegisterAddress;
//  i2c_Wait();
//
//  I2C1_D = u8Data;
//  i2c_Wait();
//
//  i2c_Stop();
//
//  Pause();
//}
//
///*******************************************************************/
///*!
// * Read first three registers from the MMA7660
// * @param u8RegisterAddress is Register Address
// * @return Data stored in Register
// */
//unsigned char I2C_ReadMultiRegisters(unsigned char u8RegisterAddress, unsigned char bytes)
//{
//  unsigned char n=bytes;
//  int i;
//
//  /* Send Slave Address */
//  I2C_StartTransmission(SlaveID,MWSR);
//  i2c_Wait();
//
//  /* Write Register Address */
//  I2C1_D = u8RegisterAddress;
//  i2c_Wait();
//
//  /* Do a repeated start */
//  I2C1_C1 |= I2C_C1_RSTA_MASK;
//
//  /* Send Slave Address */
//  I2C1_D = (ACCEL_I2C_ADDRESS << 1) | 0x01; //read address
//  i2c_Wait();
//
//  /* Put in Rx Mode */
//  I2C1_C1 &= (~I2C_C1_TX_MASK);
//
//  /* Ensure TXAK bit is 0 */
//  I2C1_C1 &= ~I2C_C1_TXAK_MASK;
//
//  /* Dummy read */
//  result[0] = I2C1_D ;
//  i2c_Wait();
//
//  for(i=0;i<n-2;i++)
//  {
//    /* Read first byte */
//    result[i] = I2C1_D;
//    i2c_Wait();
//  }
//  /* Turn off ACK since this is second to last read*/
//  I2C1_C1 |= I2C_C1_TXAK_MASK;
//
//  /* Read second byte */
//  result[i++] = I2C1_D;
//  i2c_Wait();
//
//  /* Send stop */
//  i2c_Stop();
//
//  /* Read third byte */
//  result[i++] = I2C1_D;
//
////  printf("%3d    %3d     %3d\n",result[0],result[2],result[4]);
//  return result[0];
//}
