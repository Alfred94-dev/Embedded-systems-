/*
 * MPU9808_I2C.c
 *
 *  Created on: Mar 31, 2022
 *      Author: ALFREDMOUSSA
* MPU908_temperature sensor with I2C
 */


#include "main.h"
#include "I2C_init.h"
#include "MPU9808_I2C.h"


/********************************************************************
* Function Name: MANUFACTURER_ID
* Return Value: MANUFACTURER ID REGISTER which should be reg_data= 0x0054
* Parameters: slave_addr = 0x18<<1 and reg_addr=0x06
* Description: read MANUFACTURER_ID
********************************************************************/
int Manfacture_ID(uint8_t slave_addr,uint8_t  reg_addr){

	 uint32_t  reg_data;

	 I2C_Config(1);// first intialize the start
	  uint32_t p = 250000;
		  	while(p>0)
		  		p--;

	 Start(1);// send a start
	 write_I2C(slave_addr,1);
	 while((I2C1->ISR) & I2C_ISR_TXIS);//check if Txis is not empty
	 //I2C1->ISR |=0X1;
	 I2C1->TXDR =reg_addr; //Note: These bits can be written only when TXE=1

	 I2C1->CR2 |= (I2C_CR2_RD_WRN); // requesting to a read transfer.
	 Stop(1); // GENERATE STOP;


	 while(!(I2C1->ISR & I2C_ISR_RXNE));/* //wait until the rxne is set RXNE: Receive data register not empty (receivers)*/
	 reg_data= I2C1->RXDR;
	 return reg_data;
}















/********************************************************************
* Function Name: I2C_writeByte
* Return Value: contents of SSP2BUF register
* Parameters: slave_addr = 0x18<<1 and reg_addr and reg_data
* Description: write function
********************************************************************/

void I2C_writeByte(uint8_t slave_addr,uint8_t  reg_addr, uint8_t  reg_data ){

	  // send the start condition
	  Start(1);
	  // The ACk is not expected at all--> which means go and Check Start bit in ISR


	  /*To send and address here is steps to follow
	   * Bit 3 ADDR: Address matched (slave mode) -->This bit is set by hardware as soon as the received slave address matched with one of the
	  	enabled slave addresses. It is cleared by software by setting ADDRCF bit. */

	  //Bit 3 ADDR: Address matched (slave mode), so we need to check this once sent
	   write_I2C(slave_addr & 0xFE,1);

	  while(!(I2C1->ISR & (1<<3)));// addr BIT CHECK
	 // Now we clear s the ADDR flag in the I2C_ISR register since it matched ;
	  I2C1->ICR |=I2C_ICR_ADDRCF;


	  // write the reg_address
	  write_I2C(reg_addr,1);
	  // write your data
	  write_I2C(reg_data,1);


	  // THEN WE NEED TO WAIT UNTIL TC (Transfer Complete) is Finished
	  while(!(I2C1->ISR & I2C_ISR_TC));// wait until Bit 6 TC: Transfer Complete
	  Stop(1); // GENERATE STOP;

}

/********************************************************************
* Function Name: i2c_read
* Return Value: contents of SSP2BUF register
* Parameters: ack = 1 and nak = 0
* Description: Read a byte from I2C bus and ACK/NAK device
********************************************************************/
uint8_t I2C_ReadByte(uint8_t slave_address ,uint8_t reg_address){

	uint8_t reg_data1,reg_data2;
	int arr[2];
	I2C_Config(1);

    Start(1);	  // send the start condition
	I2C1->CR2|=((slave_address<<1) &0xfE) ;// shift by 1 to show all 7 bits adress
//	write_I2C((slave_address<<1) &0xfE,1);
	write_I2C(reg_address,1);
	  	  	  /********READ from the RXDR*********/
    Start(1);	  // send the start condition
    write_I2C((slave_address<<1) |0x01,1);
	while(!(I2C1->ISR & 1<<15));		//Wait
	I2C1->CR2 |= (I2C_CR2_RD_WRN); // requesting to a read transfer.

	 while(!(I2C1->ISR & I2C_ISR_RXNE));/* //wait until the rxne is set RXNE: Receive data register not empty (receivers)*/


	 reg_data1= I2C1->RXDR;

	 reg_data2=I2C1->RXDR;
	 /*Generate the stop conditon*/
	 Stop(1);
	 arr[0]=reg_data1;
	 arr[1]=reg_data2;
	  return arr[2] ;

}





