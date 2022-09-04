/*
 * I2C_init.c
 *
 *  Created on: Mar 31, 2022
 *      Author: ALFREDMOUSSA
 */


#include<stdio.h>
#include"main.h"
#include"I2C_init.h"

/*I2C clock requirements: ->tI2CCLK < (tLOW - tfilters) / 4 and tI2CCLK < tHIGH
 * tfilters: when enabled, sum of the delays brought by the analog filter and by the digital filter.
			 Analog filter delay is maximum 260 ns. Digital filter delay is DNF x tI2CCLK.
             The PCLK clock period tPCLK must respect the following condition:
					-->>tPCLK < 4/3 tSCL with tSCL: SCL period
Caution: When the I2C kernel is clocked by PCLK, this clock must respect the conditions for tI2CCLK.

*/

/*
 * I2C initialization
 *-->I2C peripheral clock enabled AND THE GPIO CLOCK
 *-->I2C timings must be configured to guarantee a correct data hold and setupp time either in MSR or SLAVE
 	 this is done by setting PRESC[3:0], SCLDEL[3:0] and SDADEL[3:0] bits in the I2C_TIMINGR register.
 *-->CONFIGURE THE I2C PINS IN ALTERNATE FUNCTION
 *-->*/
/*I2c has indepedent clock domain allowing communication baudrate independent from system clock
 * modes --> wakeup from stop mode on address match
 * I2cc setup andd hold ttimes between sda and scl during transmisson are software programble through the PRESC , SDADEL, AND SCLDEL IN I2C_TIMINGR
 * ---> SDADEL used to generate data hold time  -->SCLDEL is used to generate data setup time
 * I2C can acknowlledgge several slave adresses. 2 adressws registers
 * I2C_OAR1:7-or 10 bit adress && I2C_OAR2: 7bit address only. and OA2MSK[2:0] allow masking of 0 to 7 LSB of oar2 to allow multiple slave adresss
 *
 * WAke up from stop mode on address  match
 * 1- I2Cclk must be set to HSI16 oscillator , the I2c is able to wakeup the MCU from Stop mode when it recieves its slave address

 * Master mode configuration
 * 1- In I2c_CR2 --> Start=1  , Slave adress is set , RD_WRN the transfer direction , and how  many bytes to be transfered and AUTOEND =1--> to stop automatically after the N data

 *SLAVE mode :
 *I2c slaves use a clock stretching.
 *REception: ACKnowledge control can be done on selected bytes in the slave byte contol SBC in I2C_CR2 mode with Relode =1 to resume after you data is transfered
   and TCR flagg is set and interrupt generated if enabled
   and in the TCR routine acknowledge can be sent or not
   its better to clear the flag of TCR
 * */



/********************************************************************
* Function Name: i2c_init
* Return Value: void
* Parameters: Enable SPI
* Description: This function sets the SPI1 for the MCU Stm32 Nucleo-L552zeq
********************************************************************/


void I2C_Config(unsigned int i2c){

	if(i2c==1){
	RCC->APB1ENR1|=RCC_APB1ENR1_I2C1EN; // ENABLE I2C CLOCK
	RCC->AHB2ENR|=RCC_AHB2ENR_GPIOBEN ; //ENABLE THE GPIOB CLOCK

	/*Configured PB8, PB9 pins for I2C1 PB8 ->I2C_A_SCL   && PB9 -->I2C_A_SDA*/
	GPIOB->MODER &=~((0x3<<2*8) & (0x3)<<2*9); //clear the pins state
	GPIOB->MODER |=(0X2<<(2*8))|(0X2<<(2*9)) ; //PA8 AND PA9 IN AF

	/*SDA This pin requires a pull-up resistor. 	*/
	GPIOB->PUPDR &= ~(0X3<<2*9);
	GPIOB->PUPDR |=  (0X1<<2*9);

	GPIOB->AFR[1]|=(4<<0)|(4<<4);

	/*I2C INITALIZATION STEPS TO FOLLOW */
	//1- RESET THE I2C1 --> TO CLEAR  I2C SCL and SDA  are put back to their reset value
	  I2C1->CR1 &= ~(I2C_CR1_PE);//PE=0, the I2C SCL and SDA  are put back to their reset value

	//2- SET THE I2C CLOCK, So, we need to set the 100 KHz clock for the I2C which is in datasheet of the slave as clock speed if in I2C mode
	I2C1->TIMINGR |=0x3<<28;//PRESC[3:0] bit 28 ->31 =3  tPRESC = (PRESC+1) x tI2CCLK
 	I2C1->TIMINGR |=0x13<<0;//SCLL[7:0]  BIT 0-> 7 SCLL = (SCLL+1) x tPRESC , SCLL=19;
 	I2C1->TIMINGR |=0xF<<8; //to generate the SCL high period in master mode. tSCLH = (SCLH+1) x tPRESC
 	//tSCL = tSYNC1 + tSYNC2 + {[(SCLH+1) + (SCLL+1)] x (PRESC+1) x tI2CCLK}

 	/* generate the delay tSDADEL between SCL falling edge and SDA edge. In
      master mode and in slave mode with NOSTRETCH = 0, the SCL line is stretched low during .*/
 	I2C1->TIMINGR |=0x2<<16; //Bits 19:16 SDADEL[3:0]: Data hold time
 	// tSCLDEL = (SCLDEL+1) x tPRESC
    I2C1->TIMINGR|= 0X4<<20;	// SCLDEL =0x4 Bits 23:20 SCLDEL[3:0]: Data setup time

	}
    I2C1->CR2|=0X1<<16;         // SETTING THE NBYTES
	I2C1->CR1|=I2C_CR1_TXIE;
	I2C1->CR1|=I2C_CR1_RXIE;

	I2C1->OAR1|=I2C_OAR1_OA1EN;//1: Own address 1 enabled. The received slave address OA1 is ACKed
	I2C1->CR1 |=  I2C_CR1_PE;// Enable I2C peripheral


}



/********************************************************************
* Function Name: i2c_start
* Return Value: void
* Parameters: void
* Description: Send I2C Start Command
********************************************************************/



void Start(unsigned int i2c){
/***I2C_Start will be used to start the I2C Communication
 * Bit 13 START: Start generation is 1 to start generation*/
	if(i2c==1){
	I2C1->CR2 |= (0X1<<13);
/*	 Bit 4 NACKF: Not Acknowledge received flag
This flag is set by hardware when a NACK is received after a byte transmission.
It is cleared by software by setting the NACKCF bit.*/
	while((I2C1->ISR & (1<<4)));/*wait for the Nack value to clear  */
	}

}

/********************************************************************
* Function Name: i2c_stop
* Return Value: void
* Parameters: void
* Description: Send I2C Stop command
*
********************************************************************/




void Stop(unsigned int i2c){
	if(i2c==1){
		I2C1->CR2|= (0X1<<14); // Bit 14 STOP: Stop generation (master mode)  1: Stop generation after current byte transfer
	}

	}






/********************************************************************
* Function Name: write_I2C
* Return Value: void.
* Parameters: void
* Description: This routine writes a single byte to the I2C bus.
********************************************************************/
void write_I2C(uint32_t Data,unsigned int i2c){
	if(i2c==1){
		while(!(I2C1->ISR & 0X1)); // This bit is set by hardware when the I2C_TXDR register is empty
		I2C1->TXDR =Data; //Note: These bits can be written only when TXE=1
		// check if the nack is sent, endicate an error

	}
}
/********************************************************************
* Function Name: I2C_SlaveInit_ADD1
* Return Value: void.
* Parameters: void.
* Description: This function to enable the slave address and set it as a 7 bit
********************************************************************/
void I2C_SlaveInit_ADD1( uint16_t ADDR,unsigned int i2c){
	if(i2c==1){
		I2C1->OAR1|=I2C_OAR1_OA1EN;//1: Own address 1 enabled. The received slave address OA1 is ACKed
		I2C1->OAR1 |= (0<<10);//mode=0 then 0: Own address 1 is a 7-bit address.
		I2C1->OAR1 |=ADDR;
		I2C1->CR1|=I2C_CR1_SBC;
	}
}

