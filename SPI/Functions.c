/*
 * Function.c
 *
 *  Created on: Mar 7, 2022
 *      Author: ALFREDMOUSSA
 */



/*
 * Functions.h
 *
 *  Created on: Mar 6, 2022
 *      Author: ALFREDMOUSSA
 */

#include "Functions.h"
#include"main.h"
#include "MB85RS2MTA.h" //  command specified in op-code

void compartor1(){


	RCC->AHB2ENR =RCC_AHB2ENR_GPIOAEN;
	GPIOA->IDR  &= ((0x1)<<2);  // 2- Set the Mode for the pin (INPUT) PA2

	COMP1->CSR|=COMP_CSR_INPSEL_1 ;//PA2
	COMP1->CSR|=COMP_CSR_INMSEL_2;// DAC1
	COMP1->CSR|=COMP_CSR_EN;///ENABLE COMP1
}


void DAC_Enable(){

GPIOA->MODER  &= ~((0x3)<<8);  // 2- Set the Mode register for the pin (output)
GPIOA->MODER  &= ~((0x1)<<8);  // 2- Set the Mode  register for the pin (output)
RCC->APB1ENR1|=1<<29;  // Enable DAC clock
DAC1->MCR=((0x5));
DAC1->CR |= 1;           // Enable DAC CH1
}
void ADC1_init(void){
RCC->AHB2ENR|=1;      // Enable clock going to GPIOA
RCC->AHB2ENR|=1<<13;  // Enable ADC clock
RCC->CCIPR1|=0x3<<28; // Route SYSCLK (HCLK) to ADC
GPIOA->MODER |= 0x3<<(2*2); // GPIOA2 to Analog Input
ADC1->CR = 1<<28;           // Turn ADC Voltage Regulator
delay_ms(10);               // give it some time to stabilize

ADC1->SQR1 = (7<<6)|(0); // L=0 (one channel to read), SQ1=channel 8 which isconnected to PA3 (Called A0 on the board)
ADC1->CR |=1;            // Enable ADC
while(!(ADC1->ISR &0x1));// Wait for ADC to be ready ADRDY
ADC1->CR|=0x1<<2;        //Start 1st conversion
}





void SPI_read(uint8_t *data, int size){
	// size her is how much we will push into the the DR to read
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	************************************************/
	while (size)
		{
			while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
			SPI1->DR = 0;  // send dummy data
			while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

			//while (!((SPI1->SR) &(1<<0))){}; // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
		  *data++ = (SPI1->DR);
			size--;
		}

// this function will take the pointer to the data and the size of the data
}


void SPI_SEND (uint32_t *data, int size)
{

	/************** STEPS  *****************
	1. Wait until the TXE bit is set in the SR(status reg)
	2. Write the data to the DR(data reg)
	3. After the data has been transmitted, wait for the (busy)BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/

	int i=0;
	while (i<size)
	{
	   while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   SPI1->DR = data;  // load the data into the Data Register
	   i++;
	}
	while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

		//  Clear the Overrun flag by reading DR and SR
	//whenever we send data to spi it send data in return, and it is some random data
	//before we begin receving data from the slave decice i need to clear it , therefore i need to make sure the data register is to make a dummy read

		uint8_t temp = SPI1->DR;
				temp = SPI1->SR;

}
void SPI_cmd_sent(uint8_t cmd){


	/*When the clock signal runs continuously, the BSY flag stays set between data frames at master but
becomes low for a minimum duration of one SPI clock at slave between each data frame
transfer. As a consequence it is 	mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
	data.
	*/
	while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
	SPI1->DR|=cmd;
	/*Before exiting the Transmit function, we will make a dummy read to the data register and the status register.
This is to clear the overrun flag, which gets set when we transfer the data to the device.
	 */
	uint8_t temp = SPI1->DR;
			temp = SPI1->SR;

}
void SPIss(uint8_t i){
	RCC->AHB2ENR|=RCC_AHB2ENR_GPIOEEN; // \ENABLING GPIOC CLOCK
	GPIOE->ODR &= (i)<<12; //Set CS to 0 or 1 depending what are you doing- NSS
}
void CS_Disable (void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;  // Enable GPIO Clock
	GPIOE->BSRR  |= (1<<28);
}
//CS=0
void CS_Enable (void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;  // Enable GPIO Clock
	GPIOE->BSRR  |= (1<<12);
}



void mem_write(uint8_t address, char value){
		uint8_t data[2];
		data[0] = address ;  //  write to address
		data[1] = value;     //write the value
		CS_Enable ();  // pull the cs pin low
		SPI_SEND (data, 2);  // write data to register
		CS_Disable ();  // pull the cs pin high

}
uint8_t RxData[4];

void mem_read (uint8_t address)
{
	address |= FRAM_READ;  // read operation
	uint8_t rec;
	CS_Enable ();  // pull the pin low
	SPI_SEND (&address, 1);  // send address
	SPI_read (RxData, 4);  // receive 8 bytes data
	CS_Disable ();  // pull the pin high
}


void SPIsend(char byte){
SPI1->DR = byte; //Send byte
while(!(SPI1->SR & 0x1<<1));
}

uint8_t SPIread() {
uint8_t byte;
while(SPI1->SR & 0x1){
byte= SPI1->DR; //Read data register until clear
}
return byte;
}
uint32_t mem_id(){                       // return 32-bit memory ID
	uint32_t fid=0;
	CS_Enable();  //ssi low
	SPI1->CR1 &=~(0X1<<8);
	SPIsend(0b10011111); //Opcode for return memory ID
	fid= SPIread()<<24;
	SPIsend(0x0);
	fid |= SPIread()<<16;
	SPIsend(0x0);
	fid |=SPIread()<<8;
	SPIsend(0x0);
	fid |=SPIread();
	//SPIsend(0xf);
	SPI1->CR1 |=(0X1<<8);
	CS_Disable();

	return fid;
}

void delay_ms(uint32_t val){

    SysTick->LOAD = 2000-1;   /* reload with number of clocks per millisecond (use
N-1)*/
    SysTick->VAL = 0;          /* clear current value register */
    SysTick->CTRL = 0x1;       /* Enable the timer,  CLKSOURCE= 0 (/8), 1 HCLK */

    for (uint32_t i=0; i<val; i++){
            while((SysTick->CTRL & 0x10000) == 0){
            }; /* wait until the COUNTFLAG is set */
        }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}


void setClks(){
RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
RCC->CCIPR1   &= ~(0x400);
RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}


void LPUART1init(){
PWR->CR2      |=0x200;   // Enable VDDIO2 Independent I/Os supply
                         // Basically power up PORTG
// LPUART1 TX is connected to Port G pin 7, RX is connected to PG8
// GPIOG is connected to the AHB2 bus.
RCC->AHB2ENR |= (1<<6);   // Enable the clock of the GPIOG
// MCU LPUART1 TX is connected the MCU pin PG7
    // PG7 must be set to use AF (Alternate Function).
    // Note that you need to set AF to 8 to connect the LPUART1 TX to the GPIOG.7
GPIOG->MODER  &= ~(0x3<<(2*7)); // Clear the two bits for PG7
GPIOG->MODER  |=  (0x2<<(2*7)); // Set the mode to AF (10--> 0x2)
// Set the AF=8
GPIOG->AFR[0] &= ~(0xF<<(4*7)); // Clear the 4 bits for PG7
    GPIOG->AFR[0] |=  (0x8<<(4*7)); // Set the 4 bits to (8)
// MCU LPUART1 RX can be connected the MCU pin PG8
    // PA3 must be set to use AF (Alternate Function).
    // Note that you need to set AF to 8 to connect the LPUART1 RX to the GPIOG.8
GPIOG->MODER  &= ~(0x3<<(2*8)); // Clear the two bits for PG8
GPIOG->MODER  |=  (0x2<<(2*8)); // Set the mode to AF (10--> 0x2)
// Set the AF=8
GPIOG->AFR[1] &= ~(0xF<<(4*0)); // Clear the 4 bits for PG8
    GPIOG->AFR[1] |=  (0x8<<(4*0)); // Set the 4 bits to (7)
// Enable the clock for the LPUART1
// LPUART1 is connected to the APB1 (Advanced Peripheral 1) bus.
    // LPUART1 enabled by setting bit 0
// LPUART1 CONFIGURATION //
    // We need to setup the baud rate to 115,200bps, 8 bit, 1 stop bit, no parity,  and no hw flow control
LPUART1-> PRESC = 0;    //MSI16 going to the UART can be divided. 0000: input clock not divided
// Buadrate = (256 X LPUARTtck_pres)/LPUART_BRR
// LPUART_BRR = 256 * 16MHz / 115200=  35,555.5  ==> 0x8AE3
LPUART1->BRR = 0x8AE3;  //  (16000000/115200)<<8
// LPUART1 input clock is the HSI (high speed internal clock) which is 16MHz.
LPUART1->CR1  = 0x0;  // clear all settings
LPUART1->CR1 |= 1<<3; // Enable Transmitter
LPUART1->CR1 |= 1<<2; // Enable Receiver
// 00: 1 stop bit
LPUART1->CR2 = 0x0000;    // 1 stop bit and all other features left to default (0)
LPUART1->CR3 = 0x0000;    // no flow control and all other features left to default (0)
// Last thing is to enable the LPUART1 module (remember that we set the  clock, configure GPIO, configure LPUART1)
LPUART1->CR1 |= 1; // Enable LPUART1
}

void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0' ){ LPUART1write(msg[idx++]);}
}

/* Write a character to LPUART1 */

void LPUART1write (int ch) {
    while (!(LPUART1->ISR & 0x0080)); // wait until Tx buffer empty
    LPUART1->TDR = (ch & 0xFF);
}

/* Read a character from LPUART1 */
int LPUART1read(void) {
    while (!(LPUART1->ISR & 0x0020)) {}   // wait until char arrives
    return LPUART1->RDR;
}


/*  Initialize TIM2
    Timer TIM2 channel 2 is configured to generate PWM at 1 kHz. The output of
    the timer signal is used to trigger ADC conversion.
 */

void Tim2_init(){
RCC->APB1ENR1 |=RCC_APB1ENR1_TIM2EN;   // ENABLING TIMER 2 CLOCK
TIM2->PSC = 16000-1   ; // setting the prescale to us
TIM2->ARR = 1000; // you set the value auto reload register
TIM2->DIER |= 1;         //Interrupt Enabler for Timer2
NVIC_SetPriority(TIM2_IRQn,1);
NVIC_EnableIRQ(TIM2_IRQn);
//NVIC_EnableIRQ(ADC1_2_IRQHandler);
TIM2->CNT =0; // clear the counter
TIM2->CR1=1; //Counter enable


}
void GPIO_Config (void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;  // Enable GPIO Clock

	GPIOE->MODER |= (2<<13*2)|(2<<14*2)|(2<<15*2)|(1<<12*2);  // Alternate functions for PE10, PE14, PE15 and Output for PE12

	GPIOE->OSPEEDR |= (3<<13*2)|(3<<12*2)|(3<<14*2)|(3<<15*2);  // HIGH Speed for PE10,PE12,PE14,PE15

	GPIOE->AFR[1] |= (5<<20)|(5<<24)|(5<<28);   // AF4 (SPI1) for PE10, PE14, PE15

}


void SPI_Config(){

	//SPI1->CR1 &=~(SPI_CR1_SPE);// DISEANBLED SPI

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;/*SPI1 CLOCK ENABLED */

	SPI1->CR1|=SPI_CR1_MSTR;// SET THE MASTER MODE

	//RCC->PLLCFGR|=0X40<<8; // NOW THE SYSTEM CLK IS 80MHZ

/*SETTING THE BAUD RATE CONTROL depending on the slave speed which is 40MHZ and my
	my SPI clock is 110MHZ, thus I have to set my prescaler to fPCLK/2 TO HAVE THE SPEED OF 40MHZ*/
	SPI1->CR1|=(0x1 << 3);//BR[2:0]=000 , fpclk/2 ,fpclk=80MHZ, SPI CLK=40MHZ

	SPI1->CR1|=SPI_CR1_CPHA|SPI_CR1_CPOL;// Read on the rising edge chosen because FRAM /Read on the rising edge chosen because FRAM

	SPI1->CR1|=(SPI_CR1_SSM)|SPI_CR1_SSI;// SSM=1, SSi=1 -> Software Slave Management

	SPI1->CR1 &= ~(1<<7);  // LSBFIRST = 0, MSB first DEVICE REQUIREMENT of the FRAM

	SPI1->CR1 &= ~(1<<10);  // RXONLY = 0, full-duplex

	SPI1->CR2|=(0x7 << (8));//configure the data length for SPI transfers 0111: 8-bit
	SPI1->CR2|=0X1<<2;//SSOE ENABLED
	SPI1->CR1|=SPI_CR1_SPE;// EANBLE SPI

}
void Port_E_Pins_Enabled(int start , int end){
	RCC->AHB2ENR|=RCC_AHB2ENR_GPIOCEN; // \ENABLING GPIOC CLOCK
	for (int i=start; i<=end ;i++){
		/* set pin to output mode */
		GPIOE->MODER |=1<<(2*i);
		GPIOE->MODER &=~(1<<((2*i)+1));
		GPIOE->BSRR  = (1<<(i+16));  /*Turn  off the leds, so we start from an off state  */

	}
}
