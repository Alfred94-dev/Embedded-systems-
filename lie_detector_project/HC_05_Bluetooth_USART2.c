/*
 * HC_05_Bluetooth.c
 *
 *  Created on: Mar 27, 2022
 *      Author: ALFREDMOUSSA
 */


#include"HC_05_Bluetooth_USART2.h"


//PD5(Tx) and PD6(Rx)
//AF 7
void USART2GPIOinit(){
	PWR->CR2      |=0x200;   // Enable VDDIO2 Independent I/Os supply
	                         // Basically power up PORTG
	RCC->AHB2ENR |= (1<<3);   // Enable the clock of the GPIOA
	GPIOD->MODER  &= ~(0x3<<(2*5)); // Clear the two bits for PD5
	GPIOD->MODER  |=  (0x2<<(2*5)); // Set the mode to AF (10--> 0x2)
	GPIOD->AFR[0] &= ~(0xF<<(4*5)); // Clear the 4 bits for PD5
    GPIOD->AFR[0] |=  (0x7<<(4*5)); // Set the 4 bits to (7)

	GPIOD->MODER  &= ~(0x3<<(2*6)); // Clear the two bits for PD6
	GPIOD->MODER  |=  (0x2<<(2*6)); // Set the mode to AF (10--> 0x2)
	// Set the AF=7
	GPIOD->AFR[0] &= ~(0xF<<(4*6)); // Clear the 4 bits for PD6
    GPIOD->AFR[0] |=  (0x7<<(4*6)); // Set the 4 bits to (7)
}

// We need to setup the baud rate to 115,200bps, 8 bit, 1 stop bit, no parity, and no hw flow control
void USART2init(){
	USART2-> PRESC = 0;    //MSI16 going to the UART can be divided. 0000: input clock not divided
	// Buadrate = 115200
	USART2->BRR = 0x8b;//0x8AE3;  //  (16000000/115200)<<8
	// LPUART1 input clock is the HSI (high speed internal clock) which is 16MHz.
	USART2->CR1  = 0x0;  // clear all settings
	USART2->CR1 |= 1<<3; // Enable Transmitter
	USART2->CR1 |= 1<<2; // Enable Receiver
	USART2->CR2 = 0x0000;    // 1 stop bit and all other features left to default (0)
	USART2->CR3 = 0x0000;    // no flow control and all other features left to default (0)
	USART2->CR1 |= 1<<0; // Enable LPUART1
}
