/*
 * BlueToothSend.c
 *
 *  Created on: Apr 22, 2022
 *      Author: jacob
 */


// Lab2  -- ECE533 - Jacob Jackson - S22

#include "stm32l552xx.h"
#include "main.h"



/// ---- The functions are defined here --- ///


/*void setClks(){
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}*/
//PB10(Rx) and PB11(Tx)
//AF 8
void UART1GPIOinit(){
	PWR->CR2      |=0x200;   // Enable VDDIO2 Independent I/Os supply
	                         // Basically power up PORTG
	RCC->AHB2ENR |= (1<<1);   // Enable the clock of the GPIOB
	GPIOB->MODER  &= ~(0x3<<(2*10)); // Clear the two bits for PB10
	GPIOB->MODER  |=  (0x2<<(2*10)); // Set the mode to AF (10--> 0x2)
	GPIOB->AFR[1] &= ~(0xF<<(4*2)); // Clear the 4 bits for PB10
    GPIOB->AFR[1] |=  (0x8<<(4*2)); // Set the 4 bits to (8)

	GPIOB->MODER  &= ~(0x3<<(2*11)); // Clear the two bits for PB11
	GPIOB->MODER  |=  (0x2<<(2*11)); // Set the mode to AF (10--> 0x2)
	// Set the AF=8
	GPIOB->AFR[1] &= ~(0xF<<(4*3)); // Clear the 4 bits for PB11
    GPIOB->AFR[1] |=  (0x8<<(4*3)); // Set the 4 bits to (7)
}

// We need to setup the baud rate to 115,200bps, 8 bit, 1 stop bit, no parity, and no hw flow control
void UART1init(){
	LPUART1-> PRESC = 0;    //MSI16 going to the UART can be divided. 0000: input clock not divided
	// Buadrate = 115200
	LPUART1->BRR = 0x8AE3;  //  (16000000/115200)<<8
	// LPUART1 input clock is the HSI (high speed internal clock) which is 16MHz.
	LPUART1->CR1  = 0x0;  // clear all settings
	LPUART1->CR1 |= 1<<3; // Enable Transmitter
	LPUART1->CR1 |= 1<<2; // Enable Receiver
	LPUART1->CR2 = 0x0000;    // 1 stop bit and all other features left to default (0)
	LPUART1->CR3 = 0x0000;    // no flow control and all other features left to default (0)
	LPUART1->CR1 |= 1<<0; // Enable LPUART1
}

void enableLED()
{
	RCC->AHB2ENR  |= 0b111; // 1- Enable the clock feeding into the GPIO port
	RCC->AHB1ENR  |= (1<<16);
	GPIOA->MODER  &= ~((0x3)<<18);  // 2- Set the Mode for the pin (output)
	GPIOB->MODER  &= ~((0x3)<<14);
	GPIOC->MODER  &= ~((0x3)<<14);
	GPIOC->MODER  &= ~((0x3)<<26); //Btn Pin Mode config
	GPIOA->MODER  |=   (0x1)<<18;	// 2- Set the Mode for the pin (output)
	GPIOB->MODER  |=   (0x1)<<14;
	GPIOC->MODER  |=   (0x1)<<14;

	GPIOC->IDR |=(0x1)<<12;			//Enable IDR
}

void delay(int x){		//Delay in x Milliseconds
	int i = 0;
	while(i<400*x)
		i++;
}
